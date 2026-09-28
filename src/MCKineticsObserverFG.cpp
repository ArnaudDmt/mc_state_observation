/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/logging.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <map>
#include <gtsam/geometry/Pose3.h>
#include <mc_state_observation/MCKineticsObserverFG.h>
#include <mc_state_observation/gui_helpers.h>

#include <RBDyn/Coriolis.h>
#include <RBDyn/CoM.h>
#include <RBDyn/FA.h>
#include <RBDyn/FD.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/MultiBodyConfig.h>

#include <mc_state_observation/conversions/kinematics.h>
#include <optional>

using namespace ko_fg;

namespace so = stateObservation;
namespace mc_state_observation
{
namespace
{
/// Controller period the configured process noises are expressed for (200 Hz).
constexpr double kProcessReferenceTimeStep = 0.005;

/// @brief Reads a noise entry as three per-axis standard deviations.
/// @details Accepts either three values or a single one, which is then used on the three axes.
/// Writing a scalar is a claim that the quantity really is isotropic, not a shortcut: an
/// anisotropic quantity given as a scalar has to take its dominant axis, which over-trusts every
/// other one.
ko_fg::NoiseSigmas3 readSigmas(const mc_rtc::Configuration & config, const std::string & key)
{
  const mc_rtc::Configuration entry = config(key);
  ko_fg::NoiseSigmas3 sigmas;
  if(entry.size() == 3) { sigmas = entry.operator so::Vector3(); }
  else if(entry.size() == 0) { sigmas = ko_fg::NoiseSigmas3::Constant(entry.operator double()); }
  else
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("{} must be a scalar or three values, got {}", key,
                                                        entry.size());
  }
  if(!sigmas.allFinite() || (sigmas.array() < 0.0).any())
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("{} must be finite and non-negative", key);
  }
  return sigmas;
}
} // namespace

MCKineticsObserverFG::MCKineticsObserverFG(const std::string & type, double dt)
: mc_observers::Observer(type, dt),
  // Horizon of the fixed-lag smoother, in seconds. It was raised to 0.1 s while the contact rest
  // poses had no absolute anchor, because a longer horizon gave the estimator time to undo a
  // transient before marginalising it. With the anchor (restPoseAnchorNoise) the horizon no longer
  // matters -- 0.016 s and 0.1 s give the same result -- so it is back to the original value, which
  // ticks twice as fast.
  observer_(0.016), removeWrenchOffset_(false)
{
  observer_.setSamplingTime(dt);
}

///////////////////////////////////////////////////////////////////////
/// --------------------------Core functions---------------------------
///////////////////////////////////////////////////////////////////////

void MCKineticsObserverFG::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());

  imuNames_ = config("imuNames", std::vector<std::string>());
  listIMUs_.clear();
  if(!imuNames_.empty())
  {
    for(size_t i = 0; i < imuNames_.size(); ++i) { listIMUs_.push_back({i, imuNames_[i]}); }
  }
  else { listIMUs_.push_back({0, ctl.robot(robot_).bodySensor().name()}); }

  config("debug", debug_);
  config("verbose", verbose_);
  config("withGui", withGui_);

  zeroPose_.translation().setZero();
  zeroPose_.rotation().setIdentity();
  zeroMotion_.linear().setZero();
  zeroMotion_.angular().setZero();

  // we set the desired type of odometry
  auto leggedOdomConfig = config("leggedOdometry");
  std::string typeOfOdometry = static_cast<std::string>(leggedOdomConfig("odometryType"));
  odometryType_ = so::odometry::stringToOdometryType(typeOfOdometry);
  observer_.setFlatOdometry(odometryType_ == so::odometry::OdometryType::Flat);

  config("withDebugLogs", withDebugLogs_);
  leggedOdomConfig("withRestPoseAverageFactor", withRestPoseAverageFactor_);

  /* configuration of the contacts manager */
  auto contactsConfig = config("contacts");

  contactsIgnoredForEstimation_.clear();
  ignoredForceSensorSurfaces_.clear();
  ignoredWrenchesInCentroid_.clear();
  for(const auto & surface :
      contactsConfig("contactsIgnoredForEstimation", std::vector<std::string>{}))
  {
    const std::string fsName = ctl.robot(robot_).indirectSurfaceForceSensor(surface).name();
    contactsIgnoredForEstimation_.insert(surface);
    ignoredForceSensorSurfaces_.emplace(fsName, surface);
    ignoredWrenchesInCentroid_.emplace(surface, Vector6::Zero());
    contactsManager_.fs_Surface_Map.emplace(fsName, surface);
    mc_rtc::log::info("[{}]: Force sensor {} on {} excluded from estimation and logged in the centroid frame.",
                      name(), fsName, surface);
  }

  std::string contactsDetectionString = static_cast<std::string>(contactsConfig("contactsDetection"));
  KoContactsDetector::ContactsDetection contactsDetectionMethod =
      KoContactsDetector::stringToContactsDetection(contactsDetectionString, name());

  if(contactsDetectionMethod == KoContactsDetector::ContactsDetection::Surfaces)
  {
    std::vector<std::string> surfacesForContactDetection =
        contactsConfig("surfacesForContactDetection", std::vector<std::string>());

    for(const auto & surface : surfacesForContactDetection)
    {
      const std::string fsName = ctl.robot(robot_).indirectSurfaceForceSensor(surface).name();
      contactsManager_.fs_Surface_Map.insert_or_assign(fsName, surface);
    }

    contactsManager_.setContactOrder(surfacesForContactDetection);

    measurements::ContactsDetectorSurfacesConfiguration contactsConf(surfacesForContactDetection);

    if(contactsConfig.has("schmittTriggerLowerPropThreshold") && contactsConfig.has("schmittTriggerUpperPropThreshold"))
    {
      double schmittTriggerLowerPropThreshold = contactsConfig("schmittTriggerLowerPropThreshold");
      double schmittTriggerUpperPropThreshold = contactsConfig("schmittTriggerUpperPropThreshold");
      contactsConf.schmittTriggerPropThresholds(schmittTriggerLowerPropThreshold, schmittTriggerUpperPropThreshold);
    }

    contactsDetector_.init(ctl, robot_, contactsConf);
  }
  if(contactsDetectionMethod == KoContactsDetector::ContactsDetection::Sensors)
  {
    measurements::ContactsDetectorSensorsConfiguration contactsConf;
    if(contactsConfig.has("schmittTriggerLowerPropThreshold") && contactsConfig.has("schmittTriggerUpperPropThreshold"))
    {
      double schmittTriggerLowerPropThreshold = contactsConfig("schmittTriggerLowerPropThreshold");
      double schmittTriggerUpperPropThreshold = contactsConfig("schmittTriggerUpperPropThreshold");
      contactsConf.schmittTriggerPropThresholds(schmittTriggerLowerPropThreshold, schmittTriggerUpperPropThreshold);
    }
    contactsDetector_.init(ctl, robot_, contactsConf);
  }
  if(contactsDetectionMethod == KoContactsDetector::ContactsDetection::Solver)
  {
    measurements::ContactsDetectorSolverConfiguration contactsConf;
    if(contactsConfig.has("schmittTriggerLowerPropThreshold") && contactsConfig.has("schmittTriggerUpperPropThreshold"))
    {
      double schmittTriggerLowerPropThreshold = contactsConfig("schmittTriggerLowerPropThreshold");
      double schmittTriggerUpperPropThreshold = contactsConfig("schmittTriggerUpperPropThreshold");
      contactsConf.schmittTriggerPropThresholds(schmittTriggerLowerPropThreshold, schmittTriggerUpperPropThreshold);
    }
    contactsDetector_.init(ctl, robot_, contactsConf);
  }

  /* Configuration of the Kinetics Observer's parameters */
  // This array is handed as-is to ko_fg::Contact, which reads it as (Kpt, Kdt, Kpr, Kdr):
  // linear stiffness, LINEAR DAMPING, angular stiffness, angular damping. It used to be filled in
  // the order (linStiffness, angStiffness, linDamping, angDamping), so the estimator received the
  // angular stiffness as its linear damping and the linear damping as its angular stiffness --
  // 727 and 150 swapped on HRP-5P. The wrapper's own use of the array below follows the same
  // convention as the core.
  so::Vector3 linStiffness = contactsConfig("linStiffness");
  contactFlexibilities_.at(0) = linStiffness.matrix().asDiagonal();
  so::Vector3 linDamping = contactsConfig("linDamping");
  contactFlexibilities_.at(1) = linDamping.matrix().asDiagonal();
  so::Vector3 angStiffness = contactsConfig("angStiffness");
  contactFlexibilities_.at(2) = angStiffness.matrix().asDiagonal();
  so::Vector3 angDamping = contactsConfig("angDamping");
  contactFlexibilities_.at(3) = angDamping.matrix().asDiagonal();

  /* Initial state noises */
  auto stateInitNoises = config("stateInitNoises");

  initNoises_.at(0) = readSigmas(stateInitNoises, "posNoise"); // pos
  initNoises_.at(1) = readSigmas(stateInitNoises, "oriNoise"); // ori
  initNoises_.at(2) = readSigmas(stateInitNoises, "linVelNoise"); // linVel
  initNoises_.at(3) = readSigmas(stateInitNoises, "angVelNoise"); // angVel
  initNoises_.at(4) = readSigmas(stateInitNoises, "linAccNoise"); // linAcc
  initNoises_.at(5) = readSigmas(stateInitNoises, "angAccNoise"); // angAcc
  initNoises_.at(6) = readSigmas(stateInitNoises, "disturbForceNoise"); // distForce
  initNoises_.at(7) = readSigmas(stateInitNoises, "disturbMomentNoise"); // distMoment
  initNoises_.at(8) = readSigmas(stateInitNoises, "contactRestPosAverageNoise"); // rest pos average
  // Pins a single scalar, the mean yaw: only the yaw (z) component is read by the estimator.
  initNoises_.at(9) = readSigmas(stateInitNoises, "contactRestYawAverageNoise"); // rest yaw average

  contactInitNoises_.at(0) = readSigmas(stateInitNoises, "restPosNoise"); // rest pos
  contactInitNoises_.at(1) = readSigmas(stateInitNoises, "restOriNoise"); // rest ori
  contactInitNoises_.at(2) = readSigmas(stateInitNoises, "forceNoise"); // force
  contactInitNoises_.at(3) = readSigmas(stateInitNoises, "momentNoise"); // moment

  // The very first contacts are initialised differently from the ones created later, as in the EKF
  // implementation (contact*InitVarianceFirstContacts vs ...NewContacts): at the start nothing else
  // anchors the estimate, so the first contacts are trusted less on the wrench and, if the block is
  // absent, more loosely on the pose.
  contactInitNoises_first_ = contactInitNoises_;
  if(config.has("stateInitNoisesFirstContacts"))
  {
    auto firstContacts = config("stateInitNoisesFirstContacts");
    contactInitNoises_first_.at(0) = readSigmas(firstContacts, "restPosNoise");
    contactInitNoises_first_.at(1) = readSigmas(firstContacts, "restOriNoise");
    contactInitNoises_first_.at(2) = readSigmas(firstContacts, "forceNoise");
    contactInitNoises_first_.at(3) = readSigmas(firstContacts, "momentNoise");
  }
  else
  {
    contactInitNoises_first_.at(0) = ko_fg::isotropicSigmas(1e-4);
    contactInitNoises_first_.at(1) = ko_fg::isotropicSigmas(1e-4);
  }

  /* State process noises */
  auto stateProcessNoises = config("stateProcessNoises");

  processNoises_.at(0) = readSigmas(stateProcessNoises, "posNoise"); // pos
  processNoises_.at(1) = readSigmas(stateProcessNoises, "oriNoise"); // ori
  processNoises_.at(2) = readSigmas(stateProcessNoises, "linVelNoise"); // linVel
  processNoises_.at(3) = readSigmas(stateProcessNoises, "angVelNoise"); // angVel
  processNoises_.at(4) = readSigmas(stateProcessNoises, "linAccNoise"); // linAcc
  processNoises_.at(5) = readSigmas(stateProcessNoises, "angAccNoise"); // angAcc
  processNoises_.at(6) = readSigmas(stateProcessNoises, "disturbForceNoise"); // distForce
  processNoises_.at(7) = readSigmas(stateProcessNoises, "disturbMomentNoise"); // distMoment

  contactProcessNoises_.at(0) = readSigmas(stateProcessNoises, "restPosNoise"); // rest pos
  contactProcessNoises_.at(1) = readSigmas(stateProcessNoises, "restOriNoise"); // rest ori
  contactProcessNoises_.at(2) = readSigmas(stateProcessNoises, "forceNoise"); // force
  contactProcessNoises_.at(3) = readSigmas(stateProcessNoises, "momentNoise"); // moment

  // The configured process noises are densities expressed for a 200 Hz controller, the rate the
  // estimator was tuned at. One process factor is added per iteration with no dt factor, so
  // running the same numbers at another period changes the noise injected per second -- a 500 Hz
  // run would get 2.5x too much. These are SIGMAS, not variances, hence the square root. Mirrors
  // MCKineticsObserver::configure.
  //
  // The contact force and moment entries are deliberately left alone: they are held fixed by the
  // tuning study rather than identified per rate.
  const double processNoiseScale = std::sqrt(ctl.timeStep / kProcessReferenceTimeStep);
  for(auto & noise : processNoises_) { noise *= processNoiseScale; }
  contactProcessNoises_.at(0) *= processNoiseScale;
  contactProcessNoises_.at(1) *= processNoiseScale;

  /* Sensor noises */
  auto sensorNoises = config("sensorNoises");
  for(size_t i = 0; i < listIMUs_.size(); ++i)
  {
    std::array<ko_fg::NoiseSigmas3, 4> imuNoise;
    imuNoise[0] = readSigmas(stateInitNoises, "gyroBiasNoise"); // gyroBias init
    // Scaled like the other process noises: the gyrometer bias random walk is a density too.
    imuNoise[1] = readSigmas(stateProcessNoises, "gyroBiasNoise") * processNoiseScale; // gyroBias process
    imuNoise[2] = readSigmas(sensorNoises, "gyroNoise"); // gyro meas
    imuNoise[3] = readSigmas(sensorNoises, "acceleroNoise"); // accelero meas

    imuNoises_.insert({i, imuNoise});
  }

  // The configuration this observer ends up with is layered: the package's etc/ file, then the
  // per-robot one, then whatever sits in ~/.config/mc_rtc/observers/, then the inline block of the
  // controller. A stale deployed copy silently wins over the package file, and the only symptom is
  // numbers that do not move. Set KO_FG_DUMP_NOISES to print what actually reached the estimator.
  if(std::getenv("KO_FG_DUMP_NOISES"))
  {
    mc_rtc::log::info("[MCKineticsObserverFG] init  pos {} ori {} linVel {} angVel {} moyRepos {}",
                      initNoises_.at(0).transpose(), initNoises_.at(1).transpose(),
                      initNoises_.at(2).transpose(), initNoises_.at(3).transpose(),
                      initNoises_.at(8).transpose());
    mc_rtc::log::info("[MCKineticsObserverFG] process pos {} ori {} reposPos {} reposOri {}",
                      processNoises_.at(0).transpose(), processNoises_.at(1).transpose(),
                      contactProcessNoises_.at(0).transpose(), contactProcessNoises_.at(1).transpose());
  }

  contactMeasNoises_.at(0) = readSigmas(sensorNoises, "forceNoise"); // force
  contactMeasNoises_.at(1) = readSigmas(sensorNoises, "momentNoise"); // torque

  const double restPoseAnchorNoise = config("restPoseAnchorNoise", 1e-3);
  if(!std::isfinite(restPoseAnchorNoise) || restPoseAnchorNoise < 0.0)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("restPoseAnchorNoise must be finite and non-negative");
  }
  observer_.setRestPoseAnchorNoise(restPoseAnchorNoise);

  const int restPoseLoopSteps = config("restPoseLoopSteps", 0);
  if(restPoseLoopSteps < 0)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("restPoseLoopSteps must be non-negative");
  }
  observer_.setRestPoseLoopSteps(static_cast<size_t>(restPoseLoopSteps));
  if(restPoseLoopSteps > 0)
  {
    mc_rtc::log::info("[MCKineticsObserverFG] contact loop over {} steps ({:.3f} s); it needs the "
                      "smoother window to cover that span, raise KO_FG_SMOOTHER_LAG if it does not",
                      restPoseLoopSteps, restPoseLoopSteps * ctl.timeStep);
  }

  // How far back the fixed-lag smoother can still revise its estimate. Hard-coded to 0.016 s, about
  // three steps at 200 Hz, which leaves almost no history to reconcile a contact removal against.
  // KO_FG_SMOOTHER_LAG overrides it for experiments; reading it from the observer's configuration
  // instead is not possible for now, see the note in the session log.
  if(const char * lag = std::getenv("KO_FG_SMOOTHER_LAG"))
  {
    const double smootherLag = std::atof(lag);
    if(!(smootherLag > 0.0) || !std::isfinite(smootherLag))
    {
      mc_rtc::log::error_and_throw<std::invalid_argument>("KO_FG_SMOOTHER_LAG must be finite and positive");
    }
    observer_.setSmootherLag(smootherLag);
    mc_rtc::log::info("[MCKineticsObserverFG] retard du lisseur force a {} s", smootherLag);
  }

  wrenchCalibration_.clear();
  for(const auto & [surface, correction] : config("wrenchCalibration", std::map<std::string, std::vector<double>>{}))
  {
    if(correction.size() != 3
       || !std::all_of(correction.begin(), correction.end(), [](double value) { return std::isfinite(value); }))
    {
      mc_rtc::log::error_and_throw<std::invalid_argument>("wrenchCalibration.{} must contain three finite values",
                                                          surface);
    }
    wrenchCalibration_.emplace(surface, so::Vector3(correction[0], correction[1], correction[2]));
  }

  if(config.has("jointTorques"))
  {
    auto jointTorquesConfig = config("jointTorques");
    useJointTorqueMeasurements_ = jointTorquesConfig("enabled", false);
    if(useJointTorqueMeasurements_)
    {
      useJointTorqueCommandAsMeasurement_ = jointTorquesConfig("useCommandAsMeasurement", true);
      jointTorquesConfig("noise", jointTorqueNoise_);
      jointTorquesConfig("modelNoise", jointMomentumModelNoise_);
      jointTorquesConfig("jointAccelerationInitNoise", jointAccelerationInitNoise_);
      jointTorquesConfig("jointAccelerationProcessNoise", jointAccelerationProcessNoise_);
      jointTorquesConfig("jointAccelerationFiniteDifferenceNoise", jointAccelerationFiniteDifferenceNoise_);
      jointTorquesConfig("momentumNoise", jointMomentumNoise_);
      const std::string formulation = jointTorquesConfig("formulation", std::string("fullDynamics"));
      if(formulation == "fullDynamics") { jointTorqueFormulation_ = JointTorqueFormulation::FullDynamics; }
      else if(formulation == "reducedMomentum") { jointTorqueFormulation_ = JointTorqueFormulation::ReducedMomentum; }
      else
      {
        mc_rtc::log::error_and_throw<std::invalid_argument>(
            "jointTorques.formulation must be 'fullDynamics' or 'reducedMomentum', got '{}'", formulation);
      }
      mc_rtc::log::info("[{}]: joint torque formulation: {}", name(), formulation);
      mc_rtc::log::info(
          "[{}]: Latent joint-acceleration torque factors enabled: torque sigma={}, model sigma={}, "
          "qdd finite-difference sigma={}, qdd process sigma={}, command fallback={}.",
          name(), jointTorqueNoise_, jointMomentumModelNoise_, jointAccelerationFiniteDifferenceNoise_,
          jointAccelerationProcessNoise_,
          useJointTorqueCommandAsMeasurement_);
    }
    else
    {
      // Keep the disabled path independent of every other jointTorques key.
      // In particular, merely adding a noise entry must not alter the graph.
      useJointTorqueCommandAsMeasurement_ = false;
      mc_rtc::log::info("[{}]: Joint acceleration torque factor disabled.", name());
    }
  }
}

void MCKineticsObserverFG::reset(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);

  mass_ = ctl.realRobot(robot_).mass();
  observer_.setMass(mass_);
  contactsManager_.reset();
  contactsDetector_.reset();
  maintainedContacts_.clear();

  /* Initialization of variables */
  X_0_fb_ = sva::PTransformd::Identity();
  v_fb_0_ = sva::MotionVecd::Zero();
  a_fb_0_ = sva::MotionVecd::Zero();
  worldAnchorPos_.setZero();
  fbAnchorPos_.setZero();
  inputWrench_.setZero();
  for(auto & [surface, wrench] : ignoredWrenchesInCentroid_)
  {
    (void)surface;
    wrench.setZero();
  }

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  my_robots_->robotCopy(realRobot, "inputRobot");
  if(withGui_)
  {
    ctl.gui()->addElement(
        {"Robots"}, mc_rtc::gui::Robot(name(), [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));
    ctl.gui()->addElement({"Robots"}, mc_rtc::gui::Robot("Real", [this, &ctl]() -> const mc_rbdyn::Robot &
                                                         { return ctl.realRobot(robot_); }));
  }

  X_0_fb_ = realRobot.posW();

  auto & inputRobot = my_robots_->robot("inputRobot");

  // Copy the real configuration except for the floating base
  const auto & realQ = realRobot.mbc().q;
  const auto & realAlpha = realRobot.mbc().alpha;
  const auto & realAlphaD = realRobot.mbc().alphaD;

  std::copy(std::next(realQ.begin()), realQ.end(), std::next(inputRobot.mbc().q.begin()));
  std::copy(std::next(realAlpha.begin()), realAlpha.end(), std::next(inputRobot.mbc().alpha.begin()));
  std::copy(std::next(realAlphaD.begin()), realAlphaD.end(), std::next(inputRobot.mbc().alphaD.begin()));

  inputRobot.forwardKinematics();
  inputRobot.forwardVelocity();
  inputRobot.forwardAcceleration();

  // The input robot copies the real robot to update the encoder values.
  // Then its floating base is brung back to the origin of the world frame and given zero velocities and accelerations
  // in order to ease the computations.
  inputRobot.posW(zeroPose_);
  inputRobot.velW(zeroMotion_);
  inputRobot.accW(zeroMotion_);

  est_worldFbKine_ = conversions::kinematics::fromSva(realRobot.posW(), realRobot.velW(), realRobot.accW());

  /** Center of mass (assumes FK, FV and FA are already done)
      Must be initialized now as used for the conversion from user to centroid frame !!! **/
  fbCentroidKine_.position = inputRobot.com();
  fbCentroidKine_.orientation.setZeroRotation();
  fbCentroidKine_.linVel = inputRobot.comVelocity();
  fbCentroidKine_.angVel.set().setZero();
  fbCentroidKine_.linAcc = inputRobot.comAcceleration();
  fbCentroidKine_.angAcc.set().setZero();

  centroidFbKine_ = fbCentroidKine_.getInverse();

  worldCentroidKine_ = est_worldFbKine_ * fbCentroidKine_;

  Vector initState;
  initState.resize(28);
  auto initPos = initState.segment<3>(0);
  auto initQuat = initState.segment<4>(3);
  auto initLinVel = initState.segment<3>(7);
  auto initAngVel = initState.segment<3>(10);
  auto initLinAcc = initState.segment<3>(13);
  auto initAngAcc = initState.segment<3>(16);
  auto initExtForce = initState.segment<3>(19);
  auto initExtTorque = initState.segment<3>(22);
  auto initBias = initState.segment<3>(25);

  initPos = worldCentroidKine_.orientation.toMatrix3().transpose() * worldCentroidKine_.position();
  initQuat = worldCentroidKine_.orientation.toQuaternion().coeffs();
  initLinVel = worldCentroidKine_.orientation.toMatrix3().transpose() * worldCentroidKine_.linVel();
  initAngVel = worldCentroidKine_.orientation.toMatrix3().transpose() * worldCentroidKine_.angVel();
  initLinAcc = worldCentroidKine_.orientation.toMatrix3().transpose() * worldCentroidKine_.linAcc();
  initAngAcc = worldCentroidKine_.orientation.toMatrix3().transpose() * worldCentroidKine_.angAcc();
  initExtForce.setZero();
  initExtTorque.setZero();
  initBias.setZero();

  observer_.init(mass_, initState, initNoises_, processNoises_, imuNoises_, std::nullopt, withRestPoseAverageFactor_);

  // These caches are outside the factor graph and must never survive a reset,
  // including when joint torque measurements have just been disabled.
  previousMomentumEndpoint_.reset();
  previousMeasuredJointTorques_.reset();
  pendingMomentumMeasurement_.reset();
  pendingMomentumContacts_.clear();
  pendingMomentumTime_ = 0;
  previousJointVelocity_.reset();
  pendingJointAccelerationMeasurement_.reset();
  pendingJointAccelerationContacts_.clear();
  pendingJointAccelerationTime_ = 0;

  if(useJointTorqueMeasurements_)
  {
    const size_t jointTorqueDim = static_cast<size_t>(actuatedJointTorqueDim(realRobot));

    if(jointTorqueDim > 0)
    {
      inputJointTorques_ = Vector::Zero(jointTorqueDim);
      measuredJointTorques_ = Vector::Zero(jointTorqueDim);
      estimatedJointTorqueResidual_ = Vector::Zero(jointTorqueDim);
      jointAccelerationFiniteDifference_ = Vector::Zero(jointTorqueDim);
      const Vector initialJointAcceleration = Vector::Zero(jointTorqueDim);
      observer_.configureJointAcceleration(
          jointTorqueDim, initialJointAcceleration,
          Vector::Constant(jointTorqueDim, jointAccelerationInitNoise_),
          Vector::Constant(jointTorqueDim, jointAccelerationProcessNoise_));
    }
    else
    {
      useJointTorqueMeasurements_ = false;
      mc_rtc::log::warning("[{}]: Joint torque measurements disabled: no actuated DoF found.", name());
    }
  }

  k_ = 0;
}

so::Matrix3 computeCentroidalInertia(const rbd::MultiBody & mb,
                                     const rbd::MultiBodyConfig & mbc,
                                     const so::Vector3 & com)
{
  using namespace Eigen;

  const std::vector<rbd::Body> & bodies = mb.bodies();
  sva::RBInertiad Ic(0.0, Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero());

  sva::PTransformd X_com_0(so::Vector3(-com));
  for(size_t i = 0; i < static_cast<size_t>(mb.nrBodies()); ++i)
  {
    auto X_i_com = mbc.bodyPosW[i] * X_com_0;
    Ic += X_i_com.transMul(bodies[i].inertia());
  }

  return Ic.inertia();
}

bool MCKineticsObserverFG::run(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  auto & inputRobot = my_robots_->robot("inputRobot");
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  // Copy the real configuration except for the floating base
  const auto & realQ = realRobot.mbc().q;
  const auto & realAlpha = realRobot.mbc().alpha;
  const auto & realAlphaD = realRobot.mbc().alphaD;

  std::copy(std::next(realQ.begin()), realQ.end(), std::next(inputRobot.mbc().q.begin()));
  std::copy(std::next(realAlpha.begin()), realAlpha.end(), std::next(inputRobot.mbc().alpha.begin()));
  std::copy(std::next(realAlphaD.begin()), realAlphaD.end(), std::next(inputRobot.mbc().alphaD.begin()));

  inputRobot.forwardKinematics();
  inputRobot.forwardVelocity();
  inputRobot.forwardAcceleration();

  // The input robot copies the real robot to update the encoder values.
  // Then its floating base is brung back to the origin of the world frame and given zero velocities and accelerations
  // in order to ease the computations.
  inputRobot.posW(zeroPose_);
  inputRobot.velW(zeroMotion_);
  inputRobot.accW(zeroMotion_);

  fbCentroidKine_.position = inputRobot.com();
  fbCentroidKine_.orientation.setZeroRotation();
  fbCentroidKine_.linVel = inputRobot.comVelocity();
  fbCentroidKine_.angVel.set().setZero();
  fbCentroidKine_.linAcc = inputRobot.comAcceleration();
  fbCentroidKine_.angAcc.set().setZero();

  centroidFbKine_ = fbCentroidKine_.getInverse();

  Matrix3 inertiaMatrix = computeCentroidalInertia(inputRobot.mb(), inputRobot.mbc(), fbCentroidKine_.position());

  Vector3 angularMomentum =
      rbd::computeCentroidalMomentum(inputRobot.mb(), inputRobot.mbc(), fbCentroidKine_.position()).moment();

  inputJointTorques_ = jointTorqueVectorFromRefOrder(robot);
  observer_.setInput(dt_, inertiaMatrix, angularMomentum, inputWrench_.segment(0, 3), inputWrench_.segment(3, 3));

  // update of the contacts
  updateContacts(ctl, logger);

  // force measurements from sensor that are not associated to a currently set contact are given to the Kinetics
  // Observer as inputs.
  inputWrench_ = inputAdditionalWrench(inputRobot, robot);

  /** Accelerometers **/
  updateIMUs(robot, inputRobot);

  if(useJointTorqueMeasurements_) { updateJointTorqueMeasurement(realRobot, inputRobot); }

  try
  {
    observer_.runIteration(k_);
    if(useJointTorqueMeasurements_) { updateEstimatedJointTorqueResidual(); }
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error("[{}]: Factor-graph iteration {} failed, keeping previous estimate. {}", name(), k_, e.what());
  }

  unbiasedDisturbanceWrench_.force() = observer_.getCurrentState().disturbForce_ - disturbanceWrenchOffset_.force();
  unbiasedDisturbanceWrench_.moment() = observer_.getCurrentState().disturbMoment_ - disturbanceWrenchOffset_.moment();

  if(removeWrenchOffset_)
  {
    if(wrenchOffsetIndex_ > 100)
    {
      removeWrenchOffset_ = false;
      mc_rtc::log::info("Disturbance wrench offset removed");
    }

    auto disturbForce = observer_.getCurrentState().disturbForce_;
    auto disturbMoment = observer_.getCurrentState().disturbMoment_;

    disturbanceWrenchOffset_.force() =
        disturbanceWrenchOffset_.force() + 0.1 * (disturbForce - disturbanceWrenchOffset_.force());
    disturbanceWrenchOffset_.moment() =
        disturbanceWrenchOffset_.moment() + 0.1 * (disturbMoment - disturbanceWrenchOffset_.moment());

    wrenchOffsetIndex_++;
  }

  // Estimated kinematics of the centroid frame in the world frame.
  est_worldCentroidKine_ = fgLocKineToSoKine(observer_.getCurrentState().kine_);
  est_worldFbKine_ = est_worldCentroidKine_ * centroidFbKine_;

  if(odometryType_ != so::odometry::OdometryType::None)
  {
    X_0_fb_.rotation() = est_worldFbKine_.orientation.toMatrix3().transpose();
    X_0_fb_.translation() = est_worldFbKine_.position();

    v_fb_0_.angular() = est_worldFbKine_.angVel();
    v_fb_0_.linear() = est_worldFbKine_.linVel();

    a_fb_0_.angular() = est_worldFbKine_.angAcc();
    a_fb_0_.linear() = est_worldFbKine_.linAcc();
  }
  else
  {
    if(!maintainedContacts_.empty())
    {
      worldAnchorPos_.setZero();
      fbAnchorPos_.setZero();

      double forceSum = 0.0;
      for(auto & [id, contact] : maintainedContacts_)
      {
        worldAnchorPos_ +=
            getCtlContactWorldKinematics(ctl, *contact, false).position() * contact->contactWrenchVector_(2);
        fbAnchorPos_ +=
            getContactWorldKinematics(ctl, *contact, inputRobot, false).position() * contact->contactWrenchVector_(2);
        forceSum += contact->contactWrenchVector_(2);
      }

      if(std::abs(forceSum) > 1e-9)
      {
        worldAnchorPos_ /= forceSum;
        fbAnchorPos_ /= forceSum;
      }
    }

    so::kine::Kinematics worldFbKine(est_worldFbKine_);
    worldFbKine.orientation = so::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(est_worldFbKine_.orientation.toMatrix3(),
                                                                             ctl.robot(robot_).posW().rotation().transpose());
    worldFbKine.position = worldAnchorPos_ - worldFbKine.orientation.toMatrix3() * fbAnchorPos_;

    X_0_fb_.rotation() = worldFbKine.orientation.toMatrix3().transpose();
    X_0_fb_.translation() = worldFbKine.position();

    v_fb_0_.angular() = worldFbKine.angVel();
    v_fb_0_.linear() = worldFbKine.linVel();

    a_fb_0_.angular() = worldFbKine.angAcc();
    a_fb_0_.linear() = worldFbKine.linAcc();
    est_worldFbKine_ = worldFbKine;
  }

  if(withDebugLogs_)
  {
    /* Update of the logged variables */
    for(auto & [id, contact] : maintainedContacts_)
    {
      contact->viscoElasticWrenchAfterCorrection_.segment(0, 3) = observer_.getContact(id).currentState_.force_;
      contact->viscoElasticWrenchAfterCorrection_.segment(3, 3) = observer_.getContact(id).currentState_.moment_;
    }
  }

  /* Update of the visual representation (only a visual feature) of the observed robot */
  my_robots_->robot().mbc().q = ctl.realRobot(robot_).mbc().q;

  /* Update of the observed robot */
  update(my_robots_->robot());

  k_++;
  return true;
} // namespace mc_state_observation

///////////////////////////////////////////////////////////////////////
/// -------------------------Called functions--------------------------
///////////////////////////////////////////////////////////////////////

void MCKineticsObserverFG::update(mc_control::MCController & ctl) // this function is called by the pipeline if the
                                                                  // update is set to true in the configuration file
{
  auto & realRobot = ctl.realRobot(robot_);
  update(realRobot);
  realRobot.forwardKinematics();
  realRobot.forwardVelocity();
}

// used only to update the visual representation of the estimated robot
void MCKineticsObserverFG::update(mc_rbdyn::Robot & robot)
{
  robot.posW(X_0_fb_);
  robot.velW(v_fb_0_.vector());
}

Vector6 MCKineticsObserverFG::inputAdditionalWrench(const mc_rbdyn::Robot & inputRobot,
                                                    const mc_rbdyn::Robot & measRobot)
{
  Vector6 additionalWrench = Vector6::Zero();

  for(const auto & forceSensor : measRobot.forceSensors())
  {
    const auto ignored = ignoredForceSensorSurfaces_.find(forceSensor.name());
    if(ignored != ignoredForceSensorSurfaces_.end())
    {
      const sva::ForceVecd measuredWrench = wrenchInFloatingBaseFrame(forceSensor, inputRobot);
      auto & centroidWrench = ignoredWrenchesInCentroid_.at(ignored->second);
      centroidWrench.segment<3>(0) = measuredWrench.force();
      centroidWrench.segment<3>(3) =
          measuredWrench.moment() - fbCentroidKine_.position().cross(measuredWrench.force());
      continue;
    }

    bool usedByContact = false;
    for(const auto & [_, contact] : contactsManager_.contacts())
    {
      if(contact.isSet() && contact.fsName() == forceSensor.name())
      {
        usedByContact = true;
        break;
      }
    }

    if(!usedByContact)
    {
      const sva::ForceVecd measuredWrench = wrenchInFloatingBaseFrame(forceSensor, inputRobot);
      additionalWrench.segment(0, 3) += measuredWrench.force();
      additionalWrench.segment(3, 3) +=
          measuredWrench.moment() - fbCentroidKine_.position().cross(measuredWrench.force());
    }
  }

  return additionalWrench;
}

sva::ForceVecd MCKineticsObserverFG::wrenchWithoutGravity(const mc_rbdyn::ForceSensor & forceSensor,
                                                          const sva::PTransformd & X_fb_parent,
                                                          const Eigen::Matrix3d & R_fb_world) const
{
  const auto & calibration = forceSensor.calib();
  const sva::PTransformd X_fb_ds = calibration.X_f_ds * forceSensor.X_p_f() * X_fb_parent;
  const sva::PTransformd X_fb_vb(X_fb_ds.inv().rotation(),
                                 (calibration.X_p_vb * X_fb_parent * X_fb_ds.inv()).translation());

  sva::ForceVecd gravityWrench = calibration.worldForce;
  gravityWrench.force() = R_fb_world * calibration.worldForce.force();
  gravityWrench.moment() = R_fb_world * calibration.worldForce.moment();
  return forceSensor.wrench() - calibration.offset - X_fb_vb.transMul(gravityWrench);
}

sva::ForceVecd MCKineticsObserverFG::wrenchInFloatingBaseFrame(const mc_rbdyn::ForceSensor & forceSensor,
                                                               const mc_rbdyn::Robot & inputRobot) const
{
  const auto & X_fb_parent = inputRobot.mbc().bodyPosW[inputRobot.bodyIndexByName(forceSensor.parentBody())];
  const sva::ForceVecd gravityFreeWrench = wrenchWithoutGravity(forceSensor, X_fb_parent, X_0_fb_.rotation());
  return (X_fb_parent.inv() * forceSensor.X_fsactual_parent()).dualMul(gravityFreeWrench);
}

void MCKineticsObserverFG::updateIMUs(const mc_rbdyn::Robot & measRobot, const mc_rbdyn::Robot & inputRobot)
{
  for(size_t i = 0; i < listIMUs_.size(); ++i)
  {
    const auto & imu = measRobot.bodySensor(listIMUs_[i].name());

    /** Position of accelerometer **/
    const sva::PTransformd & bodyImuPose = imu.X_b_s();
    so::kine::Kinematics bodyImuKine = conversions::kinematics::fromSva(
        bodyImuPose, so::kine::Kinematics::Flags::vel | so::kine::Kinematics::Flags::acc);

    so::kine::Kinematics fbBodyKine = conversions::kinematics::fromSva(
        inputRobot.mbc().bodyPosW[inputRobot.bodyIndexByName(imu.parentBody())],
        inputRobot.mbc().bodyVelW[inputRobot.bodyIndexByName(imu.parentBody())],
        inputRobot.mbc().bodyAccB[inputRobot.bodyIndexByName(imu.parentBody())], true, false);

    so::kine::Kinematics fbImuKine = fbBodyKine * bodyImuKine;

    so::kine::Kinematics centroidImuKinematics = centroidFbKine_ * fbImuKine;
    ko_fg::Kinematics centroidImuKine;
    centroidImuKine.pose(
        gtsam::Pose3(gtsam::Rot3(centroidImuKinematics.orientation.toMatrix3()), centroidImuKinematics.position()));
    centroidImuKine.linVel(centroidImuKinematics.linVel());
    centroidImuKine.angVel(centroidImuKinematics.angVel());
    centroidImuKine.linAcc(centroidImuKinematics.linAcc());
    centroidImuKine.angAcc(centroidImuKinematics.angAcc());

    observer_.updateIMU(k_, i, imu.linearAcceleration(), imu.angularVelocity(), centroidImuKine);
  }
}

Eigen::VectorXd MCKineticsObserverFG::measuredJointTorqueVector(const mc_rbdyn::Robot & robot) const
{
  const Eigen::Index expectedDim = actuatedJointTorqueDim(robot);
  const auto & tau = robot.jointTorques();

  // A robot with no joint torque sensing does not necessarily give an EMPTY vector: RHPS1 carries
  // 42 channels that are exactly zero from the first sample to the last, in tauIn and tauOut alike.
  // Taken at face value those zeros are a measurement saying "every joint is unloaded", which is
  // false and which the dynamics factors then have to fight. A standing humanoid holds tens of N.m
  // against gravity, so an all-zero vector can only mean absent data.
  const bool allZero =
      !tau.empty() && std::all_of(tau.begin(), tau.end(), [](double v) { return std::abs(v) < 1e-9; });
  if(allZero)
  {
    if(!warnedAboutNullJointTorques_)
    {
      warnedAboutNullJointTorques_ = true;
      mc_rtc::log::warning("[{}]: every joint torque reads exactly zero, so this robot has no torque "
                           "sensing; the joint-torque factors are disabled for this run.",
                           name());
    }
    return {};
  }

  if(tau.empty())
  {
    if(useJointTorqueCommandAsMeasurement_) { return jointTorqueVectorFromRefOrder(robot); }
    return {};
  }

  const auto & refJointOrder = robot.refJointOrder();
  if(tau.size() == refJointOrder.size())
  {
    Eigen::VectorXd selected(expectedDim);
    Eigen::Index out = 0;
    for(size_t refIndex = 0; refIndex < refJointOrder.size(); ++refIndex)
    {
      const auto & jointName = refJointOrder[refIndex];
      const int jointIndex = robot.mb().jointIndexByName(jointName);
      const int dof = robot.mb().joint(jointIndex).dof();
      if(dof == 0) { continue; }
      if(dof != 1)
      {
        mc_rtc::log::warning(
            "[{}]: Joint torque sensor entry '{}' maps to a {}-DoF joint and cannot be expanded unambiguously.", name(),
            jointName, dof);
        return {};
      }
      selected(out++) = tau[refIndex];
    }
    return selected;
  }

  if(static_cast<Eigen::Index>(tau.size()) == expectedDim)
  {
    return Eigen::Map<const Eigen::VectorXd>(tau.data(), static_cast<Eigen::Index>(tau.size()));
  }

  mc_rtc::log::warning(
      "[{}]: Joint torque measurement size ({}) matches neither refJointOrder ({}) nor selected torque size ({}).",
      name(), tau.size(), refJointOrder.size(), expectedDim);
  return {};
}

Eigen::Index MCKineticsObserverFG::actuatedJointTorqueDim(const mc_rbdyn::Robot & robot) const
{
  Eigen::Index dim = 0;
  for(const auto & jointName : robot.refJointOrder())
  {
    const int jointIndex = robot.mb().jointIndexByName(jointName);
    dim += robot.mb().joint(jointIndex).dof();
  }
  return dim;
}

std::vector<std::string> MCKineticsObserverFG::actuatedJointTorqueNames(const mc_rbdyn::Robot & robot) const
{
  std::vector<std::string> names;
  names.reserve(static_cast<size_t>(actuatedJointTorqueDim(robot)));

  for(const auto & jointName : robot.refJointOrder())
  {
    const int jointIndex = robot.mb().jointIndexByName(jointName);
    const int dof = robot.mb().joint(jointIndex).dof();
    if(dof == 1) { names.push_back(jointName); }
    else
    {
      for(int i = 0; i < dof; ++i) { names.push_back(jointName + "_" + std::to_string(i)); }
    }
  }

  return names;
}

Eigen::VectorXd MCKineticsObserverFG::jointTorqueVectorFromRefOrder(const mc_rbdyn::Robot & robot) const
{
  const Eigen::Index dim = actuatedJointTorqueDim(robot);
  Eigen::VectorXd tau(dim);
  Eigen::Index out = 0;

  for(const auto & jointName : robot.refJointOrder())
  {
    const int jointIndex = robot.mb().jointIndexByName(jointName);
    const auto & jointTau = robot.mbc().jointTorque[static_cast<size_t>(jointIndex)];
    for(double value : jointTau) { tau(out++) = value; }
  }

  return tau;
}

Eigen::VectorXd MCKineticsObserverFG::actuatedJointVelocityVector(const mc_rbdyn::Robot & robot) const
{
  Eigen::VectorXd velocity(actuatedJointTorqueDim(robot));
  Eigen::Index out = 0;
  for(const auto & jointName : robot.refJointOrder())
  {
    const int jointIndex = robot.mb().jointIndexByName(jointName);
    for(double value : robot.mbc().alpha[static_cast<size_t>(jointIndex)]) { velocity(out++) = value; }
  }
  return velocity;
}

std::shared_ptr<const ko_fg::RecursiveJointTorqueModel> MCKineticsObserverFG::makeJointTorqueModel(
    mc_rbdyn::Robot & dynamicsRobot) const
{
  const auto & mb = dynamicsRobot.mb();
  rbd::MultiBodyConfig mbc = dynamicsRobot.mbc();
  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  const auto & predecessors = mb.predecessors();
  const auto & successors = mb.successors();
  if(successors.empty() || successors[0] != 0 || mb.joint(0).dof() != 6)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[{}]: Latent joint acceleration model expects a six-DoF root joint.", name());
  }

  ko_fg::RecursiveJointTorqueModel::Snapshot snapshot;
  snapshot.bodies_.resize(static_cast<size_t>(mb.nrBodies()));
  std::vector<int> bodyJoint(static_cast<size_t>(mb.nrBodies()), -1);
  for(size_t joint = 0; joint < static_cast<size_t>(mb.nrJoints()); ++joint)
  {
    bodyJoint[static_cast<size_t>(successors[joint])] = static_cast<int>(joint);
  }

  for(size_t body = 0; body < snapshot.bodies_.size(); ++body)
  {
    const int joint = bodyJoint[body];
    if(joint < 0)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("[{}]: Body without predecessor joint.", name());
    }
    auto & data = snapshot.bodies_[body];
    data.parent_ = predecessors[static_cast<size_t>(joint)];
    data.motionParentToBody_ = mbc.parentToSon[static_cast<size_t>(joint)].matrix();
    data.spatialInertia_ = mb.body(static_cast<int>(body)).inertia().matrix();
    data.motionSubspace_ = mbc.motionSubspace[static_cast<size_t>(joint)];
    if(body > 0) { data.jointVelocity_ = mbc.jointVelocity[static_cast<size_t>(joint)].vector(); }
    data.jointAcceleration_.setZero();
  }

  for(const auto & jointName : dynamicsRobot.refJointOrder())
  {
    const int joint = mb.jointIndexByName(jointName);
    const size_t body = static_cast<size_t>(successors[static_cast<size_t>(joint)]);
    for(int dof = 0; dof < mb.joint(joint).dof(); ++dof)
    {
      snapshot.outputs_.push_back({body, static_cast<size_t>(dof)});
    }
  }

  snapshot.centroidToBasePosition_ = centroidFbKine_.position();
  snapshot.centroidToBaseVelocity_ = centroidFbKine_.linVel();
  const Eigen::Index jointDimension = actuatedJointTorqueDim(dynamicsRobot);
  const auto relativeAccelerationAt = [&](const Eigen::VectorXd & jointAcceleration)
  {
    rbd::MultiBodyConfig accelerationMbc = mbc;
    for(auto & joint : accelerationMbc.alphaD) { std::fill(joint.begin(), joint.end(), 0.0); }
    Eigen::Index offset = 0;
    for(const auto & jointName : dynamicsRobot.refJointOrder())
    {
      const int joint = mb.jointIndexByName(jointName);
      auto & alphaD = accelerationMbc.alphaD[static_cast<size_t>(joint)];
      for(double & value : alphaD) { value = jointAcceleration(offset++); }
    }
    rbd::forwardAcceleration(mb, accelerationMbc);

    so::kine::Kinematics fbCentroid;
    fbCentroid.position = rbd::computeCoM(mb, accelerationMbc);
    fbCentroid.orientation.setZeroRotation();
    fbCentroid.linVel = rbd::computeCoMVelocity(mb, accelerationMbc);
    fbCentroid.angVel.set().setZero();
    fbCentroid.linAcc = rbd::computeCoMAcceleration(mb, accelerationMbc);
    fbCentroid.angAcc.set().setZero();
    return fbCentroid.getInverse().linAcc();
  };
  const Eigen::VectorXd zeroJointAcceleration = Eigen::VectorXd::Zero(jointDimension);
  snapshot.centroidToBaseAcceleration_ = relativeAccelerationAt(zeroJointAcceleration);
  snapshot.centroidToBaseAccelerationJacobian_.resize(3, jointDimension);
  for(Eigen::Index column = 0; column < jointDimension; ++column)
  {
    Eigen::VectorXd unit = Eigen::VectorXd::Zero(jointDimension);
    unit(column) = 1.0;
    snapshot.centroidToBaseAccelerationJacobian_.col(column) =
        relativeAccelerationAt(unit) - snapshot.centroidToBaseAcceleration_;
  }
  snapshot.gravityWorld_ = mbc.gravity;

  for(const auto & [contactId, graphContact] : observer_.getActiveContacts())
  {
    (void)graphContact;
    const auto maintained = maintainedContacts_.find(static_cast<unsigned>(contactId));
    if(maintained == maintainedContacts_.end()) { continue; }
    const auto & contact = *maintained->second;
    const auto & surface = dynamicsRobot.surface(contact.surfaceName());
    const size_t body = dynamicsRobot.bodyIndexByName(surface.bodyName());
    Eigen::Matrix<double, 6, 6> bodyWrenchMap;
    const Eigen::Matrix3d rotation = contact.fbContactKine_.orientation.toMatrix3();
    const Eigen::Vector3d position = contact.fbContactKine_.position();
    for(Eigen::Index column = 0; column < 6; ++column)
    {
      Eigen::Matrix<double, 6, 1> unit = Eigen::Matrix<double, 6, 1>::Zero();
      unit(column) = 1.0;
      const Eigen::Vector3d force = rotation * unit.head<3>();
      const Eigen::Vector3d moment = rotation * unit.tail<3>() + position.cross(force);
      bodyWrenchMap.col(column) =
          mbc.bodyPosW[body].dualMul(sva::ForceVecd(moment, force)).vector();
    }
    snapshot.contacts_.push_back({contactId, body, bodyWrenchMap});
  }

  return std::make_shared<ko_fg::RecursiveJointTorqueModel>(std::move(snapshot));
}

ko_fg::MomentumResidualEndpoint MCKineticsObserverFG::makeMomentumResidualEndpoint(mc_rbdyn::Robot & dynamicsRobot)
{
  const auto & mb = dynamicsRobot.mb();
  rbd::MultiBodyConfig mbc = dynamicsRobot.mbc();
  rbd::forwardKinematics(mb, mbc);
  rbd::forwardVelocity(mb, mbc);

  ko_fg::MomentumResidualEndpoint endpoint;
  endpoint.linearizationPose_ = observer_.getCurrentState().kine_.pose();
  endpoint.gravityWorld_ = mbc.gravity;

  const Eigen::Index fullDimension = mb.nrDof();
  const Eigen::Index measuredDimension = actuatedJointTorqueDim(dynamicsRobot);
  const Eigen::Index residualDimension = 6 + measuredDimension;

  const auto & predecessors = mb.predecessors();
  const auto & successors = mb.successors();
  if(successors.empty() || successors[0] != 0 || mb.joint(0).dof() != 6)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}]: Momentum residual expects a six-DoF joint as tree root.",
                                                     name());
  }

  std::vector<Eigen::Index> jointOffsets(static_cast<size_t>(mb.nrJoints()));
  std::vector<int> bodyJoint(static_cast<size_t>(mb.nrBodies()), -1);
  Eigen::Index offset = 0;
  for(size_t joint = 0; joint < static_cast<size_t>(mb.nrJoints()); ++joint)
  {
    const size_t body = static_cast<size_t>(successors[joint]);
    jointOffsets[joint] = offset;
    bodyJoint[body] = static_cast<int>(joint);
    offset += mb.joint(static_cast<int>(joint)).dof();
  }

  std::vector<Eigen::Index> selectedRows;
  selectedRows.reserve(static_cast<size_t>(residualDimension));
  for(Eigen::Index row = 0; row < 6; ++row) { selectedRows.push_back(row); }
  for(const auto & jointName : dynamicsRobot.refJointOrder())
  {
    const int jointIndex = mb.jointIndexByName(jointName);
    for(int dof = 0; dof < mb.joint(jointIndex).dof(); ++dof)
    {
      selectedRows.push_back(jointOffsets[static_cast<size_t>(jointIndex)] + dof);
    }
  }
  if(static_cast<Eigen::Index>(selectedRows.size()) != residualDimension)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}]: Invalid momentum residual row selection.", name());
  }

  const auto selectRows = [&selectedRows](const Eigen::MatrixXd & matrix)
  {
    Eigen::MatrixXd selected(static_cast<Eigen::Index>(selectedRows.size()), matrix.cols());
    for(size_t row = 0; row < selectedRows.size(); ++row)
    {
      selected.row(static_cast<Eigen::Index>(row)) = matrix.row(selectedRows[row]);
    }
    return selected;
  };
  const auto selectVector = [&selectedRows](const Eigen::VectorXd & vector)
  {
    Eigen::VectorXd selected(static_cast<Eigen::Index>(selectedRows.size()));
    for(size_t row = 0; row < selectedRows.size(); ++row)
    {
      selected(static_cast<Eigen::Index>(row)) = vector(selectedRows[row]);
    }
    return selected;
  };

  Eigen::VectorXd velocityOffset = rbd::dofToVector(mb, mbc.alpha);
  Eigen::MatrixXd velocityMap = Eigen::MatrixXd::Zero(fullDimension, 6);
  velocityOffset.head<3>().setZero();
  velocityOffset.segment<3>(3) = centroidFbKine_.linVel();
  velocityMap.block<3, 3>(0, 3).setIdentity();
  velocityMap.block<3, 3>(3, 0).setIdentity();
  velocityMap.block<3, 3>(3, 3) = -sva::vector3ToCrossMatrix(centroidFbKine_.position());

  rbd::ForwardDynamics forwardDynamics(mb);
  forwardDynamics.computeH(mb, mbc);
  const Eigen::MatrixXd & inertia = forwardDynamics.H();
  endpoint.momentumOffset_ = selectVector(inertia * velocityOffset);
  endpoint.momentumJacobian_ = selectRows(inertia * velocityMap);
  endpoint.momentumPoseJacobian_ = Eigen::MatrixXd::Zero(residualDimension, 6);

  rbd::Coriolis coriolis(mb);
  const auto coriolisAt = [&](const Eigen::VectorXd & velocity)
  {
    rbd::MultiBodyConfig velocityMbc = mbc;
    velocityMbc.alpha = rbd::vectorToDof(mb, velocity);
    rbd::forwardVelocity(mb, velocityMbc);
    return Eigen::MatrixXd(coriolis.coriolis(mb, velocityMbc));
  };

  const Eigen::MatrixXd coriolisOffset = coriolisAt(velocityOffset);
  endpoint.coriolisOffset_ = selectVector(coriolisOffset.transpose() * velocityOffset);
  Eigen::MatrixXd fullLinear = coriolisOffset.transpose() * velocityMap;
  std::array<Eigen::MatrixXd, 6> fullQuadratic;
  for(Eigen::Index column = 0; column < 6; ++column)
  {
    const Eigen::MatrixXd direction = coriolisAt(velocityOffset + velocityMap.col(column)) - coriolisOffset;
    fullLinear.col(column).noalias() += direction.transpose() * velocityOffset;
    fullQuadratic[static_cast<size_t>(column)] = direction.transpose() * velocityMap;
  }
  endpoint.coriolisLinear_ = selectRows(fullLinear);
  endpoint.coriolisPoseJacobian_ = Eigen::MatrixXd::Zero(residualDimension, 6);
  for(size_t column = 0; column < 6; ++column)
  {
    endpoint.coriolisQuadratic_[column] = selectRows(fullQuadratic[column]);
  }

  rbd::MultiBodyConfig gravityMbc = mbc;
  gravityMbc.alpha = rbd::vectorToDof(mb, Eigen::VectorXd::Zero(fullDimension));
  gravityMbc.force.assign(static_cast<size_t>(mb.nrBodies()), sva::ForceVecd::Zero());
  rbd::forwardVelocity(mb, gravityMbc);
  endpoint.gravityMap_.resize(residualDimension, 3);
  for(Eigen::Index column = 0; column < 3; ++column)
  {
    gravityMbc.gravity.setZero();
    gravityMbc.gravity(column) = 1.0;
    forwardDynamics.computeC(mb, gravityMbc);
    endpoint.gravityMap_.col(column) = selectVector(forwardDynamics.C());
  }

  for(const auto & [contactId, graphContact] : observer_.getActiveContacts())
  {
    const auto maintained = maintainedContacts_.find(static_cast<unsigned>(contactId));
    if(maintained == maintainedContacts_.end()) { continue; }
    const auto & contact = *maintained->second;
    const auto & surface = dynamicsRobot.surface(contact.surfaceName());
    const size_t contactBody = dynamicsRobot.bodyIndexByName(surface.bodyName());
    Eigen::Matrix<double, 6, 6> bodyWrenchMap;
    const Eigen::Matrix3d R_fb_contact = contact.fbContactKine_.orientation.toMatrix3();
    const Eigen::Vector3d p_fb_contact = contact.fbContactKine_.position();
    for(Eigen::Index column = 0; column < 6; ++column)
    {
      Eigen::Matrix<double, 6, 1> local = Eigen::Matrix<double, 6, 1>::Zero();
      local(column) = 1.0;
      const Eigen::Vector3d forceFb = R_fb_contact * local.head<3>();
      const Eigen::Vector3d momentFb = R_fb_contact * local.tail<3>() + p_fb_contact.cross(forceFb);
      bodyWrenchMap.col(column) = mbc.bodyPosW[contactBody].dualMul(sva::ForceVecd(momentFb, forceFb)).vector();
    }

    std::vector<Eigen::Matrix<double, 6, 6>> bodyForce(static_cast<size_t>(mb.nrBodies()),
                                                       Eigen::Matrix<double, 6, 6>::Zero());
    bodyForce[contactBody] = bodyWrenchMap;
    Eigen::MatrixXd generalizedMap = Eigen::MatrixXd::Zero(fullDimension, 6);
    for(size_t reverse = static_cast<size_t>(mb.nrBodies()); reverse-- > 0;)
    {
      const int joint = bodyJoint[reverse];
      if(joint < 0) { continue; }
      const int dof = mb.joint(joint).dof();
      generalizedMap.middleRows(jointOffsets[static_cast<size_t>(joint)], dof) =
          mbc.motionSubspace[static_cast<size_t>(joint)].transpose() * bodyForce[reverse];
      const int parent = predecessors[static_cast<size_t>(joint)];
      if(parent >= 0)
      {
        Eigen::Matrix<double, 6, 6> motion;
        for(Eigen::Index column = 0; column < 6; ++column)
        {
          Eigen::Matrix<double, 6, 1> unit = Eigen::Matrix<double, 6, 1>::Zero();
          unit(column) = 1.0;
          motion.col(column) = (mbc.parentToSon[static_cast<size_t>(joint)] * sva::MotionVecd(unit)).vector();
        }
        bodyForce[static_cast<size_t>(parent)].noalias() += motion.transpose() * bodyForce[reverse];
      }
    }
    const Eigen::MatrixXd selectedMap = selectRows(generalizedMap);
    endpoint.contactForceMap_[contactId] = selectedMap.leftCols<3>();
    endpoint.contactMomentMap_[contactId] = selectedMap.rightCols<3>();
  }

  return endpoint;
}

ko_fg::ReducedJointMomentumEndpoint MCKineticsObserverFG::makeReducedJointMomentumEndpoint(
    mc_rbdyn::Robot & dynamicsRobot,
    const Eigen::VectorXd & actuatorTorque)
{
  const auto full = makeMomentumResidualEndpoint(dynamicsRobot);
  const auto & mb = dynamicsRobot.mb();
  rbd::MultiBodyConfig mbc = dynamicsRobot.mbc();
  const Eigen::Index dimension = actuatorTorque.size();
  const Eigen::Index fullDimension = mb.nrDof();

  if(full.momentumOffset_.size() != dimension + 6 || fullDimension != dimension + 6)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "[{}]: Reduced joint momentum requires refJointOrder to contain every non-root DoF.", name());
  }

  std::vector<Eigen::Index> jointOffsets(static_cast<size_t>(mb.nrJoints()));
  Eigen::Index offset = 0;
  for(size_t joint = 0; joint < static_cast<size_t>(mb.nrJoints()); ++joint)
  {
    jointOffsets[joint] = offset;
    offset += mb.joint(static_cast<int>(joint)).dof();
  }
  std::vector<Eigen::Index> selectedRows;
  selectedRows.reserve(static_cast<size_t>(dimension + 6));
  for(Eigen::Index row = 0; row < 6; ++row) { selectedRows.push_back(row); }
  for(const auto & jointName : dynamicsRobot.refJointOrder())
  {
    const int jointIndex = mb.jointIndexByName(jointName);
    for(int dof = 0; dof < mb.joint(jointIndex).dof(); ++dof)
    {
      selectedRows.push_back(jointOffsets[static_cast<size_t>(jointIndex)] + dof);
    }
  }
  const auto selectSquare = [&selectedRows](const Eigen::MatrixXd & matrix)
  {
    const Eigen::Index selectedDimension = static_cast<Eigen::Index>(selectedRows.size());
    Eigen::MatrixXd selected(selectedDimension, selectedDimension);
    for(Eigen::Index row = 0; row < selectedDimension; ++row)
    {
      for(Eigen::Index column = 0; column < selectedDimension; ++column)
      {
        selected(row, column) =
            matrix(selectedRows[static_cast<size_t>(row)], selectedRows[static_cast<size_t>(column)]);
      }
    }
    return selected;
  };

  rbd::ForwardDynamics forwardDynamics(mb);
  forwardDynamics.computeH(mb, mbc);
  const Eigen::MatrixXd inertia = selectSquare(forwardDynamics.H());
  const Eigen::MatrixXd Hbb = inertia.topLeftCorner(6, 6);
  const Eigen::MatrixXd Hjb = inertia.bottomLeftCorner(dimension, 6);
  const Eigen::LDLT<Eigen::MatrixXd> baseSolve(Hbb);
  if(baseSolve.info() != Eigen::Success)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}]: Floating-base inertia factorization failed.", name());
  }
  const Eigen::MatrixXd HbbInverse = baseSolve.solve(Eigen::MatrixXd::Identity(6, 6));
  Eigen::MatrixXd reduction = Eigen::MatrixXd::Zero(dimension, dimension + 6);
  reduction.leftCols(6) = -Hjb * HbbInverse;
  reduction.rightCols(dimension).setIdentity();

  // Since h_J^* = S(q)h, the dynamics contains both S hdot and Sdot h.
  // RBDyn's Christoffel matrix gives Hdot = C + C^T analytically.
  rbd::MultiBodyConfig shapeMbc = mbc;
  Eigen::VectorXd shapeVelocity = rbd::dofToVector(mb, shapeMbc.alpha);
  shapeVelocity.head(6).setZero();
  shapeMbc.alpha = rbd::vectorToDof(mb, shapeVelocity);
  rbd::forwardVelocity(mb, shapeMbc);
  rbd::Coriolis coriolis(mb);
  const Eigen::MatrixXd coriolisMatrix = selectSquare(coriolis.coriolis(mb, shapeMbc));
  const Eigen::MatrixXd inertiaDot = coriolisMatrix + coriolisMatrix.transpose();
  const Eigen::MatrixXd HbbDot = inertiaDot.topLeftCorner(6, 6);
  const Eigen::MatrixXd HjbDot = inertiaDot.bottomLeftCorner(dimension, 6);
  Eigen::MatrixXd reductionDot = Eigen::MatrixXd::Zero(dimension, dimension + 6);
  reductionDot.leftCols(6) = -HjbDot * HbbInverse + Hjb * HbbInverse * HbbDot * HbbInverse;

  ko_fg::ReducedJointMomentumEndpoint endpoint;
  endpoint.actuatorTorque_ = actuatorTorque;
  endpoint.measuredMomentum_ = reduction * full.momentumOffset_;
  endpoint.biasOffset_ =
      reduction * (full.coriolisOffset_ - full.gravityMap_ * full.gravityWorld_) + reductionDot * full.momentumOffset_;
  endpoint.biasAngVelLinear_ =
      reduction * full.coriolisLinear_.rightCols(3) + reductionDot * full.momentumJacobian_.rightCols(3);
  for(size_t axis = 0; axis < 3; ++axis)
  {
    endpoint.biasAngVelQuadratic_[axis] = reduction * full.coriolisQuadratic_[axis + 3].rightCols(3);
  }
  for(const auto & [contactId, map] : full.contactForceMap_) { endpoint.contactForceMap_[contactId] = reduction * map; }
  for(const auto & [contactId, map] : full.contactMomentMap_)
  {
    endpoint.contactMomentMap_[contactId] = reduction * map;
  }
  return endpoint;
}

ko_fg::ReducedJointMomentumMeasurement MCKineticsObserverFG::makeReducedJointMomentumMeasurement(
    const mc_rbdyn::Robot & measRobot,
    mc_rbdyn::Robot & dynamicsRobot)
{
  ko_fg::ReducedJointMomentumMeasurement measurement;
  const Eigen::VectorXd measuredTorque = measuredJointTorqueVector(measRobot);
  measurement.current_ = makeReducedJointMomentumEndpoint(dynamicsRobot, measuredTorque);
  measurement.previous_ = previousMomentumEndpoint_.value();
  previousMomentumEndpoint_ = measurement.current_;

  previousMeasuredJointTorques_ = measuredTorque;
  measurement.dt_ = dt_;
  measurement.measurementNoise_ = ko_fg::makeNoise(measuredTorque.size(), jointMomentumNoise_);
  const double torqueSigma = std::hypot(jointMomentumModelNoise_, jointTorqueNoise_);
  measurement.dynamicsNoise_ = ko_fg::makeNoise(measuredTorque.size(), dt_ * torqueSigma / std::sqrt(2.0));

  return measurement;
}

void MCKineticsObserverFG::updateJointTorqueMeasurement(const mc_rbdyn::Robot & measRobot,
                                                        mc_rbdyn::Robot & dynamicsRobot)
{
  const Eigen::VectorXd measuredTorque = measuredJointTorqueVector(measRobot);
  if(measuredTorque.size() == 0)
  {
    observer_.clearJointTorqueMeasurement();
    previousJointVelocity_.reset();
    pendingJointAccelerationMeasurement_.reset();
    pendingJointAccelerationContacts_.clear();
    pendingJointAccelerationTime_ = 0;
    return;
  }
  measuredJointTorques_ = measuredTorque;

  // What is actually fed to the joint-torque factors. The measurement can silently fall back to
  // the COMMANDED torque when the robot carries no torque sensor, in which case it may be the very
  // same vector the dynamics model is built from -- a measurement compared against itself, which
  // carries no information at all. Gated by KO_FG_DUMP_TORQUE.
  if(std::getenv("KO_FG_DUMP_TORQUE"))
  {
    static size_t n = 0;
    if(n++ % 400 == 0)
    {
      const auto & rawSensor = measRobot.jointTorques();
      const double diff = (inputJointTorques_.size() == measuredTorque.size())
                              ? (measuredTorque - inputJointTorques_).norm()
                              : -1.0;
      mc_rtc::log::info("[KOFGTAU] t={:.2f} capteur={} dim={} |mesure|={:.4f} |entree|={:.4f} "
                        "|mesure-entree|={:.6f}",
                        k_ * dt_, rawSensor.empty() ? "VIDE (repli sur la commande)" : "present",
                        measuredTorque.size(), measuredTorque.norm(), inputJointTorques_.norm(), diff);
    }
  }

  // Three mutually exclusive formulations of the same physics live in the core -- each setter
  // resets the other two. Only `fullDynamics` was ever wired to a configuration switch; the
  // momentum one was written on both sides and never called. It integrates instead of
  // differentiating, so it does not need the joint accelerations, which are obtained here by a
  // finite difference of the joint velocities and carry their own error.
  if(jointTorqueFormulation_ == JointTorqueFormulation::ReducedMomentum)
  {
    const auto endpoint = makeReducedJointMomentumEndpoint(dynamicsRobot, measuredTorque);
    if(!previousMomentumEndpoint_)
    {
      // No previous endpoint on the first usable iteration: nothing to difference yet.
      previousMomentumEndpoint_ = endpoint;
      previousMeasuredJointTorques_ = measuredTorque;
      observer_.clearJointTorqueMeasurement();
      return;
    }
    ko_fg::ReducedJointMomentumMeasurement measurement;
    measurement.current_ = endpoint;
    measurement.previous_ = *previousMomentumEndpoint_;
    previousMomentumEndpoint_ = endpoint;
    previousMeasuredJointTorques_ = measuredTorque;
    measurement.dt_ = dt_;
    measurement.measurementNoise_ = ko_fg::makeNoise(measuredTorque.size(), jointMomentumNoise_);
    const double torqueSigma = std::hypot(jointMomentumModelNoise_, jointTorqueNoise_);
    measurement.dynamicsNoise_ = ko_fg::makeNoise(measuredTorque.size(), dt_ * torqueSigma / std::sqrt(2.0));
    observer_.updateReducedJointMomentumMeasurement(measurement);
    pendingJointAccelerationMeasurement_.reset();
    pendingJointAccelerationContacts_.clear();
    pendingJointAccelerationTime_ = 0;
    return;
  }

  const Eigen::VectorXd jointVelocity = actuatedJointVelocityVector(dynamicsRobot);
  Eigen::VectorXd jointAccelerationPrediction = Eigen::VectorXd::Zero(jointVelocity.size());
  if(previousJointVelocity_)
  {
    jointAccelerationPrediction = (jointVelocity - *previousJointVelocity_) / dt_;
  }
  previousJointVelocity_ = jointVelocity;
  jointAccelerationFiniteDifference_ = jointAccelerationPrediction;

  ko_fg::JointTorqueMeasurement measurement;
  measurement.measuredTorque_ = measuredTorque;
  measurement.knownCentroidWrench_ = inputWrench_;
  measurement.exactJointVelocity_ = jointVelocity;
  measurement.noise_ = ko_fg::makeNoise(
      measuredTorque.size(), std::hypot(jointTorqueNoise_, jointMomentumModelNoise_));
  Eigen::VectorXd dynamicsSigmas =
      Eigen::VectorXd::Constant(measuredTorque.size() + 6, jointMomentumModelNoise_);
  dynamicsSigmas.tail(measuredTorque.size()).setConstant(
      std::hypot(jointTorqueNoise_, jointMomentumModelNoise_));
  measurement.fullDynamicsNoise_ = ko_fg::makeNoise(dynamicsSigmas);
  measurement.jointVelocityIntegrationNoise_ =
      ko_fg::makeNoise(measuredTorque.size(), dt_ * jointAccelerationFiniteDifferenceNoise_);
  measurement.model_ = makeJointTorqueModel(dynamicsRobot);
  observer_.updateJointAccelerationTorqueMeasurement(measurement);

  pendingJointAccelerationMeasurement_ = measurement;
  pendingJointAccelerationContacts_.clear();
  for(const auto & [contactId, contact] : observer_.getActiveContacts())
  {
    (void)contact;
    pendingJointAccelerationContacts_.push_back(contactId);
  }
  std::sort(pendingJointAccelerationContacts_.begin(), pendingJointAccelerationContacts_.end());
  pendingJointAccelerationTime_ = k_ + 1;
}

void MCKineticsObserverFG::updateEstimatedJointTorqueResidual()
{
  if(!pendingJointAccelerationMeasurement_ || pendingJointAccelerationTime_ == 0) { return; }

  const size_t current = pendingJointAccelerationTime_;
  gtsam::KeyVector keys = {ko_fg::X(current), ko_fg::V(current), ko_fg::W(current),
                           ko_fg::L(current), ko_fg::A(current), ko_fg::J(current)};
  for(const size_t contactId : pendingJointAccelerationContacts_)
  {
    keys.push_back(ko_fg::F(current, static_cast<uint32_t>(contactId)));
    keys.push_back(ko_fg::T(current, static_cast<uint32_t>(contactId)));
  }

  const gtsam::Values estimate = observer_.getSmoother().calculateEstimate();
  for(const gtsam::Key key : keys)
  {
    // Bootstrap buffering can leave the newest measurement outside the
    // smoother until the first batch is submitted.
    if(!estimate.exists(key)) { return; }
  }

  const ko_fg::measurementFactors::JointAccelerationTorqueFactor factor(
      keys, *pendingJointAccelerationMeasurement_, pendingJointAccelerationContacts_);
  estimatedJointTorqueResidual_ = -factor.unwhitenedError(estimate);
}

const so::kine::Kinematics MCKineticsObserverFG::getContactWorldKinematics(const mc_control::MCController & ctl,
                                                                           KoContactWithSensor & contact,
                                                                           const mc_rbdyn::Robot & currentRobot,
                                                                           bool withVel)
{
  so::kine::Kinematics worldContactKine;
  so::kine::Kinematics worldFbKine;
  if(withVel) { worldFbKine = conversions::kinematics::fromSva(currentRobot.posW(), currentRobot.velW(), true); }
  else { worldFbKine = conversions::kinematics::fromSva(currentRobot.posW(), so::kine::Kinematics::Flags::pose); }

  if(contact.fbContactKine_.position.isSet())
  {
    worldContactKine = worldFbKine * contact.fbContactKine_;
    return worldContactKine;
  }

  if(contactsDetector_.getContactsDetection() == KoContactsDetector::ContactsDetection::Sensors)
  {
    worldContactKine = getFsWorldKinematics(ctl, currentRobot, contact.fsName());
    contact.fbContactKine_ = worldFbKine.getInverse() * worldContactKine;
    return worldContactKine;
  }
  else
  {
    const mc_rbdyn::Surface & contactSurface = currentRobot.surface(contact.surfaceName());
    const sva::PTransformd & bodyContactPose = contactSurface.X_b_s();
    unsigned bodyIndex = currentRobot.bodyIndexByName(contactSurface.bodyName());

    so::kine::Kinematics bodyContactKine;
    so::kine::Kinematics worldBodyKine;

    if(withVel)
    {
      bodyContactKine = conversions::kinematics::fromSva(bodyContactPose, so::kine::Kinematics::Flags::vel);
      worldBodyKine = conversions::kinematics::fromSva(currentRobot.mbc().bodyPosW[bodyIndex],
                                                       currentRobot.mbc().bodyVelW[bodyIndex], true);
    }
    else
    {
      bodyContactKine = conversions::kinematics::fromSva(bodyContactPose, so::kine::Kinematics::Flags::pose);
      worldBodyKine =
          conversions::kinematics::fromSva(currentRobot.mbc().bodyPosW[bodyIndex], so::kine::Kinematics::Flags::pose);
    }

    worldContactKine = worldBodyKine * bodyContactKine;
    contact.fbContactKine_ = worldFbKine.getInverse() * worldContactKine;
  }

  return worldContactKine;
}

const so::kine::Kinematics MCKineticsObserverFG::getCtlContactWorldKinematics(const mc_control::MCController & ctl,
                                                                              KoContactWithSensor & contact,
                                                                              bool withVel)
{
  return getContactWorldKinematics(ctl, contact, ctl.robot(robot_), withVel);
}

const so::kine::Kinematics MCKineticsObserverFG::getFsWorldKinematics(const mc_control::MCController & ctl,
                                                                      const mc_rbdyn::Robot & currentRobot,
                                                                      const std::string & fsName)
{
  const mc_rbdyn::ForceSensor & fs = ctl.robot(robot_).forceSensor(fsName);

  const so::kine::Kinematics worldFbKine =
      conversions::kinematics::fromSva(currentRobot.posW(), currentRobot.velW(), true);

  const sva::PTransformd bodyFsPose = fs.X_fsactual_parent().inv();
  unsigned bodyIndex = currentRobot.bodyIndexByName(fs.parentBody());

  so::kine::Kinematics bodyFsKine = conversions::kinematics::fromSva(bodyFsPose, so::kine::Kinematics::Flags::vel);

  so::kine::Kinematics worldBodyKine = conversions::kinematics::fromSva(currentRobot.mbc().bodyPosW[bodyIndex],
                                                                        currentRobot.mbc().bodyVelW[bodyIndex], true);

  (void)worldFbKine;
  return worldBodyKine * bodyFsKine;
}

const so::kine::Kinematics MCKineticsObserverFG::getContactFsKinematics(const mc_control::MCController & ctl,
                                                                        KoContactWithSensor & contact,
                                                                        const mc_rbdyn::Robot & currentRobot)
{
  if(!contact.contactSensorKine_.position.isSet())
  {
    so::kine::Kinematics worldFsKine = getFsWorldKinematics(ctl, currentRobot, contact.fsName());
    so::kine::Kinematics worldContactKine = getContactWorldKinematics(ctl, contact, currentRobot, true);

    contact.contactSensorKine_ = worldContactKine.getInverse() * worldFsKine;
  }

  return contact.contactSensorKine_;
}

void MCKineticsObserverFG::updateContactForceMeasurement(KoContactWithSensor & contact,
                                                         const sva::ForceVecd & measuredWrench)
{
  // The calibration corrects the direction of the measured force in the contact frame, so it has to
  // be applied before the moment is transported: the lever-arm term below is the moment of *this*
  // force about the contact origin. Rotating the force afterwards leaves that term carrying the
  // uncalibrated force, i.e. a torque that no longer matches the force it is reported with.
  so::Matrix3 calibrationRotation = so::Matrix3::Identity();
  const auto calibration = wrenchCalibration_.find(contact.surfaceName());
  if(calibration != wrenchCalibration_.end() && calibration->second.norm() > 0.0)
  {
    calibrationRotation = so::Matrix3(Eigen::AngleAxisd(calibration->second.norm(), calibration->second.normalized()));
  }

  if(contactsDetector_.getContactsDetection() == KoContactsDetector::ContactsDetection::Sensors)
  {
    // Sensor-based contacts use the force-sensor frame as their contact frame.
    contact.contactWrenchVector_.segment<3>(0) =
        calibrationRotation * measuredWrench.force(); // retrieving the force measurement
    contact.contactWrenchVector_.segment<3>(3) = measuredWrench.moment(); // retrieving the torque measurement
  }
  else
  { // expressing the force measurement in the frame of the contact
    contact.contactWrenchVector_.segment<3>(0) =
        calibrationRotation * (contact.contactSensorKine_.orientation * measuredWrench.force());

    // expressing the torque measurement in the frame of the surface
    contact.contactWrenchVector_.segment<3>(3) =
        contact.contactSensorKine_.orientation * measuredWrench.moment()
        + contact.contactSensorKine_.position().cross(contact.contactWrenchVector_.segment<3>(0));
  }
}

void MCKineticsObserverFG::updateContactWrenchMeasNoise(const KoContactWithSensor & contact)
{
  so::Matrix3 forceCov = contactMeasNoises_.at(0).array().square().matrix().asDiagonal();
  so::Matrix3 momentCov = contactMeasNoises_.at(1).array().square().matrix().asDiagonal();

  // Sensor-based contacts use the force-sensor frame as their contact frame, so there is nothing
  // to transport.
  if(contactsDetector_.getContactsDetection() != KoContactsDetector::ContactsDetection::Sensors)
  {
    // The sensor is not at the contact origin. Transporting its covariance there with
    // T = [[R, 0], [skew(p) R, R]] gives cov = T Sigma T'; the force and moment residuals are two
    // separate 3-dimensional factors, so the diagonal blocks are kept and the cross-correlation
    // the transport also creates is dropped. The term that matters survives: the lever arm feeds
    // the FORCE covariance into the moment one, which on HRP-5P (0.105 m) dominates the sensor's
    // own moment noise.
    const so::Matrix3 & sensorContactOri = contact.contactSensorKine_.orientation.toMatrix3();
    const so::Matrix3 leverArm = so::kine::skewSymmetric(contact.contactSensorKine_.position());

    const so::Matrix3 rotatedForceCov = sensorContactOri * forceCov * sensorContactOri.transpose();
    momentCov = sensorContactOri * momentCov * sensorContactOri.transpose()
                + leverArm * rotatedForceCov * leverArm.transpose();
    forceCov = rotatedForceCov;
  }

  observer_.setContactWrenchMeasNoise(contact.id(), forceCov, momentCov);
}

so::kine::Kinematics MCKineticsObserverFG::getOdometryWorldContactRest(const mc_control::MCController &,
                                                                       KoContactWithSensor & contact,
                                                                       const so::kine::Kinematics & worldContactKine)
{
  so::kine::Kinematics worldRestPose;

  if(!contact.sensorEnabled_)
  {
    mc_rtc::log::info("The sensor is disabled but is required for the odometry. It will be used for the odometry "
                      "but not in the correction made by the Kinetics Observer.");
  }
  const so::Vector3 & contactForceMeas = contact.contactWrenchVector_.segment<3>(0); // retrieving the force measurement
  const so::Vector3 & contactTorqueMeas = contact.contactWrenchVector_.segment<3>(3);

  // The measured wrench and the visco-elastic gains live in the CONTACT frame, so the deflection
  // is computed there and only then rotated to the world. Pushing the damping term through the
  // rotation instead (R * K^-1 * (F + R' * Kd * v)) is only the same thing when Kd commutes with
  // R, i.e. never outside the isotropic case. Same formula as
  // KineticsObserver::getOdometryWorldContactRest_ in state-observation, which is the reference.
  const so::Matrix3 worldContactOri = worldContactKine.orientation.toMatrix3();

  // we get the reference position of the contact by removing the contribution of the visco-elastic model
  worldRestPose.position =
      worldContactKine.position()
      + worldContactOri * contactFlexibilities_.at(0).inverse()
            * (contactForceMeas + contactFlexibilities_.at(1) * worldContactOri.transpose() * worldContactKine.linVel());

  /* We get the reference orientation of the contact by removing the contribution of the visco-elastic model */
  // difference between the reference orientation and the real one, obtained from the visco-elastic model
  so::Matrix3 flexRotMatrix = so::Matrix3::Identity();

  // The angular stiffness can legitimately be zero (no-angular-flexibility ablation). Inverting a
  // zero matrix yields NaN on every entry, and no comparison with NaN is ever true, so the norm
  // test below would fall through and build a NaN rotation. With no angular stiffness and no
  // angular damping the contact exerts no reaction torque, hence no angular deflection to remove:
  // the rest orientation IS the contact orientation.
  if(!contactFlexibilities_.at(2).isZero())
  {
    so::Vector3 flexRotDiff =
        -2 * contactFlexibilities_.at(2).inverse()
        * (contactTorqueMeas
           + contactFlexibilities_.at(3) * worldContactOri.transpose() * worldContactKine.angVel());

    if(flexRotDiff.norm() > so::cst::epsilonAngle)
    {
      so::Vector3 flexRotAxis = flexRotDiff / flexRotDiff.norm();
      double diffNorm = std::min(1.0, flexRotDiff.norm() / 2.0);
      double flexRotAngle = std::asin(diffNorm);

      Eigen::AngleAxisd flexRotAngleAxis(flexRotAngle, flexRotAxis);
      flexRotMatrix = so::kine::Orientation(flexRotAngleAxis).toMatrix3();
    }
  }

  worldRestPose.orientation = so::Matrix3(worldContactOri * flexRotMatrix.transpose());

  if(odometryType_ == so::odometry::OdometryType::Flat) // if true, the position odometry is made only
                                                        // along the x and y axis, the position along z is
                                                        // assumed to be the one of the control robot
  {
    worldRestPose.position()(2) = 0.0;
  }
  return worldRestPose;
}

void MCKineticsObserverFG::setNewContact(const mc_control::MCController & ctl,
                                         KoContactWithSensor & contact,
                                         const std::array<ko_fg::NoiseSigmas3, 4> & initNoises,
                                         mc_rtc::Logger & logger)
{
  /*
  Uses the inputRobot, a virtual robot corresponding to the real robot whose floating base's frame is superimposed with
  the world frame. Getting kinematics associated to the inputRobot inside the world frame is the same as getting the
  same kinematics of the real robot inside the frame of its floating base, which is needed for the inputs of the
  Kinetics Observer. This allows to use the basic mc_rtc functions directly giving kinematics in the world frame and not
  do the conversion: initial frame -> world + world -> floating base as the latter is zero.
  */

  /*
  Contact init noises:
  - init pos noise
  - init ori noise
  - init force noise
  - init moment noise
  */

  auto & inputRobot = my_robots_->robot("inputRobot");

  const auto & robot = ctl.robot(robot_);

  if(contactsDetector_.getContactsDetection() == KoContactsDetector::ContactsDetection::Sensors)
  {
    contact.fsName(contact.surfaceName());
  }
  else
  {
    contact.fsName(robot.indirectSurfaceForceSensor(contact.surfaceName()).name());
  }

  const mc_rbdyn::ForceSensor & fs = robot.forceSensor(contact.fsName_);
  sva::ForceVecd measuredWrench = fs.wrenchWithoutGravity(ctl.realRobot(robot_));
  contact.fbContactKine_.reset();
  contact.contactSensorKine_.reset();
  getContactFsKinematics(ctl, contact, inputRobot);
  updateContactForceMeasurement(contact, measuredWrench);

  // if(withDebugLogs_)
  // {
  //   mc_rtc::log::info("[{}] new contact {} sensorEnabled={} forceNorm={} momentNorm={} fbPos={} {} {}", name(),
  //                     contact.id(), contact.sensorEnabled_, contact.contactWrenchVector_.segment<3>(0).norm(),
  //                     contact.contactWrenchVector_.segment<3>(3).norm(), contact.fbContactKine_.position().x(),
  //                     contact.fbContactKine_.position().y(), contact.fbContactKine_.position().z());
  // }

  so::kine::Kinematics worldContactKine = est_worldFbKine_ * contact.fbContactKine_;
  Pose3_RI worldContactPose(gtsam::Rot3(worldContactKine.orientation.toMatrix3()), worldContactKine.position());
  updateContactWrenchMeasNoise(contact);
  observer_.addContact(contact.id(), worldContactPose, worldContactKine.linVel(), worldContactKine.angVel(),
                       contact.contactWrenchVector_, initNoises, k_);

  stateObservation::kine::Kinematics centroidContactKine = centroidFbKine_ * contact.fbContactKine_;

  if(contact.sensorEnabled_) // the force sensor attached to the contact is used in
                             // the correction by the Kinetics Observer.
  {
    // if(withDebugLogs_) { mc_rtc::log::info("[{}] new contact {} -> updateContact(with meas)", name(), contact.id()); }
    observer_.updateContact(contact.id(), contact.contactWrenchVector_.segment(0, 3),
                            contact.contactWrenchVector_.segment(3, 3),
                            gtsam::Pose3(gtsam::Rot3(contact.fbContactKine_.orientation.toMatrix3()),
                                         gtsam::Point3(centroidContactKine.position())),
                            centroidContactKine.linVel(), centroidContactKine.angVel());
  }
  else
  {
    if(withDebugLogs_) { mc_rtc::log::info("[{}] new contact {} -> updateContactNoMeas", name(), contact.id()); }
    observer_.updateContactNoMeas(contact.id(),
                                  gtsam::Pose3(gtsam::Rot3(contact.fbContactKine_.orientation.toMatrix3()),
                                               gtsam::Point3(centroidContactKine.position())),
                                  centroidContactKine.linVel(), centroidContactKine.angVel());
  }

  if(withDebugLogs_)
  {
    addContactLogEntries(ctl, logger, contact);
    if(contact.sensorEnabled_) { addContactMeasurementsLogEntries(logger, contact); }
  }

  maintainedContacts_.insert({contact.id(), &contact});
}

void MCKineticsObserverFG::updateContact(const mc_control::MCController & ctl, KoContactWithSensor & contact)
{
  /*
  Uses the inputRobot, a virtual robot corresponding to the real robot whose floating base's frame is superimposed with
  the world frame. Getting kinematics associated to the inputRobot inside the world frame is the same as getting the
  same kinematics of the real robot inside the frame of its floating base, which is needed for the inputs of the
  Kinetics Observer. This allows to use the basic mc_rtc functions directly giving kinematics in the world frame and not
  do the conversion: initial frame -> world + world -> floating base as the latter is zero.
  */
  auto & inputRobot = my_robots_->robot("inputRobot");

  const auto & robot = ctl.robot(robot_);

  const mc_rbdyn::ForceSensor & fs = robot.forceSensor(contact.fsName_);
  sva::ForceVecd measuredWrench = fs.wrenchWithoutGravity(ctl.realRobot(robot_));
  contact.fbContactKine_.reset();
  contact.contactSensorKine_.reset();
  getContactFsKinematics(ctl, contact, inputRobot);
  updateContactForceMeasurement(contact, measuredWrench);
  updateContactWrenchMeasNoise(contact);
  contact.centroidContactKine_ = centroidFbKine_ * contact.fbContactKine_;

  // if(withDebugLogs_)
  // {
  //   mc_rtc::log::info("[{}] update contact {} sensorEnabled={} forceNorm={} momentNorm={}", name(), contact.id(),
  //                     contact.sensorEnabled_, contact.contactWrenchVector_.segment<3>(0).norm(),
  //                     contact.contactWrenchVector_.segment<3>(3).norm());
  // }

  gtsam::Pose3 centroidContactPose(gtsam::Rot3(contact.centroidContactKine_.orientation.toMatrix3()),
                                   contact.centroidContactKine_.position());

  if(contact.sensorEnabled_) // the force sensor attached to the contact is used in
                             // the correction by the Kinetics Observer.
  {
    // if(withDebugLogs_) { mc_rtc::log::info("[{}] contact {} -> updateContact(with meas)", name(), contact.id()); }
    observer_.updateContact(contact.id(), contact.contactWrenchVector_.segment(0, 3),
                            contact.contactWrenchVector_.segment(3, 3), centroidContactPose,
                            contact.centroidContactKine_.linVel(), contact.centroidContactKine_.angVel());
  }
  else
  {
    // if(withDebugLogs_) { mc_rtc::log::info("[{}] contact {} -> updateContactNoMeas", name(), contact.id()); }
    observer_.updateContactNoMeas(contact.id(), centroidContactPose, contact.centroidContactKine_.linVel(),
                                  contact.centroidContactKine_.angVel());
  }
}

void MCKineticsObserverFG::updateContacts(const mc_control::MCController & ctl, mc_rtc::Logger & logger)
{
  const std::array<ko_fg::NoiseSigmas3, 4> * initNoise;

  if(observer_.getActiveContacts().empty()) // The initial noise on the pose of the contact depending on
                                            // whether another contact is already set or not
  {
    initNoise = &contactInitNoises_first_;
  }
  else { initNoise = &contactInitNoises_; }

  auto onNewContact = [this, &ctl, &logger, &initNoise](KoContactWithSensor & newContact)
  {
    if(!contactsIgnoredForEstimation_.count(newContact.surfaceName()))
    {
      setNewContact(ctl, newContact, *initNoise, logger);
    }
  };
  auto onMaintainedContact = [this, &ctl](KoContactWithSensor & maintainedContact)
  {
    if(!contactsIgnoredForEstimation_.count(maintainedContact.surfaceName()))
    {
      updateContact(ctl, maintainedContact);
    }
  };
  auto onRemovedContact = [this, &logger](KoContactWithSensor & removedContact)
  {
    if(contactsIgnoredForEstimation_.count(removedContact.surfaceName())) { return; }

    observer_.removeContact(removedContact.id());

    if(withDebugLogs_)
    {
      removeContactLogEntries(logger, removedContact);
      removeContactMeasurementsLogEntries(logger, removedContact);
    }
    maintainedContacts_.erase(removedContact.id());
  };

  // Action to execute once when a contact is first added to the manager.
  auto onAddedContact = [this, &ctl, &logger](KoContactWithSensor & addedContact)
  {
    if(contactsIgnoredForEstimation_.count(addedContact.surfaceName())) { return; }

    observer_.addContactToList(addedContact.id(), contactFlexibilities_, contactProcessNoises_, contactMeasNoises_);
    addContactToGui(ctl, addedContact, logger);
  };

  std::unordered_set<std::string> & contactList = contactsDetector_.updateContacts(ctl, robot_);
  contactsManager_.updateContacts(contactList, onNewContact, onMaintainedContact, onRemovedContact, onAddedContact);
}

///////////////////////////////////////////////////////////////////////
/// -------------------------------Logs--------------------------------
///////////////////////////////////////////////////////////////////////

void MCKineticsObserverFG::addToLogger(const mc_control::MCController & ctl,
                                       mc_rtc::Logger & logger,
                                       const std::string & category)
{
  category_ = category;
  jointTorqueNames_ = actuatedJointTorqueNames(ctl.realRobot(robot_));

  logger.addLogEntry(category_ + "_fb_posW", [this]() -> sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category_ + "_fb_velW", [this]() -> sva::MotionVecd & { return v_fb_0_; });
  logger.addLogEntry(category_ + "_fb_accW", [this]() -> sva::MotionVecd & { return a_fb_0_; });
  logger.addLogEntry(category_ + "_fb_yaw",
                     [this]() -> double { return -so::kine::rotationMatrixToYawAxisAgnostic(X_0_fb_.rotation()); });

  /* Plots of the updated state */
  conversions::kinematics::addToLogger(logger, est_worldCentroidKine_, category_ + "_est_worldCentroidKine");
  logger.addLogEntry(category_ + "_MEKF_estimatedState_position",
                     [this]() -> Eigen::Vector3d { return observer_.getCurrentState().kine_.pose().translation(); });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_ori",
                     [this]() -> Eigen::Quaterniond
                     {
                       so::kine::Orientation ori(observer_.getCurrentState().kine_.pose().rotation().matrix());
                       return ori.inverse().toQuaternion();
                     });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_linVel",
                     [this]() -> Eigen::Vector3d { return observer_.getCurrentState().kine_.linVel(); });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_angVel",
                     [this]() -> Eigen::Vector3d { return observer_.getCurrentState().kine_.angVel(); });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_linAcc",
                     [this]() -> Eigen::Vector3d { return observer_.getCurrentState().kine_.linAcc(); });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_angAcc",
                     [this]() -> Eigen::Vector3d { return observer_.getCurrentState().kine_.angAcc(); });

  for(auto & imu : listIMUs_)
  {
    logger.addLogEntry(category_ + "_MEKF_estimatedState_gyroBias_" + imu.name(), [this, &imu]() -> Eigen::Vector3d
                       { return observer_.getImus().at(imu.id()).currentBiasEstimate_; });
  }
  logger.addLogEntry(category_ + "_MEKF_estimatedState_extForceCentr",
                     [this]() -> Eigen::Vector3d { return observer_.getCurrentState().disturbForce_; });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_extTorqueCentr",
                     [this]() -> Eigen::Vector3d { return observer_.getCurrentState().disturbMoment_; });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_unbiasedExtForce",
                     [this]() -> Eigen::Vector3d { return getUnbiasedEstimatedDisturbanceWrench().force(); });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_unbiasedExtMoment",
                     [this]() -> Eigen::Vector3d { return getUnbiasedEstimatedDisturbanceWrench().moment(); });

  for(const auto & [surface, wrench] : ignoredWrenchesInCentroid_)
  {
    (void)wrench;
    logger.addLogEntry(category_ + "_MEKF_measurements_ignoredWrench_Centroid_" + surface + "_force",
                       [this, surface]() -> Eigen::Vector3d
                       { return ignoredWrenchesInCentroid_.at(surface).segment<3>(0); });
    logger.addLogEntry(category_ + "_MEKF_measurements_ignoredWrench_Centroid_" + surface + "_moment",
                       [this, surface]() -> Eigen::Vector3d
                       { return ignoredWrenchesInCentroid_.at(surface).segment<3>(3); });
  }

  for(size_t i = 0; i < jointTorqueNames_.size(); ++i)
  {
    const std::string & jointName = jointTorqueNames_[i];
    logger.addLogEntry(category_ + "_MEKF_measurements_jointTorque_measured_" + jointName,
                       [this, i]() -> double {
                         return i < static_cast<size_t>(measuredJointTorques_.size()) ? measuredJointTorques_(i) : 0.0;
                       });
    logger.addLogEntry(category_ + "_MEKF_estimatedState_jointTorqueResidual_" + jointName,
                       [this, i]() -> double {
                         return i < static_cast<size_t>(estimatedJointTorqueResidual_.size())
                                    ? estimatedJointTorqueResidual_(i)
                                    : 0.0;
                       });
    logger.addLogEntry(category_ + "_MEKF_estimatedState_jointAcceleration_" + jointName,
                       [this, i]() -> double {
                         const auto & acceleration = observer_.getJointAccelerationEstimate();
                         return i < static_cast<size_t>(acceleration.size()) ? acceleration(i) : 0.0;
                       });
    logger.addLogEntry(category_ + "_MEKF_inputs_jointAccelerationFiniteDifference_" + jointName,
                       [this, i]() -> double {
                         return i < static_cast<size_t>(jointAccelerationFiniteDifference_.size())
                                    ? jointAccelerationFiniteDifference_(i)
                                    : 0.0;
                       });
    logger.addLogEntry(category_ + "_MEKF_inputs_jointTorque_" + jointName, [this, i]() -> double
                       { return i < static_cast<size_t>(measuredJointTorques_.size()) ? measuredJointTorques_(i) : 0.0; });
  }

  if(withDebugLogs_)
  {
    logger.addLogEntry(category_ + "_constants_mass", [this]() -> double { return mass_; });

    logger.addLogEntry(category_ + "_debug_disturbanceWrenchBias_force",
                       [this]() -> Eigen::Vector3d & { return disturbanceWrenchOffset_.force(); });
    logger.addLogEntry(category_ + "_debug_disturbanceWrenchBias_moment",
                       [this]() -> Eigen::Vector3d & { return disturbanceWrenchOffset_.moment(); });

    logger.addLogEntry(category_ + "_debug_config_OdometryType",
                       [this]() -> std::string { return so::odometry::odometryTypeToString(odometryType_); });

    for(auto & imu : listIMUs_)
    {
      logger.addLogEntry(category_ + "_MEKF_measurements_gyro_" + imu.name() + "_measured",
                         [this, &imu]() -> Eigen::Vector3d { return observer_.getImus().at(imu.id()).gyroMeas_; });

      logger.addLogEntry(category_ + "_MEKF_measurements_accelerometer_" + imu.name() + "_measured",
                         [this, &imu]() -> Eigen::Vector3d { return observer_.getImus().at(imu.id()).acceleroMeas_; });

      /* Inputs */
      logger.addLogEntry(category_ + "_MEKF_inputs_additionalWrench_Force",
                         [this]() -> Eigen::Vector3d { return inputWrench_.segment(0, 3); });
      logger.addLogEntry(category_ + "_MEKF_inputs_additionalWrench_Torque",
                         [this]() -> Eigen::Vector3d { return inputWrench_.segment(3, 3); });

      if(ctl.realRobot(robot_).hasBody("LeftFoot"))
      {
        logger.addLogEntry(category_ + "_realRobot_LeftFoot",
                           [this, &ctl]() { return ctl.realRobot(robot_).frame("LeftFoot").position(); });
      }

      if(ctl.realRobot(robot_).hasBody("RightFoot"))
      {
        logger.addLogEntry(category_ + "_realRobot_RightFoot",
                           [this, &ctl]() { return ctl.realRobot(robot_).frame("RightFoot").position(); });
      }

      if(ctl.realRobot(robot_).hasBody("LeftHand"))
      {
        logger.addLogEntry(category_ + "_realRobot_LeftHand",
                           [this, &ctl]() { return ctl.realRobot(robot_).frame("LeftHand").position(); });
      }
      if(ctl.realRobot(robot_).hasBody("RightHand"))
      {
        logger.addLogEntry(category_ + "_realRobot_RightHand",
                           [this, &ctl]() { return ctl.realRobot(robot_).frame("RightHand").position(); });
      }
      if(ctl.robot(robot_).hasBody("LeftFoot"))
      {
        logger.addLogEntry(category_ + "_ctlRobot_LeftFoot",
                           [this, &ctl]() { return ctl.robot(robot_).frame("LeftFoot").position(); });
      }
      if(ctl.robot(robot_).hasBody("RightFoot"))
      {
        logger.addLogEntry(category_ + "_ctlRobot_RightFoot",
                           [this, &ctl]() { return ctl.robot(robot_).frame("RightFoot").position(); });
      }

      if(ctl.robot(robot_).hasBody("LeftHand"))
      {
        logger.addLogEntry(category_ + "_ctlRobot_LeftHand",
                           [this, &ctl]() { return ctl.robot(robot_).frame("LeftHand").position(); });
      }

      if(ctl.robot(robot_).hasBody("RightHand"))
      {
        logger.addLogEntry(category_ + "_ctlRobot_RightHand",
                           [this, &ctl]() { return ctl.robot(robot_).frame("RightHand").position(); });
      }

      /* Plots of the inputs */

      logger.addLogEntry(category_ + "_MEKF_inputs_angularMomentum",
                         [this]() -> Eigen::Vector3d { return observer_.getInput().sigma_; });
      logger.addLogEntry(category_ + "_MEKF_inputs_angularMomentumDot",
                         [this]() -> Eigen::Vector3d { return observer_.getInput().sigmad_; });

      logger.addLogEntry(category_ + "_debug_worldInputRobotKine_position",
                         [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").posW().translation(); });
      logger.addLogEntry(category_ + "_debug_worldInputRobotKine_orientation",
                         [this]() -> Eigen::Quaternion<double>
                         {
                           return so::kine::Orientation(so::Matrix3(my_robots_->robot("inputRobot").posW().rotation()))
                               .inverse()
                               .toQuaternion();
                         });
      logger.addLogEntry(category_ + "_debug_worldInputRobotKine_linVel",
                         [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").velW().linear(); });
      logger.addLogEntry(category_ + "_debug_worldInputRobotKine_angVel",
                         [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").velW().angular(); });
      logger.addLogEntry(category_ + "_debug_worldInputRobotKine_linAcc",
                         [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").accW().linear(); });
      logger.addLogEntry(category_ + "_debug_worldInputRobotKine_angAcc",
                         [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").accW().angular(); });
    }
  }
}

void MCKineticsObserverFG::removeFromLogger(mc_rtc::Logger & logger, const std::string &)
{
  logger.removeLogEntry(category_ + "_posW");
  logger.removeLogEntry(category_ + "_velW");
  logger.removeLogEntry(category_ + "_mass");

  logger.removeLogEntry(category_ + "_flexStiffness");
  logger.removeLogEntry(category_ + "_flexDamping");

  for(const auto & jointName : jointTorqueNames_)
  {
    logger.removeLogEntry(category_ + "_MEKF_measurements_jointTorque_measured_" + jointName);
    logger.removeLogEntry(category_ + "_MEKF_estimatedState_jointTorqueResidual_" + jointName);
    logger.removeLogEntry(category_ + "_MEKF_inputs_jointTorque_" + jointName);
  }
}

void MCKineticsObserverFG::setOdometryType(const std::string & newOdometryType)
{
  prevOdometryType_ = odometryType_;
  odometryType_ = so::odometry::stringToOdometryType(newOdometryType);

  // if the type didn't change, we stop the function here
  if(odometryType_ == prevOdometryType_) { return; }

  mc_rtc::log::info("[{}]: Odometry mode changed to: {}", name(), newOdometryType);
  // valinor_.setOdometryType(odometryType_);
}

void MCKineticsObserverFG::addToGUI(const mc_control::MCController & ctl,
                                    mc_rtc::gui::StateBuilder & gui,
                                    const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;

  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  std::vector<std::string> removeOffsetCategory = category;
  removeOffsetCategory.insert(removeOffsetCategory.end(), {"RemoveDisturbanceWrenchOffset"});

  gui.addPlot("Unbiased external wrench", mc_rtc::gui::plot::X("t", [&logger]() { return logger.t(); }),
              mc_rtc::gui::plot::Y(
                  "Force x", [this]() { return getUnbiasedEstimatedDisturbanceWrench().force()(0); }, Color::Red),
              mc_rtc::gui::plot::Y(
                  "Force y", [this]() { return getUnbiasedEstimatedDisturbanceWrench().force()(1); }, Color::Blue),
              mc_rtc::gui::plot::Y(
                  "Force z", [this]() { return getUnbiasedEstimatedDisturbanceWrench().force()(2); }, Color::Green),
              mc_rtc::gui::plot::Y(
                  "Moment x", [this]() { return getUnbiasedEstimatedDisturbanceWrench().moment()(0); }, Color::Magenta),
              mc_rtc::gui::plot::Y(
                  "Moment y", [this]() { return getUnbiasedEstimatedDisturbanceWrench().moment()(1); }, Color::Cyan),
              mc_rtc::gui::plot::Y(
                  "Moment z", [this]() { return getUnbiasedEstimatedDisturbanceWrench().moment()(2); }, Color::Black));

  gui.addElement({category},
                 mc_rtc::gui::Button("Remove disturbance wrench offset",
                                     [this]()
                                     {
                                       // when clicking the button, the observer initializes the offset with the current
                                       // disturbance wrench estimation
                                       mc_rtc::log::info("Start removing disturbance wrench offset ");

                                       wrenchOffsetIndex_ = 0;
                                       removeWrenchOffset_ = true;
                                       disturbanceWrenchOffset_.force() = observer_.getCurrentState().disturbForce_;
                                       disturbanceWrenchOffset_.moment() = observer_.getCurrentState().disturbMoment_;
                                     }));

  if(odometryType_ != so::odometry::OdometryType::None)
  {
    std::vector<std::string> odomCategory = category;
    odomCategory.insert(odomCategory.end(), {"Odometry"});
    gui.addElement({odomCategory},
                   mc_rtc::gui::ComboInput(
                       "Choose from list",
                       {so::odometry::odometryTypeToString(so::odometry::OdometryType::Odometry6d),
                        so::odometry::odometryTypeToString(so::odometry::OdometryType::Flat)},
                       [this]() -> std::string { return so::odometry::odometryTypeToString(odometryType_); },
                       [this](const std::string & typeOfOdometry) { setOdometryType(typeOfOdometry); }));
  }
  // clang-format on
}

void MCKineticsObserverFG::addContactToGui(const mc_control::MCController & ctl,
                                           KoContactWithSensor & contact,
                                           mc_rtc::Logger & logger)
{
  std::vector<std::string> contactCategory;
  contactCategory.insert(contactCategory.end(),
                         {"ObserverPipelines", ctl.observerPipeline().name(), name(), "Contacts"});
  ctl.gui()->addElement(&contact, {contactCategory},
                        mc_rtc::gui::Checkbox(
                            contact.surfaceName() + " : " + (contact.isSet() ? "Contact is set" : "Contact is not set")
                                + ": Use wrench sensor: ",
                            [&contact]() { return contact.sensorEnabled_; },
                            [this, &contact, &logger]()
                            {
                              if(!contact.sensorEnabled_)
                              {
                                contact.sensorEnabled_ = true;
                                mc_rtc::log::info("{}: contact's sensors enabled", contact.surfaceName());
                                if(contact.isSet()) { addContactMeasurementsLogEntries(logger, contact); }
                              }
                              else
                              {
                                contact.sensorEnabled_ = false;
                                mc_rtc::log::info("{}: contact's sensors disabled", contact.surfaceName());
                                if(contact.isSet()) { removeContactMeasurementsLogEntries(logger, contact); }
                              }
                            }));
}

void MCKineticsObserverFG::addContactLogEntries(const mc_control::MCController & ctl,
                                                mc_rtc::Logger & logger,
                                                const KoContactWithSensor & contact)
{
  logger.addLogEntry(category_ + "_MEKF_estimatedState_contact_" + contact.surfaceName() + "_position", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getContact(contact.id()).currentState_.pos_; });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_contact_" + contact.surfaceName() + "_orientation", &contact,
                     [this, &contact]() -> Eigen::Quaternion<double>
                     {
                       so::kine::Orientation ori(observer_.getContact(contact.id()).currentState_.ori_.matrix());
                       return ori.inverse().toQuaternion();
                     });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_contact_" + contact.surfaceName() + "_forces", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getContact(contact.id()).currentState_.force_; });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_contact_" + contact.surfaceName() + "_torques", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getContact(contact.id()).currentState_.moment_; });

  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.surfaceName() + "_inputCentroidContactKine_position",
                     &contact,
                     [this, &contact]() -> Eigen::Vector3d {
                       return observer_.getContact(contact.id()).viscoElasticInput_.centroidContactPose_.translation();
                     });

  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.surfaceName() + "_inputCentroidContactKine_orientation", &contact,
      [this, &contact]() -> Eigen::Quaternion<double>
      {
        so::kine::Orientation ori(
            observer_.getContact(contact.id()).viscoElasticInput_.centroidContactPose_.rotation().matrix());
        return ori.inverse().toQuaternion();
      });
  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.surfaceName() + "_inputCentroidContactKine_linVel",
                     &contact, [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getContact(contact.id()).viscoElasticInput_.centroidContactLinVel_; });

  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.surfaceName() + "_inputCentroidContactKine_angVel",
                     &contact, [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getContact(contact.id()).viscoElasticInput_.centroidContactAngVel_; });
  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.surfaceName() + "_realRobot_position", &contact,
      [this, &contact, &ctl]() -> Eigen::Vector3d
      {
        const auto & realRobot = ctl.realRobot(robot_);
        return getContactWorldKinematics(ctl, const_cast<KoContactWithSensor &>(contact), realRobot, true).position();
      });

  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.surfaceName() + "_ctlRobot_position", &contact,
      [this, &contact, &ctl]() -> Eigen::Vector3d
      { return getCtlContactWorldKinematics(ctl, const_cast<KoContactWithSensor &>(contact), true).position(); });

  logger.addLogEntry(category_ + "_debug_contactState_isSet_" + contact.surfaceName(), &contact,
                     [&contact]() -> std::string { return contact.isSet() ? "Set" : "notSet"; });
}

void MCKineticsObserverFG::addContactMeasurementsLogEntries(mc_rtc::Logger & logger,
                                                            const KoContactWithSensor & contact)
{
  // Measurements
  logger.addLogEntry(category_ + "_MEKF_measurements_contacts_force_" + contact.surfaceName() + "_measured", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return *(observer_.getContact(contact.id()).forceMeas_); });
  logger.addLogEntry(category_ + "_MEKF_measurements_contacts_torque_" + contact.surfaceName() + "_measured", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return *(observer_.getContact(contact.id()).momentMeas_); });
}

void MCKineticsObserverFG::removeContactLogEntries(mc_rtc::Logger & logger, const KoContactWithSensor & contact)
{
  logger.removeLogEntries(&contact);
}

void MCKineticsObserverFG::removeContactMeasurementsLogEntries(mc_rtc::Logger & logger,
                                                               const KoContactWithSensor & contact)
{
  // Innovation
  logger.removeLogEntry(category_ + "_innovation_contacts_" + contact.surfaceName() + "_position");
  logger.removeLogEntry(category_ + "_innovation_contacts_" + contact.surfaceName() + "_orientation");
  logger.removeLogEntry(category_ + "_innovation_contacts_" + contact.surfaceName() + "_force");
  logger.removeLogEntry(category_ + "_innovation_contacts_" + contact.surfaceName() + "_torque");

  logger.removeLogEntry(category_ + "_measurements_contacts_force_" + contact.surfaceName() + "_measured");
  logger.removeLogEntry(category_ + "_measurements_contacts_force_" + contact.surfaceName() + "_predicted");
  logger.removeLogEntry(category_ + "_measurements_contacts_force_" + contact.surfaceName() + "_corrected");

  logger.removeLogEntry(category_ + "_measurements_contacts_torque_" + contact.surfaceName() + "_measured");
  logger.removeLogEntry(category_ + "_measurements_contacts_torque_" + contact.surfaceName() + "_predicted");
  logger.removeLogEntry(category_ + "_measurements_contacts_torque_" + contact.surfaceName() + "_corrected");
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("MCKineticsObserverFG", mc_state_observation::MCKineticsObserverFG)
