/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */
#include <mc_observers/ObserverMacros.h>
#include <mc_rbdyn/ForceSensor.h>
#include <RBDyn/Jacobian.h>
#include <mc_rtc/logging.h>

#include "mc_state_observation/measurements/ContactsDetector.h"
#include <mc_state_observation/MCKineticsObserver.h>
#include <mc_state_observation/gui_helpers.h>
#include <state-observation/tools/rigid-body-kinematics.hpp>

#include <mc_state_observation/conversions/kinematics.h>

#include <cmath>
#include <map>

namespace so = stateObservation;
namespace mc_state_observation
{
namespace
{
/// Controller period the configured process variances are expressed for (200 Hz).
constexpr double kProcessReferenceTimeStep = 0.005;
} // namespace

MCKineticsObserver::MCKineticsObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), maxContacts_(3), maxIMUs_(1), observer_(maxContacts_, maxIMUs_),
  valinor_(type, dt, true), removeWrenchOffset_(false)
{ observer_.setSamplingTime(dt); }

///////////////////////////////////////////////////////////////////////
/// --------------------------Core functions---------------------------
///////////////////////////////////////////////////////////////////////

void MCKineticsObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  valinor_.name(name() + "BackupValinor");
  valinor_.configure(ctl, config);

  robot_ = config("robot", ctl.robot().name());

  imuNames_ = config("imuNames", std::vector<std::string>());
  listIMUs_.clear();
  if(!imuNames_.empty())
  {
    for(size_t i = 0; i < imuNames_.size(); ++i) { listIMUs_.push_back({i, imuNames_[i]}); }
  }
  else
  {
    listIMUs_.push_back({0, ctl.robot(robot_).bodySensor().name()});
  }
  imuInputKinematics_.resize(listIMUs_.size());

  config("debug", debug_);
  config("verbose", verbose_);
  config("withGui", withGui_);

  // we set the desired type of odometry
  auto leggedOdomConfig = config("leggedOdometry");
  std::string typeOfOdometry = static_cast<std::string>(leggedOdomConfig("odometryType"));
  odometryType_ = so::odometry::stringToOdometryType(typeOfOdometry);

  config("withDebugLogs", withDebugLogs_);

  /* configuration of the contacts manager */
  auto contactsConfig = config("contacts");
  pinContacts_ = config("pinContacts", false);
  noAngularFlexibility_ = config("noAngularFlexibility", false);
  legKinematicsContacts_ = config("legKinematicsContacts", false);
  legKinematicsJointVariance_ = config("legKinematicsJointVariance", 2e-4);
  legKinematicsFloatingBaseNoise_ = config("legKinematicsFloatingBaseNoise", true);
  legKinematicsPositionOnly_ = config("legKinematicsPositionOnly", false);
  legKinematicsWrenchCorrection_ = config("legKinematicsWrenchCorrection", false);
  legKinematicsCompliant_ = config("legKinematicsCompliant", false);
  legKinematicsCompliantProcess_ = config("legKinematicsCompliantProcess", false);
  legKinematicsDeflection_ = config("legKinematicsDeflection", false);
  legKinematicsWrenchState_ = config("legKinematicsWrenchState", false);
  legKinematicsWrenchDelay_ = config("legKinematicsWrenchDelay", false);
  contactWrenchInitFromMeasurement_ = config("contactWrenchInitFromMeasurement", false);
  legKinematicsWrenchStateNoise_ = config("legKinematicsWrenchStateNoise", false);
  contactRestOrientationErrorDeg_ = contactsConfig("restOrientationErrorDeg", 0.0);
  contactRestOrientationSeed_ = contactsConfig("restOrientationErrorSeed", unsigned(0));
  if(!std::isfinite(contactRestOrientationErrorDeg_) || contactRestOrientationErrorDeg_ < 0.0
     || contactRestOrientationErrorDeg_ > 180.0)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("restOrientationErrorDeg must be in [0, 180]");
  }
  ignoredSensorWrenches_.clear();
  for(const auto & sensor : contactsConfig("ignoredSensors", std::vector<std::string>{}))
  {
    if(!ctl.robot(robot_).hasForceSensor(sensor))
    {
      mc_rtc::log::error_and_throw<std::invalid_argument>("Unknown ignored force sensor: {}", sensor);
    }
    ignoredSensorWrenches_.emplace(sensor, so::Vector6::Zero());
  }

  forceSensorMeasurements_.clear();
  for(const auto & forceSensor : ctl.robot(robot_).forceSensors())
  {
    forceSensorMeasurements_.emplace(forceSensor.name(), forceSensor.wrenchWithoutGravity(ctl.realRobot(robot_)));
  }

  std::string contactsDetectionString = static_cast<std::string>(contactsConfig("contactsDetection"));
  KoContactsDetector::ContactsDetection contactsDetectionMethod =
      KoContactsDetector::stringToContactsDetection(contactsDetectionString, name());

  if(contactsDetectionMethod == KoContactsDetector::ContactsDetection::Surfaces)
  {
    std::vector<std::string> surfacesForContactDetection =
        contactsConfig("surfacesForContactDetection", std::vector<std::string>());

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

  config("withUnmodeledWrench", withUnmodeledWrench_);
  config("withGyroBias", withGyroBias_);
  if(pinContacts_)
  {
    withUnmodeledWrench_ = false;
    withGyroBias_ = false;
  }

  bool withFiniteDifferences = false;
  config("withFiniteDifferences", withFiniteDifferences);

  observer_.setWithUnmodeledWrench(withUnmodeledWrench_);
  observer_.setWithGyroBias(withGyroBias_);

  if(withFiniteDifferences)
  {
    double finiteDifferenceStep = static_cast<double>(config("finiteDifferenceStep"));
    observer_.useFiniteDifferencesJacobians(withFiniteDifferences);
    so::Vector dx(observer_.getStateSize());
    dx.setConstant(finiteDifferenceStep);
    observer_.setFiniteDifferenceStep(dx);
  }

  bool withAccelerationEstimation = true;
  config("withAccelerationEstimation", withAccelerationEstimation);
  observer_.setWithAccelerationEstimation(withAccelerationEstimation);
  observer_.setWithDampingInMatrixA(config("withDampingInMatrixA", true));
  observer_.setWithContactKinematicsMeasurement(legKinematicsContacts_);
  observer_.setWithContactWrenchCorrection(legKinematicsContacts_ && legKinematicsWrenchCorrection_);
  observer_.setWithContactKinematicsDamping(legKinematicsContacts_ && legKinematicsCompliant_);
  observer_.setWithContactKinematicsDeflection(legKinematicsContacts_ && legKinematicsCompliant_
                                               && legKinematicsDeflection_);
  observer_.setWithContactWrenchFeedForward(legKinematicsContacts_ && legKinematicsCompliant_ && legKinematicsDeflection_
                                            && legKinematicsWrenchState_);
  observer_.setWithContactWrenchFeedForwardNoise(legKinematicsWrenchStateNoise_);

  if(config.has("withAdaptativeContactProcessCov"))
  {
    observer_.setWithAdaptativeContactProcessCov(config("withAdaptativeContactProcessCov"));
  }
  wrenchCalibration_.clear();
  for(const auto & [surface, correction] :
      config("wrenchCalibration", std::map<std::string, std::vector<double>>{}))
  {
    if(correction.size() != 3 || !std::all_of(correction.begin(), correction.end(), [](double value) { return std::isfinite(value); }))
    {
      mc_rtc::log::error_and_throw<std::invalid_argument>("wrenchCalibration.{} must contain three finite values", surface);
    }
    wrenchCalibration_.emplace(surface, so::Vector3(correction[0], correction[1], correction[2]));
  }

  linStiffness_ = (contactsConfig("linStiffness").operator so::Vector3()).matrix().asDiagonal();
  angStiffness_ = (contactsConfig("angStiffness").operator so::Vector3()).matrix().asDiagonal();
  linDamping_ = (contactsConfig("linDamping").operator so::Vector3()).matrix().asDiagonal();
  angDamping_ = (contactsConfig("angDamping").operator so::Vector3()).matrix().asDiagonal();
  if(pinContacts_)
  {
    angStiffness_.setZero();
    angDamping_(0, 0) = angDamping_(1, 1) = 0.0;
  }
  if(noAngularFlexibility_)
  {
    // The whole angular channel goes, yaw damping included, so the reaction torque no longer
    // depends on the contact orientation at all.
    angStiffness_.setZero();
    angDamping_.setZero();
  }

  zeroPose_.translation().setZero();
  zeroPose_.rotation().setIdentity();
  zeroMotion_.linear().setZero();
  zeroMotion_.angular().setZero();

  auto ekfStateProcessVariances = config("ekfStateProcessVariances");
  auto ekfSensorNoiseVariances = config("ekfSensorNoiseVariances");
  // Initial State
  statePositionInitCovariance_ =
      (ekfStateProcessVariances("statePositionInitVariance").operator so::Vector3()).matrix().asDiagonal();
  stateOriInitCovariance_ =
      (ekfStateProcessVariances("stateOriInitVariance").operator so::Vector3()).matrix().asDiagonal();
  stateLinVelInitCovariance_ =
      (ekfStateProcessVariances("stateLinVelInitVariance").operator so::Vector3()).matrix().asDiagonal();
  stateAngVelInitCovariance_ =
      (ekfStateProcessVariances("stateAngVelInitVariance").operator so::Vector3()).matrix().asDiagonal();
  gyroBiasInitCovariance_.setZero();
  unmodeledWrenchInitCovariance_.setZero();

  contactInitCovarianceFirstContacts_.setZero();
  contactInitCovarianceFirstContacts_flat_.setZero();
  // if we stick to the control robot's anchor frame, we don't allow the correction of the contacts pose
  contactInitCovarianceFirstContacts_.block<3, 3>(0, 0) =
      (ekfStateProcessVariances("contactPositionInitVarianceFirstContacts").operator so::Vector3())
          .matrix()
          .asDiagonal();
  contactInitCovarianceFirstContacts_.block<3, 3>(3, 3) =
      (ekfStateProcessVariances("contactOriInitVarianceFirstContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceFirstContacts_.block<3, 3>(6, 6) =
      (ekfStateProcessVariances("contactForceInitVarianceFirstContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceFirstContacts_.block<3, 3>(9, 9) =
      (ekfStateProcessVariances("contactTorqueInitVarianceFirstContacts").operator so::Vector3()).matrix().asDiagonal();

  contactInitCovarianceNewContacts_.setZero();
  contactInitCovarianceNewContacts_flat_.setZero();

  contactInitCovarianceNewContacts_.block<3, 3>(0, 0) =
      (ekfStateProcessVariances("contactPositionInitVarianceNewContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceNewContacts_.block<3, 3>(3, 3) =
      (ekfStateProcessVariances("contactOriInitVarianceNewContacts").operator so::Vector3()).matrix().asDiagonal();

  contactInitCovarianceNewContacts_.block<3, 3>(6, 6) =
      (ekfStateProcessVariances("contactForceInitVarianceNewContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceNewContacts_.block<3, 3>(9, 9) =
      (ekfStateProcessVariances("contactTorqueInitVarianceNewContacts").operator so::Vector3()).matrix().asDiagonal();

  // Process //
  statePositionProcessCovariance_ =
      (ekfStateProcessVariances("statePositionProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  stateOriProcessCovariance_ =
      (ekfStateProcessVariances("stateOriProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  stateLinVelProcessCovariance_ =
      (ekfStateProcessVariances("stateLinVelProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  stateAngVelProcessCovariance_ =
      (ekfStateProcessVariances("stateAngVelProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  gyroBiasProcessCovariance_.setZero();
  unmodeledWrenchProcessCovariance_.setZero();

  contactProcessCovariance_.setZero();
  // if we stick to the control robot's anchor frame, we don't allow the correction of the contacts pose

  // The rest-pose process covariance is always taken from the configuration. withAdaptativeContactProcessCov only
  // selects how it reaches Q: written directly by setContactProcessCovMat when disabled, or spread across the set
  // contacts by updateContactCovariances (covMv = M * cov * M) when enabled. Zeroing it here would make that product
  // zero as well, freezing every contact rest pose for the whole run.
  contactProcessCovariance_.block<3, 3>(0, 0) =
      (ekfStateProcessVariances("contactPositionProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  contactProcessCovariance_.block<3, 3>(3, 3) =
      (ekfStateProcessVariances("contactOrientationProcessVariance").operator so::Vector3()).matrix().asDiagonal();

  contactProcessCovariance_.block<3, 3>(6, 6) =
      (ekfStateProcessVariances("contactForceProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  contactProcessCovariance_.block<3, 3>(9, 9) =
      (ekfStateProcessVariances("contactTorqueProcessVariance").operator so::Vector3()).matrix().asDiagonal();

  // Unmodeled Wrench //
  if(withUnmodeledWrench_)
  {
    // initial
    unmodeledWrenchInitCovariance_.block<3, 3>(0, 0) =
        (ekfStateProcessVariances("unmodeledForceInitVariance").operator so::Vector3()).matrix().asDiagonal();
    unmodeledWrenchInitCovariance_.block<3, 3>(3, 3) =
        (ekfStateProcessVariances("unmodeledTorqueInitVariance").operator so::Vector3()).matrix().asDiagonal();

    // process
    unmodeledWrenchProcessCovariance_.block<3, 3>(0, 0) =
        (ekfStateProcessVariances("unmodeledForceProcessVariance").operator so::Vector3()).matrix().asDiagonal();
    unmodeledWrenchProcessCovariance_.block<3, 3>(3, 3) =
        (ekfStateProcessVariances("unmodeledTorqueProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  }
  // Gyrometer Bias
  if(withGyroBias_)
  {
    gyroBiasInitCovariance_ =
        (ekfStateProcessVariances("gyroBiasInitVariance").operator so::Vector3()).matrix().asDiagonal();
    gyroBiasProcessCovariance_ =
        (ekfStateProcessVariances("gyroBiasProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  }

  // The configured process variances are densities expressed for a 200 Hz controller, the rate
  // every dataset but HRP5P_LongWalk was tuned at. Q is added to the EKF unscaled once per
  // iteration, so running the same numbers at another period changes the noise injected per
  // second -- LongWalk at 500 Hz would get 2.5x too much. Hartley discretises the same way
  // (InEKF.cpp:181, `Qk_hat = PhiAdj * Qk * PhiAdj.transpose() * dt`), so scaling here also makes
  // a covariance mean the same thing in both estimators' configurations, up to the constant
  // 1 / kProcessReferenceTimeStep.
  //
  // The contact force and torque blocks are deliberately left alone: they are held fixed by the
  // tuning study rather than identified per rate.
  const double processCovarianceScale = ctl.timeStep / kProcessReferenceTimeStep;
  statePositionProcessCovariance_ *= processCovarianceScale;
  stateOriProcessCovariance_ *= processCovarianceScale;
  stateLinVelProcessCovariance_ *= processCovarianceScale;
  stateAngVelProcessCovariance_ *= processCovarianceScale;
  gyroBiasProcessCovariance_ *= processCovarianceScale;
  unmodeledWrenchProcessCovariance_ *= processCovarianceScale;
  contactProcessCovariance_.block<3, 3>(0, 0) *= processCovarianceScale;
  contactProcessCovariance_.block<3, 3>(3, 3) *= processCovarianceScale;

  // Sensor //
  positionSensorCovariance_ =
      (ekfSensorNoiseVariances("positionSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  orientationSensorCoVariance_ =
      (ekfSensorNoiseVariances("orientationSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  acceleroSensorCovariance_ =
      (ekfSensorNoiseVariances("acceleroSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  gyroSensorCovariance_ = (ekfSensorNoiseVariances("gyroSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  contactSensorCovariance_.setZero();
  contactSensorCovariance_.block<3, 3>(0, 0) =
      (ekfSensorNoiseVariances("forceSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  contactSensorCovariance_.block<3, 3>(3, 3) =
      (ekfSensorNoiseVariances("torqueSensorVariance").operator so::Vector3()).matrix().asDiagonal();

  if(noAngularFlexibility_)
  {
    // The rest of this option lives above, where the angular stiffness and damping are zeroed. These
    // two lines cannot sit there: neither the contact torque process covariance (read below, and
    // rescaled by the timestep afterwards) nor the torque sensor covariance exists yet at that point.
    //
    // Zeroing the angular visco-elastic law alone does NOT remove the angular contact channel, it
    // replaces it by a one-step pass-through of the raw measured torque. computeContactWrench_ then
    // predicts a contact torque that is identically zero and stateDynamics rewrites the torque state
    // from that law at every step, but the torque MEASUREMENT stays and measureDynamics predicts it
    // as the state itself, so the filter faces an innovation equal to the whole measured torque
    // forever, and the corrected value is fed back into the angular dynamics on the next step with
    // every Jacobian of that state zeroed. Removing the measurement makes the model and the
    // observation agree. Dominating it rather than deleting it is deliberate: the transformed torque
    // block keeps skew(p).R.Cf.R'.skew(p)' from the FORCE covariance (about 5e-3 for a 0.105 m
    // lever), which against 9e90 is a removal in fact, and S = HPH' + R stays far from the limit of
    // a double.
    //
    // The process covariance of the torque state goes with it. Left at its configured value it would
    // be a random walk with no measurement and no Jacobian, injected into the angular dynamics every
    // step -- harmful whatever else it does. NOTE: this is not put forward as the cause of the
    // yaw-bias jumps observed on the injected-bias long walk; that investigation is Codex's.
    contactSensorCovariance_.block<3, 3>(3, 3) = so::Matrix3::Identity() * 9e90;
    contactProcessCovariance_.block<3, 3>(9, 9).setZero();
  }

  setObserverCovariances();

  /* Configuration of the backup based on the Tilt Observer */
  // interval (in s) on which the backup will recover
  double backupInterval = config("backupInterval", 0.5);
  fbBackupCapacity_ = size_t(backupInterval / ctl.timeStep);

  koBackupFbKinematics_.set_capacity(fbBackupCapacity_);
  valinor_.backupFbKinematics_.set_capacity(fbBackupCapacity_);

  invincibilityFrame_ = int(1.5 / ctl.timeStep);

  std::vector<std::string> nanBehaviourCategory;
  nanBehaviourCategory.insert(nanBehaviourCategory.end(), {"ObserverPipelines", ctl.observerPipeline().name(), name()});
  if(withGui_)
  {
    ctl.gui()->addElement({nanBehaviourCategory},
                          mc_rtc::gui::Button("SimulateNanBehaviour", [this]() { observer_.nanDetected_ = true; }));
  }
}

void MCKineticsObserver::setObserverCovariances()
{
  // initialization of the observers covariances
  observer_.setKinematicsInitCovarianceDefault(statePositionInitCovariance_, stateOriInitCovariance_,
                                               stateLinVelInitCovariance_, stateAngVelInitCovariance_);
  observer_.setGyroBiasInitCovarianceDefault(gyroBiasInitCovariance_);
  observer_.setUnmodeledWrenchInitCovMatDefault(unmodeledWrenchInitCovariance_);
  observer_.setContactInitCovMatDefault(contactInitCovarianceFirstContacts_);
  observer_.resetStateCovarianceMat();

  observer_.setKinematicsProcessCovarianceDefault(statePositionProcessCovariance_, stateOriProcessCovariance_,
                                                  stateLinVelProcessCovariance_, stateAngVelProcessCovariance_);
  observer_.setGyroBiasProcessCovarianceDefault(gyroBiasProcessCovariance_);
  observer_.setUnmodeledWrenchProcessCovarianceDefault(unmodeledWrenchProcessCovariance_);
  observer_.setContactProcessCovarianceDefault(contactProcessCovariance_);

  observer_.resetProcessCovarianceMat();

  observer_.setIMUDefaultCovarianceMatrix(acceleroSensorCovariance_, gyroSensorCovariance_);
  observer_.setContactWrenchSensorDefaultCovarianceMatrix(contactSensorCovariance_);
  so::Matrix6 absPoseSensorDefCovariance = so::Matrix6::Zero();
  absPoseSensorDefCovariance.block(0, 0, observer_.sizePos, observer_.sizePos) = positionSensorCovariance_;
  absPoseSensorDefCovariance.block(observer_.sizePos, observer_.sizePos, observer_.sizeOriTangent,
                                   observer_.sizeOriTangent) = orientationSensorCoVariance_;
  observer_.setAbsolutePoseSensorDefaultCovarianceMatrix(absPoseSensorDefCovariance);
  // observer_.setAbsoluteOriSensorDefaultCovarianceMatrix(absoluteOriSensorCovariance_);
}

void MCKineticsObserver::reset(const mc_control::MCController & ctl)
{
  contactRestOrientationRng_.seed(contactRestOrientationSeed_);
  valinor_.reset(ctl);

  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  mass(ctl.realRobot(robot_).mass());

  for(auto & [_, contact] : contactsManager_.contacts())
  {
    if(contact.isSet()) { observer_.removeContact(contact.id()); }
  }
  contactsManager_.reset();
  contactsDetector_.reset();
  for(const auto & forceSensor : robot.forceSensors())
  {
    forceSensorMeasurements_.at(forceSensor.name()) = forceSensor.wrenchWithoutGravity(realRobot);
  }

  /* Initialization of variables */
  X_0_fb_ = sva::PTransformd::Identity();
  v_fb_0_ = sva::MotionVecd::Zero();
  a_fb_0_ = sva::MotionVecd::Zero();
  lastBackupIter_ = 0;
  invincibilityIter_ = 0;

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
  mcko_K_0_fb_ = conversions::kinematics::fromSva(X_0_fb_, so::kine::Kinematics::Flags::pose);

  initObserverStateVector(ctl, realRobot);

  disturbanceWrenchOffset_.force().setZero();
  disturbanceWrenchOffset_.moment().setZero();
  worldAnchorPos_.setZero();
  fbAnchorPos_.setZero();
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

void MCKineticsObserver::updateFloatingBaseFromObserver()
{
  so::kine::Kinematics fbFb;
  fbFb.setZero<so::Matrix3>(so::kine::Kinematics::Flags::all);

  // Given the floating-base kinematics inside its own frame, the Kinetics Observer returns the floating-base
  // kinematics in the real world frame.
  mcko_K_0_fb_ = observer_.getGlobalKinematicsOf(fbFb);
  koBackupFbKinematics_.push_back(mcko_K_0_fb_);
}

void MCKineticsObserver::updateFloatingBaseKinematicsFromBackup(const so::kine::Kinematics & fbKinematics,
                                                                double timeStep)
{
  mcko_K_0_fb_ = fbKinematics;

  // The Tilt Observer doesn't estimate acceleration, so use finite differences.
  mcko_K_0_fb_.angAcc = (mcko_K_0_fb_.angVel() - v_fb_0_.angular()) / timeStep;
  mcko_K_0_fb_.linAcc = (mcko_K_0_fb_.linVel() - v_fb_0_.linear()) / timeStep;
}

void MCKineticsObserver::resetWorldCentroidState(mc_rbdyn::Robot & inputRobot, bool resetCovariance)
{
  update(inputRobot);
  inputRobot.forwardKinematics();

  so::kine::Kinematics newWorldCentroidKine;
  newWorldCentroidKine.position = inputRobot.com();
  newWorldCentroidKine.linVel = inputRobot.comVelocity();
  // The centroid frame orientation is the floating-base orientation.
  newWorldCentroidKine.orientation = mcko_K_0_fb_.orientation;
  newWorldCentroidKine.angVel = mcko_K_0_fb_.angVel();

  observer_.setWorldCentroidStateKinematics(newWorldCentroidKine, resetCovariance);
}

void MCKineticsObserver::resetContactsAfterBackup(const mc_control::MCController & ctl,
                                                  const mc_rbdyn::Robot & robot,
                                                  const mc_rbdyn::Robot & realRobot,
                                                  const mc_rbdyn::Robot & forceSensorRobot,
                                                  bool resetCovariance)
{
  for(auto & [_, contact] : contactsManager_.contacts())
  {
    if(!contact.isSet()) { continue; }

    // Update of the force measurements: the contribution of gravity changed with the backup pose.
    const mc_rbdyn::ForceSensor & forceSensor = forceSensorRobot.forceSensor(contact.fsName_);

    updateContactForceMeasurement(contact, forceSensor.wrenchWithoutGravity(realRobot));

    contact.fbContactKine_.reset();
    so::kine::Kinematics newWorldContactKineRef = getContactWorldKinematics(ctl, contact, robot, true);
    so::kine::Kinematics newWorldContactRestPose = getOdometryWorldContactRest(contact, newWorldContactKineRef);

    observer_.setStateContact(contact.id(), newWorldContactRestPose, contact.contactWrenchVector_, resetCovariance);
  }
}

const char * MCKineticsObserver::estimationStateName() const
{
  switch(estimationState_)
  {
    case noIssue:
      return "noIssue";
    case invincibilityFrame:
      return "invincibilityFrame";
    case errorDetected:
      return "errorDetected";
  }

  return "unknown";
}

bool MCKineticsObserver::run(const mc_control::MCController & ctl)
{
  valinor_.run(ctl);

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

  // The input robot copies the real robot to update the encoder values.
  // Then its floating base is brung back to the origin of the world frame and given zero velocities and accelerations
  // in order to ease the computations.
  inputRobot.posW(zeroPose_);
  inputRobot.velW(zeroMotion_);
  inputRobot.accW(zeroMotion_);

  inputRobot.forwardKinematics();
  inputRobot.forwardVelocity();
  inputRobot.forwardAcceleration();

  /** Center of mass (assumes FK, FV and FA are already done)
      Must be initialized now as used for the conversion from user to centroid frame !!! **/
  fbCoMKine_.position = inputRobot.com();
  fbCoMKine_.linVel = inputRobot.comVelocity();
  fbCoMKine_.linAcc = inputRobot.comAcceleration();

  observer_.setCenterOfMass(fbCoMKine_.position(), fbCoMKine_.linVel(), fbCoMKine_.linAcc());

  observer_.setCoMAngularMomentum(
      rbd::computeCentroidalMomentum(inputRobot.mb(), inputRobot.mbc(), fbCoMKine_.position()).moment(),
      rbd::computeCentroidalMomentumDot(inputRobot.mb(), inputRobot.mbc(), fbCoMKine_.position(), fbCoMKine_.linVel())
          .moment());

  observer_.setCoMInertiaMatrix(computeCentroidalInertia(inputRobot.mb(), inputRobot.mbc(), fbCoMKine_.position));

  // update of the contacts
  updateContacts(ctl, logger);

  /** Accelerometers **/
  updateIMUs(robot, inputRobot);

  // force measurements from sensor that are not associated to a currently set contact are given to the Kinetics
  // Observer as inputs.
  inputAdditionalWrench(inputRobot, robot);

  res_ = observer_.update();

  unbiasedDisturbanceWrench_.force() =
      res_.template segment<3>(observer_.unmodeledWrenchIndex()) - disturbanceWrenchOffset_.force();
  unbiasedDisturbanceWrench_.moment() =
      res_.template segment<3>(observer_.unmodeledTorqueIndex()) - disturbanceWrenchOffset_.moment();

  if(removeWrenchOffset_)
  {
    if(wrenchOffsetIndex_ > 100)
    {
      removeWrenchOffset_ = false;
      mc_rtc::log::info("Disturbance wrench offset removed");
    }

    auto disturbForce = res_.segment(observer_.unmodeledWrenchIndex(), 3);
    auto disturbMoment = res_.segment(observer_.unmodeledTorqueIndex(), 3);

    disturbanceWrenchOffset_.force() =
        disturbanceWrenchOffset_.force() + 0.1 * (disturbForce - disturbanceWrenchOffset_.force());
    disturbanceWrenchOffset_.moment() =
        disturbanceWrenchOffset_.moment() + 0.1 * (disturbMoment - disturbanceWrenchOffset_.moment());

    wrenchOffsetIndex_++;
  }

  if(observer_.nanDetected_) { estimationState_ = errorDetected; }
  else if(invincibilityIter_ > 0 && invincibilityIter_ < invincibilityFrame_) { estimationState_ = invincibilityFrame; }
  else
  {
    estimationState_ = noIssue;
  }

  // if no anomaly is detected and if we aren't in the "invicibility frame", we update the floating base with the
  // results of the Kinetics Observer
  switch(estimationState_)
  {
    case noIssue:
    {
      updateFloatingBaseFromObserver();
      break;
    }
    case invincibilityFrame:
    {
      // we apply the last transformation estimated by the Tilt Observer to our previous pose to keep updating the
      // floating base with the Tilt Observer.
      updateFloatingBaseKinematicsFromBackup(valinor_.applyLastTransformation(koBackupFbKinematics_.back()),
                                             ctl.timeStep);
      koBackupFbKinematics_.push_back(mcko_K_0_fb_);

      invincibilityIter_++;
      // While converging again after being reset, the estimation made by the Kinetics Observer is very inaccurate and
      // cannot be used. So we let it converge during the invincibility frame while using the estimation of the Tilt
      // Observer to update the real robot. Then we start over using the Kinetics Observer starting from the final
      // kinematics obtained from the Tilt Observer.
      if(invincibilityIter_ == invincibilityFrame_)
      {
        resetWorldCentroidState(inputRobot, false);
        resetContactsAfterBackup(ctl, robot, realRobot, robot, false);
      }

      break;
    }
    case errorDetected:
    {
      // an error was just detected, we reset the state vector and covariances and start the invicibility frame, during
      // which we let the Kinetics Observer converge before using it again.
      auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();
      if(logger.t() / ctl.timeStep < double(fbBackupCapacity_))
      {
        mc_rtc::log::warning("The backup function was called before the required time was ellapsed. The backup will be "
                             "performed using the last {} seconds",
                             logger.t());
      }

      if(logger.t() / ctl.timeStep - lastBackupIter_ < double(fbBackupCapacity_))
      {
        mc_rtc::log::warning("The backup function was called again too quickly. The backup will be "
                             "performed using the last {} seconds",
                             logger.t() - lastBackupIter_ * ctl.timeStep);
      }

      // We add an empty Kinematics object to the floating base pose buffer. This is because the buffer of the tilt
      // observer already contains the last estimation of the floating base so we prevent a disalignment of the two
      // buffers. This empty Kinematics is filled and returned by the "runBackup" function.
      koBackupFbKinematics_.push_back(so::kine::Kinematics::zeroKinematics(so::kine::Kinematics::Flags::pose));

      mcko_K_0_fb_ = valinor_.backupFb(&koBackupFbKinematics_);

      updateFloatingBaseKinematicsFromBackup(mcko_K_0_fb_, ctl.timeStep);

      // we update update robot as it will be updated at the beginning of the next iteration anyway
      resetWorldCentroidState(inputRobot, true);
      observer_.setStateUnmodeledWrench(so::Vector6::Zero(), true);

      for(size_t i = 0; i < listIMUs_.size(); ++i)
      {
        const auto & imu = listIMUs_[i];

        observer_.setGyroBias(imu.gyroBias, static_cast<unsigned int>(i), true);
      }

      resetContactsAfterBackup(ctl, robot, realRobot, inputRobot, true);

      // this variable indicates that we entered the invincibility frame
      invincibilityIter_ = 1;
      lastBackupIter_ = int(logger.t() / ctl.timeStep);

      observer_.nanDetected_ = false;

      break;
    }
  }

  if(odometryType_ != so::odometry::OdometryType::None)
  {
    X_0_fb_.rotation() = mcko_K_0_fb_.orientation.toMatrix3().transpose();
    X_0_fb_.translation() = mcko_K_0_fb_.position();

    v_fb_0_.angular() = mcko_K_0_fb_.angVel();
    v_fb_0_.linear() = mcko_K_0_fb_.linVel();

    if(observer_.getWithAccelerationEstimation())
    {
      a_fb_0_.angular() = mcko_K_0_fb_.angAcc();
      a_fb_0_.linear() = mcko_K_0_fb_.linAcc();
    }
  }
  else
  {
    Eigen::Vector3d worldAnchor = Eigen::Vector3d::Zero();
    Eigen::Vector3d fbAnchor = Eigen::Vector3d::Zero();
    double forceSum = 0.0;
    for(auto & [_, contact] : contactsManager_.contacts())
    {
      if(contact.isSet())
      {
        worldAnchor += getCtlContactWorldKinematics(ctl, contact, false).position() * contact.contactWrenchVector_(2);
        fbAnchor +=
            getContactWorldKinematics(ctl, contact, inputRobot, false).position() * contact.contactWrenchVector_(2);
        forceSum += contact.contactWrenchVector_(2);
      }
    }
    if(std::abs(forceSum) > 1e-9)
    {
      worldAnchorPos_ = worldAnchor / forceSum;
      fbAnchorPos_ = fbAnchor / forceSum;
    }

    so::kine::LocalKinematics worldFbLocalKine = observer_.getLocalCentroidKinematics();
    worldFbLocalKine.orientation = so::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
        mcko_K_0_fb_.orientation.toMatrix3(), ctl.robot(robot_).posW().rotation().transpose());
    so::kine::Kinematics worldFbKine_(worldFbLocalKine);

    worldFbKine_.position = worldAnchorPos_ - worldFbKine_.orientation.toMatrix3() * fbAnchorPos_;

    X_0_fb_.rotation() = worldFbLocalKine.orientation.toMatrix3().transpose();
    X_0_fb_.translation() = worldFbKine_.position();

    v_fb_0_.angular() = worldFbKine_.angVel();
    v_fb_0_.linear() = worldFbKine_.linVel();

    if(observer_.getWithAccelerationEstimation())
    {
      a_fb_0_.angular() = worldFbKine_.angAcc();
      a_fb_0_.linear() = worldFbKine_.linAcc();
    }
  }

  // MEKF_estimatedState is registered unconditionally, so the state it reads has to be refreshed
  // unconditionally too. Computing it only under withDebugLogs_ left that log PRESENT and FROZEN
  // at its last value rather than absent, which is worse than missing data: nothing downstream
  // could tell the difference.
  globalCentroidKinematics_ = observer_.getGlobalCentroidKinematics();

  if(withDebugLogs_)
  {
    /* Update of the logged variables */
    for(auto & [_, contact] : contactsManager_.contacts())
    {
      if(contact.isSet())
      {
        contact.viscoElasticWrenchAfterCorrection_ = observer_.getCurrentViscoElasticWrench(contact.id());
      }
    }

    correctedMeasurements_ = observer_.getEKF().getSimulatedMeasurement(observer_.getEKF().getCurrentTime());

    contactsPosAverageStateCov_.setZero();
    for(unsigned i = 0; i < maxContacts_; i++)
    {
      if(observer_.getContactIsSetByNum(i))
      {
        contactsPosAverageStateCov_ += 1 / pow(double(observer_.getNumberOfSetContacts()), 2)
                                       * (observer_.getStateCovarianceMat().block(
                                           observer_.contactIndexTangent(i), observer_.contactIndexTangent(i), 3, 3));

        for(unsigned j = 0; j < maxContacts_; j++)
        {
          if(i != j && observer_.getContactIsSetByNum(j))
          {
            contactsPosAverageStateCov_ +=
                1 / pow(double(observer_.getNumberOfSetContacts()), 2)
                * (observer_.getStateCovarianceMat().block(observer_.contactIndexTangent(i),
                                                           observer_.contactIndexTangent(j), 3, 3));
          }
        }
      }
    }
  }

  /* Update of the visual representation (only a visual feature) of the observed robot */
  my_robots_->robot().mbc().q = ctl.realRobot(robot_).mbc().q;

  /* Update of the observed robot */
  update(my_robots_->robot());

  return true;
} // namespace mc_state_observation

///////////////////////////////////////////////////////////////////////
/// -------------------------Called functions--------------------------
///////////////////////////////////////////////////////////////////////

void MCKineticsObserver::initObserverStateVector(const mc_control::MCController & ctl, const mc_rbdyn::Robot & robot)
{
  so::kine::Orientation initOrientation(so::Matrix3(ctl.realRobot(robot_).posW().rotation().transpose()));

  Eigen::VectorXd initStateVector;
  initStateVector = Eigen::VectorXd::Zero(observer_.getStateSize());

  initStateVector.segment(observer_.posIndex(), observer_.sizePos) =
      initOrientation.toMatrix3().transpose() * robot.com();
  initStateVector.segment(observer_.oriIndex(), observer_.sizeOri) = initOrientation.toVector4();
  initStateVector.segment(observer_.linVelIndex(), observer_.sizeLinVel) =
      initOrientation.toMatrix3().transpose() * robot.comVelocity();
  initStateVector.segment(observer_.angVelIndex(), observer_.sizeAngVel) =
      initOrientation.toMatrix3().transpose() * robot.velW().angular();

  observer_.setInitWorldCentroidStateVector(initStateVector);
  initialStateVector_ = initStateVector;
}

void MCKineticsObserver::update(mc_control::MCController & ctl) // this function is called by the pipeline if the
                                                                // update is set to true in the configuration file
{
  auto & realRobot = ctl.realRobot(robot_);
  update(realRobot);
  realRobot.forwardKinematics();
  realRobot.forwardVelocity();
}

// used only to update the visual representation of the estimated robot
void MCKineticsObserver::update(mc_rbdyn::Robot & robot)
{
  robot.posW(X_0_fb_);
  robot.velW(v_fb_0_.vector());
}

void MCKineticsObserver::inputAdditionalWrench(const mc_rbdyn::Robot & inputRobot, const mc_rbdyn::Robot & measRobot)
{
  additionalUserResultingForce_.setZero();
  additionalUserResultingMoment_.setZero();

  for(const auto & forceSensor : measRobot.forceSensors())
  {
    if(ignoredSensorWrenches_.count(forceSensor.name()) || pinContacts_) { continue; }
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
      additionalUserResultingForce_ += measuredWrench.force();
      additionalUserResultingMoment_ += measuredWrench.moment();
    }
  }

  if(legKinematicsContacts_ && !legKinematicsWrenchCorrection_ && !legKinematicsWrenchState_)
  {
    std::unordered_map<int, so::Vector6> currentWrenches;
    for(const auto & [_, contact] : contactsManager_.contacts())
    {
      if(!contact.isSet() || ignoredSensorWrenches_.count(contact.fsName())) { continue; }
      currentWrenches[contact.id()] = contact.contactWrenchVector_;
      so::Vector6 wrench = contact.contactWrenchVector_;
      if(legKinematicsWrenchDelay_)
      {
        const auto previous = previousContactWrenches_.find(contact.id());
        wrench = previous == previousContactWrenches_.end() ? so::Vector6::Zero() : previous->second;
      }
      const so::Matrix3 fbContactOri = contact.fbContactKine_.orientation.toMatrix3();
      const so::Vector3 force = fbContactOri * wrench.segment<3>(0);
      additionalUserResultingForce_ += force;
      additionalUserResultingMoment_ += fbContactOri * wrench.segment<3>(3) + contact.fbContactKine_.position().cross(force);
    }
    previousContactWrenches_ = std::move(currentWrenches);
  }

  // We pass this computed wrench as an input to the Kinetics Observer
  observer_.setAdditionalWrench(additionalUserResultingForce_, additionalUserResultingMoment_);

  // Both loops feed the debug_wrenchesInCentroid_* family, which the pipeline keeps: the
  // disturbance-wrench table of the paper is built from the hand sensor's entry, and that sensor
  // is precisely an IGNORED one in the hidehand variant. Computing these only under
  // withDebugLogs_ would leave those channels frozen, so they are refreshed unconditionally.
  for(auto & contactWithSensor : contactsManager_.contacts())
  {
    KoContactWithSensor & contact = contactWithSensor.second;
    const mc_rbdyn::ForceSensor & fs = measRobot.forceSensor(contact.fsName_);
    so::Vector3 forceCentroid = so::Vector3::Zero();
    so::Vector3 torqueCentroid = so::Vector3::Zero();

    const sva::ForceVecd measuredWrench = wrenchInFloatingBaseFrame(fs, inputRobot);

    observer_.convertWrenchFromUserToCentroid(measuredWrench.force(), measuredWrench.moment(), forceCentroid,
                                              torqueCentroid);

    contact.wrenchInCentroid_.segment<3>(0) = forceCentroid;
    contact.wrenchInCentroid_.segment<3>(3) = torqueCentroid;
  }
  for(auto & [sensor, value] : ignoredSensorWrenches_)
  {
    const auto measured = wrenchInFloatingBaseFrame(measRobot.forceSensor(sensor), inputRobot);
    so::Vector3 force, torque;
    observer_.convertWrenchFromUserToCentroid(measured.force(), measured.moment(), force, torque);
    value.head<3>() = force;
    value.tail<3>() = torque;
  }
}

sva::ForceVecd MCKineticsObserver::wrenchWithoutGravity(const mc_rbdyn::ForceSensor & forceSensor,
                                                        const sva::PTransformd & X_fb_parent,
                                                        const Eigen::Matrix3d & R_fb_world) const
{
  const auto & calibration = forceSensor.calib();

  // Reproduce ForceSensorCalibData::wfToSensor, but express gravity in the estimated floating-base frame.
  const sva::PTransformd X_fb_ds = calibration.X_f_ds * forceSensor.X_p_f() * X_fb_parent;
  const sva::PTransformd X_fb_vb(X_fb_ds.inv().rotation(),
                                 (calibration.X_p_vb * X_fb_parent * X_fb_ds.inv()).translation());

  sva::ForceVecd gravityWrench = calibration.worldForce;
  gravityWrench.force() = R_fb_world * calibration.worldForce.force();
  gravityWrench.moment() = R_fb_world * calibration.worldForce.moment();

  return forceSensor.wrench() - calibration.offset - X_fb_vb.transMul(gravityWrench);
}

sva::ForceVecd MCKineticsObserver::wrenchInFloatingBaseFrame(const mc_rbdyn::ForceSensor & forceSensor,
                                                             const mc_rbdyn::Robot & inputRobot) const
{
  const unsigned parentIndex = inputRobot.bodyIndexByName(forceSensor.parentBody());
  const auto & X_fb_parent = inputRobot.mbc().bodyPosW[parentIndex];

  // X_0_fb_ is an X_world_fb transform: its rotation maps world vectors into the estimated FB frame.
  const sva::ForceVecd gravityFreeWrench = wrenchWithoutGravity(forceSensor, X_fb_parent, X_0_fb_.rotation());

  // Transport from the calibrated actual sensor frame to the floating-base frame.
  const sva::PTransformd X_parent_fb = X_fb_parent.inv();
  const sva::PTransformd X_sensor_fb = X_parent_fb * forceSensor.X_fsactual_parent();
  return X_sensor_fb.dualMul(gravityFreeWrench);
}

void MCKineticsObserver::updateIMUs(const mc_rbdyn::Robot & measRobot, const mc_rbdyn::Robot & inputRobot)
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

    so::kine::Kinematics worldImuKine = fbBodyKine * bodyImuKine;
    imuInputKinematics_[i] = worldImuKine;

    observer_.setIMU(imu.linearAcceleration(), imu.angularVelocity(), acceleroSensorCovariance_, gyroSensorCovariance_,
                     worldImuKine, so::Index(i));
  }
}

const so::kine::Kinematics MCKineticsObserver::getContactWorldKinematics(const mc_control::MCController & ctl,
                                                                         KoContactWithSensor & contact,
                                                                         const mc_rbdyn::Robot & currentRobot,
                                                                         bool withVel)
{
  /*
  Can be used with inputRobot, a virtual robot corresponding to the real robot whose floating base's frame is
  superimposed with the world frame. Getting kinematics associated to the inputRobot inside the world frame is the same
  as getting the same kinematics of the real robot inside the frame of its floating base, which is needed for the inputs
  of the Kinetics Observer. This allows to use the basic mc_rtc functions directly giving kinematics in the world frame
  and not do the conversion: initial frame -> world + world -> floating base as the latter is zero.
  */

  so::kine::Kinematics worldContactKine;
  so::kine::Kinematics worldFbKine;
  if(withVel) { worldFbKine = conversions::kinematics::fromSva(currentRobot.posW(), currentRobot.velW(), true); }
  else
  {
    worldFbKine = conversions::kinematics::fromSva(currentRobot.posW(), so::kine::Kinematics::Flags::pose);
  }

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
  else // the kinematics of the contacts are the ones of the surface.
  {
    // the kinematics of the contacts are the ones of the surface, but we must transport the measured wrench
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

const so::kine::Kinematics MCKineticsObserver::getCtlContactWorldKinematics(const mc_control::MCController & ctl,
                                                                            KoContactWithSensor & contact,
                                                                            bool withVel)
{
  /*
  Can be used with inputRobot, a virtual robot corresponding to the real robot whose floating base's frame is
  superimposed with the world frame. Getting kinematics associated to the inputRobot inside the world frame is the same
  as getting the same kinematics of the real robot inside the frame of its floating base, which is needed for the inputs
  of the Kinetics Observer. This allows to use the basic mc_rtc functions directly giving kinematics in the world frame
  and not do the conversion: initial frame -> world + world -> floating base as the latter is zero.
  */
  const auto & robot = ctl.robot(robot_);
  so::kine::Kinematics worldContactKine;
  so::kine::Kinematics worldFbKine;
  if(withVel) { worldFbKine = conversions::kinematics::fromSva(robot.posW(), robot.velW(), true); }
  else
  {
    worldFbKine = conversions::kinematics::fromSva(robot.posW(), so::kine::Kinematics::Flags::pose);
  }

  if(contactsDetector_.getContactsDetection() == KoContactsDetector::ContactsDetection::Sensors)
  {
    return getFsWorldKinematics(ctl, robot, contact.fsName());
  }
  else // the kinematics of the contacts are the ones of the surface.
  {
    // the kinematics of the contacts are the ones of the surface, but we must transport the measured wrench
    const mc_rbdyn::Surface & contactSurface = robot.surface(contact.surfaceName());

    const sva::PTransformd & bodyContactPose = contactSurface.X_b_s();
    unsigned bodyIndex = robot.bodyIndexByName(contactSurface.bodyName());

    so::kine::Kinematics bodyContactKine;
    so::kine::Kinematics worldBodyKine;

    if(withVel)
    {
      bodyContactKine = conversions::kinematics::fromSva(bodyContactPose, so::kine::Kinematics::Flags::vel);
      worldBodyKine =
          conversions::kinematics::fromSva(robot.mbc().bodyPosW[bodyIndex], robot.mbc().bodyVelW[bodyIndex], true);
    }
    else
    {
      bodyContactKine = conversions::kinematics::fromSva(bodyContactPose, so::kine::Kinematics::Flags::pose);
      worldBodyKine =
          conversions::kinematics::fromSva(robot.mbc().bodyPosW[bodyIndex], so::kine::Kinematics::Flags::pose);
    }

    worldContactKine = worldBodyKine * bodyContactKine;
  }

  return worldContactKine;
}

const so::kine::Kinematics MCKineticsObserver::getFsWorldKinematics(const mc_control::MCController & ctl,
                                                                    const mc_rbdyn::Robot & currentRobot,
                                                                    const std::string & fsName)
{
  /*
  Can be used with inputRobot, a virtual robot corresponding to the real robot whose floating base's frame is
  superimposed with the world frame. Getting kinematics associated to the inputRobot inside the world frame is the same
  as getting the same kinematics of the real robot inside the frame of its floating base, which is needed for the inputs
  of the Kinetics Observer. This allows to use the basic mc_rtc functions directly giving kinematics in the world frame
  and not do the conversion: initial frame -> world + world -> floating base as the latter is zero.
  */

  const mc_rbdyn::ForceSensor & fs = ctl.robot(robot_).forceSensor(fsName);

  so::kine::Kinematics worldFsKine;
  const so::kine::Kinematics worldFbKine =
      conversions::kinematics::fromSva(currentRobot.posW(), currentRobot.velW(), true);

  // Use the calibrated actual sensor pose, not the nominal model sensor pose.
  // X_fsactual_parent() goes from the sensor to the parent body, we need the opposite direction here.
  const sva::PTransformd bodyFsPose = fs.X_fsactual_parent().inv();
  unsigned bodyIndex = currentRobot.bodyIndexByName(fs.parentBody());

  so::kine::Kinematics bodyFsKine = conversions::kinematics::fromSva(bodyFsPose, so::kine::Kinematics::Flags::vel);

  so::kine::Kinematics worldBodyKine = conversions::kinematics::fromSva(currentRobot.mbc().bodyPosW[bodyIndex],
                                                                        currentRobot.mbc().bodyVelW[bodyIndex], true);

  worldFsKine = worldBodyKine * bodyFsKine;

  return worldFsKine;
}

const so::kine::Kinematics MCKineticsObserver::getContactFsKinematics(const mc_control::MCController & ctl,
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

void MCKineticsObserver::updateContactForceMeasurement(KoContactWithSensor & contact,
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
    calibrationRotation =
        so::Matrix3(Eigen::AngleAxisd(calibration->second.norm(), calibration->second.normalized()));
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

so::Matrix6 MCKineticsObserver::legKinematicsCovariance(const mc_control::MCController & ctl,
                                                        const KoContactWithSensor & contact) const
{
  const auto & robot = ctl.realRobot(robot_);
  const auto & surface = robot.surface(contact.surfaceName());
  rbd::Jacobian jacobian(robot.mb(), surface.bodyName(), surface.X_b_s().translation());
  Eigen::MatrixXd J = jacobian.jacobian(robot.mb(), robot.mbc());
  // The RI-EKF plugin also puts the joint noise on the free-flyer columns; without them only the encoders count.
  if(!legKinematicsFloatingBaseNoise_ && jacobian.jointsPath().front() == 0 && robot.mb().joint(0).dof() == 6)
  {
    J.leftCols(6).setZero();
  }
  const Eigen::MatrixXd worldCov = legKinematicsJointVariance_ * J * J.transpose();
  // rbd orders the rows angular then linear; the observer measures position then orientation, in the base frame.
  const so::Matrix3 E = robot.posW().rotation();
  so::Matrix6 cov;
  cov.block<3, 3>(0, 0) = E * worldCov.block<3, 3>(3, 3) * E.transpose();
  cov.block<3, 3>(3, 3) = E * worldCov.block<3, 3>(0, 0) * E.transpose();
  cov.block<3, 3>(0, 3) = E * worldCov.block<3, 3>(3, 0) * E.transpose();
  cov.block<3, 3>(3, 0) = cov.block<3, 3>(0, 3).transpose();
  if(legKinematicsCompliant_)
  {
    const so::Matrix3 R = contact.fbContactKine_.orientation.toMatrix3();
    // With legKinematicsCompliantProcess the deflection carries the Kinetics Observer's own tolerance on the
    // contact wrench: its process covariance on top of the sensor's.
    so::Matrix6 wrenchCov = contactWrenchCovariance(contact);
    if(legKinematicsCompliantProcess_) { wrenchCov += contactProcessCovariance_.block<6, 6>(6, 6); }
    const so::Matrix3 linCompliance = linStiffness_.inverse();
    const so::Matrix3 angCompliance = angStiffness_.inverse();
    cov.block<3, 3>(0, 0) += R * linCompliance * wrenchCov.block<3, 3>(0, 0) * linCompliance.transpose() * R.transpose();
    cov.block<3, 3>(3, 3) += R * angCompliance * wrenchCov.block<3, 3>(3, 3) * angCompliance.transpose() * R.transpose();
  }
  if(legKinematicsPositionOnly_)
  {
    cov.block<3, 3>(3, 3) = so::Matrix3::Identity() * 9e90;
    cov.block<3, 3>(0, 3).setZero();
    cov.block<3, 3>(3, 0).setZero();
  }
  return cov;
}

so::kine::Kinematics MCKineticsObserver::compliantRestKine(const KoContactWithSensor & contact) const
{
  const so::Matrix3 R = contact.fbContactKine_.orientation.toMatrix3();
  so::kine::Kinematics rest = contact.fbContactKine_;
  rest.position = contact.fbContactKine_.position() + R * linStiffness_.inverse() * contact.contactWrenchVector_.segment<3>(0);

  // same orientation deflection as getOdometryWorldContactRest, the contact being assumed at rest
  const so::Vector3 flexRotDiff = -2 * angStiffness_.inverse() * contact.contactWrenchVector_.segment<3>(3);
  so::Matrix3 flexRotMatrix = so::Matrix3::Identity();
  if(flexRotDiff.norm() > so::cst::epsilonAngle)
  {
    const double flexRotAngle = std::asin(std::min(1.0, flexRotDiff.norm() / 2.0));
    flexRotMatrix = so::kine::Orientation(Eigen::AngleAxisd(flexRotAngle, flexRotDiff.normalized())).toMatrix3();
  }
  rest.orientation = so::Matrix3(R * flexRotMatrix.transpose());
  return rest;
}

so::Matrix6 MCKineticsObserver::contactWrenchCovariance(const KoContactWithSensor & contact) const
{
  if(contactsDetector_.getContactsDetection() == KoContactsDetector::ContactsDetection::Sensors)
  {
    return contactSensorCovariance_;
  }

  const so::Matrix3 & sensorContactOri = contact.contactSensorKine_.orientation.toMatrix3();
  so::Matrix6 sensorContactWrenchTransform = so::Matrix6::Zero();
  sensorContactWrenchTransform.block<3, 3>(0, 0) = sensorContactOri;
  sensorContactWrenchTransform.block<3, 3>(3, 0) =
      so::kine::skewSymmetric(contact.contactSensorKine_.position()) * sensorContactOri;
  sensorContactWrenchTransform.block<3, 3>(3, 3) = sensorContactOri;

  return sensorContactWrenchTransform * contactSensorCovariance_ * sensorContactWrenchTransform.transpose();
}

so::kine::Kinematics MCKineticsObserver::getOdometryWorldContactRest(KoContactWithSensor & contact,
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

  // we get the reference position of the contact by removing the contribution of the visco-elastic model
  worldRestPose.position =
      worldContactKine.position()
      + worldContactKine.orientation.toMatrix3() * linStiffness_.inverse()
            * (contactForceMeas
               + linDamping_ * worldContactKine.orientation.toMatrix3().transpose() * worldContactKine.linVel());

  /* We get the reference orientation of the contact by removing the contribution of the visco-elastic model */
  // difference between the reference orientation and the real one, obtained from the visco-elastic model
  so::Vector3 flexRotDiff =
      -2 * angStiffness_.inverse()
      * (contactTorqueMeas
         + angDamping_ * worldContactKine.orientation.toMatrix3().transpose() * worldContactKine.angVel());

  so::Matrix3 flexRotMatrix = so::Matrix3::Identity();

  if(flexRotDiff.norm() > so::cst::epsilonAngle)
  {
    so::Vector3 flexRotAxis = flexRotDiff / flexRotDiff.norm();
    double diffNorm = std::min(1.0, flexRotDiff.norm() / 2.0);
    double flexRotAngle = std::asin(diffNorm);

    Eigen::AngleAxisd flexRotAngleAxis(flexRotAngle, flexRotAxis);
    flexRotMatrix = so::kine::Orientation(flexRotAngleAxis).toMatrix3();
  }

  worldRestPose.orientation = so::Matrix3(worldContactKine.orientation.toMatrix3() * flexRotMatrix.transpose());

  if(odometryType_ == so::odometry::OdometryType::Flat) // if true, the position odometry is made only
                                                        // along the x and y axis, the position along z is
                                                        // assumed to be the one of the control robot
  {
    worldRestPose.position()(2) = 0.0;
  }
  return worldRestPose;
}

void MCKineticsObserver::setNewContact(const mc_control::MCController & ctl,
                                       KoContactWithSensor & contact,
                                       const so::Matrix12 & initCovariance,
                                       mc_rtc::Logger & logger)
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

  if(contactsDetector_.getContactsDetection() == KoContactsDetector::ContactsDetection::Sensors)
  {
    contact.fsName(contact.surfaceName());
  }
  else
  {
    contact.fsName(robot.indirectSurfaceForceSensor(contact.surfaceName()).name());
  }

  const sva::ForceVecd & measuredWrench = forceSensorMeasurements_.at(contact.fsName_);

  contact.fbContactKine_.reset();
  contact.contactSensorKine_.reset();

  // This function updates both contact.fbContactKine_ and contact.contactSensorKine_
  getContactFsKinematics(ctl, contact, inputRobot);
  updateContactForceMeasurement(contact, measuredWrench);

  so::kine::Kinematics worldContactKine =
      observer_.getGlobalKinematicsOf(legKinematicsContacts_ && legKinematicsCompliant_ ? compliantRestKine(contact)
                                                                                         : contact.fbContactKine_);

  // addContact mutates worldContactKine into the rest pose; keep the pre-call orientation.
  const so::Matrix3 currentContactOri = worldContactKine.orientation.toMatrix3();
  if(pinContacts_ || (legKinematicsContacts_ && !legKinematicsWrenchCorrection_)) { contact.sensorEnabled_ = false; }
  so::Matrix12 processCovariance = contactProcessCovariance_;
  if(legKinematicsWrenchState_)
  {
    processCovariance.bottomRightCorner<6, 6>() =
        legKinematicsWrenchStateNoise_ ? contactWrenchCovariance(contact) : so::Matrix6::Zero();
  }
  if(pinContacts_ || legKinematicsContacts_)
  {
    // No measured-wrench rest-pose correction or inverse of zero angular stiffness.
    if(odometryType_ == so::odometry::OdometryType::Flat) { worldContactKine.position()(2) = 0.0; }
    observer_.addContact(worldContactKine, initCovariance, processCovariance, contact.id(), linStiffness_,
                         linDamping_, angStiffness_, angDamping_);
  }
  else
  {
    observer_.addContact(worldContactKine, initCovariance, processCovariance, contact.id(), linStiffness_,
                       linDamping_, angStiffness_, angDamping_, contact.contactWrenchVector_.segment<3>(0),
                       contact.contactWrenchVector_.segment<3>(3), odometryType_ == so::odometry::OdometryType::Flat);
  }
  if(contactRestOrientationErrorDeg_ > 0.0 && logger.t() > 1e-15)
  {
    std::uniform_real_distribution<double> draw(-1.0, 1.0);
    so::Vector3 axis;
    do { for(int i = 0; i < 3; ++i) { axis(i) = draw(contactRestOrientationRng_); } }
    while(axis.squaredNorm() < 1e-12);
    const so::Matrix3 rotation(Eigen::AngleAxisd(contactRestOrientationErrorDeg_ * M_PI / 180.0, axis.normalized()));
    worldContactKine.orientation = so::Matrix3(rotation * worldContactKine.orientation.toMatrix3());
    observer_.setStateContact(contact.id(), worldContactKine, so::Vector6::Zero(), false);
  }
  if(contactWrenchInitFromMeasurement_) { observer_.setStateContactWrench(contact.id(), contact.contactWrenchVector_); }
  contact.initKine_ = worldContactKine;

  // Rotation from the rest orientation just stored to the actual contact orientation.
  {
    const so::Matrix3 oriDiff = worldContactKine.orientation.toMatrix3().transpose() * currentContactOri;
    const Eigen::AngleAxisd aa(oriDiff);
    contact.initRestOriDiff_ = aa.angle() * aa.axis();
    contact.initRestOriAngleDeg_ = std::abs(aa.angle()) * 180.0 / M_PI;
  }

  // checks if the sensor is used in the correction of the Kinetics Observer or not
  if(contact.sensorEnabled_)
  {
    // we update the measurements of the sensor and the input kinematics of the contact in the user /
    // floating base's frame
    if(legKinematicsContacts_)
    {
      observer_.updateContactWithWrenchAndKinematicSensors(contact.contactWrenchVector_,
                                                           contactWrenchCovariance(contact), contact.fbContactKine_,
                                                           legKinematicsCovariance(ctl, contact), contact.id());
    }
    else
    {
      observer_.updateContactWithWrenchSensor(contact.contactWrenchVector_, contactWrenchCovariance(contact),
                                              contact.fbContactKine_, contact.id());
    }
  }
  else
  {
    // we update the input kinematics of the contact in the user / floating base's frame
    if(legKinematicsContacts_)
    {
      if(legKinematicsCompliant_)
      {
        if(legKinematicsDeflection_)
        {
          observer_.updateContactWithKinematicSensor(contact.fbContactKine_, legKinematicsCovariance(ctl, contact),
                                                     contact.contactWrenchVector_, contact.id());
        }
        else
        {
          observer_.updateContactWithKinematicSensor(compliantRestKine(contact),
                                                     legKinematicsCovariance(ctl, contact), contact.id());
        }
      }
      else
      {
        observer_.updateContactWithKinematicSensor(contact.fbContactKine_, legKinematicsCovariance(ctl, contact),
                                                   contact.id());
      }
    }
    else
    {
      observer_.updateContactWithNoSensor(contact.fbContactKine_, contact.id());
    }
  }

  // The contact channels the rest of the chain consumes -- debug_contactKine_*,
  // MEKF_estimatedState_contact_* and debug_contactState_isSet_*, the last one named explicitly in
  // observersInfos.yaml -- are registered here whatever withDebugLogs_ says; addContactLogEntries
  // guards the families nothing reads. The measurement entries stay fully behind the flag.
  addContactLogEntries(ctl, logger, contact);
  if(contact.sensorEnabled_) { addContactMeasurementsLogEntries(logger, contact); }
}

void MCKineticsObserver::updateContact(const mc_control::MCController & ctl, KoContactWithSensor & contact)
{
  /*
  Uses the inputRobot, a virtual robot corresponding to the real robot whose floating base's frame is superimposed with
  the world frame. Getting kinematics associated to the inputRobot inside the world frame is the same as getting the
  same kinematics of the real robot inside the frame of its floating base, which is needed for the inputs of the
  Kinetics Observer. This allows to use the basic mc_rtc functions directly giving kinematics in the world frame and not
  do the conversion: initial frame -> world + world -> floating base as the latter is zero.
  */
  auto & inputRobot = my_robots_->robot("inputRobot");

  const sva::ForceVecd & measuredWrench = forceSensorMeasurements_.at(contact.fsName_);

  contact.fbContactKine_.reset();
  contact.contactSensorKine_.reset();

  // This function updates both contact.fbContactKine_ and contact.contactSensorKine_
  getContactFsKinematics(ctl, contact, inputRobot);
  updateContactForceMeasurement(contact, measuredWrench);

  if(contact.sensorEnabled_) // the force sensor attached to the contact is used in the correction by the
                             // Kinetics Observer.
  {
    if(legKinematicsContacts_)
    {
      observer_.updateContactWithWrenchAndKinematicSensors(contact.contactWrenchVector_,
                                                           contactWrenchCovariance(contact), contact.fbContactKine_,
                                                           legKinematicsCovariance(ctl, contact), contact.id());
    }
    else
    {
      observer_.updateContactWithWrenchSensor(contact.contactWrenchVector_, contactWrenchCovariance(contact),
                                              contact.fbContactKine_, contact.id());
    }
  }
  else
  {
    if(legKinematicsContacts_)
    {
      if(legKinematicsCompliant_)
      {
        if(legKinematicsDeflection_)
        {
          observer_.updateContactWithKinematicSensor(contact.fbContactKine_, legKinematicsCovariance(ctl, contact),
                                                     contact.contactWrenchVector_, contact.id());
        }
        else
        {
          observer_.updateContactWithKinematicSensor(compliantRestKine(contact),
                                                     legKinematicsCovariance(ctl, contact), contact.id());
        }
      }
      else
      {
        observer_.updateContactWithKinematicSensor(contact.fbContactKine_, legKinematicsCovariance(ctl, contact),
                                                   contact.id());
      }
    }
    else
    {
      observer_.updateContactWithNoSensor(contact.fbContactKine_, contact.id());
    }
  }
}

void MCKineticsObserver::updateContacts(const mc_control::MCController & ctl, mc_rtc::Logger & logger)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  for(const auto & forceSensor : robot.forceSensors())
  {
    forceSensorMeasurements_.at(forceSensor.name()) = forceSensor.wrenchWithoutGravity(realRobot);
  }

  so::Matrix12 initCovariance;
  if(observer_.getNumberOfSetContacts() > 0) // The initial covariance on the pose of the contact depending on
                                             // whether another contact is already set or not
  {
    initCovariance = contactInitCovarianceNewContacts_;
  }
  else
  {
    initCovariance = contactInitCovarianceFirstContacts_;
  }

  if(odometryType_ == so::odometry::OdometryType::Flat) { initCovariance(2, 2) = 0.0; }

  auto onNewContact = [this, &ctl, &logger, &initCovariance](KoContactWithSensor & newContact)
  { setNewContact(ctl, newContact, initCovariance, logger); };
  auto onMaintainedContact = [this, &ctl](KoContactWithSensor & maintainedContact)
  { updateContact(ctl, maintainedContact); };
  auto onRemovedContact = [this, &logger](KoContactWithSensor & removedContact)
  {
    observer_.removeContact(removedContact.id());

    removeContactLogEntries(logger, removedContact);
  };

  // Action to execute once when a contact is first added to the manager.
  auto onAddedContact = [this, &ctl, &logger](KoContactWithSensor & addedContact)
  { addContactToGui(ctl, addedContact, logger); };

  std::unordered_set<std::string> & contactList = contactsDetector_.updateContacts(ctl, robot_);
  for(auto it = contactList.begin(); it != contactList.end();)
  {
    const auto sensor = contactsDetector_.getContactsDetection() == KoContactsDetector::ContactsDetection::Sensors
                            ? *it : robot.indirectSurfaceForceSensor(*it).name();
    if(ignoredSensorWrenches_.count(sensor)) { it = contactList.erase(it); }
    else { ++it; }
  }
  contactsManager_.updateContacts(contactList, onNewContact, onMaintainedContact, onRemovedContact, onAddedContact);
}

void MCKineticsObserver::mass(double mass)
{
  mass_ = mass;
  observer_.setMass(mass);
}

///////////////////////////////////////////////////////////////////////
/// -------------------------------Logs--------------------------------
///////////////////////////////////////////////////////////////////////

void MCKineticsObserver::addToLogger(const mc_control::MCController & ctl,
                                     mc_rtc::Logger & logger,
                                     const std::string & category)
{
  category_ = category;
  logger.addLogEntry(category_ + "_constants_mass", [this]() -> double { return observer_.getMass(); });
  // Not guarded: this is the source of the paper's disturbance-wrench table, which compares the
  // estimated wrench against what the hidden hand sensor measured.
  for(const auto & [sensor, value] : ignoredSensorWrenches_)
  {
    logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_" + sensor + "_force",
                      [this, sensor]() -> so::Vector3 { return ignoredSensorWrenches_.at(sensor).head<3>(); });
    logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_" + sensor + "_torque",
                      [this, sensor]() -> so::Vector3 { return ignoredSensorWrenches_.at(sensor).tail<3>(); });
  }

  logger.addLogEntry(category_ + "_mcko_fb_posW", [this]() -> sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category_ + "_mcko_fb_velW", [this]() -> sva::MotionVecd & { return v_fb_0_; });
  logger.addLogEntry(category_ + "_mcko_fb_accW", [this]() -> sva::MotionVecd & { return a_fb_0_; });

  logger.addLogEntry(category_ + "_mcko_fb_yaw",
                     [this]() -> double { return -so::kine::rotationMatrixToYawAxisAgnostic(X_0_fb_.rotation()); });

  /* Plots of the updated state */
  conversions::kinematics::addToLogger(logger, globalCentroidKinematics_, category_ + "_MEKF_estimatedState");
  logger.addLogEntry(category_ + "_MEKF_initialState", [this]() -> const so::Vector & { return initialStateVector_; });
  for(size_t i = 0; i < listIMUs_.size(); ++i)
  {
    conversions::kinematics::addToLogger(logger, imuInputKinematics_[i],
                                         category_ + "_MEKF_inputs_imu_" + listIMUs_[i].name());
  }
  for(const auto & [sensorName, measurement] : forceSensorMeasurements_)
  {
    const std::string prefix = category_ + "_debug_forceSensor_" + sensorName;
    logger.addLogEntry(prefix + "_measuredForce", [this, sensorName]() -> Eigen::Vector3d
                       { return forceSensorMeasurements_.at(sensorName).force(); });
    logger.addLogEntry(prefix + "_measuredTorque", [this, sensorName]() -> Eigen::Vector3d
                       { return forceSensorMeasurements_.at(sensorName).moment(); });
  }
  for(auto & imu : listIMUs_)
  {
    logger.addLogEntry(category_ + "_MEKF_estimatedState_gyroBias_" + imu.name(),
                       [this, &imu]() -> Eigen::Vector3d
                       {
                         return observer_.getCurrentStateVector().segment(observer_.gyroBiasIndex(imu.id()),
                                                                          observer_.sizeGyroBias);
                       });
  }
  logger.addLogEntry(
      category_ + "_MEKF_estimatedState_extForceCentr", [this]() -> Eigen::Vector3d
      { return observer_.getCurrentStateVector().segment(observer_.unmodeledForceIndex(), observer_.sizeForce); });

  logger.addLogEntry(
      category_ + "_MEKF_estimatedState_extTorqueCentr", [this]() -> Eigen::Vector3d
      { return observer_.getCurrentStateVector().segment(observer_.unmodeledTorqueIndex(), observer_.sizeTorque); });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_unbiasedExtForce",
                     [this]() -> Eigen::Vector3d { return getUnbiasedEstimatedDisturbanceWrench().force(); });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_unbiasedExtMoment",
                     [this]() -> Eigen::Vector3d { return getUnbiasedEstimatedDisturbanceWrench().moment(); });

  /* Plots of the inputs */

  logger.addLogEntry(category_ + "_MEKF_inputs_angularMomentum",
                     [this]() -> Eigen::Vector3d { return observer_.getAngularMomentum()(); });
  logger.addLogEntry(category_ + "_MEKF_inputs_angularMomentumDot",
                     [this]() -> Eigen::Vector3d { return observer_.getAngularMomentumDot()(); });
  logger.addLogEntry(category_ + "_MEKF_inputs_com",
                     [this]() -> Eigen::Vector3d { return observer_.getCenterOfMass()(); });
  logger.addLogEntry(category_ + "_MEKF_inputs_comDot",
                     [this]() -> Eigen::Vector3d { return observer_.getCenterOfMassDot()(); });
  logger.addLogEntry(category_ + "_MEKF_inputs_comDotDot",
                     [this]() -> Eigen::Vector3d { return observer_.getCenterOfMassDotDot()(); });
  logger.addLogEntry(category_ + "_MEKF_inputs_inertiaMatrix",
                     [this]() -> Eigen::Vector6d
                     {
                       so::Vector6 inertia;
                       inertia.segment<3>(0) = observer_.getInertiaMatrix()().diagonal();
                       inertia.segment<2>(3) = observer_.getInertiaMatrix()().block<1, 2>(0, 1);
                       inertia(5) = observer_.getInertiaMatrix()()(1, 2);
                       return inertia;
                     });

  logger.addLogEntry(category_ + "_MEKF_inputs_inertiaMatrixDot",
                     [this]() -> Eigen::Vector6d
                     {
                       so::Vector6 inertiaDot;
                       inertiaDot.segment<3>(0) = observer_.getInertiaMatrixDot()().diagonal();
                       inertiaDot.segment<2>(3) = observer_.getInertiaMatrixDot()().block<1, 2>(0, 1);
                       inertiaDot(5) = observer_.getInertiaMatrixDot()()(1, 2);
                       return inertiaDot;
                     });

  /* Inputs */
  logger.addLogEntry(category_ + "_MEKF_inputs_additionalWrench_Force", [this]() -> Eigen::Vector3d
                     { return observer_.getAdditionalWrench().segment(0, observer_.sizeForce); });
  logger.addLogEntry(
      category_ + "_MEKF_inputs_additionalWrench_Torque", [this]() -> Eigen::Vector3d
      { return observer_.getAdditionalWrench().segment(observer_.sizeForce, observer_.sizeTorque); });

  for(auto & imu : listIMUs_)
  {
    logger.addLogEntry(category_ + "_MEKF_measurements_gyro_" + imu.name() + "_measured",
                       [this, &imu]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getLastMeasurement().segment(
                             observer_.getIMUMeasIndexByNum(imu.id()) + observer_.sizeAcceleroSignal,
                             observer_.sizeGyroBias);
                       });
    logger.addLogEntry(category_ + "_MEKF_measurements_accelerometer_" + imu.name() + "_measured",
                       [this, &imu]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getLastMeasurement().segment(
                             observer_.getIMUMeasIndexByNum(imu.id()), observer_.sizeAcceleroSignal);
                       });
  }

  if(withDebugLogs_)
  {
    conversions::kinematics::addToLogger(logger, worldFbKine_, category_ + "_debug_fbFromAnchor_fbKine");
    logger.addLogEntry(category_ + "_debug_fbFromAnchor_worldAnchorPos",
                       [this]() -> so::Vector3 & { return worldAnchorPos_; });
    logger.addLogEntry(category_ + "_debug_fbFromAnchor_fbAnchorPos",
                       [this]() -> so::Vector3 & { return fbAnchorPos_; });


    logger.addLogEntry(category_ + "_debug_disturbanceWrenchBias_force",
                       [this]() -> Eigen::Vector3d & { return disturbanceWrenchOffset_.force(); });
    logger.addLogEntry(category_ + "_debug_disturbanceWrenchBias_moment",
                       [this]() -> Eigen::Vector3d & { return disturbanceWrenchOffset_.moment(); });

    valinor_.addToLogger(ctl, logger, category + "_" + valinor_.name());
    logger.addLogEntry(category_ + "_debug_estimationState", [this]() -> std::string { return estimationStateName(); });
    logger.addLogEntry(category_ + "_debug_config_OdometryType",
                       [this]() -> std::string { return so::odometry::odometryTypeToString(odometryType_); });

    logger.addLogEntry(category_ + "_debug_config_withAdaptativeContactProcessCov", [this]() -> std::string
                       { return observer_.getWithAdaptativeContactProcessCov() ? "True" : "False"; });

    for(auto & imu : listIMUs_)
    {
      logger.addLogEntry(category_ + "_MEKF_stateCovariances_gyroBias_" + imu.name(),
                         [this, &imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF()
                               .getStateCovariance()
                               .block(observer_.gyroBiasIndexTangent(imu.id()),
                                      observer_.gyroBiasIndexTangent(imu.id()), observer_.sizeGyroBiasTangent,
                                      observer_.sizeGyroBiasTangent)
                               .diagonal();
                         });
      // Which measurement block corrects the gyrometer bias. The innovation is K.(y - y^) summed over
      // every measurement, so it cannot say whether a bias correction came from the IMU or from the
      // contact wrenches; these entries split it, block by block, with the same product restricted to
      // the columns of that block. Needed to explain why a loose bias initial variance lets model
      // inconsistencies leak into the bias state.
      auto biasFromBlock = [this](so::Index imuId, so::Index measIndex, so::Index measSize) -> Eigen::Vector3d
      {
        const auto & gain = observer_.getEKF().getLastGain();
        const so::Index row = observer_.gyroBiasIndexTangent(imuId);
        if(gain.rows() < row + observer_.sizeGyroBiasTangent || gain.cols() < measIndex + measSize)
        {
          return Eigen::Vector3d::Zero();
        }
        const Eigen::VectorXd residual =
            observer_.getEKF().getLastMeasurement() - observer_.getEKF().getLastPredictedMeasurement();
        return gain.block(row, measIndex, observer_.sizeGyroBiasTangent, measSize) * residual.segment(measIndex, measSize);
      };
      logger.addLogEntry(category_ + "_MEKF_innovationFrom_accelerometer_" + imu.name(),
                         [this, &imu, biasFromBlock]() -> Eigen::Vector3d {
                           return biasFromBlock(imu.id(), observer_.getIMUMeasIndexByNum(imu.id()),
                                                observer_.sizeAcceleroSignal);
                         });
      // Same decomposition for the disturbance wrench, the state meant to absorb model errors: it says
      // whether the slack variable takes the accelerometer inconsistency, or leaves it to the bias.
      auto stateFromBlock = [this](so::Index row, so::Index rowSize, so::Index measIndex,
                                   so::Index measSize) -> Eigen::Vector3d
      {
        const auto & gain = observer_.getEKF().getLastGain();
        if(gain.rows() < row + rowSize || gain.cols() < measIndex + measSize) { return Eigen::Vector3d::Zero(); }
        const Eigen::VectorXd residual =
            observer_.getEKF().getLastMeasurement() - observer_.getEKF().getLastPredictedMeasurement();
        return gain.block(row, measIndex, rowSize, measSize) * residual.segment(measIndex, measSize);
      };
      logger.addLogEntry(category_ + "_MEKF_wrenchFrom_accelerometer_force_" + imu.name(),
                         [this, &imu, stateFromBlock]() -> Eigen::Vector3d {
                           return stateFromBlock(observer_.unmodeledForceIndexTangent(), observer_.sizeForceTangent,
                                                 observer_.getIMUMeasIndexByNum(imu.id()), observer_.sizeAcceleroSignal);
                         });
      logger.addLogEntry(category_ + "_MEKF_wrenchFrom_accelerometer_torque_" + imu.name(),
                         [this, &imu, stateFromBlock]() -> Eigen::Vector3d {
                           return stateFromBlock(observer_.unmodeledTorqueIndexTangent(), observer_.sizeTorqueTangent,
                                                 observer_.getIMUMeasIndexByNum(imu.id()), observer_.sizeAcceleroSignal);
                         });
      // Where the ACCELEROMETER residual actually goes, state block by state block. A block that is
      // wrong but pinned by a tiny covariance cannot take its share, and the correction lands on
      // whichever block is comparatively free -- here the gyrometer bias. These entries name them.
      for(const auto & [name, row, size] : std::vector<std::tuple<std::string, so::Index, so::Index>>{
              {"position", observer_.posIndexTangent(), observer_.sizePosTangent},
              {"orientation", observer_.oriIndexTangent(), observer_.sizeOriTangent},
              {"linVel", observer_.linVelIndexTangent(), observer_.sizeLinVelTangent},
              {"angVel", observer_.angVelIndexTangent(), observer_.sizeAngVelTangent}})
      {
        logger.addLogEntry(category_ + "_MEKF_accelTo_" + name + "_" + imu.name(),
                           [this, &imu, stateFromBlock, row, size]() -> Eigen::Vector3d {
                             return stateFromBlock(row, size, observer_.getIMUMeasIndexByNum(imu.id()),
                                                   observer_.sizeAcceleroSignal);
                           });
      }
      // Same decomposition on the ORIENTATION rows: how much of the yaw correction comes from the
      // gyrometer, and how much from the contact measurements (added per contact below).
      logger.addLogEntry(category_ + "_MEKF_oriFrom_gyro_" + imu.name(),
                         [this, &imu, stateFromBlock]() -> Eigen::Vector3d {
                           return stateFromBlock(observer_.oriIndexTangent(), observer_.sizeOriTangent,
                                                 observer_.getIMUMeasIndexByNum(imu.id()) + observer_.sizeAcceleroSignal,
                                                 observer_.sizeGyroSignal);
                         });
      logger.addLogEntry(category_ + "_MEKF_wrenchFrom_gyro_force_" + imu.name(),
                         [this, &imu, stateFromBlock]() -> Eigen::Vector3d {
                           return stateFromBlock(observer_.unmodeledForceIndexTangent(), observer_.sizeForceTangent,
                                                 observer_.getIMUMeasIndexByNum(imu.id()) + observer_.sizeAcceleroSignal,
                                                 observer_.sizeGyroSignal);
                         });
      logger.addLogEntry(category_ + "_MEKF_innovationFrom_gyro_" + imu.name(),
                         [this, &imu, biasFromBlock]() -> Eigen::Vector3d {
                           return biasFromBlock(imu.id(),
                                                observer_.getIMUMeasIndexByNum(imu.id()) + observer_.sizeAcceleroSignal,
                                                observer_.sizeGyroSignal);
                         });
      logger.addLogEntry(
          category_ + "_MEKF_measurements_predError_vector", [this]() -> Eigen::VectorXd
          { return (observer_.getEKF().getLastMeasurement() - observer_.getEKF().getLastPredictedMeasurement()); });
      logger.addLogEntry(
          category_ + "_MEKF_measurements_predError_norm",
          [this]() -> double
          {
            return (observer_.getEKF().getLastMeasurement() - observer_.getEKF().getLastPredictedMeasurement()).norm();
          });
      logger.addLogEntry(category_ + "_MEKF_measurements_gyro_" + imu.name() + "_predicted",
                         [this, &imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastPredictedMeasurement().segment(
                               observer_.getIMUMeasIndexByNum(imu.id()) + observer_.sizeAcceleroSignal,
                               observer_.sizeGyroBias);
                         });
      logger.addLogEntry(category_ + "_MEKF_measurements_gyro_" + imu.name() + "_corrected",
                         [this, &imu]() -> Eigen::Vector3d
                         {
                           return correctedMeasurements_.segment(observer_.getIMUMeasIndexByNum(imu.id())
                                                                     + observer_.sizeAcceleroSignal,
                                                                 observer_.sizeGyroBias);
                         });

      logger.addLogEntry(category_ + "_MEKF_measurements_accelerometer_" + imu.name() + "_predicted",
                         [this, &imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastPredictedMeasurement().segment(
                               observer_.getIMUMeasIndexByNum(imu.id()), observer_.sizeAcceleroSignal);
                         });
      logger.addLogEntry(category_ + "_MEKF_measurements_accelerometer_" + imu.name() + "_corrected",
                         [this, &imu]() -> Eigen::Vector3d
                         {
                           return correctedMeasurements_.segment(observer_.getIMUMeasIndexByNum(imu.id()),
                                                                 observer_.sizeAcceleroSignal);
                         });
      logger.addLogEntry(category_ + "_MEKF_innovation_gyroBias_" + imu.name(),
                         [this, &imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getInnovation().segment(observer_.gyroBiasIndexTangent(imu.id()),
                                                                             observer_.sizeGyroBiasTangent);
                         });
      logger.addLogEntry(category_ + "_MEKF_prediction_gyroBias_" + imu.name(),
                         [this, &imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastPrediction().segment(
                               observer_.gyroBiasIndexTangent(imu.id()), observer_.sizeGyroBias);
                         });
      logger.addLogEntry(category_ + "_debug_gyroBias_" + imu.name(),
                         [&imu]() -> Eigen::Vector3d { return imu.gyroBias; });


      /* State covariances */
      logger.addLogEntry(category_ + "_MEKF_stateCovariances_contactsPosAverage_x",
                         [this]() -> double { return contactsPosAverageStateCov_(0, 0); });
      logger.addLogEntry(category_ + "_MEKF_stateCovariances_contactsPosAverage_y",
                         [this]() -> double { return contactsPosAverageStateCov_(1, 1); });
      logger.addLogEntry(category_ + "_MEKF_stateCovariances_contactsPosAverage_z",
                         [this]() -> double { return contactsPosAverageStateCov_(2, 2); });

      logger.addLogEntry(category_ + "_MEKF_stateCovariances_positionW_",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF()
                               .getStateCovariance()
                               .block(observer_.posIndexTangent(), observer_.posIndexTangent(),
                                      observer_.sizePosTangent, observer_.sizePosTangent)
                               .diagonal();
                         });
      logger.addLogEntry(category_ + "_MEKF_stateCovariances_orientationW_",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF()
                               .getStateCovariance()
                               .block(observer_.oriIndexTangent(), observer_.oriIndexTangent(),
                                      observer_.sizeOriTangent, observer_.sizeOriTangent)
                               .diagonal();
                         });
      logger.addLogEntry(category_ + "_MEKF_stateCovariances_linVelW_",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF()
                               .getStateCovariance()
                               .block(observer_.linVelIndexTangent(), observer_.linVelIndexTangent(),
                                      observer_.sizeLinVelTangent, observer_.sizeLinVelTangent)
                               .diagonal();
                         });
      logger.addLogEntry(category_ + "_MEKF_stateCovariances_angVelW_",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF()
                               .getStateCovariance()
                               .block(observer_.angVelIndexTangent(), observer_.angVelIndexTangent(),
                                      observer_.sizeAngVelTangent, observer_.sizeAngVelTangent)
                               .diagonal();
                         });

      logger.addLogEntry(category_ + "_MEKF_stateCovariances_extForce_",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF()
                               .getStateCovariance()
                               .block(observer_.unmodeledForceIndexTangent(), observer_.unmodeledForceIndexTangent(),
                                      observer_.sizeForceTangent, observer_.sizeForceTangent)
                               .diagonal();
                         });
      logger.addLogEntry(category_ + "_MEKF_stateCovariances_extTorque_",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF()
                               .getStateCovariance()
                               .block(observer_.unmodeledTorqueIndexTangent(), observer_.unmodeledTorqueIndexTangent(),
                                      observer_.sizeTorqueTangent, observer_.sizeTorqueTangent)
                               .diagonal();
                         });

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


      /* Plots of the measurements */
      {
        logger.addLogEntry(category_ + "_MEKF_measurements_absoluteOri_measured",
                           [this]() -> Eigen::Quaterniond
                           {
                             so::kine::Orientation ori;
                             ori.fromVector4(observer_.getEKF().getLastMeasurement().tail(4));

                             return ori.toQuaternion().inverse();
                           });
        logger.addLogEntry(category_ + "_MEKF_measurements_absoluteOri_corrected",
                           [this]() -> Eigen::Quaterniond
                           {
                             so::kine::Orientation ori;
                             ori.fromVector4(correctedMeasurements_.tail(4));

                             return ori.toQuaternion().inverse();
                           });
        logger.addLogEntry(category_ + "_MEKF_measurements_absoluteOri_predicted",
                           [this]() -> Eigen::Quaterniond
                           {
                             so::kine::Orientation ori;
                             ori.fromVector4(observer_.getEKF().getLastPredictedMeasurement().tail(4));

                             return ori.toQuaternion().inverse();
                           });
      }

      /* Plots of the innovation */
      logger.addLogEntry(category_ + "_MEKF_innovation_positionW_",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getInnovation().segment(observer_.posIndexTangent(),
                                                                             observer_.sizePosTangent);
                         });
      logger.addLogEntry(category_ + "_MEKF_innovation_linVelW_",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getInnovation().segment(observer_.linVelIndexTangent(),
                                                                             observer_.sizeLinVelTangent);
                         });
      logger.addLogEntry(category_ + "_MEKF_innovation_oriW_",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getInnovation().segment(observer_.oriIndexTangent(),
                                                                             observer_.sizeOriTangent);
                         });
      logger.addLogEntry(category_ + "_MEKF_innovation_angVelW_",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getInnovation().segment(observer_.angVelIndexTangent(),
                                                                             observer_.sizeAngVelTangent);
                         });
      logger.addLogEntry(category_ + "_MEKF_innovation_unmodeledForce_",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getInnovation().segment(observer_.unmodeledForceIndexTangent(),
                                                                             observer_.sizeForceTangent);
                         });
      logger.addLogEntry(category_ + "_MEKF_innovation_unmodeledTorque_",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getInnovation().segment(observer_.unmodeledTorqueIndexTangent(),
                                                                             observer_.sizeTorqueTangent);
                         });

      /* Plots of the prediction */
      logger.addLogEntry(category_ + "_MEKF_prediction_posW",
                         [this]() -> Eigen::Vector3d
                         {
                           so::kine::LocalKinematics predictedWorldCentroidLocKine(
                               observer_.getEKF().getLastPrediction().segment(observer_.posIndex(),
                                                                              observer_.sizePos + observer_.sizeOri),
                               so::kine::Kinematics::Flags::pose);
                           so::kine::Kinematics predictedWorlCentroidKine(predictedWorldCentroidLocKine);
                           return predictedWorlCentroidKine.position();
                         });

      logger.addLogEntry(category_ + "_MEKF_prediction_worldFbPos",
                         [this]() -> Eigen::Vector3d
                         {
                           auto & inputRobot = my_robots_->robot("inputRobot");

                           so::kine::LocalKinematics predictedWorldCentroidLocKine(
                               observer_.getEKF().getLastPrediction().segment(observer_.posIndex(),
                                                                              observer_.sizePos + observer_.sizeOri),
                               so::kine::Kinematics::Flags::pose);
                           so::kine::Kinematics predictedWorldCentroidKine(predictedWorldCentroidLocKine);

                           so::kine::Kinematics fbCentroidKine;
                           fbCentroidKine.position = inputRobot.com();
                           fbCentroidKine.orientation.setZeroRotation();

                           so::kine::Kinematics predictedWorldFbKine =
                               predictedWorldCentroidKine * fbCentroidKine.getInverse();

                           return predictedWorldFbKine.position();
                         });

      logger.addLogEntry(
          category_ + "_MEKF_prediction_locPos", [this]() -> Eigen::Vector3d
          { return observer_.getEKF().getLastPrediction().segment(observer_.posIndex(), observer_.sizePos); });
      logger.addLogEntry(
          category_ + "_MEKF_prediction_locLinVel", [this]() -> Eigen::Vector3d
          { return observer_.getEKF().getLastPrediction().segment(observer_.linVelIndex(), observer_.sizeLinVel); });
      logger.addLogEntry(category_ + "_MEKF_prediction_ori",
                         [this]() -> Eigen::Quaterniond
                         {
                           so::kine::Orientation ori;
                           ori.fromVector4(
                               observer_.getEKF().getLastPrediction().segment(observer_.oriIndex(), observer_.sizeOri));
                           return ori.inverse().toQuaternion();
                         });
      logger.addLogEntry(category_ + "_MEKF_prediction_locAngVel",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastPrediction().segment(observer_.angVelIndex(),
                                                                                 observer_.sizeAngVelTangent);
                         });
      logger.addLogEntry(category_ + "_MEKF_prediction_unmodeledForce",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastPrediction().segment(observer_.unmodeledForceIndex(),
                                                                                 observer_.sizeForce);
                         });
      logger.addLogEntry(category_ + "_MEKF_prediction_unmodeledTorque",
                         [this]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastPrediction().segment(observer_.unmodeledTorqueIndex(),
                                                                                 observer_.sizeTorque);
                         });

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

void MCKineticsObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string &)
{
  for(const auto & [sensor, value] : ignoredSensorWrenches_)
  {
    logger.removeLogEntry(category_ + "_debug_wrenchesInCentroid_" + sensor + "_force");
    logger.removeLogEntry(category_ + "_debug_wrenchesInCentroid_" + sensor + "_torque");
  }
  for(const auto & [sensorName, measurement] : forceSensorMeasurements_)
  {
    const std::string prefix = category_ + "_debug_forceSensor_" + sensorName;
    logger.removeLogEntry(prefix + "_measuredForce");
    logger.removeLogEntry(prefix + "_measuredTorque");
  }
  logger.removeLogEntry(category_ + "_posW");
  logger.removeLogEntry(category_ + "_velW");
  logger.removeLogEntry(category_ + "_mass");

  logger.removeLogEntry(category_ + "_flexStiffness");
  logger.removeLogEntry(category_ + "_flexDamping");
}

void MCKineticsObserver::setOdometryType(const std::string & newOdometryType)
{
  prevOdometryType_ = odometryType_;
  odometryType_ = so::odometry::stringToOdometryType(newOdometryType);

  // if the type didn't change, we stop the function here
  if(odometryType_ == prevOdometryType_) { return; }

  mc_rtc::log::info("[{}]: Odometry mode changed to: {}", name(), newOdometryType);
  // valinor_.setOdometryType(odometryType_);
}

void MCKineticsObserver::addToGUI(const mc_control::MCController & ctl,
                                  mc_rtc::gui::StateBuilder & gui,
                                  const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;

  if(withGui_)
  {
    auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

    // clang-format off
    std::vector<std::string> covsCategory = category;
    covsCategory.insert(covsCategory.end(), {"Covariances"});

    std::vector<std::string> initCovsCategory = covsCategory;
    initCovsCategory.insert(initCovsCategory.end(), {"Init"});
    std::vector<std::string> processCovsCategory = covsCategory;
    processCovsCategory.insert(processCovsCategory.end(), {"Process"});
    std::vector<std::string> sensorCovsCategory = covsCategory;
    sensorCovsCategory.insert(sensorCovsCategory.end(), {"Sensors"});
  
    std::vector<std::string> removeOffsetCategory = category; 
    removeOffsetCategory.insert(removeOffsetCategory.end(), {"RemoveDisturbanceWrenchOffset"});
    
    gui.addPlot(  "Unbiased external wrench",
      mc_rtc::gui::plot::X( "t",    [&logger]() { return logger.t(); }),
      mc_rtc::gui::plot::Y("Force x", [this]() { return getUnbiasedEstimatedDisturbanceWrench().force()(0); }, Color::Red),
      mc_rtc::gui::plot::Y("Force y", [this]() { return getUnbiasedEstimatedDisturbanceWrench().force()(1); }, Color::Blue),
      mc_rtc::gui::plot::Y("Force z", [this]() { return getUnbiasedEstimatedDisturbanceWrench().force()(2); }, Color::Green),
      mc_rtc::gui::plot::Y("Moment x", [this]() { return getUnbiasedEstimatedDisturbanceWrench().moment()(0); }, Color::Magenta),
      mc_rtc::gui::plot::Y("Moment y", [this]() { return getUnbiasedEstimatedDisturbanceWrench().moment()(1); }, Color::Cyan),
      mc_rtc::gui::plot::Y("Moment z", [this]() { return getUnbiasedEstimatedDisturbanceWrench().moment()(2); }, Color::Black)
    );



    gui.addElement({category},
                          mc_rtc::gui::Button("Remove disturbance wrench offset", [this]() { 
                            // when clicking the button, the observer initializes the offset with the current disturbance wrench estimation
                            mc_rtc::log::info("Start removing disturbance wrench offset ");

                            wrenchOffsetIndex_ = 0;
                            removeWrenchOffset_ = true; disturbanceWrenchOffset_.force() = res_.segment(observer_.unmodeledWrenchIndex(), 3);
      disturbanceWrenchOffset_.moment() = res_.segment(observer_.unmodeledTorqueIndex(), 3);}));

    gui.addElement(initCovsCategory,
              mc_state_observation::gui::make_input_element("Contact pos x", contactInitCovarianceNewContacts_(0,0)),
              mc_state_observation::gui::make_input_element("Contact pos y", contactInitCovarianceNewContacts_(1,1)),
              mc_state_observation::gui::make_input_element("Contact pos z", contactInitCovarianceNewContacts_(2,2)),
              mc_state_observation::gui::make_input_element("Contact ori x", contactInitCovarianceNewContacts_(0,0)),
              mc_state_observation::gui::make_input_element("Contact ori y", contactInitCovarianceNewContacts_(1,1)),
              mc_state_observation::gui::make_input_element("Contact ori z", contactInitCovarianceNewContacts_(2,2)));

    gui.addElement(sensorCovsCategory,
              mc_state_observation::gui::make_input_element("Gyro x", gyroSensorCovariance_(0,0)),
              mc_state_observation::gui::make_input_element("Gyro y", gyroSensorCovariance_(1,1)),
              mc_state_observation::gui::make_input_element("Gyro z", gyroSensorCovariance_(2,2)),
              mc_state_observation::gui::make_input_element("Accelero x", acceleroSensorCovariance_(0,0)),
              mc_state_observation::gui::make_input_element("Accelero y", acceleroSensorCovariance_(1,1)),
              mc_state_observation::gui::make_input_element("Accelero z", acceleroSensorCovariance_(2,2)),
              mc_state_observation::gui::make_input_element("Force x", contactSensorCovariance_(0,0)),
              mc_state_observation::gui::make_input_element("Force y", contactSensorCovariance_(1,1)),
              mc_state_observation::gui::make_input_element("Force z", contactSensorCovariance_(2,2)),
              mc_state_observation::gui::make_input_element("Torque x", contactSensorCovariance_(3,3)),
              mc_state_observation::gui::make_input_element("Torque y", contactSensorCovariance_(4,4)),
              mc_state_observation::gui::make_input_element("Torque z", contactSensorCovariance_(5,5)));

    if(odometryType_ != so::odometry::OdometryType::None)
    {
      std::vector<std::string> odomCategory = category;
      odomCategory.insert(odomCategory.end(), {"Odometry"});
      gui.addElement({odomCategory}, mc_rtc::gui::ComboInput(
                                                                    "Choose from list",  {so::odometry::odometryTypeToString(so::odometry::OdometryType::Odometry6d), so::odometry::odometryTypeToString(so::odometry::OdometryType::Flat)},
                                                                    [this]() -> std::string {
                                                                      return so::odometry::odometryTypeToString(odometryType_);
                                                                    },
                                                                    [this](const std::string & typeOfOdometry) {
                                                                      setOdometryType(typeOfOdometry);
                                                                    }));
    } 
  } 
}

void MCKineticsObserver::addContactToGui(const mc_control::MCController & ctl,
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
                              if(pinContacts_) { return; }
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

void MCKineticsObserver::addContactLogEntries(const mc_control::MCController & ctl,
                                              mc_rtc::Logger & logger,
                                               KoContactWithSensor & contact)
{
  const std::string contactStatePrefix = category_ + "_MEKF_estimatedState_contact_" + contact.surfaceName();
  const auto initPrefix = category_ + "_debug_contactKine_" + contact.surfaceName() + "_initKine";
  const auto wrenchPrefix = category_ + "_debug_wrenchesInCentroid_" + contact.surfaceName();
  logger.addLogEntry(wrenchPrefix + "_force", &contact,
                    [&contact]() -> so::Vector3 { return contact.wrenchInCentroid_.head<3>(); });
  logger.addLogEntry(wrenchPrefix + "_torque", &contact,
                    [&contact]() -> so::Vector3 { return contact.wrenchInCentroid_.tail<3>(); });
  logger.addLogEntry(wrenchPrefix + "_forceWithUnmodeled", &contact,
                    [this, &contact]() -> so::Vector3
                    { return observer_.getCurrentStateVector().segment(observer_.unmodeledForceIndex(), observer_.sizeForce)
                             + contact.wrenchInCentroid_.head<3>(); });
  logger.addLogEntry(wrenchPrefix + "_torqueWithUnmodeled", &contact,
                    [this, &contact]() -> so::Vector3
                    { return observer_.getCurrentStateVector().segment(observer_.unmodeledTorqueIndex(), observer_.sizeTorque)
                             + contact.wrenchInCentroid_.tail<3>(); });
  logger.addLogEntry(initPrefix + "_position", &contact,
                    [&contact]() -> so::Vector3 { return contact.initKine_.position(); });
  logger.addLogEntry(initPrefix + "_ori", &contact,
                    [&contact]() -> Eigen::Quaterniond { return contact.initKine_.orientation.inverse().toQuaternion(); });
  logger.addLogEntry(contactStatePrefix + "_initialRestOrientationCorrection", &contact,
                     [&contact]() -> const so::Vector3 & { return contact.initRestOriDiff_; });
  logger.addLogEntry(contactStatePrefix + "_initialRestOrientationCorrectionDeg", &contact,
                     [&contact]() -> double { return contact.initRestOriAngleDeg_; });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_contact_" + contact.surfaceName() + "_position", &contact,
                     [this, &contact]() -> Eigen::Vector3d {
                       return observer_.getCurrentStateVector().segment(observer_.contactPosIndex(contact.id()),
                                                                        observer_.sizePos);
                     });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_contact_" + contact.surfaceName() + "_orientation", &contact,
                     [this, &contact]() -> Eigen::Quaternion<double>
                     {
                       so::kine::Orientation ori;
                       return ori
                           .fromVector4(observer_.getCurrentStateVector().segment(
                               observer_.contactOriIndex(contact.id()), observer_.sizeOri))
                           .inverse()
                           .toQuaternion();
                     });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_contact_" + contact.surfaceName() + "_orientation_RollPitchYaw",
                     &contact,
                     [this, &contact]() -> so::Vector3
                     {
                       so::kine::Orientation ori;
                       return so::kine::rotationMatrixToRollPitchYaw(
                           ori.fromVector4(observer_.getCurrentStateVector().segment(
                                               observer_.contactOriIndex(contact.id()), observer_.sizeOri))
                               .inverse()
                               .toMatrix3());
                     });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_contact_" + contact.surfaceName() + "_forces", &contact,
                     [this, &contact]() -> Eigen::Vector3d {
                       return observer_.getCurrentStateVector().segment(observer_.contactForceIndex(contact.id()),
                                                                        observer_.sizeForce);
                     });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_contact_" + contact.surfaceName() + "_torques", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getCurrentStateVector().segment(observer_.contactTorqueIndex(contact.id()),
                                                                        observer_.sizeTorque);
                     });
  if(withDebugLogs_)
  {
  logger.addLogEntry(category_ + "_MEKF_stateCovariances_contact_" + contact.surfaceName() + "_position_", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block(observer_.contactPosIndexTangent(contact.id()),
                                  observer_.contactPosIndexTangent(contact.id()), observer_.sizePosTangent,
                                  observer_.sizePosTangent)
                           .diagonal();
                     });
  logger.addLogEntry(category_ + "_MEKF_stateCovariances_contact_" + contact.surfaceName() + "_orientation_", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block(observer_.contactOriIndexTangent(contact.id()),
                                  observer_.contactOriIndexTangent(contact.id()), observer_.sizeOriTangent,
                                  observer_.sizeOriTangent)
                           .diagonal();
                     });
  logger.addLogEntry(category_ + "_MEKF_stateCovariances_contact_" + contact.surfaceName() + "_Force_", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block(observer_.contactForceIndexTangent(contact.id()),
                                  observer_.contactForceIndexTangent(contact.id()), observer_.sizeForceTangent,
                                  observer_.sizeForceTangent)
                           .diagonal();
                     });
  logger.addLogEntry(category_ + "_MEKF_stateCovariances_contact_" + contact.surfaceName() + "_Torque_", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block(observer_.contactTorqueIndexTangent(contact.id()),
                                  observer_.contactTorqueIndexTangent(contact.id()), observer_.sizeTorqueTangent,
                                  observer_.sizeTorqueTangent)
                           .diagonal();
                     });

  logger.addLogEntry(
      category_ + "_MEKF_prediction_contact_" + contact.surfaceName() + "_poseWorldFromCentroid_pos", &contact,
      [this, &contact]() -> Eigen::Vector3d
      {
        auto & inputRobot = my_robots_->robot("inputRobot");

        so::kine::LocalKinematics predictedWorldCentroidLocKine(
            observer_.getEKF().getLastPrediction().segment(observer_.posIndex(), observer_.sizePos + observer_.sizeOri),
            so::kine::Kinematics::Flags::pose);
        so::kine::Kinematics predictedWorldCentroidKine(predictedWorldCentroidLocKine);
        so::kine::Kinematics fbCentroidKine;
        fbCentroidKine.position = inputRobot.com();
        fbCentroidKine.orientation.setZeroRotation();

        so::kine::Kinematics predictedWorldContactKine =
            predictedWorldCentroidKine * fbCentroidKine.getInverse() * contact.fbContactKine_;

        return predictedWorldContactKine.position();
      });

  logger.addLogEntry(category_ + "_MEKF_prediction_contact_" + contact.surfaceName() + "_poseWorldFromCentroid_ori",
                     &contact,
                     [this, &contact]() -> Eigen::Quaterniond
                     {
                       so::kine::Orientation predictedWorldCentroidLocKine;
                       predictedWorldCentroidLocKine.fromVector4(
                           observer_.getEKF().getLastPrediction().segment(observer_.oriIndex(), observer_.sizeOri));

                       so::kine::Orientation predictedWorldContactOri(so::Matrix3(
                           predictedWorldCentroidLocKine.toMatrix3() * contact.fbContactKine_.orientation.toMatrix3()));

                       return predictedWorldContactOri.inverse().toQuaternion();
                     });

  logger.addLogEntry(category_ + "_MEKF_prediction_contact_" + contact.surfaceName() + "_poseWorldFromCentroid_linVel",
                     &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       auto & inputRobot = my_robots_->robot("inputRobot");

                       so::kine::LocalKinematics predictedWorldCentroidLocKine(
                           observer_.getEKF().getLastPrediction().segment(
                               observer_.posIndex(),
                               observer_.sizePos + observer_.sizeOri + observer_.sizeLinVel + observer_.sizeAngVel),
                           so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);
                       so::kine::Kinematics predictedWorldCentroidKine(predictedWorldCentroidLocKine);

                       so::kine::Kinematics fbCentroidKine;
                       fbCentroidKine.position = inputRobot.com();
                       fbCentroidKine.linVel = inputRobot.comVelocity();
                       fbCentroidKine.orientation.setZeroRotation();
                       fbCentroidKine.angVel = so::Vector3::Zero();

                       so::kine::Kinematics predictedWorldContactKine =
                           predictedWorldCentroidKine * fbCentroidKine.getInverse() * contact.fbContactKine_;

                       return predictedWorldContactKine.linVel();
                     });

  logger.addLogEntry(category_ + "_MEKF_prediction_contact_" + contact.surfaceName() + "_poseWorldFromCentroid_angVel",
                     &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       auto & inputRobot = my_robots_->robot("inputRobot");

                       so::kine::LocalKinematics predictedWorldCentroidLocKine(
                           observer_.getEKF().getLastPrediction().segment(
                               observer_.posIndex(),
                               observer_.sizePos + observer_.sizeOri + observer_.sizeLinVel + observer_.sizeAngVel),
                           so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);
                       so::kine::Kinematics predictedWorldCentroidKine(predictedWorldCentroidLocKine);

                       so::kine::Kinematics fbCentroidKine;
                       fbCentroidKine.position = inputRobot.com();
                       fbCentroidKine.linVel = inputRobot.comVelocity();
                       fbCentroidKine.orientation.setZeroRotation();
                       fbCentroidKine.angVel = so::Vector3::Zero();

                       so::kine::Kinematics predictedWorldContactKine =
                           predictedWorldCentroidKine * fbCentroidKine.getInverse() * contact.fbContactKine_;

                       return predictedWorldContactKine.angVel();
                     });

  logger.addLogEntry(category_ + "_MEKF_prediction_contact_" + contact.surfaceName() + "_restPos_W", &contact,
                     [this, &contact]() -> Eigen::Vector3d {
                       return observer_.getEKF().getLastPrediction().segment(observer_.contactPosIndex(contact.id()),
                                                                             observer_.sizePos);
                     });
  logger.addLogEntry(category_ + "_MEKF_prediction_contact_" + contact.surfaceName() + "_restOri_W", &contact,
                     [this, &contact]() -> Eigen::Quaternion<double>
                     {
                       so::kine::Orientation ori;
                       return ori
                           .fromVector4(observer_.getEKF().getLastPrediction().segment(
                               observer_.contactOriIndex(contact.id()), observer_.sizeOri))
                           .inverse()
                           .toQuaternion();
                     });
  logger.addLogEntry(category_ + "_MEKF_prediction_contact_" + contact.surfaceName() + "_forces", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getLastPrediction().segment(observer_.contactForceIndex(contact.id()),
                                                                             observer_.sizeForce);
                     });
  logger.addLogEntry(category_ + "_MEKF_prediction_contact_" + contact.surfaceName() + "_torques", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getLastPrediction().segment(observer_.contactTorqueIndex(contact.id()),
                                                                             observer_.sizeTorque);
                     });

  logger.addLogEntry(category_ + "_MEKF_debug_contactWrench_Centroid_" + contact.surfaceName() + "_force", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getCentroidContactWrench(contact.id()).segment(0, observer_.sizeForce); });

  logger.addLogEntry(category_ + "_MEKF_debug_contactWrench_Centroid_" + contact.surfaceName() + "_torque", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getCentroidContactWrench(contact.id()).segment(3, observer_.sizeTorque); });

  }

  conversions::kinematics::addToLogger(logger, contact.fbContactKine_, category_ + "_debug_contactKine_" + contact.surfaceName() + "_fbContactKine");

  conversions::kinematics::addToLogger(logger, contact.contactSensorKine_,
                                       category_ + "_debug_contactKine_" + contact.surfaceName() + "_contactSensorKine");

  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.surfaceName() + "_inputCentroidContactKine_position", &contact,
      [this, &contact]() -> Eigen::Vector3d { return observer_.getCentroidContactInputKine(contact.id()).position(); });

  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.surfaceName() + "_inputCentroidContactKine_orientation", &contact,
      [this, &contact]() -> Eigen::Quaternion<double> {
        return observer_.getCentroidContactInputKine(contact.id()).orientation.inverse().toQuaternion();
      });
  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.surfaceName() + "_inputCentroidContactKine_linVel", &contact,
      [this, &contact]() -> Eigen::Vector3d { return observer_.getCentroidContactInputKine(contact.id()).linVel(); });

  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.surfaceName() + "_inputCentroidContactKine_angVel", &contact,
      [this, &contact]() -> Eigen::Vector3d { return observer_.getCentroidContactInputKine(contact.id()).angVel(); });
      
  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.surfaceName() + "_realRobot_position", &contact,
      [this, &contact, &ctl]() -> Eigen::Vector3d
      { 
        const auto & realRobot = ctl.realRobot(robot_);
        return getContactWorldKinematics(ctl, contact, realRobot, false).position();
      });

  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.surfaceName() + "_ctlRobot_position", &contact,
                     [this, &contact, &ctl]() -> Eigen::Vector3d
                     {
                       const auto & robot = ctl.robot(robot_);
                       return getContactWorldKinematics(ctl, contact, robot,false).position();
                     });

  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.surfaceName() + "_worldcontactKineFromCentroid_position", &contact,
      [this, &contact]() -> Eigen::Vector3d
      { return observer_.getWorldContactKineFromCentroid(contact.id()).position(); });

  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.surfaceName() + "_worldcontactKineFromCentroid_orientation", &contact,
      [this, &contact]() -> Eigen::Quaternion<double> {
        return observer_.getWorldContactKineFromCentroid(contact.id()).orientation.inverse().toQuaternion();
      });

  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.surfaceName() + "_worldcontactKineFromCentroid_linVel",
                     &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getWorldContactKineFromCentroid(contact.id()).linVel(); });

  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.surfaceName() + "_worldcontactKineFromCentroid_angVel",
                     &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getWorldContactKineFromCentroid(contact.id()).angVel(); });

  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.surfaceName() + "_inputUserContactKine_position", &contact,
      [this, &contact]() -> Eigen::Vector3d { return observer_.getUserContactInputKine(contact.id()).position(); });
  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.surfaceName() + "_inputUserContactKine_orientation",
                     &contact,
                     [this, &contact]() -> Eigen::Quaternion<double> {
                       return observer_.getUserContactInputKine(contact.id()).orientation.inverse().toQuaternion();
                     });
  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.surfaceName() + "_inputUserContactKine_linVel", &contact,
      [this, &contact]() -> Eigen::Vector3d { return observer_.getUserContactInputKine(contact.id()).linVel(); });
  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.surfaceName() + "_inputUserContactKine_angVel", &contact,
      [this, &contact]() -> Eigen::Vector3d { return observer_.getUserContactInputKine(contact.id()).angVel(); });

  logger.addLogEntry(category_ + "_debug_contactState_isSet_" + contact.surfaceName(), &contact,
                     [&contact]() -> std::string { return contact.isSet() ? "Set" : "notSet"; });

  const auto & robot = my_robots_->robot();
  if(withDebugLogs_ && robot.hasForceSensor(contact.fsName_))
  {
    logger.addLogEntry(category_ + "_debug_contactSensorCalibrationOffset_" + contact.surfaceName() + "_force",
                       &contact,
                       [this, &contact]() -> Eigen::Vector3d
                       { return my_robots_->robot().forceSensor(contact.fsName_).calib().offset.force(); });
    logger.addLogEntry(category_ + "_debug_contactSensorCalibrationOffset_" + contact.surfaceName() + "_torque",
                       &contact,
                       [this, &contact]() -> Eigen::Vector3d
                       { return my_robots_->robot().forceSensor(contact.fsName_).calib().offset.moment(); });
  }
}

void MCKineticsObserver::addContactMeasurementsLogEntries(mc_rtc::Logger & logger, const KoContactWithSensor & contact)
{
  for(int row = 0; row < 6; ++row)
  {
    for(int column = 0; column < 6; ++column)
    {
      logger.addLogEntry(
          category_ + "_MEKF_inputs_contacts_wrenchCovariance_" + contact.surfaceName() + "_" +
              std::to_string(row * 6 + column),
          &contact.contactWrenchVector_, [this, &contact, row, column]() -> double
          { return contactWrenchCovariance(contact)(row, column); });
    }
  }

  logger.addLogEntry(category_ + "_MEKF_measurements_contacts_force_" + contact.surfaceName() + "_measured", &contact.contactWrenchVector_,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getLastMeasurement().segment(
                           observer_.getContactMeasIndexByNum(contact.id()), observer_.sizeForce);
                     });
  logger.addLogEntry(category_ + "_MEKF_measurements_contacts_torque_" + contact.surfaceName() + "_measured", &contact.contactWrenchVector_,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getLastMeasurement().segment(
                           observer_.getContactMeasIndexByNum(contact.id()) + observer_.sizeForce,
                           observer_.sizeTorque);
                     });
  if(!withDebugLogs_) { return; }

  // Innovation
  logger.addLogEntry(category_ + "_MEKF_innovation_contacts_" + contact.surfaceName() + "_position", &contact.contactWrenchVector_,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment(observer_.contactPosIndexTangent(contact.id()),
                                                                         observer_.sizePosTangent);
                     });
  logger.addLogEntry(category_ + "_MEKF_innovation_contacts_" + contact.surfaceName() + "_orientation", &contact.contactWrenchVector_,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment(observer_.contactOriIndexTangent(contact.id()),
                                                                         observer_.sizeOriTangent);
                     });
  logger.addLogEntry(category_ + "_MEKF_innovation_contacts_" + contact.surfaceName() + "_force", &contact.contactWrenchVector_,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment(
                           observer_.contactForceIndexTangent(contact.id()), observer_.sizeForceTangent);
                     });
  logger.addLogEntry(category_ + "_MEKF_innovation_contacts_" + contact.surfaceName() + "_torque", &contact.contactWrenchVector_,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment(
                           observer_.contactTorqueIndexTangent(contact.id()), observer_.sizeTorqueTangent);
                     });

  logger.addLogEntry(
      category_ + "_MEKF_measurements_contacts_force_" + contact.surfaceName() + "_viscoAfterCorrection", &contact.contactWrenchVector_,
      [&contact]() -> Eigen::Vector3d { return contact.viscoElasticWrenchAfterCorrection_.segment(0, 3); });
  logger.addLogEntry(
      category_ + "_MEKF_measurements_contacts_torque_" + contact.surfaceName() + "_viscoAfterCorrection", &contact.contactWrenchVector_,
      [&contact]() -> Eigen::Vector3d { return contact.viscoElasticWrenchAfterCorrection_.segment(3, 3); });

  // Measurements
  logger.addLogEntry(category_ + "_MEKF_measurements_contacts_force_" + contact.surfaceName() + "_predicted", &contact.contactWrenchVector_,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getLastPredictedMeasurement().segment(
                           observer_.getContactMeasIndexByNum(contact.id()), observer_.sizeForce);
                     });
  logger.addLogEntry(category_ + "_MEKF_measurements_contacts_force_" + contact.surfaceName() + "_corrected", &contact.contactWrenchVector_,
                     [this, &contact]() -> Eigen::Vector3d {
                       return correctedMeasurements_.segment(observer_.getContactMeasIndexByNum(contact.id()),
                                                             observer_.sizeForce);
                     });

  // Correction this contact's wrench measurement applies to the gyrometer bias, i.e. the innovation
  // restricted to the columns of this block: K.block(bias, block) * (y - y^).segment(block). The IMU
  // counterparts are added in addToLogger; together they say which inconsistency feeds the bias.
  for(auto & imu : listIMUs_)
  {
    auto contribution = [this, &contact, &imu](so::Index offset, so::Index size) -> Eigen::Vector3d
    {
      const auto & gain = observer_.getEKF().getLastGain();
      const so::Index row = observer_.gyroBiasIndexTangent(imu.id());
      const so::Index column = observer_.getContactMeasIndexByNum(contact.id()) + offset;
      if(!observer_.getContactIsSetByNum(contact.id()) || gain.rows() < row + observer_.sizeGyroBiasTangent
         || gain.cols() < column + size)
      {
        return Eigen::Vector3d::Zero();
      }
      const Eigen::VectorXd residual =
          observer_.getEKF().getLastMeasurement() - observer_.getEKF().getLastPredictedMeasurement();
      return gain.block(row, column, observer_.sizeGyroBiasTangent, size) * residual.segment(column, size);
    };
    logger.addLogEntry(category_ + "_MEKF_innovationFrom_contactForce_" + contact.surfaceName() + "_" + imu.name(),
                       &contact.contactWrenchVector_, [this, contribution]() -> Eigen::Vector3d
                       { return contribution(0, observer_.sizeForce); });
    logger.addLogEntry(category_ + "_MEKF_innovationFrom_contactTorque_" + contact.surfaceName() + "_" + imu.name(),
                       &contact.contactWrenchVector_, [this, contribution]() -> Eigen::Vector3d
                       { return contribution(observer_.sizeForce, observer_.sizeTorque); });

    // Same columns, but the rows of the disturbance wrench: does the slack variable take this
    // contact's residual, or does it leave it to the gyrometer bias?
    auto wrenchContribution = [this, &contact](so::Index row, so::Index rowSize, so::Index offset,
                                               so::Index size) -> Eigen::Vector3d
    {
      const auto & gain = observer_.getEKF().getLastGain();
      const so::Index column = observer_.getContactMeasIndexByNum(contact.id()) + offset;
      if(!observer_.getContactIsSetByNum(contact.id()) || gain.rows() < row + rowSize
         || gain.cols() < column + size)
      {
        return Eigen::Vector3d::Zero();
      }
      const Eigen::VectorXd residual =
          observer_.getEKF().getLastMeasurement() - observer_.getEKF().getLastPredictedMeasurement();
      return gain.block(row, column, rowSize, size) * residual.segment(column, size);
    };
    logger.addLogEntry(category_ + "_MEKF_wrenchFrom_contactForce_" + contact.surfaceName(), &contact.contactWrenchVector_,
                       [this, wrenchContribution]() -> Eigen::Vector3d {
                         return wrenchContribution(observer_.unmodeledForceIndexTangent(), observer_.sizeForceTangent,
                                                   0, observer_.sizeForce);
                       });
    logger.addLogEntry(category_ + "_MEKF_wrenchFrom_contactTorque_" + contact.surfaceName(), &contact.contactWrenchVector_,
                       [this, wrenchContribution]() -> Eigen::Vector3d {
                         return wrenchContribution(observer_.unmodeledTorqueIndexTangent(), observer_.sizeTorqueTangent,
                                                   observer_.sizeForce, observer_.sizeTorque);
                       });
    // And on the orientation rows: what this contact's wrench measurement actually corrects in the
    // attitude, the third component being the yaw.
    logger.addLogEntry(category_ + "_MEKF_oriFrom_contactForce_" + contact.surfaceName(), &contact.contactWrenchVector_,
                       [this, wrenchContribution]() -> Eigen::Vector3d {
                         return wrenchContribution(observer_.oriIndexTangent(), observer_.sizeOriTangent,
                                                   0, observer_.sizeForce);
                       });
    logger.addLogEntry(category_ + "_MEKF_oriFrom_contactTorque_" + contact.surfaceName(), &contact.contactWrenchVector_,
                       [this, wrenchContribution]() -> Eigen::Vector3d {
                         return wrenchContribution(observer_.oriIndexTangent(), observer_.sizeOriTangent,
                                                   observer_.sizeForce, observer_.sizeTorque);
                       });
  }
  logger.addLogEntry(category_ + "_MEKF_measurements_contacts_torque_" + contact.surfaceName() + "_predicted", &contact.contactWrenchVector_,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getLastPredictedMeasurement().segment(
                           observer_.getContactMeasIndexByNum(contact.id()) + observer_.sizeForce,
                           observer_.sizeTorque);
                     });
  logger.addLogEntry(category_ + "_MEKF_measurements_contacts_torque_" + contact.surfaceName() + "_corrected", &contact.contactWrenchVector_,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return correctedMeasurements_.segment(observer_.getContactMeasIndexByNum(contact.id())
                                                                 + observer_.sizeForce,
                                                             observer_.sizeTorque);
                     });
}

void MCKineticsObserver::removeContactLogEntries(mc_rtc::Logger & logger, const KoContactWithSensor & contact)
{
  logger.removeLogEntries(&contact);
  removeContactMeasurementsLogEntries(logger, contact);
  conversions::kinematics::removeFromLogger(logger, contact.fbContactKine_);
  conversions::kinematics::removeFromLogger(logger, contact.contactSensorKine_);
}

void MCKineticsObserver::removeContactMeasurementsLogEntries(mc_rtc::Logger & logger,
                                                             const KoContactWithSensor & contact)
{
  logger.removeLogEntries(&contact.contactWrenchVector_);
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("MCKineticsObserver", mc_state_observation::MCKineticsObserver)
