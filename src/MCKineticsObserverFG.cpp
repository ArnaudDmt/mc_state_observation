/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/logging.h>

#include <array>
#include <cmath>
#include <gtsam/geometry/Pose3.h>
#include <mc_state_observation/MCKineticsObserverFG.h>
#include <mc_state_observation/gui_helpers.h>

#include <RBDyn/Coriolis.h>
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
MCKineticsObserverFG::MCKineticsObserverFG(const std::string & type, double dt)
: mc_observers::Observer(type, dt), observer_(0.016), removeWrenchOffset_(false)
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

  std::string contactsDetectionString = static_cast<std::string>(contactsConfig("contactsDetection"));
  KoContactsDetector::ContactsDetection contactsDetectionMethod =
      KoContactsDetector::stringToContactsDetection(contactsDetectionString, name());

  if(contactsDetectionMethod == KoContactsDetector::ContactsDetection::Surfaces)
  {
    std::vector<std::string> surfacesForContactDetection =
        contactsConfig("surfacesForContactDetection", std::vector<std::string>());

    for(const auto & surface : surfacesForContactDetection)
    {
      const std::string fsName = ctl.robot().indirectSurfaceForceSensor(surface).name();
      contactsManager_.fs_Surface_Map.emplace(fsName, surface);
    }

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
  so::Vector3 linStiffness = contactsConfig("linStiffness");
  contactFlexibilities_.at(0) = linStiffness.matrix().asDiagonal();
  so::Vector3 angStiffness = contactsConfig("angStiffness");
  contactFlexibilities_.at(1) = angStiffness.matrix().asDiagonal();
  so::Vector3 linDamping = contactsConfig("linDamping");
  contactFlexibilities_.at(2) = linDamping.matrix().asDiagonal();
  so::Vector3 angDamping = contactsConfig("angDamping");
  contactFlexibilities_.at(3) = angDamping.matrix().asDiagonal();

  /* Initial state noises */
  auto stateInitNoises = config("stateInitNoises");

  initNoises_.at(0) = stateInitNoises("posNoise"); // pos
  initNoises_.at(1) = stateInitNoises("oriNoise"); // ori
  initNoises_.at(2) = stateInitNoises("linVelNoise"); // linVel
  initNoises_.at(3) = stateInitNoises("angVelNoise"); // angVel
  initNoises_.at(4) = stateInitNoises("linAccNoise"); // linAcc
  initNoises_.at(5) = stateInitNoises("angAccNoise"); // angAcc
  initNoises_.at(6) = stateInitNoises("disturbForceNoise"); // distForce
  initNoises_.at(7) = stateInitNoises("disturbMomentNoise"); // distMoment
  initNoises_.at(8) = stateInitNoises("contactRestPosAverageNoise"); // distMoment
  initNoises_.at(9) = stateInitNoises("contactRestYawAverageNoise"); // distMoment

  contactInitNoises_.at(0) = stateInitNoises("restPosNoise"); // rest pos
  contactInitNoises_.at(1) = stateInitNoises("restOriNoise"); // rest ori
  contactInitNoises_.at(2) = stateInitNoises("forceNoise"); // force
  contactInitNoises_.at(3) = stateInitNoises("momentNoise"); // moment

  contactInitNoises_first_ = contactInitNoises_;
  contactInitNoises_first_.at(0) = 1e-4;
  contactInitNoises_first_.at(1) = 1e-4;

  /* State process noises */
  auto stateProcessNoises = config("stateProcessNoises");

  processNoises_.at(0) = stateProcessNoises("posNoise"); // pos
  processNoises_.at(1) = stateProcessNoises("oriNoise"); // ori
  processNoises_.at(2) = stateProcessNoises("linVelNoise"); // linVel
  processNoises_.at(3) = stateProcessNoises("angVelNoise"); // angVel
  processNoises_.at(4) = stateProcessNoises("linAccNoise"); // linAcc
  processNoises_.at(5) = stateProcessNoises("angAccNoise"); // angAcc
  processNoises_.at(6) = stateProcessNoises("disturbForceNoise"); // distForce
  processNoises_.at(7) = stateProcessNoises("disturbMomentNoise"); // distMoment

  contactProcessNoises_.at(0) = stateProcessNoises("restPosNoise"); // rest pos
  contactProcessNoises_.at(1) = stateProcessNoises("restOriNoise"); // rest ori
  contactProcessNoises_.at(2) = stateProcessNoises("forceNoise"); // force
  contactProcessNoises_.at(3) = stateProcessNoises("momentNoise"); // moment

  /* Sensor noises */
  auto sensorNoises = config("sensorNoises");
  for(size_t i = 0; i < listIMUs_.size(); ++i)
  {
    std::array<double, 4> imuNoise;
    imuNoise[0] = stateInitNoises("gyroBiasNoise"); // gyroBias init
    imuNoise[1] = stateProcessNoises("gyroBiasNoise"); // gyroBias process
    imuNoise[2] = sensorNoises("gyroNoise"); // gyro meas
    imuNoise[3] = sensorNoises("acceleroNoise"); // accelero meas

    imuNoises_.insert({i, imuNoise});
  }

  contactMeasNoises_.at(0) = sensorNoises("forceNoise"); // force
  contactMeasNoises_.at(1) = sensorNoises("forceNoise"); // torque

  if(config.has("jointTorques"))
  {
    auto jointTorquesConfig = config("jointTorques");
    useJointTorqueMeasurements_ = jointTorquesConfig("enabled", false);
    if(useJointTorqueMeasurements_)
    {
      useJointTorqueCommandAsMeasurement_ = jointTorquesConfig("useCommandAsMeasurement", true);
      jointTorquesConfig("noise", jointTorqueNoise_);
      mc_rtc::log::info("[{}]: Joint impulse-momentum factor enabled: torque sigma={}, command fallback={}.", name(),
                        jointTorqueNoise_, useJointTorqueCommandAsMeasurement_);
    }
    else
    {
      // Keep the disabled path independent of every other jointTorques key.
      // In particular, merely adding a noise entry must not alter the graph.
      useJointTorqueCommandAsMeasurement_ = false;
      mc_rtc::log::info("[{}]: Joint impulse-momentum factor disabled.", name());
    }
  }
}

void MCKineticsObserverFG::reset(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);

  mass_ = ctl.realRobot(robot_).mass();
  observer_.setMass(mass_);
  maintainedContacts_.clear();

  /* Initialization of variables */
  X_0_fb_ = sva::PTransformd::Identity();
  v_fb_0_ = sva::MotionVecd::Zero();
  a_fb_0_ = sva::MotionVecd::Zero();

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  my_robots_->robotCopy(realRobot, "inputRobot");
  if(withGui_)
  {
    ctl.gui()->addElement(
        {"Robots"}, mc_rtc::gui::Robot(name(), [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));
    ctl.gui()->addElement({"Robots"},
                          mc_rtc::gui::Robot("Real", [&ctl]() -> const mc_rbdyn::Robot & { return ctl.realRobot(); }));
  }

  X_0_fb_ = realRobot.posW().translation();

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
  pendingMomentumContactIds_.clear();
  pendingMomentumTime_ = 0;

  if(useJointTorqueMeasurements_)
  {
    const size_t jointTorqueDim = static_cast<size_t>(actuatedJointTorqueDim(realRobot));

    if(jointTorqueDim > 0)
    {
      inputJointTorques_ = Vector::Zero(jointTorqueDim);
      measuredJointTorques_ = Vector::Zero(jointTorqueDim);
      estimatedJointTorqueResidual_ = Vector::Zero(jointTorqueDim);
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
  sva::RBInertiad Ic;

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
  inputWrench_ = inputAdditionalWrench(ctl, robot);

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
                                                                             ctl.robot().posW().rotation().transpose());
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
  my_robots_->robot().mbc().q = ctl.realRobot().mbc().q;

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

Vector6 MCKineticsObserverFG::inputAdditionalWrench(const mc_control::MCController & ctl,
                                                    const mc_rbdyn::Robot & measRobot)
{
  Vector6 additionalWrench = Vector6::Zero();

  for(const auto & forceSensor : measRobot.forceSensors())
  {
    const auto it = contactsManager_.fs_Surface_Map.find(forceSensor.name());

    bool useSensor = false;

    if(it == contactsManager_.fs_Surface_Map.end() || !contactsManager_.contacts().count(it->second))
    {
      useSensor = true; // not associated to any contact
    }
    else
    {
      auto * contact = contactsManager_.findContact(it->second);
      useSensor = (contact && contact->sensorEnabled_ && !contact->isSet());
    }

    if(useSensor)
    {
      sva::ForceVecd measuredWrench = forceSensor.worldWrenchWithoutGravity(ctl.realRobot(robot_));
      additionalWrench.segment(0, 3) += measuredWrench.force();
      additionalWrench.segment(3, 3) += measuredWrench.moment();
    }
  }

  return additionalWrench;
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

ko_fg::MomentumResidualMeasurement MCKineticsObserverFG::makeMomentumResidualMeasurement(
    const mc_rbdyn::Robot & measRobot,
    mc_rbdyn::Robot & dynamicsRobot)
{
  ko_fg::MomentumResidualMeasurement measurement;
  measurement.current_ = makeMomentumResidualEndpoint(dynamicsRobot);
  measurement.previous_ = previousMomentumEndpoint_.value();
  previousMomentumEndpoint_ = measurement.current_;

  const Eigen::VectorXd measuredTorque = measuredJointTorqueVector(measRobot);
  const Eigen::VectorXd previousMeasuredTorque = previousMeasuredJointTorques_.value_or(measuredTorque);
  previousMeasuredJointTorques_ = measuredTorque;
  const Eigen::Index dimension = 6 + measuredTorque.size();
  measurement.knownGeneralizedForce_ = Eigen::VectorXd::Zero(dimension);
  measurement.knownGeneralizedForce_.tail(measuredTorque.size()) = 0.5 * (previousMeasuredTorque + measuredTorque);

  measurement.dt_ = dt_;
  // The factor residual is a trapezoidal generalized impulse. Assuming
  // independent endpoint torque samples, sigma_I = dt * sigma_tau / sqrt(2).
  Eigen::VectorXd sigmas = Eigen::VectorXd::Constant(measuredTorque.size(), dt_ * jointTorqueNoise_ / std::sqrt(2.0));
  measurement.noise_ = ko_fg::makeNoise(sigmas);

  return measurement;
}

void MCKineticsObserverFG::updateJointTorqueMeasurement(const mc_rbdyn::Robot & measRobot,
                                                        mc_rbdyn::Robot & dynamicsRobot)
{
  const Eigen::VectorXd measuredTorque = measuredJointTorqueVector(measRobot);
  if(measuredTorque.size() == 0) { return; }
  measuredJointTorques_ = measuredTorque;

  if(!previousMomentumEndpoint_)
  {
    previousMomentumEndpoint_ = makeMomentumResidualEndpoint(dynamicsRobot);
    previousMeasuredJointTorques_ = measuredTorque;
    return;
  }

  const auto measurement = makeMomentumResidualMeasurement(measRobot, dynamicsRobot);
  observer_.updateMomentumResidualMeasurement(measurement);

  pendingMomentumMeasurement_ = measurement;
  pendingMomentumContactIds_.clear();
  pendingMomentumContactIds_.reserve(observer_.getActiveContacts().size());
  for(const auto & [contactId, contact] : observer_.getActiveContacts())
  {
    (void)contact;
    pendingMomentumContactIds_.push_back(contactId);
  }
  // runIteration(k_) attaches the current measurement to graph state k_ + 1.
  pendingMomentumTime_ = k_ + 1;
}

void MCKineticsObserverFG::updateEstimatedJointTorqueResidual()
{
  if(!pendingMomentumMeasurement_ || pendingMomentumTime_ == 0) { return; }

  const size_t current = pendingMomentumTime_;
  const size_t previous = current - 1;
  gtsam::KeyVector keys = {ko_fg::X(previous), ko_fg::V(previous), ko_fg::W(previous),
                           ko_fg::X(current),  ko_fg::V(current),  ko_fg::W(current)};
  for(const size_t contactId : pendingMomentumContactIds_)
  {
    const auto graphContactId = static_cast<uint32_t>(contactId);
    keys.push_back(ko_fg::F(previous, graphContactId));
    keys.push_back(ko_fg::T(previous, graphContactId));
    keys.push_back(ko_fg::F(current, graphContactId));
    keys.push_back(ko_fg::T(current, graphContactId));
  }

  const gtsam::Values estimate = observer_.getSmoother().calculateEstimate();
  for(const gtsam::Key key : keys)
  {
    // Bootstrap buffering can leave the newest measurement outside the
    // smoother until the first batch is submitted.
    if(!estimate.exists(key)) { return; }
  }

  const ko_fg::measurementFactors::MomentumResidualFactor factor(keys, *pendingMomentumMeasurement_,
                                                                 pendingMomentumContactIds_);
  estimatedJointTorqueResidual_ = factor.unwhitenedError(estimate) / pendingMomentumMeasurement_->dt_;
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
    return getFsWorldKinematics(ctl, currentRobot, contact.fsName());
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
  const mc_rbdyn::ForceSensor & fs = ctl.robot().forceSensor(fsName);

  const so::kine::Kinematics worldFbKine =
      conversions::kinematics::fromSva(currentRobot.posW(), currentRobot.velW(), true);

  const sva::PTransformd bodyFsPose = fs.X_p_f();
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
                                                         const sva::ForceVecd & measuredWrench,
                                                         const so::kine::Kinematics * contactSensorKine)
{
  if(contactSensorKine == nullptr)
  {
    // if the transformation from the sensor to the contact is not given, we assume that the wrench was directly given
    // in the frame of the contact
    contact.contactWrenchVector_.segment<3>(0) = measuredWrench.force(); // retrieving the force measurement
    contact.contactWrenchVector_.segment<3>(3) = measuredWrench.moment(); // retrieving the torque measurement
  }
  else
  { // expressing the force measurement in the frame of the contact
    contact.contactWrenchVector_.segment<3>(0) = contactSensorKine->orientation * measuredWrench.force();

    // expressing the torque measurement in the frame of the surface
    contact.contactWrenchVector_.segment<3>(3) =
        contactSensorKine->orientation * measuredWrench.moment()
        + contactSensorKine->position().cross(contact.contactWrenchVector_.segment<3>(0));
  }
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

  // we get the reference position of the contact by removing the contribution of the visco-elastic model
  worldRestPose.position = worldContactKine.orientation.toMatrix3() * contactFlexibilities_.at(0).inverse()
                               * (contactForceMeas
                                  + worldContactKine.orientation.toMatrix3().transpose() * contactFlexibilities_.at(2)
                                        * worldContactKine.linVel())
                           + worldContactKine.position();

  /* We get the reference orientation of the contact by removing the contribution of the visco-elastic model */
  // difference between the reference orientation and the real one, obtained from the visco-elastic model
  so::Vector3 flexRotDiff = -2 * worldContactKine.orientation.toMatrix3() * contactFlexibilities_.at(1).inverse()
                            * (contactTorqueMeas
                               + worldContactKine.orientation.toMatrix3().transpose() * contactFlexibilities_.at(3)
                                     * worldContactKine.angVel());

  // axis of the rotation
  so::Vector3 flexRotAxis = flexRotDiff / flexRotDiff.norm();

  double diffNorm = flexRotDiff.norm() / 2;

  if(diffNorm > 1.0) { diffNorm = 1.0; }
  else if(diffNorm < -1.0) { diffNorm = -1.0; }

  double flexRotAngle = std::asin(diffNorm);

  // angle axis representation of the rotation due to the visco-elastic model
  Eigen::AngleAxisd flexRotAngleAxis(flexRotAngle, flexRotAxis);
  // matrix representation of the rotation due to the visco-elastic model
  so::Matrix3 flexRotMatrix = so::kine::Orientation(flexRotAngleAxis).toMatrix3();
  worldRestPose.orientation = so::Matrix3(flexRotMatrix.transpose() * worldContactKine.orientation.toMatrix3());

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
                                         const std::array<double, 4> & initNoises,
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

  contact.fsName(ctl.robot().indirectSurfaceForceSensor(contact.surfaceName()).name());

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
  const std::array<double, 4> * initNoise;

  if(observer_.getActiveContacts().empty()) // The initial noise on the pose of the contact depending on
                                            // whether another contact is already set or not
  {
    initNoise = &contactInitNoises_first_;
  }
  else { initNoise = &contactInitNoises_; }

  auto onNewContact = [this, &ctl, &logger, &initNoise](KoContactWithSensor & newContact)
  { setNewContact(ctl, newContact, *initNoise, logger); };
  auto onMaintainedContact = [this, &ctl](KoContactWithSensor & maintainedContact)
  { updateContact(ctl, maintainedContact); };
  auto onRemovedContact = [this, &logger](KoContactWithSensor & removedContact)
  {
    observer_.removeContact(removedContact.id());

    if(withDebugLogs_)
    {
      removeContactLogEntries(logger, removedContact);
      removeContactMeasurementsLogEntries(logger, removedContact);
    }
    maintainedContacts_.erase(removedContact.id());
  };

  // action to execute when a contact is added to the manager during the run, which happens when the contact detection
  // is using the solver.
  auto onAddedContact = [this, &ctl, &logger](KoContactWithSensor & addedContact)
  {
    observer_.addContactToList(addedContact.id(), contactFlexibilities_, contactProcessNoises_, contactMeasNoises_);
    addContactToGui(ctl, addedContact, logger);
    if(ctl.robot(robot_).frame(addedContact.surfaceName()).hasForceSensor() == false)
    {
      mc_rtc::log::warning(
          "The surface given for the contact detection is not associated to a force sensor, it will be ignored.");
    }
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
    logger.addLogEntry(category_ + "_MEKF_inputs_jointTorque_" + jointName, [this, i]() -> double
                       { return i < static_cast<size_t>(inputJointTorques_.size()) ? inputJointTorques_(i) : 0.0; });
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

      if(ctl.realRobot().hasBody("LeftFoot"))
      {
        logger.addLogEntry(category_ + "_realRobot_LeftFoot",
                           [&ctl]() { return ctl.realRobot().frame("LeftFoot").position(); });
      }

      if(ctl.realRobot().hasBody("RightFoot"))
      {
        logger.addLogEntry(category_ + "_realRobot_RightFoot",
                           [&ctl]() { return ctl.realRobot().frame("RightFoot").position(); });
      }

      if(ctl.realRobot().hasBody("LeftHand"))
      {
        logger.addLogEntry(category_ + "_realRobot_LeftHand",
                           [&ctl]() { return ctl.realRobot().frame("LeftHand").position(); });
      }
      if(ctl.realRobot().hasBody("RightHand"))
      {
        logger.addLogEntry(category_ + "_realRobot_RightHand",
                           [&ctl]() { return ctl.realRobot().frame("RightHand").position(); });
      }
      if(ctl.robot().hasBody("LeftFoot"))
      {
        logger.addLogEntry(category_ + "_ctlRobot_LeftFoot",
                           [&ctl]() { return ctl.robot().frame("LeftFoot").position(); });
      }
      if(ctl.robot().hasBody("RightFoot"))
      {
        logger.addLogEntry(category_ + "_ctlRobot_RightFoot",
                           [&ctl]() { return ctl.robot().frame("RightFoot").position(); });
      }

      if(ctl.robot().hasBody("LeftHand"))
      {
        logger.addLogEntry(category_ + "_ctlRobot_LeftHand",
                           [&ctl]() { return ctl.robot().frame("LeftHand").position(); });
      }

      if(ctl.robot().hasBody("category"))
      {
        logger.addLogEntry(category_ + "_ctlRobot_RightHand",
                           [&ctl]() { return ctl.robot().frame("RightHand").position(); });
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
