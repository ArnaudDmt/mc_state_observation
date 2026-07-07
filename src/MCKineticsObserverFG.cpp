/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/logging.h>

#include <array>
#include <gtsam/geometry/Pose3.h>
#include <mc_state_observation/MCKineticsObserverFG.h>
#include <mc_state_observation/gui_helpers.h>

#include <RBDyn/ID.h>

#include <mc_state_observation/conversions/kinematics.h>
#include <optional>

using namespace ko_fg;

namespace so = stateObservation;
namespace mc_state_observation
{
MCKineticsObserverFG::MCKineticsObserverFG(const std::string & type, double dt)
: mc_observers::Observer(type, dt), observer_(0.05), removeWrenchOffset_(false)
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

  zeroPose_.translation().setZero();
  zeroPose_.rotation().setIdentity();
  zeroMotion_.linear().setZero();
  zeroMotion_.angular().setZero();

  // we set the desired type of odometry
  auto leggedOdomConfig = config("leggedOdometry");
  std::string typeOfOdometry = static_cast<std::string>(leggedOdomConfig("odometryType"));
  odometryType_ = so::odometry::stringToOdometryType(typeOfOdometry);

  config("withDebugLogs", withDebugLogs_);

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
  contactInitNoises_first_.at(0) = 0.0;
  contactInitNoises_first_.at(1) = 0.0;

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
    jointTorquesConfig("noise", jointTorqueNoise_);
    jointTorquesConfig("residualInitNoise", jointTorqueResidualInitNoise_);
    jointTorquesConfig("residualProcessNoise", jointTorqueResidualProcessNoise_);
    jointTorquesConfig("finiteDiffStep", jointTorqueFiniteDiffStep_);
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
  ctl.gui()->addElement(
      {"Robots"}, mc_rtc::gui::Robot(name(), [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));
  ctl.gui()->addElement({"Robots"},
                        mc_rtc::gui::Robot("Real", [&ctl]() -> const mc_rbdyn::Robot & { return ctl.realRobot(); }));

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

  observer_.init(mass_, initState, initNoises_, processNoises_, imuNoises_, std::nullopt);

  if(useJointTorqueMeasurements_)
  {
    size_t jointTorqueDim = 0;
    for(const auto & jointName : realRobot.refJointOrder())
    {
      const int jointIndex = realRobot.mb().jointIndexByName(jointName);
      jointTorqueDim += static_cast<size_t>(realRobot.mb().joint(jointIndex).dof());
    }

    if(jointTorqueDim > 0)
    {
      observer_.configureJointTorqueResidual(
          jointTorqueDim, Vector::Zero(jointTorqueDim),
          Vector::Constant(jointTorqueDim, jointTorqueResidualInitNoise_),
          Vector::Constant(jointTorqueDim, jointTorqueResidualProcessNoise_));
      measuredJointTorques_ = Vector::Zero(jointTorqueDim);
      modelJointTorques_ = Vector::Zero(jointTorqueDim);
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

  // force measurements from sensor that are not associated to a currently set contact are given to the Kinetics
  // Observer as inputs.
  inputWrench_ = inputAdditionalWrench(ctl, robot);

  Matrix3 inertiaMatrix = computeCentroidalInertia(inputRobot.mb(), inputRobot.mbc(), fbCentroidKine_.position());

  Vector3 angularMomentum =
      rbd::computeCentroidalMomentum(inputRobot.mb(), inputRobot.mbc(), fbCentroidKine_.position()).moment();
  Vector3 angularMomentum_d = rbd::computeCentroidalMomentumDot(inputRobot.mb(), inputRobot.mbc(),
                                                                fbCentroidKine_.position(), fbCentroidKine_.linVel())
                                  .moment();

  observer_.setInput(dt_, inertiaMatrix, angularMomentum, inputWrench_.segment(0, 3), inputWrench_.segment(3, 3));

  // update of the contacts
  updateContacts(ctl, logger);

  /** Accelerometers **/
  updateIMUs(robot, inputRobot);

  if(useJointTorqueMeasurements_) { updateJointTorqueMeasurement(realRobot, inputRobot); }

  observer_.runIteration(k_);

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

  // Estimated kinematics of the floating base in the world frame

  X_0_fb_.rotation() = est_worldFbKine_.orientation.toMatrix3().transpose();
  X_0_fb_.translation() = est_worldFbKine_.position();

  /* Bring velocity of the IMU to the origin of the joint : we want the
   * velocity of joint 0, so stop one before the first joint */

  v_fb_0_.angular() = est_worldFbKine_.angVel();
  v_fb_0_.linear() = est_worldFbKine_.linVel();

  a_fb_0_.angular() = est_worldFbKine_.angAcc();
  a_fb_0_.linear() = est_worldFbKine_.linAcc();

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

so::kine::Kinematics MCKineticsObserverFG::centroidStateToFloatingBaseKinematics(
    const ko_fg::LocKinematics & centroidKine) const
{
  return fgLocKineToSoKine(centroidKine) * centroidFbKine_;
}

void MCKineticsObserverFG::setFloatingBaseKinematics(mc_rbdyn::Robot & robot,
                                                     const so::kine::Kinematics & fbKine) const
{
  robot.posW(sva::PTransformd(fbKine.orientation.toMatrix3().transpose(), fbKine.position()));
  robot.velW(sva::MotionVecd(fbKine.angVel(), fbKine.linVel()));
  robot.accW(sva::MotionVecd(fbKine.angAcc(), fbKine.linAcc()));

  robot.forwardKinematics();
  robot.forwardVelocity();
  robot.forwardAcceleration();
}

Eigen::VectorXd MCKineticsObserverFG::measuredJointTorqueVector(const mc_rbdyn::Robot & robot) const
{
  const auto & tau = robot.jointTorques();
  if(tau.empty()) { return Eigen::VectorXd(); }

  return Eigen::Map<const Eigen::VectorXd>(tau.data(), static_cast<Eigen::Index>(tau.size()));
}

Eigen::VectorXd MCKineticsObserverFG::modelJointTorqueVector(const mc_rbdyn::Robot & robot) const
{
  Eigen::Index dim = 0;
  for(const auto & jointName : robot.refJointOrder())
  {
    const int jointIndex = robot.mb().jointIndexByName(jointName);
    dim += robot.mb().joint(jointIndex).dof();
  }

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

Eigen::VectorXd MCKineticsObserverFG::computeInverseDynamicsTorque(
    mc_rbdyn::Robot & robot,
    const ko_fg::LocKinematics & centroidKine,
    const std::unordered_map<unsigned, so::Vector6> & contactWrenches) const
{
  rbd::MultiBodyConfig savedMbc = robot.mbc();

  const so::kine::Kinematics fbKine = centroidStateToFloatingBaseKinematics(centroidKine);
  setFloatingBaseKinematics(robot, fbKine);

  robot.mbc().force.assign(static_cast<size_t>(robot.mb().nrBodies()), sva::ForceVecd::Zero());

  for(const auto & [contactId, wrench] : contactWrenches)
  {
    const auto contactIt = maintainedContacts_.find(contactId);
    if(contactIt == maintainedContacts_.end()) { continue; }

    const KoContactWithSensor & contact = *contactIt->second;
    const auto & surface = robot.surface(contact.surfaceName());
    const int bodyIndex = robot.bodyIndexByName(surface.bodyName());

    const so::kine::Kinematics worldContactKine = fbKine * contact.fbContactKine_;
    const Eigen::Matrix3d R_0_c = worldContactKine.orientation.toMatrix3();
    const Eigen::Vector3d forceWorld = R_0_c * wrench.segment<3>(0);
    const Eigen::Vector3d momentAtContactWorld = R_0_c * wrench.segment<3>(3);
    const Eigen::Vector3d bodyPosWorld = robot.mbc().bodyPosW[static_cast<size_t>(bodyIndex)].translation();
    const Eigen::Vector3d momentAtBodyWorld =
        momentAtContactWorld + (worldContactKine.position() - bodyPosWorld).cross(forceWorld);

    robot.mbc().force[static_cast<size_t>(bodyIndex)] += sva::ForceVecd(momentAtBodyWorld, forceWorld);
  }

  rbd::InverseDynamics inverseDynamics(robot.mb());
  inverseDynamics.inverseDynamics(robot.mb(), robot.mbc());

  Eigen::VectorXd tau = modelJointTorqueVector(robot);
  robot.mbc() = savedMbc;
  return tau;
}

ko_fg::JointTorqueMeasurement MCKineticsObserverFG::makeJointTorqueMeasurement(const mc_rbdyn::Robot & measRobot,
                                                                               mc_rbdyn::Robot & dynamicsRobot)
{
  ko_fg::JointTorqueMeasurement measurement;

  const ko_fg::LocKinematics nominalKine = observer_.getCurrentState().kine_;
  const std::unordered_map<unsigned, so::Vector6> noContactWrenches;
  const Eigen::VectorXd tau0 = computeInverseDynamicsTorque(dynamicsRobot, nominalKine, noContactWrenches);
  const Eigen::Index dim = tau0.size();
  const double eps = jointTorqueFiniteDiffStep_;

  measurement.measuredTorque_ = measuredJointTorqueVector(measRobot);
  measurement.modelTorqueNoContact_ = tau0;
  measurement.linearizationPose_ = nominalKine.pose();
  measurement.linearizationLinVel_ = nominalKine.linVel();
  measurement.linearizationAngVel_ = nominalKine.angVel();
  measurement.linearizationLinAcc_ = nominalKine.linAcc();
  measurement.linearizationAngAcc_ = nominalKine.angAcc();
  measurement.poseJacobian_.setZero(dim, 6);
  measurement.linVelJacobian_.setZero(dim, 3);
  measurement.angVelJacobian_.setZero(dim, 3);
  measurement.linAccJacobian_.setZero(dim, 3);
  measurement.angAccJacobian_.setZero(dim, 3);
  measurement.noise_ = ko_fg::makeNoise(Vector::Constant(dim, jointTorqueNoise_));

  for(Eigen::Index col = 0; col < 6; ++col)
  {
    ko_fg::LocKinematics perturbed = nominalKine;
    Vector delta = Vector::Zero(6);
    delta(col) = eps;
    perturbed.pose(gtsam::traits<ko_fg::LocalPose3>::Retract(nominalKine.pose(), delta));
    measurement.poseJacobian_.col(col) =
        (computeInverseDynamicsTorque(dynamicsRobot, perturbed, noContactWrenches) - tau0) / eps;
  }

  for(Eigen::Index col = 0; col < 3; ++col)
  {
    ko_fg::LocKinematics perturbed = nominalKine;
    Eigen::Vector3d value = nominalKine.linVel();
    value(col) += eps;
    perturbed.linVel(value);
    measurement.linVelJacobian_.col(col) =
        (computeInverseDynamicsTorque(dynamicsRobot, perturbed, noContactWrenches) - tau0) / eps;

    perturbed = nominalKine;
    value = nominalKine.angVel();
    value(col) += eps;
    perturbed.angVel(value);
    measurement.angVelJacobian_.col(col) =
        (computeInverseDynamicsTorque(dynamicsRobot, perturbed, noContactWrenches) - tau0) / eps;

    perturbed = nominalKine;
    value = nominalKine.linAcc();
    value(col) += eps;
    perturbed.linAcc(value);
    measurement.linAccJacobian_.col(col) =
        (computeInverseDynamicsTorque(dynamicsRobot, perturbed, noContactWrenches) - tau0) / eps;

    perturbed = nominalKine;
    value = nominalKine.angAcc();
    value(col) += eps;
    perturbed.angAcc(value);
    measurement.angAccJacobian_.col(col) =
        (computeInverseDynamicsTorque(dynamicsRobot, perturbed, noContactWrenches) - tau0) / eps;
  }

  for(const auto & [contactId, contact] : observer_.getActiveContacts())
  {
    Eigen::MatrixXd forceJac = Eigen::MatrixXd::Zero(dim, 3);
    Eigen::MatrixXd momentJac = Eigen::MatrixXd::Zero(dim, 3);

    for(Eigen::Index col = 0; col < 3; ++col)
    {
      std::unordered_map<unsigned, so::Vector6> contactWrenches;
      so::Vector6 wrench = so::Vector6::Zero();
      wrench(col) = eps;
      contactWrenches[static_cast<unsigned>(contactId)] = wrench;
      forceJac.col(col) = (computeInverseDynamicsTorque(dynamicsRobot, nominalKine, contactWrenches) - tau0) / eps;

      wrench.setZero();
      wrench(3 + col) = eps;
      contactWrenches[static_cast<unsigned>(contactId)] = wrench;
      momentJac.col(col) = (computeInverseDynamicsTorque(dynamicsRobot, nominalKine, contactWrenches) - tau0) / eps;
    }

    measurement.contactForceJacobian_[contactId] = forceJac;
    measurement.contactMomentJacobian_[contactId] = momentJac;
  }

  return measurement;
}

void MCKineticsObserverFG::updateJointTorqueMeasurement(const mc_rbdyn::Robot & measRobot,
                                                        mc_rbdyn::Robot & dynamicsRobot)
{
  measuredJointTorques_ = measuredJointTorqueVector(measRobot);
  if(measuredJointTorques_.size() == 0) { return; }

  ko_fg::JointTorqueMeasurement measurement = makeJointTorqueMeasurement(measRobot, dynamicsRobot);
  if(measurement.measuredTorque_.size() != measurement.modelTorqueNoContact_.size())
  {
    mc_rtc::log::warning("[{}]: Joint torque measurement size ({}) does not match model torque size ({}).", name(),
                         measurement.measuredTorque_.size(), measurement.modelTorqueNoContact_.size());
    return;
  }

  modelJointTorques_ = measurement.modelTorqueNoContact_;
  observer_.updateJointTorqueMeasurement(measurement);
}

const so::kine::Kinematics MCKineticsObserverFG::getContactWorldKinematics(const KoContactWithSensor & contact,
                                                                           const mc_rbdyn::Robot & currentRobot,
                                                                           const mc_rbdyn::ForceSensor & fs,
                                                                           const sva::ForceVecd * measuredWrench)
{
  /*
  Can be used with inputRobot, a virtual robot corresponding to the real robot whose floating base's frame is
  superimposed with the world frame. Getting kinematics associated to the inputRobot inside the world frame is the same
  as getting the same kinematics of the real robot inside the frame of its floating base, which is needed for the inputs
  of the Kinetics Observer. This allows to use the basic mc_rtc functions directly giving kinematics in the world frame
  and not do the conversion: initial frame -> world + world -> floating base as the latter is zero.
  */

  so::kine::Kinematics worldContactKine;

  const sva::PTransformd & bodyContactSensorPose = fs.X_p_f();
  so::kine::Kinematics bodyContactSensorKine =
      conversions::kinematics::fromSva(bodyContactSensorPose, so::kine::Kinematics::Flags::vel);

  // kinematics of the sensor's parent body in the world frame
  so::kine::Kinematics worldBodyKine = conversions::kinematics::fromSva(
      currentRobot.mbc().bodyPosW[currentRobot.bodyIndexByName(fs.parentBody())],
      currentRobot.mbc().bodyVelW[currentRobot.bodyIndexByName(fs.parentBody())], true);

  so::kine::Kinematics worldSensorKine = worldBodyKine * bodyContactSensorKine;

  if(contactsDetector_.getContactsDetection() == KoContactsDetector::ContactsDetection::Sensors)
  {
    // If the contact is detecting using thresholds, we will then consider the sensor frame as
    // the contact surface frame directly.
    worldContactKine = worldSensorKine;
    if(measuredWrench != nullptr)
    {
      KoContactWithSensor & nc_contact = const_cast<KoContactWithSensor &>(contact);
      updateContactForceMeasurement(nc_contact, *measuredWrench);
    }
  }
  else // the kinematics of the contacts are the ones of the surface.
  {
    // pose of the surface in the world / floating base's frame
    sva::PTransformd worldContactPose = currentRobot.surfacePose(contact.surfaceName());
    // Kinematics of the surface in the world / floating base's frame
    worldContactKine = conversions::kinematics::fromSva(worldContactPose, so::kine::Kinematics::Flags::vel);

    // the kinematics of the contacts are the ones of the surface, but we must transport the measured wrench
    const mc_rbdyn::Surface & contactSurface = currentRobot.surface(contact.surfaceName());

    sva::PTransformd bodyContactPose = contactSurface.X_b_s();
    so::kine::Kinematics bodyContactKine =
        conversions::kinematics::fromSva(bodyContactPose, so::kine::Kinematics::Flags::vel);

    so::kine::Kinematics worldBodyKine = conversions::kinematics::fromSva(
        currentRobot.mbc().bodyPosW[currentRobot.bodyIndexByName(contactSurface.bodyName())],
        currentRobot.mbc().bodyVelW[currentRobot.bodyIndexByName(contactSurface.bodyName())], true);

    worldContactKine = worldBodyKine * bodyContactKine;

    if(measuredWrench != nullptr)
    {
      KoContactWithSensor & nc_contact = const_cast<KoContactWithSensor &>(contact);
      nc_contact.contactSensorKine_ = worldContactKine.getInverse() * worldSensorKine;
      updateContactForceMeasurement(nc_contact, *measuredWrench, &contact.contactSensorKine_);
    }
  }

  return worldContactKine;
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

so::kine::Kinematics MCKineticsObserverFG::getOdometryWorldContactRest(const mc_control::MCController & ctl,
                                                                       KoContactWithSensor & contact,
                                                                       const so::kine::Kinematics & worldContactKine)
{
  so::kine::Kinematics worldRestPose;

  const auto & robot = ctl.robot(robot_);
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
    const mc_rbdyn::ForceSensor & fs = robot.forceSensor(contact.fsName_);

    // kinematics of the contact of the control robot in the world frame
    so::kine::Kinematics worldContactKineControl = getContactWorldKinematics(contact, robot, fs);

    // the reference altitude of the contact is the one in the control robot
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

  // As used on input robot, returns the kinematics of the contact in the frame of the floating base. Also expresses the
  // measured wrench in the frame of the contact.
  contact.fbContactKine_ = getContactWorldKinematics(contact, inputRobot, fs, &measuredWrench);

  if(odometryType_ != so::odometry::OdometryType::None) // the Kinetics Observer performs odometry. The estimated
                                                        // state is used to provide the new contacts references.
  {
    so::kine::Kinematics worldContactKine = est_worldFbKine_ * contact.fbContactKine_;
    Pose3_RI worldContactPose(gtsam::Rot3(worldContactKine.orientation.toMatrix3()), worldContactKine.position());

    observer_.addContact(contact.id(), worldContactPose, worldContactKine.linVel(), worldContactKine.angVel(),
                         contact.contactWrenchVector_, initNoises, k_);
  }
  else // we don't perform odometry, the reference pose of the contact is its pose in the control robot
  {
    so::kine::Kinematics worldContactKineRef = getContactWorldKinematics(contact, robot, fs);

    Pose3_RI worldContactPose(gtsam::Rot3(worldContactKineRef.orientation.toMatrix3()), worldContactKineRef.position());
    observer_.addContact(contact.id(), worldContactPose, contact.contactWrenchVector_, contactInitNoises_, k_);
  }

  stateObservation::kine::Kinematics centroidContactKine = centroidFbKine_ * contact.fbContactKine_;

  if(contact.sensorEnabled_) // the force sensor attached to the contact is used in
                             // the correction by the Kinetics Observer.
  {
    observer_.updateContact(contact.id(), contact.contactWrenchVector_.segment(0, 3),
                            contact.contactWrenchVector_.segment(3, 3),
                            gtsam::Pose3(gtsam::Rot3(contact.fbContactKine_.orientation.toMatrix3()),
                                         gtsam::Point3(centroidContactKine.position())),
                            centroidContactKine.linVel(), centroidContactKine.angVel());
  }
  else
  {
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

  // As used on input robot, returns the kinematics of the contact in the frame of the floating base. Also expresses the
  // measured wrench in the frame of the contact.
  contact.fbContactKine_ = getContactWorldKinematics(contact, inputRobot, fs, &measuredWrench);
  contact.centroidContactKine_ = centroidFbKine_ * contact.fbContactKine_;

  gtsam::Pose3 centroidContactPose(gtsam::Rot3(contact.centroidContactKine_.orientation.toMatrix3()),
                                   contact.centroidContactKine_.position());

  if(contact.sensorEnabled_) // the force sensor attached to the contact is used in
                             // the correction by the Kinetics Observer.
  {
    observer_.updateContact(contact.id(), contact.contactWrenchVector_.segment(0, 3),
                            contact.contactWrenchVector_.segment(3, 3), centroidContactPose,
                            contact.centroidContactKine_.linVel(), contact.centroidContactKine_.angVel());
  }
  else
  {
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
  {
    updateContact(ctl, maintainedContact);
    maintainedContacts_.insert({maintainedContact.id(), &maintainedContact});
  };
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

  logger.addLogEntry(category_ + "_MEKF_measurements_jointTorque_measured",
                     [this]() -> const Eigen::VectorXd & { return measuredJointTorques_; });
  logger.addLogEntry(category_ + "_MEKF_measurements_jointTorque_modelNoContact",
                     [this]() -> const Eigen::VectorXd & { return modelJointTorques_; });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_jointTorqueResidual",
                     [this]() -> const Eigen::VectorXd & { return observer_.getCurrentState().tauDisturbActuated_; });

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
        const auto & robot = ctl.robot(robot_);
        const auto & realRobot = ctl.realRobot(robot_);
        return getContactWorldKinematics(contact, realRobot, robot.forceSensor(contact.fsName_)).position();
      });

  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.surfaceName() + "_ctlRobot_position", &contact,
                     [this, &contact, &ctl]() -> Eigen::Vector3d
                     {
                       const auto & robot = ctl.robot(robot_);
                       return getContactWorldKinematics(contact, robot, robot.forceSensor(contact.fsName_)).position();
                     });

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
