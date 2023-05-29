#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <iostream>
#include <mc_state_observation/TiltObserver.h>
#include <mc_state_observation/gui_helpers.h>
#include <mc_state_observation/svaKinematicsConversion.h>

namespace mc_state_observation
{

namespace so = stateObservation;

TiltObserver::TiltObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), estimator_(alpha_, beta_, gamma_)
{
  estimator_.setSamplingTime(dt_);
  xk_.resize(9);
  xk_ << so::Vector3::Zero(), so::Vector3::Zero(), so::Vector3(0, 0, 1); // so::Vector3(0.49198, 0.66976, 0.55622);
  estimator_.setState(xk_, 0);
}

void TiltObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  updateRobotName_ = robot_;
  imuSensor_ = config("imuSensor", ctl.robot().bodySensor().name());
  updateSensorName_ = imuSensor_;
  config("alpha", alpha_);
  config("beta", beta_);
  config("gamma", gamma_);
  // anchorFrameFunction_ = config("anchorFrameFunction", name() + "::" + ctl.robot(robot_).name());
  anchorFrameFunction_ = "KinematicAnchorFrame::" + ctl.robot(robot_).name();
  config("updateRobot", updateRobot_);
  config("updateRobotName", updateRobotName_);
  config("updateSensor", updateSensor_);
  config("updateSensorName", updateSensorName_);
  desc_ = fmt::format("{}", name_);
}

void TiltObserver::reset(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  my_robots_->robotCopy(realRobot, "inputRobot");
  ctl.gui()->addElement(
      {"Robots"},
      mc_rtc::gui::Robot("TiltEstimator", [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));
}

bool TiltObserver::run(const mc_control::MCController & ctl)
{
  estimator_.setAlpha(alpha_);
  estimator_.setBeta(beta_);
  estimator_.setGamma(gamma_);

  if(!ctl.datastore().has(anchorFrameFunction_))
  {
    error_ = fmt::format(
        "Observer {} requires a \"{}\" function in the datastore to provide the observer's kinematic anchor frame.\n"
        "Please refer to https://jrl-umi3218.github.io/mc_rtc/tutorials/recipes/observers.html for further details.",
        name(), anchorFrameFunction_);
    return false;
  }

  // Anchor frame defined w.r.t control robot
  // XXX what if the feet are being moved by the stabilizer?
  X_0_C_ = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, ctl.robot(robot_));
  // we want in this anchor frame:
  // - position of the IMU
  // - orientation of the IMU
  // - linear velocity of the imu
  // - angular velocity of the imu
  // - linear velocity of the anchor frame in the world of the control robot (derivative?)

  so::kine::Kinematics worldAnchorKine = svaKinematicsConversion::poseFromSva(X_0_C_);

  const auto & robot = ctl.robot(robot_);
  const auto & imu = robot.bodySensor(imuSensor_);
  auto X_0_FB = robot.posW();

  so::kine::Kinematics worldFBKine = svaKinematicsConversion::poseFromSva(X_0_FB);

  const sva::PTransformd & imuXbs = imu.X_b_s();
  so::kine::Kinematics bodyImuKine = svaKinematicsConversion::poseFromSva(imuXbs);

  const sva::PTransformd & bodyW = robot.bodyPosW(imu.parentBody());
  so::kine::Kinematics worldBodyKine = svaKinematicsConversion::poseFromSva(bodyW);

  // auto RF = anchorFrame.oriention().transpose(); // orientation of the control anchor frame
  // auto RB = robot.posW().orientation().transpose(); // orientation of the control floating base

  // Compute velocity of the imu in the control frame
  auto v_0_imuParent = robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())];
  worldBodyKine.linVel = v_0_imuParent.linear();
  worldBodyKine.angVel = v_0_imuParent.angular();

  so::kine::Kinematics worldImuKine = worldBodyKine * bodyImuKine;

  // Pose of the imu in the control frame
  so::kine::Kinematics anchorImuKine = worldAnchorKine.getInverse() * worldImuKine;

  so::kine::Kinematics fbImuKine = worldFBKine.getInverse() * worldImuKine;

  BOOST_ASSERT((anchorImuKine.linVel.isSet() || anchorImuKine.angVel.isSet()) && "vels were not computed");

  estimator_.setSensorPositionInC(anchorImuKine.position());
  estimator_.setSensorOrientationInC(anchorImuKine.orientation.toMatrix3());
  estimator_.setSensorLinearVelocityInC(anchorImuKine.linVel());
  estimator_.setSensorAngularVelocityInC(anchorImuKine.angVel());

  worldAnchorLinVel = 1.0 / estimator_.getSamplingTime() * (worldAnchorKine.position() - previousWorldAnchorePosition);

  worldAnchorLocalLinVel = worldAnchorKine.orientation.toMatrix3().transpose() * worldAnchorLinVel;

  estimator_.setControlOriginVelocityInW(worldAnchorLocalLinVel);

  previousWorldAnchorePosition = worldAnchorKine.position();

  auto k = estimator_.getCurrentTime();

  estimator_.setMeasurement(imu.linearAcceleration(), imu.angularVelocity(), k + 1);

  xk_ = estimator_.getEstimatedState(k + 1);

  so::Vector3 tilt = xk_.tail(3);

  // Orientation of body?
  estimatedRotationIMU_ = so::kine::mergeTiltWithYaw(tilt, worldImuKine.orientation.toMatrix3());

  // update the velocities as MotionVecd for the logs
  imuVelC_.linear() = anchorImuKine.linVel();
  imuVelC_.angular() = anchorImuKine.angVel();

  // update the pose as PTransformd for the logs
  X_C_IMU_.translation() = anchorImuKine.position();
  X_C_IMU_.rotation() = anchorImuKine.orientation.toMatrix3().transpose();

  /* Update of the observed robot */
  my_robots_->robot().mbc().q = ctl.realRobot().mbc().q;
  update(my_robots_->robot(), ctl);

  return true;
}

void TiltObserver::update(mc_control::MCController & ctl)
{
  if(updateRobot_)
  {
    auto & robot = ctl.realRobots().robot(updateRobot_);
    auto posW = robot.posW();
    R_0_fb_ = estimatedRotationIMU_ * fbImuKine_.orientation.toMatrix3();
    posW.rotation() = R_0_fb_.transpose();
    mc_rtc::log::info("update robot: {}", mc_rbdyn::rpyFromMat(posW.rotation()));
    posW.translation() = Eigen::Vector3d::Zero();
    robot.posW(posW);
    robot.forwardKinematics();
  }

  if(updateSensor_)
  {
    // auto & imu = ctl.robot(robot_).bodySensor(imuSensor_);
    auto & imu = const_cast<mc_rbdyn::BodySensor &>(ctl.robot(robot_).bodySensor(imuSensor_));
    auto & rimu = const_cast<mc_rbdyn::BodySensor &>(ctl.realRobot(robot_).bodySensor(imuSensor_));

    imu.orientation(Eigen::Quaterniond{estimatedRotationIMU_});
    rimu.orientation(Eigen::Quaterniond{estimatedRotationIMU_});
  }
}

void TiltObserver::update(mc_rbdyn::Robot & robot, const mc_control::MCController & ctl)
{
  const auto & realRobot = ctl.realRobot(robot_);
  R_0_fb_ = estimatedRotationIMU_ * fbImuKine_.orientation.toMatrix3();
  poseForDisplay = realRobot.posW();
  poseForDisplay.rotation() = R_0_fb_.transpose();
  robot.posW(poseForDisplay);
}

void TiltObserver::addToLogger(const mc_control::MCController &, mc_rtc::Logger & logger, const std::string & category)
{
  logger.addLogEntry(category + "_imuVelC", [this]() -> const sva::MotionVecd & { return imuVelC_; });
  logger.addLogEntry(category + "_imuPoseC", [this]() -> const sva::PTransformd & { return X_C_IMU_; });
  logger.addLogEntry(category + "_imuEstRotW", [this]() { return Eigen::Quaterniond{estimatedRotationIMU_}; });
  logger.addLogEntry(category + "_controlAnchorFrame", [this]() -> const sva::PTransformd & { return X_0_C_; });
  logger.addLogEntry(category + "_displayedWorldFbPose",
                     [this]() -> const sva::PTransformd & { return poseForDisplay; });
  logger.addLogEntry(category + "_anchorFramelinVel_global",
                     [this]() -> const so::Vector3 & { return worldAnchorLinVel; });
  logger.addLogEntry(category + "_anchorFramelinVel_local",
                     [this]() -> const so::Vector3 & { return worldAnchorLocalLinVel; });
}

void TiltObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_imuVelC");
  logger.removeLogEntry(category + "_imuPoseC");
  logger.removeLogEntry(category + "_imuEstRotW");
  logger.removeLogEntry(category + "_controlAnchorFrame");
}

void TiltObserver::addToGUI(const mc_control::MCController & ctl,
                            mc_rtc::gui::StateBuilder & gui,
                            const std::vector<std::string> & category)
{
  using namespace mc_state_observation::gui;
  gui.addElement(category, make_input_element("alpha", alpha_), make_input_element("beta", beta_),
                 make_input_element("gamma", gamma_));
}

} // namespace mc_state_observation
EXPORT_OBSERVER_MODULE("Tilt", mc_state_observation::TiltObserver)
