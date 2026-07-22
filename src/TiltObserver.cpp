#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>

#include <mc_state_observation/conversions/kinematics.h>

#include <mc_state_observation/TiltObserver.h>
#include <mc_state_observation/gui_helpers.h>

namespace mc_state_observation
{

namespace so = stateObservation;

TiltObserver::TiltObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), estimator_(alpha_, beta_, gamma_, dt)
{
}

void TiltObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  imuSensor_ = config("imuSensor", ctl.robot().bodySensor().name());

  config("updateRobot", updateRobot_);
  config("updateSensor", updateSensor_);
  config("withDebugLogs", withDebugLogs_);

  auto filterGainsConfig = config("filterGains");

  filterGainsConfig("initAlpha", alpha_);
  filterGainsConfig("initBeta", beta_);
  filterGainsConfig("initGamma", gamma_);

  filterGainsConfig("finalAlpha", finalAlpha_);
  filterGainsConfig("finalBeta", finalBeta_);
  filterGainsConfig("finalGamma", finalGamma_);

  // filterGainsConfig("finalEta", eta_contacts_final_);

  anchorFrameFunction_ = "KinematicAnchorFrame::" + ctl.robot(robot_).name();
  // if a user-defined anchor frame function is given, we use it instead
  if(config.has("anchorFrameFunction"))
  {
    if(ctl.datastore().has(anchorFrameFunction_))
    {
      anchorFrameFunction_ = config("anchorFrameFunction", name() + "::" + ctl.robot(robot_).name());
    }
  }
}

void TiltObserver::reset(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  my_robots_->robotCopy(realRobot, "updatedRobot");

  ctl.gui()->addElement(
      {"Robots"}, mc_rtc::gui::Robot(name(), [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));

  const auto & imu = robot.bodySensor(imuSensor_);

  yk_ = Eigen::Matrix<double, 9, 1>::Zero();

  poseW_ = realRobot.posW();
  velW_ = realRobot.velW();
  prevPoseW_ = sva::PTransformd::Identity();
  velW_ = sva::MotionVecd::Zero();

  const Eigen::Matrix3d cOri = (imu.X_b_s() * ctl.robot(robot_).bodyPosW(imu.parentBody())).rotation();
  so::Vector3 initX2 = cOri * so::Vector3::UnitZ();

  initX_ = Eigen::RowVectorXd(9);
  initX_ << so::Vector3::Zero(), initX2, initX2;
  estimator_.initEstimator(initX_);

  /* Initialization of the variables */
  worldAnchorKine_ctl_ = stateObservation::kine::Kinematics::zeroKinematics(so::kine::Kinematics::Flags::position
                                                                            | so::kine::Kinematics::Flags::linVel);
  worldAnchorKine_ = stateObservation::kine::Kinematics::zeroKinematics(so::kine::Kinematics::Flags::position
                                                                        | so::kine::Kinematics::Flags::linVel);
  imuAnchorKine_ = stateObservation::kine::Kinematics::zeroKinematics(so::kine::Kinematics::Flags::position
                                                                      | so::kine::Kinematics::Flags::linVel);

  iter_ = 0;
}

bool TiltObserver::run(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  const auto & realQ = realRobot.mbc().q;
  const auto & realAlpha = realRobot.mbc().alpha;

  std::copy(std::next(realQ.begin()), realQ.end(), std::next(my_robots_->robot("updatedRobot").mbc().q.begin()));
  std::copy(std::next(realAlpha.begin()), realAlpha.end(),
            std::next(my_robots_->robot("updatedRobot").mbc().alpha.begin()));

  my_robots_->robot("updatedRobot").mbc().q[0] = robot.mbc().q[0];
  my_robots_->robot("updatedRobot").mbc().alpha[0] = robot.mbc().alpha[0];

  my_robots_->robot("updatedRobot").forwardKinematics();
  my_robots_->robot("updatedRobot").forwardVelocity();

  if(logger.t() > 1.0)
  {
    alpha_ = finalAlpha_;
    beta_ = finalBeta_;
    gamma_ = finalGamma_;
  }

  estimator_.setAlpha(alpha_);
  estimator_.setBeta(beta_);
  estimator_.setGamma(gamma_);

  updateNecessaryFrames(ctl, my_robots_->robot("updatedRobot"));

  const auto & imu = ctl.robot(robot_).bodySensor(imuSensor_);

  auto k = estimator_.getCurrentTime();

  // computation of the local linear velocity of the IMU in the world.

  yv_ = worldImuKine_ctl_.orientation.toMatrix3().transpose() * worldAnchorKine_ctl_.linVel()
        - (imu.angularVelocity()).cross(imuAnchorKine_.position()) - imuAnchorKine_.linVel();

  estimator_.setMeasurement(yv_, imu.linearAcceleration(), imu.angularVelocity(), k + 1);
  yk_.segment(0, 3) = yv_;
  yk_.segment(3, 3) = imu.linearAcceleration();
  yk_.segment(6, 3) = imu.angularVelocity();

  // estimation of the state with the complementary filters
  xk_ = estimator_.getEstimatedState(k + 1);

  // retrieving the estimated Tilt
  so::Vector3 tilt = xk_.tail(3);

  // Once we obtain the tilt (which is required by the legged odometry, estimating only the yaw), we update the pose and
  // velocities of the floating base

  // Orientation of the imu in the world obtained from the estimated tilt and the yaw of the control robot.
  // When using odometry, the tilt will be kept but the yaw will be replaced by the one of the odometry robot.
  estimatedRotationIMU_ = so::kine::mergeTiltWithYawAxisAgnostic(tilt, worldImuKine_ctl_.orientation.toMatrix3());

  // Estimated orientation of the floating base in the world (especially the tilt)
  R_0_fb_ = estimatedRotationIMU_ * fbImuKine_.orientation.toMatrix3().transpose();

  updatePoseAndVel(xk_.head(3), imu.angularVelocity());

  iter_++;

  /* Update of the observed robot */
  my_robots_->robot().mbc().q = realRobot.mbc().q;
  update(my_robots_->robot());

  return true;
}

void TiltObserver::updateAnchorFrame(const mc_control::MCController & ctl, const mc_rbdyn::Robot & updatedRobot)
{
  const auto & robot = ctl.robot(robot_);

  // We don't use the default anchorFrameFunction because the obtained anchor position is in the shape of steps and we
  // obtain very high velocities when using finite differences

  if(!ctl.datastore().has(anchorFrameFunction_))
  {
    double leftFootRatio =
        robot.indirectSurfaceForceSensor("LeftFootCenter").wrenchWithoutGravity(ctl.realRobot()).force().z()
        / (robot.indirectSurfaceForceSensor("LeftFootCenter").wrenchWithoutGravity(ctl.realRobot()).force().z()
           + robot.indirectSurfaceForceSensor("RightFootCenter").wrenchWithoutGravity(ctl.realRobot()).force().z());

    X_0_C_ctl_ =
        sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), leftFootRatio);
    X_0_C_ = sva::interpolate(updatedRobot.surfacePose("RightFootCenter"), updatedRobot.surfacePose("LeftFootCenter"),
                              leftFootRatio);
  }
  else
  {
    X_0_C_ctl_ = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, robot);
    X_0_C_ = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, updatedRobot);
  }

  // new pose of the anchor frame in the world.
  newWorldAnchorKine_ctl_ = conversions::kinematics::fromSva(X_0_C_ctl_, so::kine::Kinematics::Flags::pose);
  newWorldAnchorKine_ = conversions::kinematics::fromSva(X_0_C_, so::kine::Kinematics::Flags::pose);

  // the velocities of the anchor frames are computed by finite differences
  worldAnchorKine_ctl_.update(newWorldAnchorKine_ctl_, ctl.timeStep,
                              so::kine::Kinematics::Flags::position | so::kine::Kinematics::Flags::linVel);
  worldAnchorKine_.update(newWorldAnchorKine_, ctl.timeStep,
                          so::kine::Kinematics::Flags::position | so::kine::Kinematics::Flags::linVel);

  // we ignore the initial outlier velocities due to the position jump
  if(iter_ < itersBeforeAnchorsVel_)
  {
    worldAnchorKine_ctl_.linVel().setZero();
    worldAnchorKine_.linVel().setZero();
  }
}

void TiltObserver::updateNecessaryFrames(const mc_control::MCController & ctl, const mc_rbdyn::Robot & updatedRobot)
{
  updateAnchorFrame(ctl, updatedRobot);

  const auto & imu = ctl.robot(robot_).bodySensor(imuSensor_);

  // pose of the floating base' frame in the world for the robot with the updated encoders
  worldFbKine_ = conversions::kinematics::fromSva(updatedRobot.posW(), updatedRobot.velW(), true);

  fbAnchorKine_ = worldFbKine_.getInverse() * worldAnchorKine_;

  // we use the imu object of control robot because the copy of BodySensor objects seems to be incomplete. Anyway we
  // use it only to get information about the parent body, which is the same as in the control robot.
  const sva::PTransformd & imuXbs = imu.X_b_s();
  so::kine::Kinematics parentImuKine =
      conversions::kinematics::fromSva(imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

  // pose of the IMU's parent body in the world for the robot with the updated encoders
  const sva::PTransformd & updatedParentPoseW = updatedRobot.bodyPosW(imu.parentBody());
  // velocity of the IMU's parent body in the world for the robot with the updated encoders
  const sva::MotionVecd & updated_v_0_imuParent =
      updatedRobot.mbc().bodyVelW[updatedRobot.bodyIndexByName(imu.parentBody())];

  // kinematics of the IMU's parent body in the world for the robot with the updated encoders
  so::kine::Kinematics worldParentKine =
      conversions::kinematics::fromSva(updatedParentPoseW, updated_v_0_imuParent, true);

  // pose and velocities of the IMU in the world frame for the robot with the updated encoders
  worldImuKine_ = worldParentKine * parentImuKine;

  // pose and velocities of the IMU in the floating base for the robot with the updated encoders
  fbImuKine_ = worldFbKine_.getInverse() * worldImuKine_;

  const auto & robot = ctl.robot(robot_);
  const sva::PTransformd & parentPoseW_ctl = robot.bodyPosW(imu.parentBody());
  // Compute velocity of the imu in the control frame
  auto & v_0_imuParent_ctl = robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())];

  so::kine::Kinematics worldParentKine_ctl = conversions::kinematics::fromSva(parentPoseW_ctl, v_0_imuParent_ctl, true);
  // pose and velocities of the IMU in the world frame
  worldImuKine_ctl_ = worldParentKine_ctl * parentImuKine;

  // new pose of the anchor frame in the IMU frame. The velocity is computed right after because we don't want to use
  // the one given by mc_rtc.
  so::kine::Kinematics newImuAnchorKine = worldImuKine_.getInverse() * worldAnchorKine_;

  // The velocities of the IMU in the world (given by mc_rtc) and the ones of the anchor frame in the world (by finite
  // differences) are not computed the same way, combining them to get the velocity of the anchor frame in the IMU
  // frame therefore leads to errors. So we "unset" the erroneous newly compute velocities to compute them by finite
  // differences from the pose of the anchor frame in the IMU.
  newImuAnchorKine.linVel.set(false);
  newImuAnchorKine.angVel.set(false);

  imuAnchorKine_.update(newImuAnchorKine, ctl.timeStep,
                        so::kine::Kinematics::Flags::position | so::kine::Kinematics::Flags::linVel);

  // we ignore the initial outlier velocity due to the position jump
  // we also reset the velocity of the anchor frame when its computation mode changes.
  if(iter_ < itersBeforeAnchorsVel_) { imuAnchorKine_.linVel().setZero(); }
}

void TiltObserver::updatePoseAndVel(const so::Vector3 & localWorldImuLinVel, const so::Vector3 & localWorldImuAngVel)
{
  correctedWorldFbKine_.orientation = R_0_fb_;
  correctedWorldFbKine_.position = worldAnchorKine_ctl_.position() - R_0_fb_ * fbAnchorKine_.position();

  poseW_.translation() = correctedWorldFbKine_.position();
  poseW_.rotation() = R_0_fb_.transpose();

  // we use the newly estimated orientation and local linear velocity of the IMU to obtain the one of the floating base.
  correctedWorldImuKine_ =
      correctedWorldFbKine_
      * fbImuKine_; // corrected pose of the imu in the world. This step is used only to get the
                    // pose of the IMU in the world that is required for the kinematics composition.

  correctedWorldImuKine_.linVel = correctedWorldImuKine_.orientation * localWorldImuLinVel;
  correctedWorldImuKine_.angVel = correctedWorldImuKine_.orientation * localWorldImuAngVel;

  correctedWorldFbKine_ = correctedWorldImuKine_ * fbImuKine_.getInverse();

  velW_.linear() = correctedWorldFbKine_.linVel();
  velW_.angular() = correctedWorldFbKine_.angVel();
}

void TiltObserver::update(mc_control::MCController & ctl)
{
  auto & realRobot = ctl.realRobot(robot_);
  if(updateRobot_)
  {
    update(realRobot);
    realRobot.forwardKinematics();
    realRobot.forwardVelocity();
  }

  if(updateSensor_)
  {
    auto & robot = ctl.robot(robot_);

    auto & imu = const_cast<mc_rbdyn::BodySensor &>(robot.bodySensor(imuSensor_));
    auto & rimu = const_cast<mc_rbdyn::BodySensor &>(realRobot.bodySensor(imuSensor_));

    imu.orientation(Eigen::Quaterniond{estimatedRotationIMU_.transpose()});
    rimu.orientation(Eigen::Quaterniond{estimatedRotationIMU_.transpose()});
  }
}

void TiltObserver::update(mc_rbdyn::Robot & robot)
{
  robot.posW(poseW_);
  robot.velW(velW_);
}

void TiltObserver::addToLogger(const mc_control::MCController & ctl,
                               mc_rtc::Logger & logger,
                               const std::string & category)
{
  category_ = category;

  logger.addLogEntry(category + "_FloatingBase_world_pose", [this]() -> const sva::PTransformd & { return poseW_; });
  logger.addLogEntry(category + "_FloatingBase_world_vel", [this]() -> const sva::MotionVecd & { return velW_; });

  logger.addLogEntry(category + "_estimatedState_x1", [this]() -> so::Vector3 { return xk_.head(3); });
  logger.addLogEntry(category + "_estimatedState_x2prime",
                     [this]() -> so::Vector3 { return xk_.segment(3, 3).normalized(); });
  logger.addLogEntry(category + "_estimatedState_x2", [this]() -> so::Vector3 { return xk_.tail(3).normalized(); });

  logger.addLogEntry(category + "_constants_alpha", [this]() -> double { return estimator_.getAlpha(); });
  logger.addLogEntry(category + "_constants_beta", [this]() -> double { return estimator_.getBeta(); });
  logger.addLogEntry(category + "_constants_gamma", [this]() -> double { return estimator_.getGamma(); });

  if(withDebugLogs_)
  {
    logger.addLogEntry(category + "_realRobotState_x1",
                       [this, &ctl]() -> so::Vector3
                       {
                         const auto & realRobot = ctl.realRobot(robot_);
                         const auto & rimu = realRobot.bodySensor(imuSensor_);

                         const sva::PTransformd & rimuXbs = rimu.X_b_s();

                         so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(
                             rimuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                         const sva::PTransformd & realRobotParentPoseW = realRobot.bodyPosW(rimu.parentBody());

                         // Compute velocity of the imu in the control frame
                         auto & realRobotV_0_imuParent =
                             realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(rimu.parentBody())];

                         so::kine::Kinematics worldParentKine =
                             conversions::kinematics::fromSva(realRobotParentPoseW, realRobotV_0_imuParent, true);

                         so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                         return worldImuKine.orientation.toMatrix3().transpose() * worldImuKine.linVel();
                       });

    logger.addLogEntry(category + "_realRobotState_x2",
                       [this, &ctl]() -> so::Vector3
                       {
                         const auto & realRobot = ctl.realRobot(robot_);
                         const auto & rimu = realRobot.bodySensor(imuSensor_);

                         const sva::PTransformd & rimuXbs = rimu.X_b_s();

                         so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(
                             rimuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                         const sva::PTransformd & realRobotParentPoseW = realRobot.bodyPosW(rimu.parentBody());

                         // Compute velocity of the imu in the control frame
                         auto & realRobotV_0_imuParent =
                             realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(rimu.parentBody())];

                         so::kine::Kinematics worldParentKine =
                             conversions::kinematics::fromSva(realRobotParentPoseW, realRobotV_0_imuParent, true);

                         so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                         return (worldImuKine.orientation.toMatrix3().transpose() * so::Vector3::UnitZ()).normalized();
                       });

    logger.addLogEntry(category + "_controlAnchorFrame", [this]() -> const sva::PTransformd & { return X_0_C_ctl_; });
    logger.addLogEntry(category + "_updatedRobot",
                       [this]() -> const sva::PTransformd &
                       {
                         const auto & updatedRobot = my_robots_->robot("updatedRobot");
                         return updatedRobot.posW();
                       });

    logger.addLogEntry(category + "_IMU_world_orientation",
                       [this]() { return Eigen::Quaterniond{estimatedRotationIMU_}; });
    logger.addLogEntry(category + "_AnchorFrame_world_position",
                       [this]() -> so::Vector3 { return worldAnchorKine_ctl_.position(); });
    logger.addLogEntry(category + "_AnchorFrame_world_linVel_global",
                       [this]() -> so::Vector3 { return worldAnchorKine_ctl_.linVel(); });

    logger.addLogEntry(category + "_debug_yv", [this]() -> const so::Vector3 & { return yv_; });
    logger.addLogEntry(category + "_debug_y", [this]() -> const so::Vector & { return yk_; });
    logger.addLogEntry(category + "_debug_initX", [this]() -> const so::Vector & { return initX_; });

    logger.addLogEntry(category + "_debug_ctlWorldImuLocAngVel",
                       [this, &ctl]() -> so::Vector3
                       {
                         const sva::PTransformd & imuXbs = ctl.robot(robot_).bodySensor(imuSensor_).X_b_s();

                         so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(
                             imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                         const sva::PTransformd & parentPoseW =
                             ctl.robot(robot_).bodyPosW(ctl.robot(robot_).bodySensor(imuSensor_).parentBody());

                         // Compute velocity of the imu in the control frame
                         auto & v_0_imuParent = ctl.robot(robot_).mbc().bodyVelW[ctl.robot(robot_).bodyIndexByName(
                             ctl.robot(robot_).bodySensor(imuSensor_).parentBody())];

                         so::kine::Kinematics worldParentKine =
                             conversions::kinematics::fromSva(parentPoseW, v_0_imuParent, true);

                         so::kine::Kinematics worldImuKine_ = worldParentKine * parentImuKine;

                         return worldImuKine_.orientation.toMatrix3().transpose() * worldImuKine_.angVel();
                       });

    logger.addLogEntry(category + "_debug_ctlWorldAnchorVelExpressedInImu",
                       [this, &ctl]() -> so::Vector3
                       {
                         const auto & robot = ctl.robot(robot_);

                         const auto & imu = robot.bodySensor(imuSensor_);
                         const sva::PTransformd & imuXbs = imu.X_b_s();

                         return imuXbs.rotation() * worldAnchorKine_ctl_.linVel();
                       });

    logger.addLogEntry(category + "_debug_realX1",
                       [this, &ctl]() -> so::Vector3
                       {
                         const auto & realRobot = ctl.realRobot(robot_);
                         const auto & rimu = realRobot.bodySensor(imuSensor_);

                         const sva::PTransformd & rimuXbs = rimu.X_b_s();

                         so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(
                             rimuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                         const sva::PTransformd & parentPoseW = realRobot.bodyPosW(rimu.parentBody());

                         // Compute velocity of the imu in the control frame
                         auto & v_0_imuParent = realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(rimu.parentBody())];

                         so::kine::Kinematics worldParentKine =
                             conversions::kinematics::fromSva(parentPoseW, v_0_imuParent, true);

                         so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                         return worldImuKine.orientation.toMatrix3().transpose() * worldImuKine.linVel();
                       });

    logger.addLogEntry(category + "_debug_realImuVel",
                       [this, &ctl]() -> so::Vector3
                       {
                         const auto & realRobot = ctl.realRobot(robot_);
                         const auto & rimu = realRobot.bodySensor(imuSensor_);

                         const sva::PTransformd & rimuXbs = rimu.X_b_s();

                         so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(
                             rimuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                         const sva::PTransformd & parentPoseW = realRobot.bodyPosW(rimu.parentBody());

                         auto & v_0_imuParent = realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(rimu.parentBody())];

                         so::kine::Kinematics worldParentKine =
                             conversions::kinematics::fromSva(parentPoseW, v_0_imuParent, true);

                         so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                         return worldImuKine.linVel();
                       });

    logger.addLogEntry(category + "_debug_realBodyVel",
                       [this, &ctl]() -> so::Vector3
                       {
                         const auto & realRobot = ctl.realRobot(robot_);
                         const auto & rimu = realRobot.bodySensor(imuSensor_);

                         return realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(rimu.parentBody())].linear();
                       });

    logger.addLogEntry(category + "_debug_ctlX1",
                       [this, &ctl]() -> so::Vector3
                       {
                         const auto & robot = ctl.robot(robot_);
                         const auto & imu = robot.bodySensor(imuSensor_);

                         const sva::PTransformd & imuXbs = imu.X_b_s();

                         so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(
                             imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                         const sva::PTransformd & parentPoseW = robot.bodyPosW(imu.parentBody());

                         // Compute velocity of the imu in the control frame
                         auto & v_0_imuParent = robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())];

                         so::kine::Kinematics worldParentKine =
                             conversions::kinematics::fromSva(parentPoseW, v_0_imuParent, true);

                         so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                         return worldImuKine.orientation.toMatrix3().transpose() * worldImuKine.linVel();
                       });

    logger.addLogEntry(category + "_debug_ctlImuVel",
                       [this, &ctl]() -> so::Vector3
                       {
                         const auto & robot = ctl.robot(robot_);
                         const auto & imu = robot.bodySensor(imuSensor_);

                         const sva::PTransformd & imuXbs = imu.X_b_s();

                         so::kine::Kinematics parentImuKine = conversions::kinematics::fromSva(
                             imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                         const sva::PTransformd & parentPoseW = robot.bodyPosW(imu.parentBody());

                         auto & v_0_imuParent = robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())];

                         so::kine::Kinematics worldParentKine =
                             conversions::kinematics::fromSva(parentPoseW, v_0_imuParent, true);

                         so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                         return worldImuKine.linVel();
                       });

    logger.addLogEntry(category + "_debug_ctlBodyVel",
                       [this, &ctl]() -> so::Vector3
                       {
                         const auto & robot = ctl.robot(robot_);
                         const auto & imu = robot.bodySensor(imuSensor_);

                         return robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())].linear();
                       });

    conversions::kinematics::addToLogger(logger, worldAnchorKine_ctl_, category + "_debug_worldAnchorKine_ctl");
    conversions::kinematics::addToLogger(logger, worldAnchorKine_, category + "_debug_worldAnchorKine");
    conversions::kinematics::addToLogger(logger, worldImuKine_, category + "_debug_worldImuKine");
    conversions::kinematics::addToLogger(logger, worldImuKine_ctl_, category + "_debug_worldImuKine_ctl");
    conversions::kinematics::addToLogger(logger, imuAnchorKine_, category + "_debug_imuAnchorKine");

    conversions::kinematics::addToLogger(logger, worldFbKine_, category + "_debug_worldFbKine");
    conversions::kinematics::addToLogger(logger, correctedWorldImuKine_, category + "_debug_correctedWorldImuKine");
    conversions::kinematics::addToLogger(logger, fbAnchorKine_, category + "_debug_fbAnchorKine");
  }
}

void TiltObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_imuVelC");
  logger.removeLogEntry(category + "_imuPoseC");
  logger.removeLogEntry(category + "_imuEstRotW");
  logger.removeLogEntry(category + "_controlAnchorFrame");
}

void TiltObserver::addToGUI(const mc_control::MCController &,
                            mc_rtc::gui::StateBuilder & gui,
                            const std::vector<std::string> & category)
{
  using namespace mc_state_observation::gui;
  gui.addElement(category, make_input_element("alpha", alpha_), make_input_element("beta", beta_),
                 make_input_element("gamma", gamma_));
}

} // namespace mc_state_observation
EXPORT_OBSERVER_MODULE("Tilt", mc_state_observation::TiltObserver)
