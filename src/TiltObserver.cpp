#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_state_observation/TiltObserver.h>
#include <mc_state_observation/gui_helpers.h>
#include <mc_state_observation/observersTools/kinematicsTools.h>

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

  imuSensor_ = config("imuSensor", ctl.robot().bodySensor().name());

  config("maxAnchorFrameDiscontinuity", maxAnchorFrameDiscontinuity_);
  config("updateRobot", updateRobot_);
  config("updateSensor", updateSensor_);

  config("initAlpha", alpha_);
  config("initBeta", beta_);
  config("initGamma", gamma_);

  config("finalAlpha", finalAlpha_);
  config("finalBeta", finalBeta_);
  config("finalGamma", finalGamma_);

  anchorFrameFunction_ = "KinematicAnchorFrame::" + ctl.robot(robot_).name();
  // if a user-defined anchor frame function is given, we use it instead
  if(config.has("anchorFrameFunction"))
  {
    if(ctl.datastore().has(anchorFrameFunction_))
    {
      anchorFrameFunction_ = config("anchorFrameFunction", name() + "::" + ctl.robot(robot_).name());
    }
  }

  std::string typeOfOdometry = static_cast<std::string>(config("odometryType"));

  if(typeOfOdometry == "flatOdometry") { odometryManager_.changeOdometryType(measurements::flatOdometry); }
  else if(typeOfOdometry == "6dOdometry") { odometryManager_.changeOdometryType(measurements::odometry6d); }
  else if(typeOfOdometry == "None") { odometryManager_.changeOdometryType(measurements::None); }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Odometry type not allowed. Please pick among : [None, flatOdometry, 6dOdometry]");
  }

  if(odometryManager_.odometryType_ != measurements::None)
  {
    bool velUpdatedUpstream = config("velUpdatedUpstream");
    bool accUpdatedUpstream = config("accUpdatedUpstream");
    bool verbose = config("verbose", true);
    bool withYawEstimation = config("withYawEstimation", true);

    odometryManager_.init(ctl, robot_, observerName_, odometryManager_.odometryType_, withYawEstimation,
                          velUpdatedUpstream, accUpdatedUpstream, verbose);

    // surfaces used for the contact detection. If the desired detection method doesn't use surfaces, we make sure this
    // list is not filled in the configuration file to avoid the use of an undesired method.
    std::vector<std::string> surfacesForContactDetection;
    config("surfacesForContactDetection", surfacesForContactDetection);

    std::vector<std::string> contactsSensorsDisabledInit =
        config("contactsSensorDisabledInit", std::vector<std::string>());

    std::string contactsDetection = static_cast<std::string>(config("contactsDetection"));

    LoContactsManager::ContactsDetection contactsDetectionMethod = LoContactsManager::ContactsDetection::undefined;
    if(contactsDetection == "fromThreshold")
    {
      contactsDetectionMethod = LoContactsManager::ContactsDetection::fromThreshold;
    }
    else if(contactsDetection == "fromSurfaces")
    {
      contactsDetectionMethod = LoContactsManager::ContactsDetection::fromSurfaces;
    }
    else if(contactsDetection == "fromSolver")
    {
      contactsDetectionMethod = LoContactsManager::ContactsDetection::fromSolver;
    }

    if(contactsDetectionMethod == LoContactsManager::ContactsDetection::undefined)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Contacts detection type not allowed. Please pick among : [fromSolver, fromThreshold, fromSurfaces] or "
          "initialize a list of surfaces with the variable surfacesForContactDetection");
    }
    if(surfacesForContactDetection.size() > 0)
    {
      if(contactsDetectionMethod != LoContactsManager::ContactsDetection::fromSurfaces)
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "Another type of contacts detection is currently used, please change it to 'fromSurfaces' or empty the "
            "surfacesForContactDetection variable");
      }
    }
    else if(contactsDetectionMethod == LoContactsManager::ContactsDetection::fromSurfaces)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "You selected the contacts detection using surfaces but didn't add the list of surfaces, please add it usign "
          "the variable surfacesForContactDetection");
    }

    const auto & robot = ctl.robot(robot_);

    double contactDetectionPropThreshold = config("contactDetectionPropThreshold", 0.11);

    contactDetectionThreshold_ = robot.mass() * so::cst::gravityConstant * contactDetectionPropThreshold;

    if(contactsDetectionMethod == LoContactsManager::ContactsDetection::fromSurfaces)
    {
      odometryManager_.initDetection(ctl, robot_, contactsDetectionMethod, surfacesForContactDetection,
                                     contactsSensorsDisabledInit, contactDetectionThreshold_);
    }
    else
    {
      std::vector<std::string> forceSensorsToOmit = config("forceSensorsToOmit", std::vector<std::string>());

      odometryManager_.initDetection(ctl, robot_, contactsDetectionMethod, contactsSensorsDisabledInit,
                                     contactDetectionThreshold_, forceSensorsToOmit);
    }
  }

  // check if this observer is used as a backup. If yes we add the backup function to the datastore.
  config("asBackup", asBackup_);
  if(asBackup_)
  {
    auto & datastore = (const_cast<mc_control::MCController &>(ctl)).datastore();

    datastore.make_call("runBackup", [this, &ctl]() -> const so::kine::Kinematics { return backupFb(ctl); });
    datastore.make_call("applyLastTransformation",
                        [this](so::kine::Kinematics & kine) -> so::kine::Kinematics
                        { return applyLastTransformation(kine); });
    datastore.make_call("checkCorrectBackupConf",
                        [this](OdometryType & koOdometryType) { checkCorrectBackupConf(koOdometryType); });
    datastore.make_call("changeTiltOdometryType",
                        [this](const std::string & newOdometryType) { changeOdometryType(newOdometryType); });
  }
}

void TiltObserver::reset(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());

  // the updated robot has the same floating base's pose than the control robot, but its encoders are updated. We use it
  // to get more accurate local Kinematics.
  my_robots_->robotCopy(robot, "updatedRobot");
  ctl.gui()->addElement(
      {"Robots"},
      mc_rtc::gui::Robot("TiltEstimator", [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));
  /*
ctl.gui()->addElement(
  {"Robots"},
  mc_rtc::gui::Robot("TiltEstimatorUpdatedRobot", [this]() -> const mc_rbdyn::Robot & { return
my_robots_->robot("updatedRobot"); }));
  */
  const auto & imu = robot.bodySensor(imuSensor_);

  poseW_ = realRobot.posW();
  velW_ = realRobot.velW();
  prevPoseW_ = sva::PTransformd::Identity();
  velW_ = sva::MotionVecd::Zero();

  so::Vector3 tilt; // not exactly the tilt but Rt * ez, corresponding to the Tilt estimator's x1
  if(imu.linearAcceleration().norm() < 1e-4) { tilt = ctl.robot(robot_).posW().rotation() * so::Vector3::UnitZ(); }
  else
  {
    tilt = imu.linearAcceleration()
           / imu.linearAcceleration().norm(); // we consider the acceleration as zero on the initialization
  }

  estimator_.initEstimator(so::Vector3::Zero(), tilt, tilt);

  const Eigen::Matrix3d cOri = (imu.X_b_s() * ctl.robot(robot_).bodyPosW(imu.parentBody())).rotation();
  so::Vector3 initX2 = cOri * so::Vector3::UnitZ(); // so::kine::rotationMatrixToRotationVector(cOri.transpose());

  estimator_.initEstimator(so::Vector3::Zero(), initX2, initX2);

  /* Initialization of the variables */
  X_0_C_ = sva::PTransformd::Identity();
  X_0_C_updated_ = sva::PTransformd::Identity();
  X_0_C_updated_previous_ = sva::PTransformd::Identity();
  worldAnchorKine_ = stateObservation::kine::Kinematics::zeroKinematics(flagPoseVels_);
  updatedWorldAnchorKine_ = stateObservation::kine::Kinematics::zeroKinematics(flagPoseVels_);
  updatedImuAnchorKine_ = stateObservation::kine::Kinematics::zeroKinematics(flagPoseVels_);
  anchorFrameJumped_ = false;
  iter_ = 0;
  imuVelC_ = sva::MotionVecd::Zero();
  X_C_IMU_ = sva::PTransformd::Identity();

  // we check if this estimator is used as a backup of the Kinetics Observer
  if(asBackup_)
  {
    // BOOST_ASSERT(withOdometry_ && "The odometry must be used to perform backup");
    int backupIterInterval = ctl.datastore().get<int>("koBackupIterInterval");

    backupFbKinematics_.resize(backupIterInterval);
    ctl.gui()->addElement({"OdometryBackup"}, mc_rtc::gui::Button("OdometryBackup", [this, &ctl]() { backupFb(ctl); }));
  }
}

bool TiltObserver::run(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  std::vector<double> q0 = robot.mbc().q[0];
  my_robots_->robot("updatedRobot").mbc().q = realRobot.mbc().q;
  my_robots_->robot("updatedRobot").mbc().q[0] = q0;

  my_robots_->robot("updatedRobot").forwardKinematics();
  my_robots_->robot("updatedRobot").forwardVelocity();

  if(logger.t() > 1.0)
  {
    alpha_ = finalAlpha_;
    beta_ = finalBeta_;
    gamma_ = finalGamma_;
  }

  if(odometryManager_.odometryType_ == measurements::None) { runTiltEstimator(ctl, my_robots_->robot("updatedRobot")); }
  else { runTiltEstimator(ctl, odometryManager_.odometryRobot()); }

  iter_++;

  /* Update of the observed robot */
  my_robots_->robot().mbc().q = realRobot.mbc().q;
  update(my_robots_->robot());

  return true;
}

void TiltObserver::updateAnchorFrame(const mc_control::MCController & ctl, const mc_rbdyn::Robot & updatedRobot)
{
  // update of the pose of the anchor frame of the control and updatedRobot in the world.
  if(odometryManager_.odometryType_ != measurements::None)
  {
    // we compute the anchor frame using the lastly computed floating base so we use the previous encoders information.
    // we use the current force sensors reading.
    updateAnchorFrameOdometry(ctl);
    // newWorldAnchorKine_ is already updated in updateAnchorFrameOdometry()
    newUpdatedWorldAnchorKine_ = newWorldAnchorKine_;
  }
  else
  {
    updateAnchorFrameNoOdometry(ctl, updatedRobot);
    // new pose of the anchor frame in the world.
    newWorldAnchorKine_ = kinematicsTools::poseFromSva(X_0_C_, so::kine::Kinematics::Flags::pose);
    newUpdatedWorldAnchorKine_ = kinematicsTools::poseFromSva(X_0_C_updated_, so::kine::Kinematics::Flags::pose);
  }

  /*
  if(iter_ > itersBeforeAnchorsVel_)
  { // Check whether anchor frame jumped
    auto error = (X_0_C_.translation() - worldAnchorKine_.position()).norm();
    if(error > maxAnchorFrameDiscontinuity_)
    {
      mc_rtc::log::warning("[{}] Control anchor frame jumped from [{}] to [{}] (error norm {} > threshold {})", name(),
                           MC_FMT_STREAMED(worldAnchorKine_.position().transpose()),
                           MC_FMT_STREAMED(X_0_C_.translation().transpose()), error, maxAnchorFrameDiscontinuity_);
      anchorFrameJumped_ = true;
    }
    auto errorUpdated = (X_0_C_updated_.translation() - updatedWorldAnchorKine_.position()).norm();
    if(errorUpdated > maxAnchorFrameDiscontinuity_)
    {
      mc_rtc::log::warning("[{}] Updated anchor frame jumped from [{}] to [{}] (error norm {:.3f} > threshold {:.3f})",
                           name(), MC_FMT_STREAMED(X_0_C_updated_previous_.translation().transpose()),
                           MC_FMT_STREAMED(X_0_C_updated_.translation().transpose()), errorUpdated,
                           maxAnchorFrameDiscontinuity_);
      anchorFrameJumped_ = true;
    }
  }
  */

  X_0_C_updated_previous_ = X_0_C_updated_;

  // the velocities of the anchor frames are computed by finite differences, unless it is given in exceptional cases.
  worldAnchorKine_.update(newWorldAnchorKine_, ctl.timeStep, flagPoseVels_);
  updatedWorldAnchorKine_.update(newUpdatedWorldAnchorKine_, ctl.timeStep, flagPoseVels_);

  // we ignore the initial outlier velocities due to the position jump
  if(iter_ < itersBeforeAnchorsVel_)
  {
    worldAnchorKine_.linVel().setZero();
    worldAnchorKine_.angVel().setZero();
    updatedWorldAnchorKine_.linVel().setZero();
    updatedWorldAnchorKine_.angVel().setZero();
  }
}

void TiltObserver::updateAnchorFrameOdometry(const mc_control::MCController & ctl)
{
  // Generally contains only the pose of the anchor frame, but when no contact is detected, the anchor frame becomes the
  // frame of the IMU and its velocity is considered as zero. For that transition the one when the anchor frame gets
  // back "to normal", it will contain the zero velocity.
  newWorldAnchorKine_ = odometryManager_.getAnchorFramePose(ctl, imuSensor_);

  X_0_C_.translation() = newWorldAnchorKine_.position();
  X_0_C_.rotation() = newWorldAnchorKine_.orientation.toMatrix3().transpose();
  X_0_C_updated_ = X_0_C_;
}

void TiltObserver::updateAnchorFrameNoOdometry(const mc_control::MCController & ctl,
                                               const mc_rbdyn::Robot & updatedRobot)
{
  const auto & robot = ctl.robot(robot_);
  // const auto & robot = my_robots_->robot("updatedRobot");

  anchorFrameJumped_ = false;

  // We don't use the defautl anchorFrameFunction because the obtained anchor position is in the shape of steps and we
  // obtain very high velocities when using finite differences

  if(!ctl.datastore().has(anchorFrameFunction_))
  {
    double leftFootRatio = robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                           / (robot.indirectSurfaceForceSensor("LeftFootCenter").force().z()
                              + robot.indirectSurfaceForceSensor("RightFootCenter").force().z());

    X_0_C_ = sva::interpolate(robot.surfacePose("RightFootCenter"), robot.surfacePose("LeftFootCenter"), leftFootRatio);
    X_0_C_updated_ = sva::interpolate(updatedRobot.surfacePose("RightFootCenter"),
                                      updatedRobot.surfacePose("LeftFootCenter"), leftFootRatio);
  }
  else
  {
    X_0_C_ = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, robot);
    X_0_C_updated_ = ctl.datastore().call<sva::PTransformd>(anchorFrameFunction_, updatedRobot);
  }
}

void TiltObserver::runTiltEstimator(const mc_control::MCController & ctl, const mc_rbdyn::Robot & updatedRobot)
{
  /*
  For the kinematics of the IMU and anchor frame in the world frame, we use the control robot.
  For internal kinematics like the anchor frame in the IMU, we use the updated robot whose encoders got updated.
  */
  // const auto & robot = my_robots_->robot("updatedRobot");
  const auto & robot = ctl.robot(robot_);

  estimator_.setAlpha(alpha_);
  estimator_.setBeta(beta_);
  estimator_.setGamma(gamma_);

  // updates the anchor frame used by the tilt observer.
  // If we perform odometry, the control and real robot anchor frame are both the one of the odometry robot.
  updateAnchorFrame(ctl, updatedRobot);

  // Anchor frame defined w.r.t control robot
  // XXX what if the feet are being moved by the stabilizer?

  // we want in the anchor frame:
  // - position of the IMU
  // - orientation of the IMU
  // - linear velocity of the imu
  // - angular velocity of the imu
  // - linear velocity of the anchor frame in the world of the control robot (derivative?)

  const auto & imu = ctl.robot(robot_).bodySensor(imuSensor_);
  // const auto & rimu = updatedRobot.bodySensor(imuSensor_);

  // In the case we do odometry, the pose and velocities of the odometry robot are still not updated but the joints are.
  // It is not a problem as this kinematics object is not used to retrieve global poses and velocities.
  updatedWorldFbKine_ = kinematicsTools::poseAndVelFromSva(updatedRobot.posW(), updatedRobot.velW(), true);

  // we use the imu object of control robot because the copy of BodySensor objects seems to be incomplete. Anyway we use
  // it only to get information about the parent body, which is the same with the control robot.
  const sva::PTransformd & imuXbs = imu.X_b_s();

  so::kine::Kinematics parentImuKine =
      kinematicsTools::poseFromSva(imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

  const sva::PTransformd & parentPoseW = robot.bodyPosW(imu.parentBody());
  const sva::PTransformd & updatedParentPoseW = updatedRobot.bodyPosW(imu.parentBody());

  // Compute velocity of the imu in the control frame
  auto & v_0_imuParent = robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())];

  auto & updated_v_0_imuParent = updatedRobot.mbc().bodyVelW[updatedRobot.bodyIndexByName(imu.parentBody())];

  so::kine::Kinematics worldParentKine = kinematicsTools::poseAndVelFromSva(parentPoseW, v_0_imuParent, true);
  so::kine::Kinematics updatedWorldParentKine =
      kinematicsTools::poseAndVelFromSva(updatedParentPoseW, updated_v_0_imuParent, true);

  // pose and velocities of the IMU in the world frame
  worldImuKine_ = worldParentKine * parentImuKine;
  updatedWorldImuKine_ = updatedWorldParentKine * parentImuKine;

  // pose and velocities of the IMU in the floating base. Use of updated robot to use encoders.
  updatedFbImuKine_ = updatedWorldFbKine_.getInverse() * updatedWorldImuKine_;

  // new pose of the anchor frame in the IMU frame. The velocity is computed right after because we don't want to use
  // the one given by mc_rtc.
  so::kine::Kinematics newUpdatedImuAnchorKine = updatedWorldImuKine_.getInverse() * updatedWorldAnchorKine_;

  // The velocities of the IMU in the world (given by mc_rtc) and the ones of the anchor frame in the world (by finite
  // differences) are not computed the same way, combining them to get the velocity of the anchor frame in the IMU frame
  // therefore leads to errors. So we "unset" the erroneous newly compute velocities to compute them by finite
  // differences from the pose of the anchor frame in the IMU.
  newUpdatedImuAnchorKine.linVel.set(false);
  newUpdatedImuAnchorKine.angVel.set(false);

  updatedImuAnchorKine_.update(newUpdatedImuAnchorKine, ctl.timeStep, flagPoseVels_);

  // we ignore the initial outlier velocity due to the position jump
  // we also reset the velocity of the anchor frame when its computation mode changes.
  if(iter_ < itersBeforeAnchorsVel_ || newWorldAnchorKine_.linVel.isSet())
  {
    updatedImuAnchorKine_.linVel().setZero();
    updatedImuAnchorKine_.angVel().setZero();
  }

  so::kine::Kinematics updatedAnchorImuKine = updatedImuAnchorKine_.getInverse();

  auto k = estimator_.getCurrentTime();

  // computation of the local linear velocity of the IMU in the world.

  if(odometryManager_.odometryType_ == measurements::None) // case if we don't use odometry
  {
    x1_ = worldImuKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.linVel()
          - (imu.angularVelocity()).cross(updatedImuAnchorKine_.position()) - updatedImuAnchorKine_.linVel();

    estimator_.setMeasurement(x1_, imu.linearAcceleration(), imu.angularVelocity(), k + 1);
  }
  else
  {
    // when using the odometry, we use the x1 computed internally by the Tilt Observer
    estimator_.setSensorPositionInC(updatedAnchorImuKine.position());
    estimator_.setSensorOrientationInC(updatedAnchorImuKine.orientation.toMatrix3());
    estimator_.setSensorLinearVelocityInC(updatedAnchorImuKine.linVel());
    estimator_.setSensorAngularVelocityInC(updatedAnchorImuKine.angVel());
    estimator_.setControlOriginVelocityInW(worldAnchorKine_.orientation.toMatrix3().transpose()
                                           * worldAnchorKine_.linVel());

    estimator_.setMeasurement(imu.linearAcceleration(), imu.angularVelocity(), k + 1);

    // If the following variable is set, it means that the mode of computation of the anchor frame changed.
    if(newWorldAnchorKine_.linVel.isSet())
    {
      // The anchor frame can be obtained using 2 ways:
      // - 1: contacts are detected and can be used
      // - 2: no contact is detected, the robot is hanging. As we still need an anchor frame for the tilt estimation we
      // arbitrarily use the frame of the IMU. As we cannot perform odometry anymore as there is no contact, we cannot
      // obtain the velocity of the IMU. We will then consider it as zero and consider it as constant with the linear
      // acceleration as zero too.
      // When switching from one mode to another, we consider x1hat = x1 before the estimation to avoid discontinuities.

      updatedImuAnchorKine_.linVel().setZero();
      updatedImuAnchorKine_.angVel().setZero();

      if(odometryManager_.prevAnchorFromContacts_)
      {
        estimator_.setMeasurement(so::Vector3::Zero(), imu.linearAcceleration(), imu.angularVelocity(), k + 1);
        // estimator_.setAlpha(0);
      }
      else
      {
        // estimator_.setAlpha(alpha_);
      }

      estimator_.resetImuLocVelHat();
    }
  }

  // estimation of the state with the complementary filters
  xk_ = estimator_.getEstimatedState(k + 1);

  // retrieving the estimated Tilt
  so::Vector3 tilt = xk_.tail(3);

  // Orientation of the imu in the world obtained from the estimated tilt and the yaw of the control robot.
  // When using odometry, the tilt will be kept but the yaw will be replaced by the one of the odometry robot.
  estimatedRotationIMU_ = so::kine::mergeTiltWithYawAxisAgnostic(tilt, worldImuKine_.orientation.toMatrix3());

  // Estimated orientation of the floating base in the world (especially the tilt)
  R_0_fb_ = estimatedRotationIMU_ * updatedFbImuKine_.orientation.toMatrix3().transpose();

  // Once we obtain the tilt (which is required by the legged odometry, estimating only the yaw), we update the pose and
  // velocities of the floating base

  if(odometryManager_.odometryType_ != measurements::None)
  {
    // we can update the estimated pose using odometry. The velocity will be updated later using the estimated local
    // linear velocity of the IMU.
    auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

    odometryManager_.run(ctl, logger, poseW_, R_0_fb_);
  }

  updatePoseAndVel(xk_.head(3), imu.angularVelocity());
  backupFbKinematics_.push_back(poseW_);

  // update the velocities as MotionVecd for the logs
  imuVelC_.linear() = updatedAnchorImuKine.linVel();
  imuVelC_.angular() = updatedAnchorImuKine.angVel();

  // update the pose as PTransformd for the logs
  X_C_IMU_.translation() = updatedAnchorImuKine.position();
  X_C_IMU_.rotation() = updatedAnchorImuKine.orientation.toMatrix3().transpose();
}

void TiltObserver::updatePoseAndVel(const so::Vector3 & localWorldImuLinVel, const so::Vector3 & localWorldImuAngVel)
{
  // if we use odometry, the pose will already updated in odometryManager_.run(...)
  if(odometryManager_.odometryType_ == measurements::None)
  {
    so::kine::Kinematics updatedFbAnchorKine = updatedWorldFbKine_.getInverse() * updatedWorldAnchorKine_;

    correctedWorldFbKine_.orientation = R_0_fb_;
    correctedWorldFbKine_.position = worldAnchorKine_.position() - R_0_fb_ * updatedFbAnchorKine.position();

    poseW_.translation() = correctedWorldFbKine_.position();
    poseW_.rotation() = R_0_fb_.transpose();
  }
  else { correctedWorldFbKine_ = kinematicsTools::poseFromSva(poseW_, so::kine::Kinematics::Flags::pose); }

  // we use the newly estimated orientation and local linear velocity of the IMU to obtain the one of the floating base.
  correctedWorldImuKine_ =
      correctedWorldFbKine_
      * updatedFbImuKine_; // corrected pose of the imu in the world. This step is used only to get the
                           // pose of the IMU in the world that is required for the kinematics composition.

  correctedWorldImuKine_.linVel = correctedWorldImuKine_.orientation * localWorldImuLinVel;
  correctedWorldImuKine_.angVel = correctedWorldImuKine_.orientation * localWorldImuAngVel;

  correctedWorldFbKine_ = correctedWorldImuKine_ * updatedFbImuKine_.getInverse();

  velW_.linear() = correctedWorldFbKine_.linVel();
  velW_.angular() = correctedWorldFbKine_.angVel();

  if(odometryManager_.odometryType_ != measurements::None)
  {
    // the velocity of the odometry robot was obtained using finite differences. We give it our estimated velocity which
    // is more accurate.
    odometryManager_.odometryRobot().velW(velW_);
  }
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

const so::kine::Kinematics TiltObserver::backupFb(const mc_control::MCController & ctl)
{
  // new initial pose of the floating base

  boost::circular_buffer<so::kine::Kinematics> * koBackupFbKinematics =
      ctl.datastore().get<boost::circular_buffer<so::kine::Kinematics> *>("koBackupFbKinematics");
  so::kine::Kinematics worldResetKine = *(koBackupFbKinematics->begin());

  // so::kine::Kinematics worldResetKine = so::kine::Kinematics::zeroKinematics(so::kine::Kinematics::Flags::pose);

  // original initial pose of the floating base
  so::kine::Kinematics worldFbInitBackup =
      kinematicsTools::poseFromSva(backupFbKinematics_.front(), so::kine::Kinematics::Flags::pose);

  so::kine::Kinematics fbWorldInitBackup = worldFbInitBackup.getInverse();

  // we apply the transformation from the initial pose to the intermediates pose estimated by the tilt estimator to the
  // new starting pose of the Kinetics Observer
  for(int i = 0; i < koBackupFbKinematics->size(); i++)
  {
    // Intermediary pose of the floating base estimated by the tilt estimator
    so::kine::Kinematics worldFbIntermBackup =
        kinematicsTools::poseFromSva(backupFbKinematics_.at(i), so::kine::Kinematics::Flags::pose);
    // transformation between the initial and the intermediary pose during the backup interval
    so::kine::Kinematics initInterm = fbWorldInitBackup * worldFbIntermBackup;

    koBackupFbKinematics->at(i) = worldResetKine * initInterm;
  }

  so::Vector3 tiltLocalLinVel = poseW_.rotation() * velW_.linear();
  so::Vector3 tiltLocalAngVel = poseW_.rotation() * velW_.angular();

  // koBackupFbKinematics->back() is the new last pose of the kinetics observer
  koBackupFbKinematics->back().linVel = koBackupFbKinematics->back().orientation.toMatrix3() * tiltLocalLinVel;
  koBackupFbKinematics->back().angVel = koBackupFbKinematics->back().orientation.toMatrix3() * tiltLocalAngVel;

  return koBackupFbKinematics->back();
}

so::kine::Kinematics TiltObserver::applyLastTransformation(const so::kine::Kinematics & previousKine)
{
  so::kine::Kinematics worldFbPreviousBackup = kinematicsTools::poseFromSva(
      backupFbKinematics_.at(backupFbKinematics_.size() - 2), so::kine::Kinematics::Flags::pose);

  so::kine::Kinematics fbWorldPreviousBackup = worldFbPreviousBackup.getInverse();
  so::kine::Kinematics worldFbFinalBackup =
      kinematicsTools::poseFromSva(backupFbKinematics_.back(), so::kine::Kinematics::Flags::pose);

  so::kine::Kinematics lastTransformation = fbWorldPreviousBackup * worldFbFinalBackup;

  so::kine::Kinematics newKine = previousKine * lastTransformation;

  so::Vector3 tiltLocalLinVel = poseW_.rotation() * velW_.linear();
  so::Vector3 tiltLocalAngVel = poseW_.rotation() * velW_.angular();

  // koBackupFbKinematics->back() is the new last pose of the kinetics observer
  newKine.linVel = newKine.orientation.toMatrix3() * tiltLocalLinVel;
  newKine.angVel = newKine.orientation.toMatrix3() * tiltLocalAngVel;

  return newKine;
}

void TiltObserver::checkCorrectBackupConf(OdometryType & koOdometryType)
{
  static bool wasExecuted = false;
  if(wasExecuted) { return; }
  else
  {
    if(odometryManager_.odometryType_ != koOdometryType)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("The odometry types used for the Tilt Observer and the Kinetics "
                                                       "Observer don't match, the backup is not possible.");
    }
    wasExecuted = true;
    return;
  }
}

void TiltObserver::changeOdometryType(const std::string & newOdometryType)
{
  OdometryType prevOdometryType = odometryManager_.odometryType_;
  if(newOdometryType == "flatOdometry") { odometryManager_.changeOdometryType(measurements::flatOdometry); }
  else if(newOdometryType == "6dOdometry") { odometryManager_.changeOdometryType(measurements::odometry6d); }

  if(odometryManager_.odometryType_ != prevOdometryType)
  {
    mc_rtc::log::info("[{}]: Odometry mode changed to: {}", observerName_, newOdometryType);
  }
}

void TiltObserver::addToLogger(const mc_control::MCController & ctl,
                               mc_rtc::Logger & logger,
                               const std::string & category)
{
  logger.addLogEntry(category + "_constants_alpha", [this]() -> const double & { return alpha_; });
  logger.addLogEntry(category + "_constants_beta", [this]() -> const double & { return beta_; });
  logger.addLogEntry(category + "_constants_gamma", [this]() -> const double & { return gamma_; });

  logger.addLogEntry(category + "_debug_OdometryType",
                     [this]() -> std::string
                     {
                       switch(odometryManager_.odometryType_)
                       {
                         case measurements::flatOdometry:
                           return "flatOdometry";
                           break;
                         case measurements::odometry6d:
                           return "6dOdometry";
                           break;
                         case measurements::None:
                           return "None";
                           break;
                       }
                       return "default";
                     });

  logger.addLogEntry(category + "_controlAnchorFrame", [this]() -> const sva::PTransformd & { return X_0_C_; });
  logger.addLogEntry(category + "_updatedRobot",
                     [this]() -> const sva::PTransformd &
                     {
                       const auto & updatedRobot = my_robots_->robot("updatedRobot");
                       return updatedRobot.posW();
                     });

  logger.addLogEntry(category + "_IMU_world_orientation",
                     [this]() { return Eigen::Quaterniond{estimatedRotationIMU_}; });
  logger.addLogEntry(category + "_IMU_world_localLinVel", [this]() -> so::Vector3 { return xk_.head(3); });
  logger.addLogEntry(category + "_IMU_AnchorFrame_pose", [this]() -> const sva::PTransformd & { return X_C_IMU_; });
  logger.addLogEntry(category + "_IMU_AnchorFrame_linVel", [this]() -> const sva::MotionVecd & { return imuVelC_; });
  logger.addLogEntry(category + "_AnchorFrame_world_position",
                     [this]() -> so::Vector3 { return worldAnchorKine_.position(); });
  logger.addLogEntry(category + "_AnchorFrame_world_orientation",
                     [this]() -> so::Vector3 { return worldAnchorKine_.orientation.toRotationVector(); });
  logger.addLogEntry(category + "_AnchorFrame_world_linVel_global",
                     [this]() -> so::Vector3 { return worldAnchorKine_.linVel(); });
  logger.addLogEntry(category + "_AnchorFrame_world_linVel_local",
                     [this]() -> so::Vector3
                     { return worldAnchorKine_.orientation.toMatrix3().transpose() * worldAnchorKine_.linVel(); });
  logger.addLogEntry(category + "_AnchorFrame_world_angVel",
                     [this]() -> const so::Vector3 & { return worldAnchorKine_.angVel(); });
  logger.addLogEntry(category + "_FloatingBase_world_pose", [this]() -> const sva::PTransformd & { return poseW_; });
  logger.addLogEntry(category + "_FloatingBase_world_vel", [this]() -> const sva::MotionVecd & { return velW_; });
  logger.addLogEntry(category + "_debug_x1", [this]() -> const so::Vector3 & { return x1_; });

  logger.addLogEntry(category + "_debug_realWorldImuLocAngVel",
                     [this, &ctl]() -> so::Vector3
                     {
                       const sva::PTransformd & realImuXbs = ctl.realRobot(robot_).bodySensor(imuSensor_).X_b_s();

                       so::kine::Kinematics realParentImuKine = kinematicsTools::poseFromSva(
                           realImuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                       const sva::PTransformd & realParentPoseW =
                           ctl.realRobot(robot_).bodyPosW(ctl.realRobot(robot_).bodySensor(imuSensor_).parentBody());

                       // Compute velocity of the imu in the control frame
                       auto & real_v_0_imuParent =
                           ctl.realRobot(robot_).mbc().bodyVelW[ctl.realRobot(robot_).bodyIndexByName(
                               ctl.realRobot(robot_).bodySensor(imuSensor_).parentBody())];

                       so::kine::Kinematics realWorldParentKine =
                           kinematicsTools::poseAndVelFromSva(realParentPoseW, real_v_0_imuParent, true);

                       so::kine::Kinematics realWorldImuKine_ = realWorldParentKine * realParentImuKine;

                       return realWorldImuKine_.orientation.toMatrix3().transpose() * realWorldImuKine_.angVel();
                     });

  logger.addLogEntry(category + "_debug_ctlWorldImuLocAngVel",
                     [this, &ctl]() -> so::Vector3
                     {
                       const sva::PTransformd & imuXbs = ctl.robot(robot_).bodySensor(imuSensor_).X_b_s();

                       so::kine::Kinematics parentImuKine = kinematicsTools::poseFromSva(
                           imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                       const sva::PTransformd & parentPoseW =
                           ctl.robot(robot_).bodyPosW(ctl.robot(robot_).bodySensor(imuSensor_).parentBody());

                       // Compute velocity of the imu in the control frame
                       auto & v_0_imuParent = ctl.robot(robot_).mbc().bodyVelW[ctl.robot(robot_).bodyIndexByName(
                           ctl.robot(robot_).bodySensor(imuSensor_).parentBody())];

                       so::kine::Kinematics worldParentKine =
                           kinematicsTools::poseAndVelFromSva(parentPoseW, v_0_imuParent, true);

                       so::kine::Kinematics worldImuKine_ = worldParentKine * parentImuKine;

                       return worldImuKine_.orientation.toMatrix3().transpose() * worldImuKine_.angVel();
                     });

  logger.addLogEntry(category + "_debug_ctlWorldAnchorVelExpressedInImu",
                     [this, &ctl]() -> so::Vector3
                     {
                       const auto & robot = ctl.robot(robot_);

                       const auto & imu = robot.bodySensor(imuSensor_);
                       const sva::PTransformd & imuXbs = imu.X_b_s();

                       return imuXbs.rotation() * worldAnchorKine_.linVel();
                     });

  logger.addLogEntry(
      category + "_debug_updatedWorldAnchorVelExpressedInImu",
      [this]() -> so::Vector3
      { return updatedWorldImuKine_.orientation.toMatrix3().transpose() * updatedWorldAnchorKine_.linVel(); });

  logger.addLogEntry(category + "_debug_realX1",
                     [this, &ctl]() -> so::Vector3
                     {
                       const auto & realRobot = ctl.realRobot(robot_);
                       const auto & rimu = realRobot.bodySensor(imuSensor_);

                       const sva::PTransformd & rimuXbs = rimu.X_b_s();

                       so::kine::Kinematics parentImuKine = kinematicsTools::poseFromSva(
                           rimuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                       const sva::PTransformd & parentPoseW = realRobot.bodyPosW(rimu.parentBody());

                       // Compute velocity of the imu in the control frame
                       auto & v_0_imuParent = realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(rimu.parentBody())];

                       so::kine::Kinematics worldParentKine =
                           kinematicsTools::poseAndVelFromSva(parentPoseW, v_0_imuParent, true);

                       so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                       return worldImuKine.orientation.toMatrix3().transpose() * worldImuKine.linVel();
                     });

  logger.addLogEntry(category + "_debug_realImuVel",
                     [this, &ctl]() -> so::Vector3
                     {
                       const auto & realRobot = ctl.realRobot(robot_);
                       const auto & rimu = realRobot.bodySensor(imuSensor_);

                       const sva::PTransformd & rimuXbs = rimu.X_b_s();

                       so::kine::Kinematics parentImuKine = kinematicsTools::poseFromSva(
                           rimuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                       const sva::PTransformd & parentPoseW = realRobot.bodyPosW(rimu.parentBody());

                       auto & v_0_imuParent = realRobot.mbc().bodyVelW[realRobot.bodyIndexByName(rimu.parentBody())];

                       so::kine::Kinematics worldParentKine =
                           kinematicsTools::poseAndVelFromSva(parentPoseW, v_0_imuParent, true);

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

                       so::kine::Kinematics parentImuKine = kinematicsTools::poseFromSva(
                           imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                       const sva::PTransformd & parentPoseW = robot.bodyPosW(imu.parentBody());

                       // Compute velocity of the imu in the control frame
                       auto & v_0_imuParent = robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())];

                       so::kine::Kinematics worldParentKine =
                           kinematicsTools::poseAndVelFromSva(parentPoseW, v_0_imuParent, true);

                       so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                       return worldImuKine.orientation.toMatrix3().transpose() * worldImuKine.linVel();
                     });

  logger.addLogEntry(category + "_debug_ctlImuVel",
                     [this, &ctl]() -> so::Vector3
                     {
                       const auto & robot = ctl.robot(robot_);
                       const auto & imu = robot.bodySensor(imuSensor_);

                       const sva::PTransformd & imuXbs = imu.X_b_s();

                       so::kine::Kinematics parentImuKine = kinematicsTools::poseFromSva(
                           imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                       const sva::PTransformd & parentPoseW = robot.bodyPosW(imu.parentBody());

                       auto & v_0_imuParent = robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())];

                       so::kine::Kinematics worldParentKine =
                           kinematicsTools::poseAndVelFromSva(parentPoseW, v_0_imuParent, true);

                       so::kine::Kinematics worldImuKine = worldParentKine * parentImuKine;
                       return worldImuKine.linVel();
                     });

  logger.addLogEntry(category + "_debug_contactDetected",
                     [this]() -> std::string
                     {
                       if(odometryManager_.contactsManager().contactsFound().size() > 0) { return "contacts"; }
                       else { return "no contacts"; }
                     });

  logger.addLogEntry(category + "_debug_ctlBodyVel",
                     [this, &ctl]() -> so::Vector3
                     {
                       const auto & robot = ctl.robot(robot_);
                       const auto & imu = robot.bodySensor(imuSensor_);

                       return robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())].linear();
                     });

  kinematicsTools::addToLogger(worldAnchorKine_, logger, category + "_debug_worldAnchorKine");
  kinematicsTools::addToLogger(updatedWorldImuKine_, logger, category + "_debug_updatedWorldImuKine");
  kinematicsTools::addToLogger(worldImuKine_, logger, category + "_debug_worldImuKine");
  kinematicsTools::addToLogger(updatedWorldAnchorKine_, logger, category + "_debug_updatedWorldAnchorKine");
  kinematicsTools::addToLogger(updatedImuAnchorKine_, logger, category + "_debug_updatedImuAnchorKine_");
  logger.addLogEntry(category + "_debug_ctlImuAnchorKine.linVel()",
                     [this]() -> so::Vector3
                     {
                       so::kine::Kinematics imuAnchorKine = worldImuKine_.getInverse() * worldAnchorKine_;
                       return imuAnchorKine.linVel();
                     });

  kinematicsTools::addToLogger(updatedWorldFbKine_, logger, category + "_debug_updatedWorldFbKine_");
  kinematicsTools::addToLogger(correctedWorldImuKine_, logger, category + "_debug_correctedWorldImuKine_");
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

  if(odometryManager_.odometryType_ != measurements::None)
  {
    gui.addElement({observerName_, "Odometry"},
                   mc_rtc::gui::ComboInput(
                       "Choose from list", {"6dOdometry", "flatOdometry"},
                       [this]() -> std::string
                       {
                         if(odometryManager_.odometryType_ == measurements::flatOdometry) { return "flatOdometry"; }
                         else { return "6dOdometry"; }
                       },
                       [this](const std::string & typeOfOdometry) { changeOdometryType(typeOfOdometry); }));
  }
}

} // namespace mc_state_observation
EXPORT_OBSERVER_MODULE("Tilt", mc_state_observation::TiltObserver)
