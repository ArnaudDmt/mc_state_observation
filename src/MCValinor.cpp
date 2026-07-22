#include <mc_observers/ObserverMacros.h>
#include <mc_rbdyn/ForceSensor.h>

#include <mc_state_observation/MCValinor.h>
#include <mc_state_observation/conversions/kinematics.h>
#include <mc_state_observation/gui_helpers.h>

namespace mc_state_observation
{

namespace so = stateObservation;

using OdometryType = stateObservation::odometry::OdometryType;

MCValinor::MCValinor(const std::string & type, double dt, bool asBackup)
: TiltObserver(type, dt), odometryManager_(), asBackup_(asBackup)
{
  odometryManager_.setSamplingTime(dt);
}

void MCValinor::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  imuSensor_ = config("imuSensor", ctl.robot().bodySensor().name());

  config("updateRobot", updateRobot_);
  config("updateSensor", updateSensor_);
  config("withDebugLogs", withDebugLogs_);

  auto contactsConfig = config("contacts");

  if(config.has("filterGains"))
  {
    auto filterGainsConfig = config("filterGains");
    alpha_ = filterGainsConfig("initAlpha", 4);
    beta_ = filterGainsConfig("initBeta", 1);
    gamma_ = filterGainsConfig("initGamma", 4);

    finalAlpha_ = filterGainsConfig("finalAlpha", 4);
    finalBeta_ = filterGainsConfig("finalBeta", 1);
    finalGamma_ = filterGainsConfig("finalGamma", 4);
  }

  anchorFrameFunction_ = "KinematicAnchorFrame::" + ctl.robot(robot_).name();
  // if a user-defined anchor frame function is given, we use it instead
  if(config.has("anchorFrameFunction"))
  {
    if(ctl.datastore().has(anchorFrameFunction_))
    {
      anchorFrameFunction_ = config("anchorFrameFunction", name() + "::" + ctl.robot(robot_).name());
    }
  }

  std::vector<std::string> surfacesForContactDetection =
      contactsConfig("surfacesForContactDetection", std::vector<std::string>());

  measurements::ContactsDetectorSurfacesConfiguration contactsConf(surfacesForContactDetection);

  contactsDetector_.init(ctl, robot_, contactsConf);

  mc_rtc::Configuration odomConfig;
  if(config.has("leggedOdometry"))
  {
    odomConfig = config("leggedOdometry");
    if(odomConfig.has("odometryType"))
    {
      setOdometryType(
          stateObservation::odometry::stringToOdometryType(static_cast<std::string>(odomConfig("odometryType"))));
    }
  }
  else
  {
    odomConfig.add("correctContacts", true);
    odomConfig.add("kappa", 10);
    odomConfig.add("lambdaInf", 0.001);
  }

  // specific configurations for the use of odometry.
  if(odometryManager_.odometryType_ != OdometryType::None)
  {
    bool correctContacts = odomConfig("correctContacts", true);
    double kappa = odomConfig("kappa", 10);
    odometryManager_.kappa(kappa);
    double lambdaInf = odomConfig("lambdaInf", 0.001);
    odometryManager_.lambdaInf(lambdaInf);

    const auto & imu = ctl.robot(robot_).bodySensor(imuSensor_);

    const sva::PTransformd & imuXbs = imu.X_b_s();
    so::kine::Kinematics parentImuKine =
        conversions::kinematics::fromSva(imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

    // pose of the IMU's parent body in the world for the odometry robot
    const sva::PTransformd & parentPoseW = ctl.realRobot().bodyPosW(imu.parentBody());
    // velocity of the IMU's parent body in the world for the odometry robot
    const sva::MotionVecd & v_0_imuParent =
        ctl.realRobot().mbc().bodyVelW[ctl.realRobot().bodyIndexByName(imu.parentBody())];

    // kinematics of the IMU's parent body in the world for the odometry robot
    so::kine::Kinematics worldParentKine = conversions::kinematics::fromSva(parentPoseW, v_0_imuParent, true);

    // pose and velocities of the IMU in the world frame for the odometry robot
    worldImuKine_ = worldParentKine * parentImuKine;
    stateObservation::odometry::LeggedOdometryManager::Configuration loConfig(odometryManager_.odometryType_);
    loConfig.correctContacts(correctContacts);

    odometryManager_.init(loConfig, worldImuKine_.toVector(stateObservation::kine::Kinematics::Flags::pose));
  }
}

void MCValinor::reset(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  my_robots_->robotCopy(realRobot, "inputRobot");

  // the updated robot has the same floating base's pose than the control robot, but its encoders are updated. We use
  // it to get more accurate local Kinematics.
  if(withGui_)
  {
    ctl.gui()->addElement(
        {"Robots"}, mc_rtc::gui::Robot(name(), [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));
  }

  const auto & imu = robot.bodySensor(imuSensor_);

  // reset of the floating base kinematics
  poseW_ = realRobot.posW();
  velW_ = realRobot.velW();

  // initialization of the estimator
  so::kine::Kinematics initParentImuKine = conversions::kinematics::fromSva(
      imu.X_b_s(), so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

  // kinematics of the IMU's parent body in the world for the odometry robot
  so::kine::Kinematics initWorldParentKine =
      conversions::kinematics::fromSva(realRobot.bodyPosW(imu.parentBody()), so::kine::Kinematics::Flags::pose);

  // pose of the IMU in the world frame for the odometry robot
  so ::kine::Kinematics initWorldImuKine = initWorldParentKine * initParentImuKine;
  so::Vector3 initX2 = initWorldImuKine.orientation.toMatrix3().transpose() * so::Vector3::UnitZ();

  initX_ = Eigen::RowVectorXd(9);
  initX_ << so::Vector3::Zero(), initX2, initX2;
  estimator_.initEstimator(initX_);

  iter_ = 0;

  stateObservation::kine::Kinematics initKine =
      conversions::kinematics::fromSva(ctl.realRobot().posW(), stateObservation::kine::Kinematics::Flags::pose);

  odometryManager_.replaceOdomBodyPose(initWorldImuKine.toVector(stateObservation::kine::Kinematics::Flags::pose));

  estimator_.setAlpha(alpha_);
  estimator_.setBeta(beta_);
  estimator_.setGamma(gamma_);
}

bool MCValinor::run(const mc_control::MCController & ctl)
{
  auto & inputRobot = my_robots_->robot("inputRobot");
  auto & measRobot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  const auto & realQ = realRobot.mbc().q;
  const auto & realAlpha = realRobot.mbc().alpha;

  std::copy(std::next(realQ.begin()), realQ.end(), std::next(inputRobot.mbc().q.begin()));
  std::copy(std::next(realAlpha.begin()), realAlpha.end(), std::next(inputRobot.mbc().alpha.begin()));

  // The input robot copies the real robot to update the encoder values.
  // Then its floating base is brung back to the origin of the world frame and given zero velocities and accelerations
  // in order to ease the computations.
  inputRobot.posW(poseW_);
  inputRobot.velW(velW_);

  inputRobot.forwardKinematics();
  inputRobot.forwardVelocity();

  if(logger.t() > 1.0)
  {
    alpha_ = finalAlpha_;
    beta_ = finalBeta_;
    gamma_ = finalGamma_;
  }

  updateNecessaryFrames(ctl, inputRobot);

  auto onNewContactOdom =
      [&inputRobot, &measRobot, &logger, this, &ctl](stateObservation::odometry::LoContact & newContact)
  {
    const std::string & surfaceName = newContact.surfaceName();
    const sva::PTransformd & surfaceXbs = inputRobot.surface(surfaceName).X_b_s();
    so::kine::Kinematics parentSurfaceKine = conversions::kinematics::fromSva(
        surfaceXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);
    const sva::PTransformd & parentPoseW = inputRobot.bodyPosW(inputRobot.surface(surfaceName).bodyName());
    const sva::MotionVecd & v_0_contactParent =
        inputRobot.mbc().bodyVelW[inputRobot.bodyIndexByName(inputRobot.surface(surfaceName).bodyName())];
    so::kine::Kinematics worldParentKine = conversions::kinematics::fromSva(parentPoseW, v_0_contactParent, true);

    stateObservation::kine::Kinematics worldContactKine = worldParentKine * parentSurfaceKine;

    newContact.bodyContactKine_ = worldImuKine_.getInverse() * worldContactKine;

    const mc_rbdyn::ForceSensor & fs = measRobot.indirectSurfaceForceSensor(newContact.surfaceName());
    newContact.lambda(fs.wrenchWithoutGravity(ctl.realRobot()).force().norm());

    if(withDebugLogs_)
    {
      conversions::kinematics::addToLogger(logger, newContact.worldRefKine_,
                                           category_ + "_contacts_" + newContact.surfaceName() + "_refPose");
      conversions::kinematics::addToLogger(logger, newContact.worldBodyKineFromRef_,
                                           category_ + "_contacts_" + newContact.surfaceName()
                                               + "_worldImuKineFromRef");
      conversions::kinematics::addToLogger(logger, newContact.currentWorldKine_,
                                           category_ + "_contacts_" + newContact.surfaceName()
                                               + "_currentWorldContactKine");
      conversions::kinematics::addToLogger(logger, newContact.bodyContactKine_,
                                           category_ + "_contacts_" + newContact.surfaceName() + "_bodyContactKine_");
      conversions::kinematics::addToLogger(logger, newContact.worldRefKineBeforeCorrection_,
                                           category_ + "_contacts_" + newContact.surfaceName()
                                               + "_refPoseBeforeCorrection");
      conversions::kinematics::addToLogger(logger, newContact.newIncomingWorldRefKine_,
                                           category_ + "_contacts_" + newContact.surfaceName()
                                               + "_newIncomingWorldRefKine");

      logger.addLogEntry(category_ + "_contacts_" + newContact.surfaceName() + "_isSet", &newContact,
                         [&newContact]() -> std::string { return newContact.isSet() ? "Set" : "notSet"; });
      logger.addLogEntry(category_ + "_contacts_" + newContact.surfaceName() + "_lambda", &newContact,
                         [&newContact]() -> double { return newContact.lambda(); });
      logger.addLogEntry(category_ + "_contacts_" + newContact.surfaceName() + "_lifeTime", &newContact,
                         [&newContact]() -> double { return newContact.lifeTime(); });
      logger.addLogEntry(category_ + "_contacts_" + newContact.surfaceName() + "_correctionWeightingCoeff", &newContact,
                         [&newContact]() -> double { return newContact.correctionWeightingCoeff(); });
    }
  };

  auto onMaintainedContactOdom =
      [&inputRobot, &measRobot, this, &ctl](stateObservation::odometry::LoContact & maintainedContact)
  {
    const std::string & surfaceName = maintainedContact.surfaceName();
    const sva::PTransformd & surfaceXbs = inputRobot.surface(surfaceName).X_b_s();
    so::kine::Kinematics parentSurfaceKine = conversions::kinematics::fromSva(
        surfaceXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);
    const sva::PTransformd & parentPoseW = inputRobot.bodyPosW(inputRobot.surface(surfaceName).bodyName());
    const sva::MotionVecd & v_0_contactParent =
        inputRobot.mbc().bodyVelW[inputRobot.bodyIndexByName(inputRobot.surface(surfaceName).bodyName())];
    so::kine::Kinematics worldParentKine = conversions::kinematics::fromSva(parentPoseW, v_0_contactParent, true);

    stateObservation::kine::Kinematics worldContactKine = worldParentKine * parentSurfaceKine;

    maintainedContact.bodyContactKine_ = worldImuKine_.getInverse() * worldContactKine;
    const mc_rbdyn::ForceSensor & fs = measRobot.indirectSurfaceForceSensor(maintainedContact.surfaceName());
    const stateObservation::Vector3 & forceMeas = fs.wrenchWithoutGravity(ctl.realRobot()).force();
    double forceRatio =
        forceMeas.z()
        / (forceMeas.head(2).norm() + 1e-6 * ctl.realRobot().mass() * stateObservation::cst::gravityConstant);

    maintainedContact.lambda(forceRatio);
  };

  auto onRemovedContactOdom = [&logger](stateObservation::odometry::LoContact & removedContact)
  {
    conversions::kinematics::removeFromLogger(logger, removedContact.worldRefKine_);
    conversions::kinematics::removeFromLogger(logger, removedContact.worldRefKineBeforeCorrection_);
    conversions::kinematics::removeFromLogger(logger, removedContact.worldBodyKineFromRef_);
    conversions::kinematics::removeFromLogger(logger, removedContact.currentWorldKine_);
    conversions::kinematics::removeFromLogger(logger, removedContact.bodyContactKine_);
    conversions::kinematics::removeFromLogger(logger, removedContact.newIncomingWorldRefKine_);
    logger.removeLogEntries(&removedContact);
  };

  std::unordered_set<std::string> & contactList = contactsDetector_.updateContacts(ctl, robot_);
  auto contactUpdateFunctions = stateObservation::odometry::LeggedOdometryManager::ContactUpdateFunctions()
                                    .onNewContact(onNewContactOdom)
                                    .onMaintainedContact(onMaintainedContactOdom)
                                    .onRemovedContact(onRemovedContactOdom);

  odometryManager_.initLoop(contactList, contactUpdateFunctions, &velW_.linear(), &velW_.angular());

  // position and linear velocity of the anchor point in the frame of the IMU.
  imuAnchorKine_ = odometryManager_.getAnchorKineInBody(true);
  stateObservation::kine::Kinematics imuWorldKine = worldImuKine_.getInverse();
  worldAnchorKine_ = odometryManager_.getAnchorKineIn(imuWorldKine);

  const auto & imu = measRobot.bodySensor(imuSensor_);

  auto k = estimator_.getCurrentTime();

  // measuredOri_ = so::Matrix3(ctl.realRobot(robot_).posW().rotation().transpose());

  // The anchor frame can be obtained using 2 ways:
  // - 1: contacts are detected and can be used
  // - 2: no contact is detected, the robot is hanging. As we still need an anchor frame for the tilt estimation we
  // arbitrarily use the frame of the IMU. As we cannot perform odometry anymore as there is no contact, we cannot
  // obtain the velocity of the IMU. We will then consider it as zero and consider it as constant with the linear
  // acceleration as zero too.
  // When switching from one mode to another, we consider x1hat = x1 before the estimation to avoid discontinuities.

  if(odometryManager_.maintainedContacts().size() == 0)
  {
    estimator_.setAlpha(30);
    estimator_.setBeta(0);

    yv_.setZero();
  }
  else
  {
    estimator_.setAlpha(alpha_);
    estimator_.setBeta(beta_);
    estimator_.setGamma(gamma_);

    yv_ = -imu.angularVelocity().cross(imuAnchorKine_.position()) - imuAnchorKine_.linVel();
  }

  estimator_.setMeasurement(yv_, imu.linearAcceleration(), imu.angularVelocity(), k + 1);

  // estimation of the state with the complementary filters
  estimator_.getEstimatedState(k + 1);

  odometryManager_.run(stateObservation::odometry::LeggedOdometryManager::KineParams(estimatedWorldImuKine_)
                           .tiltMeasurement(estimator_.getEstimatedTilt()));

  estimatedWorldImuKine_.linVel = estimatedWorldImuKine_.orientation.toMatrix3() * estimator_.getEstimatedLocLinVel();
  estimatedWorldImuKine_.angVel = estimatedWorldImuKine_.orientation.toMatrix3() * imu.angularVelocity();

  updatePoseAndVel(ctl);

  /* Backups */

  // for the Kinetics Observer
  if(asBackup_)
  {
    backupFbKinematics_.push_back(conversions::kinematics::fromSva(poseW_, so::kine::Kinematics::Flags::pose));
  }

  iter_++;

  /* Update of the observed robot */
  my_robots_->robot().mbc().q = realRobot.mbc().q;
  update(my_robots_->robot());

  return true;
}

const so::kine::Kinematics MCValinor::backupFb(boost::circular_buffer<so::kine::Kinematics> * koBackupFbKinematics)
{
  // new initial pose of the floating base
  so::kine::Kinematics worldResetKine = *(koBackupFbKinematics->begin());

  // original initial pose of the floating base
  so::kine::Kinematics worldFbInitBackup = backupFbKinematics_.front();

  so::kine::Kinematics fbWorldInitBackup = worldFbInitBackup.getInverse();

  // we apply the transformation from the initial pose to the intermediates pose estimated by the tilt estimator to the
  // new starting pose of the Kinetics Observer
  for(size_t i = 0; i < koBackupFbKinematics->size(); i++)
  {
    // Intermediary pose of the floating base estimated by the tilt estimator
    so::kine::Kinematics worldFbIntermBackup = backupFbKinematics_.at(i);

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

so::kine::Kinematics MCValinor::applyLastTransformation(const so::kine::Kinematics & previousKine)
{
  so::kine::Kinematics worldFbPreviousBackup = backupFbKinematics_.at(backupFbKinematics_.size() - 2);

  so::kine::Kinematics fbWorldPreviousBackup = worldFbPreviousBackup.getInverse();
  so::kine::Kinematics worldFbFinalBackup = backupFbKinematics_.back();

  so::kine::Kinematics lastTransformation = fbWorldPreviousBackup * worldFbFinalBackup;

  so::kine::Kinematics newKine = previousKine * lastTransformation;

  so::Vector3 tiltLocalLinVel = poseW_.rotation() * velW_.linear();
  so::Vector3 tiltLocalAngVel = poseW_.rotation() * velW_.angular();

  // koBackupFbKinematics->back() is the new last pose of the kinetics observer
  newKine.linVel = newKine.orientation.toMatrix3() * tiltLocalLinVel;
  newKine.angVel = newKine.orientation.toMatrix3() * tiltLocalAngVel;

  return newKine;
}

void MCValinor::updateNecessaryFrames(const mc_control::MCController & ctl, const mc_rbdyn::Robot & odomRobot)

{
  // pose of the floating base' frame in the world for the odometry robot
  worldFbKine_ = conversions::kinematics::fromSva(odomRobot.posW(), odomRobot.velW(), true);

  const auto & imu = ctl.robot(robot_).bodySensor(imuSensor_);
  const sva::PTransformd & imuXbs = imu.X_b_s();
  so::kine::Kinematics parentImuKine =
      conversions::kinematics::fromSva(imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

  // pose of the IMU's parent body in the world for the odometry robot
  const sva::PTransformd & parentPoseW = odomRobot.bodyPosW(imu.parentBody());
  // velocity of the IMU's parent body in the world for the odometry robot
  const sva::MotionVecd & v_0_imuParent = odomRobot.mbc().bodyVelW[odomRobot.bodyIndexByName(imu.parentBody())];

  // kinematics of the IMU's parent body in the world for the odometry robot
  so::kine::Kinematics worldParentKine = conversions::kinematics::fromSva(parentPoseW, v_0_imuParent, true);

  // pose and velocities of the IMU in the world frame for the odometry robot
  worldImuKine_ = worldParentKine * parentImuKine;

  // pose and velocities of the IMU in the floating base for the odometry robot
  fbImuKine_ = worldFbKine_.getInverse() * worldImuKine_;
}

void MCValinor::updatePoseAndVel(const mc_control::MCController & ctl)
{
  estimatedWorldFbKine_ = estimatedWorldImuKine_ * fbImuKine_.getInverse();

  if(odometryManager_.odometryType_ != OdometryType::None)
  {
    poseW_.translation() = estimatedWorldFbKine_.position();
    poseW_.rotation() = estimatedWorldFbKine_.orientation.toMatrix3().transpose();

    velW_.linear() = estimatedWorldFbKine_.linVel();
    velW_.angular() = estimatedWorldFbKine_.angVel();
  }
  else
  {
    if(odometryManager_.maintainedContacts().size() > 0)
    {
      ctlWorldAnchorPos_.setZero();
      fbAnchorPos_.setZero();

      for(auto contact : odometryManager_.maintainedContacts())
      {
        const auto & robot = ctl.robot(robot_);
        const std::string & surfaceName = contact->surfaceName();
        const sva::PTransformd & ctlSurfaceXbs = robot.surface(surfaceName).X_b_s();
        so::kine::Kinematics ctlParentSurfaceKine =
            conversions::kinematics::fromSva(ctlSurfaceXbs, so::kine::Kinematics::Flags::pose);
        const sva::PTransformd & ctlParentPoseW = robot.bodyPosW(robot.surface(surfaceName).bodyName());
        so::kine::Kinematics ctlWorldParentKine =
            conversions::kinematics::fromSva(ctlParentPoseW, so::kine::Kinematics::Flags::pose);

        const sva::PTransformd & ctlPosW = robot.posW();
        so::kine::Kinematics ctlWorldFbKine =
            conversions::kinematics::fromSva(ctlPosW, so::kine::Kinematics::Flags::pose);

        stateObservation::kine::Kinematics ctlWorldContactKine = ctlWorldParentKine * ctlParentSurfaceKine;
        // stateObservation::kine::Kinematics ctlFbContactPos_ = ctlWorldFbKine.getInverse() * ctlWorldContactKine;

        ctlWorldAnchorPos_ += ctlWorldContactKine.position() * contact->lambda_;
        stateObservation::kine::Kinematics imuFbKine = fbImuKine_.getInverse();
        fbAnchorPos_ = odometryManager_.getAnchorKineIn(imuFbKine).position();
      }
    }

    worldFbKine_.position = ctlWorldAnchorPos_ - worldFbKine_.orientation.toMatrix3() * fbAnchorPos_;

    so::Matrix3 ctlYawWithEstimatedTiltOri = so::kine::mergeRoll1Pitch1WithYaw2AxisAgnostic(
        estimatedWorldFbKine_.orientation.toMatrix3(), ctl.robot().posW().rotation().transpose());
    // orientation matrix that converts the estimated kinematics in the world frame back to the local frame, then
    // express the kinematics in the frame of the control robot
    so::Matrix3 ctlRealOri = ctlYawWithEstimatedTiltOri * estimatedWorldFbKine_.orientation.toMatrix3().transpose();

    poseW_.rotation() = ctlYawWithEstimatedTiltOri.transpose();
    poseW_.translation() = ctlWorldAnchorPos_ - ctlYawWithEstimatedTiltOri * fbAnchorPos_;

    velW_.angular() = ctlRealOri * worldFbKine_.angVel();
    velW_.linear() = ctlRealOri * estimatedWorldFbKine_.linVel();
  }
}

void MCValinor::update(mc_control::MCController & ctl)
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

    imu.orientation(estimatedWorldImuKine_.orientation.toQuaternion().inverse());
    rimu.orientation(estimatedWorldImuKine_.orientation.toQuaternion().inverse());
  }
}

void MCValinor::update(mc_rbdyn::Robot & robot)
{
  robot.posW(poseW_);
  robot.velW(velW_);
}

void MCValinor::setOdometryType(OdometryType newOdometryType)
{
  odometryManager_.setOdometryType(newOdometryType);
}

void MCValinor::addToLogger(const mc_control::MCController & ctl, mc_rtc::Logger & logger, const std::string & category)
{
  category_ = category;

  logger.addLogEntry(category + "_FloatingBase_world_pose", [this]() -> const sva::PTransformd & { return poseW_; });
  logger.addLogEntry(category + "_FloatingBase_world_vel", [this]() -> const sva::MotionVecd & { return velW_; });

  logger.addLogEntry(category + "_estimatedState_x1",
                     [this]() -> so::Vector3 { return estimator_.getEstimatedLocLinVel(); });
  logger.addLogEntry(category + "_estimatedState_x2prime",
                     [this]() -> so::Vector3 { return estimator_.getEstimatedIntermediaryTilt().normalized(); });
  logger.addLogEntry(category + "_estimatedState_x2",
                     [this]() -> so::Vector3 { return estimator_.getEstimatedTilt().normalized(); });
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

    logger.addLogEntry(category + "_constants_gains_alpha", [this]() -> double { return estimator_.getAlpha(); });
    logger.addLogEntry(category + "_constants_gains_beta", [this]() -> double { return estimator_.getBeta(); });
    logger.addLogEntry(category + "_constants_gains_gamma", [this]() -> double { return gamma_; });

    logger.addLogEntry(category + "_debug_OdometryType", [this]() -> std::string
                       { return stateObservation::odometry::odometryTypeToString(odometryManager_.odometryType_); });

    logger.addLogEntry(category + "_debug_yv", [this]() -> const so::Vector3 & { return yv_; });

    logger.addLogEntry(category + "_debug_realWorldImuLocAngVel",
                       [this, &ctl]() -> so::Vector3
                       {
                         const sva::PTransformd & realImuXbs = ctl.realRobot(robot_).bodySensor(imuSensor_).X_b_s();

                         so::kine::Kinematics realParentImuKine = conversions::kinematics::fromSva(
                             realImuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

                         const sva::PTransformd & realParentPoseW =
                             ctl.realRobot(robot_).bodyPosW(ctl.realRobot(robot_).bodySensor(imuSensor_).parentBody());

                         // Compute velocity of the imu in the control frame
                         auto & real_v_0_imuParent =
                             ctl.realRobot(robot_).mbc().bodyVelW[ctl.realRobot(robot_).bodyIndexByName(
                                 ctl.realRobot(robot_).bodySensor(imuSensor_).parentBody())];

                         so::kine::Kinematics realWorldParentKine =
                             conversions::kinematics::fromSva(realParentPoseW, real_v_0_imuParent, true);

                         so::kine::Kinematics realWorldImuKine_ = realWorldParentKine * realParentImuKine;

                         return realWorldImuKine_.orientation.toMatrix3().transpose() * realWorldImuKine_.angVel();
                       });

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

    logger.addLogEntry(category + "_debug_contactDetected", [this]() -> std::string
                       { return odometryManager_.contactsManager().contactsDetected() ? "contacts" : "no contacts"; });

    logger.addLogEntry(category + "_debug_ctlBodyVel",
                       [this, &ctl]() -> so::Vector3
                       {
                         const auto & robot = ctl.robot(robot_);
                         const auto & imu = robot.bodySensor(imuSensor_);

                         return robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())].linear();
                       });

    conversions::kinematics::addToLogger(logger, worldImuKine_, category + "_debug_worldImuKine");
    conversions::kinematics::addToLogger(logger, odometryManager_.bodyKine_, category + "_debug_worldImuKineOdom");

    conversions::kinematics::addToLogger(logger, imuAnchorKine_, category + "_debug_imuAnchorKine_");
    conversions::kinematics::addToLogger(logger, fbImuKine_, category + "_debug_fbImuKine_");

    conversions::kinematics::addToLogger(logger, worldFbKine_, category + "_debug_worldFbKine_");
  }
}

void MCValinor::removeFromLogger(mc_rtc::Logger &, const std::string &) {}

void MCValinor::addToGUI(const mc_control::MCController &,
                         mc_rtc::gui::StateBuilder &,
                         const std::vector<std::string> &)
{
  using namespace mc_state_observation::gui;
}

} // namespace mc_state_observation
EXPORT_OBSERVER_MODULE("MCValinor", mc_state_observation::MCValinor)
