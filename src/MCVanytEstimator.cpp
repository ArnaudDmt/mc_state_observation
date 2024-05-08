#include <mc_observers/ObserverMacros.h>

#include "mc_state_observation/measurements/measurements.h"
#include "mc_state_observation/odometry/LeggedOdometryManager.h"
#include <mc_state_observation/MCVanytEstimator.h>
#include <mc_state_observation/gui_helpers.h>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace mc_state_observation
{

namespace so = stateObservation;

using OdometryType = measurements::OdometryType;
using LoContactsManager = odometry::LeggedOdometryManager::ContactsManager;

MCVanytEstimator::MCVanytEstimator(const std::string & type, double dt, bool asBackup, const std::string & observerName)
: mc_observers::Observer(type, dt), estimator_(alpha_, beta_, 1 / (2 * M_PI), dt), odometryManager_(observerName)
{
  asBackup_ = asBackup;
  observerName_ = observerName;
}

void MCVanytEstimator::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
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

  if(config.has("tau")) { estimator_.setTau(config("tau")); }

  estimator_.setRho1(config("rho1"));
  config("muContacts", mu_contacts_);
  config("muGyro", mu_gyro_);

  anchorFrameFunction_ = "KinematicAnchorFrame::" + ctl.robot(robot_).name();
  // if a user-defined anchor frame function is given, we use it instead
  if(config.has("anchorFrameFunction"))
  {
    if(ctl.datastore().has(anchorFrameFunction_))
    {
      anchorFrameFunction_ = config("anchorFrameFunction", name() + "::" + ctl.robot(robot_).name());
    }
  }

  std::string odometryTypeStr = static_cast<std::string>(config("odometryType"));
  // we set the odometry type now because it will be necessary for the next check
  setOdometryType(measurements::stringToOdometryType(odometryTypeStr, observerName_));

  // specific configurations for the use of odometry.
  const auto & robot = ctl.robot(robot_);

  bool verbose = config("verbose", true);
  bool withYawEstimation = config("withYawEstimation", true);

  // surfaces used for the contact detection. If the desired detection method doesn't use surfaces, we make sure this
  // list is not filled in the configuration file to avoid the use of an undesired method.
  std::vector<std::string> surfacesForContactDetection;
  config("surfacesForContactDetection", surfacesForContactDetection);

  std::string contactsDetectionString = static_cast<std::string>(config("contactsDetection"));
  LoContactsManager::ContactsDetection contactsDetectionMethod =
      odometryManager_.contactsManager().stringToContactsDetection(contactsDetectionString, observerName_);

  if(surfacesForContactDetection.size() > 0
     && contactsDetectionMethod != LoContactsManager::ContactsDetection::Surfaces)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Another type of contacts detection than Surfaces is currently "
                                                     "used, please change it to 'Surfaces' or empty the "
                                                     "surfacesForContactDetection variable");
  }

  double contactDetectionPropThreshold = config("contactDetectionPropThreshold", 0.11);

  odometry::LeggedOdometryManager::Configuration odomConfig(robot_, observerName_, odometryManager_.odometryType_);
  odomConfig.velocityUpdate(odometry::LeggedOdometryManager::VelocityUpdate::NoUpdate)
      .withYawEstimation(withYawEstimation);

  if(contactsDetectionMethod == LoContactsManager::ContactsDetection::Surfaces)
  {
    measurements::ContactsManagerSurfacesConfiguration contactsConfig(observerName_, surfacesForContactDetection);
    contactsConfig.contactDetectionPropThreshold(contactDetectionPropThreshold).verbose(verbose);
    odometryManager_.init(ctl, odomConfig, contactsConfig);
  }
  if(contactsDetectionMethod == LoContactsManager::ContactsDetection::Sensors)
  {
    std::vector<std::string> forceSensorsToOmit = config("forceSensorsToOmit", std::vector<std::string>());

    measurements::ContactsManagerSensorsConfiguration contactsConfig(observerName_);
    contactsConfig.contactDetectionPropThreshold(contactDetectionPropThreshold)
        .verbose(verbose)
        .forceSensorsToOmit(forceSensorsToOmit);
    odometryManager_.init(ctl, odomConfig, contactsConfig);
  }
  if(contactsDetectionMethod == LoContactsManager::ContactsDetection::Solver)
  {
    measurements::ContactsManagerSolverConfiguration contactsConfig(observerName_);
    contactsConfig.contactDetectionPropThreshold(contactDetectionPropThreshold).verbose(verbose);
    odometryManager_.init(ctl, odomConfig, contactsConfig);
  }
}

void MCVanytEstimator::reset(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());

  // the updated robot has the same floating base's pose than the control robot, but its encoders are updated. We use it
  // to get more accurate local Kinematics.
  ctl.gui()->addElement(
      {"Robots"},
      mc_rtc::gui::Robot(observerName_, [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));

  const auto & imu = robot.bodySensor(imuSensor_);

  // reset of the floating base kinematics
  poseW_ = robot.posW();
  velW_ = robot.velW();
  prevPoseW_ = sva::PTransformd::Identity();
  velW_ = sva::MotionVecd::Zero();

  // initialization of the estimator
  so::kine::Kinematics initParentImuKine = conversions::kinematics::fromSva(
      imu.X_b_s(), so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

  // kinematics of the IMU's parent body in the world for the odometry robot
  so::kine::Kinematics initWorldParentKine =
      conversions::kinematics::fromSva(robot.bodyPosW(imu.parentBody()), so::kine::Kinematics::Flags::pose);

  // pose and velocities of the IMU in the world frame for the odometry robot
  so ::kine::Kinematics initWorldImuKine = initWorldParentKine * initParentImuKine;
  const Eigen::Matrix3d cOri = (imu.X_b_s() * ctl.robot(robot_).bodyPosW(imu.parentBody())).rotation();

  so::Vector3 initX2 = initWorldImuKine.orientation.toMatrix3().transpose() * so::Vector3::UnitZ();

  estimator_.initEstimator(initWorldImuKine.position(), so::Vector3::Zero(), initX2,
                           initWorldImuKine.orientation.toVector4());

  anchorFrameJumped_ = false;
  iter_ = 0;
  imuVelC_ = sva::MotionVecd::Zero();
  X_C_IMU_ = sva::PTransformd::Identity();

  odometryManager_.reset();
}

bool MCVanytEstimator::run(const mc_control::MCController & ctl)
{
  const auto & realRobot = ctl.realRobot(robot_);
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  if(logger.t() > 1.0)
  {
    alpha_ = finalAlpha_;
    beta_ = finalBeta_;
    gamma_ = finalGamma_;
  }

  odometryManager_.initLoop(ctl, logger, odometry::LeggedOdometryManager::RunParameters());
  runTiltEstimator(ctl, odometryManager_.odometryRobot());

  iter_++;

  /* Update of the observed robot */
  my_robots_->robot().mbc().q = realRobot.mbc().q;
  update(my_robots_->robot());

  return true;
}

void MCVanytEstimator::updateNecessaryFramesOdom(const mc_control::MCController & ctl,
                                                 const mc_rbdyn::Robot & odomRobot)

{
  // pose of the floating base' frame in the world for the odometry robot
  updatedWorldFbKine_ = conversions::kinematics::fromSva(odomRobot.posW(), odomRobot.velW(), true);

  const auto & imu = ctl.robot(robot_).bodySensor(imuSensor_);
  const sva::PTransformd & imuXbs = imu.X_b_s();
  so::kine::Kinematics parentImuKine =
      conversions::kinematics::fromSva(imuXbs, so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

  // pose of the IMU's parent body in the world for the odometry robot
  const sva::PTransformd & updatedParentPoseW = odomRobot.bodyPosW(imu.parentBody());
  // velocity of the IMU's parent body in the world for the odometry robot
  const sva::MotionVecd & updated_v_0_imuParent = odomRobot.mbc().bodyVelW[odomRobot.bodyIndexByName(imu.parentBody())];

  // kinematics of the IMU's parent body in the world for the odometry robot
  so::kine::Kinematics updatedWorldParentKine =
      conversions::kinematics::fromSva(updatedParentPoseW, updated_v_0_imuParent, true);

  // pose and velocities of the IMU in the world frame for the odometry robot
  updatedWorldImuKine_ = updatedWorldParentKine * parentImuKine;

  // pose and velocities of the IMU in the floating base for the odometry robot
  updatedFbImuKine_ = updatedWorldFbKine_.getInverse() * updatedWorldImuKine_;

  // position and linear velocity of the anchor point in the frame of the IMU.
  updatedImuAnchorKine_ = odometryManager_.getAnchorKineIn(updatedWorldImuKine_);

  if(odometryManager_.anchorPointMethodChanged_) { updatedImuAnchorKine_.linVel().setZero(); }
}

void MCVanytEstimator::runTiltEstimator(const mc_control::MCController & ctl, const mc_rbdyn::Robot & odomRobot)
{
  estimator_.setAlpha(alpha_);
  estimator_.setBeta(beta_);
  estimator_.setGamma(gamma_);

  updateNecessaryFramesOdom(ctl, odomRobot);

  const auto & imu = ctl.robot(robot_).bodySensor(imuSensor_);

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
    //  estimator_.setGamma(0.1);
  }
  else
  {
    estimator_.setAlpha(alpha_);
    estimator_.setBeta(beta_);
    //  estimator_.setGamma(gamma_);

    yv_ = -imu.angularVelocity().cross(updatedImuAnchorKine_.position()) - updatedImuAnchorKine_.linVel();
  }

  if(odometryManager_.anchorPointMethodChanged_) { estimator_.resetImuLocVelHat(); }

  estimator_.setMeasurement(yv_, imu.linearAcceleration(), imu.angularVelocity(), k + 1);
  measurements_ = estimator_.getMeasurement(estimator_.getMeasurementTime());

  if(odometryManager_.maintainedContacts().size() > 0)
  {
    estimator_.addPositionMeasurement(odometryManager_.getWorldRefAnchorPos(), updatedImuAnchorKine_.position());
  }

  // estimation of the state with the complementary filters
  xk_ = estimator_.getEstimatedState(k + 1);

  // retrieving the estimated orientation
  so::kine::Orientation estimatedOri;
  estimatedOri.fromVector4(xk_.tail(4));

  estimatedRotationIMU_ = estimatedOri.toMatrix3();

  // Estimated orientation of the floating base in the world (especially the tilt)
  R_0_fb_ = estimatedRotationIMU_ * updatedFbImuKine_.orientation.toMatrix3().transpose();

  // retrieving the estimated position
  const so::Vector3 worldImuPos = xk_.head(3);

  so::Vector3 worldFbPos = worldImuPos + estimatedRotationIMU_ * updatedFbImuKine_.getInverse().position();

  odometryManager_.run(ctl, odometry::LeggedOdometryManager::KineParams(poseW_).attitude(R_0_fb_).position(worldFbPos));

  updatePoseAndVel(xk_.segment(3, 3), imu.angularVelocity());
  backupFbKinematics_.push_back(poseW_);

  for(auto * mContact : odometryManager_.maintainedContacts())
  {
    const so::kine::Kinematics & worldContactRefKine = mContact->worldRefKineBeforeCorrection_;

    const so::kine::Kinematics & contactFbKineOdometryRobot = mContact->contactFbKine_;

    const so::kine::Kinematics worldImuKineOdometryRobot =
        worldContactRefKine * contactFbKineOdometryRobot * updatedFbImuKine_;

    measuredOri_ = worldImuKineOdometryRobot.orientation.toMatrix3();

    estimator_.addOrientationMeasurement(measuredOri_, mu_contacts_);
  }
}

void MCVanytEstimator::updatePoseAndVel(const so::Vector3 & localWorldImuLinVel,
                                        const so::Vector3 & localWorldImuAngVel)
{
  correctedWorldFbKine_.position = poseW_.translation();
  correctedWorldFbKine_.orientation = R_0_fb_;

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

  // the velocity of the odometry robot was obtained using finite differences. We give it our estimated velocity which
  // is more accurate.
  odometryManager_.odometryRobot().velW(velW_);
}

void MCVanytEstimator::update(mc_control::MCController & ctl)
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

void MCVanytEstimator::update(mc_rbdyn::Robot & robot)
{
  robot.posW(poseW_);
  robot.velW(velW_);
}

const so::kine::Kinematics MCVanytEstimator::backupFb(
    boost::circular_buffer<so::kine::Kinematics> * koBackupFbKinematics)
{
  // new initial pose of the floating base
  so::kine::Kinematics worldResetKine = *(koBackupFbKinematics->begin());

  // original initial pose of the floating base
  so::kine::Kinematics worldFbInitBackup =
      conversions::kinematics::fromSva(backupFbKinematics_.front(), so::kine::Kinematics::Flags::pose);

  so::kine::Kinematics fbWorldInitBackup = worldFbInitBackup.getInverse();

  // we apply the transformation from the initial pose to the intermediates pose estimated by the tilt estimator to the
  // new starting pose of the Kinetics Observer
  for(int i = 0; i < koBackupFbKinematics->size(); i++)
  {
    // Intermediary pose of the floating base estimated by the tilt estimator
    so::kine::Kinematics worldFbIntermBackup =
        conversions::kinematics::fromSva(backupFbKinematics_.at(i), so::kine::Kinematics::Flags::pose);

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

so::kine::Kinematics MCVanytEstimator::applyLastTransformation(const so::kine::Kinematics & previousKine)
{
  so::kine::Kinematics worldFbPreviousBackup = conversions::kinematics::fromSva(
      backupFbKinematics_.at(backupFbKinematics_.size() - 2), so::kine::Kinematics::Flags::pose);

  so::kine::Kinematics fbWorldPreviousBackup = worldFbPreviousBackup.getInverse();
  so::kine::Kinematics worldFbFinalBackup =
      conversions::kinematics::fromSva(backupFbKinematics_.back(), so::kine::Kinematics::Flags::pose);

  so::kine::Kinematics lastTransformation = fbWorldPreviousBackup * worldFbFinalBackup;

  so::kine::Kinematics newKine = previousKine * lastTransformation;

  so::Vector3 tiltLocalLinVel = poseW_.rotation() * velW_.linear();
  so::Vector3 tiltLocalAngVel = poseW_.rotation() * velW_.angular();

  // koBackupFbKinematics->back() is the new last pose of the kinetics observer
  newKine.linVel = newKine.orientation.toMatrix3() * tiltLocalLinVel;
  newKine.angVel = newKine.orientation.toMatrix3() * tiltLocalAngVel;

  return newKine;
}

void MCVanytEstimator::setOdometryType(OdometryType newOdometryType)
{
  odometryManager_.setOdometryType(newOdometryType);
}

void MCVanytEstimator::addToLogger(const mc_control::MCController & ctl,
                                   mc_rtc::Logger & logger,
                                   const std::string & category)
{
  logger.addLogEntry(category + "_estimatedState_p", [this]() -> so::Vector3 { return xk_.segment(0, 3); });

  logger.addLogEntry(category + "_estimatedState_x1", [this]() -> so::Vector3 { return xk_.segment(3, 3); });

  logger.addLogEntry(category + "_debug_measuredOri_",
                     [this]() -> Eigen::Quaterniond { return measuredOri_.toQuaternion().inverse(); });

  logger.addLogEntry(category + "_debug_posContacts_",
                     [this]() -> const so::Vector3 & { return estimator_.getPosContacts(); });

  logger.addLogEntry(category + "_debug_oriCorrection_",
                     [this]() -> const so::Vector3 & { return estimator_.getOriCorrection(); });

  logger.addLogEntry(category + "_debug_posX1_", [this]() -> const so::Vector3 & { return estimator_.getPosX1(); });

  logger.addLogEntry(category + "_estimatedState_x2prime",
                     [this]() -> so::Vector3 { return xk_.segment(6, 3).normalized(); });
  logger.addLogEntry(category + "_estimatedState_R",
                     [this]()
                     {
                       so::kine::Orientation ori;
                       ori.fromVector4(xk_.tail(4));
                       return ori.toQuaternion().inverse();
                     });
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

  logger.addLogEntry(category + "_constants_alpha", [this]() -> double { return estimator_.getAlpha(); });
  logger.addLogEntry(category + "_constants_beta", [this]() -> double { return estimator_.getBeta(); });
  logger.addLogEntry(category + "_constants_gamma", [this]() -> double { return estimator_.getGamma(); });
  logger.addLogEntry(category + "_constants_rho1", [this]() -> double { return estimator_.getRho1(); });
  logger.addLogEntry(category + "_constants_tau", [this]() -> double { return estimator_.getTau(); });

  logger.addLogEntry(category + "_debug_OdometryType",
                     [this]() -> std::string
                     { return measurements::odometryTypeToSstring(odometryManager_.odometryType_); });

  logger.addLogEntry(category + "_IMU_world_orientation",
                     [this]() { return Eigen::Quaterniond{estimatedRotationIMU_}; });

  logger.addLogEntry(category + "_IMU_AnchorFrame_pose", [this]() -> const sva::PTransformd & { return X_C_IMU_; });
  logger.addLogEntry(category + "_IMU_AnchorFrame_linVel", [this]() -> const sva::MotionVecd & { return imuVelC_; });
  logger.addLogEntry(category + "_FloatingBase_world_pose", [this]() -> const sva::PTransformd & { return poseW_; });
  logger.addLogEntry(category + "_FloatingBase_world_vel", [this]() -> const sva::MotionVecd & { return velW_; });
  logger.addLogEntry(category + "_debug_x1", [this]() -> const so::Vector3 & { return yv_; });

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

  logger.addLogEntry(category + "_debug_contactDetected",
                     [this]() -> std::string
                     { return odometryManager_.contactsManager().contactsDetected() ? "contacts" : "no contacts"; });

  logger.addLogEntry(category + "_debug_ctlBodyVel",
                     [this, &ctl]() -> so::Vector3
                     {
                       const auto & robot = ctl.robot(robot_);
                       const auto & imu = robot.bodySensor(imuSensor_);

                       return robot.mbc().bodyVelW[robot.bodyIndexByName(imu.parentBody())].linear();
                     });

  conversions::kinematics::addToLogger(logger, updatedWorldImuKine_, category + "_debug_updatedWorldImuKine");
  conversions::kinematics::addToLogger(logger, updatedImuAnchorKine_, category + "_debug_updatedImuAnchorKine_");
  conversions::kinematics::addToLogger(logger, updatedFbImuKine_, category + "_debug_updatedFbImuKine_");

  conversions::kinematics::addToLogger(logger, updatedWorldFbKine_, category + "_debug_updatedWorldFbKine_");
  conversions::kinematics::addToLogger(logger, correctedWorldImuKine_, category + "_debug_correctedWorldImuKine_");
}

void MCVanytEstimator::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_imuVelC");
  logger.removeLogEntry(category + "_imuPoseC");
  logger.removeLogEntry(category + "_imuEstRotW");
  logger.removeLogEntry(category + "_controlAnchorFrame");
}

void MCVanytEstimator::addToGUI(const mc_control::MCController &,
                                mc_rtc::gui::StateBuilder & gui,
                                const std::vector<std::string> & category)
{
  using namespace mc_state_observation::gui;
  gui.addElement(category, make_input_element("alpha", alpha_), make_input_element("beta", beta_),
                 make_input_element("gamma", gamma_));
}

} // namespace mc_state_observation
EXPORT_OBSERVER_MODULE("MCVanytEstimator", mc_state_observation::MCVanytEstimator)
