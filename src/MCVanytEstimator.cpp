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
: mc_observers::Observer(type, dt), estimator_(alpha_, beta_, 1 / (2 * M_PI), dt), odometryManager_(observerName, dt)
{
  asBackup_ = asBackup;
  observerName_ = observerName;
}

void MCVanytEstimator::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  imuSensor_ = config("imuSensor", ctl.robot().bodySensor().name());

  if(ctl.realRobot(robot_).hasBodySensor("VisualGyroSensor"))
  {
    unsigned long delayedOriBufferCapacity = static_cast<unsigned long>(10 / ctl.timeStep);
    estimator_.setBufferCapacity(delayedOriBufferCapacity);
  }

  config("maxAnchorFrameDiscontinuity", maxAnchorFrameDiscontinuity_);
  config("updateRobot", updateRobot_);
  config("updateSensor", updateSensor_);

  auto odomConfig = config("leggedOdometry");
  auto contactsConfig = config("contacts");
  auto filterGainsConfig = config("filterGains");

  filterGainsConfig("initAlpha", alpha_);
  filterGainsConfig("initBeta", beta_);
  filterGainsConfig("initRho", rho_);
  filterGainsConfig("finalAlpha", finalAlpha_);
  filterGainsConfig("finalBeta", finalBeta_);
  filterGainsConfig("finalRho", finalRho_);
  filterGainsConfig("gammaContacts", gamma_contacts_);
  filterGainsConfig("lambdaContacts", lambda_contacts_);
  filterGainsConfig("muContacts", mu_contacts_);
  filterGainsConfig("muGyro", mu_gyroscope_);

  anchorFrameFunction_ = "KinematicAnchorFrame::" + ctl.robot(robot_).name();
  // if a user-defined anchor frame function is given, we use it instead
  if(config.has("anchorFrameFunction"))
  {
    if(ctl.datastore().has(anchorFrameFunction_))
    {
      anchorFrameFunction_ = config("anchorFrameFunction", name() + "::" + ctl.robot(robot_).name());
    }
  }

  std::string odometryTypeStr = static_cast<std::string>(odomConfig("odometryType"));
  // we set the odometry type now because it will be necessary for the next check
  setOdometryType(measurements::stringToOdometryType(odometryTypeStr, observerName_));

  // specific configurations for the use of odometry.
  bool verbose = config("verbose", true);
  bool withYawEstimation = odomConfig("withYawEstimation", true);
  bool correctContacts = odomConfig("correctContacts", true);

  // surfaces used for the contact detection. If the desired detection method doesn't use surfaces, we make sure this
  // list is not filled in the configuration file to avoid the use of an undesired method.
  std::vector<std::string> surfacesForContactDetection;
  contactsConfig("surfacesForContactDetection", surfacesForContactDetection);

  std::string contactsDetectionString = static_cast<std::string>(contactsConfig("contactsDetection"));
  LoContactsManager::ContactsDetection contactsDetectionMethod =
      odometryManager_.contactsManager().stringToContactsDetection(contactsDetectionString, observerName_);

  if(surfacesForContactDetection.size() > 0
     && contactsDetectionMethod != LoContactsManager::ContactsDetection::Surfaces)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Another type of contacts detection than Surfaces is currently "
                                                     "used, please change it to 'Surfaces' or empty the "
                                                     "surfacesForContactDetection variable");
  }

  double contactDetectionPropThreshold = contactsConfig("contactDetectionPropThreshold", 0.11);

  odometry::LeggedOdometryManager::Configuration odometryConfig(robot_, observerName_, odometryManager_.odometryType_);
  odometryConfig.velocityUpdate(odometry::LeggedOdometryManager::VelocityUpdate::NoUpdate)
      .withYawEstimation(withYawEstimation)
      .correctContacts(correctContacts);

  if(odomConfig.has("kappa"))
  {
    double kappa = odomConfig("kappa");
    odometryManager_.kappa(kappa);
  }
  if(odomConfig.has("lambdaInf"))
  {
    double lambdaInf = odomConfig("lambdaInf");
    odometryManager_.lambdaInf(lambdaInf);
  }
  if(contactsDetectionMethod == LoContactsManager::ContactsDetection::Surfaces)
  {
    if(surfacesForContactDetection.size() == 0)
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("The list of surfaces for the contact detection is empty.");
    }

    measurements::ContactsManagerSurfacesConfiguration contactsConfig(observerName_, surfacesForContactDetection);
    contactsConfig.contactDetectionPropThreshold(contactDetectionPropThreshold).verbose(verbose);
    odometryManager_.init(ctl, odometryConfig, contactsConfig);
  }
  if(contactsDetectionMethod == LoContactsManager::ContactsDetection::Sensors)
  {
    std::vector<std::string> forceSensorsToOmit = odomConfig("forceSensorsToOmit", std::vector<std::string>());

    measurements::ContactsManagerSensorsConfiguration contactsConfig(observerName_);
    contactsConfig.contactDetectionPropThreshold(contactDetectionPropThreshold)
        .verbose(verbose)
        .forceSensorsToOmit(forceSensorsToOmit);
    odometryManager_.init(ctl, odometryConfig, contactsConfig);
  }
  if(contactsDetectionMethod == LoContactsManager::ContactsDetection::Solver)
  {
    measurements::ContactsManagerSolverConfiguration contactsConfig(observerName_);
    contactsConfig.contactDetectionPropThreshold(contactDetectionPropThreshold).verbose(verbose);
    odometryManager_.init(ctl, odometryConfig, contactsConfig);
  }
}

void MCVanytEstimator::reset(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());

  // the updated robot has the same floating base's pose than the control robot, but its encoders are updated. We use it
  // to get more accurate local Kinematics.
  ctl.gui()->addElement(
      {"Robots"},
      mc_rtc::gui::Robot(observerName_, [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));

  const auto & imu = robot.bodySensor(imuSensor_);

  // reset of the floating base kinematics
  poseW_ = realRobot.posW();
  velW_ = realRobot.velW();
  velW_ = sva::MotionVecd::Zero();

  // initialization of the estimator
  so::kine::Kinematics initParentImuKine = conversions::kinematics::fromSva(
      imu.X_b_s(), so::kine::Kinematics::Flags::pose | so::kine::Kinematics::Flags::vel);

  // kinematics of the IMU's parent body in the world for the odometry robot
  so::kine::Kinematics initWorldParentKine =
      conversions::kinematics::fromSva(realRobot.bodyPosW(imu.parentBody()), so::kine::Kinematics::Flags::pose);

  // pose and velocities of the IMU in the world frame for the odometry robot
  so ::kine::Kinematics initWorldImuKine = initWorldParentKine * initParentImuKine;
  const Eigen::Matrix3d cOri = (imu.X_b_s() * realRobot.bodyPosW(imu.parentBody())).rotation();
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
    rho_ = finalRho_;
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

  // position and linear velocity of the anchor point in the frame of the IMU.
  imuAnchorKine_ = odometryManager_.getAnchorKineIn(worldImuKine_);

  if(odometryManager_.anchorPointMethodChanged_) { imuAnchorKine_.linVel().setZero(); }
}

void MCVanytEstimator::runTiltEstimator(const mc_control::MCController & ctl, const mc_rbdyn::Robot & odomRobot)
{
  if(ctl.realRobot(robot_).hasBodySensor("VisualGyroSensor"))
  {
    auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

    removeDelayedOriMeasLogs(logger);
    if(ctl.datastore().has("VisualGyroSensorDelay"))
    {
      const mc_rbdyn::BodySensor & visualGyro = ctl.realRobot(robot_).bodySensor("VisualGyroSensor");
      delayedOriMeasurementHandler(ctl, visualGyro.orientation().toRotationMatrix(),
                                   ctl.datastore().get<unsigned long>("VisualGyroSensorDelay"), mu_gyroscope_);
    }
  }

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
  }
  else
  {
    estimator_.setAlpha(alpha_);
    estimator_.setBeta(beta_);
    estimator_.setRho(rho_);

    yv_ = -imu.angularVelocity().cross(imuAnchorKine_.position()) - imuAnchorKine_.linVel();
  }

  if(odometryManager_.anchorPointMethodChanged_)
  {
    estimator_.setMeasurement(yv_, imu.linearAcceleration(), imu.angularVelocity(), k + 1, true);
  }
  else { estimator_.setMeasurement(yv_, imu.linearAcceleration(), imu.angularVelocity(), k + 1, false); }

  estimator_.setMeasurement(yv_, imu.linearAcceleration(), imu.angularVelocity(), k + 1);

  measurements_ = estimator_.getMeasurement(estimator_.getMeasurementTime());

  for(auto * mContact : odometryManager_.maintainedContacts())
  {
    const so::kine::Kinematics & worldContactRefKine = mContact->worldRefKine_;
    const so::kine::Kinematics & contactFbKine = mContact->contactFbKine_;
    const so::kine::Kinematics worldImuKine_fromContactRef = worldContactRefKine * contactFbKine * fbImuKine_;
    const so::Vector3 imuContactPos =
        -fbImuKine_.orientation.toMatrix3().transpose() * fbImuKine_.position()
        - fbImuKine_.orientation.toMatrix3().transpose()
              * (contactFbKine.orientation.toMatrix3().transpose() * contactFbKine.position());

    measuredOri_ = worldImuKine_fromContactRef.orientation.toMatrix3();

    estimator_.addOrientationMeasurement(measuredOri_, mu_contacts_ * mContact->lambda());
    estimator_.addContactPosMeasurement(worldContactRefKine.position(), imuContactPos, lambda_contacts_,
                                        gamma_contacts_);
  }

  // estimation of the state with the complementary filters
  xk_ = estimator_.getEstimatedState(k + 1);

  // retrieving the estimated orientation
  so::kine::Orientation estimatedOri;
  estimatedOri.fromVector4(xk_.tail(4));

  estimatedRotationIMU_ = estimatedOri.toMatrix3();

  // Estimated orientation of the floating base in the world (especially the tilt)
  R_0_fb_ = estimatedRotationIMU_ * fbImuKine_.orientation.toMatrix3().transpose();

  // retrieving the estimated position
  const so::Vector3 worldImuPos = xk_.segment<3>(6);

  so::Vector3 worldFbPos = worldImuPos - R_0_fb_ * fbImuKine_.position();

  odometryManager_.run(ctl, odometry::LeggedOdometryManager::KineParams(poseW_).attitude(R_0_fb_).position(worldFbPos));

  updatePoseAndVel(xk_.segment(0, 3), imu.angularVelocity());

  /* Backups */

  // for the Kinetics Observer
  backupFbKinematics_.push_back(conversions::kinematics::fromSva(poseW_, so::kine::Kinematics::Flags::pose));
}

void MCVanytEstimator::updatePoseAndVel(const so::Vector3 & localWorldImuLinVel,
                                        const so::Vector3 & localWorldImuAngVel)
{
  correctedWorldFbKine_.position = poseW_.translation();
  correctedWorldFbKine_.orientation = R_0_fb_;

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
  so::kine::Kinematics worldFbInitBackup = backupFbKinematics_.front();

  so::kine::Kinematics fbWorldInitBackup = worldFbInitBackup.getInverse();

  // we apply the transformation from the initial pose to the intermediates pose estimated by the tilt estimator to the
  // new starting pose of the Kinetics Observer
  for(int i = 0; i < koBackupFbKinematics->size(); i++)
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

void MCVanytEstimator::delayedOriMeasurementHandler(const mc_control::MCController & ctl,
                                                    const so::Matrix3 & meas,
                                                    unsigned long delay,
                                                    double gain)
{
  const auto & iterationsBuffer = estimator_.getIterationsBuffer();
  // Let us denote k the time on which the orientation measurement started to be computed, but is still not available.
  // We replay the estimation at time k using the buffered state and measurements, this time using the newly available
  // orientation measurement. We then apply the transformation between the time k+1 and the current iteration.

  delayedOriMeas_.meas_ = meas;
  delayedOriMeas_.gain_ = gain;
  delayedOriMeas_.updatedPoseWithoutMeas_ = iterationsBuffer.at(delay - 1).updatedPose_;

  mc_rtc::log::info("Received an orientation measurement with a delay of " + std::to_string(delay) + " iterations");
  if(iterationsBuffer.empty())
  {
    mc_rtc::log::warning("A delayed measurement was received although the estimation just started. Please make sure "
                         "that you pass a delayed orientation measurement. The measurement will be ignored.");
    return;
  }
  if(delay > iterationsBuffer.size() || iterationsBuffer.size() == 0)
  {
    mc_rtc::log::warning("The orientation measurement is too old, the measurement will be ignored.");
    return;
  }

  // we replay the estimation made by the filter but this time with the orientation measurement.
  so::Vector replayedWorldImuEstWithOri = estimator_.replayIterationsWithDelayedOri(delay, meas, gain);
  so::kine::Kinematics replayedWorldImuKineEst(replayedWorldImuEstWithOri.tail(7), so::kine::Kinematics::Flags::pose);

  // we get the new kinematics of the floating base in the world frame from the ones of the IMU
  so::Matrix3 replayedWorldFbOri =
      replayedWorldImuKineEst.orientation.toMatrix3() * fbImuKine_.orientation.toMatrix3().transpose();
  so::Vector3 replayedWorldFbPos = replayedWorldImuKineEst.position() - replayedWorldFbOri * fbImuKine_.position();

  sva::PTransformd newWorldFbPose_(replayedWorldFbOri.transpose(), replayedWorldFbPos);
  odometryManager_.replaceRobotPose(newWorldFbPose_);

  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();
  delayedOriMeas_.updatedPoseWithMeas_ = iterationsBuffer.at(delay - 1).updatedPose_;
  addDelayedOriMeasLogs(logger, observerName_);
}

void MCVanytEstimator::setOdometryType(OdometryType newOdometryType)
{
  if((newOdometryType != measurements::OdometryType::Odometry6d)
     && (newOdometryType != measurements::OdometryType::Flat))
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("Please choose between these two odometry types: [6D, Flat]");
  }

  odometryManager_.setOdometryType(newOdometryType);
}

void MCVanytEstimator::addDelayedOriMeasLogs(mc_rtc::Logger & logger, const std::string & category)
{
  logger.addLogEntry(category + "_delayedOriMeas_" + "meas", &delayedOriMeas_,
                     [this]() -> Eigen::Quaterniond { return Eigen::Quaterniond(delayedOriMeas_.meas_).inverse(); });
  logger.addLogEntry(category + "_delayedOriMeas_" + "gain", &delayedOriMeas_,
                     [this]() -> double { return delayedOriMeas_.gain_; });
  logger.addLogEntry(category + "_delayedOriMeas_" + "delayedoriRecieved", &delayedOriMeas_,
                     []() -> std::string { return "received"; });

  conversions::kinematics::addToLogger(logger, delayedOriMeas_.updatedPoseWithoutMeas_,
                                       category + "_delayedOriMeas_" + "updatedPoseWithoutMeas");
  conversions::kinematics::addToLogger(logger, delayedOriMeas_.updatedPoseWithMeas_,
                                       category + "_delayedOriMeas_" + "updatedPoseWithMeas");
}

void MCVanytEstimator::removeDelayedOriMeasLogs(mc_rtc::Logger & logger)
{
  logger.removeLogEntries(&delayedOriMeas_);
  conversions::kinematics::removeFromLogger(logger, delayedOriMeas_.updatedPoseWithMeas_);
  conversions::kinematics::removeFromLogger(logger, delayedOriMeas_.updatedPoseWithoutMeas_);
}

void MCVanytEstimator::addToLogger(const mc_control::MCController & ctl,
                                   mc_rtc::Logger & logger,
                                   const std::string & category)
{
  odometryManager_.addToLogger(logger, category + "_leggedOdometryManager_");
  logger.addLogEntry(category + "_estimatedState_p", [this]() -> so::Vector3 { return xk_.segment(6, 3); });

  logger.addLogEntry(category + "_estimatedState_x1", [this]() -> so::Vector3 { return xk_.segment(0, 3); });

  logger.addLogEntry(category + "_debug_measuredOri_",
                     [this]() -> Eigen::Quaterniond { return measuredOri_.toQuaternion().inverse(); });

  logger.addLogEntry(category + "_debug_corrections_oriCorrection_",
                     [this]() -> const so::Vector3 & { return estimator_.getOriCorrection(); });
  logger.addLogEntry(category + "_debug_corrections_oriCorrFromOriMeas_",
                     [this]() -> const so::Vector3 & { return estimator_.getOriCorrFromOriMeas(); });
  logger.addLogEntry(category + "_debug_corrections_posCorrFromContactPos_",
                     [this]() -> const so::Vector3 & { return estimator_.getPosCorrectionFromContactPos(); });
  logger.addLogEntry(category + "_debug_corrections_oriCorrFromContactPos_",
                     [this]() -> const so::Vector3 & { return estimator_.geOriCorrectionFromContactPos(); });

  logger.addLogEntry(category + "_estimatedState_x2prime",
                     [this]() -> so::Vector3 { return xk_.segment(3, 3).normalized(); });
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

  logger.addLogEntry(category + "_constants_gains_alpha", [this]() -> double { return estimator_.getAlpha(); });
  logger.addLogEntry(category + "_constants_gains_beta", [this]() -> double { return estimator_.getBeta(); });
  logger.addLogEntry(category + "_constants_gains_rho", [this]() -> double { return estimator_.getRho(); });
  logger.addLogEntry(category + "_constants_gains_contacts_mu", [this]() -> double { return mu_contacts_; });
  logger.addLogEntry(category + "_constants_gains_contacts_lambda", [this]() -> double { return lambda_contacts_; });
  logger.addLogEntry(category + "_constants_gains_contacts_gamma", [this]() -> double { return gamma_contacts_; });

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

  conversions::kinematics::addToLogger(logger, worldImuKine_, category + "_debug_worldImuKine");
  conversions::kinematics::addToLogger(logger, imuAnchorKine_, category + "_debug_imuAnchorKine_");
  conversions::kinematics::addToLogger(logger, fbImuKine_, category + "_debug_fbImuKine_");

  conversions::kinematics::addToLogger(logger, worldFbKine_, category + "_debug_worldFbKine_");
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
                                mc_rtc::gui::StateBuilder &,
                                const std::vector<std::string> &)
{
  using namespace mc_state_observation::gui;
  // gui.addElement(category, make_input_element("alpha", alpha_), make_input_element("beta", beta_));
}

} // namespace mc_state_observation
EXPORT_OBSERVER_MODULE("MCVanytEstimator", mc_state_observation::MCVanytEstimator)
