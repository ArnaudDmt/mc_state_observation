/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/logging.h>

#include <mc_state_observation/MCKineticsObserver.h>
#include <mc_state_observation/gui_helpers.h>

#include "mc_state_observation/measurements/measurements.h"

namespace so = stateObservation;
namespace mc_state_observation
{
MCKineticsObserver::MCKineticsObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), maxContacts_(3), maxIMUs_(1), observer_(maxContacts_, maxIMUs_),
  tiltObserver_(type, dt, true)
{
  observer_.setSamplingTime(dt);
}

///////////////////////////////////////////////////////////////////////
/// --------------------------Core functions---------------------------
///////////////////////////////////////////////////////////////////////

void MCKineticsObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  tiltObserver_.name(name() + "BackupTiltObserver");
  tiltObserver_.configure(ctl, config);

  robot_ = config("robot", ctl.robot().name());

  imuNames_ = config("imuNames", std::vector<std::string>());
  listIMUs_.clear();
  if(!imuNames_.empty())
  {
    for(size_t i = 0; i < imuNames_.size(); ++i) { listIMUs_.push_back({static_cast<int>(i), imuNames_[i]}); }
  }
  else { listIMUs_.push_back({0, ctl.robot(robot_).bodySensor().name()}); }

  config("debug", debug_);
  config("verbose", verbose_);

  // we set the desired type of odometry
  auto leggedOdomConfig = config("leggedOdometry");
  std::string typeOfOdometry = static_cast<std::string>(leggedOdomConfig("odometryType"));
  odometryType_ = measurements::stringToOdometryType(typeOfOdometry, name());

  config("withDebugLogs", withDebugLogs_);

  /* configuration of the contacts manager */
  auto contactsConfig = config("contacts");

  std::string contactsDetectionString = static_cast<std::string>(contactsConfig("contactsDetection"));
  KoContactsManager::ContactsDetection contactsDetectionMethod =
      contactsManager_.stringToContactsDetection(contactsDetectionString, name());

  contactsConfig("forceSensorsAsInput", forceSensorsAsInput_);

  if(contactsDetectionMethod == KoContactsManager::ContactsDetection::Surfaces)
  {
    std::vector<std::string> surfacesForContactDetection =
        contactsConfig("surfacesForContactDetection", std::vector<std::string>());

    measurements::ContactsManagerSurfacesConfiguration contactsConf(name(), surfacesForContactDetection);
    contactSensorsIgnored_ = contactsConfig("contactSensorsIgnored", std::vector<std::string>());

    contactsConf.verbose(true);
    if(contactsConfig.has("schmittTriggerLowerPropThreshold") && contactsConfig.has("schmittTriggerUpperPropThreshold"))
    {
      double schmittTriggerLowerPropThreshold = contactsConfig("schmittTriggerLowerPropThreshold");
      double schmittTriggerUpperPropThreshold = contactsConfig("schmittTriggerUpperPropThreshold");
      contactsConf.schmittTriggerPropThresholds(schmittTriggerLowerPropThreshold, schmittTriggerUpperPropThreshold);
    }

    auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();
    auto onAddedContact = [this, &ctl, &logger](KoContactWithSensor & addedContact)
    { addContactToGui(ctl, addedContact, logger); };

    contactsManager_.init(ctl, robot_, contactsConf, onAddedContact);

    // we set the force sensor of the desired contacts as disabled
    std::vector<std::string> contactSensorsDisabledInit =
        contactsConfig("contactSensorsDisabledInit", std::vector<std::string>());
    for(auto const & contactSensorDisabledInit : contactSensorsDisabledInit)
    {
      auto * contact = contactsManager_.findContact(contactSensorDisabledInit);
      if(!contact)
      {
        mc_rtc::log::error_and_throw("The force sensor {} set as disabled on initialization does not exist.",
                                     contactSensorDisabledInit);
      }
      contact->sensorEnabled_ = false;
    }
  }
  if(contactsDetectionMethod == KoContactsManager::ContactsDetection::Sensors)
  {
    measurements::ContactsManagerSensorsConfiguration contactsConf(name());
    contactsConf.verbose(true).forceSensorsToOmit(forceSensorsAsInput_);
    if(contactsConfig.has("schmittTriggerLowerPropThreshold") && contactsConfig.has("schmittTriggerUpperPropThreshold"))
    {
      double schmittTriggerLowerPropThreshold = contactsConfig("schmittTriggerLowerPropThreshold");
      double schmittTriggerUpperPropThreshold = contactsConfig("schmittTriggerUpperPropThreshold");
      contactsConf.schmittTriggerPropThresholds(schmittTriggerLowerPropThreshold, schmittTriggerUpperPropThreshold);
    }
    contactsManager_.init(ctl, robot_, contactsConf);

    // we set the force sensor of the desired contacts as disabled
    std::vector<std::string> contactSensorsDisabledInit =
        contactsConfig("contactSensorsDisabledInit", std::vector<std::string>());
    for(const auto & contactSensorDisabledInit : contactSensorsDisabledInit)
    {
      auto * contact = contactsManager_.findContact(contactSensorDisabledInit);
      if(!contact)
      {
        mc_rtc::log::error_and_throw("The force sensor {} set as disabled on initialization does not exist.",
                                     contactSensorDisabledInit);
      }
      contact->sensorEnabled_ = false;
    }
  }
  if(contactsDetectionMethod == KoContactsManager::ContactsDetection::Solver)
  {
    measurements::ContactsManagerSolverConfiguration contactsConf(name());
    contactsConf.verbose(true);
    if(contactsConfig.has("schmittTriggerLowerPropThreshold") && contactsConfig.has("schmittTriggerUpperPropThreshold"))
    {
      double schmittTriggerLowerPropThreshold = contactsConfig("schmittTriggerLowerPropThreshold");
      double schmittTriggerUpperPropThreshold = contactsConfig("schmittTriggerUpperPropThreshold");
      contactsConf.schmittTriggerPropThresholds(schmittTriggerLowerPropThreshold, schmittTriggerUpperPropThreshold);
    }
    contactsManager_.init(ctl, robot_, contactsConf);
  }

  /* Configuration of the Kinetics Observer's parameters */

  config("withUnmodeledWrench", withUnmodeledWrench_);
  config("withGyroBias", withGyroBias_);

  observer_.setWithUnmodeledWrench(withUnmodeledWrench_);
  observer_.setWithGyroBias(withGyroBias_);
  bool useFiniteDifferences = config("withFiniteDifferences");
  if(useFiniteDifferences)
  {
    observer_.useFiniteDifferencesJacobians(useFiniteDifferences);
    so::Vector dx(observer_.getStateSize());
    dx.setConstant(static_cast<double>(config("finiteDifferenceStep")));
    observer_.setFiniteDifferenceStep(dx);
  }

  observer_.setWithAccelerationEstimation(config("withAccelerationEstimation"));

  if(config.has("withDampingInMatrixA")) { observer_.setWithDampingInMatrixA(config("withDampingInMatrixA")); }
  if(config.has("withContactStateCovRemoval"))
  {
    observer_.setWithContactStateCovRemoval(config("withContactStateCovRemoval"));
  }
  if(config.has("withAdaptativeContactProcessCov"))
  {
    observer_.setWithAdaptativeContactProcessCov(config("withAdaptativeContactProcessCov"));
  }
  config("ignoreWrenchSensors", ignoreWrenchSensors_);

  linStiffness_ = (config("linStiffness").operator so::Vector3()).matrix().asDiagonal();
  angStiffness_ = (config("angStiffness").operator so::Vector3()).matrix().asDiagonal();
  linDamping_ = (config("linDamping").operator so::Vector3()).matrix().asDiagonal();
  angDamping_ = (config("angDamping").operator so::Vector3()).matrix().asDiagonal();

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
  // if we stick to the control robot's anchor frame, we don't allow the correction of the contacts pose

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

  if(observer_.getWithAdaptativeContactProcessCov() && !ignoreWrenchSensors_)
  {
    contactProcessCovariance_.block<3, 3>(0, 0) =
        (ekfStateProcessVariances("contactPositionProcessVariance").operator so::Vector3()).matrix().asDiagonal();
    contactProcessCovariance_.block<3, 3>(3, 3) =
        (ekfStateProcessVariances("contactOrientationProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  }
  else
  {
    contactProcessCovariance_.block<3, 3>(0, 0).setZero();
    contactProcessCovariance_.block<3, 3>(3, 3).setZero();
  }

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

  // absoluteOriSensorCovariance_ = (config("absOriSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  if(ignoreWrenchSensors_)
  {
    contactSensorCovariance_.setZero();
    contactSensorCovariance_.block<3, 3>(0, 0) = (so::Vector3(1e100, 1e100, 1e100)).matrix().asDiagonal();
    contactSensorCovariance_.block<3, 3>(3, 3) = (so::Vector3(1e100, 1e100, 1e100)).matrix().asDiagonal();

    observer_.setWithUnmodeledWrench(false);
    gyroBiasProcessCovariance_.setZero();
  }

  setObserverCovariances();

  /* Configuration of the backup based on the Tilt Observer */
  // interval (in s) on which the backup will recover
  int backupInterval = config("backupInterval", 1);
  fbBackupCapacity_ = int(backupInterval / ctl.timeStep);

  koBackupFbKinematics_.set_capacity(fbBackupCapacity_);
  tiltObserver_.backupFbKinematics_.set_capacity(fbBackupCapacity_);

  invincibilityFrame_ = int(1.5 / ctl.timeStep);

  std::vector<std::string> nanBehaviourCategory;
  nanBehaviourCategory.insert(nanBehaviourCategory.end(), {"ObserverPipelines", ctl.observerPipeline().name(), name()});
  ctl.gui()->addElement({nanBehaviourCategory},
                        mc_rtc::gui::Button("SimulateNanBehaviour", [this]() { observer_.nanDetected_ = true; }));
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
  tiltObserver_.reset(ctl);

  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  const auto & realRobotModule = realRobot.module();

  rbd::MultiBodyGraph mergeMbg(realRobotModule.mbg);
  std::map<std::string, std::vector<double>> jointPosByName;
  for(int i = 0; i < realRobotModule.mb.nrJoints(); ++i)
  {
    auto jointName = realRobotModule.mb.joint(i).name();
    auto jointIndex = static_cast<unsigned long>(realRobotModule.mb.jointIndexByName(jointName));
    jointPosByName[jointName] = realRobotModule.mbc.q[jointIndex];
  }

  std::vector<std::string> rootJoints = {};
  int nbJoints = static_cast<int>(realRobot.mb().joints().size());
  for(int i = 0; i < nbJoints; ++i)
  {
    if(realRobot.mb().predecessor(i) == 0) { rootJoints.push_back(realRobot.mb().joint(i).name()); }
  }
  for(const auto & joint : rootJoints)
  {
    if(!realRobot.hasJoint(joint))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Robot does not have a joint named {}", joint);
    }
    mergeMbg.mergeSubBodies(realRobotModule.mb.body(0).name(), joint, jointPosByName);
  }

  inertiaWaist_ = mergeMbg.nodeByName(realRobotModule.mb.body(0).name())->body.inertia();
  mass(ctl.realRobot(robot_).mass());

  if(debug_) { mc_rtc::log::info("inertiaWaist = {}", inertiaWaist_); }

  /* Initialization of variables */
  X_0_fb_ = sva::PTransformd::Identity();
  v_fb_0_ = sva::MotionVecd::Zero();
  a_fb_0_ = sva::MotionVecd::Zero();
  lastBackupIter_ = 0;
  invincibilityIter_ = 0;

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  my_robots_->robotCopy(realRobot, "inputRobot");
  ctl.gui()->addElement(
      {"Robots"}, mc_rtc::gui::Robot(name(), [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));
  ctl.gui()->addElement({"Robots"},
                        mc_rtc::gui::Robot("Real", [&ctl]() -> const mc_rbdyn::Robot & { return ctl.realRobot(); }));

  X_0_fb_ = realRobot.posW().translation();

  initObserverStateVector(ctl, realRobot);
}

void MCKineticsObserver::addSensorsAsInputs(const mc_rbdyn::Robot & inputRobot,
                                            const mc_rbdyn::Robot & measRobot,
                                            so::Vector3 & inputAddtionalForce,
                                            so::Vector3 & inputAddtionalTorque)
{
  for(const std::string & fsName : forceSensorsAsInput_)
  {
    const mc_rbdyn::ForceSensor & forceSensor = measRobot.forceSensor(fsName);
    sva::ForceVecd measuredWrench = forceSensor.worldWrenchWithoutGravity(inputRobot);

    inputAddtionalForce += measuredWrench.force();
    inputAddtionalTorque += measuredWrench.moment();
  }
}

bool MCKineticsObserver::run(const mc_control::MCController & ctl)
{
  tiltObserver_.run(ctl);

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

  /** Center of mass (assumes FK, FV and FA are already done)
      Must be initialized now as used for the conversion from user to centroid frame !!! **/
  worldCoMKine_.position = inputRobot.com();
  worldCoMKine_.linVel = inputRobot.comVelocity();
  worldCoMKine_.linAcc = inputRobot.comAcceleration();

  observer_.setCenterOfMass(worldCoMKine_.position(), worldCoMKine_.linVel(), worldCoMKine_.linAcc());

  // update of the contacts
  updateContacts(ctl, logger);

  // force measurements from sensor that are not associated to a currently set contact are given to the Kinetics
  // Observer as inputs.
  inputAdditionalWrench(inputRobot, robot);

  /** Accelerometers **/
  updateIMUs(robot, inputRobot);

  observer_.setCoMAngularMomentum(
      rbd::computeCentroidalMomentum(inputRobot.mb(), inputRobot.mbc(), inputRobot.com()).moment());

  observer_.setCoMInertiaMatrix(so::Matrix3(
      inertiaWaist_.inertia() + observer_.getMass() * so::kine::skewSymmetric2(observer_.getCenterOfMass()())));

  res_ = observer_.update();

  // Kinematics of the floating base in the real world frame (our estimation goal)
  so::kine::Kinematics mcko_K_0_fb;

  if(observer_.nanDetected_) { estimationState_ = errorDetected; }
  else if(invincibilityIter_ > 0 && invincibilityIter_ < invincibilityFrame_) { estimationState_ = invincibilityFrame; }
  else { estimationState_ = noIssue; }

  // if no anomaly is detected and if we aren't in the "invicibility frame", we update the floating base with the
  // results of the Kinetics Observer
  switch(estimationState_)
  {
    case noIssue:
    {
      /* Core */
      so::kine::Kinematics fbFb; // "Zero" Kinematics
      fbFb.setZero<so::Matrix3>(so::kine::Kinematics::Flags::all);

      // Given, the Kinematics of the floating base inside its own frame (zero kinematics) which is our user
      // frame, the Kinetics Observer will return the kinematics of the floating base in the real world frame.
      mcko_K_0_fb = observer_.getGlobalKinematicsOf(fbFb);

      koBackupFbKinematics_.push_back(mcko_K_0_fb);

      X_0_fb_.rotation() = mcko_K_0_fb.orientation.toMatrix3().transpose();
      X_0_fb_.translation() = mcko_K_0_fb.position();

      /* Bring velocity of the IMU to the origin of the joint : we want the
       * velocity of joint 0, so stop one before the first joint */

      v_fb_0_.angular() = mcko_K_0_fb.angVel();
      v_fb_0_.linear() = mcko_K_0_fb.linVel();

      a_fb_0_.angular() = mcko_K_0_fb.angAcc();
      a_fb_0_.linear() = mcko_K_0_fb.linAcc();
      break;
    }
    case invincibilityFrame:
    {
      // we apply the last transformation estimated by the Tilt Observer to our previous pose to keep updating the
      // floating base with the Tilt Observer.
      mcko_K_0_fb = tiltObserver_.applyLastTransformation(koBackupFbKinematics_.back());
      koBackupFbKinematics_.push_back(mcko_K_0_fb);

      X_0_fb_.rotation() = mcko_K_0_fb.orientation.toMatrix3().transpose();
      X_0_fb_.translation() = mcko_K_0_fb.position();

      // the tilt observer doesn't estimate the acceleration so we get it by finite differences
      a_fb_0_.angular() = (mcko_K_0_fb.angVel() - v_fb_0_.angular()) / ctl.timeStep;
      a_fb_0_.linear() = (mcko_K_0_fb.linVel() - v_fb_0_.linear()) / ctl.timeStep;

      v_fb_0_.angular() = mcko_K_0_fb.angVel();
      v_fb_0_.linear() = mcko_K_0_fb.linVel();

      invincibilityIter_++;
      // While converging again after being reset, the estimation made by the Kinetics Observer is very inaccurate and
      // cannot be used. So we let it converge during the invincibility frame while using the estimation of the Tilt
      // Observer to update the real robot. Then we start over using the Kinetics Observer starting from the final
      // kinematics obtained from the Tilt Observer.
      if(invincibilityIter_ == invincibilityFrame_)
      {
        update(inputRobot);
        inputRobot.forwardKinematics();
        so::kine::Kinematics fbFb; // "Zero" Kinematics
        fbFb.setZero<so::Matrix3>(so::kine::Kinematics::Flags::all);
        so::kine::Kinematics newWorldCentroidKine;
        newWorldCentroidKine.position = inputRobot.com();
        // the orientation of the centroid frame is the one of the floating base
        newWorldCentroidKine.orientation = mcko_K_0_fb.orientation;

        newWorldCentroidKine.linVel = inputRobot.comVelocity();
        newWorldCentroidKine.angVel = mcko_K_0_fb.angVel();

        observer_.setWorldCentroidStateKinematics(newWorldCentroidKine, false);

        for(auto & [_, contact] : contactsManager_.contacts())
        {
          if(!contact.isSet()) { continue; }

          // Update of the force measurements (the contribution of the gravity changed)
          const mc_rbdyn::ForceSensor & forceSensor = robot.forceSensor(contact.forceSensor());

          // the tilt of the robot changed so the contribution of the gravity to the measurements changed too
          if(contactsManager_.getContactsDetection() == KoContactsManager::ContactsDetection::Sensors)
          {
            updateContactForceMeasurement(contact, forceSensor.wrenchWithoutGravity(inputRobot));
          }
          else // the kinematics of the contact are the ones of the associated surface
          {
            updateContactForceMeasurement(contact, forceSensor.wrenchWithoutGravity(inputRobot),
                                          &contact.contactSensorKine_);
          }

          so::kine::Kinematics newWorldContactKineRef;

          if(odometryType_
             != measurements::OdometryType::None) // the Kinetics Observer performs odometry. The estimated
                                                  // state is used to provide the new contacts references.
          {
            getOdometryWorldContactRest(ctl, contact, newWorldContactKineRef);
          }
          else // we don't perform odometry, the reference pose of the contact is its pose in the control robot
          {
            newWorldContactKineRef = getContactWorldKinematics(contact, robot, forceSensor);
          }

          observer_.setStateContact(contact.id(), newWorldContactKineRef, contact.contactWrenchVector_, false);
        }
      }

      break;
    }
    case errorDetected:
    {
      // an error was just detected, we reset the state vector and covariances and start the invicibility frame, during
      // which we let the Kinetics Observer converge before using it again.
      auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();
      if(logger.t() / ctl.timeStep < fbBackupCapacity_)
      {
        mc_rtc::log::warning("The backup function was called before the required time was ellapsed. The backup will be "
                             "performed using the last {} seconds",
                             logger.t());
      }

      if(logger.t() / ctl.timeStep - lastBackupIter_ < fbBackupCapacity_)
      {
        mc_rtc::log::warning("The backup function was called again too quickly. The backup will be "
                             "performed using the last {} seconds",
                             logger.t() - lastBackupIter_ * ctl.timeStep);
      }

      // We add an empty Kinematics object to the floating base pose buffer. This is because the buffer of the tilt
      // observer already contains the last estimation of the floating base so we prevent a disalignment of the two
      // buffers. This empty Kinematics is filled and returned by the "runBackup" function.
      koBackupFbKinematics_.push_back(so::kine::Kinematics::zeroKinematics(so::kine::Kinematics::Flags::pose));

      mcko_K_0_fb = tiltObserver_.backupFb(&koBackupFbKinematics_);

      X_0_fb_.rotation() = mcko_K_0_fb.orientation.toMatrix3().transpose();
      X_0_fb_.translation() = mcko_K_0_fb.position();

      // the tilt observer doesn't estimate the acceleration so we get it by finite differences
      a_fb_0_.angular() = (mcko_K_0_fb.angVel() - v_fb_0_.angular()) / ctl.timeStep;
      a_fb_0_.linear() = (mcko_K_0_fb.linVel() - v_fb_0_.linear()) / ctl.timeStep;

      v_fb_0_.angular() = mcko_K_0_fb.angVel();
      v_fb_0_.linear() = mcko_K_0_fb.linVel();

      // we update update robot as it will be updated at the beginning of the next iteration anyway
      update(inputRobot);
      inputRobot.forwardKinematics();
      so::kine::Kinematics newWorldCentroidKine;
      newWorldCentroidKine.position = inputRobot.com();
      newWorldCentroidKine.linVel = inputRobot.comVelocity();
      // the orientation of the centroid frame is the one of the floating base
      newWorldCentroidKine.orientation = mcko_K_0_fb.orientation;
      newWorldCentroidKine.angVel = mcko_K_0_fb.angVel();

      observer_.setWorldCentroidStateKinematics(newWorldCentroidKine, true);
      observer_.setStateUnmodeledWrench(so::Vector6::Zero(), true);

      for(size_t i = 0; i < listIMUs_.size(); ++i)
      {
        const auto & imu = listIMUs_[i];

        observer_.setGyroBias(imu.gyroBias, static_cast<unsigned int>(i), true);
      }

      for(auto & [_, contact] : contactsManager_.contacts())
      {
        if(!contact.isSet()) { continue; }

        // Update of the force measurements (the offset due to the gravity changed)
        const mc_rbdyn::ForceSensor & forceSensor = inputRobot.forceSensor(contact.forceSensor());

        if(contactsManager_.getContactsDetection() == KoContactsManager::ContactsDetection::Sensors)
        {
          updateContactForceMeasurement(contact, forceSensor.wrenchWithoutGravity(inputRobot));
        }
        else // the kinematics of the contact are the ones of the associated surface
        {
          updateContactForceMeasurement(contact, forceSensor.wrenchWithoutGravity(inputRobot),
                                        &contact.contactSensorKine_);
        }

        so::kine::Kinematics newWorldContactKineRef;

        if(odometryType_ != measurements::OdometryType::None) // the Kinetics Observer performs odometry. The estimated
                                                              // state is used to provide the new contacts references.
        {
          getOdometryWorldContactRest(ctl, contact, newWorldContactKineRef);
        }
        else // we don't perform odometry, the reference pose of the contact is its pose in the control robot
        {
          newWorldContactKineRef = getContactWorldKinematics(contact, robot, forceSensor);
        }

        observer_.setStateContact(contact.id(), newWorldContactKineRef, contact.contactWrenchVector_, true);
      }

      // this variable indicates that we entered the invincibility frame
      invincibilityIter_ = 1;
      lastBackupIter_ = int(logger.t() / ctl.timeStep);

      observer_.nanDetected_ = false;

      break;
    }
  }

  if(withDebugLogs_)
  {
    /* Update of the logged variables */
    correctedMeasurements_ = observer_.getEKF().getSimulatedMeasurement(observer_.getEKF().getCurrentTime());
    for(auto & contact : maintainedContacts_)
    {
      contact->viscoElasticWrenchAfterCorrection_ = observer_.getCurrentViscoElasticWrench(contact->id());
    }

    globalCentroidKinematics_ = observer_.getGlobalCentroidKinematics();

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
  my_robots_->robot().mbc().q = ctl.realRobot().mbc().q;

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

  observer_.setInitWorldCentroidStateVector(initStateVector);
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

  for(auto & contactWithSensor : contactsManager_.contacts())
  {
    const KoContactWithSensor & contact = contactWithSensor.second;
    const std::string & fsName = contact.forceSensor();

    if(!contact.isSet() && contact.sensorEnabled_
       && std::find(contactSensorsIgnored_.begin(), contactSensorsIgnored_.end(), contact.name())
              == contactSensorsIgnored_.end()) // if the contact is not set but we use the force sensor measurements,
                                               // then we give the measured force as an input to the Kinetics Observer
    {
      sva::ForceVecd measuredWrench = measRobot.forceSensor(fsName).worldWrenchWithoutGravity(inputRobot);
      additionalUserResultingForce_ += measuredWrench.force();
      additionalUserResultingMoment_ += measuredWrench.moment();
    }
  }
  // we add the wrench measured by the sensors that are not associated to contacts
  for(auto & forceSensor : measRobot.forceSensors())
  {
    if(!contactsManager_.contacts().count(forceSensor.name()))
    {
      sva::ForceVecd measuredWrench = forceSensor.worldWrenchWithoutGravity(inputRobot);
      additionalUserResultingForce_ += measuredWrench.force();
      additionalUserResultingMoment_ += measuredWrench.moment();
    }
  }

  addSensorsAsInputs(inputRobot, measRobot, additionalUserResultingForce_, additionalUserResultingMoment_);

  // We pass this computed wrench as an input to the Kinetics Observer
  observer_.setAdditionalWrench(additionalUserResultingForce_, additionalUserResultingMoment_);

  if(withDebugLogs_)
  {
    for(auto & contactWithSensor : contactsManager_.contacts())
    {
      KoContactWithSensor & contact = contactWithSensor.second;
      const std::string & fsName = contact.forceSensor();
      so::Vector3 forceCentroid = so::Vector3::Zero();
      so::Vector3 torqueCentroid = so::Vector3::Zero();
      observer_.convertWrenchFromUserToCentroid(
          measRobot.forceSensor(fsName).worldWrenchWithoutGravity(inputRobot).force(),
          measRobot.forceSensor(fsName).worldWrenchWithoutGravity(inputRobot).moment(), forceCentroid, torqueCentroid);

      contact.wrenchInCentroid_.segment<3>(0) = forceCentroid;
      contact.wrenchInCentroid_.segment<3>(3) = torqueCentroid;
    }
  }
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

    so::kine::Kinematics worldBodyKine = conversions::kinematics::fromSva(
        inputRobot.mbc().bodyPosW[inputRobot.bodyIndexByName(imu.parentBody())],
        inputRobot.mbc().bodyVelW[inputRobot.bodyIndexByName(imu.parentBody())],
        inputRobot.mbc().bodyAccB[inputRobot.bodyIndexByName(imu.parentBody())], true, false);

    so::kine::Kinematics worldImuKine = worldBodyKine * bodyImuKine;
    listIMUs_.at(i).fbImuKine = worldImuKine;

    observer_.setIMU(imu.linearAcceleration(), imu.angularVelocity(), acceleroSensorCovariance_, gyroSensorCovariance_,
                     worldImuKine, i);
  }
}

const so::kine::Kinematics MCKineticsObserver::getContactWorldKinematics(const KoContactWithSensor & contact,
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

  if(contactsManager_.getContactsDetection() == KoContactsManager::ContactsDetection::Sensors)
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
    sva::PTransformd worldContactPose = currentRobot.surfacePose(contact.surface());
    // Kinematics of the surface in the world / floating base's frame
    worldContactKine = conversions::kinematics::fromSva(worldContactPose, so::kine::Kinematics::Flags::vel);

    // the kinematics of the contacts are the ones of the surface, but we must transport the measured wrench
    const mc_rbdyn::Surface & contactSurface = currentRobot.surface(contact.surface());

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

void MCKineticsObserver::updateContactForceMeasurement(KoContactWithSensor & contact,
                                                       const sva::ForceVecd & measuredWrench,
                                                       const stateObservation::kine::Kinematics * contactSensorKine)
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

void MCKineticsObserver::getOdometryWorldContactRest(const mc_control::MCController & ctl,
                                                     KoContactWithSensor & contact,
                                                     so::kine::Kinematics & worldContactKineRef)
{
  const auto & robot = ctl.robot(robot_);
  if(!contact.sensorEnabled_)
  {
    mc_rtc::log::info("The sensor is disabled but is required for the odometry. It will be used for the odometry "
                      "but not in the correction made by the Kinetics Observer.");
  }
  const so::Vector3 & contactForceMeas = contact.contactWrenchVector_.segment<3>(0); // retrieving the force measurement
  const so::Vector3 & contactTorqueMeas =
      contact.contactWrenchVector_.segment<3>(3); // retrieving the torque measurement

  // we get the kinematics of the contact in the real world from the ones of the centroid estimated by the Kinetics
  // Observer. These kinematics are not the reference kinematics of the contact as they take into account the
  // visco-elastic model of the contacts.
  const so::kine::Kinematics worldContactKine = observer_.getGlobalKinematicsOf(contact.fbContactKine_);

  // we get the reference position of the contact by removing the contribution of the visco-elastic model
  worldContactKineRef.position =
      worldContactKine.orientation.toMatrix3() * linStiffness_.inverse()
          * (contactForceMeas
             + worldContactKine.orientation.toMatrix3().transpose() * linDamping_ * worldContactKine.linVel())
      + worldContactKine.position();

  /* We get the reference orientation of the contact by removing the contribution of the visco-elastic model */
  // difference between the reference orientation and the real one, obtained from the visco-elastic model
  so::Vector3 flexRotDiff =
      -2 * worldContactKine.orientation.toMatrix3() * angStiffness_.inverse()
      * (contactTorqueMeas
         + worldContactKine.orientation.toMatrix3().transpose() * angDamping_ * worldContactKine.angVel());

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
  worldContactKineRef.orientation = so::Matrix3(flexRotMatrix.transpose() * worldContactKine.orientation.toMatrix3());

  if(odometryType_
     == measurements::OdometryType::Flat) // if true, the position odometry is made only along the x and y axis,
                                          // the position along z is assumed to be the one of the control robot
  {
    // kinematics of the contact of the control robot in the world frame
    so::kine::Kinematics worldContactKineControl =
        getContactWorldKinematics(contact, robot, robot.forceSensor(contact.forceSensor()));

    // the reference altitude of the contact is the one in the control robot
    worldContactKineRef.position()(2) = 0.0;
  }
}

// Function to generate a random rotation matrix
Eigen::Matrix3d generateRandomRotationMatrix(double totalRotationDegrees)
{
  Eigen::Vector3d rotationAxis = Eigen::Vector3d::Random();
  rotationAxis.normalize();
  Eigen::AngleAxisd aa(totalRotationDegrees / 180 * mc_rtc::constants::PI, rotationAxis);
  return aa.toRotationMatrix();
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

  sva::ForceVecd measuredWrench = robot.forceSensor(contact.forceSensor()).wrenchWithoutGravity(inputRobot);
  const mc_rbdyn::ForceSensor & forceSensor = robot.forceSensor(contact.forceSensor());

  // As used on input robot, returns the kinematics of the contact in the frame of the floating base. Also expresses the
  // measured wrench in the frame of the contact.
  contact.fbContactKine_ = getContactWorldKinematics(contact, inputRobot, forceSensor, &measuredWrench);

  // reference of the contact in the world / floating base of the input robot
  so::kine::Kinematics worldContactKineRef;

  if(odometryType_ != measurements::OdometryType::None) // the Kinetics Observer performs odometry. The estimated
                                                        // state is used to provide the new contacts references.
  {
    getOdometryWorldContactRest(ctl, contact, worldContactKineRef);
  }
  else // we don't perform odometry, the reference pose of the contact is its pose in the control robot
  {
    worldContactKineRef = getContactWorldKinematics(contact, robot, forceSensor);
  }

  // if(logger.t() > 1e-15)
  // {
  //   std::cout << std::endl << "before: " << worldContactKineRef.orientation.toRollPitchYaw() * 180 / M_PI <<
  //   std::endl; worldContactKineRef.orientation =
  //       so::Matrix3(generateRandomRotationMatrix(30) * worldContactKineRef.orientation.toMatrix3());
  //   std::cout << std::endl << "after: " << worldContactKineRef.orientation.toRollPitchYaw() * 180 / M_PI <<
  //   std::endl;
  // }
  // to remove
  contact.initKine_ = worldContactKineRef;

  observer_.addContact(worldContactKineRef, initCovariance, contactProcessCovariance_, contact.id(), linStiffness_,
                       linDamping_, angStiffness_, angDamping_);

  // checks if the sensor is used in the correction of the Kinetics Observer or not
  if(contact.sensorEnabled_)
  {
    // we update the measurements of the sensor and the input kinematics of the contact in the user /
    // floating base's frame
    observer_.updateContactWithWrenchSensor(contact.contactWrenchVector_, contactSensorCovariance_,
                                            contact.fbContactKine_, contact.id());
  }
  else
  {
    // we update the input kinematics of the contact in the user / floating base's frame
    observer_.updateContactWithNoSensor(contact.fbContactKine_, contact.id());
  }

  if(withDebugLogs_)
  {
    addContactLogEntries(ctl, logger, contact);
    if(contact.sensorEnabled_) { addContactMeasurementsLogEntries(logger, contact); }
  }
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

  const auto & robot = ctl.robot(robot_);

  sva::ForceVecd measuredWrench = robot.forceSensor(contact.forceSensor()).wrenchWithoutGravity(inputRobot);
  const mc_rbdyn::ForceSensor & forceSensor = robot.forceSensor(contact.forceSensor());

  // As used on input robot, returns the kinematics of the contact in the frame of the floating base. Also expresses the
  // measured wrench in the frame of the contact.
  contact.fbContactKine_ = getContactWorldKinematics(contact, inputRobot, forceSensor, &measuredWrench);

  if(contact.sensorEnabled_
     && std::find(contactSensorsIgnored_.begin(), contactSensorsIgnored_.end(), contact.name())
            == contactSensorsIgnored_.end()) // the force sensor attached to the contact is used in the correction by
                                             // the Kinetics Observer.
  {
    observer_.updateContactWithWrenchSensor(contact.contactWrenchVector_, contactSensorCovariance_,
                                            contact.fbContactKine_, contact.id());
  }
  else { observer_.updateContactWithNoSensor(contact.fbContactKine_, contact.id()); }
}

void MCKineticsObserver::updateContacts(const mc_control::MCController & ctl, mc_rtc::Logger & logger)
{
  maintainedContacts_.clear();
  for(auto & cont : contactSensorsIgnored_) { std::cout << std::endl << "ignored: " << cont << std::endl; }

  const so::Matrix12 * initCovariance;

  if(observer_.getNumberOfSetContacts() > 0) // The initial covariance on the pose of the contact depending on
                                             // whether another contact is already set or not
  {
    if(odometryType_ == measurements::OdometryType::Flat)
    {
      // we compute again the following matrix as contactInitCovarianceNewContacts_ can be updated.
      contactInitCovarianceNewContacts_flat_.diagonal() = contactInitCovarianceNewContacts_.diagonal();
      contactInitCovarianceNewContacts_flat_(2, 2) = 0.0;

      initCovariance = &contactInitCovarianceNewContacts_flat_;
    }
    else { initCovariance = &contactInitCovarianceNewContacts_; }
  }
  else
  {
    if(odometryType_ == measurements::OdometryType::Flat)
    {
      contactInitCovarianceFirstContacts_flat_.diagonal() = contactInitCovarianceFirstContacts_.diagonal();
      contactInitCovarianceFirstContacts_flat_(2, 2) = 0.0;

      initCovariance = &contactInitCovarianceFirstContacts_flat_;
    }
    else { initCovariance = &contactInitCovarianceFirstContacts_; }
  }

  auto onNewContact = [this, &ctl, &logger, &initCovariance](KoContactWithSensor & newContact)
  {
    if(std::find(contactSensorsIgnored_.begin(), contactSensorsIgnored_.end(), newContact.name())
       == contactSensorsIgnored_.end())
    {
      setNewContact(ctl, newContact, *initCovariance, logger);
    }
  };
  auto onMaintainedContact = [this, &ctl](KoContactWithSensor & maintainedContact)
  {
    if(std::find(contactSensorsIgnored_.begin(), contactSensorsIgnored_.end(), maintainedContact.name())
       == contactSensorsIgnored_.end())
    {
      updateContact(ctl, maintainedContact);
      maintainedContacts_.push_back(&maintainedContact);
    }
  };
  auto onRemovedContact = [this, &logger](KoContactWithSensor & removedContact)
  {
    if(std::find(contactSensorsIgnored_.begin(), contactSensorsIgnored_.end(), removedContact.name())
       == contactSensorsIgnored_.end())
    {
      observer_.removeContact(removedContact.id());

      if(withDebugLogs_)
      {
        removeContactLogEntries(logger, removedContact);
        removeContactMeasurementsLogEntries(logger, removedContact);
      }
    }
  };

  // action to execute when a contact is added to the manager during the run, which happens when the contact detection
  // is using the solver.
  auto onAddedContact = [this, &ctl, &logger](KoContactWithSensor & addedContact)
  { addContactToGui(ctl, addedContact, logger); };

  contactsManager_.updateContacts(ctl, robot_, onNewContact, onMaintainedContact, onRemovedContact, onAddedContact);
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
  tiltObserver_.addToLogger(ctl, logger, category + "_" + tiltObserver_.name());
  logger.addLogEntry(category_ + "_mcko_fb_posW", [this]() -> sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category_ + "_mcko_fb_velW", [this]() -> sva::MotionVecd & { return v_fb_0_; });
  logger.addLogEntry(category_ + "_mcko_fb_accW", [this]() -> sva::MotionVecd & { return a_fb_0_; });

  logger.addLogEntry(category_ + "_mcko_fb_yaw",
                     [this]() -> double { return -so::kine::rotationMatrixToYawAxisAgnostic(X_0_fb_.rotation()); });

  logger.addLogEntry(category_ + "_constants_mass", [this]() -> double { return observer_.getMass(); });

  logger.addLogEntry(category_ + "_debug_estimationState",
                     [this]() -> std::string
                     {
                       switch(estimationState_)
                       {
                         case noIssue:
                           return "noIssue";
                           break;
                         case invincibilityFrame:
                           return "invincibilityFrame";
                           break;
                         case errorDetected:
                           return "errorDetected";
                           break;
                       }
                       return "default";
                     });
  logger.addLogEntry(category_ + "_debug_config_OdometryType",
                     [this]() -> std::string { return measurements::odometryTypeToSstring(odometryType_); });
  logger.addLogEntry(category_ + "_debug_config_withContactStateCovRemoval",
                     [this]() -> std::string { return observer_.getWithContactStateCovRemoval() ? "True" : "False"; });

  logger.addLogEntry(category_ + "_debug_config_withAdaptativeContactProcessCov",
                     [this]() -> std::string
                     { return observer_.getWithAdaptativeContactProcessCov() ? "True" : "False"; });

  /* Plots of the updated state */
  conversions::kinematics::addToLogger(logger, globalCentroidKinematics_, category_ + "_MEKF_estimatedState");
  for(auto & imu : listIMUs_)
  {
    logger.addLogEntry(category_ + "_MEKF_estimatedState_gyroBias_" + imu.name(),
                       [this, &imu]() -> Eigen::Vector3d {
                         return observer_.getCurrentStateVector().segment(observer_.gyroBiasIndex(imu.id()),
                                                                          observer_.sizeGyroBias);
                       });
  }
  logger.addLogEntry(
      category_ + "_MEKF_estimatedState_extForceCentr",
      [this]() -> Eigen::Vector3d
      { return observer_.getCurrentStateVector().segment(observer_.unmodeledForceIndex(), observer_.sizeForce); });

  logger.addLogEntry(
      category_ + "_MEKF_estimatedState_extTorqueCentr",
      [this]() -> Eigen::Vector3d
      { return observer_.getCurrentStateVector().segment(observer_.unmodeledTorqueIndex(), observer_.sizeTorque); });
  if(withDebugLogs_)
  {
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
      logger.addLogEntry(
          category_ + "_MEKF_measurements_predError_vector",
          [this]() -> Eigen::VectorXd
          { return (observer_.getEKF().getLastMeasurement() - observer_.getEKF().getLastPredictedMeasurement()); });
      logger.addLogEntry(
          category_ + "_MEKF_measurements_predError_norm",
          [this]() -> double {
            return (observer_.getEKF().getLastMeasurement() - observer_.getEKF().getLastPredictedMeasurement()).norm();
          });
      logger.addLogEntry(category_ + "_MEKF_measurements_gyro_" + imu.name() + "_measured",
                         [this, &imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastMeasurement().segment(
                               observer_.getIMUMeasIndexByNum(imu.id()) + observer_.sizeAcceleroSignal,
                               observer_.sizeGyroBias);
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

      logger.addLogEntry(category_ + "_MEKF_measurements_accelerometer_" + imu.name() + "_measured",
                         [this, &imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastMeasurement().segment(
                               observer_.getIMUMeasIndexByNum(imu.id()), observer_.sizeAcceleroSignal);
                         });
      logger.addLogEntry(category_ + "_MEKF_measurements_accelerometer_" + imu.name() + "_predicted",
                         [this, &imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastPredictedMeasurement().segment(
                               observer_.getIMUMeasIndexByNum(imu.id()), observer_.sizeAcceleroSignal);
                         });
      logger.addLogEntry(category_ + "_MEKF_measurements_accelerometer_" + imu.name() + "_corrected",
                         [this, &imu]() -> Eigen::Vector3d {
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

      conversions::kinematics::addToLogger(logger, imu.fbImuKine, category_ + "_MEKF_inputs_fbImuKine_" + imu.name());
    }

    /* Inputs */
    logger.addLogEntry(category_ + "_MEKF_inputs_additionalWrench_Force",
                       [this]() -> Eigen::Vector3d
                       { return observer_.getAdditionalWrench().segment(0, observer_.sizeForce); });
    logger.addLogEntry(category_ + "_MEKF_inputs_additionalWrench_Torque",
                       [this]() -> Eigen::Vector3d
                       { return observer_.getAdditionalWrench().segment(observer_.sizeForce, observer_.sizeTorque); });

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
                             .block(observer_.posIndexTangent(), observer_.posIndexTangent(), observer_.sizePosTangent,
                                    observer_.sizePosTangent)
                             .diagonal();
                       });
    logger.addLogEntry(category_ + "_MEKF_stateCovariances_orientationW_",
                       [this]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF()
                             .getStateCovariance()
                             .block(observer_.oriIndexTangent(), observer_.oriIndexTangent(), observer_.sizeOriTangent,
                                    observer_.sizeOriTangent)
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
    logger.addLogEntry(
        category_ + "_MEKF_innovation_positionW_",
        [this]() -> Eigen::Vector3d
        { return observer_.getEKF().getInnovation().segment(observer_.posIndexTangent(), observer_.sizePosTangent); });
    logger.addLogEntry(category_ + "_MEKF_innovation_linVelW_",
                       [this]() -> Eigen::Vector3d {
                         return observer_.getEKF().getInnovation().segment(observer_.linVelIndexTangent(),
                                                                           observer_.sizeLinVelTangent);
                       });
    logger.addLogEntry(
        category_ + "_MEKF_innovation_oriW_",
        [this]() -> Eigen::Vector3d
        { return observer_.getEKF().getInnovation().segment(observer_.oriIndexTangent(), observer_.sizeOriTangent); });
    logger.addLogEntry(category_ + "_MEKF_innovation_angVelW_",
                       [this]() -> Eigen::Vector3d {
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

    logger.addLogEntry(category_ + "_MEKF_prediction_locPos",
                       [this]() -> Eigen::Vector3d {
                         return observer_.getEKF().getLastPrediction().segment(observer_.posIndex(), observer_.sizePos);
                       });
    logger.addLogEntry(
        category_ + "_MEKF_prediction_locLinVel",
        [this]() -> Eigen::Vector3d
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
                       [this]() -> Eigen::Vector3d {
                         return observer_.getEKF().getLastPrediction().segment(observer_.angVelIndex(),
                                                                               observer_.sizeAngVelTangent);
                       });
    logger.addLogEntry(category_ + "_MEKF_prediction_unmodeledForce",
                       [this]() -> Eigen::Vector3d {
                         return observer_.getEKF().getLastPrediction().segment(observer_.unmodeledForceIndex(),
                                                                               observer_.sizeForce);
                       });
    logger.addLogEntry(category_ + "_MEKF_prediction_unmodeledTorque",
                       [this]() -> Eigen::Vector3d {
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

    for(auto & contactWithSensor : contactsManager_.contacts())
    {
      const KoContactWithSensor & contact = contactWithSensor.second;
      logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_" + contact.name() + "_force",
                         [&contact]() -> Eigen::Vector3d { return contact.wrenchInCentroid_.segment<3>(0); });
      logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_" + contact.name() + "_torque",
                         [&contact]() -> Eigen::Vector3d { return contact.wrenchInCentroid_.segment<3>(3); });
      logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_" + contact.name() + "_forceWithUnmodeled",
                         [this, &contact]() -> Eigen::Vector3d
                         {
                           return observer_.getCurrentStateVector().segment(observer_.unmodeledForceIndex(),
                                                                            observer_.sizeForce)
                                  + contact.wrenchInCentroid_.segment<3>(0);
                         });
      logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_" + contact.name() + "_torqueWithUnmodeled",
                         [this, &contact]() -> Eigen::Vector3d
                         {
                           return observer_.getCurrentStateVector().segment(observer_.unmodeledTorqueIndex(),
                                                                            observer_.sizeTorque)
                                  + contact.wrenchInCentroid_.segment<3>(3);
                         });
    }
  }
}

void MCKineticsObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category_ + "_posW");
  logger.removeLogEntry(category_ + "_velW");
  logger.removeLogEntry(category_ + "_mass");

  logger.removeLogEntry(category_ + "_flexStiffness");
  logger.removeLogEntry(category_ + "_flexDamping");
}

void MCKineticsObserver::setOdometryType(const std::string & newOdometryType)
{
  prevOdometryType_ = odometryType_;
  odometryType_ = measurements::stringToOdometryType(newOdometryType, name());

  // if the type didn't change, we stop the function here
  if(odometryType_ == prevOdometryType_) { return; }

  mc_rtc::log::info("[{}]: Odometry mode changed to: {}", name(), newOdometryType);
  tiltObserver_.setOdometryType(odometryType_);
}

void MCKineticsObserver::addToGUI(const mc_control::MCController &,
                                  mc_rtc::gui::StateBuilder & gui,
                                  const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;
  // clang-format off
  std::vector<std::string> covsCategory = category;
  covsCategory.insert(covsCategory.end(), {"Covariances"});

  std::vector<std::string> initCovsCategory = covsCategory;
  initCovsCategory.insert(initCovsCategory.end(), {"Init"});
  std::vector<std::string> processCovsCategory = covsCategory;
  processCovsCategory.insert(processCovsCategory.end(), {"Process"});
  std::vector<std::string> sensorCovsCategory = covsCategory;
  sensorCovsCategory.insert(sensorCovsCategory.end(), {"Sensors"});

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
  
  if(odometryType_ != measurements::OdometryType::None)
  {
    std::vector<std::string> odomCategory = category;
    odomCategory.insert(odomCategory.end(), {"Odometry"});
    gui.addElement({odomCategory}, mc_rtc::gui::ComboInput(
                                                                  "Choose from list",  {measurements::odometryTypeToSstring(measurements::OdometryType::Odometry6d), measurements::odometryTypeToSstring(measurements::OdometryType::Flat)},
                                                                  [this]() -> std::string {
                                                                    return measurements::odometryTypeToSstring(odometryType_);
                                                                  },
                                                                  [this](const std::string & typeOfOdometry) {
                                                                    setOdometryType(typeOfOdometry);
                                                                  }));
  }
  // clang-format on
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
                            contact.name() + " : " + (contact.isSet() ? "Contact is set" : "Contact is not set")
                                + ": Use wrench sensor: ",
                            [&contact]() { return contact.sensorEnabled_; },
                            [this, &contact, &logger]()
                            {
                              if(!contact.sensorEnabled_)
                              {
                                contact.sensorEnabled_ = true;
                                mc_rtc::log::info("{}: contact's sensors enabled", contact.name());
                                if(contact.isSet()) { addContactMeasurementsLogEntries(logger, contact); }
                              }
                              else
                              {
                                contact.sensorEnabled_ = false;
                                mc_rtc::log::info("{}: contact's sensors disabled", contact.name());
                                if(contact.isSet()) { removeContactMeasurementsLogEntries(logger, contact); }
                              }
                            }));
}

void MCKineticsObserver::addContactLogEntries(const mc_control::MCController & ctl,
                                              mc_rtc::Logger & logger,
                                              const KoContactWithSensor & contact)
{
  logger.addLogEntry(category_ + "_MEKF_estimatedState_contact_" + contact.name() + "_position", &contact,
                     [this, &contact]() -> Eigen::Vector3d {
                       return observer_.getCurrentStateVector().segment(observer_.contactPosIndex(contact.id()),
                                                                        observer_.sizePos);
                     });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_contact_" + contact.name() + "_orientation", &contact,
                     [this, &contact]() -> Eigen::Quaternion<double>
                     {
                       so::kine::Orientation ori;
                       return ori
                           .fromVector4(observer_.getCurrentStateVector().segment(
                               observer_.contactOriIndex(contact.id()), observer_.sizeOri))
                           .inverse()
                           .toQuaternion();
                     });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_contact_" + contact.name() + "_orientation_RollPitchYaw",
                     &contact,
                     [this, &contact]() -> so::Vector3
                     {
                       so::kine::Orientation ori;
                       return so::kine::rotationMatrixToRollPitchYaw(
                           ori.fromVector4(observer_.getCurrentStateVector().segment(
                                               observer_.contactOriIndex(contact.id()), observer_.sizeOri))
                               .toMatrix3());
                     });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_contact_" + contact.name() + "_forces", &contact,
                     [this, &contact]() -> Eigen::Vector3d {
                       return observer_.getCurrentStateVector().segment(observer_.contactForceIndex(contact.id()),
                                                                        observer_.sizeForce);
                     });
  logger.addLogEntry(category_ + "_MEKF_estimatedState_contact_" + contact.name() + "_torques", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return globalCentroidKinematics_.orientation.toMatrix3()
                              * observer_.getCurrentStateVector().segment(observer_.contactTorqueIndex(contact.id()),
                                                                          observer_.sizeTorque);
                     });
  logger.addLogEntry(category_ + "_MEKF_stateCovariances_contact_" + contact.name() + "_position_", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block(observer_.contactPosIndexTangent(contact.id()),
                                  observer_.contactPosIndexTangent(contact.id()), observer_.sizePosTangent,
                                  observer_.sizePosTangent)
                           .diagonal();
                     });
  logger.addLogEntry(category_ + "_MEKF_stateCovariances_contact_" + contact.name() + "_orientation_", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block(observer_.contactOriIndexTangent(contact.id()),
                                  observer_.contactOriIndexTangent(contact.id()), observer_.sizeOriTangent,
                                  observer_.sizeOriTangent)
                           .diagonal();
                     });
  logger.addLogEntry(category_ + "_MEKF_stateCovariances_contact_" + contact.name() + "_Force_", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block(observer_.contactForceIndexTangent(contact.id()),
                                  observer_.contactForceIndexTangent(contact.id()), observer_.sizeForceTangent,
                                  observer_.sizeForceTangent)
                           .diagonal();
                     });
  logger.addLogEntry(category_ + "_MEKF_stateCovariances_contact_" + contact.name() + "_Torque_", &contact,
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
      category_ + "_MEKF_prediction_contact_" + contact.name() + "_poseWorldFromCentroid_pos", &contact,
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

  logger.addLogEntry(category_ + "_MEKF_prediction_contact_" + contact.name() + "_poseWorldFromCentroid_ori", &contact,
                     [this, &contact]() -> Eigen::Quaterniond
                     {
                       so::kine::Orientation predictedWorldCentroidLocKine;
                       predictedWorldCentroidLocKine.fromVector4(
                           observer_.getEKF().getLastPrediction().segment(observer_.oriIndex(), observer_.sizeOri));

                       so::kine::Orientation predictedWorldContactOri(so::Matrix3(
                           predictedWorldCentroidLocKine.toMatrix3() * contact.fbContactKine_.orientation.toMatrix3()));

                       return predictedWorldContactOri.inverse().toQuaternion();
                     });

  logger.addLogEntry(category_ + "_MEKF_prediction_contact_" + contact.name() + "_poseWorldFromCentroid_linVel",
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

  logger.addLogEntry(category_ + "_MEKF_prediction_contact_" + contact.name() + "_poseWorldFromCentroid_angVel",
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

  logger.addLogEntry(category_ + "_MEKF_prediction_contact_" + contact.name() + "_restPos_W", &contact,
                     [this, &contact]() -> Eigen::Vector3d {
                       return observer_.getEKF().getLastPrediction().segment(observer_.contactPosIndex(contact.id()),
                                                                             observer_.sizePos);
                     });
  logger.addLogEntry(category_ + "_MEKF_prediction_contact_" + contact.name() + "_restOri_W", &contact,
                     [this, &contact]() -> Eigen::Quaternion<double>
                     {
                       so::kine::Orientation ori;
                       return ori
                           .fromVector4(observer_.getEKF().getLastPrediction().segment(
                               observer_.contactOriIndex(contact.id()), observer_.sizeOri))
                           .inverse()
                           .toQuaternion();
                     });
  logger.addLogEntry(category_ + "_MEKF_prediction_contact_" + contact.name() + "_forces", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getLastPrediction().segment(observer_.contactForceIndex(contact.id()),
                                                                             observer_.sizeForce);
                     });
  logger.addLogEntry(category_ + "_MEKF_prediction_contact_" + contact.name() + "_torques", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getLastPrediction().segment(observer_.contactTorqueIndex(contact.id()),
                                                                             observer_.sizeTorque);
                     });

  logger.addLogEntry(category_ + "_MEKF_debug_contactWrench_Centroid_" + contact.name() + "_force", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getCentroidContactWrench(contact.id()).segment(0, observer_.sizeForce); });

  logger.addLogEntry(category_ + "_MEKF_debug_contactWrench_Centroid_" + contact.name() + "_torque", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getCentroidContactWrench(contact.id()).segment(3, observer_.sizeTorque); });

  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.name() + "_inputCentroidContactKine_position", &contact,
      [this, &contact]() -> Eigen::Vector3d { return observer_.getCentroidContactInputKine(contact.id()).position(); });

  conversions::kinematics::addToLogger(logger, contact.initKine_,
                                       category_ + "_debug_contactKine_" + contact.name() + "_initKine");

  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.name() + "_inputCentroidContactKine_orientation",
                     &contact,
                     [this, &contact]() -> Eigen::Quaternion<double> {
                       return observer_.getCentroidContactInputKine(contact.id()).orientation.inverse().toQuaternion();
                     });
  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.name() + "_inputCentroidContactKine_linVel", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getCentroidContactInputKine(contact.id()).linVel(); });

  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.name() + "_inputCentroidContactKine_angVel", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getCentroidContactInputKine(contact.id()).angVel(); });
  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.name() + "_realRobot_position", &contact,
      [this, &contact, &ctl]() -> Eigen::Vector3d
      {
        const auto & robot = ctl.robot(robot_);
        const auto & realRobot = ctl.realRobot(robot_);
        return getContactWorldKinematics(contact, realRobot, robot.forceSensor(contact.forceSensor())).position();
      });

  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.name() + "_ctlRobot_position", &contact,
      [this, &contact, &ctl]() -> Eigen::Vector3d
      {
        const auto & robot = ctl.robot(robot_);
        return getContactWorldKinematics(contact, robot, robot.forceSensor(contact.forceSensor())).position();
      });

  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.name() + "_worldcontactKineFromCentroid_position",
                     &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getWorldContactKineFromCentroid(contact.id()).position(); });

  logger.addLogEntry(
      category_ + "_debug_contactKine_" + contact.name() + "_worldcontactKineFromCentroid_orientation", &contact,
      [this, &contact]() -> Eigen::Quaternion<double>
      { return observer_.getWorldContactKineFromCentroid(contact.id()).orientation.inverse().toQuaternion(); });

  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.name() + "_worldcontactKineFromCentroid_linVel",
                     &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getWorldContactKineFromCentroid(contact.id()).linVel(); });

  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.name() + "_worldcontactKineFromCentroid_angVel",
                     &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getWorldContactKineFromCentroid(contact.id()).angVel(); });

  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.name() + "_inputUserContactKine_position", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getUserContactInputKine(contact.id()).position(); });
  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.name() + "_inputUserContactKine_orientation", &contact,
                     [this, &contact]() -> Eigen::Quaternion<double>
                     { return observer_.getUserContactInputKine(contact.id()).orientation.inverse().toQuaternion(); });
  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.name() + "_inputUserContactKine_linVel", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getUserContactInputKine(contact.id()).linVel(); });
  logger.addLogEntry(category_ + "_debug_contactKine_" + contact.name() + "_inputUserContactKine_angVel", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     { return observer_.getUserContactInputKine(contact.id()).angVel(); });

  logger.addLogEntry(category_ + "_debug_contactState_isSet_" + contact.name(), &contact,
                     [&contact]() -> std::string { return contact.isSet() ? "Set" : "notSet"; });
}

void MCKineticsObserver::addContactMeasurementsLogEntries(mc_rtc::Logger & logger, const KoContactWithSensor & contact)
{
  // Innovation
  logger.addLogEntry(category_ + "_MEKF_innovation_contacts_" + contact.name() + "_position", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment(observer_.contactPosIndexTangent(contact.id()),
                                                                         observer_.sizePosTangent);
                     });
  logger.addLogEntry(category_ + "_MEKF_innovation_contacts_" + contact.name() + "_orientation", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment(observer_.contactOriIndexTangent(contact.id()),
                                                                         observer_.sizeOriTangent);
                     });
  logger.addLogEntry(category_ + "_MEKF_innovation_contacts_" + contact.name() + "_force", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment(
                           observer_.contactForceIndexTangent(contact.id()), observer_.sizeForceTangent);
                     });
  logger.addLogEntry(category_ + "_MEKF_innovation_contacts_" + contact.name() + "_torque", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment(
                           observer_.contactTorqueIndexTangent(contact.id()), observer_.sizeTorqueTangent);
                     });

  logger.addLogEntry(
      category_ + "_MEKF_measurements_contacts_force_" + contact.name() + "_viscoAfterCorrection", &contact,
      [&contact]() -> Eigen::Vector3d { return contact.viscoElasticWrenchAfterCorrection_.segment(0, 3); });
  logger.addLogEntry(
      category_ + "_MEKF_measurements_contacts_torque_" + contact.name() + "_viscoAfterCorrection", &contact,
      [&contact]() -> Eigen::Vector3d { return contact.viscoElasticWrenchAfterCorrection_.segment(3, 3); });

  // Measurements
  logger.addLogEntry(category_ + "_MEKF_measurements_contacts_force_" + contact.name() + "_measured", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getLastMeasurement().segment(
                           observer_.getContactMeasIndexByNum(contact.id()), observer_.sizeForce);
                     });
  logger.addLogEntry(category_ + "_MEKF_measurements_contacts_force_" + contact.name() + "_predicted", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getLastPredictedMeasurement().segment(
                           observer_.getContactMeasIndexByNum(contact.id()), observer_.sizeForce);
                     });
  logger.addLogEntry(category_ + "_MEKF_measurements_contacts_force_" + contact.name() + "_corrected", &contact,
                     [this, &contact]() -> Eigen::Vector3d {
                       return correctedMeasurements_.segment(observer_.getContactMeasIndexByNum(contact.id()),
                                                             observer_.sizeForce);
                     });
  logger.addLogEntry(category_ + "_MEKF_measurements_contacts_torque_" + contact.name() + "_measured", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getLastMeasurement().segment(
                           observer_.getContactMeasIndexByNum(contact.id()) + observer_.sizeForce,
                           observer_.sizeTorque);
                     });
  logger.addLogEntry(category_ + "_MEKF_measurements_contacts_torque_" + contact.name() + "_predicted", &contact,
                     [this, &contact]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getLastPredictedMeasurement().segment(
                           observer_.getContactMeasIndexByNum(contact.id()) + observer_.sizeForce,
                           observer_.sizeTorque);
                     });
  logger.addLogEntry(category_ + "_MEKF_measurements_contacts_torque_" + contact.name() + "_corrected", &contact,
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
}

void MCKineticsObserver::removeContactMeasurementsLogEntries(mc_rtc::Logger & logger,
                                                             const KoContactWithSensor & contact)
{
  // Innovation
  logger.removeLogEntry(category_ + "_innovation_contacts_" + contact.name() + "_position");
  logger.removeLogEntry(category_ + "_innovation_contacts_" + contact.name() + "_orientation");
  logger.removeLogEntry(category_ + "_innovation_contacts_" + contact.name() + "_force");
  logger.removeLogEntry(category_ + "_innovation_contacts_" + contact.name() + "_torque");

  logger.removeLogEntry(category_ + "_measurements_contacts_force_" + contact.name() + "_measured");
  logger.removeLogEntry(category_ + "_measurements_contacts_force_" + contact.name() + "_predicted");
  logger.removeLogEntry(category_ + "_measurements_contacts_force_" + contact.name() + "_corrected");

  logger.removeLogEntry(category_ + "_measurements_contacts_torque_" + contact.name() + "_measured");
  logger.removeLogEntry(category_ + "_measurements_contacts_torque_" + contact.name() + "_predicted");
  logger.removeLogEntry(category_ + "_measurements_contacts_torque_" + contact.name() + "_corrected");
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("MCKineticsObserver", mc_state_observation::MCKineticsObserver)
