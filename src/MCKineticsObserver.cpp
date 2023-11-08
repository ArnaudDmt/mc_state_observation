/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>
#include "mc_state_observation/observersTools/measurementsTools.h"
#include <mc_state_observation/MCKineticsObserver.h>
#include <mc_state_observation/gui_helpers.h>

#include <RBDyn/CoM.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <typeinfo>

#include <mc_state_observation/observersTools/kinematicsTools.h>

namespace so = stateObservation;

namespace mc_state_observation
{
MCKineticsObserver::MCKineticsObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), observer_(4, 2)
{
  observer_.setSamplingTime(dt);
}

///////////////////////////////////////////////////////////////////////
/// --------------------------Core functions---------------------------
///////////////////////////////////////////////////////////////////////

void MCKineticsObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());

  const auto & robot = ctl.robot(robot_);

  IMUs_ = config("imuSensor", ctl.robot().bodySensors());
  config("debug", debug_);
  config("verbose", verbose_);

  std::string typeOfOdometry = static_cast<std::string>(config("odometryType"));

  if(typeOfOdometry == "flatOdometry") { odometryType_ = measurements::flatOdometry; }
  else if(typeOfOdometry == "6dOdometry") { odometryType_ = measurements::odometry6d; }
  else if(typeOfOdometry == "None") { odometryType_ = measurements::None; }
  else
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Odometry type not allowed. Please pick among : [None, flatOdometry, 6dOdometry]");
  }

  config("withDebugLogs", withDebugLogs_);

  config("withFilteredForcesContactDetection", withFilteredForcesContactDetection_);

  /* configuration of the contacts manager */

  contactsManager_.init(observerName_, true);

  double contactDetectionPropThreshold = config("contactDetectionPropThreshold", 0.11);
  contactDetectionThreshold_ = robot.mass() * so::cst::gravityConstant * contactDetectionPropThreshold;

  std::string contactsDetection = static_cast<std::string>(config("contactsDetection"));

  KoContactsManager::ContactsDetection contactsDetectionMethod = KoContactsManager::ContactsDetection::undefined;
  if(contactsDetection == "fromThreshold")
  {
    contactsDetectionMethod = KoContactsManager::ContactsDetection::fromThreshold;
  }
  else if(contactsDetection == "fromSurfaces")
  {
    contactsDetectionMethod = KoContactsManager::ContactsDetection::fromSurfaces;
  }
  else if(contactsDetection == "fromSolver")
  {
    contactsDetectionMethod = KoContactsManager::ContactsDetection::fromSolver;
  }

  std::vector<std::string> contactsSensorsDisabledInit =
      config("contactsSensorDisabledInit", std::vector<std::string>());
  config("forceSensorsAsInput", forceSensorsAsInput_);

  if(contactsDetectionMethod == KoContactsManager::ContactsDetection::fromSurfaces)
  {
    std::vector<std::string> surfacesForContactDetection =
        config("surfacesForContactDetection", std::vector<std::string>());

    contactsManager_.initDetection(ctl, robot_, contactsDetectionMethod, surfacesForContactDetection,
                                   contactsSensorsDisabledInit, contactDetectionThreshold_);
  }
  else
  {
    contactsManager_.initDetection(ctl, robot_, contactsDetectionMethod, contactsSensorsDisabledInit,
                                   contactDetectionThreshold_, forceSensorsAsInput_);
  }

  if(withFilteredForcesContactDetection_)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>("The forces filtering has an error, please don't use it now");
  }

  /* Configuration of the Kinetics Observer's parameters */

  config("withUnmodeledWrench", withUnmodeledWrench_);
  config("withGyroBias", withGyroBias_);

  observer_.setWithUnmodeledWrench(withUnmodeledWrench_);
  observer_.setWithGyroBias(withGyroBias_);
  observer_.useFiniteDifferencesJacobians(config("withFiniteDifferences"));
  so::Vector dx(observer_.getStateSize());
  dx.setConstant(static_cast<double>(config("finiteDifferenceStep")));
  observer_.setFiniteDifferenceStep(dx);
  observer_.setWithAccelerationEstimation(config("withAccelerationEstimation"));

  linStiffness_ = (config("linStiffness").operator so::Vector3()).matrix().asDiagonal();
  angStiffness_ = (config("angStiffness").operator so::Vector3()).matrix().asDiagonal();
  linDamping_ = (config("linDamping").operator so::Vector3()).matrix().asDiagonal();
  angDamping_ = (config("angDamping").operator so::Vector3()).matrix().asDiagonal();

  zeroPose_.translation().setZero();
  zeroPose_.rotation().setIdentity();
  zeroMotion_.linear().setZero();
  zeroMotion_.angular().setZero();

  // Initial State
  statePositionInitCovariance_ = (config("statePositionInitVariance").operator so::Vector3()).matrix().asDiagonal();
  stateOriInitCovariance_ = (config("stateOriInitVariance").operator so::Vector3()).matrix().asDiagonal();
  stateLinVelInitCovariance_ = (config("stateLinVelInitVariance").operator so::Vector3()).matrix().asDiagonal();
  stateAngVelInitCovariance_ = (config("stateAngVelInitVariance").operator so::Vector3()).matrix().asDiagonal();
  gyroBiasInitCovariance_.setZero();
  unmodeledWrenchInitCovariance_.setZero();
  contactInitCovarianceFirstContacts_.setZero();
  contactInitCovarianceFirstContacts_.block<3, 3>(0, 0) =
      (config("contactPositionInitVarianceFirstContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceFirstContacts_.block<3, 3>(3, 3) =
      (config("contactOriInitVarianceFirstContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceFirstContacts_.block<3, 3>(6, 6) =
      (config("contactForceInitVarianceFirstContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceFirstContacts_.block<3, 3>(9, 9) =
      (config("contactTorqueInitVarianceFirstContacts").operator so::Vector3()).matrix().asDiagonal();

  contactInitCovarianceNewContacts_.setZero();
  contactInitCovarianceNewContacts_.block<3, 3>(0, 0) =
      (config("contactPositionInitVarianceNewContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceNewContacts_.block<3, 3>(3, 3) =
      (config("contactOriInitVarianceNewContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceNewContacts_.block<3, 3>(6, 6) =
      (config("contactForceInitVarianceNewContacts").operator so::Vector3()).matrix().asDiagonal();
  contactInitCovarianceNewContacts_.block<3, 3>(9, 9) =
      (config("contactTorqueInitVarianceNewContacts").operator so::Vector3()).matrix().asDiagonal();

  // Process //
  statePositionProcessCovariance_ =
      (config("statePositionProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  stateOriProcessCovariance_ = (config("stateOriProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  stateLinVelProcessCovariance_ = (config("stateLinVelProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  stateAngVelProcessCovariance_ = (config("stateAngVelProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  gyroBiasProcessCovariance_.setZero();
  unmodeledWrenchProcessCovariance_.setZero();

  contactProcessCovariance_.setZero();
  contactProcessCovariance_.block<3, 3>(0, 0) =
      (config("contactPositionProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  contactProcessCovariance_.block<3, 3>(3, 3) =
      (config("contactOrientationProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  contactProcessCovariance_.block<3, 3>(6, 6) =
      (config("contactForceProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  contactProcessCovariance_.block<3, 3>(9, 9) =
      (config("contactTorqueProcessVariance").operator so::Vector3()).matrix().asDiagonal();

  // Unmodeled Wrench //
  if(withUnmodeledWrench_)
  {
    // initial
    unmodeledWrenchInitCovariance_.block<3, 3>(0, 0) =
        (config("unmodeledForceInitVariance").operator so::Vector3()).matrix().asDiagonal();
    unmodeledWrenchInitCovariance_.block<3, 3>(3, 3) =
        (config("unmodeledTorqueInitVariance").operator so::Vector3()).matrix().asDiagonal();

    // process
    unmodeledWrenchProcessCovariance_.block<3, 3>(0, 0) =
        (config("unmodeledForceProcessVariance").operator so::Vector3()).matrix().asDiagonal();
    unmodeledWrenchProcessCovariance_.block<3, 3>(3, 3) =
        (config("unmodeledTorqueProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  }
  // Gyrometer Bias
  if(withGyroBias_)
  {
    gyroBiasInitCovariance_ = (config("gyroBiasInitVariance").operator so::Vector3()).matrix().asDiagonal();
    gyroBiasProcessCovariance_ = (config("gyroBiasProcessVariance").operator so::Vector3()).matrix().asDiagonal();
  }

  // Sensor //
  positionSensorCovariance_ = (config("positionSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  orientationSensorCoVariance_ = (config("orientationSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  acceleroSensorCovariance_ = (config("acceleroSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  gyroSensorCovariance_ = (config("gyroSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  absoluteOriSensorCovariance_ = (config("absOriSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  contactSensorCovariance_.setZero();
  contactSensorCovariance_.block<3, 3>(0, 0) =
      (config("forceSensorVariance").operator so::Vector3()).matrix().asDiagonal();
  contactSensorCovariance_.block<3, 3>(3, 3) =
      (config("torqueSensorVariance").operator so::Vector3()).matrix().asDiagonal();

  setObserverCovariances();

  /* Configuration of the backup based on the Tilt Observer */

  if(!ctl.datastore().has("runBackup"))
  {
    // if the datastore must contain the backup function provided by the Tilt Observer
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "The Tilt Observer must be used before the Kinetics Observer when used as a backup");
  }

  // interval (in s) on which the backup will recover
  int backupInterval = config("backupInterval", 1);
  auto & datastore = (const_cast<mc_control::MCController &>(ctl)).datastore();
  datastore.make<int>("KoBackupInterval", backupInterval);

  backupIterInterval_ = int(backupInterval / ctl.timeStep);

  koBackupFbKinematics_.resize(backupIterInterval_);

  datastore.make<int>("koBackupIterInterval", backupIterInterval_);
  datastore.make<boost::circular_buffer<stateObservation::kine::Kinematics> *>("koBackupFbKinematics",
                                                                               &koBackupFbKinematics_);

  invincibilityFrame_ = int(1.5 / ctl.timeStep);

  ctl.gui()->addElement({observerName_},
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
  observer_.setAbsoluteOriSensorDefaultCovarianceMatrix(absoluteOriSensorCovariance_);
}

void MCKineticsObserver::reset(const mc_control::MCController & ctl)
{
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

  for(const auto & imu : IMUs_) { mapIMUs_.insertIMU(imu.name()); }

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
      {"Robots"},
      mc_rtc::gui::Robot(observerName_, [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));
  ctl.gui()->addElement({"Robots"},
                        mc_rtc::gui::Robot("Real", [&ctl]() -> const mc_rbdyn::Robot & { return ctl.realRobot(); }));

  X_0_fb_ = robot.posW().translation();

  initObserverStateVector(realRobot);
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
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  auto & inputRobot = my_robots_->robot("inputRobot");
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  inputRobot.mbc() = realRobot.mbc();
  inputRobot.mb() = realRobot.mb();

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

  /** Contacts
   * Note that when we use force sensors directly for the contact detection, the pose of the contact is the one of the
   * force sensor and not the contact surface!
   */
  // retrieves the list of contacts and set simStarted to true once a contact is detected
  updateContacts(ctl, findNewContacts(ctl), logger);

  // force measurements from sensor that are not associated to a currently set contact are given to the Kinetics
  // Observer as inputs.
  inputAdditionalWrench(inputRobot, robot);

  /** Accelerometers **/
  updateIMUs(robot, inputRobot);

  /*
  so::kine::Orientation oriMeasurement;
  //oriMeasurement = so::Matrix3(realRobot.posW().rotation().transpose());
  observer_.setAbsoluteOriSensor(oriMeasurement);
  */

  /** Inertias **/
  /** TODO : Merge inertias into CoM inertia and/or get it from fd() **/

  observer_.setCoMAngularMomentum(
      rbd::computeCentroidalMomentum(inputRobot.mb(), inputRobot.mbc(), inputRobot.com()).moment());

  observer_.setCoMInertiaMatrix(so::Matrix3(
      inertiaWaist_.inertia() + observer_.getMass() * so::kine::skewSymmetric2(observer_.getCenterOfMass()())));
  /* Step once, and return result */

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
      auto & datastore = (const_cast<mc_control::MCController &>(ctl)).datastore();
      // we apply the last transformation estimated by the Tilt Observer to our previous pose to keep updating the
      // floating base with the Tilt Observer.
      mcko_K_0_fb = datastore.call<so::kine::Kinematics>("applyLastTransformation", koBackupFbKinematics_.back());
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
      // cannot be used. So we let it converge during the invincibility frame while using the estimation of the Tilt to
      // update the real robot. Then we start over using the Kinetics Observer starting from the final kinematics
      // obtained from the Tilt Observer.
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

        for(const int & contactIndex : contactsManager_.contactsFound())
        {
          KoContactWithSensor contact = contactsManager_.contactWithSensor(contactIndex);

          // Update of the force measurements (the contribution of the gravity changed)
          const mc_rbdyn::ForceSensor & forceSensor = robot.forceSensor(contact.forceSensorName());

          // the tilt of the robot changed so the contribution of the gravity to the measurements changed too
          if(KoContactsManager().getContactsDetection() == KoContactsManager::ContactsDetection::fromThreshold)
          {
            updateContactForceMeasurement(contact, forceSensor.wrenchWithoutGravity(inputRobot));
          }
          else // the kinematics of the contact are the ones of the associated surface
          {
            updateContactForceMeasurement(contact, contact.surfaceSensorKine_,
                                          forceSensor.wrenchWithoutGravity(inputRobot));
          }

          so::kine::Kinematics newWorldContactKineRef;

          getOdometryWorldContactRest(ctl, contact, newWorldContactKineRef);

          observer_.setStateContact(contactIndex, newWorldContactKineRef, contact.contactWrenchVector_, false);
        }
      }

      break;
    }
    case errorDetected:
    {
      // an error was just detected, we reset the state vector and covariances and start the invicibility frame, during
      // which we let the Kinetics Observer converge before using it again.
      auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();
      if(logger.t() / ctl.timeStep < backupIterInterval_)
      {
        mc_rtc::log::warning("The backup function was called before the required time was ellapsed. The backup will be "
                             "performed using the last {} seconds",
                             logger.t());
      }

      if(logger.t() / ctl.timeStep - lastBackupIter_ < backupIterInterval_)
      {
        mc_rtc::log::warning("The backup function was called again too quickly. The backup will be "
                             "performed using the last {} seconds",
                             logger.t() - lastBackupIter_ * ctl.timeStep);
      }

      auto & datastore = (const_cast<mc_control::MCController &>(ctl)).datastore();
      // We add an empty Kinematics object to the floating base pose buffer. This is because the buffer of the tilt
      // observer already contains the last estimation of the floating base so we prevent a disalignment of the two
      // buffers. This empty Kinematics is filled and returned by the "runBackup" function.
      koBackupFbKinematics_.push_back(so::kine::Kinematics::zeroKinematics(so::kine::Kinematics::Flags::pose));
      mcko_K_0_fb = datastore.call<const so::kine::Kinematics>("runBackup");

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

      for(int i = 0; i < mapIMUs_.getList().size(); i++) { observer_.setGyroBias(so::Vector3::Zero(), i, true); }

      for(const int & contactIndex : contactsManager_.contactsFound())
      {
        KoContactWithSensor contact = contactsManager_.contactWithSensor(contactIndex);

        // Update of the force measurements (the offset due to the gravity changed)
        const mc_rbdyn::ForceSensor & forceSensor = inputRobot.forceSensor(contact.forceSensorName());

        so::kine::Kinematics bodySensorKine =
            kinematicsTools::poseFromSva(forceSensor.X_p_f(), so::kine::Kinematics::Flags::vel);

        so::kine::Kinematics bodySurfaceKine = kinematicsTools::poseFromSva(
            inputRobot.surface(contact.surfaceName()).X_b_s(), so::kine::Kinematics::Flags::vel);

        so::kine::Kinematics surfaceSensorKine = bodySurfaceKine.getInverse() * bodySensorKine;

        updateContactForceMeasurement(contact, surfaceSensorKine, forceSensor.wrenchWithoutGravity(inputRobot));

        so::kine::Kinematics newWorldContactKineRef;

        getOdometryWorldContactRest(ctl, contact, newWorldContactKineRef);

        observer_.setStateContact(contactIndex, newWorldContactKineRef, contact.contactWrenchVector_, true);
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
    globalCentroidKinematics_ = observer_.getGlobalCentroidKinematics();
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

void MCKineticsObserver::initObserverStateVector(const mc_rbdyn::Robot & robot)
{
  so::kine::Orientation initOrientation;
  initOrientation.setZeroRotation<so::Quaternion>();
  Eigen::VectorXd initStateVector;
  initStateVector = Eigen::VectorXd::Zero(observer_.getStateSize());

  initStateVector.segment(observer_.posIndex(), observer_.sizePos) = robot.com();
  initStateVector.segment(observer_.oriIndex(), observer_.sizeOri) = initOrientation.toVector4();
  initStateVector.segment(observer_.linVelIndex(), observer_.sizeLinVel) = robot.comVelocity();

  observer_.setInitWorldCentroidStateVector(initStateVector);
}

void MCKineticsObserver::update(mc_control::MCController & ctl) // this function is called by the pipeline if the
                                                                // update is set to true in the configuration file
{
  auto & datastore = (const_cast<mc_control::MCController &>(ctl)).datastore();
  // this function checks that the backup estimator uses the same odometry type than the Kinetics Observer
  datastore.call<>("checkCorrectBackupConf", odometryType_);

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

  for(auto & contactWithSensor : contactsManager_.contactsWithSensors())
  {
    KoContactWithSensor & contact = contactWithSensor.second;
    const std::string & fsName = contact.forceSensorName();

    if(!contact.isSet_
       && contact.sensorEnabled_) // if the contact is not set but we use the force sensor measurements,
                                  // then we give the measured force as an input to the Kinetics Observer
    {
      sva::ForceVecd measuredWrench = measRobot.forceSensor(fsName).worldWrenchWithoutGravity(inputRobot);
      additionalUserResultingForce_ += measuredWrench.force();
      additionalUserResultingMoment_ += measuredWrench.moment();
    }
  }

  addSensorsAsInputs(inputRobot, measRobot, additionalUserResultingForce_, additionalUserResultingMoment_);

  // We pass this computed wrench as an input to the Kinetics Observer
  observer_.setAdditionalWrench(additionalUserResultingForce_, additionalUserResultingMoment_);

  if(withDebugLogs_)
  {
    for(auto & contactWithSensor :
        contactsManager_.contactsWithSensors()) // if a force sensor is not associated to a contact, its
                                                // measurement is given as an input external wrench
    {
      KoContactWithSensor & contact = contactWithSensor.second;
      const std::string & fsName = contact.forceSensorName();
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
  for(const auto & imu : IMUs_)
  {
    /** Position of accelerometer **/

    const sva::PTransformd & bodyImuPose = inputRobot.bodySensor(imu.name()).X_b_s();
    so::kine::Kinematics bodyImuKine =
        kinematicsTools::poseFromSva(bodyImuPose, so::kine::Kinematics::Flags::vel | so::kine::Kinematics::Flags::acc);

    so::kine::Kinematics worldBodyKine = kinematicsTools::kinematicsFromSva(
        inputRobot.mbc().bodyPosW[inputRobot.bodyIndexByName(imu.parentBody())],
        inputRobot.mbc().bodyVelW[inputRobot.bodyIndexByName(imu.parentBody())],
        inputRobot.mbc().bodyAccB[inputRobot.bodyIndexByName(imu.parentBody())], true, false);

    so::kine::Kinematics worldImuKine = worldBodyKine * bodyImuKine;
    const so::kine::Kinematics fbImuKine = worldImuKine;

    observer_.setIMU(measRobot.bodySensor().linearAcceleration(), measRobot.bodySensor().angularVelocity(),
                     acceleroSensorCovariance_, gyroSensorCovariance_, fbImuKine, mapIMUs_.getNumFromName(imu.name()));
  }
}

const measurements::ContactsManager<KoContactWithSensor, measurements::ContactWithoutSensor>::ContactsSet &
    MCKineticsObserver::findNewContacts(const mc_control::MCController & ctl)
{
  contactsManager_.findContacts(ctl, robot_);

  return contactsManager_.contactsFound(); // list of currently set contacts
}

const so::kine::Kinematics MCKineticsObserver::getContactWorldKinematicsAndWrench(KoContactWithSensor & contact,
                                                                                  const mc_rbdyn::Robot & currentRobot,
                                                                                  const mc_rbdyn::ForceSensor & fs,
                                                                                  const sva::ForceVecd & measuredWrench)
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
      kinematicsTools::poseFromSva(bodyContactSensorPose, so::kine::Kinematics::Flags::vel);

  // kinematics of the sensor's parent body in the world
  so::kine::Kinematics worldBodyKine = kinematicsTools::poseAndVelFromSva(
      currentRobot.mbc().bodyPosW[currentRobot.bodyIndexByName(fs.parentBody())],
      currentRobot.mbc().bodyVelW[currentRobot.bodyIndexByName(fs.parentBody())], true);

  // kinematics of the frame of the force sensor in the world frame
  so::kine::Kinematics worldSensorKine = worldBodyKine * bodyContactSensorKine;

  if(KoContactsManager().getContactsDetection() == KoContactsManager::ContactsDetection::fromThreshold)
  {
    // If the contact is detecting using thresholds, we will then consider the sensor frame as
    // the contact surface frame directly.
    worldContactKine = worldSensorKine;
    updateContactForceMeasurement(contact, measuredWrench);
  }
  else // the kinematics of the contacts are the ones of the surface, but we must transport the measured wrench
  {
    // pose of the surface in the world / floating base's frame
    sva::PTransformd worldSurfacePose = currentRobot.surfacePose(contact.surfaceName());
    // Kinematics of the surface in the world / floating base's frame
    worldContactKine = kinematicsTools::poseFromSva(worldSurfacePose, so::kine::Kinematics::Flags::vel);

    contact.surfaceSensorKine_ = worldContactKine.getInverse() * worldSensorKine;
    // expressing the force measurement in the frame of the surface
    updateContactForceMeasurement(contact, contact.surfaceSensorKine_, measuredWrench);
  }

  return worldContactKine;
}

const so::kine::Kinematics MCKineticsObserver::getContactWorldKinematics(KoContactWithSensor & contact,
                                                                         const mc_rbdyn::Robot & currentRobot,
                                                                         const mc_rbdyn::ForceSensor & fs)
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
      kinematicsTools::poseFromSva(bodyContactSensorPose, so::kine::Kinematics::Flags::vel);

  // kinematics of the sensor's parent body in the world frame
  so::kine::Kinematics worldBodyKine = kinematicsTools::poseAndVelFromSva(
      currentRobot.mbc().bodyPosW[currentRobot.bodyIndexByName(fs.parentBody())],
      currentRobot.mbc().bodyVelW[currentRobot.bodyIndexByName(fs.parentBody())], true);

  so::kine::Kinematics worldSensorKine = worldBodyKine * bodyContactSensorKine;

  if(KoContactsManager().getContactsDetection() == KoContactsManager::ContactsDetection::fromThreshold)
  {
    // If the contact is detecting using thresholds, we will then consider the sensor frame as
    // the contact surface frame directly.
    worldContactKine = worldSensorKine;
  }
  else // the kinematics of the contacts are the ones of the surface.
  {
    // pose of the surface in the world / floating base's frame
    sva::PTransformd worldSurfacePose = currentRobot.surfacePose(contact.surfaceName());
    // Kinematics of the surface in the world / floating base's frame
    worldContactKine = kinematicsTools::poseFromSva(worldSurfacePose, so::kine::Kinematics::Flags::vel);
  }

  return worldContactKine;
}

void MCKineticsObserver::updateContactForceMeasurement(KoContactWithSensor & contact,
                                                       so::kine::Kinematics surfaceSensorKine,
                                                       const sva::ForceVecd & measuredWrench)
{
  // expressing the force measurement in the frame of the surface
  contact.contactWrenchVector_.segment<3>(0) = surfaceSensorKine.orientation * measuredWrench.force();

  // expressing the torque measurement in the frame of the surface
  contact.contactWrenchVector_.segment<3>(3) =
      surfaceSensorKine.orientation * measuredWrench.moment()
      + surfaceSensorKine.position().cross(contact.contactWrenchVector_.segment<3>(0));
}

void MCKineticsObserver::updateContactForceMeasurement(KoContactWithSensor & contact,
                                                       const sva::ForceVecd & measuredWrench)
{
  // If the contact is detecting using thresholds, we will then consider the sensor frame as
  // the contact surface frame directly.
  contact.contactWrenchVector_.segment<3>(0) = measuredWrench.force(); // retrieving the force measurement
  contact.contactWrenchVector_.segment<3>(3) = measuredWrench.moment(); // retrieving the torque measurement
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

  if(odometryType_ == measurements::flatOdometry) // if true, the position odometry is made only along the x and y axis,
                                                  // the position along z is assumed to be the one of the control robot
  {
    // kinematics of the contact of the control robot in the world frame
    so::kine::Kinematics worldContactKineControl =
        getContactWorldKinematics(contact, robot, robot.forceSensor(contact.forceSensorName()));

    // the reference altitude of the contact is the one in the control robot
    worldContactKineRef.position()(2) = worldContactKineControl.position()(2);
  }
}

void MCKineticsObserver::updateContact(const mc_control::MCController & ctl,
                                       const int & contactIndex,
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
  KoContactWithSensor & contact = contactsManager_.contactWithSensor(contactIndex);

  sva::ForceVecd measuredWrench = robot.forceSensor(contact.forceSensorName()).wrenchWithoutGravity(inputRobot);
  const mc_rbdyn::ForceSensor & forceSensor = robot.forceSensor(contact.forceSensorName());

  // As used on input robot, returns the kinematics of the contact in the frame of the floating base. Also expresses the
  // measured wrench in the frame of the contact.
  contact.fbContactKine_ = getContactWorldKinematicsAndWrench(contact, inputRobot, forceSensor, measuredWrench);

  switch(contact.wasAlreadySet_)
  {
    // the contact already exists, it is updated
    case true:
      if(contact.sensorEnabled_) // the force sensor attached to the contact is used in the correction by the
                                 // Kinetics Observer.
      {
        observer_.updateContactWithWrenchSensor(contact.contactWrenchVector_, contactSensorCovariance_,
                                                contact.fbContactKine_, contactIndex);
      }
      else { observer_.updateContactWithNoSensor(contact.fbContactKine_, contactIndex); }

      if(withDebugLogs_)
      {
        if(contact.sensorEnabled_ && !contact.sensorWasEnabled_)
        {
          addContactMeasurementsLogEntries(logger, contactIndex);
          contact.sensorWasEnabled_ = true;
        }
        if(!contact.sensorEnabled_ && contact.sensorWasEnabled_)
        {
          removeContactMeasurementsLogEntries(logger, contactIndex);
          contact.sensorWasEnabled_ = false;
        }
      }
      break;

    // the contact doesn't exist yet, it is updated
    case false:
      // reference of the contact in the world / floating base of the input robot
      so::kine::Kinematics worldContactKineRef;

      if(odometryType_ != measurements::None) // the Kinetics Observer performs odometry. The estimated state is used to
                                              // provide the new contacts references.
      {
        getOdometryWorldContactRest(ctl, contact, worldContactKineRef);
      }
      else // we don't perform odometry, the reference pose of the contact is its pose in the control robot
      {
        worldContactKineRef = getContactWorldKinematics(contact, robot, forceSensor);
      }

      if(observer_.getNumberOfSetContacts() > 0) // The initial covariance on the pose of the contact depending on
                                                 // whether another contact is already set or not
      {
        observer_.addContact(worldContactKineRef, contactInitCovarianceNewContacts_, contactProcessCovariance_,
                             contactIndex, linStiffness_, linDamping_, angStiffness_, angDamping_);
      }
      else
      {
        observer_.addContact(worldContactKineRef, contactInitCovarianceFirstContacts_, contactProcessCovariance_,
                             contactIndex, linStiffness_, linDamping_, angStiffness_, angDamping_);
      }
      if(contact.sensorEnabled_) // checks if the sensor is used in the correction of the Kinetics Observer
                                 // or not
      {
        // we update the measurements of the sensor and the input kinematics of the contact in the user /
        // floating base's frame
        observer_.updateContactWithWrenchSensor(contact.contactWrenchVector_, contactSensorCovariance_,
                                                contact.fbContactKine_, contactIndex);
      }
      else
      {
        // we update the input kinematics of the contact in the user / floating base's frame
        observer_.updateContactWithNoSensor(contact.fbContactKine_, contactIndex);
      }

      if(withDebugLogs_) { addContactLogEntries(logger, contactIndex); }
      break;
  }
}

void MCKineticsObserver::updateContacts(
    const mc_control::MCController & ctl,
    const measurements::ContactsManager<KoContactWithSensor, measurements::ContactWithoutSensor>::ContactsSet &
        updatedContactsIndexes,
    mc_rtc::Logger & logger)
{
  for(const auto & updatedContactIndex : updatedContactsIndexes) { updateContact(ctl, updatedContactIndex, logger); }
  // List of the contact that were set on last iteration but are not set anymore on the current one
  for(const int & removedContactIndex : contactsManager_.removedContacts())
  {
    observer_.removeContact(removedContactIndex);

    if(withDebugLogs_)
    {
      removeContactLogEntries(logger, removedContactIndex);
      removeContactMeasurementsLogEntries(logger, removedContactIndex);
    }
  }

  unsigned nbContacts = static_cast<unsigned>(updatedContactsIndexes.size());
  if(debug_) { mc_rtc::log::info("nbContacts = {}", nbContacts); }
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
  logger.addLogEntry(category + "_mcko_fb_posW", [this]() -> sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category + "_mcko_fb_velW", [this]() -> sva::MotionVecd & { return v_fb_0_; });
  logger.addLogEntry(category + "_mcko_fb_accW", [this]() -> sva::MotionVecd & { return a_fb_0_; });

  logger.addLogEntry(category + "_mcko_fb_yaw",
                     [this]() -> double { return -so::kine::rotationMatrixToYawAxisAgnostic(X_0_fb_.rotation()); });

  logger.addLogEntry(category + "_constants_mass", [this]() -> double { return observer_.getMass(); });

  logger.addLogEntry(category + "_constants_forceThreshold", [this]() -> double { return contactDetectionThreshold_; });
  logger.addLogEntry(category + "_debug_estimationState",
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
  logger.addLogEntry(category + "_debug_OdometryType",
                     [this]() -> std::string
                     {
                       switch(odometryType_)
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

  /* Plots of the updated state */
  kinematicsTools::addToLogger(globalCentroidKinematics_, logger, observerName_ + "_globalWorldCentroidState");
  logger.addLogEntry(observerName_ + "_globalWorldCentroidState_positionW_",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.position(); });
  logger.addLogEntry(observerName_ + "_globalWorldCentroidState_linVelW",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.linVel(); });
  logger.addLogEntry(observerName_ + "_globalWorldCentroidState_linAccW",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.linAcc(); });
  logger.addLogEntry(observerName_ + "_globalWorldCentroidState_oriW",
                     [this]() -> Eigen::Quaternion<double>
                     { return globalCentroidKinematics_.orientation.inverse().toQuaternion(); });
  logger.addLogEntry(observerName_ + "_globalWorldCentroidState_angVelW",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.angVel(); });
  logger.addLogEntry(observerName_ + "_globalWorldCentroidState_angAccW",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.angAcc(); });
  for(const auto & imu : IMUs_)
  {
    logger.addLogEntry(observerName_ + "_globalWorldCentroidState_gyroBias_" + imu.name(),
                       [this, imu]() -> Eigen::Vector3d
                       {
                         return observer_.getCurrentStateVector().segment(
                             observer_.gyroBiasIndex(mapIMUs_.getNumFromName(imu.name())), observer_.sizeGyroBias);
                       });
  }
  logger.addLogEntry(
      observerName_ + "_globalWorldCentroidState_extForceCentr",
      [this]() -> Eigen::Vector3d
      { return observer_.getCurrentStateVector().segment(observer_.unmodeledForceIndex(), observer_.sizeForce); });

  logger.addLogEntry(
      observerName_ + "_globalWorldCentroidState_extTorqueCentr",
      [this]() -> Eigen::Vector3d
      { return observer_.getCurrentStateVector().segment(observer_.unmodeledTorqueIndex(), observer_.sizeTorque); });

  /* Inputs */
  logger.addLogEntry(observerName_ + "_inputs_additionalWrench_Force",
                     [this]() -> Eigen::Vector3d
                     { return observer_.getAdditionalWrench().segment(0, observer_.sizeForce); });
  logger.addLogEntry(observerName_ + "_inputs_additionalWrench_Torque",
                     [this]() -> Eigen::Vector3d
                     { return observer_.getAdditionalWrench().segment(observer_.sizeForce, observer_.sizeTorque); });

  /* State covariances */
  logger.addLogEntry(observerName_ + "_stateCovariances_positionW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block(observer_.posIndexTangent(), observer_.posIndexTangent(), observer_.sizePosTangent,
                                  observer_.sizePosTangent)
                           .diagonal();
                     });
  logger.addLogEntry(observerName_ + "_stateCovariances_orientationW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block(observer_.oriIndexTangent(), observer_.oriIndexTangent(), observer_.sizeOriTangent,
                                  observer_.sizeOriTangent)
                           .diagonal();
                     });
  logger.addLogEntry(observerName_ + "_stateCovariances_linVelW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block(observer_.linVelIndexTangent(), observer_.linVelIndexTangent(),
                                  observer_.sizeLinVelTangent, observer_.sizeLinVelTangent)
                           .diagonal();
                     });
  logger.addLogEntry(observerName_ + "_stateCovariances_angVelW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block(observer_.angVelIndexTangent(), observer_.angVelIndexTangent(),
                                  observer_.sizeAngVelTangent, observer_.sizeAngVelTangent)
                           .diagonal();
                     });

  for(const auto & imu : IMUs_)
  {
    logger.addLogEntry(observerName_ + "_stateCovariances_gyroBias_" + imu.name(),
                       [this, imu]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF()
                             .getStateCovariance()
                             .block(observer_.gyroBiasIndexTangent(mapIMUs_.getNumFromName(imu.name())),
                                    observer_.gyroBiasIndexTangent(mapIMUs_.getNumFromName(imu.name())),
                                    observer_.sizeGyroBiasTangent, observer_.sizeGyroBiasTangent)
                             .diagonal();
                       });
  }

  logger.addLogEntry(observerName_ + "_stateCovariances_extForce_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block(observer_.unmodeledForceIndexTangent(), observer_.unmodeledForceIndexTangent(),
                                  observer_.sizeForceTangent, observer_.sizeForceTangent)
                           .diagonal();
                     });
  logger.addLogEntry(observerName_ + "_stateCovariances_extTorque_",
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
    logger.addLogEntry(observerName_ + "_realRobot_LeftFoot",
                       [&ctl]() { return ctl.realRobot().frame("LeftFoot").position(); });
  }

  if(ctl.realRobot().hasBody("RightFoot"))
  {
    logger.addLogEntry(observerName_ + "_realRobot_RightFoot",
                       [&ctl]() { return ctl.realRobot().frame("RightFoot").position(); });
  }

  if(ctl.realRobot().hasBody("LeftHand"))
  {
    logger.addLogEntry(observerName_ + "_realRobot_LeftHand",
                       [&ctl]() { return ctl.realRobot().frame("LeftHand").position(); });
  }
  if(ctl.realRobot().hasBody("RightHand"))
  {
    logger.addLogEntry(observerName_ + "_realRobot_RightHand",
                       [&ctl]() { return ctl.realRobot().frame("RightHand").position(); });
  }
  if(ctl.robot().hasBody("LeftFoot"))
  {
    logger.addLogEntry(observerName_ + "_ctlRobot_LeftFoot",
                       [&ctl]() { return ctl.robot().frame("LeftFoot").position(); });
  }
  if(ctl.robot().hasBody("RightFoot"))
  {
    logger.addLogEntry(observerName_ + "_ctlRobot_RightFoot",
                       [&ctl]() { return ctl.robot().frame("RightFoot").position(); });
  }

  if(ctl.robot().hasBody("LeftHand"))
  {
    logger.addLogEntry(observerName_ + "_ctlRobot_LeftHand",
                       [&ctl]() { return ctl.robot().frame("LeftHand").position(); });
  }

  if(ctl.robot().hasBody("RightHand"))
  {
    logger.addLogEntry(observerName_ + "_ctlRobot_RightHand",
                       [&ctl]() { return ctl.robot().frame("RightHand").position(); });
  }

  /* Plots of the inputs */

  logger.addLogEntry(observerName_ + "_inputs_angularMomentum",
                     [this]() -> Eigen::Vector3d { return observer_.getAngularMomentum()(); });
  logger.addLogEntry(observerName_ + "_inputs_angularMomentumDot",
                     [this]() -> Eigen::Vector3d { return observer_.getAngularMomentumDot()(); });
  logger.addLogEntry(observerName_ + "_inputs_com",
                     [this]() -> Eigen::Vector3d { return observer_.getCenterOfMass()(); });
  logger.addLogEntry(observerName_ + "_inputs_comDot",
                     [this]() -> Eigen::Vector3d { return observer_.getCenterOfMassDot()(); });
  logger.addLogEntry(observerName_ + "_inputs_comDotDot",
                     [this]() -> Eigen::Vector3d { return observer_.getCenterOfMassDotDot()(); });
  logger.addLogEntry(observerName_ + "_inputs_inertiaMatrix",
                     [this]() -> Eigen::Vector6d
                     {
                       so::Vector6 inertia;
                       inertia.segment<3>(0) = observer_.getInertiaMatrix()().diagonal();
                       inertia.segment<2>(3) = observer_.getInertiaMatrix()().block<1, 2>(0, 1);
                       inertia(5) = observer_.getInertiaMatrix()()(1, 2);
                       return inertia;
                     });

  logger.addLogEntry(observerName_ + "_inputs_inertiaMatrixDot",
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
    for(const auto & imu : IMUs_)
    {
      logger.addLogEntry(observerName_ + "_measurements_gyro_" + imu.name() + "_measured",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastMeasurement().segment(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name()))
                                   + observer_.sizeAcceleroSignal,
                               observer_.sizeGyroBias);
                         });
      logger.addLogEntry(observerName_ + "_measurements_gyro_" + imu.name() + "_predicted",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastPredictedMeasurement().segment(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name()))
                                   + observer_.sizeAcceleroSignal,
                               observer_.sizeGyroBias);
                         });
      logger.addLogEntry(observerName_ + "_measurements_gyro_" + imu.name() + "_corrected",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return correctedMeasurements_.segment(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name()))
                                   + observer_.sizeAcceleroSignal,
                               observer_.sizeGyroBias);
                         });

      logger.addLogEntry(observerName_ + "_measurements_accelerometer_" + imu.name() + "_measured",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastMeasurement().segment(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())),
                               observer_.sizeAcceleroSignal);
                         });
      logger.addLogEntry(observerName_ + "_measurements_accelerometer_" + imu.name() + "_predicted",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastPredictedMeasurement().segment(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())),
                               observer_.sizeAcceleroSignal);
                         });
      logger.addLogEntry(observerName_ + "_measurements_accelerometer_" + imu.name() + "_corrected",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return correctedMeasurements_.segment(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())),
                               observer_.sizeAcceleroSignal);
                         });
    }

    logger.addLogEntry(observerName_ + "_measurements_absoluteOri_measured",
                       [this]() -> Eigen::Quaterniond
                       {
                         so::kine::Orientation ori;
                         ori.fromVector4(observer_.getEKF().getLastMeasurement().tail(4));

                         return ori.toQuaternion().inverse();
                       });
    logger.addLogEntry(observerName_ + "_measurements_absoluteOri_corrected",
                       [this]() -> Eigen::Quaterniond
                       {
                         so::kine::Orientation ori;
                         ori.fromVector4(correctedMeasurements_.tail(4));

                         return ori.toQuaternion().inverse();
                       });
    logger.addLogEntry(observerName_ + "_measurements_absoluteOri_predicted",
                       [this]() -> Eigen::Quaterniond
                       {
                         so::kine::Orientation ori;
                         ori.fromVector4(observer_.getEKF().getLastPredictedMeasurement().tail(4));

                         return ori.toQuaternion().inverse();
                       });
  }

  /* Plots of the innovation */
  logger.addLogEntry(
      observerName_ + "_innovation_positionW_",
      [this]() -> Eigen::Vector3d
      { return observer_.getEKF().getInnovation().segment(observer_.posIndexTangent(), observer_.sizePosTangent); });
  logger.addLogEntry(observerName_ + "_innovation_linVelW_",
                     [this]() -> Eigen::Vector3d {
                       return observer_.getEKF().getInnovation().segment(observer_.linVelIndexTangent(),
                                                                         observer_.sizeLinVelTangent);
                     });
  logger.addLogEntry(
      observerName_ + "_innovation_oriW_",
      [this]() -> Eigen::Vector3d
      { return observer_.getEKF().getInnovation().segment(observer_.oriIndexTangent(), observer_.sizeOriTangent); });
  logger.addLogEntry(observerName_ + "_innovation_angVelW_",
                     [this]() -> Eigen::Vector3d {
                       return observer_.getEKF().getInnovation().segment(observer_.angVelIndexTangent(),
                                                                         observer_.sizeAngVelTangent);
                     });
  for(const auto & imu : IMUs_)
  {
    logger.addLogEntry(observerName_ + "_innovation_gyroBias_" + imu.name(),
                       [this, imu]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getInnovation().segment(
                             observer_.gyroBiasIndexTangent(mapIMUs_.getNumFromName(imu.name())),
                             observer_.sizeGyroBias);
                       });
  }
  logger.addLogEntry(observerName_ + "_innovation_unmodeledForce_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment(observer_.unmodeledForceIndexTangent(),
                                                                         observer_.sizeForceTangent);
                     });
  logger.addLogEntry(observerName_ + "_innovation_unmodeledTorque_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment(observer_.unmodeledTorqueIndexTangent(),
                                                                         observer_.sizeTorqueTangent);
                     });

  logger.addLogEntry(observerName_ + "_debug_worldInputRobotKine_position",
                     [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").posW().translation(); });
  logger.addLogEntry(observerName_ + "_debug_worldInputRobotKine_orientation",
                     [this]() -> Eigen::Quaternion<double>
                     {
                       return so::kine::Orientation(so::Matrix3(my_robots_->robot("inputRobot").posW().rotation()))
                           .inverse()
                           .toQuaternion();
                     });
  logger.addLogEntry(observerName_ + "_debug_worldInputRobotKine_linVel",
                     [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").velW().linear(); });
  logger.addLogEntry(observerName_ + "_debug_worldInputRobotKine_angVel",
                     [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").velW().angular(); });
  logger.addLogEntry(observerName_ + "_debug_worldInputRobotKine_linAcc",
                     [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").accW().linear(); });
  logger.addLogEntry(observerName_ + "_debug_worldInputRobotKine_angAcc",
                     [this]() -> Eigen::Vector3d { return my_robots_->robot("inputRobot").accW().angular(); });

  for(auto & contactWithSensor : contactsManager_.contactsWithSensors())
  {
    const measurements::ContactWithSensor & contact = contactWithSensor.second;
    logger.addLogEntry(observerName_ + "_debug_wrenchesInCentroid_" + contact.getName() + "_force",
                       [this, contact]() -> Eigen::Vector3d { return contact.wrenchInCentroid_.segment<3>(0); });
    logger.addLogEntry(observerName_ + "_debug_wrenchesInCentroid_" + contact.getName() + "_torque",
                       [this, contact]() -> Eigen::Vector3d { return contact.wrenchInCentroid_.segment<3>(3); });
    logger.addLogEntry(observerName_ + "_debug_wrenchesInCentroid_" + contact.getName() + "_forceWithUnmodeled",
                       [this, contact]() -> Eigen::Vector3d
                       {
                         return observer_.getCurrentStateVector().segment(observer_.unmodeledForceIndex(),
                                                                          observer_.sizeForce)
                                + contact.wrenchInCentroid_.segment<3>(0);
                       });
    logger.addLogEntry(observerName_ + "_debug_wrenchesInCentroid_" + contact.getName() + "_torqueWithUnmodeled",
                       [this, contact]() -> Eigen::Vector3d
                       {
                         return observer_.getCurrentStateVector().segment(observer_.unmodeledTorqueIndex(),
                                                                          observer_.sizeTorque)
                                + contact.wrenchInCentroid_.segment<3>(3);
                       });
  }

  for(const auto & imu : IMUs_)
  {
    logger.addLogEntry(observerName_ + "_debug_gyroBias_" + imu.name(),
                       [this, imu]() -> Eigen::Vector3d { return mapIMUs_(imu.name()).gyroBias; });
  }
}

void MCKineticsObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_posW");
  logger.removeLogEntry(category + "_velW");
  logger.removeLogEntry(category + "_mass");
  logger.removeLogEntry(category + "_flexStiffness");
  logger.removeLogEntry(category + "_flexDamping");
}

void MCKineticsObserver::changeOdometryType(const mc_control::MCController & ctl, const std::string & newOdometryType)
{
  OdometryType prevOdometryType = odometryType_;
  if(newOdometryType == "flatOdometry") { odometryType_ = measurements::flatOdometry; }
  else if(newOdometryType == "6dOdometry") { odometryType_ = measurements::odometry6d; }

  // if the type didn't change, we stop the function here
  if(odometryType_ == prevOdometryType) { return; }

  mc_rtc::log::info("[{}]: Odometry mode changed to: {}", observerName_, newOdometryType);

  // if the Tilt Observer is used as a backup, its odometry must also be changed
  if(ctl.datastore().has("changeTiltOdometryType"))
  {
    ctl.datastore().call<>("changeTiltOdometryType", newOdometryType);
  }
}

void MCKineticsObserver::addToGUI(const mc_control::MCController & ctl,
                                  mc_rtc::gui::StateBuilder & gui,
                                  const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;
  // clang-format off
  gui.addElement(category,
    mc_state_observation::gui::make_input_element("Accel Covariance", acceleroSensorCovariance_(0,0)),
    mc_state_observation::gui::make_input_element("Force Covariance", contactSensorCovariance_(0,0)),
    mc_state_observation::gui::make_input_element("Gyro Covariance", gyroSensorCovariance_(0,0)));

  if(odometryType_ != measurements::None)
  {
    gui.addElement({observerName_, "Odometry"}, mc_rtc::gui::ComboInput(
                                                                  "Choose from list", {"6dOdometry", "flatOdometry"},
                                                                  [this]() -> std::string {
                                                                    if(odometryType_ == measurements::flatOdometry)
                                                                    {
                                                                      return "flatOdometry";
                                                                    }
                                                                    else
                                                                    {
                                                                      return "6dOdometry";
                                                                    }
                                                                  },
                                                                  [this, &ctl](const std::string & typeOfOdometry) {
                                                                    changeOdometryType(ctl, typeOfOdometry);
                                                                  }));
  }
  // clang-format on
}

void MCKineticsObserver::addContactLogEntries(mc_rtc::Logger & logger, const int & contactIndex)
{
  const std::string & contactName = contactsManager_.mapContacts_.getNameFromNum(contactIndex);
  if(observer_.getContactIsSetByNum(contactIndex))
  {
    logger.addLogEntry(observerName_ + "_globalWorldCentroidState_contact_" + contactName + "_position",
                       [this, contactIndex]() -> Eigen::Vector3d {
                         return observer_.getCurrentStateVector().segment(observer_.contactPosIndex(contactIndex),
                                                                          observer_.sizePos);
                       });
    logger.addLogEntry(observerName_ + "_globalWorldCentroidState_contact_" + contactName + "_orientation",
                       [this, contactIndex]() -> Eigen::Quaternion<double>
                       {
                         so::kine::Orientation ori;
                         return ori
                             .fromVector4(observer_.getCurrentStateVector().segment(
                                 observer_.contactOriIndex(contactIndex), observer_.sizeOri))
                             .inverse()
                             .toQuaternion();
                       });
    logger.addLogEntry(observerName_ + "_globalWorldCentroidState_contact_" + contactName + "_orientation_RollPitchYaw",
                       [this, contactIndex]() -> so::Vector3
                       {
                         so::kine::Orientation ori;
                         return so::kine::rotationMatrixToRollPitchYaw(
                             ori.fromVector4(observer_.getCurrentStateVector().segment(
                                                 observer_.contactOriIndex(contactIndex), observer_.sizeOri))
                                 .toMatrix3());
                       });
    logger.addLogEntry(observerName_ + "_globalWorldCentroidState_contact_" + contactName + "_forces",
                       [this, contactIndex]() -> Eigen::Vector3d {
                         return observer_.getCurrentStateVector().segment(observer_.contactForceIndex(contactIndex),
                                                                          observer_.sizeForce);
                       });
    logger.addLogEntry(observerName_ + "_globalWorldCentroidState_contact_" + contactName + "_torques",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return globalCentroidKinematics_.orientation.toMatrix3()
                                * observer_.getCurrentStateVector().segment(observer_.contactTorqueIndex(contactIndex),
                                                                            observer_.sizeTorque);
                       });
    logger.addLogEntry(observerName_ + "_stateCovariances_contact_" + contactName + "_position_",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF()
                             .getStateCovariance()
                             .block(observer_.contactPosIndexTangent(contactIndex),
                                    observer_.contactPosIndexTangent(contactIndex), observer_.sizePosTangent,
                                    observer_.sizePosTangent)
                             .diagonal();
                       });
    logger.addLogEntry(observerName_ + "_stateCovariances_contact_" + contactName + "_orientation_",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF()
                             .getStateCovariance()
                             .block(observer_.contactOriIndexTangent(contactIndex),
                                    observer_.contactOriIndexTangent(contactIndex), observer_.sizeOriTangent,
                                    observer_.sizeOriTangent)
                             .diagonal();
                       });
    logger.addLogEntry(observerName_ + "_stateCovariances_contact_" + contactName + "_Force_",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF()
                             .getStateCovariance()
                             .block(observer_.contactForceIndexTangent(contactIndex),
                                    observer_.contactForceIndexTangent(contactIndex), observer_.sizeForceTangent,
                                    observer_.sizeForceTangent)
                             .diagonal();
                       });
    logger.addLogEntry(observerName_ + "_stateCovariances_contact_" + contactName + "_Torque_",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF()
                             .getStateCovariance()
                             .block(observer_.contactTorqueIndexTangent(contactIndex),
                                    observer_.contactTorqueIndexTangent(contactIndex), observer_.sizeTorqueTangent,
                                    observer_.sizeTorqueTangent)
                             .diagonal();
                       });

    logger.addLogEntry(observerName_ + "_innovation_contact_" + contactName + "_position",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getInnovation().segment(
                             observer_.contactPosIndexTangent(contactIndex), observer_.sizePos);
                       });
    logger.addLogEntry(observerName_ + "_innovation_contact_" + contactName + "_orientation",
                       [this, contactIndex]() -> Eigen::Quaternion<double>
                       {
                         so::kine::Orientation ori;
                         return ori
                             .fromVector4(observer_.getEKF().getInnovation().segment(
                                 observer_.contactOriIndexTangent(contactIndex), observer_.sizeOri))
                             .inverse()
                             .toQuaternion();
                       });
    logger.addLogEntry(observerName_ + "_innovation_contact_" + contactName + "_forces",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getInnovation().segment(
                             observer_.contactForceIndexTangent(contactIndex), observer_.sizeForce);
                       });
    logger.addLogEntry(observerName_ + "_innovation_contact_" + contactName + "_torques",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getInnovation().segment(
                             observer_.contactTorqueIndexTangent(contactIndex), observer_.sizeTorque);
                       });

    logger.addLogEntry(observerName_ + "_debug_contactWrench_Centroid_" + contactName + "_force",
                       [this, contactIndex]() -> Eigen::Vector3d
                       { return observer_.getCentroidContactWrench(contactIndex).segment(0, observer_.sizeForce); });

    logger.addLogEntry(observerName_ + "_debug_contactWrench_Centroid_" + contactName + "_torque",
                       [this, contactIndex]() -> Eigen::Vector3d
                       { return observer_.getCentroidContactWrench(contactIndex).segment(3, observer_.sizeTorque); });

    logger.addLogEntry(observerName_ + "_debug_contactPose_" + contactName + "_inputCentroidContactKine_position",
                       [this, contactIndex]() -> Eigen::Vector3d
                       { return observer_.getCentroidContactInputPose(contactIndex).position(); });

    logger.addLogEntry(
        observerName_ + "_debug_contactPose_" + contactName + "_inputCentroidContactKine_orientation",
        [this, contactIndex]() -> Eigen::Quaternion<double>
        { return observer_.getCentroidContactInputPose(contactIndex).orientation.inverse().toQuaternion(); });

    logger.addLogEntry(observerName_ + "_debug_contactPose_" + contactName + "_worldContactPoseFromCentroid_position",
                       [this, contactIndex]() -> Eigen::Vector3d
                       { return observer_.getWorldContactPoseFromCentroid(contactIndex).position(); });

    logger.addLogEntry(
        observerName_ + "_debug_contactPose_" + contactName + "_worldContactPoseFromCentroid_orientation",
        [this, contactIndex]() -> Eigen::Quaternion<double>
        { return observer_.getWorldContactPoseFromCentroid(contactIndex).orientation.inverse().toQuaternion(); });

    logger.addLogEntry(observerName_ + "_debug_contactPose_" + contactName + "_inputUserContactKine_position",
                       [this, contactIndex]() -> Eigen::Vector3d
                       { return observer_.getUserContactInputPose(contactIndex).position(); });

    logger.addLogEntry(observerName_ + "_debug_contactPose_" + contactName + "_inputUserContactKine_orientation",
                       [this, contactIndex]() -> Eigen::Quaternion<double> {
                         return observer_.getUserContactInputPose(contactIndex).orientation.inverse().toQuaternion();
                       });
    logger.addLogEntry(observerName_ + "_debug_contactState_isSet_" + contactName,
                       [this, contactIndex]() -> int
                       { return int(contactsManager_.contactWithSensor(contactIndex).isSet_); });
  }
}

void MCKineticsObserver::addContactMeasurementsLogEntries(mc_rtc::Logger & logger, const int & contactIndex)
{
  const std::string & contactName = contactsManager_.mapContacts_.getNameFromNum(contactIndex);
  if(observer_.getContactIsSetByNum(contactIndex))
  {
    logger.addLogEntry(observerName_ + "_measurements_contacts_force_" + contactName + "_measured",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getLastMeasurement().segment(
                             observer_.getContactMeasIndexByNum(contactIndex), observer_.sizeForce);
                       });
    logger.addLogEntry(observerName_ + "_measurements_contacts_force_" + contactName + "_predicted",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getLastPredictedMeasurement().segment(
                             observer_.getContactMeasIndexByNum(contactIndex), observer_.sizeForce);
                       });
    logger.addLogEntry(observerName_ + "_measurements_contacts_force_" + contactName + "_corrected",
                       [this, contactIndex]() -> Eigen::Vector3d {
                         return correctedMeasurements_.segment(observer_.getContactMeasIndexByNum(contactIndex),
                                                               observer_.sizeForce);
                       });

    logger.addLogEntry(observerName_ + "_measurements_contacts_torque_" + contactName + "_measured",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getLastMeasurement().segment(
                             observer_.getContactMeasIndexByNum(contactIndex) + observer_.sizeForce,
                             observer_.sizeTorque);
                       });
    logger.addLogEntry(observerName_ + "_measurements_contacts_torque_" + contactName + "_predicted",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getLastPredictedMeasurement().segment(
                             observer_.getContactMeasIndexByNum(contactIndex) + observer_.sizeForce,
                             observer_.sizeTorque);
                       });
    logger.addLogEntry(observerName_ + "_measurements_contacts_torque_" + contactName + "_corrected",
                       [this, contactIndex]() -> Eigen::Vector3d
                       {
                         return correctedMeasurements_.segment(observer_.getContactMeasIndexByNum(contactIndex)
                                                                   + observer_.sizeForce,
                                                               observer_.sizeTorque);
                       });
  }
}

void MCKineticsObserver::removeContactLogEntries(mc_rtc::Logger & logger, const int & contactIndex)
{
  const std::string & contactName = contactsManager_.mapContacts_.getNameFromNum(contactIndex);
  logger.removeLogEntry(observerName_ + "_globalWorldCentroidState_contact_" + contactName + "_position");
  logger.removeLogEntry(observerName_ + "_globalWorldCentroidState_contact_" + contactName + "_position");
  logger.removeLogEntry(observerName_ + "_globalWorldCentroidState_contact_" + contactName + "_orientation");
  logger.removeLogEntry(observerName_ + "_globalWorldCentroidState_contact_" + contactName
                        + "_orientation_RollPitchYaw");
  logger.removeLogEntry(observerName_ + "_globalWorldCentroidState_contact_" + contactName + "_forces");
  logger.removeLogEntry(observerName_ + "_globalWorldCentroidState_contact_" + contactName + "_torques");
  logger.removeLogEntry(observerName_ + "_stateCovariances_contact_" + contactName + "_position_");
  logger.removeLogEntry(observerName_ + "_stateCovariances_contact_" + contactName + "_orientation_");
  logger.removeLogEntry(observerName_ + "_stateCovariances_contact_" + contactName + "_Force_");
  logger.removeLogEntry(observerName_ + "_stateCovariances_contact_" + contactName + "_Torque_");

  logger.removeLogEntry(observerName_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_position");
  logger.removeLogEntry(observerName_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_orientation");
  logger.removeLogEntry(observerName_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_forces");
  logger.removeLogEntry(observerName_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_torques");

  logger.removeLogEntry(observerName_ + "_innovation_contact_" + contactName + "_position");
  logger.removeLogEntry(observerName_ + "_innovation_contact_" + contactName + "_orientation");
  logger.removeLogEntry(observerName_ + "_innovation_contact_" + contactName + "_forces");
  logger.removeLogEntry(observerName_ + "_innovation_contact_" + contactName + "_torques");

  logger.removeLogEntry(observerName_ + "_debug_contactWrench_World_" + contactName + "_force");

  logger.removeLogEntry(observerName_ + "_debug_contactWrench_World_" + contactName + "_torque");

  logger.removeLogEntry(observerName_ + "_debug_contactWrench_Centroid_" + contactName + "_force");

  logger.removeLogEntry(observerName_ + "_debug_contactWrench_Centroid_" + contactName + "_torque");

  logger.removeLogEntry(observerName_ + "_debug_contactPose_" + contactName + "_inputWorldRef_position");

  logger.removeLogEntry(observerName_ + "_debug_contactPose_" + contactName + "_inputWorldRef_orientation");

  logger.removeLogEntry(observerName_ + "_debug_contactPose_" + contactName + "_inputCentroidContactKine_position");

  logger.removeLogEntry(observerName_ + "_debug_contactPose_" + contactName + "_inputCentroidContactKine_orientation");

  logger.removeLogEntry(observerName_ + "_debug_contactPose_" + contactName + "_worldContactPoseFromCentroid_position");

  logger.removeLogEntry(observerName_ + "_debug_contactPose_" + contactName
                        + "_worldContactPoseFromCentroid_orientation");

  logger.removeLogEntry(observerName_ + "_debug_contactPose_" + contactName + "_inputUserContactKine_position");

  logger.removeLogEntry(observerName_ + "_debug_contactPose_" + contactName + "_inputUserContactKine_orientation");
  logger.removeLogEntry(observerName_ + "_debug_contactState_isSet_" + contactName);
  // logger.removeLogEntry(observerName_ + "_debug_zmp_" + contactName);
}

void MCKineticsObserver::removeContactMeasurementsLogEntries(mc_rtc::Logger & logger, const int & contactIndex)
{
  const std::string & contactName = contactsManager_.mapContacts_.getNameFromNum(contactIndex);
  logger.removeLogEntry(observerName_ + "_measurements_contacts_force_" + contactName + "_measured");
  logger.removeLogEntry(observerName_ + "_measurements_contacts_force_" + contactName + "_predicted");
  logger.removeLogEntry(observerName_ + "_measurements_contacts_force_" + contactName + "_corrected");

  logger.removeLogEntry(observerName_ + "_measurements_contacts_torque_" + contactName + "_measured");
  logger.removeLogEntry(observerName_ + "_measurements_contacts_torque_" + contactName + "_predicted");
  logger.removeLogEntry(observerName_ + "_measurements_contacts_torque_" + contactName + "_corrected");
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("MCKineticsObserver", mc_state_observation::MCKineticsObserver)
