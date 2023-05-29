/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>
#include <mc_state_observation/MCKineticsObserver.h>
#include <mc_state_observation/gui_helpers.h>

#include <RBDyn/CoM.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <iostream>

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
  IMUs_ = config("imuSensor", ctl.robot().bodySensors());
  config("debug", debug_);
  config("verbose", verbose_);

  config("gyroBiasStandardDeviation", gyroBiasStandardDeviation_);

  config("contactsSensorDisabledInit", contactsSensorDisabledInit_);

  config("leftHandDetection", leftHandDetection_);
  if(leftHandDetection_ != "None" && leftHandDetection_ != "asContact" && leftHandDetection_ != "asExternalWrench")
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Odometry type not allowed. Please pick among : [None, asContact, asExternalWrench]");
  }

  std::string odometryType = static_cast<std::string>(config("odometryType"));

  if(odometryType != "None")
  {
    if(odometryType == "flatOdometry")
    {
      withOdometry_ = true;
      withFlatOdometry_ = true;
    }
    else if(odometryType == "6dOdometry")
    {
      withOdometry_ = true;
    }
    else
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Odometry type not allowed. Please pick among : [None, flatOdometry, 6dOdometry]");
    }
  }

  contactsDetection_ = static_cast<std::string>(config("contactsDetection"));
  if(contactsDetection_ != "fromSolver" && contactsDetection_ != "fromThreshold"
     && contactsDetection_ != "fromSurfaces")
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Contacts detection type not allowed. Please pick among : [fromSolver, fromThreshold, fromSurfaces] or "
        "initialize a list of surfaces with the variable surfacesForContactDetection");
  }
  config("surfacesForContactDetection", surfacesForContactDetection_);
  if(surfacesForContactDetection_.size() > 0)
  {
    if(contactsDetection_ != "fromSurfaces")
    {
      mc_rtc::log::error_and_throw<std::runtime_error>(
          "Another type of contacts detection is currently used, please change it to 'fromSurfaces' or empty the "
          "surfacesForContactDetection variable");
    }
  }
  else if(contactsDetection_ == "fromSurfaces")
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "You selected the contacts detection using surfaces but didn't add the list of surfaces, please add it usign "
        "the variable surfacesForContactDetection");
  }

  config("withDebugLogs", withDebugLogs_);
  config("contactDetectionPropThreshold", contactDetectionPropThreshold_);
  config("withFilteredForcesContactDetection", withFilteredForcesContactDetection_);
  config("withUnmodeledWrench", withUnmodeledWrench_);
  config("withGyroBias", withGyroBias_);

  observer_.setWithUnmodeledWrench(withUnmodeledWrench_);
  observer_.setWithGyroBias(withGyroBias_);
  observer_.useFiniteDifferencesJacobians(config("withFiniteDifferences"));
  so::Vector dx(observer_.getStateSize());
  dx.setConstant(static_cast<double>(config("finiteDifferenceStep")));
  observer_.setFiniteDifferenceStep(dx);
  observer_.setWithAccelerationEstimation(config("withAccelerationEstimation"));
  observer_.useRungeKutta(config("withRungeKutta"));

  linStiffness_ = static_cast<so::Vector3>(config("linStiffness")).matrix().asDiagonal();
  angStiffness_ = static_cast<so::Vector3>(config("angStiffness")).matrix().asDiagonal();
  linDamping_ = static_cast<so::Vector3>(config("linDamping")).matrix().asDiagonal();
  angDamping_ = static_cast<so::Vector3>(config("angDamping")).matrix().asDiagonal();

  zeroPose_.translation().setZero();
  zeroPose_.rotation().setIdentity();
  zeroMotion_.linear().setZero();
  zeroMotion_.angular().setZero();

  // Initial State
  statePositionInitCovariance_ = static_cast<so::Vector3>(config("statePositionInitVariance")).matrix().asDiagonal();
  stateOriInitCovariance_ = static_cast<so::Vector3>(config("stateOriInitVariance")).matrix().asDiagonal();
  stateLinVelInitCovariance_ = static_cast<so::Vector3>(config("stateLinVelInitVariance")).matrix().asDiagonal();
  stateAngVelInitCovariance_ = static_cast<so::Vector3>(config("stateAngVelInitVariance")).matrix().asDiagonal();
  gyroBiasInitCovariance_.setZero();
  unmodeledWrenchInitCovariance_.setZero();
  contactInitCovarianceFirstContacts_.setZero();
  contactInitCovarianceFirstContacts_.block<3, 3>(0, 0) =
      static_cast<so::Vector3>(config("contactPositionInitVarianceFirstContacts")).matrix().asDiagonal();
  contactInitCovarianceFirstContacts_.block<3, 3>(3, 3) =
      static_cast<so::Vector3>(config("contactOriInitVarianceFirstContacts")).matrix().asDiagonal();
  contactInitCovarianceFirstContacts_.block<3, 3>(6, 6) =
      static_cast<so::Vector3>(config("contactForceInitVarianceFirstContacts")).matrix().asDiagonal();
  contactInitCovarianceFirstContacts_.block<3, 3>(9, 9) =
      static_cast<so::Vector3>(config("contactTorqueInitVarianceFirstContacts")).matrix().asDiagonal();

  contactInitCovarianceNewContacts_.setZero();
  contactInitCovarianceNewContacts_.block<3, 3>(0, 0) =
      static_cast<so::Vector3>(config("contactPositionInitVarianceNewContacts")).matrix().asDiagonal();
  contactInitCovarianceNewContacts_.block<3, 3>(3, 3) =
      static_cast<so::Vector3>(config("contactOriInitVarianceNewContacts")).matrix().asDiagonal();
  contactInitCovarianceNewContacts_.block<3, 3>(6, 6) =
      static_cast<so::Vector3>(config("contactForceInitVarianceNewContacts")).matrix().asDiagonal();
  contactInitCovarianceNewContacts_.block<3, 3>(9, 9) =
      static_cast<so::Vector3>(config("contactTorqueInitVarianceNewContacts")).matrix().asDiagonal();

  // Process //
  statePositionProcessCovariance_ =
      static_cast<so::Vector3>(config("statePositionProcessVariance")).matrix().asDiagonal();
  stateOriProcessCovariance_ = static_cast<so::Vector3>(config("stateOriProcessVariance")).matrix().asDiagonal();
  stateLinVelProcessCovariance_ = static_cast<so::Vector3>(config("stateLinVelProcessVariance")).matrix().asDiagonal();
  stateAngVelProcessCovariance_ = static_cast<so::Vector3>(config("stateAngVelProcessVariance")).matrix().asDiagonal();
  gyroBiasProcessCovariance_.setZero();
  unmodeledWrenchProcessCovariance_.setZero();

  contactProcessCovariance_.setZero();
  contactProcessCovariance_.block<3, 3>(0, 0) =
      static_cast<so::Vector3>(config("contactPositionProcessVariance")).matrix().asDiagonal();
  contactProcessCovariance_.block<3, 3>(3, 3) =
      static_cast<so::Vector3>(config("contactOrientationProcessVariance")).matrix().asDiagonal();
  contactProcessCovariance_.block<3, 3>(6, 6) =
      static_cast<so::Vector3>(config("contactForceProcessVariance")).matrix().asDiagonal();
  contactProcessCovariance_.block<3, 3>(9, 9) =
      static_cast<so::Vector3>(config("contactTorqueProcessVariance")).matrix().asDiagonal();

  // Unmodeled Wrench //
  if(withUnmodeledWrench_)
  {
    // initial
    unmodeledWrenchInitCovariance_.block<3, 3>(0, 0) =
        static_cast<so::Vector3>(config("unmodeledForceInitVariance")).matrix().asDiagonal();
    unmodeledWrenchInitCovariance_.block<3, 3>(3, 3) =
        static_cast<so::Vector3>(config("unmodeledTorqueInitVariance")).matrix().asDiagonal();

    // process
    unmodeledWrenchProcessCovariance_.block<3, 3>(0, 0) =
        static_cast<so::Vector3>(config("unmodeledForceProcessVariance")).matrix().asDiagonal();
    unmodeledWrenchProcessCovariance_.block<3, 3>(3, 3) =
        static_cast<so::Vector3>(config("unmodeledTorqueProcessVariance")).matrix().asDiagonal();
  }
  // Gyrometer Bias
  if(withGyroBias_)
  {
    gyroBiasInitCovariance_ = static_cast<so::Vector3>(config("gyroBiasInitVariance")).matrix().asDiagonal();
    gyroBiasProcessCovariance_ = static_cast<so::Vector3>(config("gyroBiasProcessVariance")).matrix().asDiagonal();
  }

  // Sensor //
  positionSensorCovariance_ = static_cast<so::Vector3>(config("positionSensorVariance")).matrix().asDiagonal();
  orientationSensorCoVariance_ = static_cast<so::Vector3>(config("orientationSensorVariance")).matrix().asDiagonal();
  acceleroSensorCovariance_ = static_cast<so::Vector3>(config("acceleroSensorVariance")).matrix().asDiagonal();
  gyroSensorCovariance_ = static_cast<so::Vector3>(config("gyroSensorVariance")).matrix().asDiagonal();
  contactSensorCovariance_.setZero();
  contactSensorCovariance_.block<3, 3>(0, 0) =
      static_cast<so::Vector3>(config("forceSensorVariance")).matrix().asDiagonal();
  contactSensorCovariance_.block<3, 3>(3, 3) =
      static_cast<so::Vector3>(config("torqueSensorVariance")).matrix().asDiagonal();

  observer_.setAllCovariances(statePositionInitCovariance_, stateOriInitCovariance_, stateLinVelInitCovariance_,
                              stateAngVelInitCovariance_, gyroBiasInitCovariance_, unmodeledWrenchInitCovariance_,
                              contactInitCovarianceFirstContacts_, statePositionProcessCovariance_,
                              stateOriProcessCovariance_, stateLinVelProcessCovariance_, stateAngVelProcessCovariance_,
                              gyroBiasProcessCovariance_, unmodeledWrenchProcessCovariance_, contactProcessCovariance_,
                              positionSensorCovariance_, orientationSensorCoVariance_, acceleroSensorCovariance_,
                              gyroSensorCovariance_, contactSensorCovariance_);
}

void MCKineticsObserver::reset(const mc_control::MCController & ctl)
{
  simStarted_ = false;
  ekfIsSet_ = false;

  // mapIMUs_.setMaxElements(2);
  // mapContacts_.setMaxElements(4);

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
    if(realRobot.mb().predecessor(i) == 0)
    {
      rootJoints.push_back(realRobot.mb().joint(i).name());
    }
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

  contactDetectionThreshold_ = robot.mass() * so::cst::gravityConstant * contactDetectionPropThreshold_;

  for(const auto & imu : IMUs_)
  {
    mapIMUs_.insertIMU(imu.name());
  }

  if(contactsDetection_ == "fromThreshold")
  {
    for(auto forceSensor : realRobot.forceSensors())
    {
      const std::string & fsName = forceSensor.name();

      mapContacts_.insertContact(forceSensor.name(), true);
      ctl.gui()->addElement(
          {"MCKineticsObserver", "Contacts"},
          mc_rtc::gui::Checkbox(
              fsName + " : " + (mapContacts_.contactWithSensor(fsName).isSet ? "Contact is set" : "Contact is not set")
                  + ": Use wrench sensor: ",
              [this, fsName]() { return mapContacts_.contactWithSensor(fsName).sensorEnabled; },
              [this, fsName]()
              {
                mapContacts_.contactWithSensor(fsName).sensorEnabled =
                    !mapContacts_.contactWithSensor(fsName).sensorEnabled;
                std::cout << std::endl
                          << "Enable / disable :" + fsName + " "
                                 + std::to_string(mapContacts_.contactWithSensor(fsName).sensorEnabled)
                          << std::endl;
              }));
    }
  }
  if(contactsDetection_ == "fromSurfaces")
  {
    for(const std::string & surface : surfacesForContactDetection_)
    {
      if(robot.surfaceHasForceSensor(surface))
      {
        const mc_rbdyn::ForceSensor & forceSensor = robot.surfaceForceSensor(surface);
        const std::string & fsName = forceSensor.name();
        mapContacts_.insertContact(forceSensor.name(), true);
        mapContacts_.contactWithSensor(fsName).surface = surface;
        ctl.gui()->addElement(
            {"MCKineticsObserver", "Contacts"},
            mc_rtc::gui::Checkbox(
                fsName + " : "
                    + (mapContacts_.contactWithSensor(fsName).isSet ? "Contact is set" : "Contact is not set")
                    + ": Use wrench sensor: ",
                [this, fsName]() { return mapContacts_.contactWithSensor(fsName).sensorEnabled; },
                [this, fsName]()
                {
                  mapContacts_.contactWithSensor(fsName).sensorEnabled =
                      !mapContacts_.contactWithSensor(fsName).sensorEnabled;
                  std::cout << std::endl
                            << "Enable / disable :" + fsName + " "
                                   + std::to_string(mapContacts_.contactWithSensor(fsName).sensorEnabled)
                            << std::endl;
                }));
      }
      else
      {
        const mc_rbdyn::ForceSensor & forceSensor = robot.indirectSurfaceForceSensor(surface);
        const std::string & fsName = forceSensor.name();

        mapContacts_.insertContact(forceSensor.name(), true);
        mapContacts_.contactWithSensor(fsName).isAttachedToSurface = false;
        mapContacts_.contactWithSensor(fsName).surface = surface;
        ctl.gui()->addElement(
            {"MCKineticsObserver", "Contacts"},
            mc_rtc::gui::Checkbox(
                fsName + " : "
                    + (mapContacts_.contactWithSensor(fsName).isSet ? "Contact is set" : "Contact is not set")
                    + ": Use wrench sensor: ",
                [this, fsName]() { return mapContacts_.contactWithSensor(fsName).sensorEnabled; },
                [this, fsName]()
                {
                  mapContacts_.contactWithSensor(fsName).sensorEnabled =
                      !mapContacts_.contactWithSensor(fsName).sensorEnabled;
                  std::cout << std::endl
                            << "Enable / disable :" + fsName + " "
                                   + std::to_string(mapContacts_.contactWithSensor(fsName).sensorEnabled)
                            << std::endl;
                }));
      }
    }
  }

  for(auto const & contactSensorDisabledInit : contactsSensorDisabledInit_)
  {
    BOOST_ASSERT(mapContacts_.hasElement(contactSensorDisabledInit) && "This sensor is not attached to the robot");
    mapContacts_.contactWithSensor(contactSensorDisabledInit).sensorEnabled = false;
  }

  if(debug_)
  {
    mc_rtc::log::info("inertiaWaist = {}", inertiaWaist_);
  }

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  my_robots_->robotCopy(realRobot, "inputRobot");
  ctl.gui()->addElement(
      {"Robots"},
      mc_rtc::gui::Robot("MCKineticsobserver", [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));
  ctl.gui()->addElement({"Robots"},
                        mc_rtc::gui::Robot("Real", [&ctl]() -> const mc_rbdyn::Robot & { return ctl.realRobot(); }));

  X_0_fb_ = robot.posW().translation();
}

bool MCKineticsObserver::run(const mc_control::MCController & ctl)
{
  // std::cout << "\033[1;31m" << std::endl << "New iteration: " << std::endl << "\033[0m\n";

  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  auto & inputRobot = my_robots_->robot("inputRobot");
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  inputRobot.mbc() = realRobot.mbc();
  inputRobot.mb() = realRobot.mb();

  inputRobot.posW(zeroPose_);
  inputRobot.velW(zeroMotion_);
  inputRobot.accW(zeroMotion_);

  /** Center of mass (assumes FK, FV and FA are already done)
      Must be initialized now as used for the conversion from user to centroid frame !!! **/

  worldCoMKine_.position = inputRobot.com();
  worldCoMKine_.linVel = inputRobot.comVelocity();
  worldCoMKine_.linAcc = inputRobot.comAcceleration();

  observer_.setCenterOfMass(worldCoMKine_.position(), worldCoMKine_.linVel(), worldCoMKine_.linAcc());

  findContacts(ctl); // retrieves the list of contacts and set simStarted to true once a contact is detected

  if(!simStarted_) // this allows to ignore the first step on which the contacts are still not detected
                   // and to avoid the jump of the com's position between this step and the following one
  {
    return true;
  }

  /** Contacts
   * Note that when we use force sensors, this should be the position of the force sensor!
   */
  updateContacts(ctl, contactsFound_, logger);

  // std::cout << std::endl << "Time: " << std::endl << observer_.getEKF().getCurrentTime() << std::endl;

  /** Accelerometers **/
  updateIMUs(robot, inputRobot);

  /** Inertias **/
  /** TODO : Merge inertias into CoM inertia and/or get it from fd() **/
  // Eigen::Vector6d inertia;
  // Eigen::Matrix3d inertiaAtOrigin =
  //     sva::inertiaToOrigin(inertiaWaist_.inertia(), mass_, inputRobot.com(), Eigen::Matrix3d::Identity().eval());
  // inertia << inertiaAtOrigin(0, 0), inertiaAtOrigin(1, 1), inertiaAtOrigin(2, 2), inertiaAtOrigin(0, 1),
  //     inertiaAtOrigin(0, 2), inertiaAtOrigin(1, 2);
  /*
  observer_.setCoMAngularMomentum(
      fbWorldKine.orientation.toMatrix3() * sigmaWorld,
      fbWorldKine.orientation.toMatrix3()
          * ((fbWorldKine_.orientation.toMatrix3() * fbWorldKine.angVel()).cross(sigmaWorld)
             + rbd::computeCentroidalMomentumDot(inputRobot.mb(), inputRobot.mbc(), inputRobot.com(),
  inputRobot.comVelocity()).moment()));
             */
  observer_.setCoMAngularMomentum(
      rbd::computeCentroidalMomentum(inputRobot.mb(), inputRobot.mbc(), inputRobot.com()).moment());
  /*
  std::cout << std::endl << "diff momentum: " << observer_.getAngularMomentum()() << std::endl;
  std::cout << std::endl
            << "diff momentum_d: "
            << observer_.getAngularMomentumDot()()
                   - (fbWorldKine_.orientation.toMatrix3()
                      * ((worldFbKine_.orientation.toMatrix3() * fbWorldKine_.angVel()).cross(sigmaWorld)
                         + rbd::computeCentroidalMomentumDot(inputRobot.mb(), inputRobot.mbc(), inputRobot.com(),
                                                             inputRobot.comVelocity())
                               .moment()))
            << std::endl;
  */
  observer_.setCoMInertiaMatrix(so::Matrix3(
      inertiaWaist_.inertia() + observer_.getMass() * so::kine::skewSymmetric2(observer_.getCenterOfMass()())));
  /* Step once, and return result */

  if(!ekfIsSet_ && withDebugLogs_) // the ekf is not updated, which means that it still has the initial values
  {
    plotVariablesBeforeUpdate(ctl, logger);
  }

  res_ = observer_.update();

  if(!ekfIsSet_ && withDebugLogs_)
  {
    plotVariablesAfterUpdate(ctl, logger);
  }

  ekfIsSet_ = true;

  /* Debug */
  if(withDebugLogs_)
  {
    for(auto & wrenchSensor : ctl.robot().forceSensors())
    {
      if(mapContacts_.hasElement(wrenchSensor.name()))
      {
        if(observer_.getContactIsSetByNum(mapContacts_.getNumFromName(wrenchSensor.name())))
        {
          totalForceCentroid_ += observer_.getCentroidContactWrench(mapContacts_.getNumFromName(wrenchSensor.name()))
                                     .segment<int(observer_.sizeForce)>(0);
          totalTorqueCentroid_ += observer_.getCentroidContactWrench(mapContacts_.getNumFromName(wrenchSensor.name()))
                                      .segment<int(observer_.sizeTorque)>(observer_.sizeForce);
        }
      }
    }

    totalForceCentroid_ +=
        observer_.getCurrentStateVector().segment<int(observer_.sizeForce)>(observer_.unmodeledForceIndex());
    totalTorqueCentroid_ +=
        observer_.getCurrentStateVector().segment<int(observer_.sizeTorque)>(observer_.unmodeledTorqueIndex());

    robotImuOri_0 =
        so::KineticsObserver::Orientation(
            so::Matrix3(
                robot.bodyPosW(robot.bodySensor(mapIMUs_.getNameFromNum(0)).parentBody()).rotation().transpose()))
            .inverse()
            .toQuaternion();

    robotImuOri_1 =
        so::KineticsObserver::Orientation(
            so::Matrix3(
                robot.bodyPosW(robot.bodySensor(mapIMUs_.getNameFromNum(1)).parentBody()).rotation().transpose()))
            .inverse()
            .toQuaternion();
    realRobotImuOri_0 =
        so::KineticsObserver::Orientation(
            so::Matrix3(realRobot.bodyPosW(realRobot.bodySensor(mapIMUs_.getNameFromNum(0)).parentBody())
                            .rotation()
                            .transpose()))
            .inverse()
            .toQuaternion();

    realRobotImuOri_1 =
        so::KineticsObserver::Orientation(
            so::Matrix3(realRobot.bodyPosW(realRobot.bodySensor(mapIMUs_.getNameFromNum(1)).parentBody())
                            .rotation()
                            .transpose()))
            .inverse()
            .toQuaternion();

    robotFbOri_ =
        so::KineticsObserver::Orientation(so::Matrix3(robot.posW().rotation().transpose())).inverse().toQuaternion();

    realRobotFbOri_ =
        so::KineticsObserver::Orientation(so::Matrix3(realRobot.posW().rotation().transpose())).inverse().toQuaternion();

    robotPosImuInFB_0 = robot.bodySensor(mapIMUs_.getNameFromNum(0)).X_b_s().rotation().transpose()
                        * robot.bodySensor(mapIMUs_.getNameFromNum(0)).X_b_s().translation();
    robotPosImuInFB_1 = robot.bodySensor(mapIMUs_.getNameFromNum(1)).X_b_s().rotation().transpose()
                        * robot.bodySensor(mapIMUs_.getNameFromNum(1)).X_b_s().translation();

    realRobotPosImuInFB_0 = realRobot.bodySensor(mapIMUs_.getNameFromNum(0)).X_b_s().rotation().transpose()
                            * realRobot.bodySensor(mapIMUs_.getNameFromNum(0)).X_b_s().translation();
    realRobotPosImuInFB_1 = realRobot.bodySensor(mapIMUs_.getNameFromNum(1)).X_b_s().rotation().transpose()
                            * realRobot.bodySensor(mapIMUs_.getNameFromNum(1)).X_b_s().translation();

    robotTilt_0 =
        robot.bodySensor(mapIMUs_.getNameFromNum(0)).X_b_s().rotation() * robot.posW().rotation() * so::cst::gravity;
    robotTilt_1 =
        robot.bodySensor(mapIMUs_.getNameFromNum(1)).X_b_s().rotation() * robot.posW().rotation() * so::cst::gravity;
    realRobotTilt_0 = realRobot.bodySensor(mapIMUs_.getNameFromNum(0)).X_b_s().rotation() * realRobot.posW().rotation()
                      * so::cst::gravity;
    realRobotTilt_1 = realRobot.bodySensor(mapIMUs_.getNameFromNum(1)).X_b_s().rotation() * realRobot.posW().rotation()
                      * so::cst::gravity;

    realRobot_centroidImuOri_0 =
        observer_.getGlobalCentroidKinematics().orientation.inverse()
        * so::kine::Orientation(realRobot.bodySensor(mapIMUs_.getNameFromNum(0)).orientation());

    realRobot_centroidImuOri_1 =
        observer_.getGlobalCentroidKinematics().orientation.inverse()
        * so::kine::Orientation(realRobot.bodySensor(mapIMUs_.getNameFromNum(1)).orientation());
  }

  /* Core */
  so::kine::Kinematics fbFb; // "Zero" Kinematics
  fbFb.setZero(so::kine::Kinematics::Flags::all);

  so::kine::Kinematics mcko_K_0_fb(observer_.getGlobalKinematicsOf(
      fbFb)); // Floating base in the 'real' world frame. The resulting state kinematics are used here
  X_0_fb_.rotation() = mcko_K_0_fb.orientation.toMatrix3().transpose();
  X_0_fb_.translation() = mcko_K_0_fb.position();

  if(withDebugLogs_)
  {
    /*

    for(auto & contactWithSensor : mapContacts_.contactsWithSensors())
    {
      ContactWithSensor & contact = contactWithSensor.second;
      if(observer_.getContactIsSetByNum(contact.getID()))
      {
        so::Vector3 normalVector;
        if(withFlatOdometry_)
        {
          normalVector = so::Vector3::UnitZ();
        }
        else // 6D odometry
        {
          so::kine::Orientation contactOri;
          contactOri.fromVector4(observer_.getCurrentStateVector().segment<int(observer_.sizeOri)>(
              observer_.contactOriIndex(contact.getID())));

          normalVector = contactOri.toAngleAxis().axis();
        }
        if(observer_.getContactWrench(contact.getID()).segment<int(observer_.sizeForce)>(0).norm()
           > contactDetectionThreshold_)
        {
          logger.addLogEntry(category_ + "_debug_zmp_" + contact.getName(),
                             [this, &contact]() -> so::Vector3
                             { return mapContacts_.getZMPFromName(contact.getName()); });
          if(!contact.zmp.isSet())
          {

          }
          contact.zmp =
              (normalVector.cross(
                   observer_.getContactWrench(contact.getID()).segment<int(observer_.sizeTorque)>(observer_.sizeForce)))
                  .cwiseQuotient(observer_.getContactWrench(contact.getID())
                                     .segment<int(observer_.sizeForce)>(0)
                                     .cross(normalVector));
        }
        else
        {
          contact.zmp.set(false);
          logger.removeLogEntry(category_ + "_debug_zmp_" + contact.getName());
        }
      }
    }

    for(auto & contactWithoutSensor : mapContacts_.contactsWithoutSensors())
    {
      ContactWithoutSensor & contact = contactWithoutSensor.second;
      if(observer_.getContactIsSetByNum(contact.getID()))
      {
        so::Vector3 normalVector;
        if(withFlatOdometry_)
        {
          normalVector = so::Vector3::UnitZ();
        }
        else // 6D odometry
        {
          so::kine::Orientation contactOri;
          contactOri.fromVector4(observer_.getCurrentStateVector().segment<int(observer_.sizeOri)>(
              observer_.contactOriIndex(contact.getID())));

          normalVector = contactOri.toAngleAxis().axis();
        }
        if(observer_.getContactWrench(contact.getID()).segment<int(observer_.sizeForce)>(0).norm()
           > contactDetectionThreshold_)
        {
          if(!contact.zmp.isSet())
          {
            logger.addLogEntry(category_ + "_debug_zmp_" + contact.getName(),
                               [this, &contact]() -> so::Vector3
                               { return mapContacts_.getZMPFromName(contact.getName()); });
          }
          contact.zmp =
              (normalVector.cross(
                   observer_.getContactWrench(contact.getID()).segment<int(observer_.sizeTorque)>(observer_.sizeForce)))
                  .cwiseQuotient(observer_.getContactWrench(contact.getID())
                                     .segment<int(observer_.sizeForce)>(0)
                                     .cross(normalVector));
        }
        else
        {
          contact.zmp.set(false);
          logger.removeLogEntry(category_ + "_debug_zmp_" + contact.getName());
        }
      }
    }
    */
    MCKOrobotTilt_0 =
        robot.bodySensor(mapIMUs_.getNameFromNum(0)).X_b_s().rotation() * X_0_fb_.rotation() * so::cst::gravity;
    MCKOrobotTilt_1 =
        robot.bodySensor(mapIMUs_.getNameFromNum(1)).X_b_s().rotation() * X_0_fb_.rotation() * so::cst::gravity;
  }

  /* Bring velocity of the IMU to the origin of the joint : we want the
   * velocity of joint 0, so stop one before the first joint */

  v_fb_0_.angular() = mcko_K_0_fb.angVel(); //  X_0_fb_.rotation() = mcko_K_0_fb.orientation.toMatrix3().transpose()
  v_fb_0_.linear() = mcko_K_0_fb.linVel();

  a_fb_0_.angular() = mcko_K_0_fb.angAcc(); //  X_0_fb_.rotation() = mcko_K_0_fb.orientation.toMatrix3().transpose()
  a_fb_0_.linear() = mcko_K_0_fb.linAcc();

  if(withDebugLogs_)
  {
    /* Updates of the logged variables */
    correctedMeasurements_ = observer_.getEKF().getSimulatedMeasurement(
        observer_.getEKF().getCurrentTime()); // Used only in the logger as debugging help
    globalCentroidKinematics_ = observer_.getGlobalCentroidKinematics(); // Used only in the logger as debugging help

    predictedGlobalCentroidState_ =
        observer_.getPredictedGlobalCentroidState(); // Used only in the logger as debugging help
    predictedAccelerometersGravityComponent_ =
        observer_.getPredictedAccelerometersGravityComponent(); // Used only in the logger as debugging help
    predictedWorldIMUsLinAcc_ =
        observer_.getPredictedAccelerometersLinAccComponent(); // Used only in the logger as debugging help
    predictedAccelerometers_ = observer_.getPredictedAccelerometers(); // Used only in the logger as debugging help
  }

  /* Update of the visual representation (only a visual feature) of the observed robot */
  my_robots_->robot().mbc().q = ctl.realRobot().mbc().q;

  /* Update of the observed robot */
  update(my_robots_->robot());

  return true;
}

///////////////////////////////////////////////////////////////////////
/// -------------------------Called functions--------------------------
///////////////////////////////////////////////////////////////////////

/*
void MCKineticsObserver::updateWorldFbKineAndViceVersa(const mc_rbdyn::Robot & inputRobot)
{
  const sva::PTransformd & worldFbPose = inputRobot.posW();
  const sva::MotionVecd & worldFbVel = inputRobot.velW();
  const sva::MotionVecd & worldFbAcc = inputRobot.accW();

  worldFbKine_.position = worldFbPose.translation();
  worldFbKine_.orientation = so::Matrix3(worldFbPose.rotation().transpose());
  worldFbKine_.linVel = worldFbVel.linear();
  worldFbKine_.angVel = worldFbVel.angular();
  worldFbKine_.linAcc = worldFbAcc.linear();
  worldFbKine_.angAcc = worldFbAcc.angular();

  fbWorldKine_ = worldFbKine_.getInverse();
}
*/

void MCKineticsObserver::initObserverStateVector(const mc_rbdyn::Robot & robot)
{
  so::kine::Orientation initOrientation;
  initOrientation.setZeroRotation<so::Quaternion>();
  Eigen::VectorXd initStateVector;
  initStateVector = Eigen::VectorXd::Zero(observer_.getStateSize());

  initStateVector.segment<int(observer_.sizePos)>(observer_.posIndex()) =
      robot.com(); // position of the centroid in fb = pos com in world - pos
                   // fb in world expressed in the frame of fb
  initStateVector.segment<int(observer_.sizeOri)>(observer_.oriIndex()) = initOrientation.toVector4();
  initStateVector.segment<int(observer_.sizeLinVel)>(observer_.linVelIndex()) =
      robot.comVelocity(); // position of the centroid in fb = pos com in world -
                           // pos fb in world expressed in the frame of fb

  observer_.setInitWorldCentroidStateVector(initStateVector);
}

void MCKineticsObserver::update(mc_control::MCController & ctl) // this function is called by the pipeline if the
                                                                // update is set to true in the configuration file
{
  auto & realRobot = ctl.realRobot(robot_);
  update(realRobot);
}

void MCKineticsObserver::update(mc_rbdyn::Robot & robot)
{
  robot.posW(X_0_fb_);
  robot.velW(v_fb_0_.vector());
}

void MCKineticsObserver::inputAdditionalWrench(const mc_rbdyn::Robot & inputRobot, const mc_rbdyn::Robot & measRobot)
{
  additionalUserResultingForce_.setZero();
  additionalUserResultingMoment_.setZero();

  totalForceCentroid_.setZero();
  totalTorqueCentroid_.setZero();

  for(auto & contactWithSensor :
      mapContacts_.contactsWithSensors()) // if a force sensor is not associated to a contact, its
                                          // measurement is given as an input external wrench
  {
    ContactWithSensor & contact = contactWithSensor.second;
    const std::string & fsName = contactWithSensor.first;
    if(fsName.find("LeftHandForceSensor") == std::string::npos
       || (fsName.find("LeftHandForceSensor") != std::string::npos && leftHandDetection_ != "None"))
    {
      if(contact.isExternalWrench == true)
      {
        sva::ForceVecd measuredWrench = measRobot.forceSensor(fsName).worldWrenchWithoutGravity(inputRobot);
        additionalUserResultingForce_ += measuredWrench.force();
        additionalUserResultingMoment_ += measuredWrench.moment();

        totalForceCentroid_ += measuredWrench.force();
        totalTorqueCentroid_ += measuredWrench.moment();
      }
    }
  }
  observer_.setAdditionalWrench(additionalUserResultingForce_, additionalUserResultingMoment_);

  if(withDebugLogs_)
  {

    for(auto & contactWithSensor :
        mapContacts_.contactsWithSensors()) // if a force sensor is not associated to a contact, its
                                            // measurement is given as an input external wrench
    {
      ContactWithSensor & contact = contactWithSensor.second;
      const std::string & fsName = contactWithSensor.first;
      so::Vector3 forceCentroid = so::Vector3::Zero();
      so::Vector3 torqueCentroid = so::Vector3::Zero();
      observer_.convertWrenchFromUserToCentroid(
          measRobot.forceSensor(fsName).worldWrenchWithoutGravity(inputRobot).force(),
          measRobot.forceSensor(fsName).worldWrenchWithoutGravity(inputRobot).moment(), forceCentroid, torqueCentroid);

      contact.wrenchInCentroid.segment<3>(0) = forceCentroid;
      contact.wrenchInCentroid.segment<3>(3) = torqueCentroid;

      contact.normForce = measRobot.forceSensor(fsName).wrenchWithoutGravity(measRobot).force().norm();
    }
  }
}

void MCKineticsObserver::updateIMUs(const mc_rbdyn::Robot & measRobot, const mc_rbdyn::Robot & inputRobot)
{
  unsigned i = 0;
  for(const auto & imu : IMUs_)
  {
    so::Vector3 & gyroBias = mapIMUs_(imu.name()).gyroBias;
    /** Position of accelerometer **/

    const sva::PTransformd & bodyImuPose = inputRobot.bodySensor(imu.name()).X_b_s();
    so::kine::Kinematics bodyImuKine = svaKinematicsConversion::poseFromSva(bodyImuPose);

    so::kine::Kinematics worldBodyKine = svaKinematicsConversion::kinematicsFromSva(
        inputRobot.mbc().bodyPosW[inputRobot.bodyIndexByName(imu.parentBody())],
        inputRobot.mbc().bodyVelW[inputRobot.bodyIndexByName(imu.parentBody())],
        inputRobot.mbc().bodyAccB[inputRobot.bodyIndexByName(imu.parentBody())], true, false);

    so::kine::Kinematics worldImuKine = worldBodyKine * bodyImuKine;
    const so::kine::Kinematics fbImuKine = worldImuKine;

    std::mt19937 generator(std::random_device{}());
    std::normal_distribution<double> dist(0.0, gyroBiasStandardDeviation_);

    // Add Gaussian noise
    for(int i = 0; i < gyroBias.size(); i++)
    {
      gyroBias(i) += dist(generator);
    }
    so::Vector3 gyroMeas = measRobot.bodySensor().angularVelocity() + gyroBias;

    observer_.setIMU(measRobot.bodySensor().linearAcceleration(), gyroMeas, acceleroSensorCovariance_,
                     gyroSensorCovariance_, fbImuKine, mapIMUs_.getNumFromName(imu.name()));

    ++i;
  }
}

const std::set<std::string> & MCKineticsObserver::findContactsFromSolver(const mc_control::MCController & ctl)
{
  const auto & measRobot = ctl.robot(robot_);
  auto & inputRobot = my_robots_->robot("inputRobot");

  contactsFound_.clear();
  for(const auto & contact : ctl.solver().contacts())
  {
    // std::cout << std::endl << contact.toStr() << std::endl;
    if(ctl.robots().robot(contact.r1Index()).name() == measRobot.name())
    {
      if(ctl.robots().robot(contact.r2Index()).mb().joint(0).type() == rbd::Joint::Fixed)
      {
        const auto & ifs = measRobot.indirectSurfaceForceSensor(contact.r1Surface()->name());
        mapContacts_.contactWithSensor(ifs.name()).isExternalWrench = true;
        if(!mapContacts_.contactWithSensor(ifs.name()).sensorEnabled)
        {
          mapContacts_.contactWithSensor(ifs.name()).isExternalWrench =
              false; // if the sensor attached to the contact is not enabled, it must not be considered as an external
                     // perturbation in any case
        }
        if(ifs.wrenchWithoutGravity(measRobot).force().norm() > contactDetectionThreshold_)
        {
          if(ifs.name().find("LeftHandForceSensor") == std::string::npos
             || (ifs.name().find("LeftHandForceSensor") != std::string::npos
                 && leftHandDetection_ == "asContact")) // checks that the contact is not attached to the left hand and
                                                        // if it is, checks that it must be considered as a contact in
                                                        // the Kinetics Observer and not as an external input wrench
          {
            contactsFound_.insert(ifs.name());
            mapContacts_.contactWithSensor(ifs.name()).isExternalWrench = false;
          }
        }
      }
    }
    else if(ctl.robots().robot(contact.r2Index()).name() == measRobot.name())
    {
      if(ctl.robots().robot(contact.r1Index()).mb().joint(0).type() == rbd::Joint::Fixed)
      {
        const auto & ifs = measRobot.indirectSurfaceForceSensor(contact.r2Surface()->name());
        if(!mapContacts_.contactWithSensor(ifs.name()).sensorEnabled)
        {
          mapContacts_.contactWithSensor(ifs.name()).isExternalWrench =
              false; // if the sensor attached to the contact is not enabled, it must not be considered as an external
                     // perturbation in any case
        }
        if(ifs.wrenchWithoutGravity(measRobot).force().norm() > contactDetectionThreshold_)
        {
          if(ifs.name().find("LeftHandForceSensor") == std::string::npos
             || (ifs.name().find("LeftHandForceSensor") != std::string::npos
                 && leftHandDetection_ == "asContact")) // checks that the contact is not attached to the left hand and
                                                        // if it is, checks that it must be considered as a contact in
                                                        // the Kinetics Observer and not as an external input wrench
          {
            contactsFound_.insert(ifs.name());
            mapContacts_.contactWithSensor(ifs.name()).isExternalWrench = false;
          }
        }
      }
    }
  }
}

const std::set<std::string> & MCKineticsObserver::findContactsFromSurfaces(const mc_control::MCController & ctl)
{
  const auto & measRobot = ctl.robot(robot_);
  auto & inputRobot = my_robots_->robot("inputRobot");

  contactsFound_.clear();

  for(auto & contact : mapContacts_.contactsWithSensors())
  {
    const std::string & fsName = contact.first;
    mapContacts_.contactWithSensor(fsName).isExternalWrench = true;
    const mc_rbdyn::ForceSensor forceSensor = measRobot.forceSensor(fsName);
    if(!mapContacts_.contactWithSensor(fsName).sensorEnabled)
    {
      mapContacts_.contactWithSensor(fsName).isExternalWrench =
          false; // if the sensor attached to the contact is not enabled, it must not be considered as an
                 // external perturbation in any case
    }
    if(fsName.find("LeftHandForceSensor") == std::string::npos
       || (fsName.find("LeftHandForceSensor") != std::string::npos && leftHandDetection_ == "asContact"))
    // checks that the contact is not attached to the left hand and if it is,
    // checks that it must be considered as a contact in the Kinetics Observer
    // and not as an external input wrench
    {
      if(withFilteredForcesContactDetection_) // NOT WORKING FOR NOW
      {
        ContactWithSensor & contact = mapContacts_.contactWithSensor(fsName);
        contact.filteredForce = (1 - ctl.timeStep * contact.lambda) * contact.filteredForce
                                + contact.lambda * ctl.timeStep * forceSensor.wrenchWithoutGravity(measRobot).force();

        if(contact.filteredForce.norm() > contactDetectionThreshold_)
        {
          contactsFound_.insert(fsName);
          contact.isExternalWrench = false;
        }
      }
      else
      {
        if(forceSensor.wrenchWithoutGravity(measRobot).force().norm() > contactDetectionThreshold_)
        {
          contactsFound_.insert(fsName);
          mapContacts_.contactWithSensor(fsName).isExternalWrench = false;
        }
      }
    }
  }
}

const std::set<std::string> & MCKineticsObserver::findContactsFromThreshold(const mc_control::MCController & ctl)
{
  const auto & measRobot = ctl.robot(robot_);
  auto & inputRobot = my_robots_->robot("inputRobot");

  contactsFound_.clear();

  for(auto & contact : mapContacts_.contactsWithSensors())
  {
    const std::string & fsName = contact.first;
    mapContacts_.contactWithSensor(fsName).isExternalWrench = true;
    const mc_rbdyn::ForceSensor forceSensor = measRobot.forceSensor(fsName);
    if(!mapContacts_.contactWithSensor(fsName).sensorEnabled)
    {
      mapContacts_.contactWithSensor(fsName).isExternalWrench =
          false; // if the sensor attached to the contact is not enabled, it must not be considered as an
                 // external perturbation in any case
    }
    if(fsName.find("LeftHandForceSensor") == std::string::npos
       || (fsName.find("LeftHandForceSensor") != std::string::npos && leftHandDetection_ == "asContact"))
    // checks that the contact is not attached to the left hand and if it is,
    // checks that it must be considered as a contact in the Kinetics Observer
    // and not as an external input wrench
    {
      if(withFilteredForcesContactDetection_) // NOT WORKING FOR NOW
      {
        ContactWithSensor & contact = mapContacts_.contactWithSensor(fsName);
        contact.filteredForce = (1 - ctl.timeStep * contact.lambda) * contact.filteredForce
                                + contact.lambda * ctl.timeStep * forceSensor.wrenchWithoutGravity(measRobot).force();

        if(contact.filteredForce.norm() > contactDetectionThreshold_)
        {
          contactsFound_.insert(fsName);
          contact.isExternalWrench = false;
        }
      }
      else
      {
        if(forceSensor.wrenchWithoutGravity(measRobot).force().norm() > contactDetectionThreshold_)
        {
          contactsFound_.insert(fsName);
          mapContacts_.contactWithSensor(fsName).isExternalWrench = false;
        }
      }
    }
  }
}

const std::set<std::string> & MCKineticsObserver::findContacts(const mc_control::MCController & ctl)
{
  const auto & measRobot = ctl.robot(robot_);
  auto & inputRobot = my_robots_->robot("inputRobot");

  if(contactsDetection_ == "fromSolver")
  {
    findContactsFromSolver(ctl);
  }
  if(contactsDetection_ == "fromThreshold")
  {
    findContactsFromThreshold(ctl);
  }
  if(contactsDetection_ == "fromSurfaces")
  {
    findContactsFromSurfaces(ctl);
  }

  inputAdditionalWrench(inputRobot, measRobot);
  if(!contactsFound_.empty() && !simStarted_) // we start the observation once a contact has been detected.
  {
    simStarted_ = true;
    initObserverStateVector(measRobot);
  }

  return contactsFound_;
}

void MCKineticsObserver::updateContact(const mc_control::MCController & ctl,
                                       const std::string & name,
                                       mc_rtc::Logger & logger)
{
  auto & inputRobot = my_robots_->robot("inputRobot");

  const auto & robot = ctl.robot(robot_);
  const mc_rbdyn::ForceSensor forceSensor = robot.forceSensor(name);
  const std::string & fsName = forceSensor.name();
  ContactWithSensor & contact = mapContacts_.contactWithSensor(fsName);

  sva::ForceVecd measuredWrench = forceSensor.wrenchWithoutGravity(robot);

  // pose of the sensor within the frame of its parent body
  const sva::PTransformd & bodySensorPoseRobot = forceSensor.X_p_f();
  // kinematics of the sensor within the frame of its parent body
  so::kine::Kinematics bodySensorKine = svaKinematicsConversion::poseFromSva(bodySensorPoseRobot);

  // pose of the sensor's parent body in the input robot within the world / floating base's frame
  const sva::PTransform posWBody = inputRobot.mbc().bodyPosW[inputRobot.bodyIndexByName(forceSensor.parentBody())];
  // velocity of the sensor's parent body in the input robot within the world / floating base's frame
  const sva::MotionVecd velWBody = inputRobot.mbc().bodyVelW[inputRobot.bodyIndexByName(forceSensor.parentBody())];
  // acceleration of the sensor's parent body in the input robot within the world / floating base's frame. Warning ! To
  // the contrary of the other variables, this acceleration is expressed in the frame of the parent body (local frame)
  const sva::MotionVecd locAccWBody = inputRobot.mbc().bodyAccB[inputRobot.bodyIndexByName(forceSensor.parentBody())];

  // kinematics of the sensor's parent body in the input robot within the world / floating base's frame
  so::kine::Kinematics worldBodyKineInputRobot =
      svaKinematicsConversion::kinematicsFromSva(posWBody, velWBody, locAccWBody, true, false);

  // kinematics of the sensor in the input robot within the world / floating base's frame
  so::kine::Kinematics worldSensorKineInputRobot = worldBodyKineInputRobot * bodySensorKine;
  // kinematics of the sensor in the input robot within the world / floating base's frame
  so::kine::Kinematics fbContactKineInputRobot;
  if(contact.isAttachedToSurface)
  {
    fbContactKineInputRobot = worldSensorKineInputRobot;
    contactWrenchVector_.segment<3>(0) = measuredWrench.force(); // retrieving the force
    contactWrenchVector_.segment<3>(3) = measuredWrench.moment(); // retrieving the torque
  }
  else
  {
    // pose of the surface in the world / floating base's frame
    sva::PTransformd worldSurfacePoseInputRobot = inputRobot.surfacePose(contact.surface);
    // Kinematics of the surface in the world / floating base's frame
    so::kine::Kinematics worldSurfaceKineInputRobot = svaKinematicsConversion::poseFromSva(worldSurfacePoseInputRobot);

    worldSurfaceKineInputRobot.orientation = so::Matrix3(worldSurfacePoseInputRobot.rotation().transpose());
    // the kinematics in the world of the sensor in the inputRobot are equal to the kinematics of the sensor in the
    // frame of the floating base.
    fbContactKineInputRobot = worldSurfaceKineInputRobot;

    so::kine::Kinematics surfaceSensorKine;
    surfaceSensorKine = worldSurfaceKineInputRobot.getInverse() * worldSensorKineInputRobot;
    contactWrenchVector_.segment<3>(0) = surfaceSensorKine.orientation * measuredWrench.force();

    contactWrenchVector_.segment<3>(3) = surfaceSensorKine.orientation * measuredWrench.moment()
                                         + surfaceSensorKine.position().cross(contactWrenchVector_.segment<3>(0));
  }

  if(contact.wasAlreadySet) // checks if the contact already exists, if yes, it is updated
  {
    if(contact.sensorEnabled)
    {
      observer_.updateContactWithWrenchSensor(contactWrenchVector_, contactSensorCovariance_, fbContactKineInputRobot,
                                              mapContacts_.getNumFromName(fsName));
    }
    else
    {
      observer_.updateContactWithNoSensor(fbContactKineInputRobot, mapContacts_.getNumFromName(fsName));
    }

    if(withDebugLogs_)
    {
      if(contact.sensorEnabled && !contact.sensorWasEnabled)
      {
        addContactMeasurementsLogEntries(logger, fsName);
        contact.sensorWasEnabled = true;
      }
      if(!contact.sensorEnabled && contact.sensorWasEnabled)
      {
        removeContactMeasurementsLogEntries(logger, fsName);
        contact.sensorWasEnabled = false;
      }
    }
  }
  else // the contact still doesn't exist, it is added to the observer
  {
    so::kine::Kinematics
        worldContactKineRef; // reference of the contact in the control world frame using the control robot
    sva::PTransformd worldSurfacePoseRobot; // used only if the sensor is not attached to a surface
    const int & numContact = mapContacts_.getNumFromName(fsName);
    /* robot for the kinematics in the world frame */

    if(withOdometry_)
    {

      if(!contact.sensorEnabled)
      {
        mc_rtc::log::info("The sensor is disabled but is required for the odometry");
      }
      const so::Vector3 & contactForceMeas = contactWrenchVector_.segment<3>(0);
      const so::Vector3 & contactTorqueMeas = contactWrenchVector_.segment<3>(3);
      const so::kine::Kinematics worldContactKine = observer_.getGlobalKinematicsOf(fbContactKineInputRobot);
      worldContactKineRef.position =
          worldContactKine.orientation.toMatrix3() * linStiffness_.inverse()
              * (contactForceMeas
                 + worldContactKine.orientation.toMatrix3().transpose() * linDamping_ * worldContactKine.linVel())
          + worldContactKine.position();
      so::Vector3 flexRotDiff =
          -2 * worldContactKine.orientation.toMatrix3() * angStiffness_.inverse()
          * (contactTorqueMeas
             + worldContactKine.orientation.toMatrix3().transpose() * angDamping_
                   * worldContactKine.angVel()); // difference between the reference orientation and
                                                 // the real one, obtained from the visco-elastic model
      so::Vector3 flexRotAxis = flexRotDiff / flexRotDiff.norm();
      double diffNorm = flexRotDiff.norm() / 2;

      if(diffNorm > 1.0)
      {
        diffNorm = 1.0;
      }
      else if(diffNorm < -1.0)
      {
        diffNorm = -1.0;
      }
      double flexRotAngle = std::asin(diffNorm);
      Eigen::AngleAxisd flexRotAngleAxis(flexRotAngle, flexRotAxis);
      so::Matrix3 flexRotMatrix = so::kine::Orientation(flexRotAngleAxis).toMatrix3();
      worldContactKineRef.orientation =
          so::Matrix3(flexRotMatrix.transpose() * worldContactKine.orientation.toMatrix3());
      if(withFlatOdometry_)
      {
        so::kine::Kinematics worldContactKineRobot;
        if(contact.isAttachedToSurface)
        {
          so::kine::Kinematics worldBodyKineRobot;
          worldBodyKineRobot.position =
              robot.mbc().bodyPosW[robot.bodyIndexByName(forceSensor.parentBody())].translation();
          worldBodyKineRobot.orientation =
              so::Matrix3(robot.mbc().bodyPosW[robot.bodyIndexByName(forceSensor.parentBody())].rotation().transpose());
          worldContactKineRobot = worldBodyKineRobot * bodySensorKine;
        }
        else
        {
          worldSurfacePoseRobot = robot.surfacePose(contact.surface);
          worldContactKineRobot.position = worldSurfacePoseRobot.translation();
          worldContactKineRobot.orientation = so::Matrix3(worldSurfacePoseRobot.rotation().transpose());
        }

        worldContactKineRef.position()(2) = worldContactKineRobot.position()(
            2); // the reference altitude of the contact is the one in the control robot
      }
    }
    else
    {
      if(contact.isAttachedToSurface)
      {
        so::kine::Kinematics worldBodyKineRobot;
        worldBodyKineRobot.position =
            robot.mbc().bodyPosW[robot.bodyIndexByName(forceSensor.parentBody())].translation();
        worldBodyKineRobot.orientation =
            so::Matrix3(robot.mbc().bodyPosW[robot.bodyIndexByName(forceSensor.parentBody())].rotation().transpose());

        worldContactKineRef = worldBodyKineRobot * bodySensorKine;
      }
      else
      {
        worldSurfacePoseRobot = robot.surfacePose(contact.surface);
        worldContactKineRef.position = worldSurfacePoseRobot.translation();
        worldContactKineRef.orientation = so::Matrix3(worldSurfacePoseRobot.rotation().transpose());
      }
    }

    if(observer_.getNumberOfSetContacts() > 0) // checks if another contact is already set
    {
      observer_.addContact(worldContactKineRef, contactInitCovarianceNewContacts_, contactProcessCovariance_,
                           numContact, linStiffness_, linDamping_, angStiffness_, angDamping_);
    }
    else
    {
      observer_.addContact(worldContactKineRef, contactInitCovarianceFirstContacts_, contactProcessCovariance_,
                           numContact, linStiffness_, linDamping_, angStiffness_, angDamping_);
    }
    if(contact.sensorEnabled)
    {
      observer_.updateContactWithWrenchSensor(contactWrenchVector_,
                                              contactSensorCovariance_, // contactInitCovariance_.block<6,6>(6,6)
                                              fbContactKineInputRobot, numContact);
    }
    else
    {
      observer_.updateContactWithNoSensor(fbContactKineInputRobot, numContact);
    }

    if(withDebugLogs_)
    {
      addContactLogEntries(logger, fsName);
    }
  }
}

void MCKineticsObserver::updateContacts(const mc_control::MCController & ctl,
                                        std::set<std::string> updatedContacts,
                                        mc_rtc::Logger & logger)
{
  const auto & robot = ctl.robot(robot_);

  /** Debugging output **/
  if(verbose_ && updatedContacts != oldContacts_)
    mc_rtc::log::info("[{}] Contacts changed: {}", name(), mc_rtc::io::to_string(updatedContacts));
  contactPositions_.clear();

  for(const auto & updatedContact : updatedContacts)
  {
    if(oldContacts_.find(updatedContact)
       != oldContacts_.end()) // checks if the contact already exists, if yes, it is updated
    {
      mapContacts_.contactWithSensor(updatedContact).wasAlreadySet = true;
    }
    else
    {
      mapContacts_.contactWithSensor(updatedContact).wasAlreadySet = false;
      mapContacts_.contactWithSensor(updatedContact).isSet = true;
    }
    /*
    else
    {
      mapContacts_.insertContact(robot.forceSensor(updatedContact).name(), true);
    }
    */

    updateContact(ctl, updatedContact, logger);
  }
  std::set<std::string>
      diffs; // List of the contact that were available on last iteration but are not set anymore on the current one
  std::set_difference(oldContacts_.begin(), oldContacts_.end(), updatedContacts.begin(), updatedContacts.end(),
                      std::inserter(diffs, diffs.end()));
  for(const auto & diff : diffs)
  {
    int numDiff = mapContacts_.getNumFromName(diff);
    observer_.removeContact(numDiff);
    mapContacts_.contactWithSensor(diff).resetContact();

    if(withDebugLogs_)
    {
      removeContactLogEntries(logger, diff);
      removeContactMeasurementsLogEntries(logger, diff);
    }
  }
  oldContacts_ = updatedContacts;

  unsigned nbContacts = static_cast<unsigned>(updatedContacts.size());
  if(debug_)
  {
    mc_rtc::log::info("nbContacts = {}", nbContacts);
  }
}

void MCKineticsObserver::mass(double mass)
{
  mass_ = mass;
  observer_.setMass(mass);
}

///////////////////////////////////////////////////////////////////////
/// -------------------------------Logs--------------------------------
///////////////////////////////////////////////////////////////////////

void MCKineticsObserver::addToLogger(const mc_control::MCController &,
                                     mc_rtc::Logger & logger,
                                     const std::string & category)
{
  logger.addLogEntry(category + "_mcko_fb_posW", [this]() -> const sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category + "_mcko_fb_velW", [this]() -> const sva::MotionVecd & { return v_fb_0_; });
  logger.addLogEntry(category + "_mcko_fb_accW", [this]() -> const sva::MotionVecd & { return a_fb_0_; });

  logger.addLogEntry(category + "_mcko_fb_yaw",
                     [this]() -> const double
                     { return -so::kine::rotationMatrixToYawAxisAgnostic(X_0_fb_.rotation()); });

  logger.addLogEntry(category + "_constants_mass", [this]() -> const double & { return observer_.getMass(); });

  logger.addLogEntry(category + "_constants_forceThreshold",
                     [this]() -> double { return mass_ * so::cst::gravityConstant * contactDetectionPropThreshold_; });
}

void MCKineticsObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_posW");
  logger.removeLogEntry(category + "_velW");
  logger.removeLogEntry(category + "_mass");
  logger.removeLogEntry(category + "_flexStiffness");
  logger.removeLogEntry(category + "_flexDamping");
}

void MCKineticsObserver::addToGUI(const mc_control::MCController &,
                                  mc_rtc::gui::StateBuilder & gui,
                                  const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;
  // clang-format off
  gui.addElement(category,
    mc_state_observation::gui::make_input_element("Gyro Biases standard deviation", gyroBiasStandardDeviation_),
    mc_state_observation::gui::make_input_element("Accel Covariance", acceleroSensorCovariance_(0,0)),
    mc_state_observation::gui::make_input_element("Force Covariance", contactSensorCovariance_(0,0)),
    mc_state_observation::gui::make_input_element("Gyro Covariance", gyroSensorCovariance_(0,0)),
    Label("contacts", [this]() { return mc_rtc::io::to_string(oldContacts_); }));
  // clang-format on
}

void MCKineticsObserver::plotVariablesBeforeUpdate(const mc_control::MCController & ctl, mc_rtc::Logger & logger)
{
  /* Plots of the updated state */
  logger.addLogEntry(category_ + "_globalWorldCentroidState_positionW_",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.position(); });
  logger.addLogEntry(category_ + "_globalWorldCentroidState_linVelW",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.linVel(); });
  logger.addLogEntry(category_ + "_globalWorldCentroidState_linAccW",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.linAcc(); });
  logger.addLogEntry(category_ + "_globalWorldCentroidState_oriW",
                     [this]() -> Eigen::Quaternion<double>
                     { return globalCentroidKinematics_.orientation.inverse().toQuaternion(); });
  logger.addLogEntry(category_ + "_globalWorldCentroidState_angVelW",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.angVel(); });
  logger.addLogEntry(category_ + "_globalWorldCentroidState_angAccW",
                     [this]() -> Eigen::Vector3d { return globalCentroidKinematics_.angAcc(); });
  for(const auto & imu : IMUs_)
  {
    logger.addLogEntry(category_ + "_globalWorldCentroidState_gyroBias_" + imu.name(),
                       [this, imu]() -> Eigen::Vector3d
                       {
                         return observer_.getCurrentStateVector().segment<int(observer_.sizeGyroBias)>(
                             observer_.gyroBiasIndex(mapIMUs_.getNumFromName(imu.name())));
                       });
  }
  logger.addLogEntry(
      category_ + "_globalWorldCentroidState_extForceCentr",
      [this]() -> Eigen::Vector3d
      { return observer_.getCurrentStateVector().segment<int(observer_.sizeForce)>(observer_.unmodeledForceIndex()); });

  logger.addLogEntry(category_ + "_globalWorldCentroidState_extTorqueCentr",
                     [this]() -> Eigen::Vector3d {
                       return observer_.getCurrentStateVector().segment<int(observer_.sizeTorque)>(
                           observer_.unmodeledTorqueIndex());
                     });

  /* Inputs */
  logger.addLogEntry(category_ + "_inputs_additionalWrench_Force",
                     [this]() -> Eigen::Vector3d
                     { return observer_.getAdditionalWrench().segment<int(observer_.sizeForce)>(0); });
  logger.addLogEntry(category_ + "_inputs_additionalWrench_Torque",
                     [this]() -> Eigen::Vector3d {
                       return observer_.getAdditionalWrench().segment<int(observer_.sizeTorque)>(observer_.sizeForce);
                     });

  /* State covariances */
  logger.addLogEntry(category_ + "_stateCovariances_positionW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block<int(observer_.sizePosTangent), int(observer_.sizePosTangent)>(
                               observer_.posIndexTangent(), observer_.posIndexTangent())
                           .diagonal();
                     });
  logger.addLogEntry(category_ + "_stateCovariances_orientationW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block<int(observer_.sizeOriTangent), int(observer_.sizeOriTangent)>(
                               observer_.oriIndexTangent(), observer_.oriIndexTangent())
                           .diagonal();
                     });
  logger.addLogEntry(category_ + "_stateCovariances_linVelW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block<int(observer_.sizeLinVelTangent), int(observer_.sizeLinVelTangent)>(
                               observer_.linVelIndexTangent(), observer_.linVelIndexTangent())
                           .diagonal();
                     });
  logger.addLogEntry(category_ + "_stateCovariances_angVelW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block<int(observer_.sizeAngVelTangent), int(observer_.sizeAngVelTangent)>(
                               observer_.angVelIndexTangent(), observer_.angVelIndexTangent())
                           .diagonal();
                     });

  for(const auto & imu : IMUs_)
  {
    logger.addLogEntry(category_ + "_stateCovariances_gyroBias_" + imu.name(),
                       [this, imu]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF()
                             .getStateCovariance()
                             .block<int(observer_.sizeGyroBiasTangent), int(observer_.sizeGyroBiasTangent)>(
                                 observer_.gyroBiasIndexTangent(mapIMUs_.getNumFromName(imu.name())),
                                 observer_.gyroBiasIndexTangent(mapIMUs_.getNumFromName(imu.name())))
                             .diagonal();
                       });
  }

  logger.addLogEntry(category_ + "_stateCovariances_extForce_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block<int(observer_.sizeForceTangent), int(observer_.sizeForceTangent)>(
                               observer_.unmodeledForceIndexTangent(), observer_.unmodeledForceIndexTangent())
                           .diagonal();
                     });
  logger.addLogEntry(category_ + "_stateCovariances_extTorque_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF()
                           .getStateCovariance()
                           .block<int(observer_.sizeTorqueTangent), int(observer_.sizeTorqueTangent)>(
                               observer_.unmodeledTorqueIndexTangent(), observer_.unmodeledTorqueIndexTangent())
                           .diagonal();
                     });

  logger.addLogEntry(category_ + "_realRobot_LeftFoot",
                     [&ctl]()
                     {
                       if(ctl.realRobot().hasBody("LeftFoot"))
                       {
                         return ctl.realRobot().frame("LeftFoot").position();
                       }
                     });

  logger.addLogEntry(category_ + "_realRobot_RightFoot",
                     [&ctl]()
                     {
                       if(ctl.realRobot().hasBody("RightFoot"))
                       {
                         return ctl.realRobot().frame("RightFoot").position();
                       }
                     });
  logger.addLogEntry(category_ + "_realRobot_LeftHand",
                     [&ctl]()
                     {
                       if(ctl.realRobot().hasBody("LeftHand"))
                       {
                         return ctl.realRobot().frame("LeftHand").position();
                       }
                     });
  logger.addLogEntry(category_ + "_realRobot_RightHand",
                     [&ctl]()
                     {
                       if(ctl.realRobot().hasBody("RightHand"))
                       {
                         return ctl.realRobot().frame("RightHand").position();
                       }
                     });
  logger.addLogEntry(category_ + "_ctlRobot_LeftFoot",
                     [&ctl]()
                     {
                       if(ctl.robot().hasBody("LeftFoot"))
                       {
                         return ctl.robot().frame("LeftFoot").position();
                       }
                     });

  logger.addLogEntry(category_ + "_ctlRobot_RightFoot",
                     [&ctl]()
                     {
                       if(ctl.robot().hasBody("RightFoot"))
                       {
                         return ctl.robot().frame("RightFoot").position();
                       }
                     });
  logger.addLogEntry(category_ + "_ctlRobot_LeftHand",
                     [&ctl]()
                     {
                       if(ctl.robot().hasBody("LeftHand"))
                       {
                         return ctl.robot().frame("LeftHand").position();
                       }
                     });
  logger.addLogEntry(category_ + "_ctlRobot_RightHand",
                     [&ctl]()
                     {
                       if(ctl.robot().hasBody("RightHand"))
                       {
                         return ctl.robot().frame("RightHand").position();
                       }
                     });
}

void MCKineticsObserver::plotVariablesAfterUpdate(const mc_control::MCController & ctl, mc_rtc::Logger & logger)
{
  /* Plots of the predicted state */
  logger.addLogEntry(category_ + "_robotsFbIMU_robot_worldImu_Ori_" + mapIMUs_.getNameFromNum(0),
                     [this]() -> so::Quaternion { return robotImuOri_0; });
  logger.addLogEntry(category_ + "_robotsFbIMU_robot_worldImu_Ori_" + mapIMUs_.getNameFromNum(1),
                     [this]() -> so::Quaternion { return robotImuOri_1; });
  logger.addLogEntry(category_ + "_robotsFbIMU_realRobot_worldImu_Ori_" + mapIMUs_.getNameFromNum(0),
                     [this]() -> so::Quaternion { return realRobotImuOri_0; });
  logger.addLogEntry(category_ + "_robotsFbIMU_realRobot_worldImu_Ori_" + mapIMUs_.getNameFromNum(1),
                     [this]() -> so::Quaternion { return realRobotImuOri_1; });
  logger.addLogEntry(category_ + "_robotsFbIMU_robot_worldFb_Ori_", [this]() -> so::Quaternion { return robotFbOri_; });
  logger.addLogEntry(category_ + "_robotsFbIMU_realRobot_worldFb_Ori_",
                     [this]() -> so::Quaternion { return realRobotFbOri_; });
  logger.addLogEntry(category_ + "_robotsFbIMU_robot_ImuInFb_Pos_" + mapIMUs_.getNameFromNum(0),
                     [this]() -> Eigen::Vector3d { return robotPosImuInFB_0; });
  logger.addLogEntry(category_ + "_robotsFbIMU_robot_ImuInFb_Pos_" + mapIMUs_.getNameFromNum(1),
                     [this]() -> Eigen::Vector3d { return robotPosImuInFB_1; });
  logger.addLogEntry(category_ + "_robotsFbIMU_realRobot_ImuInFb_Pos_" + mapIMUs_.getNameFromNum(0),
                     [this]() -> Eigen::Vector3d { return realRobotPosImuInFB_0; });
  logger.addLogEntry(category_ + "_robotsFbIMU_realRobot_ImuInFb_Pos_" + mapIMUs_.getNameFromNum(1),
                     [this]() -> Eigen::Vector3d { return realRobotPosImuInFB_1; });
  logger.addLogEntry(category_ + "_robotsFbIMU_robot_Tilt_" + mapIMUs_.getNameFromNum(0),
                     [this]() -> Eigen::Vector3d { return robotTilt_0; });
  logger.addLogEntry(category_ + "_robotsFbIMU_robot_Tilt_" + mapIMUs_.getNameFromNum(1),
                     [this]() -> Eigen::Vector3d { return robotTilt_1; });
  logger.addLogEntry(category_ + "_robotsFbIMU_realRobot_Tilt_" + mapIMUs_.getNameFromNum(0),
                     [this]() -> Eigen::Vector3d { return realRobotTilt_0; });
  logger.addLogEntry(category_ + "_robotsFbIMU_realRobot_Tilt_" + mapIMUs_.getNameFromNum(1),
                     [this]() -> Eigen::Vector3d { return realRobotTilt_1; });
  logger.addLogEntry(category_ + "_robotsFbIMU_MCKO_Tilt_" + mapIMUs_.getNameFromNum(0),
                     [this]() -> Eigen::Vector3d { return MCKOrobotTilt_0; });
  logger.addLogEntry(category_ + "_robotsFbIMU_MCKO_Tilt_" + mapIMUs_.getNameFromNum(1),
                     [this]() -> Eigen::Vector3d { return MCKOrobotTilt_1; });
  logger.addLogEntry(category_ + "_realRobot_centroidImuOri_" + mapIMUs_.getNameFromNum(0),
                     [this]() -> so::Quaternion { return realRobot_centroidImuOri_0; });
  logger.addLogEntry(category_ + "_realRobot_centroidImuOri_" + mapIMUs_.getNameFromNum(1),
                     [this]() -> so::Quaternion { return realRobot_centroidImuOri_1; });

  logger.addLogEntry(category_ + "_predictedGlobalCentroidKinematics_positionW_",
                     [this]() -> Eigen::Vector3d
                     { return predictedGlobalCentroidState_.segment<int(observer_.sizePos)>(observer_.posIndex()); });
  logger.addLogEntry(category_ + "_predictedGlobalCentroidKinematics_linVelW",
                     [this]() -> Eigen::Vector3d {
                       return predictedGlobalCentroidState_.segment<int(observer_.sizeLinVel)>(observer_.linVelIndex());
                     });

  logger.addLogEntry(
      category_ + "_predictedGlobalCentroidKinematics_oriW",
      [this]() -> Eigen::Vector3d
      {
        so::kine::Orientation ori;
        return ori.fromVector4(predictedGlobalCentroidState_.segment<int(observer_.sizeOri)>(observer_.oriIndex()))
            .toRotationVector();
      });
  logger.addLogEntry(category_ + "_predictedGlobalCentroidKinematics_angVelW",
                     [this]() -> Eigen::Vector3d {
                       return predictedGlobalCentroidState_.segment<int(observer_.sizeAngVel)>(observer_.angVelIndex());
                     });

  for(const auto & imu : IMUs_)
  {
    logger.addLogEntry(category_ + "_predictedGlobalCentroidKinematics_gyroBias" + imu.name() + "_measured",
                       [this, imu]() -> Eigen::Vector3d
                       {
                         return predictedGlobalCentroidState_.segment<int(observer_.sizeGyroBias)>(
                             observer_.gyroBiasIndexTangent(mapIMUs_.getNumFromName(imu.name())));
                       });
  }

  /* Plots of the inputs */

  logger.addLogEntry(category_ + "_inputs_angularMomentum",
                     [this]() -> Eigen::Vector3d { return observer_.getAngularMomentum()(); });
  logger.addLogEntry(category_ + "_inputs_angularMomentumDot",
                     [this]() -> Eigen::Vector3d { return observer_.getAngularMomentumDot()(); });
  logger.addLogEntry(category_ + "_inputs_com", [this]() -> Eigen::Vector3d { return observer_.getCenterOfMass()(); });
  logger.addLogEntry(category_ + "_inputs_comDot",
                     [this]() -> Eigen::Vector3d { return observer_.getCenterOfMassDot()(); });
  logger.addLogEntry(category_ + "_inputs_comDotDot",
                     [this]() -> Eigen::Vector3d { return observer_.getCenterOfMassDotDot()(); });
  logger.addLogEntry(category_ + "_inputs_inertiaMatrix",
                     [this]() -> Eigen::Vector6d
                     {
                       so::Vector6 inertia;
                       inertia.segment<3>(0) = observer_.getInertiaMatrix()().diagonal();
                       inertia.segment<2>(3) = observer_.getInertiaMatrix()().block<1, 2>(0, 1);
                       inertia(5) = observer_.getInertiaMatrix()()(1, 2);
                       return inertia;
                     });

  logger.addLogEntry(category_ + "_inputs_inertiaMatrixDot",
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
      logger.addLogEntry(category_ + "_measurements_gyro_" + imu.name() + "_measured",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastMeasurement().segment<int(observer_.sizeGyroBias)>(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name()))
                               + observer_.sizeAcceleroSignal);
                         });
      logger.addLogEntry(category_ + "_measurements_gyro_" + imu.name() + "_predicted",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastPredictedMeasurement().segment<int(observer_.sizeGyroBias)>(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name()))
                               + observer_.sizeAcceleroSignal);
                         });
      logger.addLogEntry(category_ + "_measurements_gyro_" + imu.name() + "_corrected",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return correctedMeasurements_.segment<int(observer_.sizeGyroBias)>(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name()))
                               + observer_.sizeAcceleroSignal);
                         });

      logger.addLogEntry(category_ + "_measurements_accelerometer_" + imu.name() + "_measured",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return observer_.getEKF().getLastMeasurement().segment<int(observer_.sizeAcceleroSignal)>(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())));
                         });
      logger.addLogEntry(
          category_ + "_measurements_accelerometer_" + imu.name() + "_predicted",
          [this, imu]() -> Eigen::Vector3d
          {
            return observer_.getEKF().getLastPredictedMeasurement().segment<int(observer_.sizeAcceleroSignal)>(
                observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())));
          });
      logger.addLogEntry(category_ + "_measurements_accelerometer_" + imu.name() + "_corrected",
                         [this, imu]() -> Eigen::Vector3d
                         {
                           return correctedMeasurements_.segment<int(observer_.sizeAcceleroSignal)>(
                               observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())));
                         });

      logger.addLogEntry(category_ + "_debug_IMU_Tilt" + imu.name(),
                         [this, imu]() -> Eigen::Vector3d
                         { return predictedAccelerometersGravityComponent_.at(mapIMUs_.getNumFromName(imu.name())); });

      logger.addLogEntry(category_ + "_debug_IMU_linearIMUAcc" + imu.name(),
                         [this, imu]() -> Eigen::Vector3d
                         { return predictedWorldIMUsLinAcc_.at(mapIMUs_.getNumFromName(imu.name())); });
    }
  }

  /* Plots of the innovation */
  logger.addLogEntry(category_ + "_innovation_positionW_",
                     [this]() -> Eigen::Vector3d {
                       return observer_.getEKF().getInnovation().segment<int(observer_.sizePosTangent)>(
                           observer_.posIndexTangent());
                     });
  logger.addLogEntry(category_ + "_innovation_linVelW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment<int(observer_.sizeLinVelTangent)>(
                           observer_.linVelIndexTangent());
                     });
  logger.addLogEntry(category_ + "_innovation_oriW_",
                     [this]() -> Eigen::Vector3d {
                       return observer_.getEKF().getInnovation().segment<int(observer_.sizeOriTangent)>(
                           observer_.oriIndexTangent());
                     });
  logger.addLogEntry(category_ + "_innovation_angVelW_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment<int(observer_.sizeAngVelTangent)>(
                           observer_.angVelIndexTangent());
                     });
  for(const auto & imu : IMUs_)
  {
    logger.addLogEntry(category_ + "_innovation_gyroBias_" + imu.name(),
                       [this, imu]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getInnovation().segment<int(observer_.sizeGyroBias)>(
                             observer_.gyroBiasIndexTangent(mapIMUs_.getNumFromName(imu.name())));
                       });
  }
  logger.addLogEntry(category_ + "_innovation_unmodeledForce_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment<int(observer_.sizeForceTangent)>(
                           observer_.unmodeledForceIndexTangent());
                     });
  logger.addLogEntry(category_ + "_innovation_unmodeledTorque_",
                     [this]() -> Eigen::Vector3d
                     {
                       return observer_.getEKF().getInnovation().segment<int(observer_.sizeTorqueTangent)>(
                           observer_.unmodeledTorqueIndexTangent());
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

  for(auto & contactWithSensor : mapContacts_.contactsWithSensors())
  {
    const ContactWithSensor & contact = contactWithSensor.second;
    const std::string & fsName = contactWithSensor.first;
    logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_" + fsName + "_force",
                       [this, fsName]() -> Eigen::Vector3d
                       { return mapContacts_.contactWithSensor(fsName).wrenchInCentroid.segment<3>(0); });
    logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_" + fsName + "_torque",
                       [this, fsName]() -> Eigen::Vector3d
                       { return mapContacts_.contactWithSensor(fsName).wrenchInCentroid.segment<3>(3); });
    logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_" + fsName + "_forceWithUnmodeled",
                       [this, fsName]() -> Eigen::Vector3d
                       {
                         return observer_.getCurrentStateVector().segment<int(observer_.sizeForce)>(
                                    observer_.unmodeledForceIndex())
                                + mapContacts_.contactWithSensor(fsName).wrenchInCentroid.segment<3>(0);
                       });
    logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_" + fsName + "_torqueWithUnmodeled",
                       [this, fsName]() -> Eigen::Vector3d
                       {
                         return observer_.getCurrentStateVector().segment<int(observer_.sizeTorque)>(
                                    observer_.unmodeledTorqueIndex())
                                + mapContacts_.contactWithSensor(fsName).wrenchInCentroid.segment<3>(3);
                       });
    logger.addLogEntry(category_ + "_debug_normForces_" + fsName,
                       [this, fsName]() -> double & { return mapContacts_.contactWithSensor(fsName).normForce; });
  }
  logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_total_force",
                     [this]() -> Eigen::Vector3d { return totalForceCentroid_; });
  logger.addLogEntry(category_ + "_debug_wrenchesInCentroid_total_torque",
                     [this]() -> Eigen::Vector3d { return totalTorqueCentroid_; });

  for(const auto & imu : IMUs_)
  {
    logger.addLogEntry(category_ + "_debug_gyroBias_" + imu.name(),
                       [this, imu]() -> Eigen::Vector3d { return mapIMUs_(imu.name()).gyroBias; });
  }
}

void MCKineticsObserver::addContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName)
{
  const int & numContact = mapContacts_.getNumFromName(contactName);
  if(observer_.getContactIsSetByNum(numContact))
  {
    logger.addLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_position",
                       [this, numContact]() -> Eigen::Vector3d {
                         return observer_.getCurrentStateVector().segment<int(observer_.sizePos)>(
                             observer_.contactPosIndex(numContact));
                       });
    logger.addLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_orientation",
                       [this, numContact]() -> Eigen::Quaternion<double>
                       {
                         so::kine::Orientation ori;
                         return ori
                             .fromVector4(observer_.getCurrentStateVector().segment<int(observer_.sizeOri)>(
                                 observer_.contactOriIndex(numContact)))
                             .inverse()
                             .toQuaternion();
                       });
    logger.addLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_orientation_RollPitchYaw",
                       [this, numContact]() -> so::Vector3
                       {
                         so::kine::Orientation ori;
                         return so::kine::rotationMatrixToRollPitchYaw(
                             ori.fromVector4(observer_.getCurrentStateVector().segment<int(observer_.sizeOri)>(
                                                 observer_.contactOriIndex(numContact)))
                                 .toMatrix3());
                       });
    logger.addLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_forces",
                       [this, numContact]() -> Eigen::Vector3d
                       {
                         return observer_.getCurrentStateVector().segment<int(observer_.sizeForce)>(
                             observer_.contactForceIndex(numContact));
                       });
    logger.addLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_torques",
                       [this, numContact]() -> Eigen::Vector3d
                       {
                         return globalCentroidKinematics_.orientation.toMatrix3()
                                * observer_.getCurrentStateVector().segment<int(observer_.sizeTorque)>(
                                    observer_.contactTorqueIndex(numContact));
                       });
    logger.addLogEntry(
        category_ + "_stateCovariances_contact_" + contactName + "_position_",
        [this, numContact]() -> Eigen::Vector3d
        {
          return observer_.getEKF()
              .getStateCovariance()
              .block<int(observer_.sizePosTangent), int(observer_.sizePosTangent)>(
                  observer_.contactPosIndexTangent(numContact), observer_.contactPosIndexTangent(numContact))
              .diagonal();
        });
    logger.addLogEntry(
        category_ + "_stateCovariances_contact_" + contactName + "_orientation_",
        [this, numContact]() -> Eigen::Vector3d
        {
          return observer_.getEKF()
              .getStateCovariance()
              .block<int(observer_.sizeOriTangent), int(observer_.sizeOriTangent)>(
                  observer_.contactOriIndexTangent(numContact), observer_.contactOriIndexTangent(numContact))
              .diagonal();
        });
    logger.addLogEntry(category_ + "_stateCovariances_contact_" + contactName + "_Force_",
                       [this, numContact]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF()
                             .getStateCovariance()
                             .block<int(observer_.sizeForceTangent), int(observer_.sizeForceTangent)>(
                                 observer_.contactForceIndexTangent(numContact),
                                 observer_.contactForceIndexTangent(numContact))
                             .diagonal();
                       });
    logger.addLogEntry(category_ + "_stateCovariances_contact_" + contactName + "_Torque_",
                       [this, numContact]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF()
                             .getStateCovariance()
                             .block<int(observer_.sizeTorqueTangent), int(observer_.sizeTorqueTangent)>(
                                 observer_.contactTorqueIndexTangent(numContact),
                                 observer_.contactTorqueIndexTangent(numContact))
                             .diagonal();
                       });

    logger.addLogEntry(category_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_position",
                       [this, numContact]() -> Eigen::Vector3d {
                         return predictedGlobalCentroidState_.segment<int(observer_.sizePos)>(
                             observer_.contactPosIndex(numContact));
                       });
    logger.addLogEntry(category_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_orientation",
                       [this, numContact]() -> Eigen::Quaternion<double>
                       {
                         so::kine::Orientation ori;
                         return ori
                             .fromVector4(predictedGlobalCentroidState_.segment<int(observer_.sizeOri)>(
                                 observer_.contactOriIndex(numContact)))
                             .inverse()
                             .toQuaternion();
                       });
    logger.addLogEntry(category_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_forces",
                       [this, numContact]() -> Eigen::Vector3d {
                         return predictedGlobalCentroidState_.segment<int(observer_.sizeForce)>(
                             observer_.contactForceIndex(numContact));
                       });
    logger.addLogEntry(category_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_torques",
                       [this, numContact]() -> Eigen::Vector3d {
                         return predictedGlobalCentroidState_.segment<int(observer_.sizeTorque)>(
                             observer_.contactTorqueIndex(numContact));
                       });

    logger.addLogEntry(category_ + "_innovation_contact_" + contactName + "_position",
                       [this, numContact]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getInnovation().segment<int(observer_.sizePos)>(
                             observer_.contactPosIndexTangent(numContact));
                       });
    logger.addLogEntry(category_ + "_innovation_contact_" + contactName + "_orientation",
                       [this, numContact]() -> Eigen::Quaternion<double>
                       {
                         so::kine::Orientation ori;
                         return ori
                             .fromVector4(observer_.getEKF().getInnovation().segment<int(observer_.sizeOri)>(
                                 observer_.contactOriIndexTangent(numContact)))
                             .inverse()
                             .toQuaternion();
                       });
    logger.addLogEntry(category_ + "_innovation_contact_" + contactName + "_forces",
                       [this, numContact]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getInnovation().segment<int(observer_.sizeForce)>(
                             observer_.contactForceIndexTangent(numContact));
                       });
    logger.addLogEntry(category_ + "_innovation_contact_" + contactName + "_torques",
                       [this, numContact]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getInnovation().segment<int(observer_.sizeTorque)>(
                             observer_.contactTorqueIndexTangent(numContact));
                       });

    logger.addLogEntry(category_ + "_debug_contactWrench_World_" + contactName + "_force",
                       [this, numContact]() -> Eigen::Vector3d
                       { return observer_.getWorldContactWrench(numContact).segment<int(observer_.sizeForce)>(0); });

    logger.addLogEntry(category_ + "_debug_contactWrench_World_" + contactName + "_torque",
                       [this, numContact]() -> Eigen::Vector3d
                       { return observer_.getWorldContactWrench(numContact).segment<int(observer_.sizeTorque)>(3); });

    logger.addLogEntry(category_ + "_debug_contactWrench_Centroid_" + contactName + "_force",
                       [this, numContact]() -> Eigen::Vector3d
                       { return observer_.getCentroidContactWrench(numContact).segment<int(observer_.sizeForce)>(0); });

    logger.addLogEntry(category_ + "_debug_contactWrench_Centroid_" + contactName + "_torque",
                       [this, numContact]() -> Eigen::Vector3d {
                         return observer_.getCentroidContactWrench(numContact).segment<int(observer_.sizeTorque)>(3);
                       });

    logger.addLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputWorldRef_position",
                       [this, numContact]() -> Eigen::Vector3d
                       { return observer_.getWorldContactInputRefPose(numContact).position(); });

    logger.addLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputWorldRef_orientation",
                       [this, numContact]() -> Eigen::Quaternion<double> {
                         return observer_.getWorldContactInputRefPose(numContact).orientation.inverse().toQuaternion();
                       });

    logger.addLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputCentroidContactKine_position",
                       [this, numContact]() -> Eigen::Vector3d
                       { return observer_.getCentroidContactInputPose(numContact).position(); });

    logger.addLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputCentroidContactKine_orientation",
                       [this, numContact]() -> Eigen::Quaternion<double> {
                         return observer_.getCentroidContactInputPose(numContact).orientation.inverse().toQuaternion();
                       });

    logger.addLogEntry(category_ + "_debug_contactPose_" + contactName + "_worldContactPoseFromCentroid_position",
                       [this, numContact]() -> Eigen::Vector3d
                       { return observer_.getWorldContactPose(numContact).position(); });

    logger.addLogEntry(category_ + "_debug_contactPose_" + contactName + "_worldContactPoseFromCentroid_orientation",
                       [this, numContact]() -> Eigen::Quaternion<double>
                       { return observer_.getWorldContactPose(numContact).orientation.inverse().toQuaternion(); });

    logger.addLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputUserContactKine_position",
                       [this, numContact]() -> Eigen::Vector3d
                       { return observer_.getUserContactInputPose(numContact).position(); });

    logger.addLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputUserContactKine_orientation",
                       [this, numContact]() -> Eigen::Quaternion<double>
                       { return observer_.getUserContactInputPose(numContact).orientation.inverse().toQuaternion(); });
    logger.addLogEntry(category_ + "_debug_contactState_isExternalWrench_" + contactName,
                       [this, numContact]() -> int
                       { return int(mapContacts_.contactWithSensor(numContact).isExternalWrench); });
    logger.addLogEntry(category_ + "_debug_contactState_isSet_" + contactName,
                       [this, numContact]() -> int { return int(mapContacts_.contactWithSensor(numContact).isSet); });
  }
}

void MCKineticsObserver::addContactMeasurementsLogEntries(mc_rtc::Logger & logger, const std::string & contactName)
{
  const int & numContact = mapContacts_.getNumFromName(contactName);
  if(observer_.getContactIsSetByNum(numContact))
  {
    logger.addLogEntry(category_ + "_measurements_contacts_force_" + contactName + "_measured",
                       [this, numContact]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getLastMeasurement().segment<int(observer_.sizeForce)>(
                             observer_.getContactMeasIndexByNum(numContact));
                       });
    logger.addLogEntry(category_ + "_measurements_contacts_force_" + contactName + "_predicted",
                       [this, numContact]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getLastPredictedMeasurement().segment<int(observer_.sizeForce)>(
                             observer_.getContactMeasIndexByNum(numContact));
                       });
    logger.addLogEntry(category_ + "_measurements_contacts_force_" + contactName + "_corrected",
                       [this, numContact]() -> Eigen::Vector3d {
                         return correctedMeasurements_.segment<int(observer_.sizeForce)>(
                             observer_.getContactMeasIndexByNum(numContact));
                       });

    logger.addLogEntry(category_ + "_measurements_contacts_torque_" + contactName + "_measured",
                       [this, numContact]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getLastMeasurement().segment<int(observer_.sizeTorque)>(
                             observer_.getContactMeasIndexByNum(numContact) + observer_.sizeForce);
                       });
    logger.addLogEntry(category_ + "_measurements_contacts_torque_" + contactName + "_predicted",
                       [this, numContact]() -> Eigen::Vector3d
                       {
                         return observer_.getEKF().getLastPredictedMeasurement().segment<int(observer_.sizeTorque)>(
                             observer_.getContactMeasIndexByNum(numContact) + observer_.sizeForce);
                       });
    logger.addLogEntry(category_ + "_measurements_contacts_torque_" + contactName + "_corrected",
                       [this, numContact]() -> Eigen::Vector3d
                       {
                         return correctedMeasurements_.segment<int(observer_.sizeTorque)>(
                             observer_.getContactMeasIndexByNum(numContact) + observer_.sizeForce);
                       });
  }
}

void MCKineticsObserver::removeContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName)
{
  logger.removeLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_position");
  logger.removeLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_position");
  logger.removeLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_orientation");
  logger.removeLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_orientation_RollPitchYaw");
  logger.removeLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_forces");
  logger.removeLogEntry(category_ + "_globalWorldCentroidState_contact_" + contactName + "_torques");
  logger.removeLogEntry(category_ + "_stateCovariances_contact_" + contactName + "_position_");
  logger.removeLogEntry(category_ + "_stateCovariances_contact_" + contactName + "_orientation_");
  logger.removeLogEntry(category_ + "_stateCovariances_contact_" + contactName + "_Force_");
  logger.removeLogEntry(category_ + "_stateCovariances_contact_" + contactName + "_Torque_");

  logger.removeLogEntry(category_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_position");
  logger.removeLogEntry(category_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_orientation");
  logger.removeLogEntry(category_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_forces");
  logger.removeLogEntry(category_ + "_predictedGlobalCentroidKinematics_contact_" + contactName + "_torques");

  logger.removeLogEntry(category_ + "_innovation_contact_" + contactName + "_position");
  logger.removeLogEntry(category_ + "_innovation_contact_" + contactName + "_orientation");
  logger.removeLogEntry(category_ + "_innovation_contact_" + contactName + "_forces");
  logger.removeLogEntry(category_ + "_innovation_contact_" + contactName + "_torques");

  logger.removeLogEntry(category_ + "_debug_contactWrench_World_" + contactName + "_force");

  logger.removeLogEntry(category_ + "_debug_contactWrench_World_" + contactName + "_torque");

  logger.removeLogEntry(category_ + "_debug_contactWrench_Centroid_" + contactName + "_force");

  logger.removeLogEntry(category_ + "_debug_contactWrench_Centroid_" + contactName + "_torque");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputWorldRef_position");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputWorldRef_orientation");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputCentroidContactKine_position");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputCentroidContactKine_orientation");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_worldContactPoseFromCentroid_position");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_worldContactPoseFromCentroid_orientation");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputUserContactKine_position");

  logger.removeLogEntry(category_ + "_debug_contactPose_" + contactName + "_inputUserContactKine_orientation");
  logger.removeLogEntry(category_ + "_debug_contactState_isExternalWrench_" + contactName);
  logger.removeLogEntry(category_ + "_debug_contactState_isSet_" + contactName);
  // logger.removeLogEntry(category_ + "_debug_zmp_" + contactName);
}

void MCKineticsObserver::removeContactMeasurementsLogEntries(mc_rtc::Logger & logger, const std::string & contactName)
{
  logger.removeLogEntry(category_ + "_measurements_contacts_force_" + contactName + "_measured");
  logger.removeLogEntry(category_ + "_measurements_contacts_force_" + contactName + "_predicted");
  logger.removeLogEntry(category_ + "_measurements_contacts_force_" + contactName + "_corrected");

  logger.removeLogEntry(category_ + "_measurements_contacts_torque_" + contactName + "_measured");
  logger.removeLogEntry(category_ + "_measurements_contacts_torque_" + contactName + "_predicted");
  logger.removeLogEntry(category_ + "_measurements_contacts_torque_" + contactName + "_corrected");
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("MCKineticsObserver", mc_state_observation::MCKineticsObserver)