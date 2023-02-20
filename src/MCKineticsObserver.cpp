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

namespace mc_state_observation

{
MCKineticsObserver::MCKineticsObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), observer_(2, 2)
{
  observer_.setSamplingTime(dt);
}

void MCKineticsObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  IMUs_ = config("imuSensor", ctl.robot().bodySensors());
  config("debug", debug_);
  config("verbose", verbose_);

  observer_.setWithUnmodeledWrench(config("withUnmodeledWrench"));
  observer_.useFiniteDifferencesJacobians(config("withFiniteDifferences"));
  observer_.setWithAccelerationEstimation(config("withAccelerationEstimation"));
  observer_.setWithInnovation(config("withInnovation"));
  observer_.useRungeKutta(config("withRungeKutta"));

  config("flexStiffness", flexStiffness_);
  config("flexDamping", flexDamping_);

  // State
  statePositionInitCovariance_ = static_cast<so::Vector3>(config("statePositionInitVariance")).matrix().asDiagonal();
  stateOriInitCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("stateOriInitVariance"));
  stateLinVelInitCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("stateLinVelInitVariance"));
  stateAngVelInitCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("stateAngVelInitVariance"));
  gyroBiasInitCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("gyroBiasInitVariance"));
  unmodeledWrenchInitCovariance_ = so::Matrix6::Identity() * static_cast<double>(config("unmodeledWrenchInitVariance"));
  contactInitCovariance_.setZero();
  contactInitCovariance_.block<3, 3>(0, 0) = static_cast<so::Vector3>(config("contactPositionInitVariance")).matrix().asDiagonal();
  contactInitCovariance_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * static_cast<double>(config("contactOriInitVariance"));
  contactInitCovariance_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * static_cast<double>(config("contactForceInitVariance"));
  contactInitCovariance_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * static_cast<double>(config("contactTorqueInitVariance"));

  // Process //
  statePositionProcessCovariance_ =
      static_cast<so::Vector3>(config("statePositionProcessVariance")).matrix().asDiagonal();
  stateOriProcessCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("stateOriProcessVariance"));
  stateLinVelProcessCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("stateLinVelProcessVariance"));
  stateAngVelProcessCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("stateAngVelProcessVariance"));
  gyroBiasProcessCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("gyroBiasProcessVariance"));
  unmodeledWrenchProcessCovariance_ = so::Matrix6::Identity() * static_cast<double>(config("unmodeledWrenchProcessVariance"));

  contactProcessCovariance_.setZero();
  contactProcessCovariance_.block<3, 3>(0, 0) =
      static_cast<so::Vector3>(config("contactPositionProcessVariance")).matrix().asDiagonal();
  contactProcessCovariance_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * static_cast<double>(config("contactOrientationProcessVariance"));
  contactProcessCovariance_.block<3, 3>(6, 6) =
      static_cast<so::Vector3>(config("contactForceProcessVariance")).matrix().asDiagonal();
  contactProcessCovariance_.block<3, 3>(9, 9) =
      static_cast<so::Vector3>(config("contactTorqueProcessVariance")).matrix().asDiagonal();

  // Sensor //
  positionSensorCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("positionSensorVariance"));
  orientationSensorCoVariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("orientationSensorVariance"));
  acceleroSensorCovariance_ = static_cast<so::Vector3>(config("acceleroSensorVariance")).matrix().asDiagonal();
  gyroSensorCovariance_ = static_cast<so::Vector3>(config("gyroSensorVariance")).matrix().asDiagonal();
  contactSensorCovariance_.setZero();
  contactSensorCovariance_.block<3, 3>(0, 0) =
      static_cast<so::Vector3>(config("forceSensorVariance")).matrix().asDiagonal();
  contactSensorCovariance_.block<3, 3>(3, 3) =
      static_cast<so::Vector3>(config("torqueSensorVariance")).matrix().asDiagonal();

  observer_.setAllCovariances(statePositionInitCovariance_, stateOriInitCovariance_,
        stateLinVelInitCovariance_, stateAngVelInitCovariance_, gyroBiasInitCovariance_, 
        unmodeledWrenchInitCovariance_, contactInitCovariance_, statePositionProcessCovariance_,
        stateOriProcessCovariance_, stateLinVelProcessCovariance_, stateAngVelProcessCovariance_,
        gyroBiasProcessCovariance_, unmodeledWrenchProcessCovariance_,contactProcessCovariance_, 
        positionSensorCovariance_, orientationSensorCoVariance_, acceleroSensorCovariance_,
        gyroSensorCovariance_, contactSensorCovariance_);
}

void MCKineticsObserver::reset(const mc_control::MCController & ctl)
{
  simStarted_ = false;
  ekfIsSet_ = false;

  const auto & robot = ctl.robot(robot_);
  const auto & robotModule = robot.module();

  rbd::MultiBodyGraph mergeMbg(robotModule.mbg);
  std::map<std::string, std::vector<double>> jointPosByName;
  for(int i = 0; i < robotModule.mb.nrJoints(); ++i)
  {
    auto jointName = robotModule.mb.joint(i).name();
    auto jointIndex = static_cast<unsigned long>(robotModule.mb.jointIndexByName(jointName));
    jointPosByName[jointName] = robotModule.mbc.q[jointIndex];
  }

  std::vector<std::string> rootJoints = {};
  int nbJoints = static_cast<int>(robot.mb().joints().size());
  for(int i = 0; i < nbJoints; ++i)
  {
    if(robot.mb().predecessor(i) == 0)
    {
      rootJoints.push_back(robot.mb().joint(i).name());
    }
  }
  for(const auto & joint : rootJoints)
  {
    if(!robot.hasJoint(joint))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("Robot does not have a joint named {}", joint);
    }
    mergeMbg.mergeSubBodies(robotModule.mb.body(0).name(), joint, jointPosByName);
  }

  inertiaWaist_ = mergeMbg.nodeByName(robotModule.mb.body(0).name())->body.inertia();
  mass(ctl.robot(robot_).mass());
  flexStiffness(flexStiffness_);
  flexDamping(flexDamping_);
  if(debug_)
  {
    mc_rtc::log::info("inertiaWaist = {}", inertiaWaist_);
    mc_rtc::log::info("flexStiffness_ = {}", flexStiffness_);
    mc_rtc::log::info("flexDamping_ = {}", flexDamping_);
  }

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  ctl.gui()->addElement({"Robots"}, mc_rtc::gui::Robot("MCKineticsobserver", [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));
  ctl.gui()->addElement({"Robots"}, mc_rtc::gui::Robot("Real", [&ctl]() -> const mc_rbdyn::Robot & { return ctl.realRobot(); }));
}

void MCKineticsObserver::initObserverStateVector(const mc_rbdyn::Robot & robot)
{
  so::kine::Orientation initOrientation;
  initOrientation.setZeroRotation<so::Quaternion>();
  Eigen::VectorXd initStateVector;
  initStateVector = Eigen::VectorXd::Zero(observer_.getStateSize());
  initStateVector.segment<3>(0) = robot.com();
  initStateVector.segment<4>(3) = initOrientation.toVector4();
  initStateVector.segment<3>(7) = robot.comVelocity();

  observer_.setInitWorldCentroidStateVector(initStateVector);
}

bool MCKineticsObserver::run(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);

  Eigen::Matrix<double, 3, 2> initCom;
  initCom << robot.com(), robot.comVelocity();

  /** Center of mass (assumes FK, FV and FA are already done) **/
  
  observer_.setCenterOfMass(robot.com(), robot.comVelocity(), robot.comAcceleration());


    /** Contacts
   * Note that when we use force sensors, this should be the position of the force sensor!
   */
  updateContacts(robot, findContacts(ctl));
  
  if (!simStarted_) // this allows to ignore the first step on which the contacts are still not detected 
                    // and to avoid the jump of the com's position between this step and the following one
  {
    return true;
  }

  // std::cout << std::endl << "Time: " << std::endl << observer_.getEKF().getCurrentTime() << std::endl;

  /** Accelerometers **/
  updateIMUs(robot);


  /** Inertias **/
  /** TODO : Merge inertias into CoM inertia and/or get it from fd() **/
  Eigen::Vector6d inertia;
  Eigen::Matrix3d inertiaAtOrigin =
      sva::inertiaToOrigin(inertiaWaist_.inertia(), mass_, robot.com(), Eigen::Matrix3d::Identity().eval());
  inertia << inertiaAtOrigin(0, 0), inertiaAtOrigin(1, 1), inertiaAtOrigin(2, 2), inertiaAtOrigin(0, 1),
      inertiaAtOrigin(0, 2), inertiaAtOrigin(1, 2);

  observer_.setAngularMomentum(
                rbd::computeCentroidalMomentum(robot.mb(), robot.mbc(), robot.com()).moment(),
                rbd::computeCentroidalMomentumDot(robot.mb(), robot.mbc(), robot.com(), robot.comVelocity()).moment());
  rbd::computeCentroidalMomentumDot(robot.mb(), robot.mbc(), robot.com(), robot.comVelocity());
  observer_.setInertiaMatrix(inertia);

  /* Step once, and return result */

  res_ = observer_.update();

  ekfIsSet_ = true;

  const Eigen::Vector4d & rotVec = res_.segment<4>(observer_.oriIndex());
  Eigen::Quaternion<double> resultRot = Eigen::Quaternion<double>(rotVec);
  sva::PTransformd newAccPos(resultRot.toRotationMatrix().transpose(), res_.segment<3>(observer_.posIndex()));

  so::kine::Kinematics K_0_fb; // floating base in the user frame (world of the controller)
  K_0_fb.position = robot.posW().translation();
  K_0_fb.orientation = so::Matrix3(robot.posW().rotation().transpose());
  K_0_fb.linVel = robot.velW().linear();
  K_0_fb.angVel = robot.velW().angular();

  so::kine::Kinematics mcko_K_0_fb(observer_.getGlobalKinematicsOf(
      K_0_fb)); // Floating base in the 'real' world frame. The resulting state kinematics are used here
  X_0_fb_.rotation() = mcko_K_0_fb.orientation.toMatrix3().transpose();
  X_0_fb_.translation() = mcko_K_0_fb.position();

  /* Bring velocity of the IMU to the origin of the joint : we want the
   * velocity of joint 0, so stop one before the first joint */

  v_fb_0_.angular() = X_0_fb_.rotation()
                      * mcko_K_0_fb.angVel(); //  X_0_fb_.rotation() = mcko_K_0_fb.orientation.toMatrix3().transpose()
  v_fb_0_.linear() = X_0_fb_.rotation() * mcko_K_0_fb.linVel();

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
  contactKinematics_ = observer_.getContactPoses(); // Used only in the logger as debugging help
  predictedAccelerometers_ = observer_.getPredictedAccelerometers(); // Used only in the logger as debugging help

  controlRobotContactKinematics_.clear();
  for(int j = 0; j < maxContacts_; j++)
  {
    so::kine::Kinematics controlContactKine;
    if(observer_.getContactIsSetByNum(j))
    {
      controlContactKine.position =
          robot.indirectSurfaceForceSensor(mapContacts_.getNameFromNum(j)).X_0_f(robot).translation();
      controlContactKine.orientation = so::Matrix3(
          robot.indirectSurfaceForceSensor(mapContacts_.getNameFromNum(j)).X_0_f(robot).rotation().transpose());
    }
    controlRobotContactKinematics_.push_back(
        controlContactKine); // even if the contact is not set, we add an empty Kinematics object to keep the
                             // corresponding with the indexes
  }

  innovation_ = observer_.getEKF().getInnovation(); // Used only in the logger as debugging help

  /* Update of the visual representation (only a visual feature) of the observed robot */
  my_robots_->robot().mbc().q = ctl.realRobot().mbc().q;

  /* Update of the observed robot */
  update(my_robots_->robot());

  return true;
}

void MCKineticsObserver::update(mc_control::MCController & ctl) // this function is called by the pipeline if the update is set to true in the configuration file
{
  auto & realRobot = ctl.realRobot(robot_);
  update(realRobot);
}

void MCKineticsObserver::update(mc_rbdyn::Robot & robot)
{
  robot.posW(X_0_fb_);
  robot.velW(v_fb_0_.vector());
}

void MCKineticsObserver::updateIMUs(const mc_rbdyn::Robot & robot)
{
  unsigned i = 0;
  for(const auto & imu : IMUs_)
  {
    mapIMUs_.insertPair(imu.name());
    /** Position of accelerometer **/
    accPos_ = robot.bodySensor(imu.name()).X_b_s();
    const sva::PTransformd & X_0_p = robot.bodyPosW(robot.bodySensor(imu.name()).parentBody());
    sva::PTransformd accPosW = accPos_ * X_0_p;
    so::kine::Orientation oriAccW(Eigen::Matrix3d(accPosW.rotation().transpose()));
    /** Velocity of accelerometer **/
    sva::MotionVecd velIMU =
        accPos_ * robot.mbc().bodyVelW[robot.bodyIndexByName(robot.bodySensor(imu.name()).parentBody())];

    /** Acceleration of accelerometer **/
    sva::PTransformd E_p_0(Eigen::Matrix3d(X_0_p.rotation().transpose()));
    sva::MotionVecd accIMU =
        accPos_ * E_p_0 * robot.mbc().bodyAccB[robot.bodyIndexByName(robot.bodySensor(imu.name()).parentBody())];
    /** Use material acceleration, not spatial **/
    // inputs_.segment<3>(Input::linAccIMU) = accIMU.linear() + velIMU.angular().cross(velIMU.linear());

    so::kine::Kinematics userImuKinematics;
    Eigen::Matrix<double, 3 * 5 + 4, 1> imuVector;
    imuVector << accPosW.translation(), oriAccW.toVector4(), velIMU.linear(), velIMU.angular(), accIMU.linear(),
        accIMU.angular();
    userImuKinematics.fromVector(imuVector, so::kine::Kinematics::Flags::all);

    //std::cout << std::endl << "y_" << mapIMUs_.getNumFromName(imu.name()) << " : " << std::endl << robot.bodySensor().linearAcceleration() << std::endl;
    observer_.setIMU( robot.bodySensor().linearAcceleration(), 
                      robot.bodySensor().angularVelocity(),
                      acceleroSensorCovariance_,
                      gyroSensorCovariance_,
                      userImuKinematics, 
                      mapIMUs_.getNumFromName(imu.name()));

    ++i;
  }
}

void MCKineticsObserver::addToLogger(const mc_control::MCController & ctl,
                                     mc_rtc::Logger & logger,
                                     const std::string & category)
{
  logger.addLogEntry(category + "_mcko_fb_posW", [this]() -> const sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category + "_mcko_fb_velW", [this]() -> const sva::MotionVecd & { return v_fb_0_; });

  logger.addLogEntry(category + "_constants_mass", [this]() -> const double & { return observer_.getMass(); });
  logger.addLogEntry(category + "_constants_flexStiffness", [this]() -> const sva::MotionVecd & { return flexStiffness_; });
  logger.addLogEntry(category + "_constants_flexDamping", [this]() -> const sva::MotionVecd & { return flexDamping_; });

  /* Plots of the components of the accelerometers prediction */
  /*
  int i = 0;
  for(const auto & imu : IMUs_)
  {
    logger.addLogEntry(category + "_predictedAccelerometersComponents_" + imu.name() + "_gravityComponent", [this, imu,
  i]() -> Eigen::Vector3d { if(ekfIsSet_) { return predictedAccelerometersGravityComponent_.at(i); } else { return
  Eigen::Vector3d::Zero(); }
    });
    logger.addLogEntry(category + "_predictedAccelerometersComponents_" + imu.name() + "_linAccComponent", [this, imu,
  i]() -> Eigen::Vector3d { if(ekfIsSet_) { return predictedWorldIMUsLinAcc_.at(i); } else { return
  Eigen::Vector3d::Zero(); }
    });
    logger.addLogEntry(category + "_predictedAccelerometersComponents_" + imu.name() + "_total", [this, imu, i]() ->
  Eigen::Vector3d { if(ekfIsSet_) { return predictedAccelerometers_.at(i); } else { return Eigen::Vector3d::Zero(); }
    });
    i++;
  }
  */
  /* Plots of the predicted state */
  logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_positionW_", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) {
             return predictedGlobalCentroidState_.segment<observer_.sizePos>(observer_.posIndex()); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_linVelW", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return predictedGlobalCentroidState_.segment<observer_.sizeLinVel>(observer_.linVelIndex()); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_oriW", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { 
            so::kine::Orientation ori;
            return ori.fromVector4(predictedGlobalCentroidState_.segment<observer_.sizeOri>(observer_.oriIndex())).toRotationVector(); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_angVelW", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return predictedGlobalCentroidState_.segment<observer_.sizeAngVel>(observer_.angVelIndex()); } else { return Eigen::Vector3d::Zero(); }});
  for(int j = 0; j < maxContacts_; j++)
  { 
    logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_contact" + std::to_string(j) + "_position", [this, j]() -> Eigen::Vector3d { 
      if(ekfIsSet_) 
      { 
        if (observer_.getContactIsSetByNum(j))
        {
          return predictedGlobalCentroidState_.segment<observer_.sizePos>(observer_.contactPosIndex(j));
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      }
      else 
      { 
        return Eigen::Vector3d::Zero(); 
      }
    });
    logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_contact" + std::to_string(j) + "_orientation", [this, j]() -> Eigen::Vector3d { 
      if(ekfIsSet_) 
      { 
        if (observer_.getContactIsSetByNum(j))
        {
          so::kine::Orientation ori;
          return ori.fromVector4(predictedGlobalCentroidState_.segment<observer_.sizeOri>(observer_.contactOriIndex(j))).toRotationVector();
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      }
      else 
      { 
        return Eigen::Vector3d::Zero(); 
      }
    });
    logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_contact" + std::to_string(j) + "_forces", [this, j]() -> Eigen::Vector3d { 
      if(ekfIsSet_) 
      { 
        if (observer_.getContactIsSetByNum(j))
        {
          return predictedGlobalCentroidState_.segment<observer_.sizeForce>(observer_.contactForceIndex(j));
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      }
      else 
      { 
        return Eigen::Vector3d::Zero(); 
      }
    });
    logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_contact" + std::to_string(j) + "_torques", [this, j]() -> Eigen::Vector3d { 
      if(ekfIsSet_) 
      { 
        if (observer_.getContactIsSetByNum(j))
        {
          return predictedGlobalCentroidState_.segment<observer_.sizeTorque>(observer_.contactTorqueIndex(j));
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      }
      else 
      { 
        return Eigen::Vector3d::Zero(); 
      }
    });
  }
  for(const auto & imu : IMUs_)
  { 
    logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_gyroBias" + imu.name() + "_measured", [this, imu]() -> Eigen::Vector3d { 
      if(ekfIsSet_) { return predictedGlobalCentroidState_.segment<observer_.sizeGyroBias>(observer_.gyroBiasIndexTangent(mapIMUs_.getNumFromName(imu.name()))); } else { return Eigen::Vector3d::Zero(); }
    });
  }


  /* Plots of the estimated and reference contact Kinematics after the update */
  /*
  for(int j = 0; j < maxContacts_; j++)
  {
    logger.addLogEntry(category + "_contactKinematics_" + std::to_string(j) + "_position"  + "_reference", [this, j]()
  -> Eigen::Vector3d { if(ekfIsSet_)
      {
        if (observer_.getContactIsSetByNum(j) && contactKinematics_.at(2*j).position.isSet())
        {
          return contactKinematics_.at(2*j).position();
        }
        else
        {
          return Eigen::Vector3d::Zero();
        }
      }
      else
      {
        return Eigen::Vector3d::Zero();
      }
    });
    logger.addLogEntry(category + "_contactKinematics_" + std::to_string(j) + "_orientation"  + "_reference", [this,
  j]() -> Eigen::Vector3d { if(ekfIsSet_)
      {
        if (observer_.getContactIsSetByNum(j) && contactKinematics_.at(2*j).orientation.isSet())
        {
          return contactKinematics_.at(2*j).orientation.toRotationVector();
        }
        else
        {
          return Eigen::Vector3d::Zero();
        }
      }
      else
      {
        return Eigen::Vector3d::Zero();
      }
    });
    logger.addLogEntry(category + "_contactKinematics_" + std::to_string(j) + "_linVel"  + "_reference", [this, j]() ->
  Eigen::Vector3d { if(ekfIsSet_)
      {
        if (observer_.getContactIsSetByNum(j) && contactKinematics_.at(2*j).linVel.isSet())
        {
          return contactKinematics_.at(2*j).linVel();
        }
        else
        {
          return Eigen::Vector3d::Zero();
        }
      }
      else
      {
        return Eigen::Vector3d::Zero();
      }
    });
    logger.addLogEntry(category + "_contactKinematics_" + std::to_string(j) + "_angVel"  + "_reference", [this, j]() ->
  Eigen::Vector3d { if(ekfIsSet_)
      {
        if (observer_.getContactIsSetByNum(j) && contactKinematics_.at(2*j).angVel.isSet())
        {
          return contactKinematics_.at(2*j).angVel();
        }
        else
        {
          return Eigen::Vector3d::Zero();
        }
      }
      else
      {
        return Eigen::Vector3d::Zero();
      }
    });
    logger.addLogEntry(category + "_contactKinematics_" + std::to_string(j) + "_position"  + "_controlRobot", [this,
  j]() -> Eigen::Vector3d { if(ekfIsSet_)
      {
        if (observer_.getContactIsSetByNum(j) && controlRobotContactKinematics_.at(j).position.isSet())
        {
          return controlRobotContactKinematics_.at(j).position();
        }
        else
        {
          return Eigen::Vector3d::Zero();
        }
      }
      else
      {
        return Eigen::Vector3d::Zero();
      }
    });
    logger.addLogEntry(category + "_contactKinematics_" + std::to_string(j) + "_orientation"  + "_controlRobot", [this,
  j]() -> Eigen::Vector3d { if(ekfIsSet_)
      {
        if (observer_.getContactIsSetByNum(j) && controlRobotContactKinematics_.at(j).orientation.isSet())
        {
          return controlRobotContactKinematics_.at(j).orientation.toRotationVector();
        }
        else
        {
          return Eigen::Vector3d::Zero();
        }
      }
      else
      {
        return Eigen::Vector3d::Zero();
      }
    });
    logger.addLogEntry(category + "_contactKinematics_" + std::to_string(j) + "_position"  + "_fromCorrectedCentroid",
  [this, j]() -> Eigen::Vector3d { if(ekfIsSet_)
      {
        if (observer_.getContactIsSetByNum(j) && contactKinematics_.at(2*j+1).position.isSet())
        {
          return contactKinematics_.at(2*j+1).position();
        }
        else
        {
          return Eigen::Vector3d::Zero();
        }
      }
      else
      {
        return Eigen::Vector3d::Zero();
      }
    });
    logger.addLogEntry(category + "_contactKinematics_" + std::to_string(j) + "_position"  + "_fromCorrectedCentroid",
  [this, j]() -> Eigen::Vector3d { if(ekfIsSet_)
      {
        if (observer_.getContactIsSetByNum(j) && contactKinematics_.at(2*j+1).position.isSet())
        {
          return contactKinematics_.at(2*j+1).position();
        }
        else
        {
          return Eigen::Vector3d::Zero();
        }
      }
      else
      {
        return Eigen::Vector3d::Zero();
      }
    });
    logger.addLogEntry(category + "_contactKinematics_" + std::to_string(j) + "_orientation"  +
  "_fromCorrectedCentroid", [this, j]() -> Eigen::Vector3d { if(ekfIsSet_)
      {
        if (observer_.getContactIsSetByNum(j) && contactKinematics_.at(2*j+1).orientation.isSet())
        {
          return contactKinematics_.at(2*j+1).orientation.toRotationVector();
        }
        else
        {
          return Eigen::Vector3d::Zero();
        }
      }
      else
      {
        return Eigen::Vector3d::Zero();
      }
    });
    logger.addLogEntry(category + "_contactKinematics_" + std::to_string(j) + "_linVel"  + "_fromCorrectedCentroid",
  [this, j]() -> Eigen::Vector3d { if(ekfIsSet_)
      {
        if (observer_.getContactIsSetByNum(j) && contactKinematics_.at(2*j+1).linVel.isSet())
        {
          return contactKinematics_.at(2*j+1).linVel();
        }
        else
        {
          return Eigen::Vector3d::Zero();
        }
      }
      else
      {
        return Eigen::Vector3d::Zero();
      }
    });
    logger.addLogEntry(category + "_contactKinematics_" + std::to_string(j) + "_angVel"  + "_fromCorrectedCentroid",
  [this, j]() -> Eigen::Vector3d { if(ekfIsSet_)
      {
        if (observer_.getContactIsSetByNum(j) && contactKinematics_.at(2*j+1).angVel.isSet())
        {
          return contactKinematics_.at(2*j+1).angVel();
        }
        else
        {
          return Eigen::Vector3d::Zero();
        }
      }
      else
      {
        return Eigen::Vector3d::Zero();
      }
    });
  }
  */
  /* State covariances */
  logger.addLogEntry(category + "_stateCovariances_positionW_", [this]() -> Eigen::Vector3d { 
          return observer_.getEKF().getStateCovariance().block<3,3>(0,0).diagonal();});
  logger.addLogEntry(category + "_stateCovariances_orientationW_", [this]() -> Eigen::Vector3d { 
             return observer_.getEKF().getStateCovariance().block<3,3>(3,3).diagonal();});
  logger.addLogEntry(category + "_stateCovariances_linVelW_", [this]() -> Eigen::Vector3d { 
          return observer_.getEKF().getStateCovariance().block<3,3>(6,6).diagonal();});
  logger.addLogEntry(category + "_stateCovariances_angVelW_", [this]() -> Eigen::Vector3d { 
          return observer_.getEKF().getStateCovariance().block<3,3>(9,9).diagonal();});
  for(int j = 0; j < maxContacts_; j++)
  { 
    logger.addLogEntry(category + "_stateCovariances_contact_" + std::to_string(j) + "_positionW_", [this, j]() -> Eigen::Vector3d { 
        if (observer_.getContactIsSetByNum(j))
        {
          return observer_.getEKF().getStateCovariance().block<3,3>(observer_.contactPosIndexTangent(j),observer_.contactPosIndexTangent(j)).diagonal();
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
    });
    logger.addLogEntry(category + "_stateCovariances_contact_" + std::to_string(j) + "_orientationW_", [this, j]() -> Eigen::Vector3d { 
        if (observer_.getContactIsSetByNum(j))
          {
            return observer_.getEKF().getStateCovariance().block<3,3>(observer_.contactOriIndexTangent(j),observer_.contactOriIndexTangent(j)).diagonal(); 
          } 
          else 
          { 
            return Eigen::Vector3d::Zero(); 
          }
    });
    logger.addLogEntry(category + "_stateCovariances_contact_" + std::to_string(j) + "_ForceW_", [this, j]() -> Eigen::Vector3d { 
        if (observer_.getContactIsSetByNum(j))
        {
          return observer_.getEKF().getStateCovariance().block<3,3>(observer_.contactForceIndexTangent(j),observer_.contactForceIndexTangent(j)).diagonal();
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
    });
    logger.addLogEntry(category + "_stateCovariances_contact_" + std::to_string(j) + "_TorqueW_", [this, j]() -> Eigen::Vector3d { 
        if (observer_.getContactIsSetByNum(j))
        {
          return observer_.getEKF().getStateCovariance().block<3,3>(observer_.contactTorqueIndexTangent(j),observer_.contactTorqueIndexTangent(j)).diagonal();
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
    });
  }


  /* Plots of the updated state */
  logger.addLogEntry(category + "_globalWorldCentroidState_positionW_", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) {
             return globalCentroidKinematics_.position(); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_globalWorldCentroidState_linVelW", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return globalCentroidKinematics_.linVel(); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_globalWorldCentroidState_linAccW", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return globalCentroidKinematics_.linAcc(); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_globalWorldCentroidState_oriW", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return globalCentroidKinematics_.orientation.toRotationVector(); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_globalWorldCentroidState_angVelW", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return globalCentroidKinematics_.angVel(); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_globalWorldCentroidState_angAccW", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return globalCentroidKinematics_.angAcc(); } else { return Eigen::Vector3d::Zero(); }});
          
  for(int j = 0; j < maxContacts_; j++)
  { 
    logger.addLogEntry(category + "_globalWorldCentroidState_contact" + std::to_string(j) + "_position", [this, j]() -> Eigen::Vector3d { 
      if(ekfIsSet_) 
      { 
        if (observer_.getContactIsSetByNum(j))
        {
          return res_.segment<observer_.sizePos>(observer_.contactPosIndex(j));
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      }
      else 
      { 
        return Eigen::Vector3d::Zero(); 
      }
    });
    logger.addLogEntry(category + "_globalWorldCentroidState_contact" + std::to_string(j) + "_orientation", [this, j]() -> Eigen::Vector3d { 
      if(ekfIsSet_) 
      { 
        if (observer_.getContactIsSetByNum(j))
        {
          so::kine::Orientation ori;
          return ori.fromVector4(res_.segment<observer_.sizeOri>(observer_.contactOriIndex(j))).toRotationVector();
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      }
      else 
      { 
        return Eigen::Vector3d::Zero(); 
      }
    });
    logger.addLogEntry(category + "_globalWorldCentroidState_contact" + std::to_string(j) + "_forces", [this, j]() -> Eigen::Vector3d { 
      if(ekfIsSet_) 
      { 
        if (observer_.getContactIsSetByNum(j))
        {
          return res_.segment<observer_.sizeForce>(observer_.contactForceIndex(j));
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      }
      else 
      { 
        return Eigen::Vector3d::Zero(); 
      }
    });
    logger.addLogEntry(category + "_globalWorldCentroidState_contact" + std::to_string(j) + "_torques", [this, j]() -> Eigen::Vector3d { 
      if(ekfIsSet_) 
      { 
        if (observer_.getContactIsSetByNum(j))
        {
          return res_.segment<observer_.sizeTorque>(observer_.contactTorqueIndex(j));
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      }
      else 
      { 
        return Eigen::Vector3d::Zero(); 
      }
    });
  }
  for(const auto & imu : IMUs_)
  { 
    logger.addLogEntry(category + "_globalWorldCentroidState_gyroBias" + imu.name() + "_measured", [this, imu]() -> Eigen::Vector3d { 
      if(ekfIsSet_) { return res_.segment<observer_.sizeGyroBias>(observer_.gyroBiasIndexTangent(mapIMUs_.getNumFromName(imu.name()))); } else { return Eigen::Vector3d::Zero(); }
    });
  }


  /* Plots of the innovation */
  logger.addLogEntry(category + "_innovation_positionW_", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return innovation_.segment<observer_.sizePosTangent>(observer_.posIndexTangent()); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_innovation_linVelW_", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return innovation_.segment<observer_.sizeLinVelTangent>(observer_.linVelIndexTangent()); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_innovation_oriW_", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return innovation_.segment<observer_.sizeOriTangent>(observer_.oriIndexTangent()); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_innovation_angVelW_", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return innovation_.segment<observer_.sizeAngVelTangent>(observer_.angVelIndexTangent()); } else { return Eigen::Vector3d::Zero(); }});

  for(int j = 0; j < maxContacts_; j++)
  { 
    logger.addLogEntry(category + "_innovation_contact" + std::to_string(j) + "_position", [this, j]() -> Eigen::Vector3d { 
      if(ekfIsSet_) 
      { 
        if (observer_.getContactIsSetByNum(j))
        {
          return innovation_.segment<observer_.sizePos>(observer_.contactPosIndexTangent(j));
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      }
      else 
      { 
        return Eigen::Vector3d::Zero(); 
      }
    });
    logger.addLogEntry(category + "_innovation_contact" + std::to_string(j) + "_orientation", [this, j]() -> Eigen::Vector3d { 
      if(ekfIsSet_) 
      { 
        if (observer_.getContactIsSetByNum(j))
        {
          so::kine::Orientation ori;
          return ori.fromVector4(innovation_.segment<observer_.sizeOri>(observer_.contactOriIndexTangent(j))).toRotationVector();
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      }
      else 
      { 
        return Eigen::Vector3d::Zero(); 
      }
    });
    logger.addLogEntry(category + "_innovation_contact" + std::to_string(j) + "_forces", [this, j]() -> Eigen::Vector3d { 
      if(ekfIsSet_) 
      { 
        if (observer_.getContactIsSetByNum(j))
        {
          return innovation_.segment<observer_.sizeForce>(observer_.contactForceIndexTangent(j));
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      }
      else 
      { 
        return Eigen::Vector3d::Zero(); 
      }
    });
    logger.addLogEntry(category + "_innovation_contact" + std::to_string(j) + "_torques", [this, j]() -> Eigen::Vector3d { 
      if(ekfIsSet_) 
      { 
        if (observer_.getContactIsSetByNum(j))
        {
          return innovation_.segment<observer_.sizeTorque>(observer_.contactTorqueIndexTangent(j));
        } 
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      }
      else 
      { 
        return Eigen::Vector3d::Zero(); 
      }
    });
  }


  /* Plots of the measurements */
  {
    for(const auto & imu : IMUs_)
    { 
      logger.addLogEntry(category + "_measurements_gyro" + imu.name() + "_measured", [this, imu]() -> Eigen::Vector3d { 
        if(ekfIsSet_) { return observer_.getEKF().getLastMeasurement().segment<observer_.sizeGyroBias>(observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())) + observer_.sizeAcceleroSignal); } else { return Eigen::Vector3d::Zero(); }
      });
      logger.addLogEntry(category + "_measurements_gyro" + imu.name() + "_predicted", [this, imu]() -> Eigen::Vector3d { 
        if(ekfIsSet_) { return observer_.getEKF().getLastPredictedMeasurement().segment<observer_.sizeGyroBias>(observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())) + observer_.sizeAcceleroSignal); } else { return Eigen::Vector3d::Zero(); }
      });
      logger.addLogEntry(category + "_measurements_gyro" + imu.name() + "_corrected", [this, imu]() -> Eigen::Vector3d { 
        if(ekfIsSet_) { return correctedMeasurements_.segment<observer_.sizeGyroBias>(observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())) + observer_.sizeAcceleroSignal); } else { return Eigen::Vector3d::Zero(); }
      });

      logger.addLogEntry(category + "_measurements_accelerometer" + imu.name() + "_measured", [this, imu]() -> Eigen::Vector3d { 
        if(ekfIsSet_) { return observer_.getEKF().getLastMeasurement().segment<observer_.sizeAcceleroSignal>(observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name()))); } else { return Eigen::Vector3d::Zero(); }
      });
      logger.addLogEntry(category + "_measurements_accelerometer" + imu.name() + "_predicted", [this, imu]() -> Eigen::Vector3d { 
        if(ekfIsSet_) { return observer_.getEKF().getLastPredictedMeasurement().segment<observer_.sizeAcceleroSignal>(observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name()))); } else { return Eigen::Vector3d::Zero(); }
      });
      logger.addLogEntry(category + "_measurements_accelerometer" + imu.name() + "_corrected", [this, imu]() -> Eigen::Vector3d { 
        if(ekfIsSet_) { return correctedMeasurements_.segment<observer_.sizeAcceleroSignal>(observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name()))); } else { return Eigen::Vector3d::Zero(); }
      });
    }
    for(int j = 0; j < maxContacts_; j++)
    { 
      logger.addLogEntry(category + "_measurements_force" + std::to_string(j) + "_measured", [this, j]() -> Eigen::Vector3d { 
        if(ekfIsSet_) 
        { 
          if (observer_.getContactIsSetByNum(j))
          {
            return observer_.getEKF().getLastMeasurement().segment<observer_.sizeForce>(observer_.getContactMeasIndexByNum(j)); 
          } 
          else 
          { 
            return Eigen::Vector3d::Zero(); 
          }
        }
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      });
      logger.addLogEntry(category + "_measurements_force" + std::to_string(j) + "_predicted", [this, j]() -> Eigen::Vector3d { 
        if(ekfIsSet_) 
        { 
          if (observer_.getContactIsSetByNum(j))
          {
            return observer_.getEKF().getLastPredictedMeasurement().segment<observer_.sizeForce>(observer_.getContactMeasIndexByNum(j)); 
          } 
          else 
          { 
            return Eigen::Vector3d::Zero(); 
          }
        }
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      });
      logger.addLogEntry(category + "_measurements_force" + std::to_string(j) + "_corrected", [this, j]() -> Eigen::Vector3d { 
        if(ekfIsSet_) 
        { 
          if (observer_.getContactIsSetByNum(j))
          {
            return correctedMeasurements_.segment<observer_.sizeForce>(observer_.getContactMeasIndexByNum(j)); 
          } 
          else 
          { 
            return Eigen::Vector3d::Zero(); 
          }
        }
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      });

      logger.addLogEntry(category + "_measurements_torque" + std::to_string(j) + "_measured", [this, j]() -> Eigen::Vector3d { 
        if(ekfIsSet_) 
        { 
          if (observer_.getContactIsSetByNum(j))
          {
            return observer_.getEKF().getLastMeasurement().segment<observer_.sizeTorque>(observer_.getContactMeasIndexByNum(j) + observer_.sizeForce);
          } 
          else 
          { 
            return Eigen::Vector3d::Zero(); 
          }
        }
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      });
      logger.addLogEntry(category + "_measurements_torque" + std::to_string(j) + "_predicted", [this, j]() -> Eigen::Vector3d { 
        if(ekfIsSet_) 
        { 
          if (observer_.getContactIsSetByNum(j))
          {
            return observer_.getEKF().getLastPredictedMeasurement().segment<observer_.sizeTorque>(observer_.getContactMeasIndexByNum(j) + observer_.sizeForce);
          } 
          else 
          { 
            return Eigen::Vector3d::Zero(); 
          }
        }
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      });
      logger.addLogEntry(category + "_measurements_torque" + std::to_string(j) + "_corrected", [this, j]() -> Eigen::Vector3d { 
        if(ekfIsSet_) 
        { 
          if (observer_.getContactIsSetByNum(j))
          {
            return correctedMeasurements_.segment<observer_.sizeTorque>(observer_.getContactMeasIndexByNum(j) + observer_.sizeForce);
          } 
          else 
          { 
            return Eigen::Vector3d::Zero(); 
          }
        }
        else 
        { 
          return Eigen::Vector3d::Zero(); 
        }
      });
    }
  }
}

void MCKineticsObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_posW");
  logger.removeLogEntry(category + "_velW");
  logger.removeLogEntry(category + "_mass");
  logger.removeLogEntry(category + "_flexStiffness");
  logger.removeLogEntry(category + "_flexDamping");

  unsigned i = 0;
  for(const auto & imu : IMUs_)
  {
    i++;
    logger.removeLogEntry(category + "_velIMU" + std::to_string(i));
    logger.removeLogEntry(category + "_accIMU" + std::to_string(i));
  }
  for(int j = 0; j < maxContacts_; j++)
  {
    logger.removeLogEntry(category + "_measured_force" + std::to_string(j));
    logger.removeLogEntry(category + "_predicted_force" + std::to_string(j));
    logger.removeLogEntry(category + "_measured_torque" + std::to_string(j));
    logger.removeLogEntry(category + "_predicted_torque" + std::to_string(j));
  }
}

void MCKineticsObserver::addToGUI(const mc_control::MCController &,
                                  mc_rtc::gui::StateBuilder & gui,
                                  const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;
  // clang-format off
  gui.addElement(category,
    make_input_element("Accel Covariance", acceleroSensorCovariance_(0,0)),
    make_input_element("Force Covariance", contactSensorCovariance_(0,0)),
    make_input_element("Gyro Covariance", gyroSensorCovariance_(0,0)),
    make_input_element("Flex Stiffness", flexStiffness_), make_input_element("Flex Damping", flexDamping_),
    Label("contacts", [this]() { return mc_rtc::io::to_string(oldContacts_); }));
  // clang-format on
}

void MCKineticsObserver::mass(double mass)
{
  mass_ = mass;
  observer_.setMass(mass);
}

void MCKineticsObserver::flexStiffness(const sva::MotionVecd & stiffness)
{
  flexStiffness_ = stiffness;
}

void MCKineticsObserver::flexDamping(const sva::MotionVecd & damping)
{
  flexDamping_ = damping;
}

std::set<std::string> MCKineticsObserver::findContacts(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  contactsFound_.clear();
  for(const auto & contact : ctl.solver().contacts())
  {
    //std::cout << std::endl << contact.toStr() << std::endl;
    if(ctl.robots().robot(contact.r1Index()).name() == robot.name())
    {
      if(ctl.robots().robot(contact.r2Index()).mb().joint(0).type() == rbd::Joint::Fixed)
      {
        contactsFound_.insert(contact.r1Surface()->name());
      }
    }
    else if(ctl.robots().robot(contact.r2Index()).name() == robot.name())
    {
      if(ctl.robots().robot(contact.r1Index()).mb().joint(0).type() == rbd::Joint::Fixed)
      {
        contactsFound_.insert(contact.r2Surface()->name());
      }
    }
  }
  if (!contactsFound_.empty() && !simStarted_)  // we start the observation once a contact has been detected.
  {
    simStarted_ = true;
    initObserverStateVector(robot);
  }
  return contactsFound_;
}

void MCKineticsObserver::updateContacts(const mc_rbdyn::Robot & robot, std::set<std::string> updatedContacts)
{
  /** Debugging output **/

  if (updatedContacts.empty())
  {
    if (noContact_ == 0)
    {
      mc_rtc::log::warning("No contact detected");
    }
    if (noContact_ % 1000 == 0)
    {
      mc_rtc::log::warning("No contact detected     :     x" + std::to_string(noContact_) + " iterations");
    }
    noContact_++;
    if (robot.indirectSurfaceForceSensor("LeftFootCenter").wrenchWithoutGravity(robot).vector()(5) > 0.3) // threshold on the force along z
    {
      updatedContacts.insert("LeftFootCenter");  // only to use the contacts even if they are mistakenly not detect by find contacts, to remove after
    }
    if (robot.indirectSurfaceForceSensor("RightFootCenter").wrenchWithoutGravity(robot).vector()(5) > 0.3)
    {
      updatedContacts.insert("RightFootCenter");  // only to use the contacts even if they are mistakenly not detect by find contacts, to remove after
    }


    if (!simStarted_ && !updatedContacts.empty())
    {
      simStarted_ = true;
      initObserverStateVector(robot);
    }
  }
  else
  {
    noContact_ = 0;
  }


        /** Debugging output **/
  if(verbose_ && updatedContacts != oldContacts_) mc_rtc::log::info("[{}] Contacts changed: {}", name(), mc_rtc::io::to_string(updatedContacts));
  contactPositions_.clear();
  
  for(const auto & updatedContact : updatedContacts)
  {
    const auto & ifs = robot.indirectSurfaceForceSensor(updatedContact);
    contactWrenchVector_.segment<3>(0) = ifs.wrenchWithoutGravity(robot).vector().segment<3>(3); // retrieving the torque
    contactWrenchVector_.segment<3>(3) = ifs.wrenchWithoutGravity(robot).vector().segment<3>(0); // retrieving the force

    if (contactWrenchVector_.coeff(5) > maxContactForceZ)
    {
      maxContactForceZ = contactWrenchVector_.coeff(5);
    }
    
  
    /** Position of the contact **/
    sva::PTransformd contactPos_ = ifs.X_p_f();
    sva::PTransformd X_0_p = robot.bodyPosW(ifs.parentBody());
    sva::PTransformd contactPosW = contactPos_ * X_0_p;
    so::kine::Orientation oriContactW(Eigen::Matrix3d(contactPosW.rotation().transpose()));
    /** Velocity of the contact **/
    sva::MotionVecd velContact = 
      contactPos_ * robot.mbc().bodyVelW[robot.bodyIndexByName(ifs.parentBody())];

    /** Acceleration of the contact **/

    sva::PTransformd E_p_0(Eigen::Matrix3d(X_0_p.rotation().transpose()));

    sva::MotionVecd accContact = contactPos_ * E_p_0 * robot.mbc().bodyAccB[robot.bodyIndexByName(ifs.parentBody())];

    /** Use material acceleration, not spatial **/
    // inputs_.segment<3>(Input::linAccIMU) = accIMU.linear() + velIMU.angular().cross(velIMU.linear());

    so::kine::Kinematics userContactKine;
    Eigen::Matrix<double, 3 * 5 + 4, 1> contactVector;
    contactVector << contactPosW.translation(), oriContactW.toVector4(), velContact.linear(), velContact.angular(),
        accContact.linear(), accContact.angular();
    userContactKine.fromVector(contactVector, so::kine::Kinematics::Flags::all);

    if(oldContacts_.find(updatedContact)
       != oldContacts_.end()) // checks if the contact already exists, if yes, it is updated
    {
      observer_.updateContactWithWrenchSensor(contactWrenchVector_, contactSensorCovariance_, userContactKine,
                                              mapContacts_.getNumFromName(updatedContact));
    }
    else // checks if the contact already exists, if no, it is added to the observer
    {
      mapContacts_.insertPair(updatedContact);
      observer_.addContact( userContactKine, 
                            contactInitCovariance_,     //  * (maxContactForceZ / contactWrenchVector_.coeff(5)),
                            contactProcessCovariance_,
                            mapContacts_.getNumFromName(updatedContact),
                            flexStiffness_.linear().asDiagonal(), 
                            flexDamping_.linear().asDiagonal(),
                            flexStiffness_.angular().asDiagonal(), 
                            flexDamping_.angular().asDiagonal());
      observer_.updateContactWithWrenchSensor(contactWrenchVector_, 
                                              contactSensorCovariance_, //contactInitCovariance_.block<6,6>(6,6)
                                              userContactKine,
                                              mapContacts_.getNumFromName(updatedContact));
    }
  }
  std::set<std::string>
      diffs; // List of the contact that were available on last iteration but are not set anymore on the current one
  std::set_difference(oldContacts_.begin(), oldContacts_.end(), updatedContacts.begin(), updatedContacts.end(),
                      std::inserter(diffs, diffs.end()));
  for(const auto & diff : diffs)
  {
    observer_.removeContact(mapContacts_.getNumFromName(diff));
  }

  oldContacts_ = updatedContacts;
  unsigned nbContacts = static_cast<unsigned>(oldContacts_.size());
  // observer_.setContactsNumber(nbContacts);
  if(debug_)
  {
    mc_rtc::log::info("nbContacts = {}", nbContacts);
  }

  // updateNoiseCovariance(); already set in updateContactWithWrenchSensor
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("MCKineticsObserver", mc_state_observation::MCKineticsObserver)