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
  observer_.useFiniteDifferencesJacobians(true);
}

void MCKineticsObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  IMUs_ = config("imuSensor", ctl.robot().bodySensors());
  config("debug", debug_);
  config("verbose", verbose_);

  observer_.setWithAccelerationEstimation(config("withAccelerationEstimation"));
  observer_.setWithInnovation(config("withInnovation"));
  
  config("flexStiffness", flexStiffness_);
  config("flexDamping", flexDamping_);

  statePositionInitCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("statePositionInitVariance"));
  stateOriInitCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("stateOriInitVariance"));
  stateLinVelInitCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("stateLinVelInitVariance"));
  stateAngVelInitCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("stateAngVelInitVariance"));
  gyroBiasInitCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("gyroBiasInitVariance"));
  unmodeledWrenchInitCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("unmodeledWrenchInitVariance"));
  contactInitCovariance_.setZero();
  contactInitCovariance_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * static_cast<double>(config("contactPositionInitVariance"));
  contactInitCovariance_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * static_cast<double>(config("contactOriInitVariance"));
  contactInitCovariance_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * static_cast<double>(config("contactForceInitVariance"));
  contactInitCovariance_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * static_cast<double>(config("contactTorqueInitVariance"));

  statePositionProcessCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("statePositionProcessVariance"));
  stateOriProcessCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("stateOriProcessVariance"));
  stateLinVelProcessCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("stateLinVelProcessVariance"));
  stateAngVelProcessCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("stateAngVelProcessVariance"));
  gyroBiasProcessCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("gyroBiasProcessVariance"));
  unmodeledWrenchProcessCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("unmodeledWrenchProcessVariance"));
  contactProcessCovariance_.setZero();
  contactProcessCovariance_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * static_cast<double>(config("contactPositionProcessVariance"));
  contactProcessCovariance_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * static_cast<double>(config("contactOrientationProcessVariance"));
  contactProcessCovariance_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * static_cast<double>(config("contactForceProcessVariance"));
  contactProcessCovariance_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * static_cast<double>(config("contactTorqueProcessVariance"));

  positionSensorCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("positionSensorVariance"));
  orientationSensorCoVariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("orientationSensorVariance"));
  acceleroSensorCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("acceleroSensorVariance"));
  gyroSensorCovariance_ = Eigen::Matrix3d::Identity() * static_cast<double>(config("gyroSensorVariance"));
  contactSensorCovariance_.setZero();
  contactSensorCovariance_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * static_cast<double>(config("forceSensorVariance"));
  contactSensorCovariance_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * static_cast<double>(config("torqueSensorVariance"));
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

  observer_.initWorldCentroidStateVector(initStateVector);
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
  
  if (!simStarted_)
  {
    return true;
  }


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
  sva::PTransformd newAccPos(resultRot.toRotationMatrix().transpose(),
                              res_.segment<3>(observer_.posIndex()));

  so::kine::Kinematics K_0_fb;
  K_0_fb.position = robot.posW().translation();
  K_0_fb.orientation = so::Matrix3(robot.posW().rotation().transpose());
  K_0_fb.linVel = robot.velW().linear();
  K_0_fb.angVel = robot.velW().angular();

  so::kine::Kinematics realK_0_fb(observer_.getGlobalKinematicsOf(K_0_fb));
  X_0_fb_.rotation() = realK_0_fb.orientation.toMatrix3().transpose();
  X_0_fb_.translation() = realK_0_fb.position();

  /* Bring velocity of the IMU to the origin of the joint : we want the
   * velocity of joint 0, so stop one before the first joint */

  v_fb_0_.angular() = X_0_fb_.rotation()*realK_0_fb.angVel(); //  X_0_fb_.rotation() = realK_0_fb.orientation.toMatrix3().transpose()
  v_fb_0_.linear() = X_0_fb_.rotation()*realK_0_fb.linVel();

  /* Updates of the logged variables */
  correctedMeasurements_ = observer_.getEKF().getSimulatedMeasurement(observer_.getEKF().getCurrentTime()); // Used only in the logger as debugging help
  globalCentroidKinematics_ = observer_.getGlobalCentroidKinematics();// Used only in the logger as debugging help
  predictedGlobalCentroidKinematics_ = observer_.getPredictedGlobalCentroidKinematics();// Used only in the logger as debugging help
  predictedAccelerometersGravityComponent_ = observer_.getPredictedAccelerometersGravityComponent();// Used only in the logger as debugging help
  predictedWorldIMUsLinAcc_ = observer_.getPredictedAccelerometersLinAccComponent();// Used only in the logger as debugging help
  predictedAccelerometers_ = observer_.getPredictedAccelerometers();// Used only in the logger as debugging help

  innovation_ = observer_.getEKF().getInnovation();// Used only in the logger as debugging help

  /* Update of the visual representation (only a visual feature) of the observed robot */
  my_robots_->robot().mbc().q = ctl.realRobot().mbc().q;
  update(my_robots_->robot());

  
  return true;
}

void MCKineticsObserver::update(mc_control::MCController & ctl)
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

    observer_.setIMU( robot.bodySensor().linearAcceleration(), 
                      robot.bodySensor().angularVelocity(),
                      acceleroSensorCovariance_,
                      gyroSensorCovariance_,
                      userImuKinematics, 
                      mapIMUs_.getNumFromName(imu.name()));

    ++i;
  }
}

void MCKineticsObserver::addToLogger(const mc_control::MCController &,
                                     mc_rtc::Logger & logger,
                                     const std::string & category)
{
  logger.addLogEntry(category + "_realRobot_posW", [this]() -> const sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category + "_realRobot_velW", [this]() -> const sva::MotionVecd & { return v_fb_0_; });

  logger.addLogEntry(category + "_constants_mass", [this]() -> const double & { return observer_.getMass(); });
  logger.addLogEntry(category + "_constants_flexStiffness", [this]() -> const sva::MotionVecd & { return flexStiffness_; });
  logger.addLogEntry(category + "_constants_flexDamping", [this]() -> const sva::MotionVecd & { return flexDamping_; });

  /* Plots of the components of the accelerometers prediction */
  int i = 0;
  for(const auto & imu : IMUs_)
  { 
    logger.addLogEntry(category + "_predictedAccelerometersComponents_" + imu.name() + "_gravityComponent", [this, imu, i]() -> Eigen::Vector3d { 
      if(ekfIsSet_) { return predictedAccelerometersGravityComponent_.at(i); } else { return Eigen::Vector3d::Zero(); }
    });
    logger.addLogEntry(category + "_predictedAccelerometersComponents_" + imu.name() + "_linAccComponent", [this, imu, i]() -> Eigen::Vector3d { 
      if(ekfIsSet_) { return predictedWorldIMUsLinAcc_.at(i); } else { return Eigen::Vector3d::Zero(); }
    });
    logger.addLogEntry(category + "_predictedAccelerometersComponents_" + imu.name() + "_total", [this, imu, i]() -> Eigen::Vector3d { 
      if(ekfIsSet_) { return predictedAccelerometers_.at(i); } else { return Eigen::Vector3d::Zero(); }
    });
    i++;
  }
  /* Plots of the predicted kinematics of the centroid in the world frame just before the measurements prediction (when switched to the local fram) */
  logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_positionW_", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) {
             return predictedGlobalCentroidKinematics_.position(); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_linVelW", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return predictedGlobalCentroidKinematics_.linVel(); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_linAccW", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return predictedGlobalCentroidKinematics_.linAcc(); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_oriW", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return predictedGlobalCentroidKinematics_.orientation.toRotationVector(); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_angVelW", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return predictedGlobalCentroidKinematics_.angVel(); } else { return Eigen::Vector3d::Zero(); }});
  logger.addLogEntry(category + "_predictedGlobalCentroidKinematics_angAccW", [this]() -> Eigen::Vector3d { 
          if(ekfIsSet_) { return predictedGlobalCentroidKinematics_.angAcc(); } else { return Eigen::Vector3d::Zero(); }});

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
    unsigned i = 0;
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
    Label("contacts", [this]() { return mc_rtc::io::to_string(contacts_); }));
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
  std::set<std::string> contactsFound;
  for(const auto & contact : ctl.solver().contacts())
  {

    if(ctl.robots().robot(contact.r1Index()).name() == robot.name())
    {
      if(ctl.robots().robot(contact.r2Index()).mb().joint(0).type() == rbd::Joint::Fixed)
      {
        contactsFound.insert(contact.r1Surface()->name());
      }
    }
    else if(ctl.robots().robot(contact.r2Index()).name() == robot.name())
    {
      if(ctl.robots().robot(contact.r1Index()).mb().joint(0).type() == rbd::Joint::Fixed)
      {
        contactsFound.insert(contact.r2Surface()->name());
      }
    }
  }
  if (!contactsFound.empty() && !simStarted_)
  {
    simStarted_ = true;
    initObserverStateVector(robot);
  }
  return contactsFound;
}

void MCKineticsObserver::updateContacts(const mc_rbdyn::Robot & robot, std::set<std::string> updatedContacts)
{
  std::set<std::string> & oldContacts = contacts_; // alias

  if(verbose_ && updatedContacts != oldContacts) mc_rtc::log::info("[{}] Contacts changed: {}", name(), mc_rtc::io::to_string(updatedContacts));
  contactPositions_.clear();

  for(const auto & updatedContact : updatedContacts)
  {
    const auto & ifs = robot.indirectSurfaceForceSensor(updatedContact);
    contactWrenchVector_.segment<3>(0) = ifs.wrench().vector().segment<3>(3); // retrieving the torque
    contactWrenchVector_.segment<3>(3) = ifs.wrench().vector().segment<3>(0); // retrieving the force
  
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

    if(oldContacts.find(updatedContact)
       != oldContacts.end()) // checks if the contact already exists, if yes, it is updated
    {
      observer_.updateContactWithWrenchSensor(contactWrenchVector_, 
                                              contactSensorCovariance_,
                                              userContactKine,
                                              mapContacts_.getNumFromName(updatedContact));
    }
    else // checks if the contact already exists, if no, it is added to the observer
    {
      mapContacts_.insertPair(updatedContact);
      observer_.addContact( userContactKine, 
                            contactInitCovariance_,
                            contactProcessCovariance_,
                            mapContacts_.getNumFromName(updatedContact),
                            flexStiffness_.linear().asDiagonal(), 
                            flexDamping_.linear().asDiagonal(),
                            flexStiffness_.angular().asDiagonal(), 
                            flexDamping_.angular().asDiagonal());
      observer_.updateContactWithWrenchSensor(contactWrenchVector_, 
                                              contactSensorCovariance_,
                                              userContactKine,
                                              mapContacts_.getNumFromName(updatedContact));
    }
  }
  std::set<std::string>
      diffs; // List of the contact that were available on last iteration but are not set anymore on the current one
  std::set_difference(oldContacts.begin(), oldContacts.end(), updatedContacts.begin(), updatedContacts.end(),
                      std::inserter(diffs, diffs.end()));
  for(const auto & diff : diffs)
  {
    observer_.removeContact(mapContacts_.getNumFromName(diff));
  }

  oldContacts = updatedContacts;
  unsigned nbContacts = static_cast<unsigned>(oldContacts.size());
  // observer_.setContactsNumber(nbContacts);
  if(debug_)
  {
    mc_rtc::log::info("nbContacts = {}", nbContacts);
  }

  // updateNoiseCovariance(); already set in updateContactWithWrenchSensor
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("MCKineticsObserver", mc_state_observation::MCKineticsObserver)