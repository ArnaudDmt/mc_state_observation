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
namespace so = stateObservation;

MCKineticsObserver::MCKineticsObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), observer_(maxContacts_, maxIMUs_)
{
  observer_.setSamplingTime(dt);
  observer_.useFiniteDifferencesJacobians(true);
}

void MCKineticsObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  // imuSensor_ = config("imuSensor", ctl.robot().bodySensor().name());
  IMUs_ = config("imuSensor", ctl.robot().bodySensors());
  config("debug", debug_);
  config("verbose", verbose_);

  config("accelNoiseCovariance", accelNoiseCovariance_);
  config("forceSensorNoiseCovariance", forceSensorNoiseCovariance_);
  config("gyroNoiseCovariance", gyroNoiseCovariance_);
  config("flexStiffness", flexStiffness_);
  config("flexDamping", flexDamping_);
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
  stateObservation::kine::Orientation initOrientation(Eigen::Matrix3d(robot.posW().rotation().transpose()));

  Eigen::VectorXd initStateVector;
  initStateVector = Eigen::VectorXd::Zero(observer_.getStateSize());
  initStateVector.segment<3>(0) = robot.com();
  // initStateVector.segment<3>(0) = robot.posW().translation();
  initStateVector.segment<4>(3) = initOrientation.toVector4();
  initStateVector.segment<3>(7) = robot.comVelocity();

  observer_.initWorldCentroidStateVector(initStateVector);
}

bool MCKineticsObserver::run(const mc_control::MCController & ctl)
{
  /*
  using Input = stateObservation::IMUElasticLocalFrameDynamicalSystem::input;
  using State = stateObservation::IMUElasticLocalFrameDynamicalSystem::state;
  */
  const auto & robot = ctl.robot(robot_);
  Eigen::Matrix<double, 3, 2> initCom;
  initCom << robot.com(), robot.comVelocity();
  std::cout << std::endl << "com: " << std::endl << initCom.transpose() << std::endl;


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
  // inputs_.segment<6>(Input::inertia) = inertia;
  observer_.setInertiaMatrix(inertia);


  /* Step once, and return result */

  // observer_.setMeasurementInput(inputs_);
  std::cout << std::endl << "time ekf: " << std::endl << observer_.getStateVectorTimeIndex() << std::endl;
  res_ = observer_.update();
    
  ekfIsSet_ = true;

  /* Get IMU position from res, and set the free-flyer accordingly. Note that
   * the result of the estimator is a difference from the reference, not an
   * absolute value */

  const Eigen::Vector4d & rotVec = res_.segment<4>(observer_.oriIndex());
  Eigen::Quaternion<double> resultRot = Eigen::Quaternion<double>(rotVec);
  sva::PTransformd newAccPos(resultRot.toRotationMatrix().transpose(),
                              res_.segment<3>(observer_.posIndex()));

  //const sva::PTransformd & X_0_prev = robot.mbc().bodyPosW[0];
  
  //X_0_fb_.rotation() = newAccPos.rotation() * X_0_prev.rotation();
  stateObservation::kine::Kinematics K_0_fb;
  K_0_fb.position = robot.posW().translation();
  K_0_fb.orientation = stateObservation::Matrix3(robot.posW().rotation().transpose());
  K_0_fb.linVel = robot.velW().linear();
  K_0_fb.angVel = robot.velW().angular();

  stateObservation::kine::Kinematics realK_0_fb(observer_.getGlobalKinematicsOf(K_0_fb));
  X_0_fb_.rotation() = realK_0_fb.orientation.toMatrix3().transpose();
  X_0_fb_.translation() = realK_0_fb.position();

  //X_0_fb_.rotation() = observer_.getGlobalCentroidKinematics().orientation.toMatrix3().transpose();
  //X_0_fb_.translation() = newAccPos.rotation().transpose() * X_0_prev.translation() + newAccPos.translation();

  //X_0_fb_.translation() = observer_.getGlobalKinematicsOf();

  /* Get free-flyer velocity from res */
  //sva::MotionVecd newAccVel = sva::MotionVecd::Zero();

  //newAccVel.linear() = res_.segment<3>(observer_.linVelIndex());
  //newAccVel.angular() = res_.segment<3>(observer_.angVelIndex());

  /* "Inverse velocity" : find velocity of the base that gives you velocity
   * of the accelerometer */

  /* Bring velocity of the IMU to the origin of the joint : we want the
   * velocity of joint 0, so stop one before the first joint */

  v_fb_0_.angular() = X_0_fb_.rotation()*realK_0_fb.angVel(); //  X_0_fb_.rotation() = realK_0_fb.orientation.toMatrix3().transpose()
  v_fb_0_.linear() = X_0_fb_.rotation()*realK_0_fb.linVel();
  /*
  sva::PTransformd E_0_prev(X_0_prev.rotation());
  sva::MotionVecd v_prev_0 = robot.mbc().bodyVelW[0];

  v_fb_0_.angular() = newAccVel.angular() + newAccPos.rotation() * v_prev_0.angular();
  v_fb_0_.linear() =
      stateObservation::kine::skewSymmetric(newAccVel.angular()) * newAccPos.rotation() * X_0_prev.translation()
      + newAccVel.linear() + newAccPos.rotation() * v_prev_0.linear();
  */
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
    stateObservation::kine::Orientation oriAccW(Eigen::Matrix3d(accPosW.rotation().transpose()));
    /** Velocity of accelerometer **/
    sva::MotionVecd velIMU =
        accPos_ * robot.mbc().bodyVelW[robot.bodyIndexByName(robot.bodySensor(imu.name()).parentBody())];

    /** Acceleration of accelerometer **/
    sva::PTransformd E_p_0(Eigen::Matrix3d(X_0_p.rotation().transpose()));
    sva::MotionVecd accIMU =
        accPos_ * E_p_0 * robot.mbc().bodyAccB[robot.bodyIndexByName(robot.bodySensor(imu.name()).parentBody())];
    /** Use material acceleration, not spatial **/
    // inputs_.segment<3>(Input::linAccIMU) = accIMU.linear() + velIMU.angular().cross(velIMU.linear());

    stateObservation::kine::Kinematics userImuKinematics;
    Eigen::Matrix<double, 3 * 5 + 4, 1> imuVector;
    imuVector << accPosW.translation(), oriAccW.toVector4(), velIMU.linear(), velIMU.angular(), accIMU.linear(),
        accIMU.angular();
    userImuKinematics.fromVector(imuVector, stateObservation::kine::Kinematics::Flags::all);

    observer_.setIMU(robot.bodySensor().linearAcceleration(), robot.bodySensor().angularVelocity(),
                                                userImuKinematics, mapIMUs_.getNumFromName(imu.name()));

    ++i;
  }
}

void MCKineticsObserver::addToLogger(const mc_control::MCController &,
                                     mc_rtc::Logger & logger,
                                     const std::string & category)
{
  logger.addLogEntry(category + "_posW", [this]() -> const sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category + "_velW", [this]() -> const sva::MotionVecd & { return v_fb_0_; });
  //if (ekfIsSet_)
  {
    unsigned i = 0;
    for(const auto & imu : IMUs_)
    { 
      logger.addLogEntry(category + "_measured_gyro: " + imu.name(), [this, imu]() -> Eigen::Vector3d { 
        if(ekfIsSet_) { return observer_.getEKF().getLastMeasurement().segment<observer_.sizeGyroBias>(observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())) + observer_.sizeAcceleroSignal); } else { return Eigen::Vector3d::Zero(); }
      });
      logger.addLogEntry(category + "_predicted_gyro: " + imu.name(), [this, imu]() -> Eigen::Vector3d { 
        if(ekfIsSet_) { return observer_.getEKF().getLastPredictedMeasurement().segment<observer_.sizeGyroBias>(observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())) + observer_.sizeAcceleroSignal); } else { return Eigen::Vector3d::Zero(); }
      });
      logger.addLogEntry(category + "_measured_acc: " + imu.name(), [this, imu]() -> Eigen::Vector3d { 
        if(ekfIsSet_) { return observer_.getEKF().getLastMeasurement().segment<observer_.sizeAcceleroSignal>(observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name()))); } else { return Eigen::Vector3d::Zero(); }
      });
      logger.addLogEntry(category + "_predicted_acc: " + imu.name(), [this, imu]() -> Eigen::Vector3d { 
        if(ekfIsSet_) { return observer_.getEKF().getLastPredictedMeasurement().segment<observer_.sizeAcceleroSignal>(observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name()))); } else { return Eigen::Vector3d::Zero(); }
      });
      i++;
    }
    for(int j = 0; j < maxContacts_; j++)
    { 
      logger.addLogEntry(category + "_measured_force" + std::to_string(j), [this, j]() -> Eigen::Vector3d { 
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
        //return observer_.getEKF().getLastPredictedMeasurement().segment<observer_.sizeGyroBias>(observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())) + observer_.sizeAcceleroSignal);
      });
      logger.addLogEntry(category + "_predicted_force" + std::to_string(j), [this, j]() -> Eigen::Vector3d { 
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
        //return observer_.getEKF().getLastPredictedMeasurement().segment<observer_.sizeGyroBias>(observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())) + observer_.sizeAcceleroSignal);
      });
      logger.addLogEntry(category + "_measured_torque" + std::to_string(j), [this, j]() -> Eigen::Vector3d { 
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
        //return observer_.getEKF().getLastPredictedMeasurement().segment<observer_.sizeAcceleroSignal>(observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())));
      });
      logger.addLogEntry(category + "_predicted_torque" + std::to_string(j), [this, j]() -> Eigen::Vector3d { 
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
        //return observer_.getEKF().getLastPredictedMeasurement().segment<observer_.sizeAcceleroSignal>(observer_.getIMUMeasIndexByNum(mapIMUs_.getNumFromName(imu.name())));
      });
    }
  }
}

void MCKineticsObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_posW");
  logger.removeLogEntry(category + "_velW");
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
    make_input_element("Accel Covariance", accelNoiseCovariance_),
    make_input_element("Force Covariance", forceSensorNoiseCovariance_),
    make_input_element("Gyro Covariance", gyroNoiseCovariance_),
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
  // observer_.setKfe(flexStiffness_.linear().asDiagonal());
  // observer_.setKte(flexStiffness_.angular().asDiagonal());
}

void MCKineticsObserver::flexDamping(const sva::MotionVecd & damping)
{
  flexDamping_ = damping;
  // observer_.setKfv(flexDamping_.linear().asDiagonal());
  // observer_.setKtv(flexDamping_.angular().asDiagonal());
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
  // if(contacts_ == updatedContacts) return;
  std::set<std::string> & oldContacts = contacts_; // alias

  if(verbose_) mc_rtc::log::info("[{}] Contacts changed: {}", name(), mc_rtc::io::to_string(updatedContacts));
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
    stateObservation::kine::Orientation oriContactW(Eigen::Matrix3d(contactPosW.rotation().transpose()));
    /** Velocity of the contact **/
    sva::MotionVecd velContact = 
      contactPos_ * robot.mbc().bodyVelW[robot.bodyIndexByName(ifs.parentBody())];

    /** Acceleration of the contact **/

    sva::PTransformd E_p_0(Eigen::Matrix3d(X_0_p.rotation().transpose()));

    sva::MotionVecd accContact = contactPos_ * E_p_0 * robot.mbc().bodyAccB[robot.bodyIndexByName(ifs.parentBody())];

    /** Use material acceleration, not spatial **/
    // inputs_.segment<3>(Input::linAccIMU) = accIMU.linear() + velIMU.angular().cross(velIMU.linear());

    stateObservation::kine::Kinematics userContactKine;
    Eigen::Matrix<double, 3 * 5 + 4, 1> contactVector;
    contactVector << contactPosW.translation(), oriContactW.toVector4(), velContact.linear(), velContact.angular(),
        accContact.linear(), accContact.angular();
    userContactKine.fromVector(contactVector, stateObservation::kine::Kinematics::Flags::all);

    if(oldContacts.find(updatedContact)
       != oldContacts.end()) // checks if the contact already exists, if yes, it is updated
    {
      observer_.updateContactWithWrenchSensor(contactWrenchVector_, userContactKine, mapContacts_.getNumFromName(updatedContact)); 
      //observer_.updateContactWithNoSensor(userContactKine, mapContacts_.getNumFromName(updatedContact));
    }
    else // checks if the contact already exists, if no, it is added to the observer
    {
      mapContacts_.insertPair(updatedContact);
      observer_.addContact( userContactKine, 
                            mapContacts_.getNumFromName(updatedContact),
                            flexStiffness_.linear().asDiagonal(), 
                            flexDamping_.linear().asDiagonal(),
                            flexStiffness_.angular().asDiagonal(), 
                            flexDamping_.angular().asDiagonal());
      observer_.updateContactWithWrenchSensor(contactWrenchVector_, userContactKine,
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