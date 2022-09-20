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
: mc_observers::Observer(type, dt), observer_(0,0)
{
  observer_.setSamplingTime(dt);
  observer_.useFiniteDifferencesJacobians(true);
}

void MCKineticsObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  //imuSensor_ = config("imuSensor", ctl.robot().bodySensor().name());
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
  
  initObserverStateVector(robot);

}

void MCKineticsObserver::initObserverStateVector(const mc_rbdyn::Robot & robot)
{
  stateObservation::kine::Orientation initOrientation(Eigen::Matrix3d(robot.posW().rotation().transpose()));

  Eigen::VectorXd initStateVector;
  initStateVector = Eigen::VectorXd::Zero(observer_.getStateSize());
  
  initStateVector.segment<3>(0) = robot.posW().translation();
  initStateVector.segment<4>(3) = initOrientation.toVector4();
  
  //std::cout << std::endl << "robot position: " << std::endl << robot.posW().translation() << std::endl;
  //std::cout << std::endl << "robot orientation: " << std::endl << initOrientation.toVector4() << std::endl;
  initStateVector.segment<3>(7) = robot.velW().linear();
  initStateVector.segment<3>(10) = robot.velW().angular();

  observer_.initWorldCentroidStateVector(initStateVector); //No need to initialize the other variables as only the positions and velocities of the contact reference are used

  //std::cout << std::endl << "initStateVector: " << std::endl << initStateVector << std::endl;
  //std::cout << std::endl << "centroidStateVector: " << std::endl << observer_.getCurrentStateVector() << std::endl;
  //std::cout << std::endl << "centroidKinematics: " << std::endl << observer_.getGlobalCentroidKinematics() << std::endl;
}

bool MCKineticsObserver::run(const mc_control::MCController & ctl)
{
  /*
  using Input = stateObservation::IMUElasticLocalFrameDynamicalSystem::input;
  using State = stateObservation::IMUElasticLocalFrameDynamicalSystem::state;
  */
  const auto & robot = ctl.robot(robot_);

  /** Center of mass (assumes FK, FV and FA are already done) **/
  observer_.setCenterOfMass(robot.com(), robot.comVelocity(), robot.comAcceleration());

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

  
  /** Contacts
   * Note that when we use force sensors, this should be the position of the force sensor!
   */
  updateContacts(robot, findContacts(ctl));

  /* Step once, and return result */
  
  //observer_.setMeasurementInput(inputs_);
  std::cout << std::endl << "time: " << std::endl << observer_.getStateVectorTimeIndex() << std::endl;
  res_ = observer_.update();

  /* Get IMU position from res, and set the free-flyer accordingly. Note that
   * the result of the estimator is a difference from the reference, not an
   * absolute value */
  
  const Eigen::Vector4d & rotVec = res_.segment<4>(observer_.oriIndex());
  Eigen::Quaternion<double> resultRot = Eigen::Quaternion<double>(rotVec);
  sva::PTransformd newAccPos(resultRot.toRotationMatrix().transpose(),
                             res_.segment<3>(observer_.posIndex()));

  const sva::PTransformd & X_0_prev = robot.mbc().bodyPosW[0];
  X_0_fb_.rotation() = newAccPos.rotation() * X_0_prev.rotation();
  X_0_fb_.translation() = newAccPos.rotation().transpose() * X_0_prev.translation() + newAccPos.translation();

  /* Get free-flyer velocity from res */
  sva::MotionVecd newAccVel = sva::MotionVecd::Zero();

  newAccVel.linear() = res_.segment<3>(observer_.linVelIndex());
  newAccVel.angular() = res_.segment<3>(observer_.angVelIndex());

  /* "Inverse velocity" : find velocity of the base that gives you velocity
   * of the accelerometer */

  /* Bring velocity of the IMU to the origin of the joint : we want the
   * velocity of joint 0, so stop one before the first joint */
  sva::PTransformd E_0_prev(X_0_prev.rotation());
  sva::MotionVecd v_prev_0 = robot.mbc().bodyVelW[0];

  v_fb_0_.angular() = newAccVel.angular() + newAccPos.rotation() * v_prev_0.angular();
  v_fb_0_.linear() =
      stateObservation::kine::skewSymmetric(newAccVel.angular()) * newAccPos.rotation() * X_0_prev.translation()
      + newAccVel.linear() + newAccPos.rotation() * v_prev_0.linear();
  return true;
}

void MCKineticsObserver::update(mc_control::MCController & ctl)
{
  auto & robot = ctl.realRobot(robot_);
  robot.posW(X_0_fb_);
  robot.velW(v_fb_0_.vector());
}

void MCKineticsObserver::updateIMUs(const mc_rbdyn::Robot & robot)
{
  unsigned i = 0;
  for(const auto & imu : IMUs_)
  {
    std::cout << "imuname" + imu.name();
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
    //inputs_.segment<3>(Input::linAccIMU) = accIMU.linear() + velIMU.angular().cross(velIMU.linear());

    stateObservation::kine::Kinematics userImuKinematics;
    Eigen::Matrix<double, 3*5+4, 1> imuVector;
    imuVector << accPosW.translation(), oriAccW.toVector4(), velIMU.linear(), velIMU.angular(), accIMU.linear(), accIMU.angular();
    userImuKinematics.fromVector(imuVector, stateObservation::kine::Kinematics::Flags::all);
    observer_.setIMU(robot.bodySensor().linearAcceleration(), robot.bodySensor().angularVelocity(), userImuKinematics, mapIMUs_.getNumFromName(imu.name()));

    ++i;
  }
}

void MCKineticsObserver::addToLogger(const mc_control::MCController &,
                                            mc_rtc::Logger & logger,
                                            const std::string & category)
{
  logger.addLogEntry(category + "_posW", [this]() -> const sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category + "_velW", [this]() -> const sva::MotionVecd & { return v_fb_0_; });
}

void MCKineticsObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_posW");
  logger.removeLogEntry(category + "_velW");
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
  //observer_.setKfe(flexStiffness_.linear().asDiagonal());
  //observer_.setKte(flexStiffness_.angular().asDiagonal());
}

void MCKineticsObserver::flexDamping(const sva::MotionVecd & damping)
{
  flexDamping_ = damping;
  //observer_.setKfv(flexDamping_.linear().asDiagonal());
  //observer_.setKtv(flexDamping_.angular().asDiagonal());
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
  return contactsFound;
}

void MCKineticsObserver::updateContacts(const mc_rbdyn::Robot & robot, std::set<std::string> updatedContacts)
{
  //if(contacts_ == updatedContacts) return;
  std::set<std::string> & oldContacts = contacts_; // alias

  if(verbose_) mc_rtc::log::info("[{}] Contacts changed: {}", name(), mc_rtc::io::to_string(updatedContacts));
  contactPositions_.clear();
  
  for(const auto & updatedContact : updatedContacts)
  {
    const auto & ifs = robot.indirectSurfaceForceSensor(updatedContact);
    const auto & fs = robot.surfaceForceSensor(updatedContact);
    //contactPositions_.push_back(fs.X_p_f());

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
    std::cout << std::endl << "X_0_p.rotation().transpose(): " << std::endl << X_0_p.rotation().transpose() << std::endl;
    std::cout << std::endl << "oriContactW: " << std::endl << oriContactW.getMatrixRefUnsafe()() << std::endl;
    std::cout << std::endl << "oriContactW.toVector4: " << std::endl << oriContactW.toVector4() << std::endl;
    std::cout << std::endl << "oriContactW.toVector4.quaternion: " << std::endl << stateObservation::Quaternion(oriContactW.toVector4()).coeffs() << std::endl;
    sva::MotionVecd accContact =
      contactPos_ * E_p_0 * robot.mbc().bodyAccB[robot.bodyIndexByName(ifs.parentBody())];

      
    /** Use material acceleration, not spatial **/
    // inputs_.segment<3>(Input::linAccIMU) = accIMU.linear() + velIMU.angular().cross(velIMU.linear());
    
    
    stateObservation::kine::Kinematics userContactKine;
    Eigen::Matrix<double, 3*5+4, 1> contactVector;
    contactVector << contactPosW.translation(), oriContactW.toVector4(), velContact.linear(), velContact.angular(), accContact.linear(),  accContact.angular();
    std::cout << std::endl << "contactVector: " << std::endl << contactVector << std::endl;
    userContactKine.fromVector(contactVector, stateObservation::kine::Kinematics::Flags::all);
    std::cout << std::endl << "userContactKine: " << std::endl << userContactKine << std::endl;
    std::cout << std::endl << "userContactKine.quaternion: " << std::endl << userContactKine.orientation.getQuaternionRefUnsafe()().coeffs() << std::endl;

    if (oldContacts.find(updatedContact) != oldContacts.end())  // checks if the contact already exists, if yes, it is updated
    {
      observer_.updateContactWithWrenchSensor(fs.wrench().vector(), userContactKine, mapContacts_.getNumFromName(updatedContact));
    }
    else  // checks if the contact already exists, if no, it is added to the observer
    {
      mapContacts_.insertPair(updatedContact);
      observer_.addContact( userContactKine,
                            mapContacts_.getNumFromName(updatedContact),
                            contactStiffness_.linear().asDiagonal(), 
                            contactDamping_.linear().asDiagonal(), 
                            contactStiffness_.angular().asDiagonal(), 
                            contactDamping_.angular().asDiagonal() );
      observer_.updateContactWithWrenchSensor(fs.wrench().vector(), userContactKine, mapContacts_.getNumFromName(updatedContact)); // not sure about that
    }

  }
  std::set<std::string> diffs;  // List of the contact that were available on last iteration but are not set anymore on the current one
  std::set_difference(oldContacts.begin(), oldContacts.end(), updatedContacts.begin(), updatedContacts.end(), std::inserter(diffs, diffs.end()));
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