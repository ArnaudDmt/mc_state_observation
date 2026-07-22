/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>
#include <mc_state_observation/LegacyFlexibilityObserver.h>
#include <mc_state_observation/gui_helpers.h>

#include <RBDyn/CoM.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

namespace mc_state_observation

{
namespace so = stateObservation;

LegacyFlexibilityObserver::LegacyFlexibilityObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), observer_(dt)
{
  observer_.setContactModel(SOFlexibilityObserver::contactModel::elasticContact);
  observer_.setWithForcesMeasurements(true);
}

void LegacyFlexibilityObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  imuSensor_ = config("imuSensor", ctl.robot().bodySensor().name());
  config("debug", debug_);
  config("verbose", verbose_);

  config("accelNoiseCovariance", accelNoiseCovariance_);
  config("forceSensorNoiseCovariance", forceSensorNoiseCovariance_);
  config("gyroNoiseCovariance", gyroNoiseCovariance_);
  config("flexStiffness", flexStiffness_);
  config("flexDamping", flexDamping_);
}

void LegacyFlexibilityObserver::reset(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & robotModule = robot.module();

  accPos_ = robot.bodySensor().X_b_s();

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
    if(robot.mb().predecessor(i) == 0) { rootJoints.push_back(robot.mb().joint(i).name()); }
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
}

bool LegacyFlexibilityObserver::run(const mc_control::MCController & ctl)
{
  using Input = stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::input;
  using State = stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::state;
  const auto & robot = ctl.robot(robot_);

  setContacts(robot, findContacts(ctl));

  unsigned nbContacts = static_cast<unsigned>(contacts_.size());

  /* Measurements == sensor output */
  measurements_ = Eigen::VectorXd::Zero(observer_.getMeasurementSize());
  if(measurements_.size() != 6 + 6 * nbContacts)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("[{}] Invalid measurement size", name());
  }
  measurements_.segment<3>(0) = robot.bodySensor().linearAcceleration();
  measurements_.segment<3>(3) = robot.bodySensor().angularVelocity();
  unsigned i = 0;
  for(const auto & contact : contacts_)
  {
    const mc_rbdyn::ForceSensor & fs = robot.surfaceForceSensor(contact);
    measurements_.segment<3>(6 * (i + 1) + 0) = fs.force();
    measurements_.segment<3>(6 * (i + 1) + 3) = fs.couple();
    ++i;
  }
  observer_.setMeasurement(measurements_);

  /* Input = Controller values, all in *world* frame */
  inputs_ = Eigen::VectorXd::Zero(observer_.getInputSize());
  if(inputs_.size() != Input::sizeBase + 12 * nbContacts)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("Invalid input size");
  }

  /** Center of mass (assumes FK, FV and FA are already done) **/
  Eigen::Vector3d posCom = robot.com();
  inputs_.segment<3>(Input::posCom) = posCom;
  inputs_.segment<3>(Input::velCom) = robot.comVelocity();
  inputs_.segment<3>(Input::accCom) = robot.comAcceleration();

  /** Position of accelerometer **/
  sva::PTransformd X_0_p = robot.bodyPosW(robot.bodySensor().parentBody());
  sva::PTransformd accPosW = accPos_ * X_0_p;
  inputs_.segment<3>(Input::posIMU) = accPosW.translation();
  /** Check if its the same as Mehdi **/
  inputs_.segment<3>(Input::oriIMU) =
      stateObservation::kine::rotationMatrixToRotationVector(accPosW.rotation().transpose());

  /** Velocity of accelerometer **/
  sva::MotionVecd velIMU = accPos_ * robot.mbc().bodyVelW[robot.bodyIndexByName(robot.bodySensor().parentBody())];

  inputs_.segment<3>(Input::linVelIMU) = velIMU.linear();
  inputs_.segment<3>(Input::angVelIMU) = velIMU.angular();

  /** Acceleration of accelerometer **/
  Eigen::Matrix3d E_p_0_r = X_0_p.rotation().transpose();
  sva::PTransformd E_p_0(E_p_0_r);
  sva::MotionVecd accIMU =
      accPos_ * E_p_0 * robot.mbc().bodyAccB[robot.bodyIndexByName(robot.bodySensor().parentBody())];

  /** Use material acceleration, not spatial **/
  inputs_.segment<3>(Input::linAccIMU) = accIMU.linear() + velIMU.angular().cross(velIMU.linear());

  /** Inertias **/
  /** TODO : Merge inertias into CoM inertia and/or get it from fd() **/
  Eigen::Vector6d inertia;
  Eigen::Matrix3d inertiaAtOrigin =
      sva::inertiaToOrigin(inertiaWaist_.inertia(), mass_, posCom, Eigen::Matrix3d::Identity().eval());
  inertia << inertiaAtOrigin(0, 0), inertiaAtOrigin(1, 1), inertiaAtOrigin(2, 2), inertiaAtOrigin(0, 1),
      inertiaAtOrigin(0, 2), inertiaAtOrigin(1, 2);
  inputs_.segment<6>(Input::inertia) = inertia;

  /** Contacts
   * Note that when we use force sensors, this should be the position of the force sensor!
   */
  for(unsigned i = 0; i < contacts_.size(); ++i)
  {
    /** Position of contacts **/
    const sva::PTransformd & X_0_c = contactPositions_[i];
    Eigen::Matrix3d tmp = X_0_c.rotation().transpose();
    inputs_.segment<3>(Input::contacts + 12 * i + 0) = X_0_c.translation();
    inputs_.segment<3>(Input::contacts + 12 * i + 3) = sva::rotationVelocity(tmp);

    /** TODO: Speed of contacts **/
    inputs_.segment<3>(Input::contacts + 12 * i + 6) = Eigen::Vector3d::Zero();
    inputs_.segment<3>(Input::contacts + 12 * i + 9) = Eigen::Vector3d::Zero();
  }

  /* Step once, and return result */
  observer_.setMeasurementInput(inputs_);
  res_ = observer_.getFlexibilityVector();

  /* Get IMU position from res, and set the free-flyer accordingly. Note that
   * the result of the estimator is a difference from the reference, not an
   * absolute value */
  Eigen::Vector3d rotVec = res_.segment<3>(State::ori);
  sva::PTransformd newAccPos(stateObservation::kine::rotationVectorToRotationMatrix(rotVec).transpose(),
                             res_.segment<3>(State::pos));

  const sva::PTransformd & X_0_prev = robot.mbc().bodyPosW[0];
  X_0_fb_.rotation() = newAccPos.rotation() * X_0_prev.rotation();
  X_0_fb_.translation() = newAccPos.rotation().transpose() * X_0_prev.translation() + newAccPos.translation();

  /* Get free-flyer velocity from res */
  sva::MotionVecd newAccVel = sva::MotionVecd::Zero();

  /* XXX: Linear speed is not working for the time being :'( */
  // newAccVel.linear() = res_.segment<3>(State::linVel);
  newAccVel.angular() = res_.segment<3>(State::angVel);

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

void LegacyFlexibilityObserver::update(mc_control::MCController & ctl)
{
  auto & robot = ctl.realRobot(robot_);
  robot.posW(X_0_fb_);
  robot.velW(v_fb_0_.vector());
}

void LegacyFlexibilityObserver::addToLogger(const mc_control::MCController &,
                                            mc_rtc::Logger & logger,
                                            const std::string & category)
{
  logger.addLogEntry(category + "_posW", [this]() -> const sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category + "_velW", [this]() -> const sva::MotionVecd & { return v_fb_0_; });
}

void LegacyFlexibilityObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_posW");
  logger.removeLogEntry(category + "_velW");
}

void LegacyFlexibilityObserver::addToGUI(const mc_control::MCController &,
                                         mc_rtc::gui::StateBuilder & gui,
                                         const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;
  // clang-format off
  gui.addElement(category,
    mc_state_observation::gui::make_input_element("Accel Covariance", accelNoiseCovariance_),
    mc_state_observation::gui::make_input_element("Force Covariance", forceSensorNoiseCovariance_),
    mc_state_observation::gui::make_input_element("Gyro Covariance", gyroNoiseCovariance_),
    mc_state_observation::gui::make_input_element("Flex Stiffness", flexStiffness_), mc_state_observation::gui::make_input_element("Flex Damping", flexDamping_),
    Label("contacts", [this]() { return mc_rtc::io::to_string(contacts_); }));
  // clang-format on
}

void LegacyFlexibilityObserver::mass(double mass)
{
  mass_ = mass;
  observer_.setRobotMass(mass);
}

void LegacyFlexibilityObserver::flexStiffness(const sva::MotionVecd & stiffness)
{
  flexStiffness_ = stiffness;
  observer_.setKfe(flexStiffness_.linear().asDiagonal());
  observer_.setKte(flexStiffness_.angular().asDiagonal());
}

void LegacyFlexibilityObserver::flexDamping(const sva::MotionVecd & damping)
{
  flexDamping_ = damping;
  observer_.setKfv(flexDamping_.linear().asDiagonal());
  observer_.setKtv(flexDamping_.angular().asDiagonal());
}

std::set<std::string> LegacyFlexibilityObserver::findContacts(const mc_control::MCController & ctl)
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

void LegacyFlexibilityObserver::setContacts(const mc_rbdyn::Robot & robot, std::set<std::string> contacts)
{
  if(contacts_ == contacts) return;
  contacts_ = contacts;
  if(verbose_) mc_rtc::log::info("[{}] Contacts changed: {}", name(), mc_rtc::io::to_string(contacts_));
  contactPositions_.clear();
  for(const auto & contact : contacts)
  {
    /*
    Correction of an error : the pose of the sensor in the body was used as an input but the input must be its pose in
    the world frame This correction might not be optimal
    */
    const auto & fs = robot.indirectSurfaceForceSensor(contact);
    sva::PTransformd contactPos_ = fs.X_p_f();
    sva::PTransformd X_0_p = robot.bodyPosW(fs.parentBody());
    sva::PTransformd contactPosW = contactPos_ * X_0_p;
    contactPositions_.push_back(contactPosW);
  }
  unsigned nbContacts = static_cast<unsigned>(contacts.size());
  observer_.setContactsNumber(nbContacts);
  if(debug_) { mc_rtc::log::info("nbContacts = {}", nbContacts); }

  updateNoiseCovariance();
}

void LegacyFlexibilityObserver::updateNoiseCovariance()
{
  unsigned nbContacts = static_cast<unsigned>(contacts_.size());

  Eigen::MatrixXd noiseCovariance = observer_.getMeasurementNoiseCovariance();
  if(noiseCovariance.rows() != 6 + 6 * nbContacts || noiseCovariance.cols() != 6 + 6 * nbContacts)
  {
    mc_rtc::log::error_and_throw<std::invalid_argument>("Invalid noise covariance shape");
  }
  noiseCovariance.setIdentity();
  // mesures: IMU accel, IMU gyro, contact: force, torque
  noiseCovariance.diagonal().segment<3>(0) *= accelNoiseCovariance_;
  noiseCovariance.diagonal().segment<3>(3) *= gyroNoiseCovariance_;
  noiseCovariance.diagonal().segment(6, 6 * nbContacts) *= forceSensorNoiseCovariance_;
  // this is covariance, not confidence: the higher, the less you trust
  if(debug_)
  {
    mc_rtc::log::info("Noise covariance shape: {} x {}", noiseCovariance.rows(), noiseCovariance.cols());
    mc_rtc::log::info("noiseCovariance =\n{}", noiseCovariance);
  }
  observer_.setMeasurementNoiseCovariance(noiseCovariance);

  // Update process noise covariance as well
  Eigen::MatrixXd processNoiseCovariance = observer_.getProcessNoiseCovariance();
  using State = stateObservation::flexibilityEstimation::IMUElasticLocalFrameDynamicalSystem::state;
#if 0 // with Mehdi at JRL
    processNoiseCovariance.block<3, 3>(State::fc + 3, State::fc + 3).setIdentity();
    processNoiseCovariance.block<3, 3>(State::fc + 3, State::fc + 3) *= 1e-1;
    processNoiseCovariance.block<3, 3>(State::fc + 6 + 3, State::fc + 6 + 3).setIdentity();
    processNoiseCovariance.block<3, 3>(State::fc + 6 + 3, State::fc + 6 + 3) *= 1e-1;
    processNoiseCovariance.block<3, 3>(State::fc + 3, State::fc + 3).setIdentity();
    processNoiseCovariance.block<3, 3>(State::fc + 3, State::fc + 3) *= 1e-1;
    processNoiseCovariance.block<3, 3>(State::fc + 6 + 3, State::fc + 6 + 3).setIdentity();
    processNoiseCovariance.block<3, 3>(State::fc + 6 + 3, State::fc + 6 + 3) *= 1e-1;
#endif
#if 1 // from https://gite.lirmm.fr/caron/mc_observers/issues/1#note_10040
  Eigen::VectorXd processCovariances(State::size);
  // clang-format off
    processCovariances << 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8,
                       1e-8, 1e-8, 1e-8, 10000, 10000, 10000, 10000, 10000,
                       10000, 10000, 10000, 10000, 10000, 10000, 10000, 1000,
                       1000, 1000, 1000, 1000, 1000, 0, 0, 0, 0, 0;
  // clang-format on
  // processNoiseCovariance.setZero();
  processNoiseCovariance = processCovariances.asDiagonal();
#endif
  if(debug_)
  {
    mc_rtc::log::info("Process noise covariance shape: {} x {}", processNoiseCovariance.rows(),
                      processNoiseCovariance.cols());
    mc_rtc::log::info("processNoiseCovariance =\n{}", processNoiseCovariance);
  }
  observer_.setProcessNoiseCovariance(processNoiseCovariance);
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("LegacyFlexibility", mc_state_observation::LegacyFlexibilityObserver)
