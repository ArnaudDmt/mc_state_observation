/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/io_utils.h>
#include <mc_rtc/logging.h>
#include <mc_state_observation/NaiveOdometry.h>
#include <mc_state_observation/gui_helpers.h>

#include <RBDyn/CoM.h>
#include <RBDyn/FA.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>

#include <iostream>

namespace mc_state_observation
{
NaiveOdometry::NaiveOdometry(const std::string & type, double dt) : mc_observers::Observer(type, dt) {}

///////////////////////////////////////////////////////////////////////
/// --------------------------Core functions---------------------------
///////////////////////////////////////////////////////////////////////

void NaiveOdometry::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  config("debug", debug_);
  config("verbose", verbose_);

  config("withContactsDetection", withContactsDetection_);
  config("withNaiveYawEstimation", withNaiveYawEstimation_);
  config("contactDetectionPropThreshold", contactDetectionPropThreshold_);
  config("withFilteredForcesContactDetection", withFilteredForcesContactDetection_);

  std::string odometryType = static_cast<std::string>(config("odometryType"));
  if(odometryType == "flatOdometry")
  {
    withFlatOdometry_ = true;
  }
  else if(odometryType != "6dOdometry")
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "Odometry type not allowed. Please pick among : [flatOdometry, 6dOdometry]");
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

  double contactDetectionThreshold = mass_ * so::cst::gravityConstant * contactDetectionPropThreshold_;
  std::vector<std::string> contactsSensorDisabledInit = config("contactsSensorDisabledInit");
  if(contactsDetection_ == "fromSurfaces")
  {
    std::vector<std::string> surfacesForContactDetection = config("surfacesForContactDetection");
    odometryManager_.init(ctl, robot_, "NaiveOdometry", true, true, contactsDetection_, surfacesForContactDetection,
                          contactsSensorDisabledInit, contactDetectionThreshold);
  }
  else
  {
    odometryManager_.init(ctl, robot_, "NaiveOdometry", true, true, contactsDetection_, contactsSensorDisabledInit,
                          contactDetectionThreshold);
  }
}

void NaiveOdometry::reset(const mc_control::MCController & ctl)
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

  mass(ctl.realRobot(robot_).mass());

  my_robots_ = mc_rbdyn::Robots::make();
  my_robots_->robotCopy(robot, robot.name());
  ctl.gui()->addElement(
      {"Robots"},
      mc_rtc::gui::Robot("NaiveOdometry", [this]() -> const mc_rbdyn::Robot & { return my_robots_->robot(); }));

  X_0_fb_.translation() = realRobot.posW().translation();
  X_0_fb_.rotation() = realRobot.posW().rotation();
}

bool NaiveOdometry::run(const mc_control::MCController & ctl)
{
  const auto & robot = ctl.robot(robot_);
  const auto & realRobot = ctl.realRobot(robot_);
  auto & odometryRobot = odometryManager_.odometryRobot();
  auto & logger = (const_cast<mc_control::MCController &>(ctl)).logger();

  odometryManager_.run(ctl, logger, X_0_fb_, v_fb_0_, a_fb_0_);

  /* Update of the visual representation (only a visual feature) of the observed robot */
  my_robots_->robot().mbc().q = ctl.realRobot().mbc().q;
  update(my_robots_->robot());

  return true;
}

///////////////////////////////////////////////////////////////////////
/// -------------------------Called functions--------------------------
///////////////////////////////////////////////////////////////////////

void NaiveOdometry::update(mc_control::MCController & ctl) // this function is called by the pipeline if the
                                                           // update is set to true in the configuration file
{
  auto & realRobot = ctl.realRobot(robot_);
  update(realRobot);
}

void NaiveOdometry::update(mc_rbdyn::Robot & robot)
{
  robot.posW(X_0_fb_);
  robot.velW(v_fb_0_.vector());
  robot.accW(a_fb_0_.vector());
}

void NaiveOdometry::mass(double mass)
{
  mass_ = mass;
}

///////////////////////////////////////////////////////////////////////
/// -------------------------------Logs--------------------------------
///////////////////////////////////////////////////////////////////////

void NaiveOdometry::addToLogger(const mc_control::MCController &, mc_rtc::Logger & logger, const std::string & category)
{
  logger.addLogEntry(category + "_naive_fb_posW", [this]() -> const sva::PTransformd & { return X_0_fb_; });
  logger.addLogEntry(category + "_naive_fb_velW", [this]() -> const sva::MotionVecd & { return v_fb_0_; });
  logger.addLogEntry(category + "_naive_fb_accW", [this]() -> const sva::MotionVecd & { return a_fb_0_; });

  logger.addLogEntry(category + "_naive_fb_yaw",
                     [this]() -> const double
                     { return -so::kine::rotationMatrixToYawAxisAgnostic(X_0_fb_.rotation()); });

  logger.addLogEntry(category + "_constants_forceThreshold",
                     [this]() -> double { return mass_ * so::cst::gravityConstant * contactDetectionPropThreshold_; });
}

void NaiveOdometry::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_posW");
  logger.removeLogEntry(category + "_velW");
  logger.removeLogEntry(category + "_mass");
  logger.removeLogEntry(category + "_flexStiffness");
  logger.removeLogEntry(category + "_flexDamping");
}

void NaiveOdometry::addToGUI(const mc_control::MCController &,
                             mc_rtc::gui::StateBuilder & gui,
                             const std::vector<std::string> & category)
{
  using namespace mc_rtc::gui;
  // clang-format off

  // clang-format on
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("NaiveOdometry", mc_state_observation::NaiveOdometry)