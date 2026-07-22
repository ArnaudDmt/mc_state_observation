#pragma once
#include <mc_rtc/logging.h>
#include <mc_state_observation/measurements/ContactsDetector.h>

namespace mc_state_observation::measurements
{

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////

template<typename ContactT>
void ContactsDetector<ContactT>::init(const mc_control::MCController & ctl,
                                      const std::string & robotName,
                                      Configuration conf)
{
  std::visit(
      [this, &ctl, &robotName](const auto & c)
      {
        if(c.schmittLowerPropThreshold_ && c.schmittUpperPropThreshold_)
        {
          const auto & robot = ctl.robot(robotName);

          schmittTrigger_.lowerThreshold =
              c.schmittLowerPropThreshold_ * robot.mass() * stateObservation::cst::gravityConstant;
          schmittTrigger_.upperThreshold =
              c.schmittUpperPropThreshold_ * robot.mass() * stateObservation::cst::gravityConstant;
        }
        init_manager(ctl, robotName, c);
      },
      conf);
}

template<typename ContactT>
void ContactsDetector<ContactT>::init_manager(const mc_control::MCController & ctl,
                                              const std::string & robotName,
                                              const ContactsDetectorSurfacesConfiguration & conf)
{
  contactsDetectionMethod_ = Surfaces;

  surfacesForContactDetection_ = conf.surfacesForContactDetection_;

  const auto & robot = ctl.robot(robotName);

  if(surfacesForContactDetection_.size() == 0)
  {
    mc_rtc::log::error_and_throw<std::runtime_error>(
        "You selected the contacts detection using surfaces but didn't add the list of surfaces, please add it using "
        "the variable surfacesForContactDetection");
  }

  for(const std::string & surface : surfacesForContactDetection_)
  {
    if(robot.frame(surface).hasForceSensor() == false)
    {
      mc_rtc::log::warning(
          "The surface given for the contact detection is not associated to a force sensor, it will be ignored.");
    }
  }
}
template<typename ContactT>
void ContactsDetector<ContactT>::init_manager(const mc_control::MCController &,
                                              const std::string &,
                                              const ContactsDetectorSensorsConfiguration &)
{
  contactsDetectionMethod_ = Sensors;
}

template<typename ContactT>
void ContactsDetector<ContactT>::init_manager(const mc_control::MCController &,
                                              const std::string &,
                                              const ContactsDetectorSolverConfiguration &)
{
  contactsDetectionMethod_ = Solver;
}

template<typename ContactT>
std::unordered_set<std::string> & ContactsDetector<ContactT>::updateContacts(const mc_control::MCController & ctl,
                                                                             const std::string & robotName)
{
  for(auto it = latestContactList_.begin(); it != latestContactList_.end();)
  {
    const std::string & fsName = ctl.robot(robotName).frame(*it).forceSensor().name();

    if(ctl.robot(robotName).forceSensor(fsName).wrenchWithoutGravity(ctl.realRobot(robotName)).force().z()
       <= schmittTrigger_.lowerThreshold)
    {
      it = latestContactList_.erase(it); // returns next iterator
    }
    else { ++it; }
  }

  // Detection of the contacts depending on the configured mode
  switch(contactsDetectionMethod_)
  {
    case Surfaces:
      findContactsFromSurfaces(ctl, robotName);
      break;
    case Sensors:
      findContactsFromSensors(ctl, robotName);
      break;
    case Solver:
      findContactsFromSolver(ctl, robotName);
      break;
    case Undefined:
      mc_rtc::log::error_and_throw("No contacts detection method was defined.");
      break;
  }
  return latestContactList_;
}

template<typename ContactT>
void ContactsDetector<ContactT>::findContactsFromSolver(const mc_control::MCController & ctl,
                                                        const std::string & robotName)
{
  const auto & measRobot = ctl.robot(robotName);

  for(const auto & contact : ctl.solver().contacts())
  {

    const auto & r1 = ctl.robots().robot(contact.r1Index());
    const auto & r2 = ctl.robots().robot(contact.r2Index());
    if(r1.name() == measRobot.name())
    {

      if(r2.mb().nrDof() == 0) { latestContactList_.insert(contact.r1Surface()->name()); }
    }
    else if(r2.name() == measRobot.name())
    {
      if(r1.mb().nrDof() == 0) { latestContactList_.insert(contact.r2Surface()->name()); }
    }
  }
}

template<typename ContactT>
void ContactsDetector<ContactT>::findContactsFromSurfaces(const mc_control::MCController & ctl,
                                                          const std::string & robotName)
{
  const auto & robot = ctl.robot(robotName);
  const auto & realRobot = ctl.realRobot(robotName);

  for(auto & surface : surfacesForContactDetection_)
  {
    const std::string & fsName = robot.frame(surface).forceSensor().name();
    if(robot.forceSensor(fsName).wrenchWithoutGravity(realRobot).force().z() > schmittTrigger_.upperThreshold)
    {
      latestContactList_.insert(surface);
    }
  }
}

template<typename ContactT>
void ContactsDetector<ContactT>::findContactsFromSensors(const mc_control::MCController & ctl,
                                                         const std::string & robotName)
{
  const auto & robot = ctl.robot(robotName);
  const auto & realRobot = ctl.realRobot(robotName);

  for(auto & forceSensor : robot.forceSensors())
  {
    if(forceSensor.wrenchWithoutGravity(realRobot).force().z() > schmittTrigger_.upperThreshold)
    {
      latestContactList_.insert(forceSensor.name());
    }
  }
}

template<typename ContactT>
void ContactsDetector<ContactT>::addToLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.addLogEntry(category + "_ContactsDetection_schmittTrigger_lowerThreshold",
                     [this]() -> double { return schmittTrigger_.lowerThreshold; });
  logger.addLogEntry(category + "_ContactsDetection_schmittTrigger_upperThreshold",
                     [this]() -> double { return schmittTrigger_.upperThreshold; });
}

} // namespace mc_state_observation::measurements
