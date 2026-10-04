#pragma once
#include <mc_state_observation/measurements/ContactsManager.h>

namespace mc_state_observation::measurements
{

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////

template<typename ContactT>
template<typename OnAddedContact>
void ContactsManager<ContactT>::init(const mc_control::MCController & ctl,
                                     const std::string & robotName,
                                     Configuration conf,
                                     OnAddedContact onAddedContact)
{
  std::visit([this, &ctl, &robotName, onAddedContact](const auto & c)
             { init_manager(ctl, robotName, c, onAddedContact); }, conf);
}

template<typename ContactT>
template<typename OnAddedContact>
void ContactsManager<ContactT>::init_manager(const mc_control::MCController & ctl,
                                             const std::string & robotName,
                                             const ContactsManagerSurfacesConfiguration & conf,
                                             OnAddedContact onAddedContact)
{
  observerName_ = conf.observerName_;
  verbose_ = conf.verbose_;

  contactsDetectionMethod_ = Surfaces;

  contactDetectionThreshold_ = conf.contactDetectionThreshold_;
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

    // we get the name of the force sensor associated to the surface
    const std::string & fsName = robot.frame(surface).forceSensor().name();
    // if the surface is associated to a force sensor (for example LeftFootCenter or RightFootCenter)
    if(robot.surfaceHasForceSensor(surface)) { addContactToManager(fsName, surface, onAddedContact); }
    else // if the surface is not associated to a force sensor, we will fetch the force sensor indirectly attached to
         // the surface
    {
      addContactToManager(fsName, surface, onAddedContact);
    }
  }
}
template<typename ContactT>
template<typename OnAddedContact>
void ContactsManager<ContactT>::init_manager(const mc_control::MCController & ctl,
                                             const std::string & robotName,
                                             const ContactsManagerSensorsConfiguration & conf,
                                             OnAddedContact onAddedContact)
{
  observerName_ = conf.observerName_;
  verbose_ = conf.verbose_;

  contactsDetectionMethod_ = Sensors;

  contactDetectionThreshold_ = conf.contactDetectionThreshold_;

  const auto & robot = ctl.robot(robotName);

  for(auto & forceSensor : robot.forceSensors())
  {
    if(std::find(conf.forceSensorsToOmit_.begin(), conf.forceSensorsToOmit_.end(), forceSensor.name())
       != conf.forceSensorsToOmit_.end())
    {
      continue;
    }
    const std::string & fsName = forceSensor.name();

    addContactToManager(fsName, onAddedContact);
  }
}

template<typename ContactT>
template<typename OnAddedContact>
void ContactsManager<ContactT>::init_manager(const mc_control::MCController &,
                                             const std::string &,
                                             const ContactsManagerSolverConfiguration & conf,
                                             OnAddedContact)
{
  observerName_ = conf.observerName_;
  verbose_ = conf.verbose_;

  contactsDetectionMethod_ = Solver;

  contactDetectionThreshold_ = conf.contactDetectionThreshold_;
}

template<typename ContactT>
template<typename OnNewContact, typename OnMaintainedContact, typename OnRemovedContact, typename OnAddedContact>
void ContactsManager<ContactT>::updateContacts(const mc_control::MCController & ctl,
                                               const std::string & robotName,
                                               OnNewContact onNewContact,
                                               OnMaintainedContact onMaintainedContact,
                                               OnRemovedContact onRemovedContact,
                                               OnAddedContact onAddedContact)
{
  // Reset contact detection
  contactsDetected_ = false;
  for(auto & [_, c] : listContacts_)
  {
    c.wasAlreadySet(c.isSet());
    c.isSet(false);
  }
  // Detection of the contacts depending on the configured mode
  switch(contactsDetectionMethod_)
  {
    case Surfaces:
      findContactsFromSurfaces(ctl, robotName, onNewContact, onMaintainedContact);
      break;
    case Sensors:
      findContactsFromSensors(ctl, robotName, onNewContact, onMaintainedContact);
      break;
    case Solver:
      findContactsFromSolver(ctl, robotName, onNewContact, onMaintainedContact, onAddedContact);
      break;
  }
  // Handle removed contacts
  for(auto & [_, c] : listContacts_)
  {
    if(c.wasAlreadySet() && !c.isSet()) { onRemovedContact(c); }
  }
}

template<typename ContactT>
template<typename OnAddedContact>
inline ContactT & ContactsManager<ContactT>::addContactToManager(const std::string & forceSensorName,
                                                                 const std::string & surface,
                                                                 [[maybe_unused]] OnAddedContact onAddedContact)
{
  const auto [it, inserted] = listContacts_.insert({forceSensorName, ContactT(idx_, forceSensorName, surface)});

  ContactT & contact = (*it).second;
  if(!inserted) { return contact; }

  if constexpr(!std::is_same_v<OnAddedContact, std::nullptr_t>) { onAddedContact(contact); }
  idx_++;

  return contact;
}

template<typename ContactT>
template<typename OnNewContact, typename OnMaintainedContact, typename OnAddedContact>
void ContactsManager<ContactT>::findContactsFromSolver(const mc_control::MCController & ctl,
                                                       const std::string & robotName,
                                                       OnNewContact & onNewContact,
                                                       OnMaintainedContact & onMaintainedContact,
                                                       OnAddedContact & onAddedContact)
{
  const auto & measRobot = ctl.robot(robotName);

  // Filled-up when verbose
  std::string new_contact_set;
  bool show_new_contacts = false;

  auto insert_contact = [&, this](const std::string & surfaceName)
  {
    ContactT & contactWS =
        addContactToManager(measRobot.frame(surfaceName).forceSensor().name(), surfaceName, onAddedContact);
    contactWS.forceNorm(measRobot.frame(surfaceName).wrench().force().norm());
    if(contactWS.forceNorm() > contactDetectionThreshold_)
    {
      contactsDetected_ = true;
      contactWS.isSet(true);
      if(!contactWS.wasAlreadySet())
      {
        show_new_contacts = true;
        onNewContact(contactWS);
      }
      else
      {
        onMaintainedContact(contactWS);
      }
      if(verbose_)
      {
        if(!new_contact_set.empty()) { new_contact_set += ", "; }
        new_contact_set += contactWS.name();
      }
    }
  };

  for(const auto & contact : ctl.solver().contacts())
  {

    const auto & r1 = ctl.robots().robot(contact.r1Index());
    const auto & r2 = ctl.robots().robot(contact.r2Index());
    if(r1.name() == measRobot.name())
    {

      if(r2.mb().nrDof() == 0) { insert_contact(contact.r1Surface()->name()); }
    }
    else if(r2.name() == measRobot.name())
    {
      if(r1.mb().nrDof() == 0) { insert_contact(contact.r2Surface()->name()); }
    }
  }

  if(verbose_ && show_new_contacts) { mc_rtc::log::info("[{}] Contacts changed: {}", observerName_, new_contact_set); }
}

template<typename ContactT>
template<typename OnNewContact, typename OnMaintainedContact>
void ContactsManager<ContactT>::findContactsFromSurfaces(const mc_control::MCController & ctl,
                                                         const std::string & robotName,
                                                         OnNewContact & onNewContact,
                                                         OnMaintainedContact & onMaintainedContact)
{
  const auto & measRobot = ctl.robot(robotName);

  // Filled-up when verbose
  std::string new_contact_set;
  bool show_new_contacts = false;

  for(auto & [_, contact] : contacts())
  {
    const std::string & fsName = contact.forceSensor();
    const mc_rbdyn::ForceSensor & forceSensor = measRobot.forceSensor(fsName);
    contact.forceNorm(forceSensor.wrenchWithoutGravity(measRobot).force().norm());
    if(contact.forceNorm() > contactDetectionThreshold_)
    {
      contactsDetected_ = true;
      contact.isSet(true);
      if(contact.wasAlreadySet()) { onMaintainedContact(contact); }
      else
      {
        show_new_contacts = true;
        onNewContact(contact);
      }
      if(verbose_)
      {
        if(!new_contact_set.empty()) { new_contact_set += ", "; }
        new_contact_set += contact.name();
      }
    }
  }

  if(verbose_ && show_new_contacts) { mc_rtc::log::info("[{}] Contacts changed: {}", observerName_, new_contact_set); }
}

template<typename ContactT>
template<typename OnNewContact, typename OnMaintainedContact>
void ContactsManager<ContactT>::findContactsFromSensors(const mc_control::MCController & ctl,
                                                        const std::string & robotName,
                                                        OnNewContact & onNewContact,
                                                        OnMaintainedContact & onMaintainedContact)
{ findContactsFromSurfaces(ctl, robotName, onNewContact, onMaintainedContact); }

} // namespace mc_state_observation::measurements
