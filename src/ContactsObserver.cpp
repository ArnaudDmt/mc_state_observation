#include <mc_control/Contact.h>
#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rbdyn/Contact.h>
#include <mc_state_observation/ContactsObserver.h>
#include <mc_state_observation/gui_helpers.h>

namespace mc_state_observation
{
typedef std::unordered_map<std::string, measurements::ContactWithSensor> ContactsMap;
ContactsObserver::ContactsObserver(const std::string & type, double dt) : mc_observers::Observer(type, dt), hmm_(4, 2)
{
  /// initialization of the HMM
}

ContactsMap & getContactsMap(mc_control::MCController & ctl)
{
  return ctl.datastore().get<ContactsMap>("observers_contactMap");
}

void ContactsObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());

  auto & datastore = (const_cast<mc_control::MCController &>(ctl)).datastore();
  datastore.make<ObserversContactsManager *>("observers_contactMap", &contactsManager_);
}

void ContactsObserver::reset(const mc_control::MCController & ctl) {}

bool ContactsObserver::run(const mc_control::MCController & ctl)
{
  // detects the contacts currently set with the environment
  contactsManager().updateContacts(ctl, robotName_, onNewContact, onMaintainedContact, onRemovedContact,
                                   *runParams.onAddedContactFn);
}

void ContactsObserver::update(mc_control::MCController & ctl) {}

void ContactsObserver::addToLogger(const mc_control::MCController &,
                                   mc_rtc::Logger & logger,
                                   const std::string & category)
{
}

void ContactsObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category) {}

void ContactsObserver::addToGUI(const mc_control::MCController & ctl,
                                mc_rtc::gui::StateBuilder & gui,
                                const std::vector<std::string> & category)
{
}

} // namespace mc_state_observation
EXPORT_OBSERVER_MODULE("ContactsObserver", mc_state_observation::ContactsObserver)
