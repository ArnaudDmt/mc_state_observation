#pragma once
#include <mc_control/MCController.h>
#include <mc_state_observation/measurements/ContactWithSensor.h>

#include <mc_state_observation/measurements/ContactsManagerConfiguration.h>

namespace mc_state_observation::measurements
{
/// @brief Structure that implements all the necessary functions to manage the map of contacts. Handles their detection
/// and updates the list of the detected contacts, newly removed contacts, etc., to apply the appropriate functions on
/// them.
/// @details The template allows to define other kinds of contacts and thus add custom parameters to them.
/// @tparam ContactT Contact, associated to a sensor.
template<typename ContactT>
struct ContactsManager
{
public:
  // allowed contact detection methods
  enum ContactsDetection
  {
    Solver,
    Surfaces,
    Sensors,
    Undefined
  };

  using Configuration = std::variant<ContactsManagerSolverConfiguration,
                                     ContactsManagerSurfacesConfiguration,
                                     ContactsManagerSensorsConfiguration>;

  static_assert(std::is_base_of_v<ContactWithSensor, ContactT>,
                "The template class for the contacts with sensors must inherit from the ContactWithSensor class");

private:
  // map allowing to get the ContactsDetection value associated to the given string
  inline static const std::unordered_map<std::string, ContactsDetection> strToContactsDetection =
      {{"Solver", Solver}, {"Surfaces", Surfaces}, {"Sensors", Sensors}, {"Undefined", Undefined}};

protected:
  /// @brief Inserts a contact to the map of contacts.
  /// @details Version for contacts that are associated to both a force sensor and a contact surface. The contact will
  /// be named with the name of the force sensor.
  /// @param forceSensorName The name of the force sensor.
  /// @param surface The name of the surface that will be used also to name the contact.
  /// @param onAddedContact function to call when a contact is added to the manager
  /// @return ContactT &
  template<typename OnAddedContact = std::nullptr_t>
  inline ContactT & addContactToManager(const std::string & forceSensorName,
                                        const std::string & surface,
                                        OnAddedContact onAddedContact = nullptr);
  /// @brief Insert a contact to the map of contacts.
  /// @details Version for contacts that are associated to a force sensor but to no surface.
  /// @param name The name of the contact (= name of the sensor)
  /// @param onAddedContact function to call when a contact is added to the manager
  /// @return ContactT &
  template<typename OnAddedContact = std::nullptr_t>
  inline ContactT & addContactToManager(const std::string & forceSensorName, OnAddedContact onAddedContact = nullptr)
  {
    return addContactToManager(forceSensorName, "", onAddedContact);
  }

  /// @brief Detects the currently set contacts based on the surfaces given by the user. Applies custom functions for
  /// each estimator on the newly set and maintained contacts.
  /// @details Called by \ref updateContacts(const mc_control::MCController &, const std::string & robotName,
  /// OnNewContact, OnMaintainedContact, OnRemovedContact , OnAddedContact) if @contactsDetection_ is equal to
  /// "Surfaces". The detection is based on a thresholding of the force measured by the associated force sensor.
  /// @param ctl Controller
  /// @param robotName Name of the robot
  /// @param onNewContact Function to call when a contact just got set with the environment
  /// @param onMaintainedContact Function to call when a contact is maintained with the environment
  template<typename OnNewContact, typename OnMaintainedContact>
  void findContactsFromSurfaces(const mc_control::MCController & ctl,
                                const std::string & robotName,
                                OnNewContact & onNewContact,
                                OnMaintainedContact & onMaintainedContact);
  /// @brief Detects the currently set contacts based on a thresholding of the measured forces. Applies custom functions
  /// for each estimator on the newly set and maintained contacts.
  /// @details Called by \ref updateContacts(const mc_control::MCController &, const std::string & robotName,
  /// OnNewContact, OnMaintainedContact, OnRemovedContact , OnAddedContact) if @contactsDetection_ is equal to
  /// "Sensors". The contacts are not required to be given by the controller (the detection is based on a thresholding
  /// of the measured force).
  /// @param ctl Controller
  /// @param robotName Name of the robot
  /// @param onNewContact Function to call when a contact just got set with the environment
  /// @param onMaintainedContact Function to call when a contact is maintained with the environment
  template<typename OnNewContact, typename OnMaintainedContact>
  void findContactsFromSensors(const mc_control::MCController & ctl,
                               const std::string & robotName,
                               OnNewContact & onNewContact,
                               OnMaintainedContact & onMaintainedContact);

  /// @brief Detects the currently contacts directly from the controller. Applies custom functions for
  /// each estimator on the newly set and maintained contacts, and on the contacts detected for the first time.
  /// @details Called by \ref updateContacts(const mc_control::MCController &, const std::string & robotName,
  /// OnNewContact, OnMaintainedContact, OnRemovedContact , OnAddedContact) if @contactsDetection_ is equal to
  /// "Solver". The contacts are given by the controller directly (then
  /// thresholded based on the measured force).
  /// @param ctl Controller
  /// @param robotName Name of the robot
  /// @param onNewContact Function to call when a contact just got set with the environment
  /// @param onMaintainedContact Function to call when a contact is maintained with the environment
  /// @param onAddedContact Function to call when a contact is added to the manager (detected for the first time)
  template<typename OnNewContact, typename OnMaintainedContact, typename OnAddedContact = std::nullptr_t>
  void findContactsFromSolver(const mc_control::MCController & ctl,
                              const std::string & robotName,
                              OnNewContact & onNewContact,
                              OnMaintainedContact & onMaintainedContact,
                              OnAddedContact & onAddedContact = nullptr);

public:
  // initialization of the contacts manager
  /// @param ctl Controller
  /// @param robotName Name of the robot
  /// @param conf Configutation of the manager
  /// @param onAddedContact function to call when a contact is added to the
  /// manager
  template<typename OnAddedContact = std::nullptr_t>
  void init(const mc_control::MCController & ctl,
            const std::string & robotName,
            Configuration conf,
            OnAddedContact onAddedContact = nullptr);

  /// @brief Updates the list of contacts to inform whether they are newly
  /// set, removed, etc., and execute actions accordingly
  /// @param ctl Controller
  /// @param robotName Name of the robot
  /// @param onNewContact Function to call on a newly detected contact
  /// @param onNewContact Function to call on a newly detected contact
  /// @param onMaintainedContact Function to call on a contact maintainted
  /// since the last iteration
  /// @param onRemovedContact Function to call on a removed contact
  /// @param onAddedContact function to call when a contact is added to the
  /// manager
  /// @return void
  template<typename OnNewContact,
           typename OnMaintainedContact,
           typename OnRemovedContact,
           typename OnAddedContact = std::nullptr_t>
  void updateContacts(const mc_control::MCController & ctl,
                      const std::string & robotName,
                      OnNewContact onNewContact,
                      OnMaintainedContact onMaintainedContact,
                      OnRemovedContact onRemovedContact,
                      OnAddedContact onAddedContact = nullptr);

  /// @brief Accessor for the a contact associated to a sensor contained in
  /// the map
  ///
  /// @param name The name of the contact to access
  /// @return contactsWithSensorT&
  inline ContactT & contact(const std::string & name) { return listContacts_.at(name); }

  /// @brief Get the map of all the contacts
  ///
  /// @return std::unordered_map<std::string, contactsWithSensorT>&
  inline std::unordered_map<std::string, ContactT> & contacts() { return listContacts_; }

  inline ContactsDetection getContactsDetection() const noexcept { return contactsDetectionMethod_; }

  /** Returns true if any contact is detected */
  inline bool contactsDetected() const noexcept { return contactsDetected_; }

  /// @brief Sets the contacts detection method used in the odometry.
  /// @details Allows to set the contacts detection method directly from a
  /// string, most likely obtained from a configuration file.
  /// @param str The string naming the method to be used.
  inline static ContactsDetection stringToContactsDetection(const std::string & str, const std::string & observerName)
  {
    auto it = strToContactsDetection.find(str);
    if(it != strToContactsDetection.end()) { return it->second; }
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}]: No known ContactsDetection value for {}", observerName,
                                                     str);
  }

  ContactT * findContact(const std::string & name)
  {
    auto it = listContacts_.find(name);
    if(it != listContacts_.end()) { return &(it->second); }
    return nullptr;
  }

private:
  /// @brief Initializer for a contacts detection based on contact surfaces
  /// @param ctl The controller
  /// @param robotName Name of the robot
  /// @param conf Configuration of the contacts manager
  /// @param onAddedContact Function to call when a contact is added to the
  /// manager
  template<typename OnAddedContact = std::nullptr_t>
  inline void init_manager(const mc_control::MCController & ctl,
                           const std::string & robotName,
                           const ContactsManagerSurfacesConfiguration & conf,
                           OnAddedContact onAddedContact = nullptr);
  /// @brief Initializer for a contacts detection based on force sensors
  /// @param ctl The controller
  /// @param robotName Name of the robot
  /// @param conf Configuration of the contacts manager
  /// @param onAddedContact Function to call when a contact is added to the
  /// manager
  template<typename OnAddedContact = std::nullptr_t>
  inline void init_manager(const mc_control::MCController & ctl,
                           const std::string & robotName,
                           const ContactsManagerSensorsConfiguration & conf,
                           OnAddedContact onAddedContact = nullptr);
  /// @brief Initializer for a contacts detection based on the solver's
  /// contacts
  /// @param ctl The controller
  /// @param robotName Name of the robot
  /// @param conf Configuration of the contacts manager
  /// @param onAddedContact Function to call when a contact is added to the
  /// manager. Unused here as contacts are added to the manager during the
  /// run with this detection method, but added anyway to match the syntax
  /// of the other init_manager variants.
  template<typename OnAddedContact = std::nullptr_t>
  inline void init_manager(const mc_control::MCController & ctl,
                           const std::string & robotName,
                           const ContactsManagerSolverConfiguration & conf,
                           OnAddedContact onAddedContact = nullptr);

protected:
  // map of contacts used by the manager.
  // unordered map containing all the contacts
  std::unordered_map<std::string, ContactT> listContacts_;
  // Index generator, incremented everytime a new contact is created
  unsigned idx_ = 0;

  // method used to detect the contacts
  ContactsDetection contactsDetectionMethod_ = Undefined;
  // threshold for the contacts detection
  double contactDetectionThreshold_;

  // list of surfaces used for contacts detection if @contactsDetection_ is
  // set to "Surfaces"
  std::vector<std::string> surfacesForContactDetection_;

  // name of the observer using this contacts manager.
  std::string observerName_;

  bool verbose_ = true;

  /** True if any contact is detected, false otherwise */
  bool contactsDetected_ = false;
};
} // namespace mc_state_observation::measurements

#include <mc_state_observation/measurements/ContactsManager.hpp>
