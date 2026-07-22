#pragma once
#include <mc_control/MCController.h>
#include <mc_state_observation/measurements/ContactsDetectorConfiguration.h>
#include <state-observation/tools/measurements-manager/Contact.hpp>

namespace mc_state_observation::measurements
{

struct SchmittTrigger
{
  double lowerThreshold;
  double upperThreshold;
};

/// @brief Structure that implements all the necessary functions to manage the map of contacts. Handles their detection
/// and updates the list of the detected contacts, newly removed contacts, etc., to apply the appropriate functions on
/// them.
/// @details The template allows to define other kinds of contacts and thus add custom parameters to them.
/// @tparam ContactT Contact, associated to a sensor.
template<typename ContactT>
struct ContactsDetector
{
public:
  using Configuration = std::variant<ContactsDetectorSolverConfiguration,
                                     ContactsDetectorSurfacesConfiguration,
                                     ContactsDetectorSensorsConfiguration>;

  enum ContactsDetection
  {
    Solver,
    Surfaces,
    Sensors,
    Undefined
  };

  static_assert(std::is_base_of_v<stateObservation::measurements::Contact, ContactT>,
                "The template class for the contacts with sensors must inherit from the Contact class");

private:
  // map allowing to get the ContactsDetection value associated to the given string
  inline static const std::unordered_map<std::string, ContactsDetection> strToContactsDetection =
      {{"Solver", Solver}, {"Surfaces", Surfaces}, {"Sensors", Sensors}, {"Undefined", Undefined}};

protected:
  /// @brief Detects the currently set contacts based on the surfaces given by the user. The detection is based on a
  /// thresholding of the force measured by the associated force sensor.
  /// @param ctl Controller
  /// @param robotName Name of the robot
  void findContactsFromSurfaces(const mc_control::MCController & ctl, const std::string & robotName);
  /// @brief Detects the currently set contacts based on a thresholding of the measured forces. Applies custom functions
  /// for each estimator on the newly set and maintained contacts.  The contacts are not required to be given by the
  /// controller (the detection is based on a thresholding of the measured force).
  /// @param ctl Controller
  /// @param robotName Name of the robot
  void findContactsFromSensors(const mc_control::MCController & ctl, const std::string & robotName);

  /// @brief Detects the currently contacts directly from the controller. Applies custom functions for
  /// each estimator on the newly set and maintained contacts, and on the contacts detected for the first time.
  /// @param ctl Controller
  /// @param robotName Name of the robot
  void findContactsFromSolver(const mc_control::MCController & ctl, const std::string & robotName);

public:
  // initialization of the contacts manager
  /// @param ctl Controller
  /// @param robotName Name of the robot
  /// @param conf Configutation of the manager
  void init(const mc_control::MCController & ctl, const std::string & robotName, Configuration conf);

  /// @brief Updates the list of contacts
  /// @param ctl Controller
  /// @param robotName Name of the robot
  /// manager
  /// @return void
  std::unordered_set<std::string> & updateContacts(const mc_control::MCController & ctl, const std::string & robotName);

  /// @brief Get the map of all the contacts
  ///
  /// @return std::unordered_map<std::string, contactsWithSensorT>&
  inline std::unordered_set<std::string> & latestContacts() { return latestContactList_; }

  inline ContactsDetection getContactsDetection() const noexcept { return contactsDetectionMethod_; }

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

  /*! \brief Add the contacts manager to the logger
   *
   * @param category Category in which to log the contacts manager
   */
  void addToLogger(mc_rtc::Logger &, const std::string & category);

private:
  /// @brief Initializer for a contacts detection based on contact surfaces
  /// @param ctl The controller
  /// @param robotName Name of the robot
  inline void init_manager(const mc_control::MCController & ctl,
                           const std::string & robotName,
                           const ContactsDetectorSurfacesConfiguration & conf);
  /// @brief Initializer for a contacts detection based on force sensors
  /// @param ctl The controller
  /// @param robotName Name of the robot
  inline void init_manager(const mc_control::MCController & ctl,
                           const std::string & robotName,
                           const ContactsDetectorSensorsConfiguration & conf);
  /// @brief Initializer for a contacts detection based on the solver's
  /// contacts
  /// @param ctl The controller
  /// @param robotName Name of the robot
  inline void init_manager(const mc_control::MCController & ctl,
                           const std::string & robotName,
                           const ContactsDetectorSolverConfiguration & conf);

protected:
  std::unordered_set<std::string> latestContactList_;

  // method used to detect the contacts
  ContactsDetection contactsDetectionMethod_ = Undefined;
  // threshold for the contacts detection
  // double contactDetectionThreshold_;
  SchmittTrigger schmittTrigger_;

  // list of surfaces used for contacts detection if @contactsDetection_ is
  // set to "Surfaces"
  std::vector<std::string> surfacesForContactDetection_;
};
} // namespace mc_state_observation::measurements

#include <mc_state_observation/measurements/ContactsDetector.hpp>
