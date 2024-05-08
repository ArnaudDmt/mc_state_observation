#include <string>
#include <variant>
#include <vector>

namespace mc_state_observation::measurements
{
namespace internal
{

/// @brief Configuration structure that helps setting up the parameters of a ContactsManager.
/// @details The configuration is used once passed in the ContactManager's init function
template<class ConfigurationType>
struct ContactsManagerConfigurationPrvt
{
  inline ContactsManagerConfigurationPrvt(const std::string & observerName) noexcept : observerName_(observerName) {}

  inline ConfigurationType & contactDetectionPropThreshold(double contactDetectionPropThreshold) noexcept
  {
    contactDetectionPropThreshold_ = contactDetectionPropThreshold;
    return static_cast<ConfigurationType &>(*this);
  }
  inline ConfigurationType & verbose(bool verbose) noexcept
  {
    verbose_ = verbose;
    return static_cast<ConfigurationType &>(*this);
  }

  std::string observerName_;

  double contactDetectionPropThreshold_ = 0.11;
  bool verbose_ = true;
};
} // namespace internal

/// @brief Configuration structure that helps setting up the parameters of a ContactsManager whose contacts detection
/// method is based on contact surfaces.
/// @details The configuration is used once passed in the ContactManager's init function
struct ContactsManagerSurfacesConfiguration
: public internal::ContactsManagerConfigurationPrvt<ContactsManagerSurfacesConfiguration>
{
  inline ContactsManagerSurfacesConfiguration(const std::string & observerName,
                                              const std::vector<std::string> & surfacesForContactDetection) noexcept
  : ContactsManagerConfigurationPrvt<ContactsManagerSurfacesConfiguration>(observerName),
    surfacesForContactDetection_(surfacesForContactDetection)
  {
  }

  // list of admissible contact surfaces for the detection
  std::vector<std::string> surfacesForContactDetection_;
};

/// @brief Configuration structure that helps setting up the parameters of a ContactsManager whose contacts detection
/// method is based on the force sensor measurements.
/// @details The configuration is used once passed in the ContactManager's init function
struct ContactsManagerSensorsConfiguration
: public internal::ContactsManagerConfigurationPrvt<ContactsManagerSensorsConfiguration>
{

  inline ContactsManagerSensorsConfiguration(const std::string & observerName) noexcept
  : ContactsManagerConfigurationPrvt<ContactsManagerSensorsConfiguration>(observerName)
  {
  }

  inline ContactsManagerSensorsConfiguration & forceSensorsToOmit(
      const std::vector<std::string> & forceSensorsToOmit) noexcept
  {
    forceSensorsToOmit_ = forceSensorsToOmit;
    return *this;
  }

  // force sensors that must not be used for the contacts detection (ex: hands when holding an object)
  std::vector<std::string> forceSensorsToOmit_;
};

/// @brief Configuration structure that helps setting up the parameters of a ContactsManager whose contacts detection
/// method is based on the solver.
/// @details The configuration is used once passed in the ContactManager's init function
struct ContactsManagerSolverConfiguration
: public internal::ContactsManagerConfigurationPrvt<ContactsManagerSolverConfiguration>
{
  inline ContactsManagerSolverConfiguration(const std::string & observerName) noexcept
  : ContactsManagerConfigurationPrvt<ContactsManagerSolverConfiguration>(observerName)
  {
  }
};

} // namespace mc_state_observation::measurements
