#include <string>
#include <vector>

#include <mc_control/MCController.h>
#include <state-observation/tools/definitions.hpp>

namespace mc_state_observation::measurements
{

namespace internal
{

/// @brief Configuration structure that helps setting up the parameters of a ContactsDetector.
/// @details The configuration is used once passed in the ContactManager's init function
template<class ConfigurationType>
struct ContactsDetectorConfigurationPrvt
{
  inline ContactsDetectorConfigurationPrvt() noexcept
  {
    schmittLowerPropThreshold_ = 0.10;
    schmittUpperPropThreshold_ = 0.15;
  }

  inline ConfigurationType & schmittTriggerPropThresholds(double lowerPropThreshold, double upperPropThreshold) noexcept
  {
    schmittLowerPropThreshold_ = lowerPropThreshold;
    schmittUpperPropThreshold_ = upperPropThreshold;
    return static_cast<ConfigurationType &>(*this);
  }

  double schmittLowerPropThreshold_;
  double schmittUpperPropThreshold_;
};
} // namespace internal

/// @brief Configuration structure that helps setting up the parameters of a ContactsDetector whose contacts detection
/// method is based on contact surfaces.
/// @details The configuration is used once passed in the ContactManager's init function
struct ContactsDetectorSurfacesConfiguration
: public internal::ContactsDetectorConfigurationPrvt<ContactsDetectorSurfacesConfiguration>
{
  inline ContactsDetectorSurfacesConfiguration(const std::vector<std::string> & surfacesForContactDetection) noexcept
  : ContactsDetectorConfigurationPrvt<ContactsDetectorSurfacesConfiguration>(),
    surfacesForContactDetection_(surfacesForContactDetection)
  {
  }

  // list of admissible contact surfaces for the detection
  std::vector<std::string> surfacesForContactDetection_;
};

/// @brief Configuration structure that helps setting up the parameters of a ContactsDetector whose contacts detection
/// method is based on the force sensor measurements.
/// @details The configuration is used once passed in the ContactManager's init function
struct ContactsDetectorSensorsConfiguration
: public internal::ContactsDetectorConfigurationPrvt<ContactsDetectorSensorsConfiguration>
{

  inline ContactsDetectorSensorsConfiguration() noexcept
  : ContactsDetectorConfigurationPrvt<ContactsDetectorSensorsConfiguration>()
  {
  }

  inline ContactsDetectorSensorsConfiguration & forceSensorsToOmit(
      const std::vector<std::string> & forceSensorsToOmit) noexcept
  {
    forceSensorsToOmit_ = forceSensorsToOmit;
    return *this;
  }

  // force sensors that must not be used for the contacts detection (ex: hands when holding an object)
  std::vector<std::string> forceSensorsToOmit_;
};

/// @brief Configuration structure that helps setting up the parameters of a ContactsDetector whose contacts detection
/// method is based on the solver.
/// @details The configuration is used once passed in the ContactManager's init function
struct ContactsDetectorSolverConfiguration
: public internal::ContactsDetectorConfigurationPrvt<ContactsDetectorSolverConfiguration>
{
  inline ContactsDetectorSolverConfiguration() noexcept
  : ContactsDetectorConfigurationPrvt<ContactsDetectorSolverConfiguration>()
  {
  }
};

} // namespace mc_state_observation::measurements
