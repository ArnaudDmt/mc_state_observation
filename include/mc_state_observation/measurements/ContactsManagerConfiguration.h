#include <string>
#include <vector>

#include <mc_control/MCController.h>
#include <state-observation/tools/definitions.hpp>

namespace mc_state_observation::measurements
{

enum ContactsDetection
{
  Solver,
  Surfaces,
  Sensors,
  Undefined
};

struct ContactsManagerConfiguration
{
  ContactsManagerConfiguration() {}

  ContactsManagerConfiguration(ContactsDetection contactsDetection, const std::string & observerName) noexcept
  : observerName_(observerName), contactsDetection_(contactsDetection)
  {
    schmittLowerPropThreshold_ = 0.10;
    schmittUpperPropThreshold_ = 0.15;
  }

  inline ContactsManagerConfiguration & surfacesForContactDetection(
      const std::vector<std::string> & surfacesForContactDetection) noexcept
  {
    surfacesForContactDetection_ = surfacesForContactDetection;
    return static_cast<ContactsManagerConfiguration &>(*this);
  }

  inline ContactsManagerConfiguration & schmittTriggerPropThresholds(double lowerPropThreshold,
                                                                     double upperPropThreshold) noexcept
  {
    schmittLowerPropThreshold_ = lowerPropThreshold;
    schmittUpperPropThreshold_ = upperPropThreshold;
    return static_cast<ContactsManagerConfiguration &>(*this);
  }
  inline ContactsManagerConfiguration & verbose(bool verbose) noexcept
  {
    verbose_ = verbose;
    return static_cast<ContactsManagerConfiguration &>(*this);
  }
  inline ContactsManagerConfiguration & forceSensorsToOmit(const std::vector<std::string> & forceSensorsToOmit) noexcept
  {
    forceSensorsToOmit_ = forceSensorsToOmit;
    return *this;
  }

  std::string observerName_;

  double schmittLowerPropThreshold_;
  double schmittUpperPropThreshold_;
  bool verbose_ = true;

  ContactsDetection contactsDetection_;

  // list of admissible contact surfaces for the detection
  std::vector<std::string> surfacesForContactDetection_;
  // force sensors that must not be used for the contacts detection (ex: hands when holding an object)
  std::vector<std::string> forceSensorsToOmit_;
};

} // namespace mc_state_observation::measurements
