#pragma once

#include <mc_observers/Observer.h>
#include <mc_rtc/Configuration.h>

#include <state-observation/tools/measurements-manager/ContactsManager.hpp>

#include <mc_state_observation/measurements/ContactsDetector.h>
#include <mc_state_observation/measurements/ContactsDetector.hpp>

#include <state-observation/tools/measurements-manager/IMU.hpp>

namespace mc_state_observation
{

struct DoContactWithSensor : public stateObservation::measurements::Contact
{
  using stateObservation::measurements::Contact::Contact;

  inline const std::string & fsName() const noexcept { return fsName_; }

  inline void fsName(const std::string & fsName) { fsName_ = fsName; }

  inline void resetContact() noexcept { Contact::resetContact(); }

  std::string fsName_;
  bool sensorEnabled_ = true;
};

struct DoContactsManager : public stateObservation::measurements::ContactsManager<DoContactWithSensor>
{
  std::unordered_map<std::string, std::string> fs_Surface_Map;
};

struct MCDisturbanceObserver : public mc_observers::Observer
{
  MCDisturbanceObserver(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config) override;
  void reset(const mc_control::MCController & ctl) override;
  bool run(const mc_control::MCController & ctl) override;
  void update(mc_control::MCController & ctl) override;

  inline const Eigen::Vector3d & estimatedExternalForce() const noexcept { return estimatedExternalForce_; }
  inline const Eigen::Vector3d & estimatedExternalForceRaw() const noexcept { return estimatedExternalForceRaw_; }

protected:
  void addToLogger(const mc_control::MCController & ctl,
                   mc_rtc::Logger & logger,
                   const std::string & category) override;
  void removeFromLogger(mc_rtc::Logger & logger, const std::string & category) override;

private:
  void updateContacts(const mc_control::MCController & ctl);
  Eigen::Matrix3d worldRotationOfIMU(const mc_rbdyn::Robot & robot) const;
  void computeEstimatedForce(const mc_control::MCController & ctl, const mc_rbdyn::Robot & realRobot);
  void applyFirstOrderLowPassFilter();

private:
  std::string robot_ = "";
  std::string imuName_ = "";
  std::string category_;

  double nominalMass_ = 0.0;
  double cutoffFrequency_ = 100.0;

  using DoContactsDetector = measurements::ContactsDetector<DoContactWithSensor>;
  DoContactsDetector contactsDetector_;
  DoContactsManager contactsManager_;

  std::vector<stateObservation::measurements::IMU> listIMUs_;

  Eigen::Vector3d estimatedExternalForceRaw_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d estimatedExternalForce_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d previousEstimatedExternalForce_ = Eigen::Vector3d::Zero();
};

} // namespace mc_state_observation