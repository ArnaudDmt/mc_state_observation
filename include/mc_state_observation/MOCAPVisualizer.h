/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#pragma once

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robot.h>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

#include <mc_observers/Observer.h>

namespace mc_state_observation
{
namespace so = stateObservation;
/** Flexibility observer from:
 *
 *    "Tilt estimator for 3D non-rigid pendulum based on a tri-axial
 *    accelerometer and gyrometer". Mehdi Benallegue, Abdelaziz Benallegue,
 *    Yacine Chitour. IEEE-RAS Humanoids 2017. <hal-01499167>
 *
 */
struct Contact
{
  Contact() {}
  so::kine::Kinematics worldRefKine_;
  so::Quaternion quat_;
};
struct MocapData
{
  int indexReader = 0;
  double time = 0.0;
  so::kine::Kinematics kine;
};

struct MOCAPVisualizer : public mc_observers::Observer
{

  MOCAPVisualizer(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

protected:
  void update(mc_rbdyn::Robot & robot);

  // void updateWorldFbKineAndViceVersa(const mc_rbdyn::Robot & robot);

  /*! \brief Add observer from logger
   *
   * @param category Category in which to log this observer
   */

  void addToLogger(const mc_control::MCController &, mc_rtc::Logger &, const std::string & category) override;

  /*! \brief Remove observer from logger
   *
   * @param category Category in which this observer entries are logged
   */
  void removeFromLogger(mc_rtc::Logger &, const std::string & category) override;

  /*! \brief Add observer information the GUI.
   *
   * @param category Category in which to add this observer
   */
  void addToGUI(const mc_control::MCController &,
                mc_rtc::gui::StateBuilder &,
                const std::vector<std::string> & /* category */) override;

  void updateContacts(const mc_control::MCController & ctl);

  void addContactsLogs(const std::string & name, mc_rtc::Logger & logger);

protected:
  /**
   * Find established contacts between the observed robot and the fixed robots
   *
   * \param ctl Controller that defines the contacts
   * \return Name of surfaces in contact with the environment
   */

  void extractMocapData();

protected:
  std::string robot_ = "";
  // std::string imuSensor_ = "";

public:
private:
  std::unordered_map<std::string, Contact> contacts_;
  std::string csvPath_;
  sva::PTransformd X_0_fb_ = sva::PTransformd::Identity();
  sva::PTransformd X_0_fb_init_ = sva::PTransformd::Identity();
  MocapData tempMocapData_; // for the insertion in the map
  std::map<std::string, MocapData> mocapDataTable_; // {timecode, data}
  double currentMocapDataTime_ = 0.0;
  bool mocapFinished = false;

  std::string category_ = "Observer_LIPMStabilizerObserverPipeline";
  /* custom list of robots to display */
  std::shared_ptr<mc_rbdyn::Robots> my_robots_;
};

} // namespace mc_state_observation
