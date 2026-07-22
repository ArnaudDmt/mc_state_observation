/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#pragma once

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robot.h>

#include <state-observation/tools/rigid-body-kinematics.hpp>

#include <mc_observers/Observer.h>

namespace mc_state_observation
{
/** Flexibility observer from:
 *
 *    "Tilt estimator for 3D non-rigid pendulum based on a tri-axial
 *    accelerometer and gyrometer". Mehdi Benallegue, Abdelaziz Benallegue,
 *    Yacine Chitour. IEEE-RAS Humanoids 2017. <hal-01499167>
 *
 */
// class MocapContact : public measurements::ContactWithSensor
// {
//   using measurements::ContactWithSensor::ContactWithSensor;

// public:
//   // pose of the contact in the world
//   stateObservation::kine::Kinematics worldKine_;
// };

struct MocapVisualizer : public mc_observers::Observer
{

  MocapVisualizer(const std::string & type, double dt);

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
  // const stateObservation::kine::Kinematics & getContactKinematics(MocapContact & contact,
  //                                                                 const mc_rbdyn::ForceSensor & fs);

  // void addContactsLogs(MocapContact & contact, mc_rtc::Logger & logger);

protected:
  /**
   * Find established contacts between the observed robot and the fixed robots
   *
   * \param ctl Controller that defines the contacts
   * \return Name of surfaces in contact with the environment
   */

  void extractTransformFromMocap();

protected:
  std::string robot_ = "";
  // std::string imuSensor_ = "";

public:
private:
  // name of the body on which the markers are set.
  std::string mocapBodyName_;

  std::string csvPath_;

  stateObservation::kine::Kinematics init_worldBodyKine_;
  stateObservation::kine::Kinematics current_WorldBodyKine_;
  stateObservation::kine::Kinematics bodyFbKine_;

  stateObservation::kine::Kinematics worldFbKine_;

  sva::PTransformd X_0_fb_;

  bool firstRun_ = true;

  double currentMocapDataTime_ = 0.0;
  int currentIter_ = 0;
  bool mocapFinished = false;

  /* custom list of robots to display */
  std::shared_ptr<mc_rbdyn::Robots> my_robots_;

  std::unordered_map<int, stateObservation::kine::Kinematics> mocap_worldBodyKine_;
  std::unordered_map<int, stateObservation::kine::Kinematics> mocapTransforms_;

  bool overlappingDatas_;
  std::unordered_map<int, int> overlapTime_;

  // measurements::ContactsManager<MocapContact> contactsManager_;
};

} // namespace mc_state_observation
