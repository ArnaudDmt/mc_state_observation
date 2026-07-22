/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#pragma once

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robot.h>

#include <mc_state_observation/observersTools/leggedOdometryTools.h>
#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

#include <mc_observers/Observer.h>

namespace mc_state_observation
{
namespace so = stateObservation;

struct NaiveOdometry : public mc_observers::Observer
{

  NaiveOdometry(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

protected:
  void update(mc_rbdyn::Robot & robot);

  /*! \brief Add observer from logger
   *
   * @param category Category in which to log this observer
   */

  void plotVariablesBeforeUpdate(const mc_control::MCController & ctl, mc_rtc::Logger & logger);

  void plotVariablesAfterUpdate(const mc_control::MCController & ctl, mc_rtc::Logger & logger);

  void addToLogger(const mc_control::MCController &, mc_rtc::Logger &, const std::string & category) override;

  void addContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName);

  void removeContactLogEntries(mc_rtc::Logger & logger, const std::string & contactName);

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

protected:
  /**
   * Find established contacts between the observed robot and the fixed robots
   *
   * \param ctl Controller that defines the contacts
   * \return Name of surfaces in contact with the environment
   */

  /*
    void updateContacts(const mc_control::MCController & ctl,
                        mc_rbdyn::Robot & odometryRobot,
                        std::set<std::string> contacts,
                        mc_rtc::Logger & logger);

    void setNewContact(const mc_rbdyn::Robot & odometryRobot, const mc_rbdyn::ForceSensor forceSensor);
  */
protected:
  std::string robot_ = "";
  // std::string imuSensor_ = "";
  mc_rbdyn::BodySensorVector IMUs_; ///< list of IMUs

public:
  /** Get robot mass.
   *
   */
  inline double mass() const { return mass_; }

  /** Set robot mass.
   *
   * \param mass Robot mass.
   *
   */
  void mass(double mass);

  /** Set stiffness of the robot-environment flexibility.
   *
   * \param stiffness Flexibility stiffness.
   *
   */

  /** Floating-base transform estimate.
   *
   */
  inline const sva::PTransformd & posW() const { return X_0_fb_; }

  /** Floating-base velocity estimate.
   *
   */
  inline const sva::MotionVecd & velW() const { return v_fb_0_; }

private:
  std::string category_ = "NaiveOdometry_";
  /* custom list of robots to display */
  std::shared_ptr<mc_rbdyn::Robots> my_robots_;

  // threshold on the force for the contact detection.
  double contactDetectionThreshold_;

  double mass_ = 42; // [kg]
  // std::set<std::string> contacts_; ///< Sorted list of contacts
  std::set<std::string> oldContacts_;

  sva::MotionVecd v_fb_0_ = sva::MotionVecd::Zero();
  sva::PTransformd X_0_fb_ = sva::PTransformd::Identity();
  sva::MotionVecd a_fb_0_ = sva::MotionVecd::Zero();

  leggedOdometry::LeggedOdometryManager odometryManager_;

  bool accUpdatedUpstream_ = false;

  using LoContactsManager = leggedOdometry::LeggedOdometryManager::ContactsManager;
};

} // namespace mc_state_observation
