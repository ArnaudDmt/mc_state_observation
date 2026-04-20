/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#pragma once

#include <cassert>
#include <mc_state_observation/measurements/ContactsDetector.hpp>
#include <state-observation/dynamics-estimators/kinetics-observer.hpp>
#include <state-observation/tools/measurements-manager/ContactsManager.hpp>
#include <state-observation/tools/measurements-manager/IMU.hpp>
#include <state-observation/tools/odometry/legged-odometry-manager.hpp>

#include <kinetics_observer_fg.hpp>

#include <state-observation/tools/rigid-body-kinematics.hpp>
#include <string_view>

namespace mc_state_observation
{
/** Interface for the use of the Kinetics Observer within mc_rtc: \n
 * The Kinetics Observer requires inputs expressed in the frame of the floating base. It then performs a conversion to
 *the centroid frame, a frame located at the center of mass of the robot and with the orientation of the floating
 *base of the real robot.
 *The inputs are obtained from a robot called the inputRobot. Its configuration is the one of real robot, but
 *its floating base's frame is superimposed with the world frame. This allows to ease computations performed in the
 *local frame of the robot.
 **/

/// @brief Class containing the information of a contact.
/// @details This class is an enhancement of the ContactWithSensor class with the kinematics of the contact in the
/// floating base and the kinematics of the frame of the sensor in the frame of the contact surface
struct KoContactWithSensor : public stateObservation::measurements::Contact
{
  using stateObservation::measurements::Contact::Contact;

  inline const std::string & fsName() const noexcept { return fsName_; }

  inline void fsName(const std::string_view & fsName) { fsName_ = fsName; }

  inline void resetContact() noexcept { Contact::resetContact(); }

public:
  // kinematics of the contact frame in the floating base's frame
  stateObservation::kine::Kinematics fbContactKine_;
  // kinematics of the sensor frame in the frame of the contact surface
  stateObservation::kine::Kinematics contactSensorKine_;
  // kinematics of the contact  frame in the centroid frame
  stateObservation::kine::Kinematics centroidContactKine_;
  // measured contact wrench, expressed in the frame of the contact.
  Eigen::Matrix<double, 6, 1> contactWrenchVector_;
  // for debug only
  stateObservation::Vector6 viscoElasticWrenchAfterCorrection_;
  std::string fsName_;

  // the sensor measurement has to be used by the observer
  bool sensorEnabled_ = true;
};

struct KoContactsManager : public stateObservation::measurements::ContactsManager<KoContactWithSensor>
{
  // map that relates a force sensor to the associated surface
  std::unordered_map<std::string, std::string> fs_Surface_Map;
};

struct MCKineticsObserverFG : public mc_observers::Observer
{

  MCKineticsObserverFG(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

protected:
  /// @brief Update the pose and velocities of the robot in the world frame. Used only to update the ones of the robot
  /// used for the visualization of the estimation made by the Kinetics Observer.
  /// @param robot The robot to update.
  void update(mc_rbdyn::Robot & robot);

  /// @brief Sums up the wrenches measured by the unused force sensors expressed in the centroid frame to give them as
  /// an input to the Kinetics Observer
  /// @param measRobot The control robot. Used to retrieve the measurements.
  stateObservation::Vector6 inputAdditionalWrench(const mc_control::MCController & ctl,
                                                  const mc_rbdyn::Robot & measRobot);

  /// @brief Update the IMUs, including the measurements and kinematics in the centroid frame
  /// @param measRobot The control robot
  /// @param inputRobot A robot whose configuration is the one of real robot, but whose pose, velocities and
  /// accelerations are set to zero in the control frame. Allows to ease computations performed in the local frame of
  /// the robot.
  void updateIMUs(const mc_rbdyn::Robot & measRobot, const mc_rbdyn::Robot & inputRobot);

  /*! \brief Add observer from logger
   *
   * @param category Category in which to log this observer
   */

  /// @brief Add the logs of the desired contact.
  /// @param Controller Controller
  /// @param contact contact
  /// @param logger
  void addContactLogEntries(const mc_control::MCController & ctl,
                            mc_rtc::Logger & logger,
                            const KoContactWithSensor & contact);
  /// @brief Remove the logs of the desired contact.
  /// @param contact Contact
  /// @param logger
  void removeContactLogEntries(mc_rtc::Logger & logger, const KoContactWithSensor & contact);

  /// @brief Add the measurements logs of the desired contact.
  /// @param contact Contact
  /// @param logger
  void addContactMeasurementsLogEntries(mc_rtc::Logger & logger, const KoContactWithSensor & contact);
  /// @brief Remove the measurements logs of the desired contact.
  /// @param contact Contact
  /// @param logger
  void removeContactMeasurementsLogEntries(mc_rtc::Logger & logger, const KoContactWithSensor & contact);

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

  void addContactToGui(const mc_control::MCController & ctl, KoContactWithSensor & contact, mc_rtc::Logger & logger);

  /// @brief Sets the type of the odometry
  /// @param newOdometryType The new type of odometry to use.
  void setOdometryType(const std::string & newOdometryType);

protected:
  /// @brief Update the currently set contacts.
  /// @param ctl Controller
  /// @param logger Logger
  void updateContacts(const mc_control::MCController & ctl, mc_rtc::Logger & logger);

  /// @brief Computes the kinematics of the contact attached to the robot in the world frame.
  /// @details Also updates the wrench measured at the contact if required.
  /// @param contact Contact of which we want to compute the kinematics
  /// @param robot robot the contacts belong to
  /// @param fs force sensor
  /// @return stateObservation::kine::Kinematics &
  const stateObservation::kine::Kinematics getContactWorldKinematics(const KoContactWithSensor & contact,
                                                                     const mc_rbdyn::Robot & robot,
                                                                     const mc_rbdyn::ForceSensor & fs,
                                                                     const sva::ForceVecd * measuredWrench = nullptr);

  /// @brief Updates the measurements of the force sensor attached to a contact.
  /// @details Expresses the measured wrench in the frame of the contact. The sensor is generally not directly attached
  /// to the contact, so the transformation from the sensor to the contact might be necessary.
  /// @param contact Contact associated to the sensor
  /// @param measuredWrench measured wrench
  /// @param surfaceSensorKine transformation from the sensor to the contact.
  void updateContactForceMeasurement(KoContactWithSensor & contact,
                                     const sva::ForceVecd & measuredWrench,
                                     const stateObservation::kine::Kinematics * contactSensorKine = nullptr);

  /// @brief Computes the rest pose of the contact in the world.
  /// @details At contact detection, a wrench is already applied, which means the contact frame obtained by forward
  /// kinematics is not the rest pose. We thus remove it using the viscoelastic model and the measured wrench.
  /// @param ctl Controller
  /// @param contact Contact
  /// @param worldContactKine Contact frame kinematics, which are affected by the deformation of flexiblities.
  /// @param worldRestPose Rest pose of the contact, updated in the function
  /// @return The contact rest pose.
  stateObservation::kine::Kinematics getOdometryWorldContactRest(
      const mc_control::MCController & ctl,
      KoContactWithSensor & contact,
      const stateObservation::kine::Kinematics & worldContactKine);

  /// @brief Creates a new contact
  /// @param ctl Controller
  /// @param contact Contact to update
  /// @param initNoises The initial noises associated with the contact.
  /// @param logger Logger
  void setNewContact(const mc_control::MCController & ctl,
                     KoContactWithSensor & contact,
                     const std::array<double, 4> & initNoises,
                     mc_rtc::Logger & logger);

  /// @brief Updates an already set contact
  /// @param ctl Controller
  /// @param contact Contact to update
  /// @param logger Logger
  void updateContact(const mc_control::MCController & ctl, KoContactWithSensor & contact);

  inline stateObservation::kine::Kinematics fgLocKineToSoKine(const ko_fg::LocKinematics & locK)
  {
    stateObservation::KineticsObserver::Kinematics kine;

    if(locK.hasPose())
    {
      kine.position = locK.pose().rotation() * locK.pose().translation();
      kine.orientation = locK.pose().rotation().matrix();
    }
    else { assert(false && "Cannot convert from local kinematics to kinematics without an orientation."); }

    if(locK.hasLinVel()) { kine.linVel = locK.pose().rotation() * locK.linVel(); }

    if(locK.hasLinAcc()) { kine.linAcc = locK.pose().rotation() * locK.linAcc(); }

    if(locK.hasAngVel()) { kine.angVel = locK.pose().rotation() * locK.angVel(); }

    if(locK.hasAngAcc()) { kine.angAcc = locK.pose().rotation() * locK.angAcc(); }
    return kine;
  }

public:
  inline const sva::ForceVecd & getUnbiasedEstimatedDisturbanceWrench() { return unbiasedDisturbanceWrench_; }

  /** Set debug flag.
   *
   * \param flag New debug flag.
   *
   */
  inline void debug(bool flag) { debug_ = flag; }

  /** Floating-base transform estimate.
   *
   */
  inline const sva::PTransformd & posW() const { return X_0_fb_; }

  /** Floating-base velocity estimate.
   *
   */
  inline const sva::MotionVecd & velW() const { return v_fb_0_; }

private: // instance of the Kinetics Observer
  ko_fg::KineticsObserverFG observer_;

  // contacts maintained during the current iteration
  std::unordered_map<unsigned, KoContactWithSensor *> maintainedContacts_;

  // category to plot the estimator in
  std::string category_;
  // name of the robot
  std::string robot_ = "";
  /* custom list of robots to display */
  std::shared_ptr<mc_rbdyn::Robots> my_robots_;
  // std::string imuSensor_ = "";
  std::vector<std::string> imuNames_; ///< list of IMUs

  /* Estimation parameters */
  bool debug_ = false;
  bool verbose_ = true;

  /* Estimation results */

  // state vector resulting from the Kinetics Observer esimation
  Eigen::VectorXd res_;
  stateObservation::kine::Kinematics centroidFbKine_;
  stateObservation::kine::Kinematics worldCentroidKine_;
  // kinematics of the centroid frame in the floating base
  stateObservation::kine::Kinematics fbCentroidKine_;

  // pose of the floating base within the world frame (real one, not the one of the control robot)
  sva::PTransformd X_0_fb_;
  // velocity of the floating base within the world frame (real one, not the one of the control robot)
  sva::MotionVecd v_fb_0_;
  // acceleration of the floating base within the world frame (real one, not the one of the control robot)
  sva::MotionVecd a_fb_0_;

  /* Parameters of the robot */
  // mass of the robot
  double mass_; // [kg]

  sva::ForceVecd disturbanceWrenchOffset_;
  sva::ForceVecd unbiasedDisturbanceWrench_;
  bool removeWrenchOffset_;
  size_t wrenchOffsetIndex_;

  // indicates if the debug logs have to be added.
  bool withDebugLogs_ = false;

  // indicates if we want to perform odometry, and if yes, flat or 6d odometry
  stateObservation::odometry::OdometryType odometryType_;
  // odometry method used on last iteration. Used to check if it changed in order to apply the change to the Tilt
  // Observer if necessary.
  stateObservation::odometry::OdometryType prevOdometryType_;
  // indicates if we want to estimate the unmodeled wrench within the Kinetics Observer.

  using KoContactsDetector = measurements::ContactsDetector<KoContactWithSensor>;
  KoContactsDetector contactsDetector_;

  KoContactsManager contactsManager_;

  /* IMU variables */
  // manager for the IMUs
  std::vector<stateObservation::measurements::IMU> listIMUs_;

  /* Utilitary variables */
  // zero frame transformation
  sva::PTransformd zeroPose_;
  // zero velocity or acceleration
  sva::MotionVecd zeroMotion_;

  stateObservation::Vector6 inputWrench_;

  size_t k_;

  /*
  - pos
  - ori
  - linVel
  - angvel
  - linAcc
  - angAcc
  - disturbForce
  - disturbMoment
  */
  std::array<double, 8> initNoises_;
  std::array<double, 8> processNoises_;
  std::unordered_map<size_t, std::array<double, 4>> imuNoises_;

  /*
  Contact flexibility tuning
  - linear stiffness
  - angular stiffness
  - linear damping
  - angular damping
  */
  std::array<stateObservation::Matrix3, 4> contactFlexibilities_;
  std::array<double, 4> contactInitNoises_;
  std::array<double, 4> contactInitNoises_first_;
  std::array<double, 4> contactProcessNoises_;
  std::array<double, 2> contactMeasNoises_;

  // estimated kinematics of the centroid frame in the world frame
  stateObservation::kine::Kinematics est_worldCentroidKine_;
  // estimated kinematics of the floating base in the world frame
  stateObservation::kine::Kinematics est_worldFbKine_;

  // total force measured by the sensors that are not associated to a currently set contact and expressed in the
  // floating base's frame. Used as an input for the Kinetics Observer.
  stateObservation::Vector3 additionalUserResultingForce_ = stateObservation::Vector3::Zero();
  // total torque measured by the sensors that are not associated to a currently set contact and expressed in the
  // floating base's frame. Used as an input for the Kinetics Observer.
  stateObservation::Vector3 additionalUserResultingMoment_ = stateObservation::Vector3::Zero();
};

} // namespace mc_state_observation
