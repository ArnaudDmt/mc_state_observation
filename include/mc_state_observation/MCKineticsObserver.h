/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#pragma once

#include <boost/circular_buffer.hpp>

#include <mc_state_observation/MCValinor.h>
#include <mc_state_observation/measurements/ContactsDetector.h>
#include <mc_state_observation/measurements/ContactsDetector.hpp>
#include <state-observation/dynamics-estimators/kinetics-observer.hpp>
#include <state-observation/tools/measurements-manager/IMU.hpp>
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
 *The Kinetics Observer is associated to Valinor as a backup. If an anomaly is detected, the Kinetics Observer
 *will recover the last ellapsed second (or less) using the displacement made by Valinor.
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
  // measured contact wrench, expressed in the frame of the contact.
  Eigen::Matrix<double, 6, 1> contactWrenchVector_;
  // contact wrench expressed in the centroid frame. Used for logs.
  Eigen::Matrix<double, 6, 1> wrenchInCentroid_ = Eigen::Matrix<double, 6, 1>::Zero();
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

struct MCKineticsObserver : public mc_observers::Observer
{

  MCKineticsObserver(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

protected:
  /// @brief sets all the covariances required by the Kinetics Observer
  void setObserverCovariances();
  /// @brief Update the pose and velocities of the robot in the world frame. Used only to update the ones of the robot
  /// used for the visualization of the estimation made by the Kinetics Observer.
  /// @param robot The robot to update.
  void update(mc_rbdyn::Robot & robot);

  /// @brief Initializer for the Kinetics Observer's state vector
  /// @param robot The control robot
  void initObserverStateVector(const mc_control::MCController & ctl, const mc_rbdyn::Robot & robot);

  /// @brief Sums up the wrenches measured by the unused force sensors expressed in the centroid frame to give them as
  /// an input to the Kinetics Observer
  /// @param inputRobot A robot whose configuration is the one of real robot, but whose pose, velocities and
  /// accelerations are set to zero in the control frame. Allows to ease computations performed in the local frame of
  /// the robot.
  /// @param measRobot The control robot. Used to retrieve the measurements.
  void inputAdditionalWrench(const mc_control::MCController & ctl,
                             const mc_rbdyn::Robot & inputRobot,
                             const mc_rbdyn::Robot & measRobot);

  /// @brief Update the IMUs, including the measurements, measurement covariances and kinematics in the floating
  /// base's frame (user frame)
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
                            KoContactWithSensor & contact);
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
  const stateObservation::kine::Kinematics getContactWorldKinematics(const mc_control::MCController & ctl,
                                                                     KoContactWithSensor & contact,
                                                                     const mc_rbdyn::Robot & robot,
                                                                     bool withVel);

  const stateObservation::kine::Kinematics getCtlContactWorldKinematics(const mc_control::MCController & ctl,
                                                                        KoContactWithSensor & contact,
                                                                        bool withVel);

  const stateObservation::kine::Kinematics getFsWorldKinematics(const mc_control::MCController & ctl,
                                                                const mc_rbdyn::Robot & robot,
                                                                const std::string & fsName);

  const stateObservation::kine::Kinematics getContactFsKinematics(const mc_control::MCController & ctl,
                                                                  KoContactWithSensor & contact,
                                                                  const mc_rbdyn::Robot & currentRobot);

  // const stateObservation::kine::Kinematics getContactWorldPose(const mc_rbdyn::Robot & currentRobot,
  //                                                              const stateObservation::kine::Kinematics &
  //                                                              fbContactPose);

  /// @brief Updates the measurements of the force sensor attached to a contact.
  /// @details Expresses the measured wrench in the frame of the contact. The sensor is generally not directly
  /// attached to the contact, so the transformation from the sensor to the contact might be necessary.
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
      KoContactWithSensor & contact,
      const stateObservation::kine::Kinematics & worldContactKine);

  /// @brief Creates a new contact
  /// @param ctl Controller
  /// @param contact Contact to update
  /// @param initCovariance The initial covariance associated with the contact.
  /// @param logger Logger
  void setNewContact(const mc_control::MCController & ctl,
                     KoContactWithSensor & contact,
                     const stateObservation::Matrix12 & initCovariance,
                     mc_rtc::Logger & logger);

  /// @brief Updates an already set contact
  /// @param ctl Controller
  /// @param contact Contact to update
  /// @param logger Logger
  void updateContact(const mc_control::MCController & ctl, KoContactWithSensor & contact);

  /// @brief Applies the current Kinetics Observer floating-base estimate.
  void updateFloatingBaseFromObserver();

  /// @brief Applies backup floating-base kinematics and estimates acceleration from the previous velocity.
  void updateFloatingBaseKinematicsFromBackup(const stateObservation::kine::Kinematics & fbKinematics, double timeStep);

  /// @brief Reinitializes the observer centroid state from the current backup floating-base kinematics.
  void resetWorldCentroidState(mc_rbdyn::Robot & inputRobot, bool resetCovariance);

  /// @brief Reinitializes the observer contacts after backup.
  void resetContactsAfterBackup(const mc_control::MCController & ctl,
                                const mc_rbdyn::Robot & robot,
                                const mc_rbdyn::Robot & realRobot,
                                const mc_rbdyn::Robot & forceSensorRobot,
                                bool resetCovariance);

  const char * estimationStateName() const;

public:
  inline Eigen::VectorBlock<Eigen::VectorXd, 6> getEstimatedDisturbanceWrench()
  {
    return res_.template segment<6>(observer_.unmodeledWrenchIndex());
  }

  inline const sva::ForceVecd & getUnbiasedEstimatedDisturbanceWrench() { return unbiasedDisturbanceWrench_; }

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

  /** Set debug flag.
   *
   * \param flag New debug flag.
   *
   */
  inline void debug(bool flag) { debug_ = flag; }

  /** Get last measurement vector sent to observer.
   *
   */
  inline const Eigen::VectorXd measurements() const { return observer_.getEKF().getLastMeasurement(); }

  /** Floating-base transform estimate.
   *
   */
  inline const sva::PTransformd & posW() const { return X_0_fb_; }

  /** Floating-base velocity estimate.
   *
   */
  inline const sva::MotionVecd & velW() const { return v_fb_0_; }

private:
  /* Settings of the Kinetics Observers */
  // maximum amount of contacts that we want to use with the Kinetics Observer.
  unsigned maxContacts_;
  // maximum amount of IMUs that we want to use with the Kinetics Observer.
  unsigned maxIMUs_;

  // instance of the Kinetics Observer
  stateObservation::KineticsObserver observer_;
  // instance of the Tilt Observer used as a backup
  MCValinor valinor_;

  // contacts maintained during the current iteration
  std::unordered_map<unsigned, KoContactWithSensor *> maintainedContacts_;

  enum EstimationState
  {
    noIssue,
    errorDetected,
    invincibilityFrame
  };

  // indicates if the current iteration encountered no issue, encountered one, or is inside the invincibility frame
  // (recovery frame after an error)
  EstimationState estimationState_;

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
  // pose of the floating base within the world frame (real one, not the one of the control robot)
  sva::PTransformd X_0_fb_;
  // velocity of the floating base within the world frame (real one, not the one of the control robot)
  sva::MotionVecd v_fb_0_;
  // acceleration of the floating base within the world frame (real one, not the one of the control robot)
  sva::MotionVecd a_fb_0_;

  /* Parameters of the robot */
  // mass of the robot
  double mass_; // [kg]
  // linear stiffness of contacts
  stateObservation::Matrix3 linStiffness_;
  // linear damping of contacts
  stateObservation::Matrix3 linDamping_;
  // angular stiffness of contacts
  stateObservation::Matrix3 angStiffness_;
  // linear damping of contacts
  stateObservation::Matrix3 angDamping_;

  sva::ForceVecd disturbanceWrenchOffset_;
  sva::ForceVecd unbiasedDisturbanceWrench_;
  bool removeWrenchOffset_;
  size_t wrenchOffsetIndex_;

  // indicates if the debug logs have to be added.
  bool withDebugLogs_ = false;
  stateObservation::Matrix3 contactsPosAverageStateCov_;

  // indicates if we want to perform odometry, and if yes, flat or 6d odometry
  stateObservation::odometry::OdometryType odometryType_;
  // odometry method used on last iteration. Used to check if it changed in order to apply the change to the Tilt
  // Observer if necessary.
  stateObservation::odometry::OdometryType prevOdometryType_;
  // indicates if we want to estimate the unmodeled wrench within the Kinetics Observer.
  bool withUnmodeledWrench_ = true;
  // indicates if we want to estimate the bias on the gyrometer measurement within the Kinetics Observer.
  bool withGyroBias_ = true;

  /* Kalman Filter's covariances */

  // initial covariance on the position estimate
  stateObservation::Matrix3 statePositionInitCovariance_;
  // initial covariance on the orientation estimate
  stateObservation::Matrix3 stateOriInitCovariance_;
  // initial covariance on the local linear velocity estimate
  stateObservation::Matrix3 stateLinVelInitCovariance_;
  // initial covariance on the local angular velocity estimate
  stateObservation::Matrix3 stateAngVelInitCovariance_;
  // initial covariance on the gyrometer bias estimate
  stateObservation::Matrix3 gyroBiasInitCovariance_;
  // initial covariance on the unmodeled wrench estimate
  stateObservation::Matrix6 unmodeledWrenchInitCovariance_;
  // initial covariance on the contact rest pose estimate, when no other contact is currently set
  stateObservation::Matrix12 contactInitCovarianceFirstContacts_;
  // initial covariance on the contact rest pose estimate, when other contacts are currently set
  stateObservation::Matrix12 contactInitCovarianceNewContacts_;
  // initial covariance on the contact rest pose estimate, when no other contact is currently set. Version when using
  // flat odometry.
  stateObservation::Matrix12 contactInitCovarianceFirstContacts_flat_;
  // initial covariance on the contact rest pose estimate, when other contacts are currently set. Version when using
  // flat odometry.
  stateObservation::Matrix12 contactInitCovarianceNewContacts_flat_;

  // covariance on the position's state transition
  stateObservation::Matrix3 statePositionProcessCovariance_;
  // covariance on the orientation's state transition
  stateObservation::Matrix3 stateOriProcessCovariance_;
  // covariance on the local linear velocity's state transition
  stateObservation::Matrix3 stateLinVelProcessCovariance_;
  // covariance on the angular velocity's state transition
  stateObservation::Matrix3 stateAngVelProcessCovariance_;
  // covariance on the gyrometer bias' state transition
  stateObservation::Matrix3 gyroBiasProcessCovariance_;
  // covariance on the unmodeled wrench's state transition
  stateObservation::Matrix6 unmodeledWrenchProcessCovariance_;
  // covariance on the contact rest pose's state transition
  stateObservation::Matrix12 contactProcessCovariance_;

  // covariance on the absolute position measurement
  stateObservation::Matrix3 positionSensorCovariance_;
  // covariance on the absolute orientation measurement
  stateObservation::Matrix3 orientationSensorCoVariance_;
  // covariance on the accelerometer measurement
  stateObservation::Matrix3 acceleroSensorCovariance_;
  // covariance on the gyrometer measurement
  stateObservation::Matrix3 gyroSensorCovariance_;
  // covariance on the contact's force sensors measurement
  stateObservation::Matrix6 contactSensorCovariance_;
  // covariance on the gyrometer measurement
  stateObservation::Matrix3 absoluteOriSensorCovariance_;

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
  // kinematics of the CoM within the floating base
  stateObservation::kine::Kinematics fbCoMKine_;
  // total force measured by the sensors that are not associated to a currently set contact and expressed in the
  // floating base's frame. Used as an input for the Kinetics Observer.
  stateObservation::Vector3 additionalUserResultingForce_ = stateObservation::Vector3::Zero();
  // total torque measured by the sensors that are not associated to a currently set contact and expressed in the
  // floating base's frame. Used as an input for the Kinetics Observer.
  stateObservation::Vector3 additionalUserResultingMoment_ = stateObservation::Vector3::Zero();

  stateObservation::Vector3 worldAnchorPos_;
  stateObservation::Vector3 fbAnchorPos_;
  stateObservation::kine::Kinematics worldFbKine_;

  /* Variables for the backup */
  // iteration on which the backup was required for the last time
  int lastBackupIter_;
  // number of iterations on which we perform the backup
  size_t fbBackupCapacity_ = 0;
  // time during which the Kinetics Observer is still getting updated by Valinor after the need of a backup,
  // so the Kalman Filter has time to converge again
  int invincibilityFrame_ = 0;
  // iterations ellapsed within the invincibility frame
  int invincibilityIter_;

  bool withGui_ = false;

  // Buffer containing the estimated pose of the floating base in the world over the whole backup interval.
  boost::circular_buffer<stateObservation::kine::Kinematics> koBackupFbKinematics_;

  // Kinematics of the floating base in the real world frame (our estimation goal)
  stateObservation::kine::Kinematics mcko_K_0_fb_;

  /* Debug variables */
  // For logs only. Prediction of the measurements from the newly corrected state
  stateObservation::Vector correctedMeasurements_;
  // For logs only. Kinematics of the centroid frame within the world frame
  stateObservation::kine::Kinematics globalCentroidKinematics_;
};

} // namespace mc_state_observation
