/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#pragma once

#include <mc_rbdyn/Contact.h>
#include <mc_rbdyn/Robot.h>
#include <boost/circular_buffer.hpp>
#include <mc_state_observation/observersTools/measurementsTools.h>
#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

#include <mc_observers/Observer.h>

namespace mc_state_observation
{
/** Interface for the use of the Kinetics Observer within mc_rtc: \n
 * The Kinetics Observer requires inputs expressed in the frame of the floating base. It then performs a conversion to
 *the centroid frame, a frame located at the center of mass of the robot and with the orientation of the floating
 *base of the real robot.
 *The inputs are obtained from a robot called the inputRobot. Its configuration is the one of real robot, but
 *its floating base's frame is superimposed with the world frame. This allows to ease computations performed in the
 *local frame of the robot.
 *The Kinetics Observer is associated to the Tilt Observer as a backup. If an anomaly is detected, the Kinetics Observer
 *will recover the last ellapsed second (or less) using the displacement made by the Tilt Observer.
 **/

/// @brief Class containing the information of a contact.
/// @details This class is an enhancement of the ContactWithSensor class with the kinematics of the contact in the
/// floating base and the kinematics of the frame of the sensor in the frame of the contact surface
struct KoContactWithSensor : public measurements::ContactWithSensor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  KoContactWithSensor() {}

public:
  KoContactWithSensor(int id, std::string forceSensorName)
  {
    id_ = id;
    name_ = forceSensorName;
    forceSensorName_ = forceSensorName;

    resetContact();
  }

  KoContactWithSensor(int id,
                      const std::string & forceSensorName,
                      const std::string & surfaceName,
                      bool sensorAttachedToSurface)
  {
    id_ = id;
    name_ = forceSensorName;
    resetContact();

    surface_ = surfaceName;
    forceSensorName_ = forceSensorName;
    sensorAttachedToSurface_ = sensorAttachedToSurface;
  }

public:
  // kinematics of the contact frame in the floating base's frame
  stateObservation::kine::Kinematics fbContactKine_;
  // kinematics of the sensor frame in the frame of the contact surface
  stateObservation::kine::Kinematics surfaceSensorKine_;
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
  void initObserverStateVector(const mc_rbdyn::Robot & robot);

  /// @brief Sums up the wrenches measured by the unused force sensors expressed in the centroid frame to give them as
  /// an input to the Kinetics Observer
  /// @param inputRobot A robot whose configuration is the one of real robot, but whose pose, velocities and
  /// accelerations are set to zero in the control frame. Allows to ease computations performed in the local frame of
  /// the robot.
  /// @param measRobot The control robot. Used to retrieve the measurements.
  void inputAdditionalWrench(const mc_rbdyn::Robot & inputRobot, const mc_rbdyn::Robot & measRobot);

  /// @brief Adds the measurement of the desired sensors to the external force given as an input to the Kinetics
  /// Observer
  /// @details The force sensors must be given with the list forceSensorsAsInput_
  /// @param inputRobot A robot whose configuration is the one of real robot, but whose pose, velocities and
  /// accelerations are set to zero in the control frame. Allows to ease computations performed in the local frame of
  /// the robot.
  /// @param measRobot The control robot. Used to retrieve the measurements.
  /// @param inputAddtionalForce the external force given as input
  /// @param inputAddtionalTorque the external torque given as input
  void addSensorsAsInputs(const mc_rbdyn::Robot & inputRobot,
                          const mc_rbdyn::Robot & measRobot,
                          stateObservation::Vector3 & inputAddtionalForce,
                          stateObservation::Vector3 & inputAddtionalTorque);

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
  /// @param contactIndex The index of the contact.
  /// @param logger
  void addContactLogEntries(mc_rtc::Logger & logger, const int & contactIndex);
  /// @brief Remove the logs of the desired contact.
  /// @param contactIndex The index of the contact.
  /// @param logger
  void removeContactLogEntries(mc_rtc::Logger & logger, const int & contactIndex);

  /// @brief Add the measurements logs of the desired contact.
  /// @param contactIndex The index of the contact.
  /// @param logger
  void addContactMeasurementsLogEntries(mc_rtc::Logger & logger, const int & contactIndex);
  /// @brief Remove the measurements logs of the desired contact.
  /// @param contactIndex The index of the contact.
  /// @param logger
  void removeContactMeasurementsLogEntries(mc_rtc::Logger & logger, const int & contactIndex);

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

  /// @brief Changes the type of the odometry
  /// @param ctl Controller.
  /// @param newOdometryType The new type of odometry to use.
  void changeOdometryType(const mc_control::MCController & ctl, const std::string & newOdometryType);

protected:
  /// @brief Updates the list of currently set contacts.
  /// @return measurements::ContactsManager<measurements::ContactWithSensor,
  /// measurements::ContactWithoutSensor>::ContactsSet &
  const measurements::ContactsManager<KoContactWithSensor, measurements::ContactWithoutSensor>::ContactsSet &
      findNewContacts(const mc_control::MCController & ctl);

  /// @brief Update the currently set contacts.
  /// @details The list of contacts is returned by \ref findNewContacts(const mc_control::MCController & ctl). Calls
  /// \ref updateContact(const mc_control::MCController & ctl, const std::string & name, mc_rtc::Logger & logger).
  /// @param contacts The list of contacts returned by \ref findNewContacts(const mc_control::MCController & ctl).
  void updateContacts(const mc_control::MCController & ctl,
                      const measurements::ContactsManager<KoContactWithSensor,
                                                          measurements::ContactWithoutSensor>::ContactsSet & contacts,
                      mc_rtc::Logger & logger);

  /// @brief Computes the kinematics of the contact attached to the robot in the world frame. Also expresses the wrench
  /// measured at the sensor in the frame of the contact.
  /// @param contact Contact of which we want to compute the kinematics
  /// @param robot robot the contacts belong to
  /// @param fs force sensor
  /// @param measuredWrench wrench measured at the sensor
  /// @return stateObservation::kine::Kinematics &
  const stateObservation::kine::Kinematics getContactWorldKinematicsAndWrench(KoContactWithSensor & contact,
                                                                              const mc_rbdyn::Robot & robot,
                                                                              const mc_rbdyn::ForceSensor & fs,
                                                                              const sva::ForceVecd & measuredWrench);

  /// @brief Computes the kinematics of the contact attached to the robot in the world frame.
  /// @param contact Contact of which we want to compute the kinematics
  /// @param robot robot the contacts belong to
  /// @param fs force sensor
  /// @return stateObservation::kine::Kinematics &
  const stateObservation::kine::Kinematics getContactWorldKinematics(KoContactWithSensor & contact,
                                                                     const mc_rbdyn::Robot & robot,
                                                                     const mc_rbdyn::ForceSensor & fs);

  /// @brief Updates the measurements of the force sensor attached to a contact.
  /// @details Expresses the measured wrench in the frame of the contact, as the sensor is not necessarily attached to
  /// it.
  /// @param contact Contact associated to the sensor
  /// @param surfaceSensorKine transformation from the sensor to the contact.
  /// @param measuredWrench measured wrench
  void updateContactForceMeasurement(KoContactWithSensor & contact,
                                     stateObservation::kine::Kinematics surfaceSensorKine,
                                     const sva::ForceVecd & measuredWrench);

  /// @brief Updates the measurements of the force sensor attached to a contact.
  /// @details Expresses the measured wrench in the frame of the contact, as the sensor is not necessarily attached to
  /// it.
  /// @param contact Contact associated to the sensor
  /// @param surfaceSensorKine transformation from the sensor to the contact.
  /// @param measuredWrench measured wrench
  void updateContactForceMeasurement(KoContactWithSensor & contact, const sva::ForceVecd & measuredWrench);

  /// @brief Computes the rest pose of the contact in the world using the visco-elastic model.
  /// @details Uses the measured wrench to obtain the rest pose of the contact from the one obtained by forward
  /// kinematics. The visco-elastic model allows to compute the slight displacement resulting from the applied wrench.
  /// @param ctl Controller
  /// @param contact Contact for which we compute the rest pose.
  /// @param worldContactKineRef rest pose of the contact in the world, which is modified by this function.
  void getOdometryWorldContactRest(const mc_control::MCController & ctl,
                                   KoContactWithSensor & contact,
                                   stateObservation::kine::Kinematics & worldContactKineRef);

  /// @brief Update the contact or create it if it still does not exist.
  /// @details Called by \ref updateContacts(const mc_control::MCController & ctl, std::set<std::string> contacts,
  /// mc_rtc::Logger & logger).
  /// @param name The name of the contact to update.
  void updateContact(const mc_control::MCController & ctl, const int & contactIndex, mc_rtc::Logger & logger);

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
  void flexStiffness(const sva::MotionVecd & stiffness);

  /** Set damping of the robot-environment flexibility.
   *
   * \param damping Flexibility damping.
   *
   */
  void flexDamping(const sva::MotionVecd & damping);

  /** Update measurement-noise covariance matrix.
   *
   */
  void updateNoiseCovariance();

  /** Get accelerometer measurement noise covariance.
   *
   */
  inline double accelNoiseCovariance() const { return acceleroSensorCovariance_(0, 0); }

  /** Change accelerometer measurement noise covariance.
   *
   * \param covariance New covariance.
   *
   */
  inline void accelNoiseCovariance(double covariance)
  {
    acceleroSensorCovariance_ = stateObservation::Matrix3::Identity() * covariance;
    updateNoiseCovariance();
  }

  /** Set debug flag.
   *
   * \param flag New debug flag.
   *
   */
  inline void debug(bool flag) { debug_ = flag; }

  /** Get force-sensor measurement noise covariance.
   *
   */
  inline double forceSensorNoiseCovariance() const { return contactSensorCovariance_(0, 0); }

  /** Change force-sensor measurement noise covariance.
   *
   * \param covariance New covariance.
   *
   */
  inline void forceSensorNoiseCovariance(double covariance)
  {
    contactSensorCovariance_.block<3, 3>(0, 0) = stateObservation::Matrix3::Identity() * covariance;
    updateNoiseCovariance();
  }

  /** Get gyrometer measurement noise covariance.
   *
   */
  inline double gyroNoiseCovariance() const { return gyroSensorCovariance_(0, 0); }

  /** Change gyrometer measurement noise covariance.
   *
   * \param covariance New covariance.
   *
   */
  inline void gyroNoiseCovariance(double covariance)
  {
    gyroSensorCovariance_ = stateObservation::Matrix3::Identity() * covariance;
    updateNoiseCovariance();
  }

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
  enum EstimationState
  {
    noIssue,
    errorDetected,
    invincibilityFrame
  };

  // indicates if the current iteration encountered no issue, encountered one, or is inside the invincibility frame
  // (recovery frame after an error)
  EstimationState estimationState_;

  // instance of the Kinetics Observer
  stateObservation::KineticsObserver observer_;
  // name of the estimator
  std::string observerName_ = "MCKineticsObserver";
  // name of the robot
  std::string robot_ = "";
  /* custom list of robots to display */
  std::shared_ptr<mc_rbdyn::Robots> my_robots_;
  // std::string imuSensor_ = "";
  mc_rbdyn::BodySensorVector IMUs_; ///< list of IMUs

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

  /* Settings of the Kinetics Observers */
  // mass of the robot
  double mass_ = 42; // [kg]
  // maximum amount of contacts that we want to use with the Kinetics Observer.
  int maxContacts_ = 4;
  // maximum amount of IMUs that we want to use with the Kinetics Observer.
  int maxIMUs_ = 2;

  // linear stiffness of contacts
  stateObservation::Matrix3 linStiffness_;
  // linear damping of contacts
  stateObservation::Matrix3 linDamping_;
  // angular stiffness of contacts
  stateObservation::Matrix3 angStiffness_;
  // linear damping of contacts
  stateObservation::Matrix3 angDamping_;

  // indicates if the debug logs have to be added.
  bool withDebugLogs_ = true;
  // indicates if we want to perform odometry, and if yes, flat or 6d odometry
  using OdometryType = measurements::OdometryType;
  OdometryType odometryType_;
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

  /* Contacts manager variables */
  using KoContactsManager = measurements::ContactsManager<KoContactWithSensor, measurements::ContactWithoutSensor>;
  KoContactsManager contactsManager_;
  // indicates if the forces measurement have to be filtered with a low-pass filter.
  bool withFilteredForcesContactDetection_ = false;
  // threshold on the measured force for contact detection.
  double contactDetectionThreshold_ = 0.0;
  // list of the force sensors that cannot be used with contacts but we want to use their measurements as inputs to the
  // Kinetics Observer
  std::vector<std::string> forceSensorsAsInput_ = std::vector<std::string>();

  /* IMU variables */
  // manager for the IMUs
  measurements::MapIMUs mapIMUs_;

  /* Utilitary variables */
  // zero frame transformation
  sva::PTransformd zeroPose_;
  // zero velocity or acceleration
  sva::MotionVecd zeroMotion_;
  // kinematics of the CoM within the world frame of the input robot
  stateObservation::kine::Kinematics worldCoMKine_;
  /**< grouped inertia */
  sva::RBInertiad inertiaWaist_;
  // total force measured by the sensors that are not associated to a currently set contact and expressed in the
  // floating base's frame. Used as an input for the Kinetics Observer.
  stateObservation::Vector3 additionalUserResultingForce_ = stateObservation::Vector3::Zero();
  // total torque measured by the sensors that are not associated to a currently set contact and expressed in the
  // floating base's frame. Used as an input for the Kinetics Observer.
  stateObservation::Vector3 additionalUserResultingMoment_ = stateObservation::Vector3::Zero();

  /* Variables for the backup */
  // iteration on which the backup was required for the last time
  int lastBackupIter_;
  // number of iterations on which we perform the backup
  int backupIterInterval_ = 0;
  // time during which the Kinetics Observer is still getting updated by the Tilt Observer after the need of a backup,
  // so the Kalman Filter has time to converge again
  int invincibilityFrame_ = 0;
  // iterations ellapsed within the invincibility frame
  int invincibilityIter_;

  // Buffer containing the estimated pose of the floating base in the world over the whole backup interval.
  boost::circular_buffer<stateObservation::kine::Kinematics> koBackupFbKinematics_;

  /* Debug variables */
  // For logs only. Prediction of the measurements from the newly corrected state
  stateObservation::Vector correctedMeasurements_;
  // For logs only. Kinematics of the centroid frame within the world frame
  stateObservation::kine::Kinematics globalCentroidKinematics_;
};

} // namespace mc_state_observation
