/* Copyright 2017-2020 CNRS-AIST JRL, CNRS-UM LIRMM */

#pragma once

#include <cassert>
#include <unordered_map>
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
  void reset()
  {
    for(auto & [_, contact] : listContacts_) { contact.resetContact(); }
    currentContactsList_.clear();
    contactsDetected_ = false;
  }

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
  stateObservation::Vector6 inputAdditionalWrench(const mc_rbdyn::Robot & inputRobot,
                                                  const mc_rbdyn::Robot & measRobot);

  sva::ForceVecd wrenchWithoutGravity(const mc_rbdyn::ForceSensor & forceSensor,
                                      const sva::PTransformd & X_fb_parent,
                                      const Eigen::Matrix3d & R_fb_world) const;

  sva::ForceVecd wrenchInFloatingBaseFrame(const mc_rbdyn::ForceSensor & forceSensor,
                                           const mc_rbdyn::Robot & inputRobot) const;

  /// @brief Update the IMUs, including the measurements and kinematics in the centroid frame
  /// @param measRobot The control robot
  /// @param inputRobot A robot whose configuration is the one of real robot, but whose pose, velocities and
  /// accelerations are set to zero in the control frame. Allows to ease computations performed in the local frame of
  /// the robot.
  void updateIMUs(const mc_rbdyn::Robot & measRobot, const mc_rbdyn::Robot & inputRobot);

  void updateJointTorqueMeasurement(const mc_rbdyn::Robot & measRobot, mc_rbdyn::Robot & dynamicsRobot);

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

  /// @brief Updates the measurements of the force sensor attached to a contact.
  /// @details Expresses the measured wrench in the frame of the contact. Sensor-based contacts already use the force
  /// sensor frame; surface-based contacts are transformed from the sensor frame to the surface frame.
  /// @param contact Contact associated to the sensor
  /// @param measuredWrench measured wrench
  void updateContactForceMeasurement(KoContactWithSensor & contact, const sva::ForceVecd & measuredWrench);

  /// @brief Hands the observer the contact's wrench measurement covariances, transported from the force
  /// sensor frame to the contact origin.
  void updateContactWrenchMeasNoise(const KoContactWithSensor & contact);

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
                     const std::array<ko_fg::NoiseSigmas3, 4> & initNoises,
                     mc_rtc::Logger & logger);

  /// @brief Updates an already set contact
  /// @param ctl Controller
  /// @param contact Contact to update
  /// @param logger Logger
  void updateContact(const mc_control::MCController & ctl, KoContactWithSensor & contact);

  Eigen::Index actuatedJointTorqueDim(const mc_rbdyn::Robot & robot) const;

  std::vector<std::string> actuatedJointTorqueNames(const mc_rbdyn::Robot & robot) const;

  Eigen::VectorXd jointTorqueVectorFromRefOrder(const mc_rbdyn::Robot & robot) const;

  Eigen::VectorXd measuredJointTorqueVector(const mc_rbdyn::Robot & robot) const;

  Eigen::VectorXd actuatedJointVelocityVector(const mc_rbdyn::Robot & robot) const;

  std::shared_ptr<const ko_fg::RecursiveJointTorqueModel> makeJointTorqueModel(
      mc_rbdyn::Robot & dynamicsRobot) const;

  ko_fg::MomentumResidualEndpoint makeMomentumResidualEndpoint(mc_rbdyn::Robot & dynamicsRobot);

  ko_fg::ReducedJointMomentumEndpoint makeReducedJointMomentumEndpoint(
      mc_rbdyn::Robot & dynamicsRobot, const Eigen::VectorXd & actuatorTorque);

  ko_fg::ReducedJointMomentumMeasurement makeReducedJointMomentumMeasurement(
      const mc_rbdyn::Robot & measRobot, mc_rbdyn::Robot & dynamicsRobot);

  void updateEstimatedJointTorqueResidual();

  inline stateObservation::kine::Kinematics fgLocKineToSoKine(const ko_fg::LocKinematics & locK) const
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
  bool withGui_ = false;

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

  // Surfaces whose force sensors are reserved for validating the disturbance-wrench estimate. Their measurements are
  // logged in the centroid frame and never passed to the factor graph, either as contacts or as additional wrenches.
  std::unordered_set<std::string> contactsIgnoredForEstimation_;
  std::unordered_map<std::string, std::string> ignoredForceSensorSurfaces_;
  std::unordered_map<std::string, stateObservation::Vector6> ignoredWrenchesInCentroid_;

  bool useJointTorqueMeasurements_ = false;
  bool useJointTorqueCommandAsMeasurement_ = false;
  double jointTorqueNoise_ = 10.0;
  double jointAccelerationInitNoise_ = 100.0;
  double jointAccelerationProcessNoise_ = 50.0;
  double jointAccelerationFiniteDifferenceNoise_ = 25.0;
  /// Which of the core's three mutually exclusive joint-torque formulations to use. FullDynamics
  /// needs the joint accelerations; ReducedMomentum integrates instead and does not.
  enum class JointTorqueFormulation
  {
    FullDynamics,
    ReducedMomentum
  };
  JointTorqueFormulation jointTorqueFormulation_ = JointTorqueFormulation::FullDynamics;
  double jointMomentumNoise_ = 1.0;
  double jointMomentumModelNoise_ = 10.0;
  Eigen::VectorXd inputJointTorques_;
  /// Warn once, not at every iteration, when the robot turns out to have no torque sensing.
  mutable bool warnedAboutNullJointTorques_ = false;
  Eigen::VectorXd measuredJointTorques_;
  Eigen::VectorXd estimatedJointTorqueResidual_;
  Eigen::VectorXd jointAccelerationFiniteDifference_;
  std::optional<Eigen::VectorXd> previousJointVelocity_;
  std::optional<ko_fg::JointTorqueMeasurement> pendingJointAccelerationMeasurement_;
  std::vector<size_t> pendingJointAccelerationContacts_;
  size_t pendingJointAccelerationTime_ = 0;
  std::vector<std::string> jointTorqueNames_;
  std::optional<ko_fg::ReducedJointMomentumEndpoint> previousMomentumEndpoint_;
  std::optional<Eigen::VectorXd> previousMeasuredJointTorques_;
  std::optional<ko_fg::ReducedJointMomentumMeasurement> pendingMomentumMeasurement_;
  std::vector<ko_fg::ReducedJointMomentumContactInterval> pendingMomentumContacts_;
  size_t pendingMomentumTime_ = 0;

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
  // Every noise is per-axis: the estimated quantities are not isotropic. The contact rest position
  // drifts far less vertically than horizontally, the rest orientation drifts in yaw and almost not
  // in roll and pitch, and gravity makes two axes of the gyrometer bias observable and not the
  // third. These are the same standard deviations as the EKF implementation, which has always been
  // configured axis by axis.
  std::array<ko_fg::NoiseSigmas3, 10> initNoises_;
  std::array<ko_fg::NoiseSigmas3, 8> processNoises_;
  std::unordered_map<size_t, std::array<ko_fg::NoiseSigmas3, 4>> imuNoises_;

  /*
  Contact flexibility tuning
  - linear stiffness
  - angular stiffness
  - linear damping
  - angular damping
  */
  std::array<stateObservation::Matrix3, 4> contactFlexibilities_;
  std::array<ko_fg::NoiseSigmas3, 4> contactInitNoises_;
  std::array<ko_fg::NoiseSigmas3, 4> contactInitNoises_first_;
  std::array<ko_fg::NoiseSigmas3, 4> contactProcessNoises_;
  std::array<ko_fg::NoiseSigmas3, 2> contactMeasNoises_;
  // Per-surface rotation correcting the direction of the measured force (rotation vector).
  std::unordered_map<std::string, stateObservation::Vector3> wrenchCalibration_;


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

  // Anchor positions used when running without odometry to reconstruct a world floating-base pose from maintained
  // contacts, following the same semantics as MCKineticsObserver.
  stateObservation::Vector3 worldAnchorPos_ = stateObservation::Vector3::Zero();
  stateObservation::Vector3 fbAnchorPos_ = stateObservation::Vector3::Zero();

  bool withRestPoseAverageFactor_ = false;
};

} // namespace mc_state_observation
