/**
 * \file      leggedOdometryTools.h
 * \author    Arnaud Demont, Mehdi Benallegue
 * \date       2023
 * \brief      Library for an easened legged odometry implementation.
 *
 * \details
 *
 *
 */

#pragma once

#include <mc_state_observation/observersTools/measurementsTools.h>
#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

namespace mc_state_observation
{
namespace leggedOdometry
{

/**
 * Interface for the implementation of legged odometry. This odometry is based on the tracking of successive contacts
 * for the estimation of the pose of the floating base of the robot.
 * The tilt cannot be estimated from this method (but the yaw can), it has to be estimated beforehand by another
 * observer. One can decide to perform flat or 6D odometry. The flat odometry considers that the robot walks on a flat
 * ground and corrects the estimated height accordingly, it is preferable in this use case.
 * The odometry manager must be initialized once all the configuration parameters are retrieved using the init function,
 * and called on every iteration with \ref LeggedOdometryManager::run(const mc_control::MCController & ctl,
 * mc_rtc::Logger & logger, sva::PTransformd & pose, sva::MotionVecd & vel, sva::MotionVecd & acc).
 **/

///////////////////////////////////////////////////////////////////////
/// ------------------------------Contacts-----------------------------
///////////////////////////////////////////////////////////////////////
// Enhancement of the class ContactWithSensor with the reference of the contact in the world and the force measured by
// the associated sensor
class LoContactWithSensor : public measurements::ContactWithSensor
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
protected:
  LoContactWithSensor() {}

public:
  LoContactWithSensor(int id, std::string forceSensorName)
  {
    id_ = id;
    name_ = forceSensorName;
    forceSensorName_ = forceSensorName;

    resetContact();
  }

  LoContactWithSensor(int id,
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
  // reference of the contact in the world
  stateObservation::kine::Kinematics worldRefKine_;
  // indicates whether the contact can be used for the orientation odometry or not
  bool useForOrientation_ = false;
  // norm of the force measured by the sensor
  double forceNorm_ = 0.0;
  // current estimation of the kinematics of the floating base in the world, obtained from the reference pose of the
  // contact in the world
  stateObservation::kine::Kinematics currentWorldFbPose_;
  // current estimation of the kinematics of the contact in the world
  stateObservation::kine::Kinematics currentWorldKine_;
};

// Inherits from the class ContactWithoutSensor to prevent
class LoContactWithoutSensor : public measurements::ContactWithoutSensor
{
  // the legged odometry requires the use of contacts associated to force sensors, this class must therefore not be
  // implemented
public:
  LoContactWithoutSensor(int id, std::string name)
  {
    throw std::runtime_error("The legged odometry requires to use only contacts with sensors.");
    // BOOST_ASSERT(false && "The legged odometry requires to use only contacts with sensors.");
    id_ = id;
    name_ = name;
  }

protected:
  LoContactWithoutSensor()
  {
    throw std::runtime_error("The legged odometry requires to use only contacts with sensors.");
    // BOOST_ASSERT(false && "The legged odometry requires to use only contacts with sensors.");
  }
};

/// @brief Structure that implements all the necessary functions to perform legged odometry.
/// @details Handles the odometry from the contacts detection to the final pose estimation of the floating base. Also
/// allows to compute the pose of an anchor frame linked to the robot.
struct LeggedOdometryManager
{
public:
  typedef measurements::ContactsManager<LoContactWithSensor, LoContactWithoutSensor> ContactsManager;

public:
  LeggedOdometryManager() {}

protected:
  ///////////////////////////////////////////////////////////////////////
  /// ------------------------Contacts Manager---------------------------
  ///////////////////////////////////////////////////////////////////////

  /// @brief Adaptation of the structure ContactsManager to the legged odometry, using personalized contacts classes.
  struct LeggedOdometryContactsManager : public ContactsManager
  {
  protected:
    // comparison function that sorts the contacts based on their measured force.
    struct sortByForce
    {
      inline bool operator()(const LoContactWithSensor & contact1, const LoContactWithSensor & contact2) const
      {
        return (contact1.forceNorm_ < contact2.forceNorm_);
      }
    };

  public:
    // list of contacts used for the orientation odometry. At most two contacts can be used for this estimation, and
    // contacts at hands are not considered. The contacts with the highest measured force are used.
    std::set<std::reference_wrapper<LoContactWithSensor>, sortByForce> oriOdometryContacts_;
  };

public:
  /// @brief Initializer for the odometry manager.
  /// @details Version for the contact detection using a thresholding on the contact force sensors measurements or by
  /// direct input from the solver.
  /// @param ctl Controller
  /// @param robotName Name of the robot
  /// @param odometryName Name of the odometry, used in logs and in the gui.
  /// @param odometryType Indicates if the desired odometry must be a flat or a 6D odometry.
  /// @param withYawEstimation Indicates if the orientation must be estimated by this odometry.
  /// @param velUpdatedUpstream Informs whether the 6D velocity was updated by upstream observers
  /// @param accUpdatedUpstream Informs whether the acceleration was updated by upstream observers
  /// @param verbose
  /// @param withModeSwitchInGui If true, adds the possiblity to switch between 6d and flat odometry from the gui.
  /// Should be set to false if this feature is implemented in the estimator using this library.
  void init(const mc_control::MCController & ctl,
            const std::string & robotName,
            const std::string & odometryName,
            measurements::OdometryType & odometryType,
            const bool withYawEstimation,
            const bool velUpdatedUpstream,
            const bool accUpdatedUpstream,
            const bool verbose,
            const bool withModeSwitchInGui = false);

  /// @brief Initialization for a detection based on contact surfaces
  /// @param ctl Controller
  /// @param robotName name of the robot
  /// @param contactsDetection mean of detection for the contacts
  /// @param surfacesForContactDetection list of possible contact surfaces
  /// @param contactsSensorDisabledInit list of the force sensors that must be disabled on initialization.
  /// @param contactDetectionThreshold threshold on the measured force for the contact detection
  void initDetection(const mc_control::MCController & ctl,
                     const std::string & robotName,
                     const ContactsManager::ContactsDetection & contactsDetection,
                     const std::vector<std::string> & surfacesForContactDetection,
                     const std::vector<std::string> & contactsSensorDisabledInit,
                     const double & contactDetectionThreshold);

  /// @brief initialization for a detection based on a threshold on the measured contact forces or for contacts given by
  /// the controller
  /// @param ctl Controller
  /// @param robotName name of the robot
  /// @param contactsDetection mean of detection for the contacts
  /// @param contactsSensorDisabledInit list of the force sensors that must be disabled on initialization.
  /// @param contactDetectionThreshold threshold on the measured force for the contact detection
  /// @param forceSensorsToOmit list of force sensors that cannot be used for the contacts detection
  void initDetection(const mc_control::MCController & ctl,
                     const std::string & robotName,
                     const ContactsManager::ContactsDetection & contactsDetection,
                     const std::vector<std::string> & contactsSensorDisabledInit,
                     const double & contactDetectionThreshold,
                     const std::vector<std::string> & forceSensorsToOmit);

  /// @brief @copybrief run(const mc_control::MCController &, mc_rtc::Logger &, sva::PTransformd &, sva::MotionVecd &,
  /// sva::MotionVecd &, stateObservation::Matrix3). This version uses the tilt estimated by the upstream observers.
  /// @copydetails run(const mc_control::MCController &, mc_rtc::Logger &, sva::PTransformd &, sva::MotionVecd &,
  /// sva::MotionVecd &, stateObservation::Matrix3)
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param vel The 6D velocity of the floating base in the world that we want to update
  /// @param acc The acceleration of the floating base in the world that we want to update
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           sva::MotionVecd & vel,
           sva::MotionVecd & acc);

  /// @brief @copybrief run(const mc_control::MCController &, mc_rtc::Logger &, sva::PTransformd &, sva::MotionVecd &,
  /// stateObservation::Matrix3). This version uses the tilt estimated by the upstream observers.
  /// @copydetails run(const mc_control::MCController &, mc_rtc::Logger &, sva::PTransformd &, sva::MotionVecd &,
  /// stateObservation::Matrix3)
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           sva::MotionVecd & vel);

  /// @brief @copybrief run(const mc_control::MCController &, mc_rtc::Logger &, sva::PTransformd &,
  /// stateObservation::Matrix3). This version uses the tilt estimated by the upstream observers.
  /// @copydetails run(const mc_control::MCController &, mc_rtc::Logger &, sva::PTransformd &, stateObservation::Matrix3)
  void run(const mc_control::MCController & ctl, mc_rtc::Logger & logger, sva::PTransformd & pose);

  /// @brief Core function runing the odometry.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param vel The 6D velocity of the floating base in the world that we want to update
  /// @param acc The acceleration of the floating base in the world that we want to update
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           sva::MotionVecd & vel,
           sva::MotionVecd & acc,
           const stateObservation::Matrix3 & tilt);

  /// @brief Core function runing the odometry.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param vel The 6D velocity of the floating base in the world that we want to update
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           sva::MotionVecd & vel,
           const stateObservation::Matrix3 & tilt);

  /// @brief Core function runing the odometry.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           const stateObservation::Matrix3 & tilt);

  /// @brief Updates the joints configuration of the odometry robot. Has to be called at the beginning of each
  /// iteration.
  /// @param ctl Controller
  void updateJointsConfiguration(const mc_control::MCController & ctl);

  /// @brief Updates the pose of the contacts and estimates the floating base from them.
  /// @param ctl Controller.
  /// @param logger Logger.
  /// @param updateVels Indicates if the 6D velocity of the floating base must be updated
  /// @param updateAccs Indicates if the acceleration of the floating base must be updated
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  void updateFbAndContacts(const mc_control::MCController & ctl,
                           mc_rtc::Logger & logger,
                           const bool updateVels,
                           const bool updateAccs,
                           const stateObservation::Matrix3 & tilt);

  /// @brief If the contacts respect the conditions, computes the pose of the floating base for each set contact.
  /// @details Combines the reference pose of the contact in the world and the transformation from the contact to the
  /// frame.
  /// @param ctl Controller.
  /// @param posUpdatable Indicates if the position can be updated using contacts
  /// @param oriUpdatable Indicates if the orientation can be updated using contacts
  /// @param sumForces_position Sum of the measured force of all the contacts that will be used for the position
  /// estimation
  /// @param sumForces_orientation Sum of the measured force of all the contacts that will be used for the orientation
  /// estimation
  void getFbFromContacts(const mc_control::MCController & ctl,
                         bool & posUpdatable,
                         bool & oriUpdatable,
                         double & sumForces_position,
                         double & sumForces_orientation);

  /// @brief Updates the floating base kinematics given as argument by the observer.
  /// @details Must be called after \ref updateFbAndContacts(const mc_control::MCController & ctl, mc_rtc::Logger &
  /// logger, const bool updateVels, const bool updateAccs).Beware, only the pose is updated by the odometry, the
  /// 6D velocity (except if not updated by an upstream observer) and acceleration update only performs a
  /// transformation from the real robot to our newly estimated robot. If you want to update the acceleration of
  /// the floating base, you need to add an observer computing them beforehand.
  /// @param ctl Controller.
  /// @param updateVels If true, the velocity of the floating base of the odometry robot is updated from the one of
  /// the real robot.
  /// @param updateAccs If true, the acceleration of the floating base of the odometry robot is updated from the one
  /// of the real robot. This acceleration must be computed by an upstream observer..
  void updateOdometryRobot(const mc_control::MCController & ctl, const bool updateVels, const bool updateAccs);

  /// @brief Updates the floating base kinematics given as argument by the observer.
  /// @details Beware, only the pose is updated by the odometry, the 6D velocity (except if not updated by an upstream
  /// observer) and acceleration update only performs a transformation from the real robot to our newly estimated
  /// robot. If you want to update the acceleration of the floating base, you need to add an observer computing them
  /// beforehand.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param vel The 6D velocity of the floating base in the world that we want to update.
  /// @param acc The acceleration of the floating base in the world that we want to update. This acceleration must
  /// come from an upstream observer.
  /// @param logger logger
  void updateFbKinematics(sva::PTransformd & pose, sva::MotionVecd & vel, sva::MotionVecd & acc);

  /// @brief Updates the floating base kinematics given as argument by the observer.
  /// @details Beware, only the pose is updated by the odometry, the 6D velocity update only performs a transformation
  /// from the real robot to our newly estimated robot.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param vel The 6D velocity of the floating base in the world that we want to update.
  /// @param logger logger
  void updateFbKinematics(sva::PTransformd & pose, sva::MotionVecd & vel);

  /// @brief Updates the floating base kinematics given as argument by the observer.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param logger logger
  void updateFbKinematics(sva::PTransformd & pose);

  /// @brief Computes the reference kinematics of the newly set contact in the world.
  /// @param contact The new contact
  /// @param measurementsRobot The robot containing the contact's force sensor
  void setNewContact(LoContactWithSensor & contact, const mc_rbdyn::Robot & measurementsRobot);

  /// @brief Computes the kinematics of the contact attached to the odometry robot in the world frame.
  /// @param contact Contact of which we want to compute the kinematics
  /// @param fs The force sensor associated to the contact
  /// @return stateObservation::kine::Kinematics &
  const stateObservation::kine::Kinematics & getCurrentContactKinematics(LoContactWithSensor & contact,
                                                                         const mc_rbdyn::ForceSensor & fs);

  /// @brief Select which contacts to use for the orientation odometry
  /// @details The two contacts with the highest measured force are selected. The contacts at hands are ignored because
  /// their orientation is less trustable.
  void selectForOrientationOdometry();

  /// @brief Returns the pose of the odometry robot's anchor frame based on the current floating base and encoders.
  /// @details The anchor frame can be obtained using 2 ways:
  /// - 1: contacts are detected and can be used to compute the anchor frame.
  /// - 2: no contact is detected, the robot is hanging. If we still need an anchor frame for the tilt estimation we
  /// arbitrarily use the frame of the bodySensor used by the estimator.
  /// @param ctl controller
  /// @param bodySensorName name of the body sensor.
  stateObservation::kine::Kinematics & getAnchorFramePose(const mc_control::MCController & ctl,
                                                          const std::string & bodySensorName);

  /// @brief Returns the pose of the odometry robot's anchor frame. If no contact is detected, this version does not
  /// update the anchor frame.
  /// @param ctl controller
  stateObservation::kine::Kinematics & getAnchorFramePose(const mc_control::MCController & ctl);

  /// @brief Changes the type of the odometry
  /// @param newOdometryType The new type of odometry to use.
  void changeOdometryType(const std::string & newOdometryType);

  /// @brief Changes the type of the odometry.
  /// @details Version meant to be called by the observer using the odometry.
  /// @param newOdometryType The new type of odometry to use.
  void changeOdometryType(const measurements::OdometryType & newOdometryType);

  /// @brief Add the log entries corresponding to the contact.
  /// @param logger
  /// @param contactName
  void addContactLogEntries(mc_rtc::Logger & logger, const LoContactWithSensor & contact);

  /// @brief Remove the log entries corresponding to the contact.
  /// @param logger
  /// @param contactName
  void removeContactLogEntries(mc_rtc::Logger & logger, const LoContactWithSensor & contact);

  /// @brief Getter for the odometry robot used for the estimation.
  mc_rbdyn::Robot & odometryRobot() { return odometryRobot_->robot("odometryRobot"); }

  /// @brief Getter for the contacts manager.
  LeggedOdometryContactsManager & contactsManager() { return contactsManager_; }

public:
  // Indicates if the mode of computation of the anchor frame changed. Might me needed by the estimator (ex;
  // TiltObserver)
  bool prevAnchorFromContacts_ = true;
  // Indicates if the desired odometry must be a flat or a 6D odometry.
  using OdometryType = measurements::OdometryType;
  measurements::OdometryType odometryType_;

protected:
  // Name of the odometry, used in logs and in the gui.
  std::string odometryName_;
  // Name of the robot
  std::string robotName_;

  // indicates whether we want to update the yaw using this method or not
  bool withYawEstimation_;
  // tracked pose of the floating base
  sva::PTransformd fbPose_ = sva::PTransformd::Identity();

protected:
  // contacts manager used by this odometry manager
  LeggedOdometryContactsManager contactsManager_;
  // odometry robot that is updated by the legged odometry and can then update the real robot if required.
  std::shared_ptr<mc_rbdyn::Robots> odometryRobot_;
  // pose of the anchor frame of the robot in the world
  stateObservation::kine::Kinematics worldAnchorPose_;

  // Indicates whether the velocity is updated by an upstream estimator. If yes, it is expressed in the newly obtained
  // floating base frame. Otherwise, it is computed by finite differences.
  bool velUpdatedUpstream_ = false;
  // Indicates whether the acceleration is updated by an upstream estimator. If yes, it is expressed in the newly
  // obtained floating base frame. Otherwise, it is not updated.
  bool accUpdatedUpstream_ = false;
};

} // namespace leggedOdometry

} // namespace mc_state_observation
