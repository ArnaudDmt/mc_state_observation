#pragma once

#include <mc_state_observation/measurements/ContactsManager.h>
#include <mc_state_observation/measurements/measurements.h>

#include <state-observation/dynamics-estimators/kinetics-observer.hpp>

namespace mc_state_observation::odometry
{

/**
 * Interface for the implementation of legged odometry. This odometry is based on the tracking of successive contacts
 * for the estimation of the pose of the floating base of the robot.

 * The tilt cannot be estimated from this method (but the yaw can), it has to be estimated beforehand by another
 * observer.
 * One can decide to perform flat or 6D odometry. The flat odometry considers that the robot walks on a flat
 * ground and corrects the estimated height accordingly, it is preferable in this use case.
 *
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
  using measurements::ContactWithSensor::ContactWithSensor;

public:
  // reference of the contact in the world
  stateObservation::kine::Kinematics worldRefKine_;
  // indicates whether the contact can be used for the orientation odometry or not
  bool useForOrientation_ = false;
  // current estimation of the kinematics of the floating base in the world, obtained from the reference pose of the
  // contact in the world
  stateObservation::kine::Kinematics currentWorldFbPose_;
  // current estimation of the kinematics of the contact in the world
  stateObservation::kine::Kinematics currentWorldKine_;
};

/// @brief Structure that implements all the necessary functions to perform legged odometry.
/// @details Handles the odometry from the contacts detection to the final pose estimation of the floating base. Also
/// allows to compute the pose of an anchor frame linked to the robot.
struct LeggedOdometryManager
{
public:
  using ContactsManager = measurements::ContactsManager<LoContactWithSensor>;

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
      inline bool operator()(const LoContactWithSensor & contact1, const LoContactWithSensor & contact2) const noexcept
      { return (contact1.forceNorm() < contact2.forceNorm()); }
    };

  public:
    // list of contacts used for the orientation odometry. At most two contacts can be used for this estimation, and
    // contacts at hands are not considered. The contacts with the highest measured force are used.
    std::set<std::reference_wrapper<LoContactWithSensor>, sortByForce> oriOdometryContacts_;
  };

public:
  enum class VelocityUpdate
  {
    NoUpdate,
    FiniteDiff,
    FromUpstream
  };

private:
  // map allowing to get the VelocityUpdate value associated to the given string
  inline static const std::unordered_map<std::string, VelocityUpdate> strToVelocityUpdate_ = {
      {"FiniteDiff", VelocityUpdate::FiniteDiff},
      {"FromUpstream", VelocityUpdate::FromUpstream},
      {"NoUpdate", VelocityUpdate::NoUpdate}};

public:
  ////////////////////////////////////////////////////////////////////
  /// ------------------------Configuration---------------------------
  ////////////////////////////////////////////////////////////////////

  /// @brief Configuration structure that helps setting up the odometry parameters
  /// @details The configuration is used once passed in the @ref init(const mc_control::MCController &, Configuration,
  /// ContactsManagerConfiguration) function
  struct Configuration
  {
    /// @brief Configuration's constructor
    /// @details This version allows to set the odometry type directly from a string, most likely obtained from a
    /// configuration file.
    inline Configuration(const std::string & robotName,
                         const std::string & odometryName,
                         const std::string & odometryTypeString) noexcept
    : robotName_(robotName), odometryName_(odometryName)
    {
      odometryType_ = measurements::stringToOdometryType(odometryTypeString, odometryName);
      if(odometryType_ != measurements::OdometryType::Flat && odometryType_ != measurements::OdometryType::Odometry6d)
      {
        mc_rtc::log::error_and_throw<std::runtime_error>(
            "Odometry type not allowed. Please pick among : [Odometry6d, Flat] or use the other Configuration "
            "constructor for an estimator that can run without odometry.");
      }
    }

    /// @brief Configuration's constructor
    /// @details This versions allows to initialize the type of odometry directly with an OdometryType object.
    inline Configuration(const std::string & robotName,
                         const std::string & odometryName,
                         measurements::OdometryType odometryType) noexcept
    : robotName_(robotName), odometryName_(odometryName), odometryType_(odometryType)
    {
    }

    // Name of the robot
    std::string robotName_;
    // Name of the odometry, used in logs and in the gui.
    std::string odometryName_;
    // Desired kind of odometry (6D or flat)
    measurements::OdometryType odometryType_;

    // Indicates if the orientation must be estimated by this odometry.
    bool withYaw_ = true;
    // If true, adds the possiblity to switch between 6d and flat odometry from the gui.
    // Should be set to false if this feature is implemented in the estimator using this library.
    bool withModeSwitchInGui_ = false;
    // Indicates if we want to update the velocity and what method it must be updated with.
    VelocityUpdate velocityUpdate_ = LeggedOdometryManager::VelocityUpdate::NoUpdate;

    inline Configuration & withModeSwitchInGui(bool withModeSwitchInGui) noexcept
    {
      withModeSwitchInGui_ = withModeSwitchInGui;
      return *this;
    }
    inline Configuration & withYawEstimation(bool withYaw) noexcept
    {
      withYaw_ = withYaw;
      return *this;
    }

    /// @brief Sets the velocity update method used in the odometry.
    /// @details Allows to set the velocity update method directly from a string, most likely obtained from a
    /// configuration file.
    /// @param velocityUpdate The method to be used.
    inline Configuration & velocityUpdate(VelocityUpdate velocityUpdate) noexcept
    {
      velocityUpdate_ = velocityUpdate;
      return *this;
    }

    /// @brief Sets the velocity update method used in the odometry.
    /// @details Allows to set the velocity update method directly from a string, most likely obtained from a
    /// configuration file.
    /// @param str The string naming the desired velocity update method
    inline Configuration & velocityUpdate(const std::string & str) noexcept
    {
      velocityUpdate_ = LeggedOdometryManager::stringToVelocityUpdate(str, odometryName_);
      return *this;
    }
  };

  using ContactsManagerConfiguration = ContactsManager::Configuration;

  inline LeggedOdometryManager(const std::string & odometryName) { odometryName_ = odometryName; }

  /// @brief Initializer for the odometry manager.
  /// @details Version for the contact detection using a thresholding on the contact force sensors measurements or by
  /// direct input from the solver.
  /// @param ctl Controller
  /// @param odomConfig Desired configuraion of the odometry
  /// @param verbose
  void init(const mc_control::MCController & ctl,
            const Configuration & odomConfig,
            const ContactsManagerConfiguration & contactsConf);

  /// @brief @copybrief run(const mc_control::MCController & ctl, mc_rtc::Logger &, sva::PTransformd &, const
  /// stateObservation::Matrix3 &). This version uses the tilt estimated by the upstream observers.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param logger Logger
  void run(const mc_control::MCController & ctl, mc_rtc::Logger & logger, sva::PTransformd & pose);

  /// @brief Core function runing the odometry.
  /// @param ctl Controller
  /// @param logger Logger
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           const stateObservation::Matrix3 & tilt);

  /// @brief @copybrief run(const mc_control::MCController &, mc_rtc::Logger &, sva::PTransformd &, const
  /// stateObservation::Matrix3 &, sva::MotionVecd &). This version uses the tilt estimated by the upstream observers.
  /// @param ctl Controller
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param logger Logger
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           sva::MotionVecd & vel);

  /// @brief Core function runing the odometry.
  /// @param ctl Controller
  /// @param logger Logger
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           const stateObservation::Matrix3 & tilt,
           sva::MotionVecd & vel);

  /// @brief @copybrief run(const mc_control::MCController & ctl, mc_rtc::Logger &,  sva::PTransformd &, const
  /// stateObservation::Matrix3 &, sva::MotionVecd &, sva::MotionVecd &) run(const mc_control::MCController &,
  /// mc_rtc::Logger &, sva::PTransformd &, sva::MotionVecd &, sva::MotionVecd &, stateObservation::Matrix3). This
  /// version uses the tilt estimated by the upstream observers.
  /// @param ctl Controller
  /// @param logger Logger
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  /// @param acc The acceleration of the floating base in the world that we want to update
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           sva::MotionVecd & vel,
           sva::MotionVecd & acc);

  /// @brief Core function runing the odometry.
  /// @param ctl Controller
  /// @param logger Logger
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  /// @param acc The acceleration of the floating base in the world that we want to update
  void run(const mc_control::MCController & ctl,
           mc_rtc::Logger & logger,
           sva::PTransformd & pose,
           const stateObservation::Matrix3 & tilt,
           sva::MotionVecd & vel,
           sva::MotionVecd & acc);

  /// @brief Returns the pose of the odometry robot's anchor frame based on the current floating base and encoders.
  /// @details The anchor frame can can from 2 sources:
  /// - 1: contacts are detected and can be used to compute the anchor frame.
  /// - 2: no contact is detected, the robot is hanging. If we still need an anchor frame for the tilt estimation we
  /// arbitrarily use the frame of the bodySensor used by the estimator.
  /// @param ctl controller
  /// @param bodySensorName name of the body sensor.
  stateObservation::kine::Kinematics & getAnchorFramePose(const mc_control::MCController & ctl,
                                                          const std::string & bodySensorName);

  /// @brief Changes the type of the odometry
  /// @details Version meant to be called by the observer using the odometry during the run through the gui.
  /// @param newOdometryType The string naming the new type of odometry to use.
  void setOdometryType(measurements::OdometryType newOdometryType);

  /// @brief Getter for the odometry robot used for the estimation.
  inline mc_rbdyn::Robot & odometryRobot() { return odometryRobot_->robot("odometryRobot"); }

  /// @brief Getter for the contacts manager.
  inline LeggedOdometryContactsManager & contactsManager() { return contactsManager_; }

  /// @brief Returns a VelocityUpdate object corresponding to the given string.
  /// @details Allows to set the velocity update method directly from a string, most likely obtained from a
  /// configuration file.
  /// @param str The string naming the desired velocity update method
  inline static VelocityUpdate stringToVelocityUpdate(const std::string & str, const std::string & odometryName)
  {
    auto it = strToVelocityUpdate_.find(str);
    if(it != strToVelocityUpdate_.end()) { return it->second; }
    mc_rtc::log::error_and_throw<std::runtime_error>("[{}]: No known VelocityUpdate value for {}", odometryName, str);
  }

private:
  /// @brief Core function runing the odometry.
  /// @param ctl Controller
  /// @param logger Logger
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param tilt The floating base's tilt, estimated either by the estimator using this library or by an upstream
  /// estimator.
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  /// @param acc The acceleration of the floating base in the world that we want to update
  void runPvt(const mc_control::MCController & ctl,
              mc_rtc::Logger & logger,
              sva::PTransformd & pose,
              const stateObservation::Matrix3 * tilt = nullptr,
              sva::MotionVecd * vel = nullptr,
              sva::MotionVecd * acc = nullptr);

  /// @brief Updates the floating base kinematics given as argument by the observer.
  /// @details Beware, only the pose is updated by the odometry, the 6D velocity (except if not updated by an upstream
  /// observer) and acceleration update only performs a transformation from the real robot to our newly estimated
  /// robot. If you want to update the acceleration of the floating base, you need to add an observer computing them
  /// beforehand.
  /// @param pose The pose of the floating base in the world that we want to update
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  /// @param acc The acceleration of the floating base in the world that we want to update. This acceleration must
  /// come from an upstream observer.
  void updateFbKinematicsPvt(sva::PTransformd & pose, sva::MotionVecd * vel = nullptr, sva::MotionVecd * acc = nullptr);

  /// @brief Updates the joints configuration of the odometry robot. Has to be called at the beginning of each
  /// iteration.
  /// @param ctl Controller
  void updateJointsConfiguration(const mc_control::MCController & ctl);

  /// @brief Updates the pose of the contacts and estimates the floating base from them.
  /// @param ctl Controller.
  /// @param logger Logger.
  /// @param tilt The floating base's tilt (only the yaw is estimated).
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  /// @param acc The floating base's tilt (only the yaw is estimated).
  void updateFbAndContacts(const mc_control::MCController & ctl,
                           mc_rtc::Logger & logger,
                           const stateObservation::Matrix3 & tilt,
                           sva::MotionVecd * vel = nullptr,
                           sva::MotionVecd * acc = nullptr);

  /// @brief Updates the floating base kinematics given as argument by the observer.
  /// @details Must be called after \ref updateFbAndContacts(const mc_control::MCController & ctl, mc_rtc::Logger &,
  /// const stateObservation::Matrix3 &, sva::MotionVecd *, sva::MotionVecd *).
  /// Beware, only the pose is updated by the odometry. The 6D velocity (except if not updated by an upstream observer)
  /// is obtained using finite differences or by expressing the one given in input in the new robot frame. The
  /// acceleration can only be updated if estimated by an upstream estimator.
  /// @param ctl Controller.
  /// @param vel The 6D velocity of the floating base in the world that we want to update. \velocityUpdate_ must be
  /// different from noUpdate, otherwise, it will not be updated.
  /// @param acc The floating base's tilt (only the yaw is estimated).
  void updateOdometryRobot(const mc_control::MCController & ctl,
                           sva::MotionVecd * vel = nullptr,
                           sva::MotionVecd * acc = nullptr);

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
  /// @brief Selects which contacts to use for the orientation odometry and computes the orientation of the floating
  /// base for each of them
  /// @details The two contacts with the highest measured force are selected. The contacts at hands are ignored because
  /// their orientation is less trustable.
  /// @param oriUpdatable Indicates that contacts can be used to estimated the orientation.
  /// @param sumForcesOrientation Sum of the forces measured at the contacts used for the orientation estimation
  /// @param worldFbPose Current estimate of the pose of the floating base in the world.
  void selectForOrientationOdometry(bool & oriUpdatable,
                                    double & sumForcesOrientation,
                                    const stateObservation::kine::Kinematics & worldFbPose);

  /// @brief Add the log entries corresponding to the contact.
  /// @param logger
  /// @param contactName
  void addContactLogEntries(mc_rtc::Logger & logger, const LoContactWithSensor & contact);

  /// @brief Remove the log entries corresponding to the contact.
  /// @param logger
  /// @param contactName
  void removeContactLogEntries(mc_rtc::Logger & logger, const LoContactWithSensor & contact);

public:
  // Indicates if the mode of computation of the anchor frame changed. Might me needed by the estimator (ex;
  // TiltObserver)
  bool prevAnchorFromContacts_ = true;
  // Indicates if the desired odometry must be a flat or a 6D odometry.
  using OdometryType = measurements::OdometryType;
  measurements::OdometryType odometryType_;

  // indicates if the velocity has to be updated, if yes, how it must be updated
  VelocityUpdate velocityUpdate_;

protected:
  // Name of the odometry, used in logs and in the gui.
  std::string odometryName_;
  // Name of the robot
  std::string robotName_;

  // indicates whether we want to update the yaw using this method or not
  bool withYawEstimation_;
  // tracked pose of the floating base
  sva::PTransformd fbPose_ = sva::PTransformd::Identity();

  // contacts manager used by this odometry manager
  LeggedOdometryContactsManager contactsManager_;
  // odometry robot that is updated by the legged odometry and can then update the real robot if required.
  std::shared_ptr<mc_rbdyn::Robots> odometryRobot_;
  // pose of the anchor frame of the robot in the world
  stateObservation::kine::Kinematics worldAnchorPose_;
};

} // namespace mc_state_observation::odometry
