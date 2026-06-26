#pragma once

#include <mc_control/MCController.h>
#include <mc_observers/Observer.h>
#include <boost/circular_buffer.hpp>

#include <mc_state_observation/measurements/ContactsDetector.hpp>

#include <state-observation/observer/waiko-humanoid.hpp>
#include <state-observation/tools/odometry/legged-odometry-manager.hpp>

namespace mc_state_observation
{

struct MCWaiko : public mc_observers::Observer
{
  // we define MCKineticsObserver as a friend as it can instantiate this observer as a backup
  friend struct MCKineticsObserver;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  /// @brief Constructor for the MCWaiko.
  /// @details The parameters asBackup is given only if Waiko is used as a backup by the
  /// Kinetics Observer
  MCWaiko(const std::string & type, double dt, bool asBackup = false);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  /**
   * @brief Updates the frames that are necessary for the state estimation.
   * @details In particular the kinematics of the anchor in the IMU frame.
   *
   * @param ctl Controller.
   * @param odomRobot
   */
  void updateNecessaryFramesOdom(const mc_control::MCController & ctl, const mc_rbdyn::Robot & odomRobot);

  /// @brief updates the pose and the velcoity of the floating base in the world frame using our estimation results
  void updatePoseAndVel();

  /// @brief Updates the real robot and/or the IMU signal using our estimation results
  /// @param ctl Controller
  void update(mc_control::MCController & ctl) override;

  /// @brief Sets the type of the odometry
  /// @param newOdometryType The new type of odometry to use.
  void setOdometryType(stateObservation::odometry::OdometryType newOdometryType);

  /// @brief Backup function that returns the estimated displacement of the floating base in the world wrt to the
  /// initial one over the backup interval.
  /// @param koBackupFbKinematics Buffer containing the pose of the floating base in the world estimated by the Kinetics
  /// Observer over the whole backup interval.
  /// @return const stateObservation::kine::Kinematics
  const stateObservation::kine::Kinematics backupFb(
      boost::circular_buffer<stateObservation::kine::Kinematics> * koBackupFbKinematics);

  /// @brief Re-estimates the current state using a delayed orientation measurement.
  /// @details Let us denote k the time on which the orientation measurement started to be computed, but is still not
  /// available. We replay the estimation at time k using the buffered state and measurements, this time using the newly
  /// available orientation measurement. We then apply the transformation between the pose at time k+1 and the current
  /// iteration.
  /// @param ctl The delayed orientation measurement.
  /// @param delayedOriMeas The delayed orientation measurement.
  /// @param delayIters Number of iterations corresponding to the measurement delay.
  /// @param delayedOriGain The gain associated to the delayed orientation within the filter.
  void delayedOriMeasurementHandler(const mc_control::MCController & ctl,
                                    const stateObservation::Matrix3 & delayedOriMeas,
                                    unsigned long delayIters,
                                    double delayedOriGain);

  inline const stateObservation::odometry::LeggedOdometryManager & odometryManager() { return odometryManager_; }

protected:
  /*! \brief update the robot pose in the world only for visualization purpose
   *
   * @param robot Robot to update
   */

  void update(mc_rbdyn::Robot & robot);

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

  /*! \brief Add logs related to delayed orientation measurements
   * @param logger
   * @param category Category in which to log this observer
   */
  void addDelayedOriMeasLogs(mc_rtc::Logger &, const std::string & category);

  /*! \brief Remove the logs related to delayed orientation measurements
   * @param logger
   */
  void removeDelayedOriMeasLogs(mc_rtc::Logger &);

public:
  // estimated kinematics of the IMU in the world
  stateObservation::kine::Kinematics estimatedWorldImuKine_;
  // estimated kinematics of the floating base in the world
  stateObservation::kine::Kinematics estimatedWorldFbKine_;

protected:
  // category to plot the estimator in
  std::string category_;

  // container for our robots
  std::shared_ptr<mc_rbdyn::Robots> my_robots_;

  std::string robot_; // name of the robot
  bool updateRobot_ = true; // indicates whether we use our estimation to update the real robot or not
  std::string imuSensor_; // IMU used for the estimation
  bool updateSensor_ = true; // indicates whether we update the IMU signal or not

  // /// gain of the gyro bias correction by the velocity
  // double finalGamma_ = 1;

  /*!
   * initial value of the parameter related to the convergence of the linear velocity
   * of the IMU expressed in the control frame
   */
  double alpha_ = 5;
  /// initial value of the parameter related to the fast convergence of the tilt
  double beta_ = 1;
  /// initial value of the parameter related to the orthogonality
  double gamma_ = 2; // gain associated with the correction of the orientation by the contact orientation
  double mu_ = 2;
  // gain associated with the correction of the position by the contact position
  double rho_ = 2;
  // gain associated with the correction of the yaw by the contact position
  double psi_ = 0.3;

  /*!
   * parameter related to the convergence of the linear velocity
   * of the IMU expressed in the control frame
   */
  double finalAlpha_ = 5;
  ///  parameter related to the fast convergence of the tilt
  double finalBeta_ = 1;
  /// parameter related to the orthogonality
  double finalGamma_ = 2;
  // gain associated with the correction of the orientation by the contact orientation
  double finalMu_ = 2;
  // gain associated with the correction of the position by the contact position
  double finalRho_ = 2;
  // gain associated with the correction of the yaw by the contact position
  double finalPsi_ = 0.3;

  // flag indicating the variables we want in the resulting Kinematics object
  stateObservation::kine::Kinematics::Flags::Byte flagPoseVels_ =
      stateObservation::kine::Kinematics::Flags::position | stateObservation::kine::Kinematics::Flags::orientation
      | stateObservation::kine::Kinematics::Flags::linVel | stateObservation::kine::Kinematics::Flags::angVel;

  // function used to compute the anchor frame of the robot in the world.
  std::string anchorFrameFunction_;
  // instance of the Tilt Estimator for humanoid robots.
  stateObservation::WaikoHumanoid estimator_;

  /* kinematics used for computation */
  // kinematics of the IMU in the floating base after the encoders update
  stateObservation::kine::Kinematics fbImuKine_;
  // kinematics of the floating base in the world after the encoders update
  stateObservation::kine::Kinematics worldFbKine_;
  // kinematics of the anchor frame in the IMU frame after the encoders update
  stateObservation::kine::Kinematics imuAnchorKine_;
  // kinematics of the IMU in the world after the encoders update
  stateObservation::kine::Kinematics worldImuKine_;

  /* Estimation results */

  /* Floating base's kinematics */
  // Eigen::Matrix3d R_0_fb_; // estimated orientation of the floating base in the world frame
  sva::PTransformd poseW_; ///< Estimated pose of the floating-base in world frame */
  sva::MotionVecd velW_; ///< Estimated velocity of the floating-base in world frame */

  // anchor frame's variables
  double maxAnchorFrameDiscontinuity_ =
      0.01; ///< Threshold (norm) above wich the anchor frame is considered to have had a discontinuity
  bool anchorFrameJumped_; /** Detects whether the anchor frame had a discontinuity */
  int iter_; // iterations ellapsed since the beginning of the  estimation. We don't compute the anchor frame
             // velocity while it is below "itersBeforeAnchorsVel_"
  int itersBeforeAnchorsVel_ = 10; // iteration from which we start to compute the velocity of the anchor frame. Avoids
                                   // initial jumps due to the finite differences.

  stateObservation::odometry::LeggedOdometryManager odometryManager_; // manager for the legged odometry
  using WaikoContact = stateObservation::odometry::LoContact;
  using ContactsDetector = measurements::ContactsDetector<stateObservation::odometry::LoContact>;
  ContactsDetector contactsDetector_;

  /* Variables for the use as a backup */
  // indicates if the estimator is used as a backup or not
  bool asBackup_ = false;
  // Buffer containing the estimated pose of the floating base in the world over the whole backup interval.
  boost::circular_buffer<stateObservation::kine::Kinematics> backupFbKinematics_;

  /* Debug variables */
  // "measured" local linear velocity of the IMU
  stateObservation::Vector3 yv_;

  stateObservation::kine::Orientation measuredOri_ = stateObservation::kine::Orientation::zeroRotation();
  stateObservation::Vector measurements_;

  stateObservation::kine::Kinematics worldImuKineFromAnchor_;
  stateObservation::kine::LocalKinematics worldImuLocKineFromAnchor_;
  stateObservation::kine::Kinematics worldAnchorKine_;

  bool withDebugLogs_ = false;
  bool withOriCorrectFromContactOri_ = true;
};

} // namespace mc_state_observation
