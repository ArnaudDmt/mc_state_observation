#pragma once

#include <boost/circular_buffer.hpp>

#include <forward_list>
#include <mc_state_observation/odometry/LeggedOdometryManager.h>
#include <state-observation/observer/vanyt-estimator.hpp>

namespace mc_state_observation
{

struct MCVanytEstimator : public mc_observers::Observer
{
  /// @brief Structure containing information about delayed orientation measurements.
  struct DelayedOriMeasurement
  {
    stateObservation::Matrix3 meas_;
    double gain_;
    stateObservation::kine::Kinematics updatedPoseWithoutMeas_;
    stateObservation::kine::Kinematics updatedPoseWithMeas_;
  };

  // we define MCKineticsObserver as a friend as it can instantiate this observer as a backup
  friend struct MCKineticsObserver;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  /// @brief Constructor for the MCVanytEstimator.
  /// @details The parameters asBackup and observerName are given only if the Tilt Observer is used as a backup by the
  /// Kinetics Observer
  MCVanytEstimator(const std::string & type,
                   double dt,
                   bool asBackup = false,
                   const std::string & observerName = "MCVanytEstimator");

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
  /// @param localWorldImuLinVel estimated local linear velocity of the IMU in the world frame
  /// @param localWorldImuAngVelestimated measurement of the gyrometer
  void updatePoseAndVel(const stateObservation::Vector3 & localWorldImuLinVel,
                        const stateObservation::Vector3 & localWorldImuAngVel);

  /*! \brief update the robot pose in the world only for visualization purpose
   *
   * @param odomRobot Robot with the kinematics of the control robot but with updated joint values.
   */
  void runTiltEstimator(const mc_control::MCController & ctl, const mc_rbdyn::Robot & odomRobot);

  /// @brief Updates the real robot and/or the IMU signal using our estimation results
  /// @param ctl Controller
  void update(mc_control::MCController & ctl) override;

  /// @brief Sets the type of the odometry
  /// @param newOdometryType The new type of odometry to use.
  void setOdometryType(measurements::OdometryType newOdometryType);

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

  inline const odometry::LeggedOdometryManager & odometryManager() { return odometryManager_; }

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
  stateObservation::kine::Kinematics correctedWorldImuKine_;

protected:
  // name of the observer
  std::string observerName_;

  // container for our robots
  std::shared_ptr<mc_rbdyn::Robots> my_robots_;

  std::string robot_; // name of the robot
  bool updateRobot_ = true; // indicates whether we use our estimation to update the real robot or not
  std::string imuSensor_; // IMU used for the estimation
  bool updateSensor_ = true; // indicates whether we update the IMU signal or not

  /*!
   * parameter related to the convergence of the linear velocity
   * of the IMU expressed in the control frame
   */
  double finalAlpha_ = 5;
  ///  parameter related to the fast convergence of the tilt
  double finalBeta_ = 1;
  /// parameter related to the orthogonality
  double finalRho_ = 2;

  /*!
   * initial value of the parameter related to the convergence of the linear velocity
   * of the IMU expressed in the control frame
   */
  double alpha_ = 5;
  /// initial value of the parameter related to the fast convergence of the tilt
  double beta_ = 1;
  /// initial value of the parameter related to the orthogonality
  double rho_ = 2;

  // flag indicating the variables we want in the resulting Kinematics object
  stateObservation::kine::Kinematics::Flags::Byte flagPoseVels_ =
      stateObservation::kine::Kinematics::Flags::position | stateObservation::kine::Kinematics::Flags::orientation
      | stateObservation::kine::Kinematics::Flags::linVel | stateObservation::kine::Kinematics::Flags::angVel;

  // function used to compute the anchor frame of the robot in the world.
  std::string anchorFrameFunction_;
  // instance of the Tilt Estimator for humanoid robots.
  stateObservation::VanytEstimator estimator_;

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

  // The observed tilt of the sensor
  Eigen::Matrix3d estimatedRotationIMU_;
  /// State vector estimated by the Tilt Observer
  stateObservation::Vector xk_;
  // estimated kinematics of the floating base in the world
  stateObservation::kine::Kinematics correctedWorldFbKine_;

  /* Floating base's kinematics */
  Eigen::Matrix3d R_0_fb_; // estimated orientation of the floating base in the world frame
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

  /* Odometry parameters */
  odometry::LeggedOdometryManager odometryManager_; // manager for the legged odometry

  double contactDetectionThreshold_; // threshold used for the contacts detection

  /* Variables for the use as a backup */
  // indicates if the estimator is used as a backup or not
  bool asBackup_ = false;
  // Buffer containing the estimated pose of the floating base in the world over the whole backup interval.
  boost::circular_buffer<stateObservation::kine::Kinematics> backupFbKinematics_;

  /* Debug variables */
  // "measured" local linear velocity of the IMU
  stateObservation::Vector3 yv_;
  // velocity of the IMU in the anchor frame
  sva::MotionVecd imuVelC_;
  // pose of the IMU in the anchor frame
  sva::PTransformd X_C_IMU_;

  stateObservation::kine::Orientation measuredOri_ = stateObservation::kine::Orientation::zeroRotation();
  stateObservation::Vector measurements_;

  double mu_contacts_ = 2;
  double mu_gyroscope_ = 2;
  double lambda_contacts_ = 2;
  double gamma_contacts_ = 1;

  // delayed IMU orientation measurement
  DelayedOriMeasurement delayedOriMeas_;
};

} // namespace mc_state_observation
