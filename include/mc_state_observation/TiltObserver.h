#pragma once

#include <mc_observers/Observer.h>
#include <boost/circular_buffer.hpp>
#include <mc_state_observation/observersTools/leggedOdometryTools.h>
#include <state-observation/observer/tilt-estimator-humanoid.hpp>
#include <state-observation/tools/rigid-body-kinematics.hpp>

namespace mc_state_observation
{

struct TiltObserver : public mc_observers::Observer
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
  TiltObserver(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  /// @brief updates the kinematics of the anchor frame of the robot in the world
  /// @param ctl Controller
  /// @param updatedRobot robot corresponding to the control robot with updated encoders
  void updateAnchorFrame(const mc_control::MCController & ctl, const mc_rbdyn::Robot & updatedRobot);

  /// @brief updates the kinematics of the anchor frame of our odometry robot in the world
  /// @param ctl Controller
  void updateAnchorFrameOdometry(const mc_control::MCController & ctl);
  /// @brief updates the kinematics of the anchor frame of the robot when not performing odometry
  /// @param ctl Controller
  /// @param updatedRobot robot corresponding to the control robot with updated encoders
  void updateAnchorFrameNoOdometry(const mc_control::MCController & ctl, const mc_rbdyn::Robot & updatedRobot);

  /// @brief updates the pose and the velcoity of the floating base in the world frame using our estimation results
  /// @param localWorldImuLinVel estimated local linear velocity of the IMU in the world frame
  /// @param localWorldImuAngVelestimated measurement of the gyrometer
  void updatePoseAndVel(const stateObservation::Vector3 & localWorldImuLinVel,
                        const stateObservation::Vector3 & localWorldImuAngVel);

  /*! \brief update the robot pose in the world only for visualization purpose
   *
   * @param updatedRobot Robot with the kinematics of the control robot but with updated joint values.
   */
  void runTiltEstimator(const mc_control::MCController & ctl, const mc_rbdyn::Robot & updatedRobot);

  /// @brief Updates the real robot and/or the IMU signal using our estimation results
  /// @param ctl Controller
  void update(mc_control::MCController & ctl) override;

  /// @brief Backup function that returns the estimated displacement of the floating base in the world wrt to the
  /// initial one over the backup interval.
  /// @param ctl Controller
  /// @return const stateObservation::kine::Kinematics
  const stateObservation::kine::Kinematics backupFb(const mc_control::MCController & ctl);

  /// @brief Computes the pose transformation estimated by the Tilt Observer between the last two iterations and
  /// applies it to the given kinematics.
  /// @details Also fills the velocity with the velocity estimated by the Tilt Observer (expressed in the new frame)
  /// @param kine The kinematics on which to apply the transformation
  /// @return stateObservation::kine::Kinematics
  stateObservation::kine::Kinematics applyLastTransformation(const stateObservation::kine::Kinematics & kine);

  /// @brief checks that the odometry type used for the Kinetics Observer and the Tilt Observer match for the backup
  /// @param koOdometryType type of odometry used by the Kinetics Observer
  void checkCorrectBackupConf(measurements::OdometryType & koOdometryType);

  /// @brief Changes the type of the odometry
  /// @param newOdometryType The new type of odometry to use.
  void changeOdometryType(const std::string & newOdometryType);

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

protected:
  // name of the observer
  std::string observerName_ = "TiltObserver";

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
  double finalAlpha_ = 50;
  ///  parameter related to the fast convergence of the tilt
  double finalBeta_ = 5;
  /// parameter related to the orthogonality
  double finalGamma_ = 15;

  /*!
   * initial value of the parameter related to the convergence of the linear velocity
   * of the IMU expressed in the control frame
   */
  double alpha_ = 50;
  /// initial value of the parameter related to the fast convergence of the tilt
  double beta_ = 5;
  /// initial value of the parameter related to the orthogonality
  double gamma_ = 15;

  // flag indicating the variables we want in the resulting Kinematics object
  stateObservation::kine::Kinematics::Flags::Byte flagPoseVels_ =
      stateObservation::kine::Kinematics::Flags::position | stateObservation::kine::Kinematics::Flags::orientation
      | stateObservation::kine::Kinematics::Flags::linVel | stateObservation::kine::Kinematics::Flags::angVel;

  // function used to compute the anchor frame of the robot in the world.
  std::string anchorFrameFunction_;
  // instance of the Tilt Estimator for humanoid robots.
  stateObservation::TiltEstimatorHumanoid estimator_;

  /* kinematics used for computation */
  // kinematics of the IMU in the floating base after the encoders update
  stateObservation::kine::Kinematics updatedFbImuKine_;
  // kinematics of the anchor frame of the control robot in the world. Version as a PTransform object.
  sva::PTransformd X_0_C_;
  // kinematics of the anchor frame of the control robot updated with the encoders in the world.  Version as a
  // PTransform object.
  sva::PTransformd X_0_C_updated_;
  // previous value of X_0_C_updated_, used to compute the velocity by finite differences
  sva::PTransformd X_0_C_updated_previous_;

  // kinematics of the anchor frame of the control robot in the world. Version as a Kinematics object.
  stateObservation::kine::Kinematics worldAnchorKine_;
  // kinematics of the anchor frame of the control robot updated with the encoders in the world. Version as a Kinematics
  // object.
  stateObservation::kine::Kinematics updatedWorldAnchorKine_;

  // kinematics of the floating base in the world after the encoders update
  stateObservation::kine::Kinematics updatedWorldFbKine_;
  // kinematics of the anchor frame in the IMU frame after the encoders update
  stateObservation::kine::Kinematics updatedImuAnchorKine_;
  // kinematics of the anchor frame in the world frame for the new iteration
  stateObservation::kine::Kinematics newWorldAnchorKine_;
  // kinematics of the anchor frame in the world frame for the new iteration, after the encoders update
  stateObservation::kine::Kinematics newUpdatedWorldAnchorKine_;
  // kinematics of the IMU in the world for the control robot
  stateObservation::kine::Kinematics worldImuKine_;
  // kinematics of the IMU in the world after the encoders update
  stateObservation::kine::Kinematics updatedWorldImuKine_;

  /* Estimation results */

  // The observed tilt of the sensor
  Eigen::Matrix3d estimatedRotationIMU_;
  /// State vector estimated by the Tilt Observer
  stateObservation::Vector xk_;
  // estimated kinematics of the floating base in the world
  stateObservation::kine::Kinematics correctedWorldFbKine_;
  // estimated kinematics of the IMU in the world
  stateObservation::kine::Kinematics correctedWorldImuKine_;

  /* Floating base's kinematics */
  Eigen::Matrix3d R_0_fb_; // estimated orientation of the floating base in the world frame
  sva::PTransformd poseW_; ///< Estimated pose of the floating-base in world frame */
  sva::PTransformd prevPoseW_; ///< Estimated pose of the floating-base in world frame */
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
  using OdometryType = measurements::OdometryType;

  leggedOdometry::LeggedOdometryManager odometryManager_; // manager for the legged odometry
  using LoContactsManager = leggedOdometry::LeggedOdometryManager::ContactsManager;
  double contactDetectionThreshold_; // threshold used for the contacts detection

  /* Backup function's parameters */

  bool asBackup_ = false; // indicates if the estimator is used as a backup or not
  // Buffer containing the estimated pose of the floating base in the world over the whole backup interval.
  boost::circular_buffer<sva::PTransformd> backupFbKinematics_ = boost::circular_buffer<sva::PTransformd>(100);

  /* Debug variables */
  // "measured" local linear velocity of the IMU
  stateObservation::Vector3 x1_;
  // velocity of the IMU in the anchor frame
  sva::MotionVecd imuVelC_;
  // pose of the IMU in the anchor frame
  sva::PTransformd X_C_IMU_;
};

} // namespace mc_state_observation
