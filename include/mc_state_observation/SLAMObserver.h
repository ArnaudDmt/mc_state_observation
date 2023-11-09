#pragma once

#include <mc_state_observation/filtering.h>
#include <mc_state_observation/ros.h>

#include <mc_observers/Observer.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <thread>

namespace mc_state_observation
{

struct SLAMObserver : public mc_observers::Observer
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SLAMObserver(const std::string & type, double dt);

  ~SLAMObserver();

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

protected:
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

  /** Add plots to the GUI */
  void addPlots(mc_rtc::gui::StateBuilder & gui);

  /** Remove plots from the GUI */
  void removePlots(mc_rtc::gui::StateBuilder & gui);

  /** Toggle plots on/off */
  void togglePlots(mc_rtc::gui::StateBuilder & gui);

protected:
  /// @{
  std::string robot_ = ""; ///< Name of robot to estimate thanks to SLAM
  std::string camera_ = ""; ///< Name of robot's camera body
  std::string body_ = ""; ///< Name of robot's freeflyer
  std::shared_ptr<mc_rbdyn::Robots> robots_; ///< Store robot estimated state
  /// @}

  /// @{
  std::string map_ = ""; ///< Name of map TF in ROS
  std::string estimated_ = ""; ///< Name of estimated camera TF in ROS
  bool isInitialized_ = false; ///< Check if the observer is initialized or not
  sva::PTransformd X_Slam_Estimated_Camera_ =
      sva::PTransformd::Identity(); ///< Transformation to go from SLAM world to estimated camera
  sva::PTransformd X_0_Slam_ = sva::PTransformd::Identity(); ///< Transformation to go from Robot world to SLAM world
  sva::PTransformd X_0_Estimated_camera_ =
      sva::PTransformd::Identity(); ///< Camera pose in Robot world estimated by SLAM
  bool isSLAMAlive_ = false; ///< Check if slam is alive or not
  /// @}

  /// @{
  std::string ground_ = "";
  sva::PTransformd X_Slam_Ground_ = sva::PTransformd::Identity(); ///< Ground frame in map frame
  /// @}

  mc_rtc::NodeHandlePtr nh_ = nullptr;
  void rosSpinner();
  std::thread thread_;
  bool thread_run_ = true;
  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_{tfBuffer_};
  tf2_ros::TransformBroadcaster tfBroadcaster_;

  /// @{
  bool isFiltered_ = false; ///< Check if a filter is apply or not
  std::unique_ptr<filter::Transform> filter_; ///< Filter based on savitzky-golay
  sva::PTransformd X_0_Filtered_estimated_camera_ =
      sva::PTransformd::Identity(); ///< Estimated camera pose in robot_map
  /// @}

  /// @{
  bool isPublished_ = true; ///< Check if estimated robot is publish or not
  /// @}

  /// @{
  bool isSimulated_ = false;
  bool isUsingNoise_ = false;
  Eigen::Vector3d minOrientationNoise_ = Eigen::Vector3d(-0.05, -0.05, -0.05);
  Eigen::Vector3d maxOrientationNoise_ = Eigen::Vector3d(0.05, 0.05, 0.05);
  Eigen::Vector3d minTranslationNoise_ = Eigen::Vector3d(-0.01, -0.01, -0.01);
  Eigen::Vector3d maxTranslationNoise_ = Eigen::Vector3d(0.01, 0.01, 0.01);
  /// @}

  /// @{
  bool plotsEnabled_ = false; ///< Are GUI plots enabled
  /// @}

  double t_ = 0.0;
};
} // namespace mc_state_observation
