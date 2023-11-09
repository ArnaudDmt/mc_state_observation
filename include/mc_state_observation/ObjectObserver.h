#pragma once

#include <mc_state_observation/filtering.h>
#include <mc_state_observation/ros.h>

#include <mc_observers/Observer.h>

#include <mutex>
#include <thread>

namespace mc_state_observation
{

struct ObjectObserver : public mc_observers::Observer
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ObjectObserver(const std::string & type, double dt);

  ~ObjectObserver();

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

protected:
  /*! \brief Callback for ros topic_ defined in the configuration file
   *
   * @param msg Message sent by the vision process containing estimated object information
   */
  void callback(const PoseStamped & msg);

  /// @{
  std::string robot_ = ""; ///< Name of robot to estimate thanks to SLAM
  std::string camera_ = ""; ///< Name of robot's camera body
  std::shared_ptr<mc_rbdyn::Robots> robots_; ///< Store robot estimated state if SLAM Observer is used
  /// @}

  /// @{
  std::string object_ = ""; ///< Name of map TF in ROS
  std::string topic_ = ""; ///< Name of estimated camera TF in ROS
  PoseSubscriber subscriber_; ///< Subscribe to topic_ name
  std::mutex mutex_; ///< Mutex to ensure thread safe with ROS callback
  sva::PTransformd X_Camera_EstimatedObject_ = sva::PTransformd::Identity(); ///< Estimated object in camera frame
  bool isInRobotMap_ = false; ///< If true then we have X_0_EstimatedObject_ instead of X_Camera_EstimatedObject_
  bool isNewEstimatedPose_ = false; ///< Check if there is a new update received from ROS
  /// @}

  /// @{
  bool isPublished_ = true; ///< Check if estimated robot is publish or not
  /// @}

  mc_rtc::NodeHandlePtr nh_ = nullptr;
  void rosSpinner();
  std::thread thread_;
  bool thread_run_ = true;

  bool isEstimatedPoseValid_ = false;

  bool isNotFirstTimeInCallback_ = false;
};
} // namespace mc_state_observation
