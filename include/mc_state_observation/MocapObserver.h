#pragma once

#include <mc_control/MCController.h>
#include <mc_observers/Observer.h>
#include <mc_rbdyn/Robot.h>

namespace mc_state_observation
{

struct MocapObserver : public mc_observers::Observer
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  MocapObserver(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

  /** Set the marker frame pose, expressed in mocap origin frame */
  void markerPose(const sva::PTransformd & pose)
  {
    X_m_marker_ = pose;
    gotMarker_ = true;
  }

  /** Pose of the mocap marker frame expressed in the mocap origin frame */
  const sva::PTransformd & markerPose() const noexcept { return X_m_marker_; }

  mc_rbdyn::Robot & robot(mc_control::MCController & ctl)
  { return useReal_ ? ctl.realRobot(robot_) : ctl.robot(robot_); }

  const mc_rbdyn::Robot & robot(const mc_control::MCController & ctl) const
  { return useReal_ ? ctl.realRobot(robot_) : ctl.robot(robot_); }

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

  bool checkPipelines(const mc_control::MCController &);
  bool calibrateMarkerToBody(const mc_control::MCController &);
  bool initializeOrigin(const mc_control::MCController &);

protected:
  /// @{
  std::string robot_ = ""; ///< Name of robot to which the MOCAP marker is attached
  bool useReal_ = false;
  std::string updateRobot_ = ""; ///< Name of the robot to update
  std::string body_ = ""; ///< Body to which the marker is attached
  /// @}

  /// @{
  bool calibrated_ = false;
  sva::PTransformd X_marker_body_ =
      sva::PTransformd::Identity(); ///< Extrinsic calibration from body to marker frame (auto-determined)
  bool originInitialized_ = false;
  sva::PTransformd X_0_mocap_ =
      sva::PTransformd::Identity(); ///< Transformation from robot world to mocap world (auto-determined)
  /// @}

  // {@
  sva::PTransformd X_m_marker_ = sva::PTransformd::Identity();
  bool gotMarker_ = false;
  sva::PTransformd X_0_marker_ = sva::PTransformd::Identity(); // Estimated pose of the marker frame
  sva::PTransformd X_0_fb_ = sva::PTransformd::Identity(); // Estimated pose of the floating base
  // @}
};
} // namespace mc_state_observation
