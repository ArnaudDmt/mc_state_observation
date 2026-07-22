#pragma once

#include <mc_observers/Observer.h>

#include <state-observation/dynamical-system/imu-dynamical-system.hpp>
#include <state-observation/observer/extended-kalman-filter.hpp>

namespace mc_state_observation
{

struct AttitudeObserver : public mc_observers::Observer
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  using indexes = stateObservation::kine::indexes<stateObservation::kine::rotationVector>;

public:
  AttitudeObserver(const std::string & type, double dt);

  void configure(const mc_control::MCController & ctl, const mc_rtc::Configuration &) override;

  void reset(const mc_control::MCController & ctl) override;

  bool run(const mc_control::MCController & ctl) override;

  void update(mc_control::MCController & ctl) override;

public:
  struct KalmanFilterConfig
  {
    bool compensateMode = false;
    double acceleroCovariance = 0.003;
    double gyroCovariance = 1e-10;
    double orientationAccCov = 0.003;
    double linearAccCov = 1e-13;
    double stateCov = 3e-14;
    double stateInitCov = 1e-8;
    Eigen::Matrix3d offset = Eigen::Matrix3d::Identity(); ///< Offset to apply to the estimation result
    void addToLogger(mc_rtc::Logger & logger, const std::string & category);
    void removeFromLogger(mc_rtc::Logger & logger, const std::string & category);
    void addToGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category);
    void removeFromGUI(mc_rtc::gui::StateBuilder & gui, const std::vector<std::string> & category);
  };

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
  /// @{
  std::string robot_ = ""; ///< Name of robot to which the IMU sensor belongs
  std::string imuSensor_ = ""; ///< Name of the sensor used for IMU readings
  std::string updateSensor_ = ""; ///< Name of the sensor to update with the results (default: imuSensor_)
  std::string datastoreName_ = ""; ///< Name on the datastore (default name())
  KalmanFilterConfig defaultConfig_; ///< Default configuration for the KF (as set by configure())
  KalmanFilterConfig config_; ///< Current configuration for the KF (GUI, etc...)
  bool log_kf_ = false; ///< Whether to log the parameters of the kalman filter
  bool initFromControl_ = true; ///< Whether to initialize from the control state
  /// @}

  /// Sizes of the states for the state, the measurement, and the input vector
  static constexpr unsigned STATE_SIZE = 18;
  static constexpr unsigned MEASUREMENT_SIZE = 6;
  static constexpr unsigned INPUT_SIZE = 6;

  double lastStateInitCovariance_;

  /// initialization of the extended Kalman filter
  stateObservation::ExtendedKalmanFilter filter_;

  /// initalization of the functor
  stateObservation::IMUDynamicalSystem imuFunctor_;

  stateObservation::Matrix q_;
  stateObservation::Matrix r_;

  stateObservation::VectorInput uk_;
  stateObservation::Vector xk_;

  stateObservation::Matrix3 Kpt_, Kdt_;
  stateObservation::Matrix3 Kpo_, Kdo_;

  Eigen::Matrix3d m_orientation = Eigen::Matrix3d::Identity(); ///< Result
};

} // namespace mc_state_observation

namespace mc_rtc
{
template<>
struct ConfigurationLoader<mc_state_observation::AttitudeObserver::KalmanFilterConfig>
{
  static mc_state_observation::AttitudeObserver::KalmanFilterConfig load(const mc_rtc::Configuration & config)
  {
    mc_state_observation::AttitudeObserver::KalmanFilterConfig c;
    config("compensateMode", c.compensateMode);
    config("offset", c.offset);
    config("acc_cov", c.acceleroCovariance);
    config("gyr_cov", c.gyroCovariance);
    config("ori_acc_cov", c.orientationAccCov);
    config("lin_acc_cov", c.linearAccCov);
    config("state_cov", c.stateCov);
    config("state_init_cov", c.stateInitCov);
    return c;
  }
  static mc_rtc::Configuration save(const mc_state_observation::AttitudeObserver::KalmanFilterConfig & c)
  {
    mc_rtc::Configuration config;
    config.add("compensateMode", c.compensateMode);
    config.add("offset", c.offset);
    config.add("acc_cov", c.acceleroCovariance);
    config.add("gyr_cov", c.gyroCovariance);
    config.add("ori_acc_cov", c.orientationAccCov);
    config.add("lin_acc_cov", c.linearAccCov);
    config.add("state_cov", c.stateCov);
    config.add("state_init_cov", c.stateInitCov);
    return config;
  }
};
} // namespace mc_rtc
