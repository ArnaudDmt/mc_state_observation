#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_state_observation/AttitudeObserver.h>
#include <mc_state_observation/gui_helpers.h>

namespace mc_state_observation
{

/// Sizes of the states for the state, the measurement, and the input vector
constexpr unsigned AttitudeObserver::STATE_SIZE;
constexpr unsigned AttitudeObserver::MEASUREMENT_SIZE;
constexpr unsigned AttitudeObserver::INPUT_SIZE;

namespace so = stateObservation;

AttitudeObserver::AttitudeObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt),
  filter_(STATE_SIZE, MEASUREMENT_SIZE, false, true, std::make_shared<stateObservation::IndexedInputVectorArray>()),
  q_(so::Matrix::Identity(STATE_SIZE, STATE_SIZE) * defaultConfig_.stateCov),
  r_(so::Matrix::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE) * defaultConfig_.acceleroCovariance), uk_(INPUT_SIZE),
  xk_(STATE_SIZE)
{
  /// initialization of the extended Kalman filter
  imuFunctor_.setSamplingPeriod(dt_);
  filter_.setFunctor(&imuFunctor_);
  Kpt_ << -20, 0, 0, 0, -20, 0, 0, 0, -20;
  Kdt_ << -10, 0, 0, 0, -10, 0, 0, 0, -10;
  Kpo_ << -0.0, 0, 0, 0, -0.0, 0, 0, 0, -10;
  Kdo_ << -0.0, 0, 0, 0, -0.0, 0, 0, 0, -10;
}

void AttitudeObserver::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  robot_ = config("robot", ctl.robot().name());
  imuSensor_ = config("imuSensor", ctl.robot().bodySensor().name());
  if(config.has("updateSensor")) { updateSensor_ = static_cast<std::string>(config("updateSensor")); }
  else
  {
    updateSensor_ = imuSensor_;
  }
  datastoreName_ = config("datastoreName", name());
  config("log_kf", log_kf_);
  config("init_from_control", initFromControl_);
  defaultConfig_ = config("KalmanFilter", KalmanFilterConfig{});
  config_ = defaultConfig_;
  desc_ = fmt::format("{} (sensor={})", name_, imuSensor_);
}

void AttitudeObserver::reset(const mc_control::MCController & ctl)
{
  const auto & c = config_;

  q_.noalias() = so::Matrix::Identity(STATE_SIZE, STATE_SIZE) * c.stateCov;
  r_.noalias() = so::Matrix::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE) * c.acceleroCovariance;
  q_(9, 9) = q_(10, 10) = q_(11, 11) = c.orientationAccCov;
  q_(6, 6) = q_(7, 7) = q_(8, 8) = c.linearAccCov;
  r_(3, 3) = r_(4, 4) = r_(5, 5) = c.gyroCovariance;

  filter_.reset();
  filter_.setQ(q_);
  filter_.setR(r_);
  xk_.setZero();
  if(initFromControl_)
  {
    const auto & imuSensor = ctl.robot(robot_).bodySensor(imuSensor_);
    const Eigen::Matrix3d cOri = (imuSensor.X_b_s() * ctl.robot(robot_).bodyPosW(imuSensor.parentBody())).rotation();
    xk_.segment<3>(indexes::ori) = so::kine::rotationMatrixToRotationVector(cOri.transpose());
  }

  uk_.setZero();

  if(filter_.stateIsSet()) { filter_.setState(xk_, filter_.getCurrentTime()); }
  else
  {
    filter_.setState(xk_, 0);
  }
  filter_.setStateCovariance(so::Matrix::Identity(STATE_SIZE, STATE_SIZE) * c.stateInitCov);

  lastStateInitCovariance_ = c.stateInitCov;
}

bool AttitudeObserver::run(const mc_control::MCController & ctl)
{
  const auto & c = config_;
  bool ret = true;

  q_.noalias() = so::Matrix::Identity(STATE_SIZE, STATE_SIZE) * c.stateCov;
  r_.noalias() = so::Matrix::Identity(MEASUREMENT_SIZE, MEASUREMENT_SIZE) * c.acceleroCovariance;
  q_(9, 9) = q_(10, 10) = q_(11, 11) = c.orientationAccCov;
  q_(6, 6) = q_(7, 7) = q_(8, 8) = c.linearAccCov;
  r_(3, 3) = r_(4, 4) = r_(5, 5) = c.gyroCovariance;

  filter_.setQ(q_);
  filter_.setR(r_);

  if(lastStateInitCovariance_ != c.stateInitCov) /// if the value of the state Init Covariance has changed
  {
    filter_.setStateCovariance(so::Matrix::Identity(STATE_SIZE, STATE_SIZE) * c.stateInitCov);
    lastStateInitCovariance_ = c.stateInitCov;
  }

  // Get sensor values
  const mc_rbdyn::BodySensor & imu = ctl.robot(robot_).bodySensor(imuSensor_);
  const Eigen::Vector3d & accIn = imu.linearAcceleration();
  const Eigen::Vector3d & rateIn = imu.angularVelocity();

  // processing
  so::Vector6 measurement;
  if(c.compensateMode)
  {
    if(ctl.datastore().has(datastoreName_ + "::accRef"))
    {
      const Eigen::Vector3d & accRef = ctl.datastore().get<Eigen::Vector3d>(datastoreName_ + "::accRef");
      measurement.head<3>() = accIn - accRef;
    }
    else
    {
      measurement.head<3>() = accIn;
      ret = false;
    }
  }
  else
  {
    measurement.head<3>() = accIn;
  }
  measurement.tail<3>() = rateIn;

  auto time = filter_.getCurrentTime();

  /// damped linear and angular spring
  uk_.head<3>() = Kpt_ * xk_.segment<3>(indexes::pos) + Kdt_ * xk_.segment<3>(indexes::linVel);
  uk_.tail<3>() = Kpo_ * xk_.segment<3>(indexes::ori) + Kdo_ * xk_.segment<3>(indexes::angVel);

  filter_.setInput(uk_, time);
  filter_.setMeasurement(measurement, time + 1);

  /// set the derivation step for the finite difference method
  const so::Vector dx = filter_.stateVectorConstant(1) * 1e-8;

  filter_.setA(filter_.getAMatrixFD(dx));
  filter_.setC(filter_.getCMatrixFD(dx));

  /// get the estimation and give it to the array
  xk_ = filter_.getEstimatedState(time + 1);

  // result
  const so::Vector3 orientation(xk_.segment<3>(indexes::ori));
  m_orientation = c.offset * so::kine::rotationVectorToRotationMatrix(orientation);

  return ret;
}

void AttitudeObserver::update(mc_control::MCController & ctl)
{
  auto & robot = ctl.robot(robot_);
  auto & data = *robot.data();
  auto & sensor = data.bodySensors.at(data.bodySensorsIndex.at(updateSensor_));
  Eigen::Quaterniond quat(m_orientation.transpose());
  sensor.orientation(quat);
}

void AttitudeObserver::addToLogger(const mc_control::MCController &,
                                   mc_rtc::Logger & logger,
                                   const std::string & category)
{
  logger.addLogEntry(category + "_orientation", [this]() -> sva::PTransformd
                     { return sva::PTransformd{m_orientation.transpose(), Eigen::Vector3d::Zero()}; });
  if(log_kf_) { config_.addToLogger(logger, category); }
}

void AttitudeObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_orientation");
  if(log_kf_) { config_.removeFromLogger(logger, category); }
}

void AttitudeObserver::addToGUI(const mc_control::MCController & ctl,
                                mc_rtc::gui::StateBuilder & gui,
                                const std::vector<std::string> & category)
{
  auto kf_category = category;
  kf_category.push_back("KalmanFilter");
  config_.addToGUI(gui, kf_category);

  gui.addElement(category, mc_rtc::gui::Button("Reset config to default", [this]() { config_ = defaultConfig_; }),
                 mc_rtc::gui::Button("Reset",
                                     [this, &ctl]()
                                     {
                                       mc_rtc::log::info("[{}] Manual reset triggerred", name());
                                       reset(ctl);
                                     }),
                 mc_rtc::gui::ArrayLabel(
                     "Result", {"r [deg]", "p [deg]", "y [deg]"}, [this]() -> Eigen::Vector3d
                     { return mc_rbdyn::rpyFromMat(m_orientation.transpose()) * 180. / mc_rtc::constants::PI; }));
}

void AttitudeObserver::KalmanFilterConfig::addToLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.addLogEntry(category + "_covariance_state", [this]() { return stateCov; });
  logger.addLogEntry(category + "_covariance_ori_acc", [this]() { return orientationAccCov; });
  logger.addLogEntry(category + "_covariance_acc", [this]() { return acceleroCovariance; });
  logger.addLogEntry(category + "_covariance_gyr", [this]() { return gyroCovariance; });
}

void AttitudeObserver::KalmanFilterConfig::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_covariance_state");
  logger.removeLogEntry(category + "_covariance_ori_acc");
  logger.removeLogEntry(category + "_covariance_acc");
  logger.removeLogEntry(category + "_covariance_gyr");
}

void AttitudeObserver::KalmanFilterConfig::addToGUI(mc_rtc::gui::StateBuilder & gui,
                                                    const std::vector<std::string> & category)
{
  // clang-format off
  gui.addElement(category,
    mc_state_observation::gui::make_input_element("Compensate Mode", compensateMode),
    mc_state_observation::gui::make_input_element("acceleroCovariance", acceleroCovariance),
    mc_state_observation::gui::make_input_element("gyroCovariance", gyroCovariance),
    mc_state_observation::gui::make_input_element("orientationAccCov", orientationAccCov),
    mc_state_observation::gui::make_input_element("linearAccCov", linearAccCov),
    mc_state_observation::gui::make_input_element("stateCov", stateCov),
    mc_state_observation::gui::make_input_element("stateInitCov", stateInitCov),
    mc_state_observation::gui::make_rpy_input("offset", offset));
  // clang-format on
}

void AttitudeObserver::KalmanFilterConfig::removeFromGUI(mc_rtc::gui::StateBuilder & gui,
                                                         const std::vector<std::string> & category)
{ gui.removeCategory(category); }

} // namespace mc_state_observation
EXPORT_OBSERVER_MODULE("Attitude", mc_state_observation::AttitudeObserver)
