#include <mc_rbdyn/rpy_utils.h>
#include <mc_rtc/constants.h>
#include <mc_rtc/gui.h>
#include <mc_rtc/type_name.h>

namespace mc_state_observation
{
namespace gui
{

inline auto make_checkbox(std::string name, bool & variable)
{
  auto GetF = [&variable]() { return variable; };
  auto SetF = [&variable]() { variable = !variable; };
  return mc_rtc::gui::Checkbox(name, GetF, SetF);
}

template<typename T>
auto make_number_input(std::string name, T & value)
{
  auto GetF = [&value]() { return value; };
  auto SetF = [&value](const T & newValue) { value = newValue; };
  return mc_rtc::gui::NumberInput(name, GetF, SetF);
}

inline auto make_rpy_input(std::string name, Eigen::Matrix3d & orientation)
{
  auto GetF = [&orientation]() -> Eigen::Vector3d
  { return mc_rbdyn::rpyFromMat(orientation) * 180. / mc_rtc::constants::PI; };
  auto SetF = [&orientation](const Eigen::Vector3d & rpy)
  { orientation = mc_rbdyn::rpyToMat(rpy * mc_rtc::constants::PI / 180); };
  return mc_rtc::gui::ArrayInput(name, {"r [deg]", "p [deg]", "y [deg]"}, GetF, SetF);
}

inline auto make_motionvecd_input(std::string name, sva::MotionVecd & vel)
{
  auto GetF = [&vel]() -> const sva::MotionVecd & { return vel; };
  auto SetF = [&vel](const sva::MotionVecd & newVel) { vel = newVel; };
  return mc_rtc::gui::ArrayInput(name, {"wx [rad/s]", "wy [rad/s]", "wz [rad/s]", "vx [m/s]", "vy [m/s]", "vz [m/s]"},
                                 GetF, SetF);
}

inline auto make_admittancevecd_input(std::string name, sva::AdmittanceVecd & vel)
{
  auto GetF = [&vel]() -> Eigen::Vector6d { return vel.vector(); };
  auto SetF = [&vel](const Eigen::Vector6d & newVel) { vel = newVel; };
  return mc_rtc::gui::ArrayInput(name, {"wx", "wy", "wz", "vx", "vy", "vz"}, GetF, SetF);
}

inline auto make_rpy_label(std::string name, const Eigen::Matrix3d & orientation)
{
  auto GetF = [&orientation]() -> Eigen::Vector3d
  { return mc_rbdyn::rpyFromMat(orientation) * 180. / mc_rtc::constants::PI; };
  return mc_rtc::gui::ArrayLabel<decltype(GetF)>(name, {"r [deg]", "p [deg]", "y [deg]"}, GetF);
}

inline auto make_label(std::string name, std::string && label)
{
  auto GetF = [label]() { return label; };
  return mc_rtc::gui::Label<decltype(GetF)>(name, GetF);
}

inline auto make_label(std::string name, const std::string & label)
{
  auto GetF = [&label]() { return label; };
  return mc_rtc::gui::Label<decltype(GetF)>(name, GetF);
}

inline auto make_input_element(const std::string & name, sva::MotionVecd & ref)
{ return make_motionvecd_input(name, ref); }

inline auto make_input_element(const std::string & name, sva::AdmittanceVecd & ref)
{ return make_admittancevecd_input(name, ref); }

inline auto make_input_element(const std::string & name, bool & ref)
{ return make_checkbox(name, ref); }

template<typename T, typename std::enable_if<std::is_arithmetic<T>::value, int>::type = 0>
auto make_input_element(const std::string & name, T & ref)
{ return make_number_input(name, ref); }

} // namespace gui

} // namespace mc_state_observation
