#pragma once

#include <mc_rtc/logging.h>
#include <mc_state_observation/measurements/IMU.h>

#include <vector>

namespace mc_state_observation::measurements
{

// allowed odometry types
enum class OdometryType
{
  Odometry6d,
  Flat,
  None
};
namespace internal
{
// map allowing to get the OdometryType value associated to the given string
inline static const std::unordered_map<std::string, OdometryType> strToOdometryType_ = {
    {"6D", OdometryType::Odometry6d},
    {"Flat", OdometryType::Flat},
    {"None", OdometryType::None}};
// map allowing to get the string value associated to the given OdometryType object
inline static const std::unordered_map<OdometryType, std::string> odometryTypToStr_ = {{OdometryType::Odometry6d, "6D"},
                                                                                       {OdometryType::Flat, "Flat"},
                                                                                       {OdometryType::None, "None"}};
} // namespace internal

/// @brief Returns an OdometryType object corresponding to the given string
/// @details Allows to set the odometry type directly from a string, most likely obtained from a configuration file.
/// This version checks beforehand that the given method name is valid.
/// @param str The string naming the desired odometry
/// @param observerName The name of the observer
/// @return OdometryType
inline static OdometryType stringToOdometryType(const std::string & str, const std::string & observerName)
{
  auto it = internal::strToOdometryType_.find(str);
  if(it != internal::strToOdometryType_.end()) { return it->second; }
  mc_rtc::log::error_and_throw<std::runtime_error>("[{}]: No known OdometryType value for {}", observerName, str);
}

/// @brief Returns an OdometryType object corresponding to the given string
/// @details Allows to set the odometry type directly from a string. This version performs no check, and thus assumes
/// the given string is valid, for example coming from a ComboInput.
/// @param str The string naming the desired odometry
/// @return OdometryType
inline static OdometryType stringToOdometryType(const std::string & str)
{ return internal::strToOdometryType_.at(str); }

/// @brief Returns the string value associated to the given OdometryType object
/// @details This can be used to display the name of the method in the gui for example. This function assumes the given
/// type is valid.
/// @param odometryType The current odometry type
/// @return std::string
inline static std::string odometryTypeToSstring(OdometryType odometryType)
{ return internal::odometryTypToStr_.at(odometryType); }

// IMUs can be handled using only a vector containing the IMU objects.
typedef std::vector<IMU> ImuList;

} // namespace mc_state_observation::measurements
