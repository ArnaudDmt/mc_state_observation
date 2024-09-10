#include <mc_rtc/log/Logger.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <state-observation/tools/rigid-body-kinematics.hpp>

/**
 * Conversion functions  between the SVA representation of kinematics (PTransform for pose, MotionVec for velocities and
 * accelerations) and the one used in rigid-body-kinematics (Kinematics).
 **/

namespace mc_state_observation::conversions::kinematics
{

/// @brief Creates a Kinematics object from a PTransformd object that contains the position and the orientation of a
/// frame within another.
/// @param pTransform The pose of the frame within the other frame, stored as a sva PTransform object.
/// @param zeroKine Defines the kinematic variables to initialize to zero
stateObservation::kine::Kinematics fromSva(const sva::PTransformd & pTransform,
                                           stateObservation::kine::Kinematics::Flags::Byte zeroKine);

/// @brief Creates a Kinematics object from a PTransformd object that contains the position and the orientation of a
/// frame A within another frame B, and from a MotionVecd object that contains the associated velocities.
/// @details The motion vectors given by mc_rtc can be expressed in the global frame B (example:
/// rbd::MultiBodyConfig::bodyVelW), but some are expressed in the local frame A (example:
/// rbd::MultiBodyConfig::bodyAccB), this information must be passed as a parameter.
/// @param pTransform The pose of the frame within the other frame, stored as a sva PTransform object.
/// @param vel The velocity of the frame A inside B.
/// @param velIsGlobal If true, the velocity vectors are expressed in the global frame (B), if false, they are expressed
/// in the local frame (A).
stateObservation::kine::Kinematics fromSva(const sva::PTransformd & pTransform,
                                           const sva::MotionVecd & vel,
                                           bool velIsGlobal = true);

/// @brief Creates a Kinematics object from a PTransformd object that contains the position and the orientation of a
/// frame A within another frame B, and from two MotionVecd object that contain the associated velocities and
/// accelerations.
/// @details The motion vectors given by mc_rtc can be expressed in the global frame B (example:
/// rbd::MultiBodyConfig::bodyVelW), but some are expressed in the local frame A (example:
/// rbd::MultiBodyConfig::bodyAccB), this information must be passed as a parameter.
/// @param pTransform The pose of the frame within the other frame, stored as a sva PTransform object.
/// @param vel The linear and angular velocities of the frame A inside B.
/// @param acc The linear and angular accelerations of the frame A inside B.
/// @param velIsGlobal If true, the velocity vectors are expressed in the global frame (B), if false, they are expressed
/// in the local frame (A).
/// @param accIsGlobal If true, the acceleration vectors are expressed in the global frame (B), if false, they are
/// expressed in the local frame (A).
stateObservation::kine::Kinematics fromSva(const sva::PTransformd & pTransform,
                                           const sva::MotionVecd & vel,
                                           const sva::MotionVecd & acc,
                                           bool velIsGlobal = true,
                                           bool accIsGlobal = true);

/// @brief Sets the velocity variables of a frame A within a frame B contained in a MotionVectord object
/// to the corresponding Kinematics object.
/// @details The motion vectors given by mc_rtc can be expressed in the global frame B (example:
/// rbd::MultiBodyConfig::bodyVelW), but some are expressed in the local frame A (example:
/// rbd::MultiBodyConfig::bodyAccB), this information must be passed as a parameter.
/// @param kine The Kinematics object to enhance with the velocity / acceleration variables.
/// @param vel The linear and angular velocities of the frame A inside B.
/// @param velIsGlobal If true, the velocity vectors are expressed in the global frame (B), if false, they are expressed
/// in the local frame (A).
stateObservation::kine::Kinematics & setVelocities(stateObservation::kine::Kinematics & kine,
                                                   const sva::MotionVecd & vel,
                                                   bool velIsGlobal = true);
/// @brief Sets the velocity variables of a frame A within a frame B contained in a MotionVectord object
/// to the corresponding Kinematics object.
/// @details The motion vectors given by mc_rtc can be expressed in the global frame B (example:
/// rbd::MultiBodyConfig::bodyVelW), but some are expressed in the local frame A (example:
/// rbd::MultiBodyConfig::bodyAccB), this information must be passed as a parameter.
/// @param kine The Kinematics object to enhance with the velocity / acceleration variables.
/// @param vel The linear and angular velocities of the frame A inside B.
/// @param acc The linear and angular accelerations of the frame A inside B.
/// @param velIsGlobal If true, the velocity vectors are expressed in the global frame (B), if false, they are expressed
/// in the local frame (A).
/// @param accIsGlobal If true, the acceleration vectors are expressed in the global frame (B), if false, they are
/// expressed in the local frame (A).
stateObservation::kine::Kinematics & setVelocitiesAndAccelerations(stateObservation::kine::Kinematics & kine,
                                                                   const sva::MotionVecd & vel,
                                                                   const sva::MotionVecd & acc,
                                                                   bool velIsGlobal = true,
                                                                   bool accIsGlobal = true);

/// @brief Converts a Kinematics object to the corresponding PTransformd (position + orientation) object
sva::PTransformd pTransformFromKinematics(const stateObservation::kine::Kinematics & kine);

void addToLogger(mc_rtc::Logger & logger, const stateObservation::kine::Kinematics & kine, const std::string & prefix);

template<typename SourceT = void>
void addToLogger(mc_rtc::Logger & logger,
                 const stateObservation::kine::Kinematics & kine,
                 const std::string & prefix,
                 const SourceT * source)
{
  logger.addLogEntry(prefix + "_position", source,
                     [&kine]() -> const stateObservation::Vector3
                     {
                       if(kine.position.isSet()) { return kine.position(); }
                       else { return stateObservation::Vector3::Zero(); }
                     });
  logger.addLogEntry(prefix + "_ori", source,
                     [&kine]() -> Eigen::Quaterniond
                     {
                       if(kine.orientation.isSet()) { return kine.orientation.inverse().toQuaternion(); }
                       else { return stateObservation::kine::Orientation::zeroRotation().toQuaternion(); }
                     });
  logger.addLogEntry(prefix + "_linVel", source,
                     [&kine]() -> const stateObservation::Vector3
                     {
                       if(kine.linVel.isSet()) { return kine.linVel(); }
                       else { return stateObservation::Vector3::Zero(); };
                     });
  logger.addLogEntry(prefix + "_angVel", source,
                     [&kine]() -> const stateObservation::Vector3
                     {
                       if(kine.angVel.isSet()) { return kine.angVel(); }
                       else { return stateObservation::Vector3::Zero(); };
                     });
  logger.addLogEntry(prefix + "_linAcc", source,
                     [&kine]() -> const stateObservation::Vector3
                     {
                       if(kine.linAcc.isSet()) { return kine.linAcc(); }
                       else { return stateObservation::Vector3::Zero(); };
                     });
  logger.addLogEntry(prefix + "_angAcc", source,
                     [&kine]() -> const stateObservation::Vector3
                     {
                       if(kine.angAcc.isSet()) { return kine.angAcc(); }
                       else { return stateObservation::Vector3::Zero(); };
                     });
}

void removeFromLogger(mc_rtc::Logger & logger, const stateObservation::kine::Kinematics & kine);

void removeFromLogger(mc_rtc::Logger & logger, const std::string & prefix);

} // namespace mc_state_observation::conversions::kinematics
