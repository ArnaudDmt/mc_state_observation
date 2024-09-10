#include <mc_state_observation/conversions/kinematics.h>

namespace so = stateObservation;

namespace mc_state_observation::conversions::kinematics
{

so::kine::Kinematics fromSva(const sva::PTransformd & pTransform, so::kine::Kinematics::Flags::Byte zeroKine)
{
  so::kine::Kinematics kine;
  kine.setZero(zeroKine);
  kine.position = pTransform.translation();
  kine.orientation = so::Matrix3(pTransform.rotation().transpose());
  return kine;
}

so::kine::Kinematics fromSva(const sva::PTransformd & pTransform, const sva::MotionVecd & vel, bool velIsGlobal)
{
  auto kine = fromSva(pTransform, 0);
  if(velIsGlobal)
  {
    kine.linVel = vel.linear();
    kine.angVel = vel.angular();
  }
  else
  {
    kine.linVel = kine.orientation.toMatrix3() * vel.linear();
    kine.angVel = kine.orientation.toMatrix3() * vel.angular();
  }

  return kine;
}

so::kine::Kinematics fromSva(const sva::PTransformd & pTransform,
                             const sva::MotionVecd & vel,
                             const sva::MotionVecd & acc,
                             bool velIsGlobal,
                             bool accIsGlobal)
{
  so::kine::Kinematics kine = fromSva(pTransform, vel, velIsGlobal);
  if(accIsGlobal)
  {
    kine.linAcc = acc.linear();
    kine.angAcc = acc.angular();
  }
  else
  {
    kine.linAcc = kine.orientation.toMatrix3() * acc.linear();
    kine.angAcc = kine.orientation.toMatrix3() * acc.angular();
  }

  return kine;
}

so::kine::Kinematics & setVelocities(so::kine::Kinematics & kine, const sva::MotionVecd & vel, bool velIsGlobal)
{
  BOOST_ASSERT((kine.position.isSet() && kine.orientation.isSet())
               && "The position and the orientation are not set, please give them first");
  if(velIsGlobal)
  {
    kine.linVel = vel.linear();
    kine.angVel = vel.angular();
  }
  else
  {
    kine.linVel = kine.orientation.toMatrix3() * vel.linear();
    kine.angVel = kine.orientation.toMatrix3() * vel.angular();
  }

  return kine;
}

so::kine::Kinematics & setVelocitiesAndAccelerations(so::kine::Kinematics & kine,
                                                     const sva::MotionVecd & vel,
                                                     const sva::MotionVecd & acc,
                                                     bool velIsGlobal,
                                                     bool accIsGlobal) // bodyAccB is local
{
  setVelocities(kine, vel, velIsGlobal);

  if(accIsGlobal)
  {
    kine.linAcc = acc.linear();
    kine.angAcc = acc.angular();
  }
  else
  {
    kine.linAcc = kine.orientation.toMatrix3() * acc.linear();
    kine.angAcc = kine.orientation.toMatrix3() * acc.angular();
  }

  return kine;
}

void addToLogger(mc_rtc::Logger & logger, const stateObservation::kine::Kinematics & kine, const std::string & prefix)
{
  logger.addLogEntry(prefix + "_position", &kine,
                     [&kine]() -> const stateObservation::Vector3
                     {
                       if(kine.position.isSet()) { return kine.position(); }
                       else { return stateObservation::Vector3::Zero(); }
                     });
  logger.addLogEntry(prefix + "_ori", &kine,
                     [&kine]() -> Eigen::Quaterniond
                     {
                       if(kine.orientation.isSet()) { return kine.orientation.inverse().toQuaternion(); }
                       else { return stateObservation::kine::Orientation::zeroRotation().toQuaternion(); }
                     });
  logger.addLogEntry(prefix + "_linVel", &kine,
                     [&kine]() -> const stateObservation::Vector3
                     {
                       if(kine.linVel.isSet()) { return kine.linVel(); }
                       else { return stateObservation::Vector3::Zero(); };
                     });
  logger.addLogEntry(prefix + "_angVel", &kine,
                     [&kine]() -> const stateObservation::Vector3
                     {
                       if(kine.angVel.isSet()) { return kine.angVel(); }
                       else { return stateObservation::Vector3::Zero(); };
                     });
  logger.addLogEntry(prefix + "_linAcc", &kine,
                     [&kine]() -> const stateObservation::Vector3
                     {
                       if(kine.linAcc.isSet()) { return kine.linAcc(); }
                       else { return stateObservation::Vector3::Zero(); };
                     });
  logger.addLogEntry(prefix + "_angAcc", &kine,
                     [&kine]() -> const stateObservation::Vector3
                     {
                       if(kine.angAcc.isSet()) { return kine.angAcc(); }
                       else { return stateObservation::Vector3::Zero(); };
                     });
}

void removeFromLogger(mc_rtc::Logger & logger, const stateObservation::kine::Kinematics & kine)
{
  logger.removeLogEntries(&kine);
}

void removeFromLogger(mc_rtc::Logger & logger, const std::string & prefix)
{
  logger.removeLogEntry(prefix + "_position");
  logger.removeLogEntry(prefix + "_ori");
  logger.removeLogEntry(prefix + "_linVel");
  logger.removeLogEntry(prefix + "_angVel");
  logger.removeLogEntry(prefix + "_linAcc");
  logger.removeLogEntry(prefix + "_angAcc");
}

///////////////////////////////////////////////////////////////////////
/// -------------------Kinematics to SVA conversion--------------------
///////////////////////////////////////////////////////////////////////

sva::PTransformd pTransformFromKinematics(const so::kine::Kinematics & kine)
{
  sva::PTransformd pose;
  pose.translation() = kine.position();
  pose.rotation() = kine.orientation.toMatrix3().transpose();
  return pose;
}

} // namespace mc_state_observation::conversions::kinematics
