#include <mc_state_observation/svaKinematicsConversion.h>

namespace so = stateObservation;

namespace svaKinematicsConversion
{

so::kine::Kinematics poseFromSva(const sva::PTransformd & pTransform)
{
  so::kine::Kinematics kine;
  kine.setZero(so::kine::Kinematics::Flags::all);
  kine.position = pTransform.translation();
  kine.orientation = so::Matrix3(pTransform.rotation().transpose());
  return kine;
}

so::kine::Kinematics poseAndVelFromSva(const sva::PTransformd & pTransform,
                                       const sva::MotionVecd & vel,
                                       bool velIsGlobal)
{
  so::kine::Kinematics kine;
  kine.setZero(so::kine::Kinematics::Flags::all);
  kine.position = pTransform.translation();
  kine.orientation = so::Matrix3(pTransform.rotation().transpose());
  switch(velIsGlobal)
  {
    case true: // the velocity is expressed in the global frame
      kine.linVel = vel.linear();
      kine.angVel = vel.angular();
      break;
    case false: // the velocity is expressed in the local frame
      kine.linVel = kine.orientation.toMatrix3().transpose() * vel.linear();
      kine.angVel = kine.orientation.toMatrix3().transpose() * vel.angular();
  }

  return kine;
}

so::kine::Kinematics kinematicsFromSva(const sva::PTransformd & pTransform,
                                       const sva::MotionVecd & vel,
                                       const sva::MotionVecd & acc,
                                       bool velIsGlobal,
                                       bool accIsGlobal)
{
  so::kine::Kinematics kine;
  kine.setZero(so::kine::Kinematics::Flags::all);
  kine.position = pTransform.translation();
  kine.orientation = so::Matrix3(pTransform.rotation().transpose());
  switch(velIsGlobal)
  {
    case true: // the velocity is expressed in the global frame
      kine.linVel = vel.linear();
      kine.angVel = vel.angular();
      break;
    case false: // the velocity is expressed in the local frame
      kine.linVel = kine.orientation.toMatrix3().transpose() * vel.linear();
      kine.angVel = kine.orientation.toMatrix3().transpose() * vel.angular();
  }
  switch(accIsGlobal)
  {
    case true: // the velocity is expressed in the global frame
      kine.linAcc = acc.linear();
      kine.angAcc = acc.angular();
      break;
    case false: // the velocity is expressed in the local frame
      kine.linAcc = kine.orientation.toMatrix3().transpose() * acc.linear();
      kine.angAcc = kine.orientation.toMatrix3().transpose() * acc.angular();
  }

  return kine;
}

so::kine::Kinematics & addVelocities(so::kine::Kinematics & kine, const sva::MotionVecd & vel, bool velIsGlobal)
{
  BOOST_ASSERT((kine.position.isSet() && kine.orientation.isSet())
               && "The position and the orientation are not set, please give them first");
  switch(velIsGlobal)
  {
    case true: // the velocity is expressed in the global frame
      kine.linVel = vel.linear();
      kine.angVel = vel.angular();
      break;
    case false: // the velocity is expressed in the local frame
      kine.linVel = kine.orientation.toMatrix3().transpose() * vel.linear();
      kine.angVel = kine.orientation.toMatrix3().transpose() * vel.angular();
  }

  return kine;
}

so::kine::Kinematics & addVelsAndAccs(so::kine::Kinematics & kine,
                                      const sva::MotionVecd & vel,
                                      const sva::MotionVecd & acc,
                                      bool velIsGlobal,
                                      bool accIsGlobal) // bodyAccB is local
{
  BOOST_ASSERT((kine.position.isSet() && kine.orientation.isSet())
               && "The position and the orientation are not set, please give them first");
  switch(velIsGlobal)
  {
    case true: // the velocity is expressed in the global frame
      kine.linVel = vel.linear();
      kine.angVel = vel.angular();
      break;
    case false: // the velocity is expressed in the local frame
      kine.linVel = kine.orientation.toMatrix3().transpose() * vel.linear();
      kine.angVel = kine.orientation.toMatrix3().transpose() * vel.angular();
  }
  switch(accIsGlobal)
  {
    case true: // the velocity is expressed in the global frame
      kine.linAcc = acc.linear();
      kine.angAcc = acc.angular();
      break;
    case false: // the velocity is expressed in the local frame
      kine.linAcc = kine.orientation.toMatrix3().transpose() * acc.linear();
      kine.angAcc = kine.orientation.toMatrix3().transpose() * acc.angular();
  }

  return kine;
}
} // namespace svaKinematicsConversion