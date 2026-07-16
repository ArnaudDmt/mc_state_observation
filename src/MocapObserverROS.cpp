#include <mc_state_observation/MocapObserverROS.h>

#include <mc_observers/ObserverMacros.h>

#include <SpaceVecAlg/Conversions.h>

namespace mc_state_observation
{

MocapObserverROS::MocapObserverROS(const std::string & type, double dt)
: MocapObserver(type, dt), nh_(mc_rtc::ROSBridge::get_node_handle())
#ifdef MC_STATE_OBSERVATION_ROS_IS_ROS2
  ,
  tfBuffer_(nh_->get_clock())
#endif
{
}

void MocapObserverROS::configure(const mc_control::MCController & ctl, const mc_rtc::Configuration & config)
{
  MocapObserver::configure(ctl, config);
  config("marker_tf", marker_);
  config("marker_origin_tf", markerOrigin_);
  desc_ =
      fmt::format("{} (Marker TF: {} -> {}, Body: {}, Update: {})", name_, markerOrigin_, marker_, body_, updateRobot_);
}

void MocapObserverROS::reset(const mc_control::MCController & ctl)
{ MocapObserver::reset(ctl); }

bool MocapObserverROS::run(const mc_control::MCController & ctl)
{
  TransformStamped transformStamped;
  try
  {
    transformStamped = tfBuffer_.lookupTransform(markerOrigin_, marker_, RosTime(0));
  }
  catch(tf2::TransformException & ex)
  {
    error_ = ex.what();
    return false;
  }
  auto pose = sva::conversions::fromHomogeneous(tf2::transformToEigen(transformStamped).matrix());
  MocapObserver::markerPose(pose);

  return MocapObserver::run(ctl);
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("MocapObserverROS", mc_state_observation::MocapObserverROS)
