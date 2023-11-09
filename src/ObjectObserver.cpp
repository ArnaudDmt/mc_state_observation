#include <mc_control/MCController.h>
#include <mc_observers/ObserverMacros.h>
#include <mc_rtc/ros.h>
#include <mc_rtc/version.h>
#include <SpaceVecAlg/Conversions.h>
#include <SpaceVecAlg/SpaceVecAlg>
#include <Eigen/src/Geometry/Transform.h>
#include <mc_state_observation/ObjectObserver.h>
#include <mc_state_observation/gui_helpers.h>

namespace mc_state_observation
{

ObjectObserver::ObjectObserver(const std::string & type, double dt)
: mc_observers::Observer(type, dt), nh_(mc_rtc::ROSBridge::get_node_handle())
{
}

void ObjectObserver::configure(const mc_control::MCController & controller, const mc_rtc::Configuration & config)
{
  mc_control::MCController & ctl = const_cast<mc_control::MCController &>(controller);
  if(config.has("Robot"))
  {
    robot_ = config("Robot")("robot", ctl.robot().name());
    camera_ = static_cast<std::string>(config("Robot")(robot_)("camera"));
    if(!ctl.robot(robot_).hasBody(camera_))
    {
      mc_rtc::log::error_and_throw<std::runtime_error>("No {} body found in {}", camera_, robot_);
    }
  }
  else { mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Robot configuration is mandatory.", name()); }

  if(config.has("Object"))
  {
    object_ = static_cast<std::string>(config("Object")("robot"));
    topic_ = static_cast<std::string>(config("Object")("topic"));
    isInRobotMap_ = config("Object")("inRobotMap", false);

    robots_ = mc_rbdyn::Robots::make();
    robots_->load(object_, ctl.robot(object_).module());
  }
  else { mc_rtc::log::error_and_throw<std::runtime_error>("[{}] Object configuration is mandatory.", name()); }

  if(config.has("Publish")) { isPublished_ = config("Publish")("use", true); }

  ctl.datastore().make_call(object_ + "::Robot",
                            [this, &ctl]() -> const mc_rbdyn::Robot & { return ctl.realRobot(object_); });

  ctl.datastore().make_call(object_ + "::SLAM::Robot",
                            [this]() -> const mc_rbdyn::Robot & { return robots_->robot(object_); });

  ctl.datastore().make_call(object_ + "::X_0_Object",
                            [this, &ctl]() -> const sva::PTransformd & { return ctl.realRobot(object_).posW(); });

  ctl.datastore().make_call(object_ + "::X_S_Object",
                            [this]() -> const sva::PTransformd & { return robots_->robot(object_).posW(); });

  ctl.datastore().make_call(object_ + "::X_Camera_Object_Estimated",
                            [this]() -> const sva::PTransformd &
                            {
                              const std::lock_guard<std::mutex> lock(mutex_);
                              return X_Camera_EstimatedObject_;
                            });

  ctl.datastore().make_call(object_ + "::X_Camera_Object_Control",
                            [this, &ctl]() -> const sva::PTransformd
                            {
                              sva::PTransformd X_0_camera = ctl.robot(robot_).bodyPosW(camera_);
                              sva::PTransformd X_0_object = ctl.robot(object_).posW();
                              return X_0_object * X_0_camera.inv();
                            });

  ctl.datastore().make_call(object_ + "::X_Camera_Object_Real",
                            [this, &ctl]() -> const sva::PTransformd
                            {
                              sva::PTransformd X_0_camera = ctl.realRobot(robot_).bodyPosW(camera_);
                              sva::PTransformd X_0_object = ctl.realRobot(object_).posW();
                              return X_0_object * X_0_camera.inv();
                            });

  ctl.datastore().make<bool>("Object::" + object_ + "::IsValid", false);

#ifdef MC_STATE_OBSERVATION_ROS_IS_ROS2
  subscriber_ = nh_->create_subscription<PoseStamped>(topic_, 1, [this](const PoseStamped & msg) { callback(msg); });
#else
  subscriber_ = nh_->subscribe(topic_, 1, &ObjectObserver::callback, this);
#endif

  desc_ = fmt::format("{} (Object: {}, Topic: {}, inRobotMap: {})", name(), object_, topic_, isInRobotMap_);

  thread_run_ = true;
  thread_ = std::thread(std::bind(&ObjectObserver::rosSpinner, this));
}

void ObjectObserver::reset(const mc_control::MCController &) {}

bool ObjectObserver::run(const mc_control::MCController &)
{
  return true;
}

void ObjectObserver::update(mc_control::MCController & ctl)
{
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    ctl.datastore().assign<bool>("Object::" + object_ + "::IsValid", isEstimatedPoseValid_);
    if(!isNewEstimatedPose_) { return; }
  }

  const auto & real_robot = ctl.realRobot(robot_);
  const sva::PTransformd X_0_Camera = real_robot.bodyPosW(camera_);
  sva::PTransformd X_Camera_EstimatedObject;
  auto & object = ctl.realRobot(object_);
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    X_Camera_EstimatedObject = X_Camera_EstimatedObject_;
  }
  if(isInRobotMap_)
  {
    const sva::PTransformd & X_0_EstimatedObject = X_Camera_EstimatedObject;
    const auto & real_robot = ctl.realRobot();
    const sva::PTransformd & X_0_FF = real_robot.posW();
    const sva::PTransformd & X_0_Camera = real_robot.bodyPosW(camera_);
    const sva::PTransformd X_Camera_FF = X_0_FF * X_0_Camera.inv();
    const sva::PTransformd X_0_FF_sensor(ctl.robot().bodySensor().orientation(), ctl.robot().bodySensor().position());
    const sva::PTransformd X_0_Camera_sensor = X_Camera_FF.inv() * X_0_FF_sensor;
    X_Camera_EstimatedObject = X_0_EstimatedObject * X_0_Camera_sensor.inv();
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      X_Camera_EstimatedObject_ = X_Camera_EstimatedObject;
    }
  }
  const sva::PTransformd X_0_EstimatedObject = X_Camera_EstimatedObject * X_0_Camera;
  object.posW(X_0_EstimatedObject);
  isNewEstimatedPose_ = false;
  object.forwardKinematics();

  if(ctl.datastore().has("SLAM::Robot"))
  {
    const sva::PTransformd & X_0_Camera =
        ctl.datastore().call<const mc_rbdyn::Robot &>("SLAM::Robot").bodyPosW(camera_);
    auto & object = robots_->robot(object_);
    object.posW(X_Camera_EstimatedObject_ * X_0_Camera);
    object.forwardKinematics();
  }

  if(isPublished_)
  {
    mc_rtc::ROSBridge::update_robot_publisher(object_ + "_estimated", ctl.timeStep, object);
    if(ctl.datastore().has("SLAM::Robot"))
    {
      mc_rtc::ROSBridge::update_robot_publisher(object_ + "_estimated_in_SLAM", ctl.timeStep, robots_->robot(object_));
    }
  }
}

void ObjectObserver::addToLogger(const mc_control::MCController & ctl,
                                 mc_rtc::Logger & logger,
                                 const std::string & category)
{
  logger.addLogEntry(category + "_posW", [this, &ctl]() { return ctl.realRobot(object_).posW(); });
  logger.addLogEntry(category + "_posW_in_SLAM", [this]() { return robots_->robot(object_).posW(); });
  logger.addLogEntry(category + "_X_Camera_Object_Estimated", [this]() { return X_Camera_EstimatedObject_; });
  logger.addLogEntry(category + "_X_Camera_Object_Real",
                     [this, &ctl]() -> const sva::PTransformd
                     {
                       sva::PTransformd X_0_camera = ctl.realRobot(robot_).bodyPosW(camera_);
                       sva::PTransformd X_0_object = ctl.realRobot(object_).posW();
                       return X_0_object * X_0_camera.inv();
                     });
  logger.addLogEntry(category + "_X_Camera_Object_Control",
                     [this, &ctl]() -> const sva::PTransformd
                     {
                       sva::PTransformd X_0_camera = ctl.robot(robot_).bodyPosW(camera_);
                       sva::PTransformd X_0_object = ctl.robot(object_).posW();
                       return X_0_object * X_0_camera.inv();
                     });
}

void ObjectObserver::removeFromLogger(mc_rtc::Logger & logger, const std::string & category)
{
  logger.removeLogEntry(category + "_posW");
  logger.removeLogEntry(category + "_posW_in_SLAM");
  logger.removeLogEntry(category + "_X_Camera_Object_Estimated");
  logger.removeLogEntry(category + "_X_Camera_Object_Real");
  logger.removeLogEntry(category + "_X_Camera_Object_Control");
}

void ObjectObserver::addToGUI(const mc_control::MCController & ctl,
                              mc_rtc::gui::StateBuilder & gui,
                              const std::vector<std::string> & category)
{
  gui.addElement(category,
                 mc_rtc::gui::Transform("X_0_" + object_, [this, &ctl]() { return ctl.realRobot(object_).posW(); }),
                 mc_rtc::gui::Transform("X_Camera_" + object_,
                                        [this]()
                                        {
                                          const std::lock_guard<std::mutex> lock(mutex_);
                                          return X_Camera_EstimatedObject_;
                                        }));
}

void ObjectObserver::callback(const PoseStamped & msg)
{
  Eigen::Affine3d affine;
  tf2::fromMsg(msg.pose, affine);
  const sva::PTransformd newX_Camera_EstimatedObject = sva::conversions::fromHomogeneous(affine.matrix());
  const sva::MotionVecd error = sva::transformError(newX_Camera_EstimatedObject, X_Camera_EstimatedObject_);
  const std::lock_guard<std::mutex> lock(mutex_);
  if(isNotFirstTimeInCallback_ || error.vector().norm() < 0.5)
  {
    X_Camera_EstimatedObject_ = newX_Camera_EstimatedObject;
    isNewEstimatedPose_ = true;
    isEstimatedPoseValid_ = true;
  }
  else { isEstimatedPoseValid_ = false; }

  isNotFirstTimeInCallback_ = true;
}

void ObjectObserver::rosSpinner()
{
  mc_rtc::log::info("[{}] rosSpinner started", name());
  RosRate rate(200);
  while(thread_run_ && ros_ok())
  {
    spinOnce(nh_);
    rate.sleep();
  }
  mc_rtc::log::info("[{}] rosSpinner finished", name());
}

ObjectObserver::~ObjectObserver()
{
  if(thread_.joinable())
  {
    thread_run_ = false;
    thread_.join();
  }
}

} // namespace mc_state_observation

EXPORT_OBSERVER_MODULE("Object", mc_state_observation::ObjectObserver)
