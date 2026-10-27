#pragma once

#include <mc_rtc_ros/ros.h>

#ifdef MC_STATE_OBSERVATION_ROS_IS_ROS2
#  include <geometry_msgs/msg/pose_stamped.hpp>
#  include <geometry_msgs/msg/transform_stamped.hpp>
#  include <tf2_eigen/tf2_eigen.hpp>
#  include <rclcpp/rclcpp.hpp>
#else
#  include <geometry_msgs/PoseStamped.h>
#  include <geometry_msgs/TransformStamped.h>
#  include <ros/ros.h>
#  include <tf2_eigen/tf2_eigen.h>
#endif

namespace mc_state_observation
{

#ifdef MC_STATE_OBSERVATION_ROS_IS_ROS2
using PoseStamped = geometry_msgs::msg::PoseStamped;
using TransformStamped = geometry_msgs::msg::TransformStamped;
using PoseSubscriber = rclcpp::Subscription<PoseStamped>::SharedPtr;
using RosRate = rclcpp::Rate;
using RosTime = rclcpp::Time;

inline bool ros_ok()
{ return rclcpp::ok(); }

inline void spinOnce(const mc_rtc::NodeHandlePtr & nh)
{ rclcpp::spin_some(nh); }

inline RosTime RosTimeNow()
{ return rclcpp::Clock().now(); }
#else
using PoseStamped = geometry_msgs::PoseStamped;
using TransformStamped = geometry_msgs::TransformStamped;
using PoseSubscriber = ros::Subscriber;
using RosRate = ros::Rate;
using RosTime = ros::Time;

inline bool ros_ok()
{ return ros::ok(); }

inline void spinOnce(const mc_rtc::NodeHandlePtr &)
{ ros::spinOnce(); }

inline RosTime RosTimeNow()
{ return RosTime::now(); }
#endif

} // namespace mc_state_observation
