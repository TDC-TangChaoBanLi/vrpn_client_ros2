// Copyright 2015 Clearpath Robotics, Inc.
// Copyright 2026 TDC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the conditions of the BSD
// 3-Clause license are met.

#ifndef VRPN_CLIENT_ROS2__VRPN_CLIENT_ROS2_H_
#define VRPN_CLIENT_ROS2__VRPN_CLIENT_ROS2_H_

#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "geometry_msgs/msg/accel_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "vrpn_Connection.h"
#include "vrpn_Tracker.h"

namespace vrpn_client_ros2
{

using ConnectionPtr = std::shared_ptr<vrpn_Connection>;
using TrackerRemotePtr = std::shared_ptr<vrpn_Tracker_Remote>;

struct TrackerOptions
{
  std::string frame_id{"world"};
  std::string topic_prefix;
  std::string pose_topic{"pose"};
  std::string twist_topic{"twist"};
  std::string accel_topic{"accel"};
  bool use_server_time{false};
  bool broadcast_tf{true};
  bool process_sensor_id{false};
};

class VrpnTrackerRos
{
public:
  using Ptr = std::shared_ptr<VrpnTrackerRos>;

  VrpnTrackerRos(
    const std::string & raw_tracker_name,
    const std::string & tracker_topic_name,
    ConnectionPtr connection,
    rclcpp::Node::SharedPtr node,
    const TrackerOptions & options);

  ~VrpnTrackerRos();

  void mainloop();

private:
  using PosePublisher = rclcpp::Publisher<geometry_msgs::msg::PoseStamped>;
  using TwistPublisher = rclcpp::Publisher<geometry_msgs::msg::TwistStamped>;
  using AccelPublisher = rclcpp::Publisher<geometry_msgs::msg::AccelStamped>;

  static void VRPN_CALLBACK handle_pose(void * user_data, const vrpn_TRACKERCB tracker_pose);
  static void VRPN_CALLBACK handle_twist(
    void * user_data,
    const vrpn_TRACKERVELCB tracker_twist);
  static void VRPN_CALLBACK handle_accel(
    void * user_data,
    const vrpn_TRACKERACCCB tracker_accel);

  builtin_interfaces::msg::Time get_stamp(const timeval & msg_time) const;
  std::string make_topic_name(int sensor_id, const std::string & suffix) const;
  std::string make_child_frame_id(int sensor_id) const;
  PosePublisher::SharedPtr get_pose_publisher(int sensor_id);
  TwistPublisher::SharedPtr get_twist_publisher(int sensor_id);
  AccelPublisher::SharedPtr get_accel_publisher(int sensor_id);

  TrackerRemotePtr tracker_remote_;
  rclcpp::Node::SharedPtr node_;
  TrackerOptions options_;
  std::string raw_tracker_name_;
  std::string tracker_topic_name_;

  std::unordered_map<int, PosePublisher::SharedPtr> pose_pubs_;
  std::unordered_map<int, TwistPublisher::SharedPtr> twist_pubs_;
  std::unordered_map<int, AccelPublisher::SharedPtr> accel_pubs_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

class VrpnClientRos : public rclcpp::Node
{
public:
  using Ptr = std::shared_ptr<VrpnClientRos>;
  using TrackerMap = std::unordered_map<std::string, VrpnTrackerRos::Ptr>;

  explicit VrpnClientRos(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  void initialize();
  std::string get_host_string_from_params() const;
  void mainloop();
  void update_trackers();

private:
  void read_parameters();
  void validate_parameters() const;
  void create_tracker(const std::string & raw_tracker_name);
  std::string make_unique_tracker_name(const std::string & raw_tracker_name);
  static std::string sanitize_name_segment(
    const std::string & value,
    const std::string & fallback);
  static std::string normalize_topic_prefix(const std::string & value);

  bool initialized_{false};
  std::string host_;
  int port_{3883};
  double update_frequency_{100.0};
  double refresh_tracker_frequency_{1.0};
  std::vector<std::string> configured_trackers_;
  TrackerOptions tracker_options_;

  ConnectionPtr connection_;
  bool connection_doing_okay_{true};
  rclcpp::Time last_connection_poll_time_;
  TrackerMap trackers_;
  std::unordered_set<std::string> tracker_topic_names_;
  std::unordered_set<std::string> sender_blacklist_{"VRPN Control"};

  rclcpp::TimerBase::SharedPtr refresh_tracker_timer_;
  rclcpp::TimerBase::SharedPtr mainloop_timer_;
};

}  // namespace vrpn_client_ros2

#endif  // VRPN_CLIENT_ROS2__VRPN_CLIENT_ROS2_H_
