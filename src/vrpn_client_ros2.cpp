// Copyright 2015 Clearpath Robotics, Inc.
// Copyright 2026 TDC
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the conditions of the BSD
// 3-Clause license are met.

#include "vrpn_client_ros2/vrpn_client_ros2.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <stdexcept>
#include <string>
#include <utility>

#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace vrpn_client_ros2
{
namespace
{

std::string strip_slashes(const std::string & value)
{
  const auto first = value.find_first_not_of('/');
  if (first == std::string::npos) {
    return "";
  }

  const auto last = value.find_last_not_of('/');
  return value.substr(first, last - first + 1);
}

bool is_valid_first_char(unsigned char c)
{
  return std::isalpha(c) != 0 || c == '_';
}

bool is_valid_name_char(unsigned char c)
{
  return std::isalnum(c) != 0 || c == '_';
}

}  // namespace

VrpnTrackerRos::VrpnTrackerRos(
  const std::string & raw_tracker_name,
  const std::string & tracker_topic_name,
  ConnectionPtr connection,
  rclcpp::Node::SharedPtr node,
  const TrackerOptions & options)
: node_(std::move(node)),
  options_(options),
  raw_tracker_name_(raw_tracker_name),
  tracker_topic_name_(tracker_topic_name)
{
  if (!connection) {
    throw std::invalid_argument("VRPN connection must not be null");
  }
  if (!node_) {
    throw std::invalid_argument("ROS node must not be null");
  }

  tracker_remote_ = std::make_shared<vrpn_Tracker_Remote>(
    raw_tracker_name_.c_str(),
    connection.get());
  tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_pose);
  tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_twist);
  tracker_remote_->register_change_handler(this, &VrpnTrackerRos::handle_accel);
  tracker_remote_->shutup = true;

  if (options_.broadcast_tf) {
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "Created VRPN tracker '%s' as ROS name '%s'",
    raw_tracker_name_.c_str(),
    tracker_topic_name_.c_str());
}

VrpnTrackerRos::~VrpnTrackerRos()
{
  if (!tracker_remote_) {
    return;
  }

  tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_pose);
  tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_twist);
  tracker_remote_->unregister_change_handler(this, &VrpnTrackerRos::handle_accel);

  if (node_) {
    RCLCPP_INFO(
      node_->get_logger(),
      "Destroyed VRPN tracker '%s'",
      raw_tracker_name_.c_str());
  }
}

void VrpnTrackerRos::mainloop()
{
  tracker_remote_->mainloop();
}

builtin_interfaces::msg::Time VrpnTrackerRos::get_stamp(const timeval & msg_time) const
{
  if (!options_.use_server_time) {
    return node_->now();
  }

  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(msg_time.tv_sec);
  stamp.nanosec = static_cast<uint32_t>(msg_time.tv_usec) * 1000U;
  return stamp;
}

std::string VrpnTrackerRos::make_topic_name(
  int sensor_id,
  const std::string & suffix) const
{
  std::string topic_name;
  if (!options_.topic_prefix.empty()) {
    topic_name += options_.topic_prefix + "/";
  }

  topic_name += tracker_topic_name_;
  if (options_.process_sensor_id) {
    topic_name += "/" + std::to_string(sensor_id);
  }

  topic_name += "/" + suffix;
  return topic_name;
}

std::string VrpnTrackerRos::make_child_frame_id(int sensor_id) const
{
  if (options_.process_sensor_id) {
    return tracker_topic_name_ + "/" + std::to_string(sensor_id);
  }

  return tracker_topic_name_;
}

VrpnTrackerRos::PosePublisher::SharedPtr VrpnTrackerRos::get_pose_publisher(int sensor_id)
{
  auto & publisher = pose_pubs_[sensor_id];
  if (!publisher) {
    publisher = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      make_topic_name(sensor_id, options_.pose_topic),
      rclcpp::QoS(rclcpp::KeepLast(1)));
  }
  return publisher;
}

VrpnTrackerRos::TwistPublisher::SharedPtr VrpnTrackerRos::get_twist_publisher(int sensor_id)
{
  auto & publisher = twist_pubs_[sensor_id];
  if (!publisher) {
    publisher = node_->create_publisher<geometry_msgs::msg::TwistStamped>(
      make_topic_name(sensor_id, options_.twist_topic),
      rclcpp::QoS(rclcpp::KeepLast(1)));
  }
  return publisher;
}

VrpnTrackerRos::AccelPublisher::SharedPtr VrpnTrackerRos::get_accel_publisher(int sensor_id)
{
  auto & publisher = accel_pubs_[sensor_id];
  if (!publisher) {
    publisher = node_->create_publisher<geometry_msgs::msg::AccelStamped>(
      make_topic_name(sensor_id, options_.accel_topic),
      rclcpp::QoS(rclcpp::KeepLast(1)));
  }
  return publisher;
}

void VRPN_CALLBACK VrpnTrackerRos::handle_pose(
  void * user_data,
  const vrpn_TRACKERCB tracker_pose)
{
  auto * tracker = static_cast<VrpnTrackerRos *>(user_data);
  const int sensor_id = tracker_pose.sensor;

  if (sensor_id < 0) {
    RCLCPP_WARN_THROTTLE(
      tracker->node_->get_logger(),
      *tracker->node_->get_clock(),
      5000,
      "Ignoring pose from tracker '%s' with negative sensor id %d",
      tracker->raw_tracker_name_.c_str(),
      sensor_id);
    return;
  }

  auto publisher = tracker->get_pose_publisher(sensor_id);
  auto stamp = tracker->get_stamp(tracker_pose.msg_time);

  if (publisher->get_subscription_count() > 0) {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = tracker->options_.frame_id;
    pose_msg.pose.position.x = tracker_pose.pos[0];
    pose_msg.pose.position.y = tracker_pose.pos[1];
    pose_msg.pose.position.z = tracker_pose.pos[2];
    pose_msg.pose.orientation.x = tracker_pose.quat[0];
    pose_msg.pose.orientation.y = tracker_pose.quat[1];
    pose_msg.pose.orientation.z = tracker_pose.quat[2];
    pose_msg.pose.orientation.w = tracker_pose.quat[3];
    publisher->publish(pose_msg);
  }

  if (tracker->options_.broadcast_tf && tracker->tf_broadcaster_) {
    geometry_msgs::msg::TransformStamped transform_msg;
    transform_msg.header.stamp = stamp;
    transform_msg.header.frame_id = tracker->options_.frame_id;
    transform_msg.child_frame_id = tracker->make_child_frame_id(sensor_id);
    transform_msg.transform.translation.x = tracker_pose.pos[0];
    transform_msg.transform.translation.y = tracker_pose.pos[1];
    transform_msg.transform.translation.z = tracker_pose.pos[2];
    transform_msg.transform.rotation.x = tracker_pose.quat[0];
    transform_msg.transform.rotation.y = tracker_pose.quat[1];
    transform_msg.transform.rotation.z = tracker_pose.quat[2];
    transform_msg.transform.rotation.w = tracker_pose.quat[3];
    tracker->tf_broadcaster_->sendTransform(transform_msg);
  }
}

void VRPN_CALLBACK VrpnTrackerRos::handle_twist(
  void * user_data,
  const vrpn_TRACKERVELCB tracker_twist)
{
  auto * tracker = static_cast<VrpnTrackerRos *>(user_data);
  const int sensor_id = tracker_twist.sensor;

  if (sensor_id < 0) {
    RCLCPP_WARN_THROTTLE(
      tracker->node_->get_logger(),
      *tracker->node_->get_clock(),
      5000,
      "Ignoring twist from tracker '%s' with negative sensor id %d",
      tracker->raw_tracker_name_.c_str(),
      sensor_id);
    return;
  }

  auto publisher = tracker->get_twist_publisher(sensor_id);
  if (publisher->get_subscription_count() == 0) {
    return;
  }

  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3 rot_mat(
    tf2::Quaternion(
      tracker_twist.vel_quat[0],
      tracker_twist.vel_quat[1],
      tracker_twist.vel_quat[2],
      tracker_twist.vel_quat[3]));
  rot_mat.getRPY(roll, pitch, yaw);

  geometry_msgs::msg::TwistStamped twist_msg;
  twist_msg.header.stamp = tracker->get_stamp(tracker_twist.msg_time);
  twist_msg.header.frame_id = tracker->options_.frame_id;
  twist_msg.twist.linear.x = tracker_twist.vel[0];
  twist_msg.twist.linear.y = tracker_twist.vel[1];
  twist_msg.twist.linear.z = tracker_twist.vel[2];
  twist_msg.twist.angular.x = roll;
  twist_msg.twist.angular.y = pitch;
  twist_msg.twist.angular.z = yaw;
  publisher->publish(twist_msg);
}

void VRPN_CALLBACK VrpnTrackerRos::handle_accel(
  void * user_data,
  const vrpn_TRACKERACCCB tracker_accel)
{
  auto * tracker = static_cast<VrpnTrackerRos *>(user_data);
  const int sensor_id = tracker_accel.sensor;

  if (sensor_id < 0) {
    RCLCPP_WARN_THROTTLE(
      tracker->node_->get_logger(),
      *tracker->node_->get_clock(),
      5000,
      "Ignoring accel from tracker '%s' with negative sensor id %d",
      tracker->raw_tracker_name_.c_str(),
      sensor_id);
    return;
  }

  auto publisher = tracker->get_accel_publisher(sensor_id);
  if (publisher->get_subscription_count() == 0) {
    return;
  }

  double roll;
  double pitch;
  double yaw;
  tf2::Matrix3x3 rot_mat(
    tf2::Quaternion(
      tracker_accel.acc_quat[0],
      tracker_accel.acc_quat[1],
      tracker_accel.acc_quat[2],
      tracker_accel.acc_quat[3]));
  rot_mat.getRPY(roll, pitch, yaw);

  geometry_msgs::msg::AccelStamped accel_msg;
  accel_msg.header.stamp = tracker->get_stamp(tracker_accel.msg_time);
  accel_msg.header.frame_id = tracker->options_.frame_id;
  accel_msg.accel.linear.x = tracker_accel.acc[0];
  accel_msg.accel.linear.y = tracker_accel.acc[1];
  accel_msg.accel.linear.z = tracker_accel.acc[2];
  accel_msg.accel.angular.x = roll;
  accel_msg.accel.angular.y = pitch;
  accel_msg.accel.angular.z = yaw;
  publisher->publish(accel_msg);
}

VrpnClientRos::VrpnClientRos(const rclcpp::NodeOptions & options)
: Node("vrpn_client_node", options)
{
  declare_parameter<std::string>("server", "127.0.0.1");
  declare_parameter<int>("port", 3883);
  declare_parameter<double>("update_frequency", 100.0);
  declare_parameter<double>("refresh_tracker_frequency", 1.0);
  declare_parameter<std::string>("frame_id", "world");
  declare_parameter<bool>("use_server_time", false);
  declare_parameter<bool>("broadcast_tf", true);
  declare_parameter<bool>("process_sensor_id", false);
  declare_parameter<std::vector<std::string>>("trackers", std::vector<std::string>{});
  declare_parameter<std::string>("topic_prefix", "");
  declare_parameter<std::string>("pose_topic", "pose");
  declare_parameter<std::string>("twist_topic", "twist");
  declare_parameter<std::string>("accel_topic", "accel");
}

void VrpnClientRos::initialize()
{
  if (initialized_) {
    return;
  }

  read_parameters();
  validate_parameters();
  host_ = get_host_string_from_params();

  RCLCPP_INFO(get_logger(), "Connecting to VRPN server at %s", host_.c_str());
  connection_ = ConnectionPtr(vrpn_get_connection_by_name(host_.c_str()));
  if (!connection_) {
    throw std::runtime_error("VRPN connection allocation failed");
  }

  if (!connection_->connected()) {
    RCLCPP_WARN(
      get_logger(),
      "VRPN connection is not connected yet; mainloop will keep polling");
  } else {
    RCLCPP_INFO(get_logger(), "VRPN connection established");
  }
  connection_doing_okay_ = connection_->doing_okay();
  last_connection_poll_time_ = now() - rclcpp::Duration::from_seconds(1.0);

  for (const auto & tracker_name : configured_trackers_) {
    create_tracker(tracker_name);
  }

  mainloop_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / update_frequency_),
    std::bind(&VrpnClientRos::mainloop, this));

  if (refresh_tracker_frequency_ > 0.0) {
    refresh_tracker_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / refresh_tracker_frequency_),
      std::bind(&VrpnClientRos::update_trackers, this));
  }

  initialized_ = true;
}

void VrpnClientRos::read_parameters()
{
  get_parameter("server", host_);
  get_parameter("port", port_);
  get_parameter("update_frequency", update_frequency_);
  get_parameter("refresh_tracker_frequency", refresh_tracker_frequency_);
  get_parameter("frame_id", tracker_options_.frame_id);
  get_parameter("use_server_time", tracker_options_.use_server_time);
  get_parameter("broadcast_tf", tracker_options_.broadcast_tf);
  get_parameter("process_sensor_id", tracker_options_.process_sensor_id);
  get_parameter("trackers", configured_trackers_);
  get_parameter("topic_prefix", tracker_options_.topic_prefix);
  get_parameter("pose_topic", tracker_options_.pose_topic);
  get_parameter("twist_topic", tracker_options_.twist_topic);
  get_parameter("accel_topic", tracker_options_.accel_topic);

  configured_trackers_.erase(
    std::remove_if(
      configured_trackers_.begin(),
      configured_trackers_.end(),
      [](const std::string & tracker_name) {
        return tracker_name.empty();
      }),
    configured_trackers_.end());

  tracker_options_.topic_prefix = normalize_topic_prefix(tracker_options_.topic_prefix);
  tracker_options_.pose_topic = sanitize_name_segment(tracker_options_.pose_topic, "pose");
  tracker_options_.twist_topic = sanitize_name_segment(tracker_options_.twist_topic, "twist");
  tracker_options_.accel_topic = sanitize_name_segment(tracker_options_.accel_topic, "accel");
}

void VrpnClientRos::validate_parameters() const
{
  if (host_.empty()) {
    throw std::invalid_argument("Parameter 'server' must not be empty");
  }
  if (port_ <= 0 || port_ > 65535) {
    throw std::invalid_argument("Parameter 'port' must be in the range [1, 65535]");
  }
  if (update_frequency_ <= 0.0) {
    throw std::invalid_argument("Parameter 'update_frequency' must be greater than 0");
  }
  if (refresh_tracker_frequency_ < 0.0) {
    throw std::invalid_argument(
      "Parameter 'refresh_tracker_frequency' must be greater than or equal to 0");
  }
}

std::string VrpnClientRos::get_host_string_from_params() const
{
  return host_ + ":" + std::to_string(port_);
}

void VrpnClientRos::mainloop()
{
  if (!connection_) {
    return;
  }

  const auto current_time = now();
  if (!connection_doing_okay_ &&
    (current_time - last_connection_poll_time_) < rclcpp::Duration::from_seconds(1.0))
  {
    return;
  }

  last_connection_poll_time_ = current_time;
  connection_->mainloop();
  connection_doing_okay_ = connection_->doing_okay();
  if (!connection_doing_okay_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "VRPN connection is not doing okay");
    return;
  }

  for (auto & tracker : trackers_) {
    tracker.second->mainloop();
  }
}

void VrpnClientRos::update_trackers()
{
  if (!connection_) {
    return;
  }

  int index = 0;
  while (connection_->sender_name(index) != nullptr) {
    const std::string sender_name = connection_->sender_name(index);
    if (trackers_.count(sender_name) == 0 && sender_blacklist_.count(sender_name) == 0) {
      create_tracker(sender_name);
    }
    ++index;
  }
}

void VrpnClientRos::create_tracker(const std::string & raw_tracker_name)
{
  if (raw_tracker_name.empty()) {
    RCLCPP_WARN(get_logger(), "Ignoring VRPN tracker with an empty name");
    return;
  }

  const auto tracker_topic_name = make_unique_tracker_name(raw_tracker_name);
  if (tracker_topic_name != raw_tracker_name) {
    RCLCPP_WARN(
      get_logger(),
      "Using ROS-safe tracker name '%s' for VRPN tracker '%s'",
      tracker_topic_name.c_str(),
      raw_tracker_name.c_str());
  }

  trackers_.emplace(
    raw_tracker_name,
    std::make_shared<VrpnTrackerRos>(
      raw_tracker_name,
      tracker_topic_name,
      connection_,
      std::static_pointer_cast<rclcpp::Node>(shared_from_this()),
      tracker_options_));
}

std::string VrpnClientRos::make_unique_tracker_name(const std::string & raw_tracker_name)
{
  const auto base_name = sanitize_name_segment(raw_tracker_name, "tracker");
  std::string candidate = base_name;
  int suffix = 2;

  while (tracker_topic_names_.count(candidate) != 0) {
    candidate = base_name + "_" + std::to_string(suffix);
    ++suffix;
  }

  tracker_topic_names_.insert(candidate);
  return candidate;
}

std::string VrpnClientRos::sanitize_name_segment(
  const std::string & value,
  const std::string & fallback)
{
  std::string sanitized;
  sanitized.reserve(value.size());

  for (const unsigned char c : value) {
    sanitized.push_back(is_valid_name_char(c) ? static_cast<char>(c) : '_');
  }

  sanitized.erase(
    std::unique(sanitized.begin(), sanitized.end(), [](char lhs, char rhs) {
      return lhs == '_' && rhs == '_';
    }),
    sanitized.end());

  while (!sanitized.empty() && sanitized.front() == '_') {
    sanitized.erase(sanitized.begin());
  }
  while (!sanitized.empty() && sanitized.back() == '_') {
    sanitized.pop_back();
  }

  if (sanitized.empty()) {
    sanitized = fallback;
  }
  if (!is_valid_first_char(static_cast<unsigned char>(sanitized.front()))) {
    sanitized.insert(sanitized.begin(), '_');
  }

  return sanitized;
}

std::string VrpnClientRos::normalize_topic_prefix(const std::string & value)
{
  const auto stripped = strip_slashes(value);
  if (stripped.empty()) {
    return "";
  }

  std::string normalized;
  normalized.reserve(stripped.size());
  bool previous_was_slash = false;

  for (const char c : stripped) {
    if (c == '/') {
      if (!previous_was_slash) {
        normalized.push_back('/');
      }
      previous_was_slash = true;
      continue;
    }

    normalized.push_back(c);
    previous_was_slash = false;
  }

  return normalized;
}

}  // namespace vrpn_client_ros2
