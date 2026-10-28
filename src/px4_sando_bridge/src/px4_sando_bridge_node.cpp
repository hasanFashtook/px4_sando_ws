// Copyright 2026, SANDO Lyrical / PX4 integration
// SPDX-License-Identifier: MIT

#include "px4_sando_bridge/px4_sando_bridge_node.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <limits>
#include <sstream>
#include <iomanip>
#include <stdexcept>
#include <utility>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/LinearMath/Vector3.hpp>

#include "px4_sando_bridge/frame_transforms.hpp"

namespace px4_sando_bridge
{
namespace
{

bool allFinite(const std::array<float, 3> & values)
{
  return std::isfinite(values[0]) && std::isfinite(values[1]) &&
         std::isfinite(values[2]);
}

Quaternion quaternionFromPx4(const std::array<float, 4> & q)
{
  return {static_cast<double>(q[0]), static_cast<double>(q[1]),
    static_cast<double>(q[2]), static_cast<double>(q[3])};
}

double sanitize(double value, double fallback = 0.0)
{
  return std::isfinite(value) ? value : fallback;
}

}  // namespace

Px4SandoBridgeNode::Px4SandoBridgeNode(const rclcpp::NodeOptions & options)
: Node("px4_sando_bridge_node", options),
  watchdog_(std::chrono::milliseconds(250)),
  timestamp_sync_()
{
  declareAndLoadParameters();
  timestamp_sync_.updateClocks();
  node_start_time_ = std::chrono::steady_clock::now();
  last_timestamp_log_time_ = node_start_time_;

  watchdog_.setTimeout(std::chrono::milliseconds(watchdog_timeout_ms_));

  if (auto_arm_and_offboard_) {
    engage_requested_.store(true);
  }

  createRosInterfaces();

  RCLCPP_INFO(
    get_logger(),
    "px4_sando_bridge started: ns=%s state=%s goal=%s trajectory=%s term_goal=%s",
    drone_namespace_.c_str(), state_topic_.c_str(), goal_topic_.c_str(),
    trajectory_topic_.c_str(), term_goal_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "PX4 interface: odom=%s status=%s setpoint=%s offboard=%s command=%s",
    px4_odometry_topic_.c_str(), px4_status_topic_.c_str(),
    px4_setpoint_topic_.c_str(), px4_offboard_mode_topic_.c_str(),
    px4_vehicle_command_topic_.c_str());
  RCLCPP_INFO(
    get_logger(),
    "rates: heartbeat=%.1f Hz setpoint=%.1f Hz watchdog=%d ms warmup=%.2f s",
    heartbeat_rate_hz_, setpoint_stream_rate_hz_, watchdog_timeout_ms_,
    offboard_warmup_duration_sec_);
  RCLCPP_INFO(
    get_logger(),
    "auto_takeoff=%s (altitude=%.2f m) accel_feedforward=%s yawspeed_feedforward=%s yaw_mode=%s auto_arm_and_offboard=%s enu_ned_conversion=%s",
    auto_takeoff_ ? "true" : "false", takeoff_altitude_m_,
    enable_acceleration_feedforward_ ? "true" : "false",
    enable_yawspeed_feedforward_ ? "true" : "false",
    yaw_mode_.c_str(),
    auto_arm_and_offboard_ ? "true" : "false",
    enable_enu_ned_conversion_ ? "true" : "false");
}

void Px4SandoBridgeNode::declareAndLoadParameters()
{
  drone_namespace_ = declare_parameter<std::string>("drone_namespace", "NX01");
  world_frame_id_ = declare_parameter<std::string>("world_frame_id", "world");
  body_frame_id_ = declare_parameter<std::string>("body_frame_id", "base_link");
  livox_frame_id_ = declare_parameter<std::string>("livox_frame_id", "livox_frame");

  heartbeat_rate_hz_ = declare_parameter<double>("heartbeat_rate_hz", 20.0);
  setpoint_stream_rate_hz_ =
    declare_parameter<double>("setpoint_stream_rate_hz", 20.0);
  telemetry_rate_hz_ = declare_parameter<double>("telemetry_rate_hz", 50.0);
  tf_rate_hz_ = declare_parameter<double>("tf_rate_hz", 20.0);
  watchdog_timeout_ms_ = declare_parameter<int>("watchdog_timeout_ms", 250);
  offboard_warmup_duration_sec_ =
    declare_parameter<double>("offboard_warmup_duration_sec", 1.0);

  enable_enu_ned_conversion_ =
    declare_parameter<bool>("enable_enu_ned_conversion", true);
  auto_arm_and_offboard_ =
    declare_parameter<bool>("auto_arm_and_offboard", false);
  auto_takeoff_ = declare_parameter<bool>("auto_takeoff", true);
  takeoff_altitude_m_ = declare_parameter<double>("takeoff_altitude", 2.5);
  enable_acceleration_feedforward_ =
    declare_parameter<bool>("enable_acceleration_feedforward", true);
  enable_yawspeed_feedforward_ =
    declare_parameter<bool>("enable_yawspeed_feedforward", false);
  yaw_mode_ = declare_parameter<std::string>("yaw_mode", "none");
  fixed_yaw_rad_ = declare_parameter<double>("fixed_yaw_rad", 0.0);
  max_acceleration_m_s2_ =
    declare_parameter<double>("max_acceleration_m_s2", 5.0);
  publish_tf_ = declare_parameter<bool>("publish_tf", true);
  publish_odom_ = declare_parameter<bool>("publish_odom", true);
  publish_setpoint_marker_ =
    declare_parameter<bool>("publish_setpoint_marker", true);
  setpoint_marker_topic_ = declare_parameter<std::string>(
    "setpoint_marker_topic", nsTopic("desired_setpoint_marker"));
  setpoint_marker_color_rgb_ = declare_parameter<std::vector<double>>(
    "setpoint_marker_color_rgb", std::vector<double>{0.0, 0.8, 1.0});  // Cyan
  setpoint_marker_scale_ =
    declare_parameter<double>("setpoint_marker_scale", 0.35);
  publish_velocity_text_ =
    declare_parameter<bool>("publish_velocity_text", true);
  velocity_text_topic_ = declare_parameter<std::string>(
    "velocity_text_topic", nsTopic("drone_speed_text"));
  velocity_text_offset_z_ =
    declare_parameter<double>("velocity_text_offset_z", 0.65);
  velocity_text_scale_ =
    declare_parameter<double>("velocity_text_scale", 0.35);

  max_horizontal_velocity_m_s_ =
    declare_parameter<double>("max_horizontal_velocity_m_s", 3.0);
  max_vertical_velocity_m_s_ =
    declare_parameter<double>("max_vertical_velocity_m_s", 1.5);
  max_yaw_rate_rad_s_ = declare_parameter<double>("max_yaw_rate_rad_s", 1.0);
  geofence_min_z_m_ = declare_parameter<double>("geofence_min_z_m", 0.2);
  geofence_max_z_m_ = declare_parameter<double>("geofence_max_z_m", 10.0);

  if (heartbeat_rate_hz_ <= 0.0) {
    throw std::invalid_argument("heartbeat_rate_hz must be positive");
  }
  if (setpoint_stream_rate_hz_ <= 0.0) {
    throw std::invalid_argument("setpoint_stream_rate_hz must be positive");
  }
  if (telemetry_rate_hz_ <= 0.0) {
    throw std::invalid_argument("telemetry_rate_hz must be positive");
  }

  state_topic_ = declare_parameter<std::string>(
    "state_topic", nsTopic("state"));
  odom_topic_ = declare_parameter<std::string>(
    "odom_topic", nsTopic("odom"));
  goal_topic_ = declare_parameter<std::string>(
    "goal_topic", nsTopic("goal"));
  trajectory_topic_ = declare_parameter<std::string>(
    "trajectory_topic", nsTopic("trajectory"));
  term_goal_topic_ = declare_parameter<std::string>(
    "term_goal_topic", nsTopic("term_goal"));

  px4_odometry_topic_ = declare_parameter<std::string>(
    "px4_odometry_topic", "/fmu/out/vehicle_odometry");
  px4_status_topic_ = declare_parameter<std::string>(
    "px4_status_topic", "/fmu/out/vehicle_status");
  px4_setpoint_topic_ = declare_parameter<std::string>(
    "px4_setpoint_topic", "/fmu/in/trajectory_setpoint");
  px4_offboard_mode_topic_ = declare_parameter<std::string>(
    "px4_offboard_mode_topic", "/fmu/in/offboard_control_mode");
  px4_vehicle_command_topic_ = declare_parameter<std::string>(
    "px4_vehicle_command_topic", "/fmu/in/vehicle_command");
}

std::string Px4SandoBridgeNode::nsTopic(const std::string & suffix) const
{
  std::string ns = drone_namespace_;
  while (!ns.empty() && ns.front() == '/') {
    ns.erase(ns.begin());
  }
  while (!ns.empty() && ns.back() == '/') {
    ns.pop_back();
  }

  if (ns.empty()) {
    return "/" + suffix;
  }
  return "/" + ns + "/" + suffix;
}

void Px4SandoBridgeNode::createRosInterfaces()
{
  const auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort();
  const auto planner_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

  state_pub_ = create_publisher<dynus_interfaces::msg::State>(
    state_topic_, planner_qos);
  if (publish_odom_) {
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(
      odom_topic_, planner_qos);
  }

  setpoint_pub_ = create_publisher<px4_msgs::msg::TrajectorySetpoint>(
    px4_setpoint_topic_, px4_qos);
  if (publish_setpoint_marker_) {
    setpoint_marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      setpoint_marker_topic_, rclcpp::QoS(10));
  }
  if (publish_velocity_text_) {
    velocity_text_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      velocity_text_topic_, rclcpp::QoS(10));
  }
  offboard_mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
    px4_offboard_mode_topic_, px4_qos);
  vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
    px4_vehicle_command_topic_, command_qos);
  term_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
    term_goal_topic_, planner_qos);

  odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
    px4_odometry_topic_, px4_qos,
    std::bind(&Px4SandoBridgeNode::odometryCallback, this,
    std::placeholders::_1));
  status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    px4_status_topic_, px4_qos,
    std::bind(&Px4SandoBridgeNode::vehicleStatusCallback, this,
    std::placeholders::_1));
  status_versioned_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
    px4_status_topic_ + "_v4", px4_qos,
    std::bind(&Px4SandoBridgeNode::vehicleStatusCallback, this,
    std::placeholders::_1));
  goal_sub_ = create_subscription<dynus_interfaces::msg::Goal>(
    goal_topic_, planner_qos,
    std::bind(&Px4SandoBridgeNode::goalCallback, this,
    std::placeholders::_1));
  trajectory_sub_ = create_subscription<dynus_interfaces::msg::Trajectory>(
    trajectory_topic_, planner_qos,
    std::bind(&Px4SandoBridgeNode::trajectoryCallback, this,
    std::placeholders::_1));
  rviz_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", rclcpp::QoS(10).reliable(),
    std::bind(&Px4SandoBridgeNode::rvizGoalCallback, this,
    std::placeholders::_1));

  engage_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/engage",
    std::bind(&Px4SandoBridgeNode::handleEngage, this,
    std::placeholders::_1, std::placeholders::_2));
  disarm_srv_ = create_service<std_srvs::srv::Trigger>(
    "~/disarm",
    std::bind(&Px4SandoBridgeNode::handleDisarm, this,
    std::placeholders::_1, std::placeholders::_2));
  arm_srv_ = create_service<std_srvs::srv::Trigger>(
    nsTopic("arm"),
    std::bind(&Px4SandoBridgeNode::handleArm, this,
    std::placeholders::_1, std::placeholders::_2));
  takeoff_srv_ = create_service<std_srvs::srv::Trigger>(
    nsTopic("takeoff"),
    std::bind(&Px4SandoBridgeNode::handleTakeoff, this,
    std::placeholders::_1, std::placeholders::_2));
  land_srv_ = create_service<std_srvs::srv::Trigger>(
    nsTopic("land"),
    std::bind(&Px4SandoBridgeNode::handleLand, this,
    std::placeholders::_1, std::placeholders::_2));

  heartbeat_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / heartbeat_rate_hz_),
    std::bind(&Px4SandoBridgeNode::heartbeatTimerCallback, this));
  setpoint_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / setpoint_stream_rate_hz_),
    std::bind(&Px4SandoBridgeNode::setpointTimerCallback, this));
  telemetry_timer_ = create_wall_timer(
    std::chrono::duration<double>(1.0 / telemetry_rate_hz_),
    std::bind(&Px4SandoBridgeNode::telemetryTimerCallback, this));
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / tf_rate_hz_),
      std::bind(&Px4SandoBridgeNode::publishTransformFromLatest, this));
  }
}

void Px4SandoBridgeNode::rvizGoalCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(
    get_logger(),
    "relaying RViz goal target to %s: (%.2f, %.2f, %.2f)",
    term_goal_topic_.c_str(),
    msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
  term_goal_pub_->publish(*msg);
}

void Px4SandoBridgeNode::odometryCallback(
  const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
{
  timestamp_sync_.updatePx4Timestamp(msg->timestamp);

  if (!allFinite(msg->position) || !allFinite(msg->velocity)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "ignoring vehicle odometry containing non-finite values");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_odom_ = *msg;
    latest_odom_time_ = std::chrono::steady_clock::now();
    has_odom_ = true;
  }

  publishState(*msg);
}

dynus_interfaces::msg::State Px4SandoBridgeNode::odometryToState(
  const px4_msgs::msg::VehicleOdometry & msg) const
{
  dynus_interfaces::msg::State state;
  state.header.stamp = this->now();
  state.header.frame_id = world_frame_id_;

  Vector3 p{static_cast<double>(msg.position[0]),
    static_cast<double>(msg.position[1]),
    static_cast<double>(msg.position[2])};
  Vector3 v{static_cast<double>(msg.velocity[0]),
    static_cast<double>(msg.velocity[1]),
    static_cast<double>(msg.velocity[2])};
  Quaternion q = quaternionFromPx4(msg.q);

  if (enable_enu_ned_conversion_) {
    p = nedToEnu(p);
    v = nedToEnu(v);
    q = nedToEnuQuaternion(q);
  }

  state.pos.x = p[0];
  state.pos.y = p[1];
  state.pos.z = p[2];

  state.vel.x = v[0];
  state.vel.y = v[1];
  state.vel.z = v[2];

  state.quat.w = q[0];
  state.quat.x = q[1];
  state.quat.y = q[2];
  state.quat.z = q[3];

  return state;
}

void Px4SandoBridgeNode::publishState(
  const px4_msgs::msg::VehicleOdometry & msg)
{
  const dynus_interfaces::msg::State state = odometryToState(msg);

  if (auto_takeoff_ && !state_publication_started_.load()) {
    const double current_z = state.pos.z;
    if (!offboard_.load() && current_z < (takeoff_altitude_m_ - 0.4)) {
      return;
    }
    state_publication_started_.store(true);
    RCLCPP_INFO(
      get_logger(),
      "vehicle reached flight altitude (%.2f m); starting /%s/state stream to SANDO",
      current_z, drone_namespace_.c_str());
  }

  state_pub_->publish(state);
}

void Px4SandoBridgeNode::telemetryTimerCallback()
{
  px4_msgs::msg::VehicleOdometry odom;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_odom_) {
      return;
    }
    odom = latest_odom_;
  }

  publishOdometry(odom);
}

void Px4SandoBridgeNode::publishOdometry(
  const px4_msgs::msg::VehicleOdometry & msg)
{
  const dynus_interfaces::msg::State state = odometryToState(msg);

  if (odom_pub_) {
    nav_msgs::msg::Odometry odom;
    odom.header = state.header;
    odom.child_frame_id = body_frame_id_;
    odom.pose.pose.position.x = state.pos.x;
    odom.pose.pose.position.y = state.pos.y;
    odom.pose.pose.position.z = state.pos.z;
    odom.pose.pose.orientation = state.quat;
    odom.twist.twist.linear.x = state.vel.x;
    odom.twist.twist.linear.y = state.vel.y;
    odom.twist.twist.linear.z = state.vel.z;
    odom_pub_->publish(odom);
  }
}

void Px4SandoBridgeNode::publishTransformFromLatest()
{
  dynus_interfaces::msg::State state;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!has_odom_) {
      return;
    }
    state = odometryToState(latest_odom_);
  }
  publishTransform(state);
}

void Px4SandoBridgeNode::publishTransform(
  const dynus_interfaces::msg::State & state)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = state.header.stamp;
  transform.header.frame_id = world_frame_id_;
  transform.child_frame_id = body_frame_id_;
  transform.transform.translation.x = state.pos.x;
  transform.transform.translation.y = state.pos.y;
  transform.transform.translation.z = state.pos.z;
  transform.transform.rotation = state.quat;
  tf_broadcaster_->sendTransform(transform);
}

void Px4SandoBridgeNode::vehicleStatusCallback(
  const px4_msgs::msg::VehicleStatus::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_status_ = *msg;
    has_status_ = true;
  }

  const bool is_armed =
    msg->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  const bool is_offboard =
    msg->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;

  armed_.store(is_armed);
  offboard_.store(is_offboard);

  if (is_offboard) {
    if (!was_offboard_) {
      RCLCPP_INFO(
        get_logger(), "vehicle entered OFFBOARD mode (armed=%s)",
        is_armed ? "true" : "false");
    }
    was_offboard_ = true;
    manual_override_.store(false);
    setpoint_stream_enabled_.store(true);
    warned_manual_override_ = false;
  } else if (was_offboard_) {
    was_offboard_ = false;
    manual_override_.store(true);
    setpoint_stream_enabled_.store(false);
    watchdog_.reset();
    RCLCPP_WARN(
      get_logger(),
      "vehicle left OFFBOARD (nav_state=%u, failsafe=%s): setpoint streaming stopped",
      static_cast<unsigned>(msg->nav_state),
      msg->failsafe ? "true" : "false");
  }
}

void Px4SandoBridgeNode::goalCallback(
  const dynus_interfaces::msg::Goal::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_goal_ = *msg;
    latest_goal_time_ = std::chrono::steady_clock::now();
    has_goal_ = true;
    hold_active_ = false;
  }
  watchdog_.feed();
}

void Px4SandoBridgeNode::trajectoryCallback(
  const dynus_interfaces::msg::Trajectory::SharedPtr msg)
{
  if (msg->goals.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "ignoring empty trajectory message");
    return;
  }

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_trajectory_ = *msg;
    latest_trajectory_time_ = std::chrono::steady_clock::now();
    has_trajectory_ = true;
    hold_active_ = false;
  }
  watchdog_.feed();
}

void Px4SandoBridgeNode::heartbeatTimerCallback()
{
  publishOffboardControlMode();
  ++heartbeat_count_;

  if (auto_takeoff_ || auto_arm_and_offboard_ || engage_requested_.load()) {
    manageAutoFlight();
  }
}

void Px4SandoBridgeNode::setpointTimerCallback()
{
  if (manual_override_.load() || !setpoint_stream_enabled_.load()) {
    if (!warned_manual_override_) {
      warned_manual_override_ = true;
      RCLCPP_WARN(
        get_logger(),
        "setpoint publishing disabled by manual override; call ~/engage to resume");
    }
    return;
  }

  dynus_interfaces::msg::Goal goal;
  const SetpointSource source = selectSetpoint(goal);
  if (source == SetpointSource::kNone) {
    if (!warned_no_odometry_) {
      warned_no_odometry_ = true;
      RCLCPP_WARN(
        get_logger(),
        "no vehicle odometry yet; holding offboard setpoint stream");
    }
    return;
  }

  publishSetpoint(goal, source);
  ++setpoint_count_;
}

Px4SandoBridgeNode::SetpointSource Px4SandoBridgeNode::selectSetpoint(
  dynus_interfaces::msg::Goal & goal)
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  const auto now = std::chrono::steady_clock::now();
  const auto goal_timeout = std::chrono::milliseconds(watchdog_timeout_ms_);

  if (has_goal_ && (now - latest_goal_time_) <= goal_timeout) {
    goal = latest_goal_;
    hold_active_ = false;
    return SetpointSource::kGoal;
  }

  if (has_trajectory_) {
    const double dt = latest_trajectory_.dt > 0.0 ?
      latest_trajectory_.dt : (1.0 / setpoint_stream_rate_hz_);
    const double duration =
      dt * static_cast<double>(latest_trajectory_.goals.size());
    const double elapsed =
      std::chrono::duration<double>(now - latest_trajectory_time_).count();

    if (elapsed <= duration + 0.001 * watchdog_timeout_ms_) {
      if (trajectoryGoalAt(latest_trajectory_, elapsed, goal)) {
        watchdog_.feed();
        hold_active_ = false;
        return SetpointSource::kTrajectory;
      }
    } else {
      has_trajectory_ = false;
    }
  }

  if (!has_odom_) {
    return SetpointSource::kNone;
  }

  if (!offboard_.load() || !hold_active_) {
    const auto & odom = latest_odom_;
    Vector3 p{static_cast<double>(odom.position[0]),
      static_cast<double>(odom.position[1]),
      static_cast<double>(odom.position[2])};
    Quaternion q = quaternionFromPx4(odom.q);
    if (enable_enu_ned_conversion_) {
      p = nedToEnu(p);
      q = nedToEnuQuaternion(q);
    }
    hold_position_ = p;
    hold_yaw_ = yawFromQuaternion(q);
    hold_active_ = true;

    if (offboard_.load()) {
      RCLCPP_WARN(
        get_logger(),
        "setpoint watchdog expired (%.0f ms); position hold at ENU (%.2f, %.2f, %.2f)",
        1000.0 * std::chrono::duration<double>(watchdog_.timeSinceLastFeed()).count(),
        hold_position_[0], hold_position_[1], hold_position_[2]);
    }
  }

  goal = dynus_interfaces::msg::Goal{};
  goal.header.stamp = this->now();
  goal.header.frame_id = world_frame_id_;
  goal.p.x = hold_position_[0];
  goal.p.y = hold_position_[1];
  goal.p.z = hold_position_[2];

  if (auto_takeoff_ && enable_enu_ned_conversion_ && goal.p.z < takeoff_altitude_m_) {
    goal.p.z = takeoff_altitude_m_;
  }
  goal.v.x = 0.0;
  goal.v.y = 0.0;
  goal.v.z = 0.0;
  goal.a.x = 0.0;
  goal.a.y = 0.0;
  goal.a.z = 0.0;
  goal.j.x = 0.0;
  goal.j.y = 0.0;
  goal.j.z = 0.0;
  goal.yaw = hold_yaw_;
  goal.dyaw = 0.0;
  goal.power = true;
  goal.mode_xy = dynus_interfaces::msg::Goal::MODE_POSITION_CONTROL;
  goal.mode_z = dynus_interfaces::msg::Goal::MODE_POSITION_CONTROL;
  return SetpointSource::kHold;
}

bool Px4SandoBridgeNode::trajectoryGoalAt(
  const dynus_interfaces::msg::Trajectory & trajectory,
  double elapsed_sec, dynus_interfaces::msg::Goal & goal) const
{
  if (trajectory.goals.empty()) {
    return false;
  }

  const double dt = trajectory.dt > 0.0 ?
    trajectory.dt : (1.0 / setpoint_stream_rate_hz_);
  const double raw_index = std::max(0.0, elapsed_sec) / std::max(dt, 1e-6);
  const size_t index = std::min(
    trajectory.goals.size() - 1,
    static_cast<size_t>(std::floor(raw_index)));

  goal = trajectory.goals[index];
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = world_frame_id_;
  }
  return true;
}

void Px4SandoBridgeNode::applySafetyLimits(
  dynus_interfaces::msg::Goal & goal) const
{
  goal.v.x = sanitize(goal.v.x);
  goal.v.y = sanitize(goal.v.y);
  goal.v.z = sanitize(goal.v.z);
  goal.p.x = sanitize(goal.p.x);
  goal.p.y = sanitize(goal.p.y);
  goal.p.z = sanitize(goal.p.z);
  goal.a.x = sanitize(goal.a.x);
  goal.a.y = sanitize(goal.a.y);
  goal.a.z = sanitize(goal.a.z);
  goal.yaw = sanitize(goal.yaw);
  goal.dyaw = sanitize(goal.dyaw);

  const double horizontal =
    std::hypot(goal.v.x, goal.v.y);
  if (horizontal > max_horizontal_velocity_m_s_ && horizontal > 1e-9) {
    const double scale = max_horizontal_velocity_m_s_ / horizontal;
    goal.v.x *= scale;
    goal.v.y *= scale;
  }
  goal.v.z = std::clamp(
    goal.v.z, -max_vertical_velocity_m_s_, max_vertical_velocity_m_s_);
  goal.dyaw = std::clamp(
    goal.dyaw, -max_yaw_rate_rad_s_, max_yaw_rate_rad_s_);

  const double accel_mag = std::sqrt(
    goal.a.x * goal.a.x + goal.a.y * goal.a.y + goal.a.z * goal.a.z);
  if (accel_mag > max_acceleration_m_s2_ && accel_mag > 1e-9) {
    const double scale = max_acceleration_m_s2_ / accel_mag;
    goal.a.x *= scale;
    goal.a.y *= scale;
    goal.a.z *= scale;
  }

  if (goal.p.z > 0.0) {
    goal.p.z = std::clamp(goal.p.z, geofence_min_z_m_, geofence_max_z_m_);
  } else {
    goal.p.z = std::min(goal.p.z, geofence_max_z_m_);
  }
}

void Px4SandoBridgeNode::publishSetpoint(
  const dynus_interfaces::msg::Goal & input_goal, SetpointSource source)
{
  dynus_interfaces::msg::Goal goal = input_goal;
  applySafetyLimits(goal);

  px4_msgs::msg::TrajectorySetpoint msg{};
  msg.timestamp = nowUs();

  if (enable_enu_ned_conversion_) {
    const Vector3 p = enuToNed({goal.p.x, goal.p.y, goal.p.z});
    const Vector3 v = enuToNed({goal.v.x, goal.v.y, goal.v.z});
    const Vector3 a = enuToNed({goal.a.x, goal.a.y, goal.a.z});
    const Vector3 j = enuToNed({goal.j.x, goal.j.y, goal.j.z});

    msg.position[0] = static_cast<float>(p[0]);
    msg.position[1] = static_cast<float>(p[1]);
    msg.position[2] = static_cast<float>(p[2]);
    msg.velocity[0] = static_cast<float>(v[0]);
    msg.velocity[1] = static_cast<float>(v[1]);
    msg.velocity[2] = static_cast<float>(v[2]);
    msg.acceleration[0] = static_cast<float>(a[0]);
    msg.acceleration[1] = static_cast<float>(a[1]);
    msg.acceleration[2] = static_cast<float>(a[2]);
    msg.jerk[0] = static_cast<float>(j[0]);
    msg.jerk[1] = static_cast<float>(j[1]);
    msg.jerk[2] = static_cast<float>(j[2]);
  } else {
    msg.position[0] = static_cast<float>(goal.p.x);
    msg.position[1] = static_cast<float>(goal.p.y);
    msg.position[2] = static_cast<float>(goal.p.z);
    msg.velocity[0] = static_cast<float>(goal.v.x);
    msg.velocity[1] = static_cast<float>(goal.v.y);
    msg.velocity[2] = static_cast<float>(goal.v.z);
    msg.acceleration[0] = static_cast<float>(goal.a.x);
    msg.acceleration[1] = static_cast<float>(goal.a.y);
    msg.acceleration[2] = static_cast<float>(goal.a.z);
    msg.jerk[0] = static_cast<float>(goal.j.x);
    msg.jerk[1] = static_cast<float>(goal.j.y);
    msg.jerk[2] = static_cast<float>(goal.j.z);
  }

  if (yaw_mode_ == "none" || yaw_mode_ == "nan") {
    msg.yaw = std::numeric_limits<float>::quiet_NaN();
    msg.yawspeed = std::numeric_limits<float>::quiet_NaN();
  } else if (yaw_mode_ == "zero") {
    msg.yaw = 0.0f;
    msg.yawspeed = 0.0f;
  } else if (yaw_mode_ == "fixed") {
    if (enable_enu_ned_conversion_) {
      msg.yaw = static_cast<float>(enuYawToNed(fixed_yaw_rad_));
    } else {
      msg.yaw = static_cast<float>(fixed_yaw_rad_);
    }
    msg.yawspeed = 0.0f;
  } else {
    if (enable_enu_ned_conversion_) {
      msg.yaw = static_cast<float>(enuYawToNed(goal.yaw));
      if (enable_yawspeed_feedforward_) {
        msg.yawspeed = static_cast<float>(-goal.dyaw);
      } else {
        msg.yawspeed = std::numeric_limits<float>::quiet_NaN();
      }
    } else {
      msg.yaw = static_cast<float>(goal.yaw);
      if (enable_yawspeed_feedforward_) {
        msg.yawspeed = static_cast<float>(goal.dyaw);
      } else {
        msg.yawspeed = std::numeric_limits<float>::quiet_NaN();
      }
    }
  }

  setpoint_pub_->publish(msg);

  if (publish_setpoint_marker_ && setpoint_marker_pub_) {
    publishSetpointMarker(goal);
  }

  if (publish_velocity_text_ && velocity_text_pub_ && has_odom_) {
    const double v_des = std::sqrt(
      goal.v.x * goal.v.x + goal.v.y * goal.v.y + goal.v.z * goal.v.z);
    double v_act = 0.0;
    Vector3 drone_pos{0.0, 0.0, 0.0};
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      v_act = std::sqrt(
        latest_odom_.velocity[0] * latest_odom_.velocity[0] +
        latest_odom_.velocity[1] * latest_odom_.velocity[1] +
        latest_odom_.velocity[2] * latest_odom_.velocity[2]);
      Vector3 p_ned{
        static_cast<double>(latest_odom_.position[0]),
        static_cast<double>(latest_odom_.position[1]),
        static_cast<double>(latest_odom_.position[2])};
      drone_pos = enable_enu_ned_conversion_ ? nedToEnu(p_ned) : p_ned;
    }
    publishVelocityTextMarker(drone_pos, v_act, v_des);
  }

  if (source == SetpointSource::kHold) {
    RCLCPP_DEBUG_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "publishing position-hold setpoint at NED (%.2f, %.2f, %.2f)",
      msg.position[0], msg.position[1], msg.position[2]);
  }
}

void Px4SandoBridgeNode::publishSetpointMarker(
  const dynus_interfaces::msg::Goal & goal)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = this->now();
  marker.header.frame_id = world_frame_id_;
  marker.ns = "px4_desired_setpoint";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = goal.p.x;
  marker.pose.position.y = goal.p.y;
  marker.pose.position.z = goal.p.z;
  marker.pose.orientation.w = 1.0;

  const double s = std::max(0.05, setpoint_marker_scale_);
  marker.scale.x = s;
  marker.scale.y = s;
  marker.scale.z = s;

  if (setpoint_marker_color_rgb_.size() >= 3) {
    marker.color.r = static_cast<float>(setpoint_marker_color_rgb_[0]);
    marker.color.g = static_cast<float>(setpoint_marker_color_rgb_[1]);
    marker.color.b = static_cast<float>(setpoint_marker_color_rgb_[2]);
  } else {
    marker.color.r = 0.0f;
    marker.color.g = 0.8f;
    marker.color.b = 1.0f;  // Cyan default
  }
  marker.color.a = 0.9f;

  setpoint_marker_pub_->publish(marker);
}

void Px4SandoBridgeNode::publishVelocityTextMarker(
  const Vector3 & drone_pos_enu, double v_act, double v_des)
{
  visualization_msgs::msg::Marker marker;
  marker.header.stamp = this->now();
  marker.header.frame_id = world_frame_id_;
  marker.ns = "drone_speed";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose.position.x = drone_pos_enu[0];
  marker.pose.position.y = drone_pos_enu[1];
  marker.pose.position.z = drone_pos_enu[2] + velocity_text_offset_z_;
  marker.pose.orientation.w = 1.0;

  marker.scale.z = std::max(0.1, velocity_text_scale_);

  // Bright cyan/yellow text
  marker.color.r = 0.2f;
  marker.color.g = 0.9f;
  marker.color.b = 1.0f;
  marker.color.a = 1.0f;

  std::ostringstream ss;
  ss << std::fixed << std::setprecision(2);
  ss << "v: " << v_act << " m/s (cmd: " << v_des << " m/s)";
  marker.text = ss.str();

  velocity_text_pub_->publish(marker);
}

void Px4SandoBridgeNode::publishOffboardControlMode()
{
  px4_msgs::msg::OffboardControlMode msg{};
  msg.timestamp = nowUs();
  msg.position = true;
  msg.velocity = true;
  msg.acceleration = enable_acceleration_feedforward_;
  msg.attitude = false;
  msg.body_rate = false;
  msg.thrust_and_torque = false;
  msg.direct_actuator = false;
  offboard_mode_pub_->publish(msg);
}

void Px4SandoBridgeNode::sendVehicleCommand(
  uint32_t command, float param1, float param2, float param3, float param4,
  float param5, float param6, float param7)
{
  px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = nowUs();
  msg.command = command;
  msg.param1 = param1;
  msg.param2 = param2;
  msg.param3 = param3;
  msg.param4 = param4;
  msg.param5 = param5;
  msg.param6 = param6;
  msg.param7 = param7;
  msg.target_system = 1;
  msg.target_component = 1;
  msg.source_system = 1;
  msg.source_component = 1;
  msg.confirmation = 0;
  msg.from_external = true;
  vehicle_command_pub_->publish(msg);
}

void Px4SandoBridgeNode::manageAutoFlight()
{
  if (!warmupComplete()) {
    return;
  }

  if (armed_.load() && offboard_.load()) {
    if (auto_flight_state_ != AutoFlightState::kCompleted) {
      RCLCPP_INFO(
        get_logger(),
        "vehicle is armed and in OFFBOARD mode; bridge fully engaged");
      auto_flight_state_ = AutoFlightState::kCompleted;
    }
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  if ((now - last_command_send_time_) < std::chrono::milliseconds(1000)) {
    return;
  }

  double current_z = 0.0;
  if (has_odom_) {
    current_z = enable_enu_ned_conversion_ ? -latest_odom_.position[2] : latest_odom_.position[2];
  }

  if (auto_takeoff_) {
    if (!armed_.load()) {
      RCLCPP_INFO(get_logger(), "auto-takeoff: arming vehicle...");
      sendVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        1.0F);
      last_command_send_time_ = now;
      auto_flight_state_ = AutoFlightState::kArming;
      return;
    }

    if (current_z < (takeoff_altitude_m_ - 0.4)) {
      if (has_status_ &&
          latest_status_.nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF)
      {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "auto-takeoff: climbing... current_alt=%.2f m, target=%.2f m",
          current_z, takeoff_altitude_m_);
        return;
      }

      RCLCPP_INFO(
        get_logger(),
        "auto-takeoff: sending TAKEOFF command (target_alt=%.2f m, current_alt=%.2f m)...",
        takeoff_altitude_m_, current_z);
      const float nan_val = std::numeric_limits<float>::quiet_NaN();
      sendVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF,
        nan_val, nan_val, nan_val, nan_val, nan_val, nan_val, nan_val);
      last_command_send_time_ = now;
      auto_flight_state_ = AutoFlightState::kTakingOff;
      return;
    }

    if (!offboard_.load()) {
      RCLCPP_INFO(
        get_logger(),
        "auto-takeoff: target altitude reached (%.2f m); switching to OFFBOARD mode...",
        current_z);
      sendVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        1.0F,
        6.0F);
      last_command_send_time_ = now;
      auto_flight_state_ = AutoFlightState::kSwitchingOffboard;
      return;
    }
  } else if (auto_arm_and_offboard_ || engage_requested_.load()) {
    if (!armed_.load()) {
      sendVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
        1.0F);
      RCLCPP_INFO(get_logger(), "sent arm command");
    } else if (!offboard_.load()) {
      sendVehicleCommand(
        px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE,
        1.0F,
        6.0F);
      RCLCPP_INFO(get_logger(), "requested OFFBOARD mode");
    }
    last_command_send_time_ = now;
  }
}

bool Px4SandoBridgeNode::warmupComplete() const
{
  if (heartbeat_count_ == 0 && setpoint_count_ == 0) {
    return false;
  }

  const auto required = static_cast<uint64_t>(
    std::ceil(offboard_warmup_duration_sec_ * heartbeat_rate_hz_));
  const auto elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - node_start_time_).count();

  return heartbeat_count_ >= required &&
         setpoint_count_ >= required &&
         elapsed >= offboard_warmup_duration_sec_;
}

uint64_t Px4SandoBridgeNode::nowUs() const
{
  timestamp_sync_.updateClocks();
  return timestamp_sync_.px4NowUs();
}

void Px4SandoBridgeNode::handleEngage(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  engage_requested_.store(true);
  manual_override_.store(false);
  setpoint_stream_enabled_.store(true);
  warned_manual_override_ = false;
  watchdog_.reset();
  auto_flight_state_ = AutoFlightState::kIdle;

  response->success = true;
  response->message =
    "engage requested: streaming setpoints and managing auto flight";
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void Px4SandoBridgeNode::handleDisarm(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  engage_requested_.store(false);
  setpoint_stream_enabled_.store(false);
  manual_override_.store(false);
  state_publication_started_.store(false);

  sendVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    0.0F);

  response->success = true;
  response->message = "disarm command sent; setpoint streaming stopped";
  RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
}

void Px4SandoBridgeNode::handleArm(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  sendVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM,
    1.0F);

  response->success = true;
  response->message = "arm command sent";
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

void Px4SandoBridgeNode::handleTakeoff(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
  std::shared_ptr<std_srvs::srv::Trigger::Response> res)
{
  handleEngage(req, res);
}

void Px4SandoBridgeNode::handleLand(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  sendVehicleCommand(
    px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND);

  response->success = true;
  response->message = "land command sent";
  RCLCPP_INFO(get_logger(), "%s", response->message.c_str());
}

}  // namespace px4_sando_bridge
