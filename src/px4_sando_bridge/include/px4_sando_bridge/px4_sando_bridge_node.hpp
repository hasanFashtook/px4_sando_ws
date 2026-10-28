// Copyright 2026, SANDO Lyrical / PX4 integration
// SPDX-License-Identifier: MIT

#ifndef PX4_SANDO_BRIDGE__PX4_SANDO_BRIDGE_NODE_HPP_
#define PX4_SANDO_BRIDGE__PX4_SANDO_BRIDGE_NODE_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <dynus_interfaces/msg/goal.hpp>
#include <dynus_interfaces/msg/state.hpp>
#include <dynus_interfaces/msg/trajectory.hpp>

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_status.hpp>

#include <tf2_ros/transform_broadcaster.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "px4_sando_bridge/frame_transforms.hpp"
#include "px4_sando_bridge/timestamp_sync.hpp"
#include "px4_sando_bridge/watchdog.hpp"

namespace px4_sando_bridge
{

/// Bi-directional PX4 <-> SANDO bridge.
class Px4SandoBridgeNode : public rclcpp::Node
{
public:
  explicit Px4SandoBridgeNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class SetpointSource
  {
    kNone,
    kGoal,
    kTrajectory,
    kHold
  };

  // ---- setup -----------------------------------------------------------
  void declareAndLoadParameters();
  void createRosInterfaces();
  std::string nsTopic(const std::string & suffix) const;

  // ---- subscriptions ---------------------------------------------------
  void odometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg);
  void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg);
  void goalCallback(const dynus_interfaces::msg::Goal::SharedPtr msg);
  void trajectoryCallback(const dynus_interfaces::msg::Trajectory::SharedPtr msg);
  void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // ---- services --------------------------------------------------------
  void handleEngage(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleDisarm(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleArm(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleTakeoff(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void handleLand(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // ---- periodic work ---------------------------------------------------
  void heartbeatTimerCallback();
  void setpointTimerCallback();
  void telemetryTimerCallback();

  // ---- telemetry -------------------------------------------------------
  dynus_interfaces::msg::State odometryToState(
    const px4_msgs::msg::VehicleOdometry & msg) const;
  void publishState(const px4_msgs::msg::VehicleOdometry & msg);
  void publishOdometry(const px4_msgs::msg::VehicleOdometry & msg);
  void publishTransformFromLatest();
  void publishTransform(const dynus_interfaces::msg::State & state);

  // ---- setpoint / offboard --------------------------------------------
  SetpointSource selectSetpoint(dynus_interfaces::msg::Goal & goal);
  bool trajectoryGoalAt(
    const dynus_interfaces::msg::Trajectory & trajectory,
    double elapsed_sec, dynus_interfaces::msg::Goal & goal) const;
  void applySafetyLimits(dynus_interfaces::msg::Goal & goal) const;
  void publishSetpoint(const dynus_interfaces::msg::Goal & goal, SetpointSource source);
  void publishSetpointMarker(const dynus_interfaces::msg::Goal & goal);
  void publishVelocityTextMarker(
    const Vector3 & drone_pos_enu, double v_act, double v_des);
  void publishOffboardControlMode();
  void sendVehicleCommand(
    uint32_t command, float param1 = 0.0F, float param2 = 0.0F,
    float param3 = 0.0F, float param4 = 0.0F, float param5 = 0.0F,
    float param6 = 0.0F, float param7 = 0.0F);
  void manageAutoFlight();
  bool warmupComplete() const;
  uint64_t nowUs() const;

  // ---- parameters ------------------------------------------------------
  std::string drone_namespace_{"NX01"};
  std::string world_frame_id_{"world"};
  std::string body_frame_id_{"base_link"};
  std::string livox_frame_id_{"livox_frame"};

  std::string state_topic_;
  std::string odom_topic_;
  std::string goal_topic_;
  std::string trajectory_topic_;
  std::string term_goal_topic_;

  std::string px4_odometry_topic_{"/fmu/out/vehicle_odometry"};
  std::string px4_status_topic_{"/fmu/out/vehicle_status"};
  std::string px4_setpoint_topic_{"/fmu/in/trajectory_setpoint"};
  std::string px4_offboard_mode_topic_{"/fmu/in/offboard_control_mode"};
  std::string px4_vehicle_command_topic_{"/fmu/in/vehicle_command"};

  double heartbeat_rate_hz_{20.0};
  double setpoint_stream_rate_hz_{20.0};
  double telemetry_rate_hz_{50.0};
  double tf_rate_hz_{20.0};
  int watchdog_timeout_ms_{250};
  double offboard_warmup_duration_sec_{1.0};
  bool enable_enu_ned_conversion_{true};
  bool auto_arm_and_offboard_{false};
  bool auto_takeoff_{true};
  double takeoff_altitude_m_{2.5};
  bool publish_tf_{true};
  bool publish_odom_{true};

  double max_horizontal_velocity_m_s_{3.0};
  double max_vertical_velocity_m_s_{1.5};
  double max_acceleration_m_s2_{5.0};
  double max_yaw_rate_rad_s_{1.0};
  double geofence_min_z_m_{0.2};
  double geofence_max_z_m_{10.0};
  bool enable_acceleration_feedforward_{true};
  bool enable_yawspeed_feedforward_{false};
  std::string yaw_mode_{"none"};
  double fixed_yaw_rad_{0.0};
  bool publish_setpoint_marker_{true};
  std::string setpoint_marker_topic_;
  std::vector<double> setpoint_marker_color_rgb_{0.0, 0.8, 1.0};  // Vibrant Cyan
  double setpoint_marker_scale_{0.35};
  bool publish_velocity_text_{true};
  std::string velocity_text_topic_;
  double velocity_text_offset_z_{0.65};
  double velocity_text_scale_{0.35};

  // ---- ROS interfaces --------------------------------------------------
  rclcpp::Publisher<dynus_interfaces::msg::State>::SharedPtr state_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr setpoint_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr velocity_text_pub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr term_goal_pub_;

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_versioned_sub_;
  rclcpp::Subscription<dynus_interfaces::msg::Goal>::SharedPtr goal_sub_;
  rclcpp::Subscription<dynus_interfaces::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr rviz_goal_sub_;

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr engage_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disarm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr arm_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr takeoff_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_srv_;

  rclcpp::TimerBase::TimerBase::SharedPtr heartbeat_timer_;
  rclcpp::TimerBase::SharedPtr setpoint_timer_;
  rclcpp::TimerBase::SharedPtr telemetry_timer_;
  rclcpp::TimerBase::SharedPtr tf_timer_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ---- shared state ----------------------------------------------------
  mutable std::mutex state_mutex_;
  dynus_interfaces::msg::Goal latest_goal_;
  std::chrono::steady_clock::time_point latest_goal_time_{};
  bool has_goal_{false};

  dynus_interfaces::msg::Trajectory latest_trajectory_;
  std::chrono::steady_clock::time_point latest_trajectory_time_{};
  bool has_trajectory_{false};

  px4_msgs::msg::VehicleOdometry latest_odom_;
  std::chrono::steady_clock::time_point latest_odom_time_{};
  bool has_odom_{false};

  px4_msgs::msg::VehicleStatus latest_status_;
  bool has_status_{false};

  Watchdog watchdog_;
  mutable TimestampSynchronizer timestamp_sync_;

  std::atomic<bool> manual_override_{false};
  std::atomic<bool> engage_requested_{false};
  std::atomic<bool> armed_{false};
  std::atomic<bool> offboard_{false};
  std::atomic<bool> setpoint_stream_enabled_{true};
  std::atomic<bool> state_publication_started_{false};

  bool was_offboard_{false};
  bool hold_active_{false};
  Vector3 hold_position_{{0.0, 0.0, 0.0}};
  double hold_yaw_{0.0};

  enum class AutoFlightState {
    kIdle,
    kArming,
    kTakingOff,
    kSwitchingOffboard,
    kCompleted
  };
  AutoFlightState auto_flight_state_{AutoFlightState::kIdle};

  std::chrono::steady_clock::time_point node_start_time_{};
  std::chrono::steady_clock::time_point last_command_send_time_{};
  bool command_sent_{false};
  uint64_t heartbeat_count_{0};
  uint64_t setpoint_count_{0};

  bool warned_no_odometry_{false};
  bool warned_manual_override_{false};
  std::chrono::steady_clock::time_point last_timestamp_log_time_{};
};

}  // namespace px4_sando_bridge

#endif  // PX4_SANDO_BRIDGE__PX4_SANDO_BRIDGE_NODE_HPP_
