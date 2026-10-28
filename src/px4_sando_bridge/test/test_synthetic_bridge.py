#!/usr/bin/env python3
"""
Synthetic test script verifying bidirectional communication rates and coordinate transformations
for px4_sando_bridge.
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from px4_msgs.msg import VehicleOdometry, TrajectorySetpoint, OffboardControlMode
from dynus_interfaces.msg import State, Goal

class SyntheticBridgeTester(Node):
    def __init__(self):
        super().__init__('synthetic_bridge_tester')

        best_effort_qos = QoSProfile(depth=10)
        best_effort_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        best_effort_qos.durability = DurabilityPolicy.VOLATILE

        reliable_qos = QoSProfile(depth=10)
        reliable_qos.reliability = ReliabilityPolicy.RELIABLE

        # Publishers to bridge
        self.odom_pub = self.create_publisher(
            VehicleOdometry, '/fmu/out/vehicle_odometry', best_effort_qos)
        self.goal_pub = self.create_publisher(
            Goal, '/NX01/goal', reliable_qos)

        # Subscribers from bridge
        self.state_sub = self.create_subscription(
            State, '/NX01/state', self.on_state, reliable_qos)
        self.setpoint_sub = self.create_subscription(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.on_setpoint, best_effort_qos)
        self.heartbeat_sub = self.create_subscription(
            OffboardControlMode, '/fmu/in/offboard_control_mode', self.on_heartbeat, best_effort_qos)

        self.state_count = 0
        self.setpoint_count = 0
        self.heartbeat_count = 0

        self.last_state = None
        self.last_setpoint = None

        # 100 Hz timer simulating PX4 telemetry output
        self.odom_timer = self.create_timer(0.01, self.publish_fake_odometry)

        # 50 Hz timer simulating SANDO planner goal output
        self.goal_timer = self.create_timer(0.02, self.publish_fake_goal)

        self.start_time = time.time()

    def publish_fake_odometry(self):
        msg = VehicleOdometry()
        msg.timestamp = int(time.time() * 1e6)
        # Simulated NED position: North=10.0m, East=5.0m, Down=-2.5m (Altitude 2.5m)
        msg.position = [10.0, 5.0, -2.5]
        # Simulated NED velocity: North=1.0m/s, East=2.0m/s, Down=-0.5m/s
        msg.velocity = [1.0, 2.0, -0.5]
        # Identity quaternion [w=1, x=0, y=0, z=0]
        msg.q = [1.0, 0.0, 0.0, 0.0]
        msg.angular_velocity = [0.0, 0.0, 0.0]
        self.odom_pub.publish(msg)

    def publish_fake_goal(self):
        msg = Goal()
        # Simulated ENU target: East=5.0m, North=10.0m, Up=2.5m
        msg.p.x = 5.0
        msg.p.y = 10.0
        msg.p.z = 2.5
        msg.v.x = 2.0
        msg.v.y = 1.0
        msg.v.z = 0.5
        msg.yaw = 0.0  # East
        msg.dyaw = 0.1
        self.goal_pub.publish(msg)

    def on_state(self, msg: State):
        self.state_count += 1
        self.last_state = msg

    def on_setpoint(self, msg: TrajectorySetpoint):
        self.setpoint_count += 1
        self.last_setpoint = msg

    def on_heartbeat(self, msg: OffboardControlMode):
        self.heartbeat_count += 1

def main():
    rclpy.init()
    tester = SyntheticBridgeTester()

    print("[Synthetic Test] Running synthetic telemetry and setpoint stream for 3.0 seconds...")
    test_duration = 3.0
    start = time.time()

    while rclpy.ok() and (time.time() - start) < test_duration:
        rclpy.spin_once(tester, timeout_sec=0.01)

    elapsed = time.time() - start
    state_rate = tester.state_count / elapsed
    setpoint_rate = tester.setpoint_count / elapsed
    heartbeat_rate = tester.heartbeat_count / elapsed

    print(f"\n--- Synthetic Benchmark Results ({elapsed:.2f} s) ---")
    print(f"State Messages Received: {tester.state_count} ({state_rate:.1f} Hz) [Target >= 50 Hz]")
    print(f"Trajectory Setpoints:     {tester.setpoint_count} ({setpoint_rate:.1f} Hz) [Target >= 20 Hz]")
    print(f"Heartbeat Messages:       {tester.heartbeat_count} ({heartbeat_rate:.1f} Hz) [Target ~ 20 Hz]")

    passed = True
    if state_rate < 40.0:
        print("FAIL: State rate below acceptable threshold")
        passed = False
    else:
        print("PASS: State publication rate verified")

    if setpoint_rate < 15.0:
        print("FAIL: Setpoint rate below acceptable threshold")
        passed = False
    else:
        print("PASS: Setpoint publication rate verified")

    # Verify coordinate transformation
    if tester.last_state is not None:
        # NED (North=10, East=5, Down=-2.5) -> ENU (East=5, North=10, Up=2.5)
        print(f"State ENU position: ({tester.last_state.pos.x:.2f}, {tester.last_state.pos.y:.2f}, {tester.last_state.pos.z:.2f})")
        assert abs(tester.last_state.pos.x - 5.0) < 0.1, "ENU East mapping error"
        assert abs(tester.last_state.pos.y - 10.0) < 0.1, "ENU North mapping error"
        assert abs(tester.last_state.pos.z - 2.5) < 0.1, "ENU Up mapping error"
        print("PASS: ENU coordinates accurately match NED telemetry")

    if tester.last_setpoint is not None:
        # ENU (East=5, North=10, Up=2.5) -> NED (North=10, East=5, Down=-2.5)
        print(f"Setpoint NED position: ({tester.last_setpoint.position[0]:.2f}, {tester.last_setpoint.position[1]:.2f}, {tester.last_setpoint.position[2]:.2f})")
        assert abs(tester.last_setpoint.position[0] - 10.0) < 0.1, "NED North setpoint error"
        assert abs(tester.last_setpoint.position[1] - 5.0) < 0.1, "NED East setpoint error"
        assert abs(tester.last_setpoint.position[2] - (-2.5)) < 0.1, "NED Down setpoint error"
        print("PASS: NED setpoint coordinates accurately match ENU goal")

    tester.destroy_node()
    rclpy.shutdown()

    if passed:
        print("\nAll synthetic bridge checks PASSED successfully.")
        return 0
    else:
        print("\nChecks FAILED.")
        return 1

if __name__ == '__main__':
    exit(main())
