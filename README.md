# PX4-SANDO Integration Workspace (`px4_sando_ws`)

Dedicated, isolated ROS 2 overlay workspace integrating the **SANDO** trajectory planner with **PX4 Autopilot** (Micro XRCE-DDS) and Gazebo Sim Harmonic (Ubuntu 26.04 / ROS 2 Lyrical Luth).

---

## Architecture Overview

```
                      +-----------------------------+
                      |   PX4 Autopilot (SITL/HW)   |
                      +--------------+--------------+
                                     | Micro XRCE-DDS (UDP 8888)
                                     v
                      +-----------------------------+
                      |      MicroXRCEAgent         |
                      +--------------+--------------+
                                     | Native ROS 2 Topics (/fmu/...)
                                     v
+-----------------------------------------------------------------------+
|  px4_sando_bridge_node (px4_sando_ws)                                 |
|                                                                       |
|  * NED -> ENU Telemetry Downlink:                                     |
|      /fmu/out/vehicle_odometry -> /NX01/state (50+ Hz)                |
|                                -> /NX01/odom                          |
|  * ENU -> NED Setpoint Uplink:                                        |
|      /NX01/goal               -> /fmu/in/trajectory_setpoint (20 Hz)  |
|  * Offboard Heartbeat:                                                |
|      -> /fmu/in/offboard_control_mode (20 Hz)                         |
|  * Arming & Mode Management:                                          |
|      -> /fmu/in/vehicle_command (Arm / Offboard)                      |
|  * Watchdog Safety Fallback:                                          |
|      Staleness > 250 ms -> Position Hold at latest confirmed position |
+-----------------------------------------------------------------------+
         ^                          |
         | /NX01/state              | /NX01/goal
         v                          v
+------------------+       +------------------+
|    sando_node    | <---> | global_mapper_ros|
+------------------+       +------------------+
                                    ^
                                    | /NX01/mid360_PointCloud2
                           +------------------+
                           |  ros_gz_bridge   |
                           +------------------+
                                    ^
                                    | /lidar/points/points
                           +------------------+
                           | Gazebo Harmonic  |
                           |  x500_lidar_3d   |
                           +------------------+
```

---

## Workspace Structure

- `src/px4_msgs`: Official PX4 message definitions compiled for ROS 2.
- `src/px4_sando_bridge`:
  - `include/px4_sando_bridge/`: Header files (`px4_sando_bridge_node.hpp`, `frame_transforms.hpp`, `watchdog.hpp`, `timestamp_sync.hpp`).
  - `src/`: Implementation files.
  - `test/`: Unit tests (21 tests passing) and synthetic integration benchmark.
  - `config/bridge_params.yaml`: Node parameters.
  - `launch/`: Launch files (`px4_sando.launch.py`, `px4_sando_full_sim.launch.py`, `ros_gz_bridge.launch.py`).
  - `models/x500_lidar_3d/`: Self-contained Gazebo airframe model with 3D GPU LiDAR sensor.
  - `worlds/`: Gazebo worlds (`easy_forest.sdf`, `mighty_forest.sdf`, etc.).
- `run_px4_sitl.sh`: Helper script to automatically sync models/worlds and run PX4 SITL.
- `setup_env.sh`: Sourcing script cascading ROS 2 Lyrical, `sando_lyrical_ws` underlay, and overlay workspace.

---

## Quickstart: How to Run PX4 and SANDO

### Step 1: Run PX4 SITL & Gazebo (Terminal 1)

**Headless mode (Default, saves CPU and RAM resources):**
```bash
cd ~/code/px4_sando_ws
./run_px4_sitl.sh easy_forest
```

**With Gazebo 3D GUI window (if you want visual inspection):**
```bash
cd ~/code/px4_sando_ws
./run_px4_sitl.sh easy_forest --gui
```

---

### Step 2: Run Micro XRCE-DDS Agent (Terminal 2)

```bash
cd ~/code/px4_sando_ws
source setup_env.sh
MicroXRCEAgent udp4 -p 8888
```

---

### Step 3: Run SANDO + PX4 Bridge + RViz2 (Terminal 3)

```bash
cd ~/code/px4_sando_ws
source setup_env.sh
ros2 launch px4_sando_bridge px4_sando_full_sim.launch.py launch_rviz:=true
```

The bridge will automatically:
1. Stream 20 Hz heartbeats to PX4.
2. Arm the drone and take off to 2.5 meters.
3. Switch PX4 smoothly to `OFFBOARD` mode.
4. Stream SANDO trajectory goals to PX4 and visualize the cyan setpoint marker in RViz.

---

### Step 4: Command Autonomous Flight Targets

In RViz, use the **2D Nav Goal** tool to click any target on the map, or publish directly via terminal:

```bash
ros2 topic pub --once /NX01/term_goal geometry_msgs/msg/PoseStamped "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'world'},
  pose: {position: {x: 15.0, y: 5.0, z: 2.5}, orientation: {w: 1.0}}
}"
```
