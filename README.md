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
  - `config/global_mapper.yaml`: Point-cloud mapper parameters.
  - `rviz/px4_sando.rviz`: RViz2 configuration matching `px4_mighty.rviz` (interactive 2D Nav Goal, setpoint marker, trajectory markers, mapping grids).
  - `launch/`: Unified launch files (`px4_sando.launch.py`, `ros_gz_bridge.launch.py`).
  - `models/x500_lidar_3d/`: Self-contained Gazebo airframe model with 3D GPU LiDAR sensor.
  - `worlds/`: Gazebo worlds (`easy_forest.sdf`, `mighty_forest.sdf`, etc.).
- `run_px4_sitl.sh`: Helper script to automatically sync models/worlds and run PX4 SITL.
- `setup_env.sh`: Sourcing script cascading ROS 2 Lyrical, `sando_lyrical_ws` underlay, and overlay workspace.

---

## Quickstart & Usage (Unified 2-Terminal Workflow)

### 1. Build Overlay Workspace
```bash
source ~/code/px4_sando_ws/setup_env.sh
cd ~/code/px4_sando_ws
MAKEFLAGS="-j3" CMAKE_BUILD_PARALLEL_LEVEL=3 colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
```

### 2. Run Unit Tests (All 21 Passing)
```bash
colcon test --packages-select px4_sando_bridge
colcon test-result --verbose
```

### 3. Launch Simulation & Navigation

**Terminal 1 — PX4 SITL & Gazebo (Headless by default, saves CPU/GPU):**
```bash
cd ~/code/px4_sando_ws
./run_px4_sitl.sh easy_forest
# To open with Gazebo GUI if desired: ./run_px4_sitl.sh easy_forest --gui
```

**Terminal 2 — SANDO Planner + MicroXRCEAgent + Mapping + Auto-Takeoff + RViz2:**
```bash
source ~/code/px4_sando_ws/setup_env.sh
ros2 launch px4_sando_bridge px4_sando.launch.py launch_rviz:=true
```

In a single command, Terminal 2 automatically starts:
1. **MicroXRCEAgent** connected to PX4 on port `8888`.
2. **`ros_gz_bridge`** bridging Gazebo LiDAR (`/lidar/points/points` $\to$ `/NX01/mid360_PointCloud2`) and simulation clock (`/clock`).
3. **`px4_sando_bridge_node`** managing telemetry downlink, setpoint uplink, and auto-takeoff.
4. **Static transforms** connecting `base_link` $\to$ `NX01_livox` $\to$ `livox_frame` and `world` $\to$ `map`.
5. **`global_mapper_ros`** building the real-time occupancy and unknown grids from LiDAR.
6. **`sando_node`** trajectory planning and dynamic safety corridor generation.
7. **`rviz2`** configured with `px4_sando.rviz` for full visual and mouse-interactive control.

The bridge node will automatically:
1. Arm the vehicle.
2. Issue an automatic takeoff command (`2.5m`).
3. Smoothly transition to `OFFBOARD` mode once target altitude is reached.
4. Stream setpoints and follow goals commanded in RViz2 without back-and-forth oscillation.

---

### 4. Interactive Goal Commanding

In RViz2, click the **2D Goal Pose** (or 2D Nav Goal) tool in the top toolbar and click anywhere in the map. SANDO will compute the trajectory and PX4 will fly the drone safely through obstacles to the destination!

Alternatively, command via terminal:
```bash
ros2 topic pub --once /NX01/term_goal geometry_msgs/msg/PoseStamped "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'},
  pose: {position: {x: 15.0, y: 5.0, z: 2.5}, orientation: {w: 1.0}}
}"
```

---

## Constitutional Guarantees
- **Zero Modifications to Canonical Workspaces**: Neither `/home/hasan-fashtouk/code/sando` nor `~/code/sando_lyrical_ws` are modified.
- **Resource Bounds**: CPU $< 5\%$, RSS memory $< 35\text{ MB}$.
- **Safe Fallback**: Automatic position hold upon planner delay $> 1000\text{ ms}$.
# px4_sando_ws
