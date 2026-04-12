# gazebo_simulation

Gazebo simulation for 2R robot and mobile robot — supports both Gazebo Classic (v11) and Gazebo Fortress, with ros2_control-based joint control.

## Contents

### URDFs

| File | Description |
|---|---|
| `urdf/mobile_robot.urdf` | Mobile robot for Gazebo Fortress (default) |
| `urdf/two_r_robot.urdf` | 2R robot for Gazebo Fortress (default) |
| `urdf/mobile_robot_classic.urdf` | Mobile robot for Gazebo Classic |
| `urdf/two_r_robot_classic.urdf` | 2R robot for Gazebo Classic |

### Launch files

| File | Description |
|---|---|
| `launch/bringup_mobile_robot.launch.py` | Spawn mobile robot in Gazebo Fortress |
| `launch/bringup_two_r_robot.launch.py` | Spawn 2R robot in Gazebo Fortress |
| `launch/bringup_classic_mobile_robot.launch.py` | Spawn mobile robot in Gazebo Classic |
| `launch/bringup_classic_two_r_robot.launch.py` | Spawn 2R robot in Gazebo Classic |
| `launch/control_mobile_robot.launch.py` | Load controllers for mobile robot (Gazebo-agnostic) |
| `launch/control_two_r_robot.launch.py` | Load controllers for 2R robot (Gazebo-agnostic) |

### Config

| File | Description |
|---|---|
| `config/mobile_robot_controllers.yaml` | `joint_state_broadcaster` + `diff_drive_controller` |
| `config/two_r_robot_controllers.yaml` | `joint_state_broadcaster` + `joint_trajectory_controller` |

### Scripts

| File | Description |
|---|---|
| `gazebo_simulation/trajectory_publisher.py` | Send a 3-waypoint trajectory to the 2R robot |

## Differences from `02_robot_description`

The Gazebo URDFs add three things the RViz-only URDFs don't have:

| Addition | Why |
|---|---|
| `<collision>` | Physics engine needs geometry for contact |
| `<inertial>` (mass + inertia tensor) | Physics engine needs this to simulate dynamics |
| `<ros2_control>` + `<gazebo>` plugin | Actuators wired to ros2_control hardware interface |

The 2R robot also adds a `world` link with a fixed joint to anchor it to the ground.

## Classic vs Fortress

The `control_*.launch.py` files are **Gazebo-agnostic** — they only communicate with `controller_manager` over ROS 2. The difference between Classic and Fortress is only in the URDF hardware plugin:

| | Gazebo Classic | Gazebo Fortress |
|---|---|---|
| ROS package | `gazebo_ros` | `ros_gz_sim` |
| ros2_control hardware plugin | `gazebo_ros2_control/GazeboSystem` | `gz_ros2_control/GazeboSimSystem` |
| ros2_control Gazebo plugin | `libgazebo_ros2_control.so` | `gz_ros2_control-system` |
| LiDAR plugin | `libgazebo_ros_ray_sensor.so` | `gpu_lidar` sensor type |
| Spawn | `bringup_entity.py` | `ros_gz_sim create` |

## Dependencies

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
colcon build --packages-select gazebo_simulation --symlink-install
source install/setup.bash
```

## Run

### (Gazebo Fortress) 2R robot

```bash
# Terminal 1 — Gazebo + robot
ros2 launch gazebo_simulation bringup_two_r_robot.launch.py

# Terminal 2 — load controllers
ros2 launch gazebo_simulation control_two_r_robot.launch.py
```

## (Gazebo Fortress) Interact with 2R robot 

Send a trajectory via the script:

```bash
ros2 run gazebo_simulation trajectory_publisher
```

Or publish manually:

```bash
ros2 topic pub /joint_trajectory_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint_1, joint_2], points: [{positions: [0.5, 0.5], time_from_start: {sec: 2}}]}"
```

Or publish trhough rqt gui:

```bash
ros2 run rqt_joint_trajectory_controller rqt_joint_trajectory_controller --force-discover
```

View joint states:

```bash
ros2 topic echo /joint_states
```

---

## Run — Mobile robot (Gazebo Fortress)

```bash
# Terminal 1 — Gazebo + robot
ros2 launch gazebo_simulation bringup_mobile_robot.launch.py

# Terminal 2 — load controllers
ros2 launch gazebo_simulation control_mobile_robot.launch.py
```

## Interact with mobile robot

Drive via `diff_drive_controller`:

```bash
ros2 topic pub /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

View LiDAR:

```bash
ros2 topic echo /scan
```

View odometry:

```bash
ros2 topic echo /diff_drive_controller/odom
```
