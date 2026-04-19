# 04 Sensor Simulation

Simulates a differential-drive mobile robot equipped with a **2D LiDAR** and an **RGB camera** in both Gazebo Fortress and Gazebo Classic.

---

## Package structure

```
04_sensor/
├── config/
│   ├── bridge.yaml              # ros_gz_bridge topic config (Fortress)
│   └── sensor_robot.rviz        # RViz2 config (shared by both simulators)
├── launch/
│   ├── sim_sensor_robot.launch.py          # Gazebo Fortress
│   └── sim_sensor_robot_classic.launch.py  # Gazebo Classic
├── sensor/
│   ├── lidar_subscriber.py      # Subscribes to /scan, logs range stats
│   └── camera_subscriber.py    # Subscribes to /camera/image_raw, logs frame info
├── urdf/
│   ├── sensor_robot.urdf        # Robot with Fortress sensor plugins
│   └── sensor_robot_classic.urdf # Robot with Classic sensor plugins
└── worlds/
    ├── sensor_world.sdf         # Gazebo Fortress world
    └── sensor_world_classic.world # Gazebo Classic world
```

---

## Robot description

| Link | Description |
|------|-------------|
| `base_link` | Main body (0.3 × 0.2 × 0.1 m box) |
| `left_wheel` / `right_wheel` | Drive wheels (r = 0.06 m, separation = 0.25 m) |
| `caster_wheel` | Frictionless rear caster |
| `laser_frame` | LiDAR mount, 0.1 m forward, 0.075 m above base |
| `camera_frame` | Camera mount, 0.16 m forward, 0.06 m above base |

### LiDAR

| Parameter | Value |
|-----------|-------|
| Type | 2D GPU LiDAR |
| Range | 0.1 – 10.0 m |
| Resolution | 0.01 m |
| Beams | 360 (1° per beam) |
| Rate | 10 Hz |

### RGB Camera

| Parameter | Value |
|-----------|-------|
| Resolution | 640 × 480 |
| FOV | 60° horizontal |
| Rate | 30 Hz |
| Format | R8G8B8 |

---

## Gazebo Fortress

### Launch

```bash
ros2 launch sensor sim_sensor_robot.launch.py
```

### Topics

| Topic | Type | Direction |
|-------|------|-----------|
| `/scan` | `sensor_msgs/msg/LaserScan` | Gz → ROS |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | Gz → ROS |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Gz → ROS |
| `/odom` | `nav_msgs/msg/Odometry` | Gz → ROS |
| `/tf` | `tf2_msgs/msg/TFMessage` | Gz → ROS |
| `/joint_states` | `sensor_msgs/msg/JointState` | Gz → ROS |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | ROS → Gz |

Bridge configuration is defined in `config/bridge.yaml`.

---

## Gazebo Classic

### Launch

```bash
ros2 launch sensor sim_sensor_robot_classic.launch.py
```

### Topics

Same topic names as Fortress. No bridge is needed — Classic plugins publish directly to ROS 2 topics.

| Plugin | File |
|--------|------|
| Diff drive | `libgazebo_ros_diff_drive.so` |
| LiDAR | `libgazebo_ros_ray_sensor.so` |
| RGB Camera | `libgazebo_ros_camera.so` |
| Joint states | `libgazebo_ros_joint_state_publisher.so` |

---

## Subscriber nodes

### LiDAR subscriber

Logs minimum and maximum range from each scan.

```bash
ros2 run sensor lidar_subscriber
```

### Camera subscriber

Logs frame number, resolution, encoding, and timestamp for each image.

```bash
ros2 run sensor camera_subscriber
```

---

## Teleoperation

Drive the robot with keyboard:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| Key | Action |
|-----|--------|
| `i` | Forward |
| `,` | Backward |
| `j` | Rotate left |
| `l` | Rotate right |
| `k` | Stop |
