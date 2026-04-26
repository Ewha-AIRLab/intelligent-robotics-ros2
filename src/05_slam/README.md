# 05 SLAM

Demonstrates **online SLAM** and **localization** using [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) (mapping) and [Nav2 AMCL](https://navigation.ros.org/configuration/packages/configuring-amcl.html) (localization) with the sensor robot from package `04_sensor` in a closed indoor environment simulated in Gazebo Fortress.

---

## Package structure

```
05_slam/
├── config/
│   ├── slam_params.yaml    # slam_toolbox online-async (mapping mode)
│   ├── amcl_params.yaml    # Nav2 AMCL + map_server (localization mode)
│   ├── slam.rviz           # RViz2 config for mapping
│   └── localization.rviz   # RViz2 config for localization (+ covariance ellipse)
├── launch/
│   ├── slam.launch.py          # Mapping — build a map from scratch
│   └── localization.launch.py  # Localization — AMCL on a saved map, random spawn
├── worlds/
│   └── slam_world.sdf      # Closed 10 × 10 m indoor room
├── slam/
│   └── __init__.py
├── package.xml
├── setup.cfg
└── setup.py
```

> **Reused from `04_sensor`** (no duplication): `sensor_robot.urdf`, `bridge.yaml`, `sim_sensor_robot.launch.py`

---

## Dependencies

| Package | Role |
|---------|------|
| `sensor` | Robot URDF, bridge config, and simulation launch |
| `ros_gz_sim` | Gazebo Fortress simulator |
| `ros_gz_bridge` | Gazebo ↔ ROS 2 topic bridge |
| `robot_state_publisher` | Publishes TF from the URDF |
| `slam_toolbox` | Online async SLAM (mapping) |
| `nav2_map_server` | Serves the saved map on `/map` |
| `nav2_amcl` | Particle filter localization |
| `nav2_lifecycle_manager` | Activates map_server and amcl |
| `rviz2` | Visualisation |


---

## 1. Mapping (slam_toolbox)

Build a map by driving the robot around the room.

```bash
# Terminal 1 — simulator + slam_toolbox + RViz2
ros2 launch slam slam.launch.py

# Terminal 2 — keyboard teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

| Key | Action |
|-----|--------|
| `i` | Forward |
| `,` | Backward |
| `j` | Rotate left |
| `l` | Rotate right |
| `k` | Stop |

### Saving the map

Once the full room is explored, save the map:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

Produces `my_map.pgm` (occupancy image) and `my_map.yaml` (metadata).

---

## 2. Localization (Nav2)

Spawns the robot at a **random pose** and localises against the saved map using a particle filter.

```bash
# Default map: ~/my_map.yaml
ros2 launch slam localization.launch.py

# Custom map path
ros2 launch slam localization.launch.py map:=/path/to/my_map.yaml

# Terminal 2 — keyboard teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Move around the robot to see how robot localizes.
```bash
# Terminal 2 — keyboard teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


### Correcting a poor initial estimate

If AMCL diverges, use **2D Pose Estimate** in RViz2 to click the robot's true position on the map. This publishes to `/initialpose`, which AMCL uses to reinitialise its particle distribution.

---

## Key parameters

 `config/slam_params.yaml` (mapping)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `base_frame` | `base_link` | Matches the sensor_robot URDF |
| `scan_topic` | `/scan` | 2D LiDAR topic |
| `mode` | `mapping` | Online async map building |
| `resolution` | `0.05` m | 5 cm per map cell |
| `map_update_interval` | `2.0` s | Map regeneration period |
| `minimum_travel_distance` | `0.3` m | New scan threshold |
| `do_loop_closing` | `true` | Drift correction on revisit |

`config/amcl_params.yaml` (localization)

| Parameter | Value | Notes |
|-----------|-------|-------|
| `base_frame_id` | `base_link` | Matches the sensor_robot URDF |
| `laser_model_type` | `likelihood_field` | Standard 2D LiDAR model |
| `min_particles` | `500` | Minimum particle count |
| `max_particles` | `2000` | Maximum particle count |
| `update_min_d` | `0.2` m | Translation threshold for filter update |
| `set_initial_pose` | `true` | Uses random spawn coords as initial pose |
