# 06 Nav2

Demonstrates **autonomous point-to-point navigation** using the full [Nav2](https://navigation.ros.org/) stack — AMCL localization, global/local costmaps, NavFn planner, and Regulated Pure Pursuit controller — with the sensor robot from package `04_sensor` on a pre-built map in Gazebo Fortress.

---

## Package structure

```
06_nav2/
├── config/
│   ├── nav2_params.yaml    # All Nav2 parameters (AMCL, planner, controller, costmaps, BT)
│   └── navigation.rviz     # RViz2 config (map, costmaps, paths, particle cloud, Nav2 panel, waypoints)
├── launch/
│   └── navigation.launch.py    # Full navigation stack, random spawn
├── nav2/
│   ├── __init__.py
│   └── waypoint_navigator.py   # Sends a series of poses sequentially, visualises progress in RViz
├── package.xml
├── setup.cfg
└── setup.py
```

> **Reused from `04_sensor`** (no duplication): `sensor_robot.urdf`, `bridge.yaml`, `sim_sensor_robot.launch.py`  
> **Requires a saved map** from `05_slam` — run mapping first if you don't have one.

---

## Dependencies

| Package | Role |
|---------|------|
| `sensor` | Robot URDF, bridge config, and simulation launch |
| `slam` | Provides `slam_world.sdf` (simulation environment) |
| `nav2_map_server` | Serves the saved map on `/map` |
| `nav2_amcl` | Particle filter localization |
| `nav2_planner` | Global path planning (NavFn / Dijkstra) |
| `nav2_controller` | Local path tracking (Regulated Pure Pursuit) |
| `nav2_behaviors` | Recovery behaviors (spin, backup, wait) |
| `nav2_bt_navigator` | Behavior tree orchestration |
| `nav2_lifecycle_manager` | Activates and manages all Nav2 nodes |
| `nav2_msgs` | `NavigateToPose` action definition |
| `rviz2` | Visualisation |

---

## Prerequisites — build a map first

If you do not have a saved map, follow the mapping steps in `05_slam` and save:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/my_map
```

This produces `~/my_map.pgm` and `~/my_map.yaml`.

---

## Running

```bash
# Full navigation stack (Gazebo + Nav2 + RViz2)
ros2 launch nav2 navigation.launch.py

# Custom map path
ros2 launch nav2 navigation.launch.py map:=/path/to/my_map.yaml
```

The robot spawns at a random pose. AMCL is seeded with the actual spawn coordinates so navigation is ready immediately.

---

## Sending goals

**Option 1 — RViz2 Navigation 2 panel** (added automatically by `navigation.rviz`):

1. Click **"Nav2 Goal"** in the Navigation 2 panel
2. Click on the map to set the target position and drag to set the heading
3. The panel's built-in action client sends the goal directly to the `bt_navigator`

Use **"Cancel"** in the panel to abort navigation at any time.

**Option 2 — terminal**:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0, z: 0.0}, orientation: {w: 1.0}}}}"
```

Change `x`, `y` to the desired position in the map frame. The `orientation` sets the final heading — `{w: 1.0}` means 0° (facing the +x direction).

To correct a poor initial localization estimate, use **2D Pose Estimate** in RViz2 to click the robot's true position — this reinitialises the AMCL particle distribution.

**Option 3 — python script** :

```bash
ros2 run nav2 waypoint_navigator
```

Edit the `WAYPOINTS` list at the top of [nav2/waypoint_navigator.py](nav2/waypoint_navigator.py) to define your route as `(x, y, yaw_degrees)` tuples. The node navigates to each pose in order and visualises progress in RViz:

| Colour | Meaning |
|--------|---------|
| Green  | Current target |
| Orange | Upcoming waypoints |
| Grey   | Completed waypoints |

---

## Key parameters

`config/nav2_params.yaml`

| Parameter | Value | Notes |
|-----------|-------|-------|
| `robot_radius` | `0.22` m | Circumscribed radius of sensor_robot |
| `inflation_radius` | `0.55` m | Costmap inflation around obstacles |
| `desired_linear_vel` | `0.3` m/s | Target cruising speed |
| `xy_goal_tolerance` | `0.25` m | Positional goal tolerance |
| `yaw_goal_tolerance` | `0.25` rad | Angular goal tolerance |
| `use_astar` | `false` | NavFn uses Dijkstra (set `true` for A*) |
| `min_particles` | `500` | AMCL minimum particle count |
| `max_particles` | `2000` | AMCL maximum particle count |
| `update_min_d` | `0.2` m | Translation threshold for AMCL update |
