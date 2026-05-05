# Multi-Robot Demo

Demonstrates two independent differential-drive robots navigating in the same Gazebo world.
Each robot has its own namespaced Nav2 stack — separate map, AMCL, planner, controller, and costmaps — so they plan and localize independently without interfering.

---

## Concepts

| Term | Meaning |
|------|---------|
| **Namespace** | ROS 2 prefix that scopes all topics, services, and nodes for one robot |
| **TF frame prefix** | Prefix applied to all URDF link names so each robot has its own TF tree |
| **RewrittenYaml** | Nav2 utility that rewrites a params file at launch time to substitute per-robot values |
| **Nav2 lifecycle manager** | Manages startup/shutdown ordering of Nav2 nodes |

---

## Package structure

```
multi_robot/
├── urdf/
│   └── multi_robot.urdf.xacro      # Single xacro, parameterized by namespace
├── config/
│   └── nav2_params.yaml            # Generic Nav2 params (frames overridden at launch)
└── launch/
    ├── robot_nav.launch.py         # Launches one robot + full Nav2 stack
    └── multi_robot.launch.py       # Top-level: Gazebo + two robot_nav instances
```

---

## How namespace isolation works

Three mechanisms work together so the two robots don't interfere:

### 1. URDF / xacro — namespaced Gazebo topics and TF frames

The xacro file takes a `namespace` argument. All Gazebo plugin topics and frame IDs use it:

```xml
<topic>${ns}/cmd_vel</topic>
<odom_topic>${ns}/odom</odom_topic>
<frame_id>${ns}/odom</frame_id>
<child_frame_id>${ns}/base_link</child_frame_id>
<topic>${ns}/scan</topic>
```

Result: `robot1` publishes on `/robot1/cmd_vel`, `/robot1/odom`, `/robot1/scan`; `robot2` on `/robot2/…`

### 2. `robot_state_publisher` — namespaced TF tree

```python
parameters=[{'frame_prefix': f'{ns}/'}]
```

All URDF link names (`base_link`, `laser_frame`, etc.) become `robot1/base_link`, `robot1/laser_frame`, etc. Each robot has a completely separate TF tree.

### 3. `RewrittenYaml` — per-robot Nav2 params

`nav2_params.yaml` uses generic names (`base_link`, `odom`). At launch time, `RewrittenYaml` substitutes the correct namespaced values:

| Generic | robot1 | robot2 |
|---------|--------|--------|
| `base_link` | `robot1/base_link` | `robot2/base_link` |
| `odom` | `robot1/odom` | `robot2/odom` |
| `/odom` | `/robot1/odom` | `/robot2/odom` |
| `/scan` | `/robot1/scan` | `/robot2/scan` |

---

## Prerequisites

This package reuses the Gazebo world from the `slam` package:

```bash
# Build the workspace (includes slam and multi_robot)
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

You also need a pre-built map YAML (e.g. from the slam demo):

```bash
# Example: map saved from the slam package
~/my_map.yaml
```

---

## Quick Start

### 1. Launch everything

```bash
ros2 launch multi_robot multi_robot.launch.py map:=$HOME/my_map.yaml
```

This starts:
- Gazebo with the slam world
- `robot1` at (-2, 0, 0°) with full Nav2
- `robot2` at (2, 0, 180°) with full Nav2
- RViz

Nav2 starts with a 5-second delay to allow the robots to spawn first.

### 2. Wait for AMCL to initialize (optional - already done)

Each robot's AMCL auto-initializes at its spawn position (injected from the launch arguments).
No manual initial pose is required. Wait for the particle clouds to appear in RViz (~5 s for
robot1, ~15 s for robot2).

If you need to manually correct a robot's pose, publish to its namespaced topic
(the RViz "2D Pose Estimate" button publishes to `/initialpose` and will NOT reach namespaced nodes):

```bash
# robot1
ros2 topic pub /robot1/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: -2.0, y: 0.0}, orientation: {w: 1.0}}}}" --once

# robot2
ros2 topic pub /robot2/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: map}, pose: {pose: {position: {x: 2.0, y: 0.0}, orientation: {z: 1.0, w: 0.0}}}}" --once
```

### 3. Send navigation goals

In RViz, select a robot's namespace context and use **2D Nav Goal**. Or via CLI:

```bash
# Send robot1 to (2, 1)
ros2 action send_goal /robot1/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"

# Send robot2 to (-2, -1)
ros2 action send_goal /robot2/navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: -2.0, y: -1.0}, orientation: {w: 1.0}}}}"
```

> The SLAM world is ~10 m × 10 m. Keep goals within roughly ±4 m from the origin to stay inside the map.

### 4. Teleoperate (optional)

```bash
# robot1
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/robot1/cmd_vel

# robot2 (separate terminal)
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -r /cmd_vel:=/robot2/cmd_vel
```

---

## Key topics per robot

| Topic | Description |
|-------|-------------|
| `/{ns}/cmd_vel` | Velocity command input |
| `/{ns}/odom` | Odometry from diff-drive plugin |
| `/{ns}/scan` | 2D LiDAR scan |
| `/{ns}/amcl_pose` | AMCL localization estimate |
| `/{ns}/navigate_to_pose` | Nav2 goal action |

---

## Troubleshooting

| Symptom | Cause | Fix |
|---------|-------|-----|
| `map` frame does not exist | AMCL hasn't received initial pose | Set initial pose via RViz or CLI after Nav2 starts |
| `{ns}/odom` frame missing | Nav2 started before robot spawned | The 5 s `TimerAction` handles this; wait for it |
| Both robots collide | Each robot's costmap only sees its own LiDAR | Expected — robots avoid static obstacles but not each other |
| Nav goal ignored | Wrong namespace in action name | Use `/robot1/navigate_to_pose`, not `/navigate_to_pose` |
