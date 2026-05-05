# Dynamic Actor Example

Demonstrates a **dynamic actor** — an animated mesh following a scripted trajectory — inside Gazebo Fortress (Ignition Gazebo 6), with full ROS 2 integration and sensor detectability.

Unlike regular models, actors are **not physically simulated**. They move by interpolating keyframe poses defined in the SDF `<script>` block while playing back a skeletal animation stored in a `.dae` (COLLADA) file.

---

## Table of Contents

- [Concepts](#concepts)
- [Package Structure](#package-structure)
- [Quick Start](#quick-start)
- [Plugins](#plugins)
  - [ActorPosePublisher](#1-actorposepublisher-world-plugin)
  - [AttachModelPlugin](#2-attachmodelplugin-actor-plugin)
  - [TrajectoryActorPlugin](#3-trajectoryactorplugin-actor-plugin)
- [Trajectory Tips](#trajectory-tips)
- [Collision Avoidance with Nav2](#collision-avoidance-with-nav2)
- [Troubleshooting](#troubleshooting)

---

## Concepts

| Term | Meaning |
|------|---------|
| **Actor** | An SDF entity combining a skinned mesh with a motion script |
| **Skin** | The visual mesh (COLLADA `.dae`) rendered for the actor |
| **Animation** | The skeletal animation clip embedded in the `.dae` file |
| **Trajectory** | A sequence of world-space waypoints the actor's root follows over time |
| **Static model** | A regular physics model (cylinder) that shadows the actor so sensors detect it |

---

## Package Structure

```
dynamic_actor/
├── meshes/
│   └── actor.dae                      ← place your custom DAE file here
├── worlds/
│   └── actor_world.sdf                ← world definition
├── launch/
│   └── dynamic_actor.launch.py
├── src/
│   ├── ActorPosePublisher.cc          ← world plugin: ROS 2 pose publisher
│   ├── AttachModelPlugin.cc           ← actor plugin: static model tracker
│   └── TrajectoryActorPlugin.cc       ← actor plugin: programmatic movement
├── dynamic_actor/
│   └── actor_pose_subscriber.py       ← Python ROS 2 subscriber node
└── README.md
```

---

## Quick Start

**1. Add your DAE file:**
```bash
cp /path/to/your/model.dae \
   ~/ros2_ws/src/08_extra/dynamic_actor/meshes/actor.dae
```

**2. Build and launch:**
```bash
cd ~/ros2_ws
colcon build --packages-select dynamic_actor
source install/setup.bash
ros2 launch dynamic_actor dynamic_actor.launch.py
```

**3. Verify pose is publishing:**
```bash
ros2 topic echo /actor/walking_actor/pose
```

---

## Plugins

### 1. `ActorPosePublisher` — World Plugin

**Source:** [src/ActorPosePublisher.cc](src/ActorPosePublisher.cc)

Publishes the live world pose of every actor in the scene to ROS 2.

#### Why a custom plugin is needed

In Gazebo Fortress, actor positions are computed client-side by the rendering engine directly from SDF trajectory data. No ECM component (`WorldPose`, `TrajectoryPose`, etc.) is ever populated with the live animated pose at runtime — standard bridges and `PosePublisher` plugins do not work for actors.

#### How it works

On every `PostUpdate` tick, the plugin reads the waypoints from `components::Actor` (which stores the full `sdf::Actor` SDF object) and **interpolates the pose using simulation time** — matching exactly what the actor renders.

#### SDF usage (world-level)
```xml
<plugin filename="ActorPosePublisher"
        name="dynamic_actor::ActorPosePublisher"/>
```

#### Published topic

| Topic | Type | Description |
|-------|------|-------------|
| `/actor/<name>/pose` | `geometry_msgs/msg/PoseStamped` | Live interpolated world pose, `frame_id: world` |

A publisher is created automatically per actor — scales to any number of actors with no extra configuration.

#### Subscriber node

```bash
# Auto-discovers all actors
ros2 run dynamic_actor actor_pose_subscriber

# Single named actor
ros2 run dynamic_actor actor_pose_subscriber \
  --ros-args -p actor_name:=walking_actor
```

> **Note on orientation:** The published quaternion includes the `roll=1.5708` correction for Y-up DAE meshes. For navigation purposes only the heading yaw is needed.

---

### 2. `AttachModelPlugin` — Actor Plugin

**Source:** [src/AttachModelPlugin.cc](src/AttachModelPlugin.cc)

Fortress equivalent of Gazebo Classic's `libAttachModelPlugin.so`.

Teleports a separate **static collision model** to the actor's position every tick, making the actor detectable by Gazebo LiDAR sensors and contact sensors.

#### Why a static model

Actors are not physics entities — LiDAR rays pass straight through them regardless of any `<collision>` block defined inside the actor. The static model is a regular physics entity (a cylinder) that the physics engine registers in its broadphase. Any sensor that hits it produces real data.

#### How it works

1. A `<model>` with collision geometry is defined in the SDF, parked underground (`z = -100`) initially.
2. The plugin attaches inside the `<actor>` element and holds a reference to that model.
3. Each `PreUpdate` tick, it interpolates the actor's current pose from the SDF trajectory data and writes it to the static model via `components::WorldPoseCmd`.
4. The physics system processes `WorldPoseCmd` and teleports the static model to that pose.

#### SDF usage

**Step 1** — Define the static model in the world:
```xml
<model name="walking_actor_static">
  <pose>0 0 -100 0 0 0</pose>   <!-- parked underground at startup -->
  <static>false</static>
  <link name="link">
    <collision name="collision">
      <pose>0 0 0.85 0 0 0</pose>
      <geometry>
        <cylinder><radius>0.3</radius><length>1.7</length></cylinder>
      </geometry>
    </collision>
    <!-- optional: semi-transparent red visual for debugging -->
    <visual name="visual">
      <pose>0 0 0.85 0 0 0</pose>
      <geometry>
        <cylinder><radius>0.3</radius><length>1.7</length></cylinder>
      </geometry>
      <material>
        <ambient>1 0 0 0.4</ambient>
        <diffuse>1 0 0 0.4</diffuse>
      </material>
    </visual>
  </link>
</model>
```

**Step 2** — Add the plugin inside the `<actor>`:
```xml
<actor name="walking_actor">
  <!-- ... skin, animation, script ... -->
  <plugin filename="AttachModelPlugin"
          name="dynamic_actor::AttachModelPlugin">
    <model_name>walking_actor_static</model_name>
  </plugin>
</actor>
```

#### Parameters

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `model_name` | string | Yes | Name of the static model defined in the world |

#### For multiple actors

Add one static model and one plugin block per actor:
```xml
<model name="actor2_static"> ... </model>

<actor name="actor2">
  ...
  <plugin filename="AttachModelPlugin" name="...">
    <model_name>actor2_static</model_name>
  </plugin>
</actor>
```

---

### 3. `TrajectoryActorPlugin` — Actor Plugin

**Source:** [src/TrajectoryActorPlugin.cc](src/TrajectoryActorPlugin.cc)

Fortress equivalent of Gazebo Classic's `libTrajectoryActorPlugin.so`.

Drives an actor along a series of target waypoints at a constant velocity, using **potential-field steering** to navigate around obstacles dynamically.

#### Difference from SDF `<script>`

| | SDF `<script>` | `TrajectoryActorPlugin` |
|--|---|---|
| Motion defined by | Timed waypoints (fixed schedule) | Spatial targets (constant speed) |
| Obstacle awareness | None | Steers around obstacles using potential fields |
| Runtime control | Not possible | Extendable via plugin code |

#### How it works

1. The actor has `<skin>` and `<animation>` blocks but **no `<script>` block**.
2. The actor's SDF `<pose>` is **neutral** (`0 0 0 0 0 0`) — the plugin owns all pose state from tick 1.
3. On startup the plugin reads `<init>` for the spawn position and the mesh correction orientation (roll, pitch, yaw).
4. Each `PreUpdate` tick it computes a steering force: **attraction** (unit vector toward the current target) plus **repulsion** from any nearby obstacle models.
5. Only models with actual collision geometry are considered obstacles. Static models (ground, walls) are ignored automatically. Models listed in `<ignore>` are always excluded.
6. The combined force is normalised to constant speed `<velocity>`, so the actor always moves at the same speed even while avoiding obstacles.
7. `components::AnimationTime` advances proportionally to the step taken, keeping the walking animation in sync with movement speed.
8. The actor's heading yaw is `atan2(fy, fx) + initYaw`, where `initYaw` from `<init>` corrects the mesh's native forward direction to align with +X.

#### Obstacle detection

Detection is automatic — no configuration needed. Every tick, the plugin scans all entities that have a `Collision` component descendant, skips any model marked `<static>true</static>`, and skips any model name listed in `<ignore>`.

Repulsion follows the classic potential field formula: `gain × (1/d − 1/margin) / d²`, which gives a smooth force that grows as the actor approaches an obstacle and vanishes at `obstacle_margin`.

#### SDF usage

```xml
<actor name="my_actor">
  <!-- Neutral pose — plugin takes over from tick 1. -->
  <pose>0 0 0  0 0 0</pose>

  <skin>
    <filename>../meshes/actor.dae</filename>
    <scale>1.0</scale>
  </skin>
  <animation name="walking">
    <filename>../meshes/actor.dae</filename>
    <scale>1.0</scale>
    <interpolate_x>true</interpolate_x>
  </animation>
  <!-- NOTE: no <script> block -->

  <plugin filename="TrajectoryActorPlugin"
          name="dynamic_actor::TrajectoryActorPlugin">
    <!-- Spawn pose (x y z roll pitch yaw):
         x y z      = world start position
         roll/pitch = upright correction (1.5708 0 for Y-up DAE → +Z up)
         yaw        = mesh-forward correction to align native front with +X -->
    <init>-3 -3 0.36  1.5708 0 0</init>
    <!-- Waypoints (x y z roll pitch yaw): only x y z are used.
         Heading yaw is computed from movement direction + initYaw. -->
    <target>-3 -3 0.36  0 0 0</target>
    <target> 3 -3 0.36  0 0 0</target>
    <target> 3  3 0.36  0 0 0</target>
    <target>-3  3 0.36  0 0 0</target>
    <velocity>1.0</velocity>
    <obstacle_margin>1.5</obstacle_margin>   <!-- optional -->
    <repulsion_gain>2.0</repulsion_gain>     <!-- optional -->
    <ignore>my_collision_cylinder</ignore>   <!-- optional, repeatable -->
  </plugin>
</actor>
```

#### Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `init` | `x y z roll pitch yaw` | Yes | — | Spawn pose. `x y z` sets the world start position. `roll`/`pitch` correct the mesh upright (e.g. `1.5708 0` for Y-up DAE). `yaw` offsets the mesh forward direction to align it with +X. |
| `target` | `x y z roll pitch yaw` | Yes (one or more) | — | Waypoint — same format as SDF `<pose>`. Only `x y z` are used; orientation fields are ignored. Multiple elements are cycled in order. |
| `velocity` | double | No | `1.0` | Walking speed in m/s |
| `obstacle_margin` | double | No | `1.5` | Distance (m) at which repulsion begins |
| `repulsion_gain` | double | No | `2.0` | Strength of the repulsion force — increase if actor clips obstacles |
| `ignore` | string | No | — | Model name to exclude from obstacle detection. Repeatable. Use this to exclude the actor's own static collision model. |

---

## Trajectory Tips

### `TrajectoryActorPlugin` orientation setup

The mesh correction lives entirely in `<init>` — targets only need positions.

1. **Upright correction** — set `roll=1.5708 pitch=0` in `<init>` for a Y-up DAE to stand the actor upright in a Z-up world. Adjust if your DAE uses a different native orientation.
2. **Forward correction** — set `yaw` in `<init>` so the mesh's native front aligns with world +X after the roll/pitch correction. This offset is added to the movement heading every tick. If your mesh already faces +X after the roll/pitch, leave `yaw=0`.
3. **Yaw in `<target>` is ignored** — heading is computed every tick as `atan2(fy, fx) + initYaw`. The actor always faces the direction it is moving.
4. **Z height** — set a consistent `z` in `<init>` and all `<target>` entries (e.g. `0.35` for a DAE with ground-level root joint).

### SDF `<script>` actors (legacy)
1. **Separate walk and turn segments** — position and yaw changes in the same segment cause overshoot. Use two consecutive waypoints: arrive, then turn in place.
2. **Monotonically increasing yaw** — never let yaw decrease between consecutive waypoints. Use values like 0 → π/2 → π → 3π/2 → 2π so Gazebo always interpolates in the same rotational direction.
3. **`interpolate_x: false`** — makes the animation play at a constant rate based on wall time rather than path speed.

---

## Collision Avoidance with Nav2

With `AttachModelPlugin` active, the static model is a real physics entity. Its collision contacts propagate naturally into the sensor pipeline:

1. Add a LiDAR sensor to your robot model in the SDF.
2. Bridge the LiDAR scan topic with `ros_gz_bridge` to ROS 2.
3. Feed the scan into Nav2's costmap obstacle layer as usual.

The static model will appear as an obstacle in the costmap and the planner will route around it automatically — no custom bridge node needed.

---

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|-------------|-----|
| Actor lies flat | Y-up DAE — missing roll correction | Set `roll=1.5708 pitch=0` in `<init>` |
| Actor faces sideways | Mesh native front is not +X | Set `yaw` in `<init>` to rotate the mesh forward onto +X |
| Mesh not found | DAE not in `meshes/`, or not rebuilt | Add file, `colcon build`, re-source |
| `/actor/.../pose` not publishing | Plugin `.so` not found | Check `IGN_GAZEBO_SYSTEM_PLUGIN_PATH` includes `install/lib/dynamic_actor/` |
| Static model not following actor | `AttachModelPlugin` not loaded, or wrong `<model_name>` | Check SDF plugin block and model name match |
| LiDAR does not detect actor | Static model not in world, or `AttachModelPlugin` not active | Verify static model defined in SDF and plugin attached to actor |
| `TrajectoryActorPlugin` actor does not move | Actor has a `<script>` block overriding plugin | Remove `<script>` from the actor element |
