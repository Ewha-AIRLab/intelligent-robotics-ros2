# Intelligent Robotics — ROS2 Lecture Materials

Lecture materials for Intelligent Robotics — Ewha Womans University

### Instructor
- [Daeun Song](https://robotics.ewha.ac.kr/), Department of Artificial Intelligence, Ewha Womans University

- Contact: songd@ewha.ac.kr

## Lectures

| # | Package | Topic |
|---|---------|-------|
| 01 | `basics` | ROS2 Basics — nodes, topics, publisher/subscriber |
| 02 | `robot_description` | Robot modeling — URDF, joint/link, robot_state_publisher |
| 03 | `gazebo_simulation` | Gazebo Fortress simulation — differential drive, spawning |
| 04 | `sensor` | Sensor simulation — 2D LiDAR, RGB/depth camera, ros_gz_bridge |
| 05 | `slam` | SLAM & Localization — slam_toolbox (mapping), Nav2 AMCL (localization) |
| 06 | `nav2` | Navigation — Nav2 stack, costmaps, path planning, controller |
| 07 | `moveit2` | Manipulation — MoveIt2, motion planning, pick and place |
| 08-1 | `dynamic_actor` | Dynamic actors in Gazebo — trajectory following, collision avoidance |
| 08-2 | `multi_robot` | Multi-robot navigation — two independent Nav2 stacks in one Gazebo world |

## Extra

| Package | Topic |
|---------|-------|
| `mobile_manipulator` | Mobile manipulation — Clearpath Husky + Kinova arm, Nav2 + MoveIt2 |

## Environment

| Item | Version |
|------|---------|
| OS | Ubuntu 22.04 LTS (Jammy Jellyfish) |
| ROS | ROS2 Humble Hawksbill |
| Python | 3.10 |

## Repository Structure

Each directory under `src/` corresponds to one lecture and is organized as a ROS2 package.

```
ros2_ws/
└── src/
    ├── 01_basics/
    ├── 02_robot_description/
    ├── 03_gazebo_simulation/
    ├── 04_sensor/
    ├── 05_slam/
    ├── 06_nav2/
    ├── 07_moveit2/
    └── 08_extra/
        ├── dynamic_actor/
        <!-- ├── mobile_manipulator/ -->
        └── multi_robot/
```

## Setup

### 1. Install ROS2 Humble

Follow the official installation guide:
https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html

After installation, source the ROS2 setup file:

```bash
source /opt/ros/humble/setup.bash
```

### 2. Clone this repository

```bash
git clone https://github.com/Ewha-AIRLab/intelligent-robotics-ros2 ~/ros2_ws
cd ~/ros2_ws
```

### 3. Install dependencies
```bash
rosdep install --from-paths src --ignore-src -r -y
```


### 4. Build the workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

To build a single package:

```bash
colcon build --packages-select <package_name>
```


## Running Examples

Source the workspace before running any node:

```bash
source ~/ros2_ws/install/setup.bash
```

Then run a node with:

```bash
ros2 run <package_name> <executable_name>
```

## License

See [LICENSE](LICENSE).
