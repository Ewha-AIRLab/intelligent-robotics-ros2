# Lecture 7 — MoveIt2 with UR5e + Robotiq 2F-85

This lecture integrates a Universal Robots UR5e arm with a Robotiq 2F-85 parallel-jaw gripper using MoveIt2.

---

## Prerequisites — MoveIt2 Getting Started

Before working through this lecture, complete the official MoveIt2 getting-started guide to install all dependencies and verify your environment:

[MoveIt2 Getting Started](https://moveit.picknik.ai/main/doc/tutorials/getting_started/getting_started.html)

Key steps from that guide:

```bash
# Install MoveIt2 for ROS 2 Humble
sudo apt install ros-humble-moveit
```

Once MoveIt2 is installed you are ready to continue below.

---

## MoveIt config files

SRDF, kinematics, joint limits, and planning configs are loaded from the apt-installed package `moveit_resources_panda_moveit_config`:

```bash
cd $(ros2 pkg prefix moveit_resources_panda_moveit_config)/share/moveit_resources_panda_moveit_config
```

---

## Package layout

```
07_moveit2/           # ROS2 package: moveit2
├── launch/
├── config/
├── urdf/
├── worlds/
└── README.md
```

---

## Step 1 — Build

```bash
cd ~/ros2_ws
rosdep install -r --from-paths src/07_moveit2 --ignore-src --rosdistro humble -y
colcon build --symlink-install --packages-select moveit2
source install/setup.bash
```

---

## Step 2 — Run the quickstart demo (fake hardware)

Follows the tutorial at [MoveIt2 Quickstart in RViz](https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html):

```bash
ros2 launch moveit2 demo.launch.py
```

The robot runs with `mock_components/GenericSystem` — motion planning works in RViz, no physics simulation.

---

## Step 3 — Generate MoveIt config with Setup Assistant

To create a MoveIt config for a new robot, use the Setup Assistant:

Tutorial: [MoveIt Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html)

```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

---

## Step 4 — Run the Gazebo simulation

Swaps fake hardware for `gz_ros2_control/GazeboSimSystem` and adds a Gazebo world with a table:

```bash
ros2 launch moveit2 sim.launch.py
```