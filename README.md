# Intelligent Robotics — ROS2 Lecture Materials

Lecture materials for Intelligent Robotics — Ewha Womans University

### Instructor
- [Daeun Song](https://robotics.ewha.ac.kr/), Department of Artificial Intelligence, Ewha Womans University

- Contact: songd@ewha.ac.kr


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
    ├── 01_basics/        # Lecture 01
    ├── 02_*/             # Lecture 02
    └── ...
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
git clone <repo-url> ~/ros2_ws
cd ~/ros2_ws
```

### 3. Build the workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

To build a single package:

```bash
colcon build --packages-select <package_name>
```

## Lectures

| # | Package | Topic |
|---|---------|-------|
| 01 | `01_basics` | ROS2 Basics — nodes, topics, publisher/subscriber |

> New packages will be added as the course progresses.

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
