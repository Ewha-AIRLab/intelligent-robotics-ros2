# 01_Basics

ROS2 basics — Node, Topic, Service, Action

## Contents

| File | Description |
|---|---|
| `basics/talker.py` | Publisher node — publishes `std_msgs/String` to `/chatter` at 1 Hz |
| `basics/listener.py` | Subscriber node — subscribes to `/chatter` and logs received messages |

## Build

From the workspace root:

```bash
colcon build --packages-select basics
source install/setup.bash
```

## Run

**Terminal 1 — talker:**
```bash
ros2 run basics talker
```

**Terminal 2 — listener:**
```bash
ros2 run basics listener
```

**Or launch both at once:**
```bash
ros2 launch basics talker_listener.launch.py
```

## Inspect

```bash
ros2 node list
ros2 topic list
ros2 topic echo /chatter
ros2 topic hz /chatter
ros2 topic info /chatter
rqt_graph
```