# TurtleBot3 Navigation: Wall Following & Bug2 Algorithm

This repository contains a ROS 2 implementation of autonomous navigation behaviors for the TurtleBot3 (Burger). It features a reactive **Wall Follower** and a **Bug2 Algorithm** that enables the robot to navigate toward a global goal while avoiding obstacles by switching between goal-seeking and boundary-following states.

## Features

* **Reactive Wall Following**: Utilizes LiDAR data to maintain a consistent distance (0.5m - 0.7m) from walls on the right side of the robot.
* **Bug2 Navigation**: Implements a finite state machine to navigate to specific (x, y) coordinates.
* **PID Control**: Uses proportional control for smooth angular and linear velocity adjustments during goal seeking.
* **M-Line Tracking**: Calculates the distance to the direct path between the start and goal to determine when to exit obstacle avoidance.


## Getting Started

### Prerequisites

* ROS 2 (Humble or Foxy)
* TurtleBot3 Simulation Packages
* Gazebo

### Installation

1. Clone this repo into your workspace `src` folder:
```bash
cd ~/ros2_ws/src
git clone <your-repo-link>

```


2. Build the package:
```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot3_navigation
source install/setup.bash

```



## 🕹 Usage

### Part 1: Wall Follower

To launch the robot in an environment where it will follow the right-hand wall indefinitely:

```bash
ros2 launch turtlebot3_navigation wall_follow.launch.py

```

### Part 2: Bug2 Algorithm

To navigate to a specific goal (default defined in `params.yaml`):

```bash
ros2 launch turtlebot3_navigation bug2.launch.py

```

## How it Works

### Wall Follower Logic

The robot samples LiDAR ranges at specific angles:

* **Front**: `0°` to `10°` and `350°` to `360°` to detect obstacles ahead.
* **Right**: `320°` to `350°` to monitor the wall distance.
It maintains a distance between `0.5` and `0.7` meters. If it gets too close, it steers left; if too far, it steers right.

### Bug2 State Machine

1. **Goal Seek**: The robot calculates the heading to the goal and moves forward.
2. **Wall Follow**: If an obstacle is detected, the robot switches to wall following.
3. **Transition**: It continues following the wall until it intersects the "M-line" (the original straight line from start to goal) and is closer to the goal than when it started following the wall.

---

### Customizing the Goal

You can change the navigation target by editing `turtlebot3_navigation/config/params.yaml`:

```yaml
goal_position: [2.0, -1.2]

```

---
## Result

### Wall-follow:
![Wall Follower Demo](media/wall_follow.gif)

### BUG2:
![Bug2 Algorithm Demo](media/bug2.gif)

---
## Resources
- [Including Launch Files](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Using-Substitutions.html)
- [ROS Params](https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html)
- [`rclpy` Parameters](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html#write-the-python-node)
- [LaserScan Message](https://docs.ros.org/en/ros2_packages/humble/api/sensor_msgs/msg/LaserScan.html)
- [Bug Algorithms](https://www.cs.cmu.edu/~motionplanning/lecture/Chap2-Bug-Alg_howie.pdf)
