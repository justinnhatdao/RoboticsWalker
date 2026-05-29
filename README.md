# Home Guide Robot Navigation System

A ROS2 simulation of a guide robot that helps visually impaired users navigate their home.
The user selects a room destination via a terminal input interface and the robot autonomously
navigates there, announcing its status along the way.

---

## Packages and Libraries Used

### Nav2 (Navigation 2)
- **What it is:** The standard ROS2 autonomous navigation stack.
- **How it's used:** Handles all path planning, localization (AMCL), and obstacle avoidance. The robot receives a navigation goal and Nav2 computes and executes a safe path to it automatically.
- **Source:** https://docs.nav2.org

### turtlebot3_simulations 
- **What it is:** Simulation package for the TurtleBot3 robot platform.
- **How it's used:** Provides the TurtleBot3 Waffle robot model, sensor plugins, and Ignition Gazebo integration for the simulation environment.
- **Source:** https://github.com/azeey/turtlebot3_simulations

### rclpy
- **What it is:** The official ROS2 Python client library.
- **How it's used:** All custom nodes in this project are written in Python using rclpy. It handles node creation, publishers, subscribers, and action clients for Nav2 goal sending.
- **Source:** https://docs.ros2.org/latest/api/rclpy/

### turtlebot3_cartographer (Google Cartographer)
- **What it is:** A SLAM (Simultaneous Localization and Mapping) package.
- **How it's used:** Used to generate the map of the simulated home environment. The robot is driven manually through the space and Cartographer builds a 2D occupancy grid map, which is then saved and loaded by Nav2 for autonomous navigation.

### Ignition Gazebo (Ignition Fortress)
- **What it is:** A robot simulation environment.
- **How it's used:** Hosts the custom 3-room house world (`guide_house.world`) and simulates the TurtleBot3 Waffle including its LiDAR sensor (`/scan` topic) for Nav2 to use.

---

## Original Contributions (Novel Functionality)

The following components were written entirely from scratch for this project. None of this functionality exists in Nav2 or any other library used.

### 1. Named Waypoint System
Nav2 works with raw x/y coordinates , it has no concept of named rooms. This project includes a Python dictionary that maps human-readable room names to their specific coordinates in the simulated environment:

```python
WAYPOINTS = {
    '1': ('living room', 0.05, -2.93),
    '2': ('bedroom', -6.23, -0.38),
    '3': ('office', -5.89, -5.12),
}
```

When a user selects a destination, the node looks up the corresponding coordinates and sends them to Nav2 as a navigation goal. This is the core bridge between user intent and robot action.

### 2. Accessible Terminal Input Interface (Dead-Man's Switch Design)
A custom input interface that requires the user to hold a "dead-man's switch" key while pressing a number key to select a destination. This is intentionally designed with safety in mind , accidental single keystrokes cannot send the robot moving. Current work in progress.

### 3. Guide Status Announcer Node
A ROS2 publisher/subscriber node that generates human-readable status messages as the robot navigates. Messages such as `"Heading to Bedroom..."` and `"Arrived at Bedroom"` are published to a dedicated ROS2 topic. This gives a visually impaired user the verbal feedback they would need to follow the robot to their destination. I also have annoucements along the way updating every meter of movement like  `"Bedroom 6.1m remaining..."`

### 4. Wanderer Node
A custom autonomous exploration node that drives the robot through the environment during the SLAM mapping phase. Rather than requiring fully manual teleoperation to build the map, the wanderer node moves the robot through the space automatically while Cartographer builds the occupancy grid.

### 5. Custom RViz Configuration
The default Nav2 RViz configuration caused the robot visualization to move too fast and snap erratically, making it difficult to monitor navigation. A custom RViz config was created to tune the display behavior , smoothing out robot movement in the visualizer and making it easier to observe and debug the navigation stack during development.

### 6. Custom WASD Teleop Node
A custom keyboard teleoperation node written from scratch with a game-style WASD control scheme. Unlike the standard `turtlebot3_teleop_key`, this node also supports automatic map-following behavior while preserving the ability to manually override and take direct control of the robot at any time. This dual-mode design (auto + manual override) was built specifically to support the SLAM mapping workflow.

---

## Environment

| Item | Detail |
|---|---|
| OS | Ubuntu 22.04.5 LTS (ARM64, UTM VM) |
| ROS2 | Humble Hawksbill |
| Robot | TurtleBot3 Waffle (simulated) |
| Simulator | Ignition Gazebo (Ignition Fortress) |
| Language | Python 3.10 via rclpy |

---
