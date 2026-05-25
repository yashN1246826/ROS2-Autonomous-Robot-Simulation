````md
# ROS2 Autonomous Robot Simulation

ROS2 and Gazebo-based autonomous robot simulation featuring navigation, object detection, traffic-rule behaviours, odometry, TF broadcasting and mapping support.

---

## Project Overview

This project demonstrates an autonomous robot simulation built using ROS2 Humble and Gazebo. The system allows a simulated robot to navigate through a maze-style environment, process sensor data, detect objects/signs, react to traffic rules, publish odometry transforms and support mapping/navigation workflows.

The project brings together robotics simulation, perception, navigation logic, ROS2 communication and environment modelling.

---

## Key Features

- ROS2-based robotic simulation environment
- Gazebo maze world with robot models and simulation assets
- Autonomous navigation and exploration behaviour
- Object and traffic sign detection support
- Stop, slow and fast sign behaviour logic
- Odometry-to-TF broadcasting
- Point cloud and scan processing utilities
- Landmark detection and logging logic
- RViz configuration for visualisation
- Mapping support using OctoMap-related packages
- Modular ROS2 package structure

---

## Tech Stack

- ROS2 Humble
- Gazebo / Ignition Gazebo
- Python
- C++
- CMake
- RViz
- TF2
- Nav2-style navigation concepts
- OctoMap
- Point cloud processing
- Linux / Docker-compatible workflow

---

## Repository Structure

```text
ROS2-Autonomous-Robot-Simulation/
│
├── src/
│   ├── ntu_robotsim/
│   │   ├── config/
│   │   ├── hooks/
│   │   ├── launch/
│   │   ├── models/
│   │   ├── scripts/
│   │   ├── worlds/
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   ├── octomap2/
│   │   ├── octomap_msgs/
│   │   ├── octomap_server2/
│   │   ├── pcl_msgs/
│   │   └── perception_pcl/
│   │
│   └── odom_to_tf_ros2/
│       ├── launch/
│       ├── src/
│       ├── CMakeLists.txt
│       └── package.xml
│
├── .gitignore
└── README.md
````

> The `ntu_robotsim` package name has been kept unchanged to preserve compatibility with ROS2 launch files, package references and simulation paths.

---

## Main Components

### 1. Simulation Environment

The `ntu_robotsim` package contains the main simulation assets, including robot models, Gazebo worlds, launch files and RViz configurations.

Main folders:

```text
src/ntu_robotsim/launch/
src/ntu_robotsim/models/
src/ntu_robotsim/worlds/
src/ntu_robotsim/config/
```

---

### 2. Autonomous Navigation Scripts

The project includes Python scripts for robot behaviour, exploration, navigation and perception handling.

Important scripts include:

```text
auto_explore_node.py
basic_mapping_node.py
nav2_goal_node.py
traffic_rules_node.py
goal_detection_node.py
landmark_database_node.py
object_counter_node.py
```

These scripts support autonomous movement, object detection handling, traffic-rule behaviours and landmark tracking.

---

### 3. Traffic Rule Behaviour

The robot can respond to detected signs using behaviour rules:

| Detection               | Robot Behaviour              |
| ----------------------- | ---------------------------- |
| Stop sign               | Stops the robot temporarily  |
| Slow sign               | Reduces robot speed          |
| Fast sign               | Increases robot speed        |
| Obstacle / blocked path | Adjusts navigation behaviour |

This demonstrates how perception can be connected to robot motion control.

---

### 4. Odometry and TF Support

The project includes odometry-to-TF functionality through:

```text
src/odom_to_tf_ros2/
```

This package helps publish transform information from odometry data, allowing the robot frame and world frame to be visualised and used correctly in ROS2 tools.

---

### 5. Mapping and Point Cloud Support

The repository includes OctoMap and point cloud-related packages under:

```text
src/octomap2/
```

It also includes helper scripts such as:

```text
pc_to_scan_node.py
pc_to_depth_image.py
scan_restamper.py
rgbd_sync_republisher.py
```

These support sensor data conversion, scan handling and mapping-related workflows.

---

## Example ROS2 Topics

Depending on the launched setup, the project can use topics such as:

```text
/atlas/rgbd_camera/image
/atlas/odom_ground_truth
/atlas/cmd_vel
/atlas/cmd_vel_raw
/yolo/detections
/landmark_summary
/detected_goal
/all_detected_goals
/scan
```

These topics allow communication between robot sensors, perception nodes, navigation logic and velocity control.

---

## How to Run

### 1. Clone the repository

```bash
git clone https://github.com/yashN1246826/ROS2-Autonomous-Robot-Simulation.git
cd ROS2-Autonomous-Robot-Simulation
```

### 2. Source ROS2

```bash
source /opt/ros/humble/setup.bash
```

### 3. Build the workspace

```bash
colcon build
```

### 4. Source the workspace

```bash
source install/setup.bash
```

### 5. Launch the maze simulation

```bash
ros2 launch ntu_robotsim cwmaze.launch.py
```

Alternative launch option:

```bash
ros2 launch ntu_robotsim single_robot_sim.launch.py
```

---

## Useful Commands

List active ROS2 topics:

```bash
ros2 topic list
```

Check odometry:

```bash
ros2 topic echo /atlas/odom_ground_truth
```

Check detection output:

```bash
ros2 topic echo /yolo/detections
```

View TF frames:

```bash
ros2 run tf2_tools view_frames
```

Run odometry-to-TF launch:

```bash
ros2 launch odom_to_tf_ros2 atlas_odom_to_tf.launch.py
```

Publish a test velocity command:

```bash
ros2 topic pub /atlas/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

---

## Skills Demonstrated

This project demonstrates practical experience with:

* ROS2 package structure
* Gazebo robot simulation
* Autonomous navigation logic
* Object detection integration
* ROS2 topics and nodes
* Odometry and TF broadcasting
* RViz visualisation
* Mapping and point cloud processing
* Python robotics scripting
* C++ ROS2 node development
* Linux-based robotics workflow

---

## Future Improvements

* Add a cleaner launch sequence for all nodes
* Improve navigation recovery behaviour
* Add screenshots and demo videos
* Add clearer parameter documentation
* Improve object detection confidence handling
* Add a visual dashboard for robot state and detections
* Add automated setup scripts
* Add more detailed mapping examples

---

## Author

**Yash Kumar**
Computer Science graduate interested in software engineering, robotics, AI, computer vision and cloud systems.

GitHub: [yashN1246826](https://github.com/yashN1246826)

````

After adding it, run:

```bash
git add README.md
git commit -m "Add professional project README"
git push
````
