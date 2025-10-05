# Catch Them All ROS2 Package

## Overview

The `catch_them_all` package is a ROS2 application built using **C++** that simulates a "catch the turtle" game in the `turtlesim` simulator. The package contains two primary nodes:

1. **TurtleSpawnNode** – Spawns turtles at random positions and publishes their information.
2. **TurtleChaseNode** – Controls `turtle1` to chase and "kill" other turtles using a proportional controller.

---

## Nodes

### 1. TurtleSpawnNode

**File:** `src/turtle_spawn.cpp`  

**Description:**  
This node is responsible for spawning turtles at random positions in the turtlesim window. Each turtle’s position and orientation are published on the `positions` topic for other nodes to subscribe.

**Key Features:**
- Generates random positions and orientations for turtles.
- Publishes turtle information using the custom `TurtleInfo` message.
- Calls the `/spawn` service to create turtles in the simulator.
- Uses a 1-second timer to continuously spawn turtles.

**Topics:**
- `positions` (`catch_them_all_interface/msg/TurtleInfo`) – Publishes information about all spawned turtles.

**Services:**
- `/spawn` (`turtlesim/srv/Spawn`) – Spawns a turtle at a random position.

**Dependencies:**
- `rclcpp`
- `turtlesim`
- `catch_them_all_interface`

---

### 2. TurtleChaseNode

**File:** `src/turtle_chase.cpp`  

**Description:**  
This node controls `turtle1` to chase other spawned turtles. It subscribes to `turtle1`’s pose and the positions of spawned turtles, then calculates the nearest turtle and moves toward it. If `turtle1` reaches a turtle (within a defined distance), it calls the `/kill` service to remove that turtle.

**Key Features:**
- Subscribes to Turtle1's pose (`/turtle1/pose`) to track its position.
- Subscribes to the `positions` topic to get the positions of all spawned turtles.
- Moves Turtle1 toward the nearest turtle using a simple proportional controller.
- Calls the `/kill` service to remove turtles that are close enough.
- Continuously updates Turtle1’s velocity via `/turtle1/cmd_vel`.

**Topics:**
- `/turtle1/cmd_vel` (`geometry_msgs/msg/Twist`) – Publishes velocity commands for Turtle1.
- `positions` (`catch_them_all_interface/msg/TurtleInfo`) – Subscribes to spawned turtle positions.

**Services:**
- `/kill` (`turtlesim/srv/Kill`) – Removes turtles when Turtle1 is close enough.

**Dependencies:**
- `rclcpp`
- `turtlesim`
- `geometry_msgs`
- `catch_them_all_interface`

---

## Custom Message

**TurtleInfo** (`catch_them_all_interface/msg/TurtleInfo.msg`)  

---
## To Launch file
```bash
cd ~/ali_ws
ros2 launch catch_them_all_bringup catch_them_all_launch.xml
```

