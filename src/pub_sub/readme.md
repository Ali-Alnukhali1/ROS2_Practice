# ROS2 Publisher-Subscriber Practice (pub_sub)

This project is a **practice implementation** of the publisher–subscriber pattern in ROS2.  
It is **not intended for production use**, but rather to get familiar with how publishers, subscribers, and timers work in ROS2.

---

## Overview

The project contains two ROS2 nodes:

- **`number_publisher`**  
  Publishes a constant integer (`1`) to the topic `/number` every 500 ms.

- **`number_counter`**  
  Subscribes to the `/number` topic, logs any received values, and publishes an incrementing counter to the `/number_count` topic every 500 ms.

---

## Building and Running the Package 

   ```bash
   cd ~/ros2_ws/src
   git clone <your-repo-link> pub_sub
   cd ~/ros2_ws
   colcon build --packages-select pub_sub
   source install/setup.bash
   ros2 run pub_sub number_publisher
   ros2 run pub_sub number_counter
   ```
---

## Building and Running the Package 

   ```bash
   rqt_graph
   ```