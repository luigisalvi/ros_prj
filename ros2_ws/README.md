# ROS2 Workspace Nodes Documentation

This document provides an overview of the nodes present in the `ros2_ws/src` directory, detailing their functionality, topics, and usage.

---

## Nodes Overview

### 1. **`mdg_pkg`**

- **Node Name**: `mdg_node`
- **Description**:
  - Generates temperature data and publishes it to the `temperature` topic.
- **Topics**:
  - **Publishes**: `/temperature` (std_msgs/Float64)
  - **Subscribes**: None
- **Usage**:
  ```bash
  ros2 run mdg_pkg mdg_node
  ```

---

### 2. **`controller_pkg`**

- **Node Name**: `controller_node`
- **Description**:
  - Implements a PID controller that subscribes to the `temperature` topic and publishes corrections to the `correction` topic. It also listens to the `setpoint` topic for desired temperature values.
- **Topics**:
  - **Publishes**: `/correction` (std_msgs/Float64)
  - **Subscribes**:
    - `/temperature` (std_msgs/Float64)
    - `/setpoint` (std_msgs/Float64)
- **Usage**:
  ```bash
  ros2 run controller_pkg controller_node
  ```

---

### 3. **`visualizer_pkg`**

- **Node Name**: `visualizer_node`
- **Description**:
  - Visualizes the temperature and correction data by subscribing to the `temperature` and `correction` topics.
- **Topics**:
  - **Publishes**: None
  - **Subscribes**:
    - `/temperature` (std_msgs/Float64)
    - `/correction` (std_msgs/Float64)
- **Usage**:
  ```bash
  ros2 run visualizer_pkg visualizer_node
  ```

---

### 4. **`input_pkg`**

- **Node Name**: `input_node`
- **Description**:
  - Allows manual input of setpoints for the PID controller via the `setpoint` topic.
- **Topics**:
  - **Publishes**: `/setpoint` (std_msgs/Float64)
  - **Subscribes**: None
- **Usage**:
  ```bash
  ros2 run input_pkg input_node
  ```

---

### 5. **`bringup_pkg`**

- **Launch Files**:
  - **`all_nodes.launch.py`**:
    - Launches all nodes (`mdg_node`, `controller_node`, `visualizer_node`, and `input_node`) together.
    - Usage:
      ```bash
      ros2 launch bringup_pkg all_nodes.launch.py
      ```
  - **Individual Launch Files**:
    - Launches specific nodes independently.

---

## Notes

- Ensure all dependencies are installed and sourced before running the nodes.
- Use `ros2 topic list` and `ros2 topic echo <topic_name>` to verify topic communication.
- For development, use `colcon build --symlink-install` to reflect code changes immediately.

---

## References

- [ROS2 Documentation](https://docs.ros.org/en/foxy/index.html)
