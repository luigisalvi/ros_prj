# ROS2 Workspace with Docker Compose

This project provides a ROS2 workspace (`ros2_ws`) and a `docker-compose` setup for running and managing ROS2 nodes in a development environment. The workspace includes multiple ROS2 packages for generating, controlling, visualizing, and interacting with data.

## Project Structure

### Workspace Layout

The `ros2_ws` directory contains the ROS2 packages:

- **`mdg_pkg`**: Generates temperature data and publishes it to the `temperature` topic.
- **`controller_pkg`**: Implements a PID controller that subscribes to the `temperature` topic and publishes corrections to the `correction` topic.
- **`visualizer_pkg`**: Visualizes the temperature and correction data by subscribing to the `temperature` and `correction` topics.
- **`input_pkg`**: Allows manual input of setpoints for the PID controller via the `setpoint` topic.
- **`bringup_pkg`**: Contains launch files to start individual or all nodes.

## Docker Compose Services

The `docker-compose.dev.yaml` file defines the following services:

### `ros2_system-dev`

- **Purpose**: Runs the main ROS2 system, including all nodes defined in the `bringup_pkg` launch file.
- **Build Context**: Uses `Dockerfile.dev` to build the image.
- **Volumes**:
  - Mounts the `ros2_ws` directory to `/app/ros2_ws` inside the container for live development.
  - Mounts `/tmp/.X11-unix` for GUI applications like RViz.
- **Command**:
  - Sources ROS2 (`jazzy`) setup files.
  - Builds the workspace with `colcon build --symlink-install`.
  - Launches all nodes using `ros2 launch bringup_pkg all_nodes.launch.py`.

### `ros2_input-dev`

- **Purpose**: Runs the `input_pkg` node for manual setpoint input.
- **Build Context**: Uses `Dockerfile.dev` to build the image.
- **Volumes**:
  - Mounts the `ros2_ws` directory to `/app/ros2_ws` inside the container for live development.
  - Mounts `/tmp/.X11-unix` for GUI applications.
- **Dependencies**: Depends on `ros2_system-dev` to ensure the main system is running before starting.

## Usage Instructions

### Development Environment

1. **Start the Docker Compose Services**:
   ```sh
   docker compose -f docker-compose.dev.yaml up -d
   ```
2. **Access the Main ROS2 Container**:

   ```sh
   docker exec -it ros2_main_container bash
   ```

   - Source ROS2 setup:
     ```sh
     source /opt/ros/jazzy/setup.bash
     ```
   - Run ROS2 commands or interact with nodes.

3. **Access the Input Node Container**:
   ```sh
   docker exec -it ros2_input_container bash
   ```
   - Source ROS2 setup:
     ```sh
     source /opt/ros/jazzy/setup.bash
     ```
   - Run the input node:
     ```sh
     ros2 run input_pkg input_node
     ```

### Building and Running Nodes

1. **Build a Specific Package**:
   ```sh
   colcon build --packages-select <package_name> --symlink-install
   ```
2. **Run a Node**:
   ```sh
   ros2 run <package_name> <node_name>
   ```


## ROS2 Packages Overview

### mdg_pkg

- **Node**: `mdg_node`
- **Topics**:
  - Publishes: `temperature`
  - Subscribes: `correction`

### controller_pkg

- **Node**: `controller_node`
- **Topics**:
  - Publishes: `correction`
  - Subscribes: `temperature`, `setpoint`

### visualizer_pkg

- **Node**: `visualizer_node`
- **Topics**:
  - Subscribes: `temperature`, `correction`

### input_pkg

- **Node**: `input_node`
- **Topics**:
  - Publishes: `setpoint`

### `bringup_pkg`

- **Launch Files**:
  - all_nodes.launch.py: Launches all nodes.
  - Individual launch files for each node.

## Development

### initialize

1. open docker container's terminal
2. type `bash` command
3. do the ROS sourcing by typing `source /opt/ros/jazzy/setup.bash`
4. `ros2` command is ready to run

### Create a new package

1. open docker container's terminal
2. type `bash` command
3. do the ROS sourcing by typing `source /opt/ros/jazzy/setup.bash`
4. type `cd /app/ros2_ws/src`
5. `ros2 pkg create input_pkg --build-type ament_python --dependencies rclpy std_msgs`
6. `colcon build --packages-select <nome_del_pacchetto> --symlink-install`

### Run input node
1. open docker container's terminal
2. type `bash` command
3. do the ROS sourcing by typing `source /opt/ros/jazzy/setup.bash`
4. move to `cd /app/ros2_ws/src`
5. type `colcon build --packages-select input_pkg --symlink-install` to build input package
6. type `source install/setup.bash` to install
7. type `ros2 run input_pkg input_node` to start the input node

## Notes

- Ensure the `DISPLAY` environment variable is set correctly for GUI applications.
- Use `colcon build --symlink-install` for live development to reflect changes immediately.
- For production, use the docker-compose.prod.yaml file.

## Troubleshooting

- **GUI Issues**: Ensure X11 forwarding is enabled and `/tmp/.X11-unix` is mounted correctly.
- **Build Errors**: Check that all dependencies are installed and sourced correctly.
- **Node Communication**: Verify topics and message types using:
  ```sh
  ros2 topic list
  ros2 topic echo <topic_name>
  ```

## References

- [ROS2 Documentation](https://docs.ros.org/en/foxy/index.html)
- [Docker Compose Documentation](https://docs.docker.com/compose/)
