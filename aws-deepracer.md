# AWS DeepRacer Dockerfile Documentation

This document provides an overview of the `Dockerfile.deepracer`, which is used to set up a Docker environment for AWS DeepRacer development and simulation with ROS2.

## `Dockerfile.deepracer`

The `Dockerfile.deepracer` is designed to create a containerized environment for AWS DeepRacer, including all necessary dependencies for navigation, simulation, and ROS2 integration.

### Key Features:

- **Base Image**: Uses `osrf/ros:foxy-desktop` as the base image, which includes ROS2 Foxy and desktop tools like RViz.
- **DeepRacer Dependencies**:
  - Installs required ROS2 packages, Gazebo plugins, and Python tools for DeepRacer.
- **Workspace Setup**:
  - Clones the AWS DeepRacer repository and sets up the workspace.
  - Resolves dependencies using `rosdep` and builds the workspace with `colcon`.
- **Entrypoint**:
  - Configures the container to use a custom entrypoint script (`ros_entrypoint_deepracer.sh`).

### File Content:

```dockerfile
# filepath: Dockerfile.deepracer
FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# Copy the custom entrypoint script
COPY ros_entrypoint_deepracer.sh ros_entrypoint.sh

# Install required dependencies
RUN sudo apt update && \
    sudo apt install -y ros-foxy-gazebo-ros-pkgs python3-rosinstall ros-foxy-rviz2

# Clone and build the DeepRacer workspace
RUN source /opt/ros/foxy/setup.bash && \
    mkdir -p deepracer_nav2_ws && \
    cd deepracer_nav2_ws && \
    git clone https://github.com/aws-deepracer/aws-deepracer.git && \
    rosdep install -i -r -y --from-paths . && \
    cd aws-deepracer && \
    rosinstall deepracer_description; \
    cd gazebo_ros2_control && git reset --hard 04b2c6f0ff0e977b6fc6af5dfc5e96e5bdd570d0 && cd .. && \
    colcon build

# Set the custom entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]
```

## Explanation of Key Steps

1. **Base Image**:

   - The image is based on `osrf/ros:foxy-desktop`, which includes ROS2 Foxy and tools like RViz for visualization.

2. **Custom Entrypoint**:

   - The `ros_entrypoint_deepracer.sh` script is copied into the container and set as the entrypoint. This script is expected to handle environment setup and container initialization.

3. **Dependency Installation**:

   - Installs essential ROS2 packages:
     - `ros-foxy-gazebo-ros-pkgs`: Provides Gazebo plugins for ROS2.
     - `python3-rosinstall`: Used for managing ROS2 dependencies.
     - `ros-foxy-rviz2`: Visualization tool for ROS2.

4. **Workspace Setup**:

   - Creates a workspace directory (`deepracer_nav2_ws`) and clones the AWS DeepRacer repository.
   - Resolves dependencies using `rosdep`.
   - Builds the workspace using `colcon build`.

5. **Specific Git Reset**:
   - Resets the `gazebo_ros2_control` package to a specific commit (`04b2c6f0ff0e977b6fc6af5dfc5e96e5bdd570d0`) to ensure compatibility.

## Usage Instructions

### Build the Docker Image

1. Navigate to the directory containing the `Dockerfile.deepracer`.
2. Build the image:
   ```bash
   docker build -t deepracer-simulation -f Dockerfile.deepracer .
   ```

### Run the Container

1. Start the container:

   ```bash
   xhost +
   docker run -it --rm --name deepracer-sim \
       --ipc=host --net=host --gpus all \
       -v /tmp/.X11-unix:/tmp/.X11-unix \
       -v /tmp/.X11-unix/:/tmp/.X11-unix -v ~/.Xauthority:/root/.Xauthority \
       -e DISPLAY=$DISPLAY \
       deepracer-simulation
   ```

2. The container will initialize using the `ros_entrypoint_deepracer.sh` script.

3. Run the simulation:

```bash
# 1st terminal
ros2 launch deepracer_bringup nav_amcl_demo_sim.launch.py

# 2nd terminal
rviz2
```

## Notes

- Ensure that your host system has X11 forwarding enabled for GUI applications like RViz and Gazebo.
- The `DISPLAY` environment variable must be correctly set for rendering simulation visuals.
- Use `colcon build --symlink-install` during development to reflect code changes without rebuilding the image.

## Troubleshooting

There might be some warning / errors during the Gazebo + Rviz2 execution.  
Please, read [./about-transform-errors.md](./about-transform-errors.md)

## References

- [AWS DeepRacer GitHub Repository](https://github.com/aws-deepracer/aws-deepracer)
- [ROS2 Gazebo Integration](https://gazebosim.org/tutorials?tut=ros2_overview)
- [Docker Documentation](https://docs.docker.com/)
