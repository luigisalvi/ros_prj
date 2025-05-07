# Production Setup Documentation

This document provides an overview of the `Dockerfile.prod` and `docker-compose.prod.yaml` files used for deploying the ROS2 workspace in a production environment.

---

## `Dockerfile.prod`

The `Dockerfile.prod` is designed to create a lightweight and optimized Docker image for running the ROS2 application in production.

### Key Features:

- **Base Image**: Uses a stable ROS2 distribution (e.g., `osrf/ros:<distro>`).
- **Workspace Setup**:
  - Copies the `ros2_ws` directory into the container.
  - Builds the workspace using `colcon build`.
- **Environment Variables**:
  - Sets up necessary environment variables for ROS2.
- **Entrypoint**:
  - Configures the container to run the ROS2 launch file automatically.

### Example Content:

```dockerfile
# filepath: Dockerfile.prod
FROM osrf/ros:foxy

# Set working directory
WORKDIR /app

# Copy ROS2 workspace
COPY ./ros2_ws /app/ros2_ws

# Install dependencies and build the workspace
RUN apt-get update && apt-get install -y \
    python3-colcon-common-extensions && \
    source /opt/ros/foxy/setup.bash && \
    colcon build --workspace /app/ros2_ws

# Source the workspace setup
RUN echo "source /app/ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command to run the application
CMD ["bash", "-c", "source /app/ros2_ws/install/setup.bash && ros2 launch bringup_pkg all_nodes.launch.py"]
```

---

## `docker-compose.prod.yaml`

The `docker-compose.prod.yaml` file defines the services and configurations for running the ROS2 application in a production environment.

### Key Features:

- **Single Service**:
  - Combines all nodes into a single container for simplicity.
- **Volumes**:
  - No local volume mounts to ensure the container is self-contained.
- **Environment Variables**:
  - Configures necessary variables for ROS2 runtime.
- **Restart Policy**:
  - Ensures the container restarts automatically in case of failure.

### Example Content:

```yaml
# filepath: docker-compose.prod.yaml
version: "3.8"

services:
  ros2_system:
    build:
      context: .
      dockerfile: Dockerfile.prod
    container_name: ros2_production_container
    environment:
      - DISPLAY
    restart: always
    tty: true
    command: >
      bash -c "
        source /app/ros2_ws/install/setup.bash &&
        ros2 launch bringup_pkg all_nodes.launch.py
      "
```

---

## Usage Instructions

### Build and Run the Production Environment

1. **Build the Docker Image**:
   ```bash
   docker compose -f docker-compose.prod.yaml build
   ```
2. **Start the Container**:
   ```bash
   docker compose -f docker-compose.prod.yaml up -d
   ```
3. **Check Logs**:
   ```bash
   docker logs ros2_production_container
   ```

### Stop the Container

```bash
docker compose -f docker-compose.prod.yaml down
```

---

## Notes

- Ensure all dependencies are installed in the production image.
- The production setup does not include live code updates; rebuild the image for changes.
- Use a stable ROS2 distribution for production to avoid compatibility issues.

---

## References

- [ROS2 Docker Documentation](https://docs.ros.org/en/foxy/Tutorials/Docker.html)
- [Docker Compose Documentation](https://docs.docker.com/compose/)
