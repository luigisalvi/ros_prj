FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

WORKDIR /app

# Copia solo il workspace
COPY ros2_ws /app/ros2_ws
COPY ros_entrypoint.sh /ros_entrypoint.sh

# Setup dei permessi e creazione delle directory necessarie
RUN mkdir -p /app/ros2_ws/src/bringup_pkg/bringup_pkg && \
    touch /app/ros2_ws/src/bringup_pkg/bringup_pkg/__init__.py && \
    mkdir -p /app/ros2_ws/src/bringup_pkg/resource && \
    touch /app/ros2_ws/src/bringup_pkg/resource/bringup_pkg && \
    chmod -R 755 /app/ros2_ws

RUN chmod +x /ros_entrypoint.sh

# Modifica del ros_entrypoint.sh
RUN echo '#!/bin/bash' > /ros_entrypoint.sh && \
    echo 'set -e' >> /ros_entrypoint.sh && \
    echo 'source /opt/ros/foxy/setup.bash' >> /ros_entrypoint.sh && \
    echo 'cd /app/ros2_ws' >> /ros_entrypoint.sh && \
    echo 'colcon build' >> /ros_entrypoint.sh && \
    echo 'source /app/ros2_ws/install/setup.bash' >> /ros_entrypoint.sh && \
    echo 'exec "$@"' >> /ros_entrypoint.sh

# Build iniziale del workspace
RUN source /opt/ros/foxy/setup.bash && \
    cd /app/ros2_ws && \
    colcon build

# Entrypoint e comando
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash", "-c", "ros2 launch bringup_pkg all_nodes.launch.py"]