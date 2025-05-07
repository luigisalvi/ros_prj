# Mobile Robotics Project 2024/2025

Questo repository ha l'obiettivo di documentare tutto il lavoro effettuato per il progetto di robotica 2.

## Prime sperimentazioni

Sono state fatte le prime sperimentazioni di sviluppo con ROS2 in Docker attraverso lo sviluppo di più nodi personalizzati inter-comunicanti. è possibile lanciare il progetto in locale seguendo le indicazioni presenti in [ros2-dev](./dev.md). Questo sottoprogetto comprende un ambiente di sviluppo interativvo con cui è possibile lanciare i nodi, modificare il codice sorgente e fare diverse sperimentazioni.

Successivamente è stata studiata la possibilità di erogare un gruppo autonomo di nodi ros2 come servizio containerizzato. il risultato di questa sperimentazione è eseguibile seguendi le indicazioni in [ros2-prod](./prod.md).

## AWS Deepracer

Successivamente è stato studiato l'ecosistema AWS Deepracer.

Il veicolo AWS DeepRacer Evo è una piattaforma di sterzo Ackermann a 4 ruote in scala 1/18 con Wi-Fi, dotata di due telecamere RGB e un sensore LiDAR. Questo repository contiene i file di configurazione e di avvio per abilitare lo stack di navigazione ROS su AWS DeepRacer e controllare il veicolo tramite teleop-twist-keyboard, insieme ai componenti principali per integrare AWS DeepRacer con lo stack di navigazione ROS. Per informazioni dettagliate, consultare [Introduzione allo stack di navigazione ROS con AWS DeepRacer Evo](https://github.com/aws-deepracer/aws-deepracer/blob/main/introduction-to-the-ros-navigation-stack-using-aws-deepracer-evo.md).

[in laboratorio] sono state effettuate varie sperimentazioni per l'utilizzo ed il controllo dell'aws deepracer.

è stato configurato un container docker pronto per l'uso all'interno del quale è possibile lanciare una simulazione Gazebo + Rviz2 sviluppata ufficialmente da AWS. è possibile utilizzare il Dockerfile sviluppato da noi appositamente seguendo le istruzioni in [./aws-deepracer.md](./aws-deepracer.md).

Inoltre è possibile creare un container interattivo all'interno del quale sperimentare le stesse funzionalità:

```bash
xhost +
docker run -it --ipc=host --net=host --gpus all -e DISPLAY=$DISPLAY -v /tmp/.X11-unix/:/tmp/.X11-unix -v ~/.Xauthority:/root/.Xauthority -e XAUTHORITY=/root/.Xauthority osrf/ros:foxy-desktop bash
sudo apt update

sudo apt install ros-foxy-gazebo-ros-pkgs python3-rosinstall ros-foxy-rviz2

mkdir -p deepracer_nav2_ws
cd deepracer_nav2_ws
git clone https://github.com/aws-deepracer/aws-deepracer.git

rosdep install -i -r -y --from-paths .
cd aws-deepracer
rosinstall deepracer_description
cd gazebo_ros2_control && git reset --hard 04b2c6f0ff0e977b6fc6af5dfc5e96e5bdd570d0 && cd ..

colcon build

cd environment/deepracer_nav2_ws/aws-deepracer
export GAZEBO_RESOURCE_PATH=/usr/share/gazebo-11:{$GAZEBO_RESOURCE_PATH}
source install/setup.bash

# 1st terminal
ros2 launch deepracer_bringup nav_amcl_demo_sim.launch.py

# 2nd terminal
rviz2
```

---

> SALVI Luigi, SANTILIO Nicolo
