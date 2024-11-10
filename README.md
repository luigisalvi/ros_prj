## dev

docker compose -f ./docker-compose.dev.yaml up -d

## prod

docker compose -f ./docker-compose.prod.yaml up -d (now works, no differences)

## Run `ros2` command on container's terminal
1. open docker container's terminal 
2. type `bash` command
3. do the ROS sourcing by typing `source /opt/ros/foxy/setup.bash`
4. `ros2` command is ready to run