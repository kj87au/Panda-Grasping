#!/usr/bin/env bash

NAME="giga-net"
echo "MAKING IMAGE, NAME = $NAME"

# Create a non-root user
USER="giga"
ID="1000"
GID="100"

# Check to see if the image already exists
have_image="$(sudo docker image ls -a | grep -c $NAME)"
have_container="$(sudo docker container ls -a | grep -c $NAME)"
is_running="$(sudo docker container ls | grep -c $NAME)"

# DEBUG CODE
# echo "$have_image"
# echo "$have_container"
# echo "$is_running"

# Check for image and if not found build it.
if [[ "$have_image" -eq 1 ]]; then  
  echo "Image Found!"
else
  echo "Building Image!"
  sudo docker build -f docker/$NAME.Dockerfile -t $NAME --build-arg username=$USER \
  --build-arg uid=$UID \
  --build-arg gid=$GID \
  ../ 
fi

if [[ "$have_container" -eq 1 ]]; then
  echo "Container Found!"
else
  echo "Starting Container"
  echo "sudo docker run -it --gpus all --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -v $(cd ../../ && pwd):/panda_grasp --network="host" $NAME bash"
  sudo docker run -it --gpus all --env="DISPLAY" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -v $(cd ../../ && pwd):/panda_grasp --network="host" $NAME bash
  exit 0
fi

if [[ "$is_running" -eq 1 ]]; then
  echo "Container already running, exec into it!"

  # Check running containers
  con_name=$(sudo docker container ls | grep $NAME)
  con_name=${con_name:0:12}
  xhost +local:docker
  sudo docker exec -it $con_name bash
else
  # Check stopped containers
  con_name=$(sudo docker container ls -a | grep $NAME)
  con_name=${con_name:0:12}

  echo "No running container found, Launching $con_name!"
  xhost +local:docker
  sudo docker start -i $con_name
fi
