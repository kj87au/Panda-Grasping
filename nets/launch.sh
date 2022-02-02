#!/usr/bin/env bash

echo "MAKING IMAGE FOR PPR-NET"

NAME="ppr-net"
echo "IMAGE NAME: $NAME"

# Check to see if the image already exists
have_image="$(sudo docker image ls -a | grep -c $NAME)"
have_container="$(sudo docker container ls -a | grep -c $NAME)"
is_running="$(sudo docker container ls | grep -c $NAME)"

# DEBUG CODE
# echo "$have_image"
# echo "$have_container"
# echo "$is_running"

# Check for PPR-net image and if not found build it.
if [[ "$have_image" -eq 1 ]]; then  
  echo "Image Found!"
else
  echo "Building Image!"
  sudo docker build -f docker/$NAME.Dockerfile -t $NAME .
fi

if [[ "$have_container" -eq 1 ]]; then
  echo "Container Found!"
else
  echo "Starting Container"
  echo "sudo docker run -it --gpus all --rm -v $(cd ../../ && pwd):/panda_grasp --network="host" $NAME bash"
  sudo docker run -it --gpus all --rm -v $(cd ../../ && pwd):/panda_grasp --network="host" $NAME bash
  exit 0
fi

if [[ "$is_running" -eq 1 ]]; then
  echo "Container already running, exec into it!"
  
  # Check running containers
  con_name=$(sudo docker container ls | grep $NAME)
  con_name=${con_name:0:12}
  sudo docker exec -it con_name bash
else
  # Check stopped containers
  con_name=$(sudo docker container ls -a | grep $NAME)
  con_name=${con_name:0:12}
  
  echo "No running container found, Launching $con_name!"
  sudo docker start -i $con_name
  
  echo "Removing container $con_name"
  sudo docker container rm $con_name
fi
