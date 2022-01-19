#!/bin/bash

echo
echo "Usage: run.sh <cmd to be executed, default bash> <container's name, default carla-ros2-<10 random chars>"
echo

shell=${1:-bash}

name=${2:-carla-ros2-$(echo $RANDOM | md5sum | head -c 10)}

echo "This is the container's name (and hostname): $name"
echo "Hint: this container uses avahi, so you should be able to access it by $name.local"

echo 
echo "This script mounted the current directory ($(pwd))" 
echo "inside the container at /home/ros2user/host."
echo "Therefore, remember that when you decide to use something like \"rm -rf *\" ;)"
echo 

docker run --rm -it \
             --name $name \
             --hostname=$name \
             --user $(id -u):$(id -g) \
             --volume $(pwd):/home/ros2user/host \
             --gpus 0 --env=NVIDIA_VISIBLE_DEVICES=0 --env=NVIDIA_DRIVER_CAPABILITIES=all --env=QT_X11_NO_MITSHM=1  \
             --group-add video --group-add sudo \
             --device=/dev/video0:/dev/video0 \
             --device=/dev/dri/card0:/dev/dri/card0 \
             --env=DISPLAY=$DISPLAY \
             --volume /tmp/.X11-unix:/tmp/.X11-unix \
             --workdir="/home/ros2user/" \
             --mount type=bind,source=/home/$USER/.bash_history,target=/home/ros2user/.bash_history \
             carla-ros2-bridge:galactic $shell