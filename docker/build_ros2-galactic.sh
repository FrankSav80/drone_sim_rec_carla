echo "Building the image carla-ros-bridge:galactic..."
docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) \
  -f docker/ros2-galactic.dockerfile \
  -t carla-ros2-bridge:galactic \
  github.com/ricardodeazambuja/carla-ros

echo "Downloading the script you use to launch the container..."
wget https://raw.githubusercontent.com/ricardodeazambuja/carla-ros/master/docker/run_ros2-galactic.sh