docker build --build-arg UID=$(id -u) --build-arg GID=$(id -g) \
  -f docker/ros2-galactic.dockerfile \
  -t carla-ros-bridge:galactic \
  github.com/ricardodeazambuja/carla-ros
