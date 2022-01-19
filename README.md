# ROS/ROS2 bridge for CARLA simulator

[![Actions Status](https://github.com/carla-simulator/ros-bridge/workflows/CI/badge.svg)](https://github.com/carla-simulator/ros-bridge)
[![Documentation](https://readthedocs.org/projects/carla/badge/?version=latest)](http://carla.readthedocs.io)
[![GitHub](https://img.shields.io/github/license/carla-simulator/ros-bridge)](https://github.com/carla-simulator/ros-bridge/blob/master/LICENSE)
[![GitHub release (latest by date)](https://img.shields.io/github/v/release/carla-simulator/ros-bridge)](https://github.com/carla-simulator/ros-bridge/releases/latest)

 This ROS package is a bridge that enables two-way communication between ROS and CARLA. The information from the CARLA server is translated to ROS topics. In the same way, the messages sent between nodes in ROS get translated to commands to be applied in CARLA.

![rviz setup](./docs/images/ad_demo.png "AD Demo")

**This version requires CARLA 0.9.13**

## Features

- Provide Sensor Data (Lidar, Semantic lidar, Cameras (depth, segmentation, rgb, dvs), GNSS, Radar, IMU)
- Provide Object Data (Transforms (via [tf](http://wiki.ros.org/tf)), Traffic light status, Visualization markers, Collision, Lane invasion)
- Control AD Agents (Steer/Throttle/Brake)
- Control CARLA (Play/pause simulation, Set simulation parameters)

## Getting started and documentation

Installation instructions and further documentation of the ROS bridge and additional packages are found [__here__](https://carla.readthedocs.io/projects/ros-bridge/en/latest/).

## My modifications
I modified this repo to allow you to use hostnames (avahi) and to allow you to download just one script to use together with a headless CARLA server from another docker image.  
It's expected that you are running linux and you have docker and nvidia-docker2 installed (and working).

Download the script to launch the headless server:
```
$ wget https://raw.githubusercontent.com/ricardodeazambuja/carla-simulator-python/main/launch_simulator_docker.sh
```
And launch it:
```
$ source launch_simulator_docker.sh
```

Now, in a new terminal, download this second script:
```
$ wget https://raw.githubusercontent.com/ricardodeazambuja/carla-ros/master/docker/build_ros2-galactic.sh
```

Build your image (it will download another script after that):
```
$ source build_ros2-galactic.sh
```

And finally launch the CARLA ROS2 stuff :)
```
$ source run_ros2-galactic.sh
```

If everything went as expected, you should be able to test your container with this:
```
$ ros2 launch carla_ros_bridge carla_ros_bridge_with_example_ego_vehicle.launch.py
```

When you need to open more terminals inside the same container (considering the name is `carla-ros2-8589d9bd2d`):
```
$ docker exec -it carla-ros2-8589d9bd2d bash
```

And if you just want to execute something (e.g. `ros2 topic list`):
```
$ docker exec -t carla-ros2-8589d9bd2d bash -i -c "ros2 topic list"
```