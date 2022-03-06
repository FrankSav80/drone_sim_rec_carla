FROM ricardodeazambuja/ros2-galactic-desktop

USER ros2user
RUN mkdir -p /home/ros2user/carla-ros
WORKDIR /home/ros2user/carla-ros
RUN git clone --recurse-submodules https://github.com/ricardodeazambuja/carla-ros.git src/ros-bridge

# RUN python3 -m pip install numpy>=1.22.0
# RUN python3 -m pip install scipy>=1.7.3
RUN sudo -H python3 -m pip install carla==0.9.13
RUN sudo -H python3 -m pip install -r src/ros-bridge/requirements.txt

RUN sudo apt-get update
RUN /bin/bash -c 'source /opt/ros/galactic/setup.bash; rosdep update; rosdep install --from-paths src --ignore-src -r; colcon build'

RUN echo "export PATH=$PATH:/home/ros2user/.local/bin" >> /home/ros2user/.bashrc
RUN echo "source /home/ros2user/carla-ros/install/setup.bash" >> /home/ros2user/.bashrc