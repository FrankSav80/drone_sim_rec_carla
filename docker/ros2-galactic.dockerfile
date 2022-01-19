FROM ricardodeazambuja/ros2-galactic-desktop

USER ros2user
RUN mkdir -p /home/ros2user/carla-ros
WORKDIR /home/ros2user/carla-ros
RUN git clone --recurse-submodules https://github.com/ricardodeazambuja/carla-ros.git src/ros-bridge

RUN python3 -m pip install carla==0.9.13
RUN python3 -m pip install -r src/ros-bridge/requirements.txt

RUN sudo apt-get update
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r
RUN colcon build

RUN echo "source /home/ros2user/carla-ros/install/setup.bash" >> /home/ros2user/.bashrc
RUN echo "export PATH=$PATH:/home/ros2user/.local/bin" >> /home/ros2user/.bashrc