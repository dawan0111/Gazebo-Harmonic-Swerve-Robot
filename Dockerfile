FROM rwthika/ros2-cuda:jazzy-desktop-full-v24.12

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
  ros-$ROS_DISTRO-ros-gz \
  ros-$ROS_DISTRO-gz-ros2-control \
  ros-$ROS_DISTRO-joint-state-publisher \
  ros-$ROS_DISTRO-joint-state-publisher-gui \
  ros-$ROS_DISTRO-ros2-control \
  ros-$ROS_DISTRO-ros2-controllers \
  && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

RUN git clone https://github.com/dawan0111/Gazebo-Harmonic-Swerve-Robot.git amr_description

WORKDIR /ros2_ws
RUN /bin/bash -c '. /opt/ros/jazzy/setup.bash; colcon build'

RUN echo "source /ros2_ws/install/local_setup.bash" >> /root/.bashrc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]