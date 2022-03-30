ARG ROS_DISTRO=foxy

FROM ros:${ROS_DISTRO}-ros-base

RUN adduser --gecos '' --disabled-password user
RUN groupadd wheel
RUN groupadd render
RUN usermod -a -G dialout,render,video,wheel,sudo user
RUN echo "%sudo ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN apt-get update && apt-get install -y build-essential python3 \
	ros-${ROS_DISTRO}-gazebo-ros-pkgs ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-joint-state-publisher ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-controller-manager ros-${ROS_DISTRO}-velocity-controllers ros-${ROS_DISTRO}-position-controllers \
    ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers ros-${ROS_DISTRO}-gazebo-ros2-control tmux micro \
    ros-${ROS_DISTRO}-rqt-common-plugins ros-${ROS_DISTRO}-joy python3-pip ros-${ROS_DISTRO}-vision-opencv

RUN pip3 install opencv-python

RUN chown /dev/

USER user
RUN HOME=/home/user
RUN touch ~/.sudo_as_admin_successful
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/user/.bashrc
RUN echo "source ~/ros/install/setup.bash" >> /home/user/.bashrc

WORKDIR /home/user
