FROM ros:noetic-ros-core

RUN adduser --gecos '' --disabled-password user
RUN groupadd wheel
RUN groupadd render
RUN usermod -a -G dialout,render,video,wheel,sudo user
RUN echo "%sudo ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN apt-get update && apt-get install -y build-essential python3 \
	ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

USER user
RUN HOME=/home/user
RUN touch ~/.sudo_as_admin_successful
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/user/.bashrc
RUN echo "source /home/user/ros/devel/setup.bash" >> /home/user/.bashrc

WORKDIR /home/user
