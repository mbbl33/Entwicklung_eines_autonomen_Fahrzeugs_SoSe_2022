FROM ros:noetic-ros-core

RUN adduser --gecos '' --disabled-password user
RUN groupadd wheel
RUN usermod -a -G dialout,wheel,sudo user
RUN echo "%sudo ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER user
RUN HOME=/home/user
RUN touch ~/.sudo_as_admin_successfull
RUN echo "source /opt/ros/noetic/setup.bash" >> /home/user/.bashrc
RUN echo "source /home/user/ros/devel/setup.bash" >> /home/user/.bashrc

WORKDIR /home/user
