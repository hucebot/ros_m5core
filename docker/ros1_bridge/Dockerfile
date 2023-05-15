
FROM ros:noetic


# https://docs.ros.org/en/humble/How-To-Guides/Using-ros1_bridge-Jammy-upstream.html
ENV DEBIAN_FRONTEND="noninteractive" TZ="Europe/Paris"


RUN apt update && apt install -y software-properties-common &&  add-apt-repository universe
RUN apt install -y curl python3-pip git
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null


RUN python3 -m pip install -U colcon-common-extensions vcstool

RUN mkdir -p ~/ros2_humble/src \
 && cd ~/ros2_humble \
 && vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src


RUN apt update && apt-get install -y libignition-math6-dev
RUN apt update && apt install -y python3-catkin-pkg-modules
RUN apt upgrade -y
RUN rosdep update && cd ~/ros2_humble/ && rosdep install --from-paths src --ignore-src -y --skip-keys "fastcdr rti-connext-dds-6.0.1 urdfdom_headers"

RUN cd ~/ros2_humble/ && colcon build --symlink-install

# Create a workspace for the ros1_bridge
RUN mkdir -p ~/ros1_bridge/src \
 && cd ~/ros1_bridge/src \
 && git clone https://github.com/ros2/ros1_bridge \
 && cd ~/ros1_bridge \
 && /bin/bash -c "source /opt/ros/noetic/setup.bash && source /root/ros2_humble/install/local_setup.bash && colcon build  --event-handlers console_direct+ --cmake-args -DCMAKE_VERBOSE_MAKEFILE=ON"

