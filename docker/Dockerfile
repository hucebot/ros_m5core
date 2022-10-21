FROM espressif/idf:release-v4.4

ENV DEBIAN_FRONTEND noninteractive
RUN echo "Set disable_coredump false" >> /etc/sudo.conf
RUN apt update -q && \
    apt install -yq curl gnupg2 lsb-release gosu nano
    
#RUN mount -o remount,rw /dev/mapper/system-root /etc/timezone 
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN bash -c 'echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null' 

RUN apt update && apt upgrade -yq && apt install  -yq  ros-foxy-ros-base
#RUN pip3 install catkin_pkg lark-parser empy colcon-common-extensions

RUN rm -rf /var/lib/apt/lists/*

RUN cd /root && git clone --recursive https://github.com/hucebot/ros_m5core --progress
#RUN bash -c "cd /root/ros_m5core &&  source /opt/esp/idf/export.sh && idf.py build"
