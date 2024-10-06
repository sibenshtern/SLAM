FROM osrf/ros:humble-desktop

SHELL [ "/bin/bash" , "-c" ]

RUN apt-get update && apt-get install -y nano git
RUN apt-get install -y cmake libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev libceres-dev
RUN apt-get install -y python3-dev python3-matplotlib python3-numpy python3-psutil python3-tk python3-colcon-common-extensions
RUN apt-get install -y ros-humble-cv-bridge ros-humble-image-transport ros-humble-ros2bag ros-humble-rosbag2*

RUN apt-get install -y ssh build-essential gcc g++ \
    cmake && apt-get clean

RUN mkdir -p ~/workspace/colcon_ws_ov/src/ \
    && cd ~/workspace/colcon_ws_ov/src/ \
    && git clone https://github.com/rpng/open_vins/

RUN sed -i 's/image_transport.h/image_transport.hpp/' ~/workspace/colcon_ws_ov/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h
RUN sed -i 's/tf2_geometry_msgs.h/tf2_geometry_msgs.hpp/' ~/workspace/colcon_ws_ov/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h
RUN sed -i 's/tf2_geometry_msgs.h/tf2_geometry_msgs.hpp/' ~/workspace/colcon_ws_ov/src/open_vins/ov_msckf/src/ros/ROSVisualizerHelper.h

RUN sudo mkdir -p /var/cache/swap/
RUN sudo dd if=/dev/zero of=/var/cache/swap/swap0 bs=64M count=64
RUN sudo chmod 0600 /var/cache/swap/swap0
RUN sudo mkswap /var/cache/swap/swap0
RUN sudo swapon -s

RUN source /opt/ros/humble/setup.bash 

WORKDIR /root/workspace/colcon_ws_ov

RUN source install/setup.bash
