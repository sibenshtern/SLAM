FROM osrf/ros:humble-desktop

SHELL [ "/bin/bash" , "-c" ]

RUN apt-get update && apt-get install -y \
    build-essential \
    libceres-dev \
    libgoogle-glog-dev \
    python3-dev 

ARG WORKSPACE=/home/workspace/colcon_ws_ov

RUN mkdir -p ${WORKSPACE}/src/ \
    && cd ${WORKSPACE}/src/ \
    && git clone --depth 1 --branch v2.7 https://github.com/rpng/open_vins/

RUN sed -i 's/image_transport.h/image_transport.hpp/' ${WORKSPACE}/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h \
    && sed -i 's/tf2_geometry_msgs.h/tf2_geometry_msgs.hpp/' ${WORKSPACE}/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h \
    && sed -i 's/tf2_geometry_msgs.h/tf2_geometry_msgs.hpp/' ${WORKSPACE}/src/open_vins/ov_msckf/src/ros/ROSVisualizerHelper.h

RUN source /opt/ros/humble/setup.bash 

WORKDIR ${WORKSPACE}
