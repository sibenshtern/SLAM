FROM osrf/ros:humble-desktop@sha256:2107f0aba78629a9a248820b11286b8a5aa007c31eee6378c1e34901bd8a8e09

SHELL [ "/bin/bash" , "-c" ]

RUN apt-get update && apt-get install -y \
    build-essential \
    libceres-dev \
    libgoogle-glog-dev \
    python3-dev 

ARG WORKSPACE_OV=/home/workspace/colcon_ws_ov

RUN source /opt/ros/humble/setup.bash \
    && mkdir -p ${WORKSPACE_OV}/src/ \
    && cd ${WORKSPACE_OV}/src/ \
    && git clone --depth 1 --branch v2.7 https://github.com/rpng/open_vins/

RUN sed -i 's/image_transport.h/image_transport.hpp/' ${WORKSPACE_OV}/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h \
    && sed -i 's/tf2_geometry_msgs.h/tf2_geometry_msgs.hpp/' ${WORKSPACE_OV}/src/open_vins/ov_msckf/src/ros/ROS2Visualizer.h \
    && sed -i 's/tf2_geometry_msgs.h/tf2_geometry_msgs.hpp/' ${WORKSPACE_OV}/src/open_vins/ov_msckf/src/ros/ROSVisualizerHelper.h

WORKDIR ${WORKSPACE_OV}