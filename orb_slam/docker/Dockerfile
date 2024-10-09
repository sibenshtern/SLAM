FROM ubuntu:20.04@sha256:4334ab5016f708cf4a15794e48ebea4c5666fd2bad49679badc4930bf42d9ccb

SHELL ["bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive 

RUN apt update && apt install -y \
    libopencv-dev \
    python3-opencv \
    libeigen3-dev \
    git \
    cmake \
    # Pangolin dependences
    g++ \
    libboost-all-dev \
    libssl-dev \
    libgl1-mesa-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libegl1-mesa-dev \
    libc++-dev \
    libepoxy-dev \
    libglew-dev \
    ninja-build

# install Pangolin
RUN cd tmp/ && \
    git clone --recursive https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    cmake -B build && \
    cmake --build build && \
    rm -rf Pangolin && \
    cd /

# install ORB-SLAM3
RUN mkdir /orb-slam && \
    cd /orb-slam && \
    git clone -b c++14_comp --depth=1 https://github.com/UZ-SLAMLab/ORB_SLAM3.git . && \
    sed -i 's/OpenCV 4.4/OpenCV 4.2/g' CMakeLists.txt && \
    chmod +x build.sh && \
    ./build.sh