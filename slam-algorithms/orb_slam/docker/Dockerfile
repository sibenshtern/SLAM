FROM ubuntu:20.04

SHELL ["bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive 

RUN apt update && apt install -y \
    libopencv-dev \
    libeigen3-dev \
    git \
    cmake \
    # Pangolin dependences
    g++ \
    libboost-serialization-dev \
    python3-setuptools \
    libssl-dev \
    libgl1-mesa-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libegl1-mesa-dev \
    libc++-dev \
    libepoxy-dev \
    libglew-dev \
    ninja-build && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# install Pangolin
RUN cd tmp/ && \
    git clone --recursive https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    cmake -B build && \
    cmake --build build && \
    rm -rf Pangolin
    
# install ORB-SLAM3
RUN mkdir /orb-slam && \
    cd /orb-slam && \
    git clone -b c++14_comp --depth=1 https://github.com/UZ-SLAMLab/ORB_SLAM3.git . && \
    sed -i 's/OpenCV 4.4/OpenCV 4.2/g' CMakeLists.txt && \
    chmod +x build.sh && \
    ./build.sh