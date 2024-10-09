FROM osrf/ros:noetic-desktop@sha256:637e06dd30895c510efc3ae579e07e59e4bea36eba48b3ba9aedf8afa236856d

RUN apt update && apt install -y \
    libopencv-dev \
    python3-opencv \
    libeigen3-dev \
    git \
    cmake \
    # Pangolin dependences
    libepoxy-dev

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