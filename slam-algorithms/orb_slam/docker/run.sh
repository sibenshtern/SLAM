#!/bin/bash

XAUTH=/tmp/.docker.xauth

function show_help() {
    echo "Usage: $0 [ARGUMENT] [--volume=<VOLUME>:<VOLUME_IN_DOCKER>]"
    echo
    echo "ARGUMENT:"
    echo "  ros                              Build the ORB_SLAM3 with ROS"
    echo "  usual                            Build the ORB_SLAM3 without ROS"
    echo "  -h, --help                       Display this help message"
    echo
    echo "--volume=<VOLUME>:<DOCKER_VOLUME>  (optional) Specify a directory to mount as a volume in the container."
}

if [ "$#" -lt 1 ]; then
    echo "Error: Invalid number of arguments."
    show_help
    exit 1
fi

VOLUME=""
HOST_VOLUME=""
DOCKER_VOLUME=""
MODE=""

for arg in "$@"; do
    case $arg in
        --volume=*)
            VOLUME="${arg#*=}"
            HOST_VOLUME="${VOLUME%%:*}"
            DOCKER_VOLUME="${VOLUME##*:}"
            ;;
        ros|usual|-h|--help)
            MODE="$arg"
            ;;
        *)
            echo "Error: Unknown argument '$arg'."
            show_help
            exit 1
            ;;
    esac
done

case "$MODE" in
    ros)
        echo "Starting ORB_SLAM3 with ROS..."
        docker run -it \
            --name=orb-slam-ros \
            --env="DISPLAY=$DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            --platform=linux/amd64 \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            --volume="/Users/egorivanov/.Xauthority" \
            --env="XAUTHORITY=$XAUTH" \
            --volume="$XAUTH:$XAUTH" \
            ${VOLUME:+--volume="$HOST_VOLUME:$DOCKER_VOLUME"} \
            --net=host \
            --privileged \
            orb_slam:ros
        echo "Delete container with name 'orb-slam-ros'"
        docker rm orb-slam-ros
        ;;
    usual)
        echo "Starting ORB_SLAM3 without ROS..."
        docker run -it \
            --name=orb-slam-usual \
            --env="DISPLAY=$DISPLAY" \
            --env="QT_X11_NO_MITSHM=1" \
            --platform=linux/amd64 \
            --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
            --volume="/Users/egorivanov/.Xauthority" \
            --env="XAUTHORITY=$XAUTH" \
            --volume="$XAUTH:$XAUTH" \
            ${VOLUME:+--volume="$HOST_VOLUME:$DOCKER_VOLUME"} \
            --net=host \
            --privileged \
            orb_slam:usual
        echo "Delete container with name 'orb-slam-usual'"
        docker rm orb-slam-usual
        ;;
    -h|--help)
        show_help
        ;;
    *)
        echo "Error: Build mode not specified."
        show_help
        exit 1
        ;;
esac


