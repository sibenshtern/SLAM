#!/bin/bash

function show_help() {
    echo "Usage: $0 [ARGUMENT]"
    echo
    echo "ARGUMENT:"
    echo "  ros           Connect to the ORB_SLAM3 container with ROS and source ROS setup"
    echo "  usual         Connect to the ORB_SLAM3 container without ROS"
    echo "  -h, --help    Display this help message"
}

if [ "$#" -lt 1 ]; then
    echo "Error: Invalid number of arguments."
    show_help
    exit 1
fi

CONTAINER_NAME=""

case "$1" in
    ros)
        CONTAINER_NAME="orb-slam-ros"
        ;;
    usual)
        CONTAINER_NAME="orb-slam-usual"
        ;;
    -h|--help)
        show_help
        exit 0
        ;;
    *)
        echo "Error: Unknown argument '$1'."
        show_help
        exit 1
        ;;
esac

if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "Connecting to the container '$CONTAINER_NAME'..."
    
    if [ "$1" == "ros" ]; then
        docker exec -it $CONTAINER_NAME bin/bash -c "source /opt/ros/noetic/setup.sh && exec bin/bash"
    else
        docker exec -it $CONTAINER_NAME bin/bash
    fi
else
    echo "Error: The container '$CONTAINER_NAME' is not running."
    exit 1
fi
