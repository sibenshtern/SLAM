#!/bin/bash

# Function to display help
function show_help() {
    echo "Usage: $0 [ARGUMENT]"
    echo
    echo "ARGUMENT:"
    echo "  ros           Build the ORB_SLAM3 with ROS"
    echo "  usual         Build the ORB_SLAM3 without ROS"
    echo "  -h, --help    Display this help message"
}

# Check for the number of arguments
if [ "$#" -ne 1 ]; then
    echo "Error: Invalid number of arguments."
    show_help
    exit 1
fi

# Determine the build type
case "$1" in
    ros)
        echo "Starting ORB_SLAM3 build with ROS..."
        docker build --platform=linux/amd64 -t orb_slam:ros -f Dockerfile.ROS .
        ;;
    usual)
        echo "Starting ORB_SLAM3 build without ROS..."
        docker build --platform=linux/amd64 -t orb_slam:usual -f Dockerfile .
        ;;
    -h|--help)
        show_help
        ;;
    *)
        echo "Error: Unknown argument '$1'."
        show_help
        exit 1
        ;;
esac
