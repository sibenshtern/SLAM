#!/bin/bash

XAUTH=/tmp/.docker.xauth

function show_help() {
    echo "Usage: $0 [ARGUMENT] [--volume=<VOLUME>:<VOLUME_IN_DOCKER>] [--remove=<true|false>] [--display=<DISPLAY>]"
    echo
    echo "ARGUMENT:"
    echo "  ros                              Build the ORB_SLAM3 with ROS"
    echo "  usual                            Build the ORB_SLAM3 without ROS"
    echo "  -h, --help                       Display this help message"
    echo
    echo "--volume=<VOLUME>:<DOCKER_VOLUME>  (optional) Specify a directory to mount as a volume in the container."
    echo "--remove=<true|false>              (optional, default: true) Automatically remove the container after it stops."
    echo "--display=<DISPLAY>                (optional, default: $DISPLAY) Specify the DISPLAY environment variable."
}

# Set default values for optional arguments
REMOVE=true
DISPLAY_VALUE="$DISPLAY"
VOLUME=""
HOST_VOLUME=""
DOCKER_VOLUME=""
MODE=""

# Parse command-line arguments
for arg in "$@"; do
    case $arg in
        --volume=*)
            VOLUME="${arg#*=}"
            HOST_VOLUME="${VOLUME%%:*}"
            DOCKER_VOLUME="${VOLUME##*:}"
            ;;
        --remove=*)
            REMOVE="${arg#*=}"
            ;;
        --display=*)
            DISPLAY_VALUE="${arg#*=}"
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

# Check that a valid mode was provided
if [ -z "$MODE" ]; then
    echo "Error: Build mode not specified."
    show_help
    exit 1
fi

# Determine the container settings based on the mode
CONTAINER_NAME=""
IMAGE_TAG=""

case "$MODE" in
    ros)
        CONTAINER_NAME="orb-slam-ros"
        IMAGE_TAG="orb_slam:ros"
        ;;
    usual)
        CONTAINER_NAME="orb-slam-usual"
        IMAGE_TAG="orb_slam:usual"
        ;;
    -h|--help)
        show_help
        exit 0
        ;;
    *)
        echo "Error: Unknown argument '$MODE'."
        show_help
        exit 1
        ;;
esac

# Start the Docker container with the appropriate settings
echo "Starting ORB_SLAM3 with mode '$MODE'..."
docker run -it \
    --name=$CONTAINER_NAME \
    --env="DISPLAY=$DISPLAY_VALUE" \
    --env="QT_X11_NO_MITSHM=1" \
    --platform=linux/amd64 \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    ${VOLUME:+--volume="$HOST_VOLUME:$DOCKER_VOLUME"} \
    --net=host \
    --privileged \
    $IMAGE_TAG

# Remove the container if --remove is true
if [ "$REMOVE" = "true" ]; then
    echo "Removing container '$CONTAINER_NAME'..."
    docker rm $CONTAINER_NAME
fi
