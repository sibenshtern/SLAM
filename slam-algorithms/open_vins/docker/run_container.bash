XAUTH=/tmp/.docker.xauth
VOLUME=""
DOCKER_VOLUME="/home/workspace/colcon_ws_ov/data"
CONTAINER_NAME=r2_ov_gui
IMAGE_NAME=r2_open_vins

if [ $# -ge 1 ]
then
    VOLUME="--volume="$1:$DOCKER_VOLUME"";
fi

docker run -it \
    --name=$CONTAINER_NAME \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    $VOLUME \
    --net=host \
    --privileged \
    --rm \
    $IMAGE_NAME \
    bash -c "source ./install/setup.bash && exec /bin/bash"
