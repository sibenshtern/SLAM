XAUTH=/tmp/.docker.xauth

if [ $# -ge 1 ];
then
    docker run -it \
    --name=r2_ov_gui \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --volume="$1:/home/workspace/colcon_ws_ov/data" \
    --net=host \
    --privileged \
    r2_open_vins \
    bash
else
    docker run -it \
    --name=r2_ov_gui \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    --net=host \
    --privileged \
    r2_open_vins \
    bash
fi
