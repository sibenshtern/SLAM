CONTAINER_NAME=r2_ov_gui

docker exec -it $CONTAINER_NAME bash -c "source ./install/setup.bash && exec /bin/bash"