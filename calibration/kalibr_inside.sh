#!/bin/bash

# $1 - name of bag file
# $2 - path to yaml file and name of yaml file
# $3 - 4 - models
# $5 - 6 - topics

source /catkin_ws/devel/setup.bash

rosrun kalibr kalibr_calibrate_cameras \
    --bag /data/$1 --target /data/$2 \
    --models $3 $4 \
    --topics $5 $6 

echo "calibrate finished"

first_part=$(echo "$1" | cut -d '.' -f 1)

chmod a+wr /data/$first_part-camchain.yaml /data/$first_part-report-cam.pdf

exit
