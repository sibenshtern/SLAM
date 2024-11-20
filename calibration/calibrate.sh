#!/bin/bash

# $1 - root folder
# $2 - name of bag file
# $3 - path to yaml file and name of yaml file
# $4 - 5 - models
# $6 - 7 - topics

xhost +
docker run -i --rm \
    -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$1/data:/data" \
    -d --name=kalibr \
    -v "$1/kalibr_inside.sh:/kalibr_inside.sh" kalibr bin/bash -c "source /opt/ros/noetic/setup.sh && roscore"

docker exec kalibr /bin/bash -c "source /opt/ros/noetic/setup.sh && ./kalibr_inside.sh $2 $3 $4 $5 $6 $7"

docker rm -f kalibr

python3 $1/scripts/import_pictures_for_undistortion.py $1/data/$2 $6 $7 $1/data/pictures

first_part=$(echo "$2" | cut -d '.' -f 1)

if [[ $4 = $5 ]] && [[ $5 = "pinhole-radtan" ]]; then
    python3 $1/scripts/undistortion_radtan.py $1/data/pictures/img_0.png $1/data/pictures/img_1.png $1/$first_part-camchain.yaml $1/data/pictures
elif [[ $4 == $5 ]] && [[ $5 = "pinhole-equi" ]]; then
    python3 $1/scripts/undistortion_equi.py $1/data/pictures/img_0.png $1/data/pictures/img_1.png $1/data/$first_part-camchain.yaml $1/data/pictures
else 
    echo "unsupported camera model"
fi
