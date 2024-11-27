# Unpacking data from a rosbag file

## Content

`convert_bag.py` — script for unpacking images from rosbag file

`merge_imu.py` — script for merging imu from rosbag file from RealSense T265 camera

## Usage

```shell
python3 convert_bag.py [--output_dir OUTPUT_DIR] bag_file cam0_topic cam1_topic
```

```text
usage: convert_bag.py [-h] [--output_dir OUTPUT_DIR] bag_file cam0_topic cam1_topic

positional arguments:
  bag_file                 input ROS bag file
  cam0_topic               image topic of first camera
  cam1_topic               image topic of second camera

options:
  -h, --help               show this help message and exit
  --output_dir OUTPUT_DIR  output directory for converted dataset
```

```shell
python3 merge_imu.py input_bag output_bag
```