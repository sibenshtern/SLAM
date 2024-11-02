# ORB SLAM 3

## About

Docker files for building the ORB SLAM 3 algorithm in docker. Dockerfile builds a container with an algorithm without ROS, and Dockerfile.ROS — together with ROS

## How to build container
```bash
chmod +x build.sh
./build.sh <ros/usual>
```

## How to run container
```bash
chmod +x run.sh
./run.sh <ros/usual>
```

`run.sh` also has some options. Type `./run.sh --help` to see all of them.

## How to open external terminal to container

The container must be running

```bash
chmod +x exec.sh
./exec.sh <ros/usual>
```
