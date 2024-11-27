import os
import csv
import pathlib
import argparse

import cv2
import rosbag
from cv_bridge import CvBridge


class NotEmptyDirectory(OSError):
    pass


def save_images_and_data(bag_file: rosbag.bag.Bag, topic: str,
                output_dir: pathlib.Path):
    bridge = CvBridge()
    data_directory = output_dir.joinpath('data')

    with open(f'{output_dir}/data.csv', mode='w', newline='') as file:
        csv_writer = csv.writer(file, delimiter=',')

        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            filename = f"{data_directory}/{t}.png"
            cv2.imwrite(filename,
                        bridge.imgmsg_to_cv2(msg,
                                             desired_encoding="passthrough"))
            csv_writer.writerow([t, filename])


def fix_timestamps(directory: pathlib.Path):
    with open(f'{directory}/cam0/data.csv', 'r') as file1:
        csv_reader1 = csv.reader(file1, delimiter=',')

        timestamps_1 = []
        for row in csv_reader1:
            timestamps_1.append(int(row[0]))

    with open(f'{directory}/cam1/data.csv', 'r') as file2:
        csv_reader2 = csv.reader(file2, delimiter=',')
        timestamps_2 = []
        for row in csv_reader2:
            timestamps_2.append(int(row[0]))

    timestamps = []
    for i in range(len(timestamps_1)):
        timestamps.append(min(timestamps_1[i], timestamps_2[i]) + round(
            abs(timestamps_1[i] - timestamps_2[i]) / 2))

    with open(f'{directory}/timestamps.txt', 'w') as file:
        for timestamp in timestamps:
            file.write(f"{int(timestamp)}\n")

    with open(f'{directory}/cam0/data.csv', 'w') as file1:
        csv_writer = csv.writer(file1, delimiter=',')

        for i in range(len(timestamps)):
            csv_writer.writerow(
                [int(timestamps[i]), f"{int(timestamps[i])}.png"])

    with open(f'{directory}/cam1/data.csv', 'w') as file1:
        csv_writer = csv.writer(file1, delimiter=',')

        for i in range(len(timestamps)):
            csv_writer.writerow(
                [int(timestamps[i]), f"{int(timestamps[i])}.png"])

    cam0_png = sorted(os.listdir(f'{directory}/cam0/data'),
                      key=lambda el: int(el.split('.')[0]))
    cam1_png = sorted(os.listdir(f'{directory}/cam1/data'),
                      key=lambda el: int(el.split('.')[0]))

    for i in range(len(timestamps)):
        os.rename(f"{directory}/cam0/data/{cam0_png[i]}",
                  f"{directory}/cam0/data/{timestamps[i]}.png")
        os.rename(f"{directory}/cam1/data/{cam1_png[i]}",
                  f"{directory}/cam1/data/{timestamps[i]}.png")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_file",
                        help="input ROS bag file")
    parser.add_argument("cam0_topic",
                        help="image topic of first camera")
    parser.add_argument("cam1_topic",
                        help="image topic of second camera")
    parser.add_argument("--output_dir", default="./dataset",
                        help="output directory for converted dataset")

    args = parser.parse_args()

    if not pathlib.Path(args.bag_file).exists():
        raise FileNotFoundError(f"{args.bag_file} does not exist.")
    elif not args.bag_file.endswith('.bag'):
        raise Exception(f"{args.bag_file} not a bag file.")

    root_folder = pathlib.Path(args.output_dir)
    if root_folder.exists() and len(os.listdir(root_folder)) > 0:
        raise NotEmptyDirectory(f"{root_folder} not empty.")

    cam0_directory = root_folder.joinpath('mav0').joinpath('cam0')
    cam1_directory = root_folder.joinpath('mav0').joinpath('cam1')

    os.makedirs(cam0_directory.joinpath('data'))
    os.makedirs(cam1_directory.joinpath('data'))

    bag_file = rosbag.Bag(args.bag_file, 'r')

    save_images_and_data(bag_file, args.cam0_topic, cam0_directory)
    save_images_and_data(bag_file, args.cam1_topic, cam1_directory)

    bag_file.close()

    fix_timestamps(root_folder.joinpath('mav0'))


if __name__ == '__main__':
    main()
