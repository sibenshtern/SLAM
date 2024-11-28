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
    data_directory = output_dir / 'data'

    with open(output_dir / 'data.csv', mode='w', newline='') as file:
        csv_writer = csv.writer(file, delimiter=',')

        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            filename = data_directory / f"{t}.png"
            cv2.imwrite(filename,
                        bridge.imgmsg_to_cv2(msg,
                                             desired_encoding="passthrough"))
            csv_writer.writerow([t, filename])


def read_timestamps(filename: pathlib.Path) -> list[int]:
    result = []

    with open(filename, 'r') as file1:
        csv_reader1 = csv.reader(file1, delimiter=',')

        for row in csv_reader1:
            result.append(int(row[0]))

    return result


def write_timestamps(filename: pathlib.Path, timestamps: list[int]) -> None:
    with open(filename, 'w') as file1:
        csv_writer = csv.writer(file1, delimiter=',')

        for i in range(len(timestamps)):
            csv_writer.writerow(
                [int(timestamps[i]), f"{int(timestamps[i])}.png"])


def fix_timestamps(directory: pathlib.Path):
    timestamps_1 = read_timestamps(directory / 'cam0' / 'data.csv')
    timestamps_2 = read_timestamps(directory / 'cam1' / 'data.csv')

    timestamps = []
    for i in range(len(timestamps_1)):
        timestamps.append(min(timestamps_1[i], timestamps_2[i]) + round(
            abs(timestamps_1[i] - timestamps_2[i]) / 2))

    with open(directory / 'timestamps.txt', 'w') as file:
        for timestamp in timestamps:
            file.write(f"{int(timestamp)}\n")

    write_timestamps(directory / 'cam0' / 'data.csv', timestamps)
    write_timestamps(directory / 'cam1' / 'data.csv', timestamps)

    cam0_png = sorted((directory / 'cam0' / 'data').iterdir(),
                      key=lambda el: int(el.split('.')[0]))
    cam1_png = sorted((directory / 'cam1' / 'data').iterdir(),
                      key=lambda el: int(el.split('.')[0]))

    for i in range(len(timestamps)):
        os.rename(directory / 'cam0' / 'data' / f"{cam0_png[i]}",
                  directory / "cam0" / "data" / f"{timestamps[i]}.png")
        os.rename(directory / 'cam1' / 'data' / f"{cam0_png[i]}",
                  directory / "cam1" / "data" / f"{timestamps[i]}.png")


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

    cam0_directory = root_folder / 'mav0'/ 'cam0'
    cam1_directory = root_folder / 'mav0'/ 'cam1'

    os.makedirs(cam0_directory / 'data')
    os.makedirs(cam1_directory / 'data')

    bag_file = rosbag.Bag(args.bag_file, 'r')

    save_images_and_data(bag_file, args.cam0_topic, cam0_directory)
    save_images_and_data(bag_file, args.cam1_topic, cam1_directory)

    bag_file.close()

    fix_timestamps(root_folder / 'mav0')


if __name__ == '__main__':
    main()
