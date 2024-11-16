import os
import csv
import pathlib
import argparse

import cv2
import rosbag
from cv_bridge import CvBridge


class NotEmptyDirectory(OSError):
    pass


class Dataset:

    def __init__(self, root_folder: pathlib.Path | str | os.PathLike[str]):
        self.root_folder = root_folder
        if not isinstance(root_folder, pathlib.Path):
            self.root_folder = pathlib.Path(root_folder)

        if self.root_folder.exists() and len(os.listdir()) > 0:
            raise NotEmptyDirectory(f"{self.root_folder} not empty.")

        self.cam0_directory = self.root_folder.joinpath('cam0')
        self.__cam0_directory = self.cam0_directory.joinpath('data')

        self.cam1_directory = self.root_folder.joinpath('cam1')
        self.__cam1_directory = self.cam1_directory.joinpath('data')

        self.imu_directory = self.root_folder.joinpath('imu0')

        self.__create_directories()

    def __create_directories(self):
        os.makedirs(self.cam0_directory)
        os.makedirs(self.__cam0_directory)

        os.makedirs(self.cam1_directory)
        os.makedirs(self.__cam1_directory)

        os.mkdir(self.imu_directory)


def save_images_and_data(bag_file: rosbag.bag.Bag, topic: str,
                output_dir: pathlib.Path):
    bridge = CvBridge()
    data_directory = output_dir.joinpath('data')

    with open(f'{output_dir}/data.csv', mode='w', newline='') as file:
        file.write('#timestamp [ns],filename\n')
        csv_writer = csv.writer(file, delimiter=',')

        for topic, msg, t in bag_file.read_messages(topics=[topic]):
            filename = f"{data_directory}/{t}.png"
            cv2.imwrite(filename,
                        bridge.imgmsg_to_cv2(msg,
                                             desired_encoding="passthrough"))
            csv_writer.writerow([t, filename])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_file",
                        help="input ROS bag file")
    parser.add_argument("cam0_topic",
                        help="image topic of first camera")
    parser.add_argument("cam1_topic",
                        help="image topic of second camera")
    parser.add_argument("imu_topic",
                        help="imu topic in bag file")
    parser.add_argument("--output_dir", default="./dataset",
                        help="output directory for converted dataset")

    args = parser.parse_args()

    if not pathlib.Path(args.bag_file).exists():
        raise FileNotFoundError(f"{args.bag_file} does not exist.")
    elif not args.bag_file.endswith('.bag'):
        raise Exception(f"{args.bag_file} not a bag file.")

    dataset = Dataset(args.output_dir)

    bag_file = rosbag.Bag(args.bag_file, 'r')

    save_images_and_data(bag_file, args.cam0_topic, dataset.cam0_directory)
    save_images_and_data(bag_file, args.cam1_topic, dataset.cam1_directory)

    bag_file.close()

if __name__ == '__main__':
    main()