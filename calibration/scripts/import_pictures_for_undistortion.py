#!/usr/bin/env python

import os
import sys

import cv2
import random

import rosbag
from cv_bridge import CvBridge


def main():

    args = sys.argv # bag_file, image_topic1, image_topic2, output_dir

    bag = rosbag.Bag(args[1], "r")
    bridge = CvBridge()
    count = 0
    
    bag_r0 = list(bag.read_messages(topics=[args[2]]))
    bag_r1 = list(bag.read_messages(topics=[args[3]]))
    
    n = random.randint(0, min(len(bag_r0), len(bag_r1)))
    
    cv_img0 = bridge.imgmsg_to_cv2(bag_r0[n][1], desired_encoding="passthrough")
    cv2.imwrite(os.path.join(args[4], "img_0.png"), cv_img0)
    
    cv_img1 = bridge.imgmsg_to_cv2(bag_r1[n][1], desired_encoding="passthrough")
    cv2.imwrite(os.path.join(args[4], "img_1.png"), cv_img1)

    bag.close()

    return


if __name__ == '__main__':
    main()
