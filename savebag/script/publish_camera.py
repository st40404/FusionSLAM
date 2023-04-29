#!/usr/bin/env python


import sys
import os
import yaml

import rospy

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import inspect
import csv

## get date path (config.yaml)
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
sys.path.insert(1, parentdir)

with open(os.path.join(parentdir, 'config.yaml'), 'r') as f:
    cfg = yaml.safe_load(f)

platform = 'publish_dataset'  # UGV
pendrive_dir = cfg[platform]['dataroot']
DATA_PATH = pendrive_dir


def publish_rgb(rgb_pub, bridge, image, count):
    with open(DATA_PATH +'/rgb.csv', 'r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        msg_rgb = bridge.cv2_to_imgmsg(image,'bgr8')

        for line in csv_reader:
            if (msg_rgb.header.stamp.secs == 0):
                msg_rgb.header.stamp.secs  = int(line['%010d'%count])
            else:
                msg_rgb.header.stamp.nsecs = float(line['%010d'%count])

        msg_rgb.header.frame_id    = 'camera_rgb_optical_frame'
        rgb_pub.publish(msg_rgb)




def publish_depth(depth_pub, bridge, depth, count):
    with open(DATA_PATH +'/depth.csv', 'r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        msg_depth = bridge.cv2_to_imgmsg(depth,encoding="passthrough")

        for line in csv_reader:
            if (msg_depth.header.stamp.secs == 0):
                msg_depth.header.stamp.secs  = int(line['%010d'%count])
            else:
                msg_depth.header.stamp.nsecs = float(line['%010d'%count])

        msg_depth.header.frame_id    = 'camera_rgb_optical_frame'
        depth_pub.publish(msg_depth)



def read_camera(path):
    return cv2.imread(path)

if __name__=='__main__':
    rospy.init_node('bag_node',anonymous=True)
    depth_pub=rospy.Publisher('/camera/depth/image_raw',Image,queue_size=10)
    rgb_pub=rospy.Publisher('/camera/rgb/image_raw',Image,queue_size=10)

    bridge=CvBridge()

    # rate=rospy.Rate(160)
    rgb_count=1
    depth_count=1

    while not rospy.is_shutdown():
        # img=read_camera(os.path.join(DATA_PATH,'rgb/%010d.png'%frame))
        rgb=read_camera(os.path.join(DATA_PATH,'rgb/%010d.png'%rgb_count))
        depth=read_camera(os.path.join(DATA_PATH,'depth/%010d.png'%depth_count))


        publish_depth(depth_pub,bridge,depth,depth_count)
        publish_rgb(rgb_pub,bridge,rgb,rgb_count)

        # rate.sleep()
        rgb_count+=1
        depth_count+=1
        
        # frame%=154



