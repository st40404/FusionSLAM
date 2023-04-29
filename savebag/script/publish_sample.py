#!/usr/bin/env python



import sys
import os
import yaml

# from data_utils import *
# from publish_utils import *
import rospy
from sensor_msgs.msg import Image,PointCloud2,Imu,NavSatFix, LaserScan
# import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
# from visualization_msgs.msg import Marker,MarkerArray
# from geometry_msgs.msg import Point
import tf
import numpy as np
import cv2
import csv
import inspect


currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
sys.path.insert(1, parentdir)

with open(os.path.join(parentdir, 'config.yaml'), 'r') as f:
    cfg = yaml.safe_load(f)


platform = 'publish_dataset'  # UGV

pendrive_dir = cfg[platform]['dataroot']
DATA_PATH = pendrive_dir

# DATA_PATH='/home/ron/work/src/savebag/bag/sim_bag'
# DATA_PATH='/home/ron/work/src/savebag/bag/sim_bag'


def publish_rgb(rgb_pub,bridge,image):

    # bridge.header.secs = '111'
    rgb_pub.publish(bridge.cv2_to_imgmsg(image,'bgr8'))
    # rgb_pub.publish(bridge.cv2_to_imgmsg(image,'bgr8'))

def publish_depth(depth_pub,bridge,depth):
    # depth_pub.publish(bridge.cv2_to_imgmsg(depth,'8UC3'))
    depth_pub.publish(bridge.cv2_to_imgmsg(depth,encoding="passthrough"))

def publish_scan(scan_pub, scann, scan_count):
    with open(DATA_PATH +'/scan/%d.csv'%scan_count,'r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for line in csv_reader:
            scann.header.stamp.secs  = int(line['Header_secs'])
            scann.header.stamp.nsecs = float(line['Header_nsecs'])
            scann.angle_min          = float(line['angle_min'])
            scann.angle_max          = float(line['angle_max'])
            scann.angle_increment    = float(line['angle_increment'])
            scann.time_increment     = float(line['time_increment'])
            scann.scan_time          = float(line['scan_time'])
            scann.range_min          = float(line['range_min'])
            scann.range_max          = float(line['range_max'])
            for i in range (0,360):
                scann.ranges.append(float(line['Range%d'%(i+1)]))
                scann.intensities.append(float(line['Intensity%d'%(i+1)]))
    scan_pub.publish(scann)
    


def read_camera(path):
    return cv2.imread(path)

if __name__=='__main__':
    rospy.init_node('bag_node',anonymous=True)
    rgb_pub=rospy.Publisher('/camera/rgb/image_raw',Image,queue_size=10)
    depth_pub=rospy.Publisher('/camera/depth/image_raw',Image,queue_size=10)
    scan_pub=rospy.Publisher('scan',LaserScan,queue_size=10)
    # pcl_pub=rospy.Publisher('kitti_pcl',PointCloud2,queue_size=10)
    # ego_pub=rospy.Publisher('kitti_ego_car',MarkerArray,queue_size=10)
    # imu_pub=rospy.Publisher('kitti_imu',Imu,queue_size=10)
    # gps_pub=rospy.Publisher('kitti_gps',NavSatFix,queue_size=10)
    bridge=CvBridge()
    scann = LaserScan()
    rate=rospy.Rate(10)
    frame=1
    scan_count = 1

    while not rospy.is_shutdown():
        # img=read_camera(os.path.join(DATA_PATH,'rgb/%010d.png'%frame))
        rgb=read_camera(os.path.join(DATA_PATH,'rgb/%d.png'%frame))
        depth=read_camera(os.path.join(DATA_PATH,'depth/%d.png'%frame))
        # scan=read_scan(os.path.join(DATA_PATH,'scan/%d.csv'%frame))
        # pcl=read_point_cloud(os.path.join(DATA_PATH,'velodyne_points/data/%010d.bin'%frame))
        # imu=read_imu(os.path.join(DATA_PATH,'oxts/data/%010d.txt'%frame))

        # publish_rgb(rgb_pub,bridge,rgb)
        # publish_depth(depth_pub,bridge,depth)
        publish_scan(scan_pub, scann, scan_count)
        # publish_pcl(pcl_pub,pcl)
        # publish_ego_car(ego_pub)
        # publish_imu(imu_pub,imu)
        # publish_gps(gps_pub,imu)
        rospy.loginfo('published: ' + str(frame))
        rate.sleep()
        frame+=1
        # frame%=154
        scan_count += 1



