#!/usr/bin/env python



import sys
import os
import yaml

import rospy
from sensor_msgs.msg import LaserScan

import csv
import inspect


## get date path (config.yaml)
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(currentdir)
sys.path.append(parentdir)
sys.path.insert(1, parentdir)

with open(os.path.join(parentdir, 'config.yaml'), 'r') as f:
    cfg = yaml.safe_load(f)

platform = 'publish_dataset'
pendrive_dir = cfg[platform]['dataroot']
DATA_PATH = pendrive_dir


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
    

if __name__=='__main__':
    rospy.init_node('bag_node',anonymous=True)

    scan_pub=rospy.Publisher('scan',LaserScan,queue_size=10)

    scann = LaserScan()
    rate=rospy.Rate(5)
    scan_count = 1

    while not rospy.is_shutdown():
        publish_scan(scan_pub, scann, scan_count)
        scan_count += 1

        # let scan topic publish in five date per seconds
        rate.sleep()
        rospy.loginfo('published: ' + str(scan_count))


