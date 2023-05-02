#!/usr/bin/env python
import os
import time


import rospy
from all_process.srv import Trigger

def trigger_kill_node():
    rospy.wait_for_service('/trigger')
    trigger_sig = rospy.ServiceProxy('/trigger', Trigger)
    trigger_return = trigger_sig(True)

if __name__=='__main__':

    time.sleep(10)

    # nodes = os.popen("rosnode list").readlines()

    # for i in range(len(nodes)):
    #     nodes[i] = nodes[i].replace("\n","")

    while True:

        response = os.popen("rosnode ping /player")
        result = response.read()
        if "node is" in result:
            trigger_kill_node()
            os.system("rosnode kill RGBD ")
            os.system("rosnode kill player ")
            os.system("rosnode kill laser_trajectory_server ")
            os.system("rosnode kill robot_state_publisher ")
            os.system("rosnode kill rviz ")
            os.system("rosnode kill turtlebot3_slam_gmapping ")

            time.sleep(20)
        else:
            pass

        time.sleep(1)

        # response = os.popen("rosnode ping /player")
        # print(response)
        # time.sleep(1)

