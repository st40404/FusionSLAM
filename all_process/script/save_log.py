#!/usr/bin/env python
import os
import time
import rospy
from all_process.srv import Trigger


def trigger_kill_node():
    rospy.wait_for_service('/trigger')
    trigger_sig = rospy.ServiceProxy('/trigger', Trigger)
    trigger_return = trigger_sig(True)
    return trigger_return.trigger
    

if __name__=='__main__':

    rospy.init_node("save_log")
    time.sleep(10)

    while True:
        response = os.popen("rosnode ping /player")
        result = response.read()

        if "node is" in result:
            trigger = trigger_kill_node()
            if (trigger == True):
                # os.system("rosnode kill RGBD ")
                os.system("rosnode kill player ")
                os.system("rosnode kill laser_trajectory_server ")
                os.system("rosnode kill robot_state_publisher ")
                os.system("rosnode kill rviz ")
                os.system("rosnode kill turtlebot3_slam_gmapping ")
            elif (trigger == False):
                # rospy.shutdown()
                break

            time.sleep(20)
        else:
            pass

        time.sleep(1)
    

