#!/usr/bin/env python

import roslib
roslib.load_manifest('joint_listener')
import rospy
from joint_listener.srv import *
from sensor_msgs.msg import JointState
import threading
import moveit_commander
import time


#WE'RE JUST GOING TO BE SENDING POSES TO MOVEIT, AND THIS WILL ALL BE CALLED FROM THE WORKSTATION
class MoveItPoseListener:

    def __init__(self):
        rospy.init_node('moveit_pose_listener')
        self.name = []

        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("arm")

        s = rospy.Service('send_moveit_pose', SendMoveitPose, self.send_moveit_pose)
        
    def send_moveit_pose(self, req):

        found = [1]
        
        self.group.clear_pose_targets()
        self.group.set_pose_target(req.pose)
        self.group.plan()
        time.sleep(2)
        self.group.go(wait=True)

        current_pose = self.group.get_current_pose().pose

        #returns this just in case
        return SendMoveitPoseResponse(current_pose)
#run the server
if __name__ == "__main__":

    MoveItPoseListener()

    print "moveit_pose_listener server started, waiting for commands that we'll take"
    rospy.spin()
