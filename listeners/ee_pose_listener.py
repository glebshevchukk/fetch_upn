#!/usr/bin/env python
#spins off a thread to listen for joint_states messages
#and provides the same information (or subsets of) as a service

import roslib
roslib.load_manifest('joint_listener')
import rospy
from joint_listener.srv import *
from sensor_msgs.msg import JointState
import threading
import moveit_commander


#holds the latest states obtained from joint_states messages
class LatestEEPose:

    def __init__(self):
        rospy.init_node('ee_pose_listener')
        self.name = []
        self.x = 0
        self.y = 0
        self.z = 0

        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("arm")

        s = rospy.Service('return_ee_pose', ReturnEEPose, self.ee_pose_listener)
        
    def ee_pose_listener(self, req):

        found = [1]
        current_pose = self.group.get_current_pose()

        print(current_pose)

        return ReturnEEPoseResponse(found, current_pose)
#run the server
if __name__ == "__main__":

    LatestEEPose()

    print "ee_pose_listener server started, waiting for queries"
    rospy.spin()
