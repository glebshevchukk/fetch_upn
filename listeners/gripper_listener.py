#!/usr/bin/env python

import rospy
from joint_listener.srv import *
from sensor_msgs.msg import JointState
import threading
import moveit_commander
import time

import actionlib
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal,
)


# Send a trajectory to controller
class GripperClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
        rospy.loginfo("Waiting for gripper client...")
        self.client.wait_for_server()

    def send(self, action, duration=4.0):
        goal = GripperCommandGoal()
        goal.command.max_effort = 10000000.0
        goal.command.position = action
        self.client.send_goal(goal)
        self.client.wait_for_result(rospy.Duration(duration))

class GripperListener(object):

    def __init__(self):
        rospy.init_node('gripper_listener')
        self.name = []
        self.gripper_action = GripperClient()

        s = rospy.Service('send_gripper_command', SendGripperAction, self.send_gripper_action)
        
    def send_gripper_action(self, req):
        action = req.action
        print(action)
        self.gripper_action.send(action)

        #returns this just in case
        return SendGripperActionResponse(action)
#run the server
if __name__ == "__main__":

    GripperListener()

    print "gripper_action_listener server started, waiting for commands that we'll take"
    rospy.spin()
