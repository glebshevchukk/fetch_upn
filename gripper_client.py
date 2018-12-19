#!/usr/bin/python

import rospy, actionlib
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

############################################
## MoveGripperClient.py                   ##
##                                        ##
## Move the gripper of the Fetch Robot    ##
## using a simple action client           ##
##                                        ##
## This is the preferred method to        ##
## control the gripper of the Fetch Robot ##
##                                        ##
## Ian de Vlaming, dvlaming@stanford.edu  ##
## 02/03/2018                             ##
############################################

# moveGripper()
# handles gripper control
def move_gripper(action):
  # create a simple action client gripperClient
  # which publishes over and subscribes to
  # the parent topic gripper_controller/gripper_action
  # with control_msgs/GripperCommand Action and Goal
  gripperClient = actionlib.SimpleActionClient("gripper_controller/gripper_action", GripperCommandAction)
  # wait for server response
  gripperClient.wait_for_server()
  # create empty control_msgs/GripperCommandGoal message
  goal = GripperCommandGoal()
  # set maximum exerted effort for opening or closing
  goal.command.max_effort = 10.0
  # set goal position
  # range is 0-1 (discrete, 0 is closed, 1 is open)
  goal.command.position = action
  # send the goal to the action server (gripper)
  gripperClient.send_goal(goal)
  # wait for result to report that the goal is set
  gripperClient.wait_for_result(rospy.Duration(4.0))