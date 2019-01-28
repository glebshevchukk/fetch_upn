#!/usr/bin/env python

import roslib
roslib.load_manifest('joint_listener')
import rospy
from joint_listener.srv import *
from sensor_msgs.msg import JointState
import threading
import moveit_commander
import time

import actionlib
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

# Send a trajectory to controller
class FollowTrajectoryClient(object):

    def __init__(self, name, joint_names):
        self.client = actionlib.SimpleActionClient("%s/follow_joint_trajectory" % name,
                                                   FollowJointTrajectoryAction)
        rospy.loginfo("Waiting for follow joint trajectory client...")
        self.client.wait_for_server()
        self.joint_names = joint_names

    def move_to(self, positions, duration=5.0):
        if len(self.joint_names) != len(positions):
            print("Invalid trajectory position")
            return False
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        trajectory.points.append(JointTrajectoryPoint())
        trajectory.points[0].positions = positions
        trajectory.points[0].velocities = [0.0 for _ in positions]
        trajectory.points[0].accelerations = [0.0 for _ in positions]
        trajectory.points[0].time_from_start = rospy.Duration(duration)
        follow_goal = FollowJointTrajectoryGoal()
        follow_goal.trajectory = trajectory

        self.client.send_goal(follow_goal)
        self.client.wait_for_result()

class TorsoHeightListener:

    def __init__(self):
        rospy.init_node('torso_height_listener')
        self.name = []
        self.torso_action = FollowTrajectoryClient("torso_controller", ["torso_lift_joint"])

        s = rospy.Service('send_torso_height', SendTorsoHeight, self.send_torso_height)
        
    def send_torso_height(self, req):

        height = req.height

        print(height)

        self.torso_action.move_to([height, ])

        #returns this just in case
        return SendTorsoHeightResponse(height)
#run the server
if __name__ == "__main__":

    TorsoHeightListener()

    print "torso_height_listener server started, waiting for commands that we'll take"
    rospy.spin()
