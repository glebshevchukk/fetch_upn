#!/usr/bin/env python

import roslib
roslib.load_manifest('joint_listener')
import rospy
from joint_listener.srv import *
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
import threading
import moveit_commander
import time


class VelocityListener:

    def __init__(self):
        rospy.init_node('velocity_command_listener')

        self.pub = rospy.Publisher('/arm_controller/cartesian_twist/command', TwistStamped)
        self.rate = rospy.Rate(2)
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("arm")

        s = rospy.Service('send_velocity_command', SendVelocity, self.send_velocity)
    
    #check if you're out of bounds, if you are, you should correct yourself
    def send_velocity(self, req):
        twist = new_twist_command(req.x, req.y, req.z)
        self.pub.publish(twist)
        self.rate.sleep()
        print("Taking velocity command")

        z_twist = new_twist_command(0,0,0)
        self.pub.publish(z_twist)
        self.rate.sleep()
        current_pose = self.group.get_current_pose().pose

        #returns this just in case
        return SendVelocityResponse(current_pose)

def new_twist_command(x=0, y=0, z=0):
    twist = TwistStamped()
    twist.header.frame_id = 'base_link'
    twist.twist.linear.x = x
    twist.twist.linear.y = y
    twist.twist.linear.z = z

    return twist
#run the server
if __name__ == "__main__":

    VelocityListener()

    print "velocity_listener server started, waiting for commands that we'll take"
    rospy.spin()
