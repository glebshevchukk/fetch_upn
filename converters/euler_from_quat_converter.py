#!/usr/bin/env python

import roslib
roslib.load_manifest('joint_listener')
import rospy
from joint_listener.srv import *
import threading
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from joint_listener.srv import ReturnJointStates, ReturnEEPose, ReturnQuat, ReturnEuler


#holds the latest states obtained from joint_states messages
class EulerFromQuat:

    def __init__(self):
        rospy.init_node('euler_from_quaternion')

        s = rospy.Service('return_euler_from_quat', ReturnEuler, self.convert)
        
    def convert(self, req):

        found = [1]
        
        euler = euler_from_quaternion(req.x, req.y, req.z, req.w)
        
        x = euler[0]
        y = euler[1]
        z = euler[2]

        print("Request recieved, responding")
        return ReturnEulerResponse(found, x, y, z)
#run the server
if __name__ == "__main__":

    EulerFromQuat()

    print "euler_from_quaternion_converter server started, waiting for queries"
    rospy.spin()
