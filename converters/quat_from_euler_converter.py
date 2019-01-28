#!/usr/bin/env python

import roslib
roslib.load_manifest('joint_listener')
import rospy
from joint_listener.srv import *
import threading
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from joint_listener.srv import ReturnJointStates, ReturnEEPose, ReturnQuat, ReturnEuler


#holds the latest states obtained from joint_states messages
class QuatFromEuler:

    def __init__(self):
        rospy.init_node('quaternion_from_euler')

        s = rospy.Service('return_quat_from_euler', ReturnQuat, self.convert)
        
    def convert(self, req):

        found = [1]
        
        quat = quaternion_from_euler(req.x, req.y, req.z)
        
        x = quat[0]
        y = quat[1]
        z = quat[2]
        w = quat[3]

        print("Request recieved, responding")
        return ReturnQuatResponse(found, x, y, z, w)
#run the server
if __name__ == "__main__":

    QuatFromEuler()

    print "quaternion_from_euler_converter server started, waiting for queries"
    rospy.spin()
