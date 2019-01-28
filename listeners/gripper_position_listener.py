#!/usr/bin/env python

import roslib
roslib.load_manifest('joint_listener')
import rospy
from joint_listener.srv import *
from sensor_msgs.msg import JointState
import threading

#holds the latest states obtained from joint_states messages
class LatestGripperPosition:

    def __init__(self):
        rospy.init_node('gripper_position_listener')
        self.lock = threading.Lock()
        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []
        self.thread = threading.Thread(target=self.joint_states_listener)
        self.thread.start()

        s = rospy.Service('return_gripper_position', ReturnJointStates, self.return_joint_states)
        

    #thread function: listen for joint_states messages
    def joint_states_listener(self):
        rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        rospy.spin()


    #callback function: when a joint_states message arrives, save the values
    def joint_states_callback(self, msg):
        if len(msg.position) is 2:
            self.lock.acquire()
            self.name = msg.name
            self.position = msg.position
            self.velocity = msg.velocity
            self.effort = msg.effort
            self.lock.release()


    #returns (found, position, velocity, effort) for the joint joint_name 
    #(found is 1 if found, 0 otherwise)
    def return_joint_state(self, joint_name):

        #no messages yet
        if self.name == []:
            rospy.logerr("No robot_state messages received!\n")
            return (0, 0., 0., 0.)

        #return info for this joint
        self.lock.acquire()
        #print(self.name)
        if joint_name in self.name:
            index = self.name.index(joint_name)
            position = self.position[index]
            velocity = self.velocity[index]
            effort = self.effort[index]

        #unless it's not found
        else:
            rospy.logerr("Joint %s not found!", (joint_name))
            self.lock.release()
            return (0, 0., 0., 0.)
        self.lock.release()
        print(1)
        return (1, position, velocity, effort)


    #server callback: returns arrays of position, velocity, and effort
    #for a list of joints specified by name
    def return_joint_states(self, req):
        joints_found = []
        positions = []
        velocities = []
        efforts = []
        for joint_name in req.name:
            (found, position, velocity, effort) = self.return_joint_state(joint_name)
            joints_found.append(found)
            positions.append(position)
            velocities.append(velocity)
            efforts.append(effort)
        return ReturnJointStatesResponse(joints_found, positions, velocities, efforts)


#run the server
if __name__ == "__main__":

    latestjointstates = LatestGripperPosition()

    print "gripper_position_listener server started, waiting for queries"
    rospy.spin()
