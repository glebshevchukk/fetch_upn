import sys
import numpy as np
import numpy.random as npr
import rospy
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import copy
import geometry_msgs.msg
from geometry_msgs.msg import Point,Pose, Quaternion
from std_msgs.msg import String
import roslib
roslib.load_manifest('joint_listener')
from joint_listener.srv import *
from tf.transformations import quaternion_from_euler,euler_from_quaternion

scale_control = 0.5
noise_control=0.2
joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

gripper_names = ["l_gripper_finger_joint", "r_gripper_finger_joint"]

def random_twist_command():
    control = npr.uniform(low=-scale_control, high=scale_control, size=(2))
    twist = TwistStamped()

    twist.header.frame_id = 'base_link'
    twist.twist.linear.x = control[0]
    twist.twist.linear.y = control[1]
    twist.twist.linear.z = 0

    return twist

def new_twist_command(x=0, y=0, z=0):
    twist = TwistStamped()

    twist.header.frame_id = 'base_link'
    twist.twist.linear.x = x
    twist.twist.linear.y = y
    twist.twist.linear.z = z

    return twist

#returns velocity to a fixed pose
def fixed_twist_command(moveit, fixed_pose, random_noise=False):
    twist = TwistStamped()

    current_pose = moveit.get_ee_pose()

    if random_noise:
        random = npr.uniform(low=-noise_control, high=noise_control, size=(2))
    else:
        random = npr.uniform(low=0, high=0, size=(2))

    twist.header.frame_id = 'base_link'
    twist.twist.linear.x = fixed_pose.position.x - current_pose.position.x + random[0]
    twist.twist.linear.y = fixed_pose.position.y - current_pose.position.y + random[1]
    twist.twist.linear.z = 0

    return twist

def z_twist_command(current_z, expected_z):

    twist = TwistStamped()
    twist.header.frame_id = 'base_link'
    twist.twist.linear.x = 0.015
    twist.twist.linear.y = 0
    twist.twist.linear.z = expected_z - current_z

    return twist

def get_joint_states():
    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("/return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException:
        print("error when calling return_joint_states: %s")
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print("joint %s not found!"%joint_name)
    return (np.array(resp.position), np.array(resp.velocity), np.array(resp.effort))

def get_gripper_position():
    rospy.wait_for_service("return_gripper_position")
    try:
        s = rospy.ServiceProxy("/return_gripper_position", ReturnJointStates)
        resp = s(gripper_names)
    except rospy.ServiceException:
        print("error when calling return_gripper_position")
        sys.exit(1)
    for (ind, joint_name) in enumerate(gripper_names):
        if(not resp.found[ind]):
            print("joint %s not found!"%joint_name)
    return np.array(resp.position)



def send_gripper_command(action):
    rospy.wait_for_service("send_gripper_command")
    try:
        s = rospy.ServiceProxy("/send_gripper_command", SendGripperAction)
        resp = s(action)
    except rospy.ServiceException:
        print("error when calling send_gripper_command")
        sys.exit(1)

    return resp

def max_torso_height():
    rospy.wait_for_service("send_torso_height")
    try:
        s = rospy.ServiceProxy("/send_torso_height", SendTorsoHeight)
        resp = s(0.37)
    except rospy.ServiceException:
        print("error when calling send_torso_height")
        sys.exit(1)

    return resp

if __name__ == "__main__":
    #max_torso_height()
    send_gripper_command(0.0)
    #pos,_,_=get_gripper_position()
    #print(pos)
