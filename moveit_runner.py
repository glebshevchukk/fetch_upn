import argparse
import numpy as np
import numpy.random as npr


import sys
import time
import re
import rospy
from geometry_msgs.msg import TwistStamped, Pose, Quaternion, Point
from std_msgs.msg import Header
import pickle

#from real_camera import *
#from record_saver import *


import copy
import geometry_msgs.msg
from std_msgs.msg import String

import roslib
roslib.load_manifest('joint_listener')
from joint_listener.srv import ReturnJointStates, ReturnEEPose

import moveit_commander
from tf.transformations import quaternion_from_euler

scale_control = 1
global_random_counter = 0


parser = argparse.ArgumentParser()
parser.add_argument('--episodes')
parser.add_argument('--timesteps')
parser.add_argument('--saveinterval')
parser.add_argument('--savepath')
parser.add_argument('--port')

joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]


quat = list(quaternion_from_euler(0, 1.5707, 1.5707))
upright = Quaternion(quat[0],quat[1],quat[2],quat[3])
fixed_lower_corner = Point(0.5, -0.2, 0.94)
fixed_lower_corner = Pose(position=fixed_lower_corner, orientation=None)

fixed_upper_corner = Point(0.7, 0.2, 0.94)
fixed_upper_corner = Pose(position=fixed_upper_corner, orientation=None)

fixed_start = Point(0.6, 0, 0.94)
fixed_start = Pose(position=fixed_start, orientation=upright)


class MoveIt(object):
    def __init__(self):
        self.lower_point = fixed_lower_corner
        self.upper_point = fixed_upper_corner
        self.start_point = fixed_start
        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("arm")

    def get_ee_bounds(self):
        try:
          input("Please set the end effector to the first corner of your bounding rectangle, then click ENTER")
        except:
          pass        
        first_pose = self.get_ee_pose()
        print(first_pose)
        try:
          input("Please set the end effector to the second corner of your bounding rectangle, then click ENTER")
        except:
          pass        
        second_pose = self.get_ee_pose()
        print(second_pose)
        if first_pose.position.x > second_pose.position.x and first_pose.position.y > second_pose.position.y:
            self.upper_point=first_pose
            self.lower_point=second_pose
            return True
        elif first_pose.position.x < second_pose.position.x and first_pose.position.y < second_pose.position.y:
            self.upper_point=second_pose
            self.lower_point=first_pose
            return True
        else:
            print("Boundaries are incorrect, please try again")
            return False


    def get_start_point(self):
        try:
          input("Please set the end effector to the point you want it to always start at, then click ENTER")
        except:
          pass
        if not self.exceeds_bounds():
            self.start_point = self.get_ee_pose()
            print(self.start_point)
            return True
        else:
            print("Start state is incorrect, please try again")
            return False

    def get_ee_pose(self):
        return self.group.get_current_pose().pose

    def go_to_start(self):
        print("Going to start")
        self.group.clear_pose_targets()
        self.group.set_pose_target(self.start_point)
        self.group.plan()
        self.group.go(wait=True)


    def go_to(self, pose):

        self.group.clear_pose_targets()
        self.group.set_pose_target(pose)
        self.group.plan()
        self.group.go(wait=True)


    def random_pose_command(self):
        global global_random_counter
        pose = Pose()
        current_pose = self.get_ee_pose()
        #twist.header.stamp = rospy.Time.now()
        global_random_counter += 1

        # twist.twist.linear.x = scale_control * control[0]
        # twist.twist.linear.y = scale_control * control[1]
        pose.position.x = npr.uniform(low=self.lower_point.position.x, high=self.upper_point.position.x)
        pose.position.y = npr.uniform(low=self.lower_point.position.y, high=self.upper_point.position.y)
        pose.position.z = self.get_ee_pose().position.z
        pose.orientation = upright

        return pose


    def exceeds_bounds(self):

        current_pose = self.get_ee_pose()

        if (current_pose.position.x < self.lower_point.position.x and current_pose.position.y < self.lower_point.position.y) or (current_pose.position.x > self.upper_point.position.x and current_pose.position.y > self.upper_point.position.y):
            return True
        else:
            return False

def start_data_collection(episodes, timesteps, saveinterval, save_path,port):

    rospy.init_node('send_poses')
    rate = rospy.Rate(2)
    camera = RealCamera(port)
    camera.view()

    moveit = MoveIt()

    #good_bounds = moveit.get_ee_bounds()

    #if not good_bounds:
    #    return

    #good_start = moveit.get_start_point()

    #if not good_start:
    #    return   

    start_time = 0

    all_images = []
    all_actions = []
    all_qts = []

    for episode in range(episodes):

        images = []
        actions = []
        qts = []
        moveit.go_to_start()

        for timestep in range(timesteps):
            print(timestep)
            pose_command = moveit.random_pose_command()

            #need to run this on a seperate thread perhaps?
            moveit.go_to(pose_command)
            rate.sleep()
            image = camera.capture()
            #image = np.zeros((100,100,3))
            position, velocity, effort = get_joint_states()

            images.append(image)
            actions.append(velocity)
            qts.append(position)

        all_images.append(images)
        all_actions.append(actions)
        all_qts.append(qts)
        print("Collected single set for episode %d" % episode)

        if episode +1 % saveinterval:
            time_interval = str(start_time) + '-' + str(episode)
            pickle.dump(all_images,open(save_path + 'images/images_' + time_interval + '.pickle','wb'))
            pickle.dump(all_actions,open(save_path + 'actions/actions_' + time_interval + '.pickle','wb'))
            pickle.dump(all_qts,open(save_path + 'qts/qts_' + time_interval + '.pickle','wb'))

            all_images = []
            all_actions = []
            all_qts = []

            start_time = episode

            rate.sleep()

    #camera.close()

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


if __name__ == "__main__":
    args = parser.parse_args()
    start_data_collection(int(args.episodes), int(args.timesteps), int(args.saveinterval), args.savepath, int(args.port))
