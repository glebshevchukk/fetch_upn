import argparse
import numpy as np
import numpy.random as npr


import sys
import time
import re
import rospy
import copy
import glob
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Header
import pickle
import multiprocessing
from multiprocessing import Process

from real_camera import *
#from record_saver import *


import copy
import geometry_msgs.msg
from geometry_msgs.msg import Point,Pose, Quaternion
from std_msgs.msg import String

import roslib
roslib.load_manifest('joint_listener')
from joint_listener.srv import ReturnJointStates, ReturnEEPose
import moveit_commander
from tf.transformations import quaternion_from_euler,euler_from_quaternion

from gripper_client import *
from shell import *

import glob
from PIL import Image

from moveit import *


scale_control = 0.5


parser = argparse.ArgumentParser()
parser.add_argument('--episodes')
parser.add_argument('--timesteps')
parser.add_argument('--saveinterval')
parser.add_argument('--savepath')
parser.add_argument('--port')
parser.add_argument('--startepisode')
joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        

def start_data_collection(episodes, timesteps, saveinterval, save_path,port, startepisode, camera):

    pub = rospy.Publisher('/arm_controller/cartesian_twist/command', TwistStamped)
    rospy.init_node('send_velocities')
    rate = rospy.Rate(2)

    moveit = MoveIt()
    moveit.get_ee_bounds(fixed=True)
    moveit.get_start_point(fixed=True)

    start_time = -1

    total_errors = 0

    all_images = []
    all_actions = []
    all_vel_actions = []
    all_qts = []
    all_ee_qts = []

    episode = 0

    while episode < episodes:
        #moveit.shuffle_reset()
        while moveit.not_at_start():
            time.sleep(1)
            moveit.go_to_start()

        red_pose = moveit.make_new_pose(0.5,0.23)
        tan_pose = moveit.make_new_pose(0.5,-0.23)
        goal_pose = moveit.make_new_pose(0.7,0.1)

        #FIRST REAL TRAJ: go to red object then to goal
        #fixed_goals = [red_pose,goal_pose]
        #FAKE TRAJ 1: go to tan object then to goal
        #fixed_goals = [tan_pose,goal_pose]
        #FAKEST TRAJ: just go to goal
        fixed_goals = [goal_pose,goal_pose]
        images = []
        actions = []
        vel_actions = []
        qts = []
        ee_qts = []
        
        #move_gripper(1)
        #moveit.shuffle_reset()
        #moveit.trace_perimeter()\
        print("Starting episode %d" % episode)

        no_errors = True

        #check that start position is correct, otherwise retry

        for timestep in range(timesteps):
            print(timestep)

            if timestep < 5:
                fixed_goal=fixed_goals[0]
            else:
                fixed_goal=fixed_goals[1]

            if not moveit.not_at_start():
                print("Still at start!")
      
            position, velocity, effort = get_joint_states()

            current_pose = moveit.get_ee_pose()
            
            if current_pose.position.x < moveit.lower_point.position.x:
                twist_command = new_twist_command(x=scale_control)
            elif current_pose.position.y < moveit.lower_point.position.y:
                twist_command = new_twist_command(y=scale_control)
            elif current_pose.position.x > moveit.upper_point.position.x:
                twist_command = new_twist_command(x=-scale_control)
            elif current_pose.position.y > moveit.upper_point.position.y:
                twist_command = new_twist_command(y=-scale_control)
            else:
              #twist_command = random_twist_command()
              twist_command = fixed_twist_command(moveit,fixed_goal)

            image = None
            while image is None:
                image = camera.capture(timestep)
                current_pose = moveit.get_ee_pose()
                ee_vel = [twist_command.twist.linear.x, twist_command.twist.linear.y, twist_command.twist.linear.z]
                ee_pos = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
            
            pub.publish(twist_command)
            rate.sleep()

            if current_pose.position.z < 0.94 or current_pose.position.z > 1:
                print(current_pose.position.z)
                print("Z bounds violated, restarting episode")
                total_errors += 1
                no_errors = False
                break

            if moveit.orientation_violated():
                print("Orientation violated, restarting episode")
                total_errors += 1
                no_errors = False
                break

            images.append(image)
            if timestep is not timesteps - 1:
                actions.append(ee_vel)
                vel_actions.append(velocity)
                qts.append(position)
                ee_qts.append(ee_pos)

            twist_command = new_twist_command()
            pub.publish(twist_command)

        if no_errors:
            all_images.append(images)
            all_actions.append(actions)
            all_vel_actions.append(vel_actions)
            all_qts.append(qts)
            all_ee_qts.append(ee_qts)
            print("Collected single set for episode %d" % episode)

            if (episode % saveinterval) == 0:

                time_interval = str(startepisode + start_time) + '-' + str(startepisode+episode)
                pickle.dump(all_images,open(save_path + '/images/images_' + time_interval + '.pickle','wb'))
                pickle.dump(all_actions,open(save_path + '/actions/actions_' + time_interval + '.pickle','wb'))
                pickle.dump(all_vel_actions,open(save_path + '/vel_actions/vel_actions_' + time_interval + '.pickle','wb'))
                pickle.dump(all_qts,open(save_path + '/qts/qts_' + time_interval + '.pickle','wb'))
                pickle.dump(all_ee_qts,open(save_path + '/ee_qts/ee_qts_' + time_interval + '.pickle','wb'))

                all_images = []
                all_actions = []
                all_vel_actions = []
                all_qts = []
                all_ee_qts = []
                start_time =  episode

        del images
        del actions
        del vel_actions
        del qts
        del ee_qts

        twist_command = new_twist_command()
        pub.publish(twist_command)

        if no_errors:
            episode += 1


    del all_images
    del all_actions
    del all_vel_actions
    del all_qts
    del all_ee_qts

    camera.close()


def random_valid_point(moveit):

    x = npr.uniform(low=moveit.lower_x, high=moveit.upper_x)
    y = npr.uniform(low=moveit.lower_y, high=moveit.upper_y)
    z = moveit.height

    point = Point(x, y, z)
    pose = Pose(position=point, orientation=upright)

    return pose



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
def fixed_twist_command(moveit, fixed_pose):
    twist = TwistStamped()

    scale = 5.0

    current_pose = moveit.get_ee_pose()

    twist.header.frame_id = 'base_link'
    twist.twist.linear.x = (fixed_pose.position.x - current_pose.position.x) * scale
    twist.twist.linear.y = (fixed_pose.position.y - current_pose.position.y) * scale
    twist.twist.linear.z = (fixed_pose.position.z - current_pose.position.z) * scale

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


def main(episodes, timesteps, saveinterval, save_path,port, startepisode):

    try:
        camera = RealCamera(port)
        processes = camera.processes
        processes.append(Process(target=start_data_collection, args=(episodes, timesteps, saveinterval, save_path, port, startepisode, camera)))

        for p in processes:
          p.start()

        print("Starting all processes")
        for p in processes:
          p.join()

    except KeyboardInterrupt:
        camera.close()
        try:
          moveit_commander.roscpp_shutdown()
          sys.exit(0)
        except SystemExit:
          moveit_commander.roscpp_shutdown()
          os._exit(0)  # pylint: disable=protected-access

if __name__ == "__main__":
    args = parser.parse_args()
    main(int(args.episodes), int(args.timesteps), int(args.saveinterval), args.savepath, int(args.port), int(args.startepisode))
