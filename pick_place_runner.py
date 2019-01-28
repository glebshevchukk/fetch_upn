import argparse
import numpy as np
import numpy.random as npr
import random


import sys
import time
import os
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
import copy
import geometry_msgs.msg
from geometry_msgs.msg import Point,Pose, Quaternion
from std_msgs.msg import String
import roslib
roslib.load_manifest('joint_listener')
from joint_listener.srv import *
import moveit_commander
from tf.transformations import quaternion_from_euler,euler_from_quaternion
from shell import *
import glob
from PIL import Image
from pick_place_moveit import *
from ros_methods import *

parser = argparse.ArgumentParser()
parser.add_argument('--episodes')
parser.add_argument('--random_motion_timesteps')
parser.add_argument('--grasping_timesteps')
parser.add_argument('--reaching_timesteps')
parser.add_argument('--dropping_timesteps')
parser.add_argument('--saveinterval')
parser.add_argument('--savepath')
parser.add_argument('--port')
parser.add_argument('--startepisode')
parser.add_argument('--num_objects')

        
#Have two versions: pick and place and grasping saved to the same spot
#action = x vel, y vel, z vel, gripper open/close, terminate?
def start_data_collection(episodes, random_motion_timesteps, grasping_timesteps, 
    reaching_timesteps, dropping_timesteps, saveinterval, save_path,port, startepisode, camera, num_objects, newstdin):

    rospy.init_node('moveit_actual')
    pub = rospy.Publisher('/arm_controller/cartesian_twist/command', TwistStamped)
    rate = rospy.Rate(1)
    moveit = MoveIt()
    moveit.get_ee_bounds(fixed=True)
    moveit.get_start_point(fixed=True)

    object_poses = moveit.get_object_positions(num_objects, newstdin)

    start_time = -1
    total_errors = 0

    all_images = []
    all_actions = []
    all_vel_actions = []
    all_qts = []
    all_ee_qts = []

    episode = 0

    #=====================================
        #Identify all object positions by manually moving gripper to those positions
    #=====================================

    while episode < episodes:

        fixed_random_object_index = random.randrange(len(object_poses))
        fixed_random_object_pose = object_poses[fixed_random_object_index]

        print(fixed_random_object_pose)

        max_torso_height()
        send_gripper_command(1)
        while moveit.not_at_start():
            moveit.go_to_start()
        images = []
        actions = []
        vel_actions = []
        qts = []
        ee_qts = []
        print("Starting episode %d" % episode)
        no_errors = True
        #=====================================
        #Random Motion Stage
        #=====================================
        for timestep in range(random_motion_timesteps):
            print("Random motion timestep: %d" % timestep)

            if not moveit.not_at_start():
                print("Still at start!")
      
            current_pose = moveit.get_ee_pose()
            
            if current_pose.position.x < moveit.lower_point.position.x:
                twist_command = new_twist_command(x=scale_control)
            elif current_pose.position.y < moveit.lower_point.position.y:
                twist_command = new_twist_command(y=scale_control)
            elif current_pose.position.x > moveit.upper_point.position.x:
                twist_command = new_twist_command(x=-scale_control)
            elif current_pose.position.y > moveit.upper_point.position.y:
                twist_command = new_twist_command(y=-scale_control)
            elif timestep < random_motion_timesteps - 4:
                twist_command = fixed_twist_command(moveit,fixed_random_object_pose, random_noise=True)
            else:
              #twist_command = random_twist_command()
              twist_command = fixed_twist_command(moveit,fixed_random_object_pose, random_noise=False)

            image = None
            while image is None:
                image = camera.capture(timestep)
            ee_vel = [twist_command.twist.linear.x, twist_command.twist.linear.y, twist_command.twist.linear.z]
            ee_pos = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
            position, _, _ = get_joint_states()

            #publish the random command, then get your current vel, append all your info, then sleep
            pub.publish(twist_command)
            _, velocity, _ = get_joint_states()

            images.append(image)

            #1 is the gripper command, since we are not currently gripping anything
            full_action = ee_vel + [1]
            actions.append(full_action)
            vel_actions.append(velocity)
            qts.append(position)
            ee_qts.append(ee_pos)

            rate.sleep()

            #finally publish a zero command to stop arm from moving
            twist_command = new_twist_command()
            pub.publish(twist_command)


        #=====================================
        #Grasping Stage
        #=====================================
        grasped = False
        all_grips = []
        for timestep in range(grasping_timesteps):
            print("Grasping timestep: %d" % timestep)
      
            current_pose = moveit.get_ee_pose()
            gripper_command = 1

            #if we've reached the grasp height, stop and send grasp command, then pull back up
            if not grasped and (current_pose.position.z < 0.93 or timestep is 5):
                grasped = True
                gripper_command = 0
                twist_command = new_twist_command()
                print("Grasping has occured succesfully")
            #if we've already grasped, we want to go upward
            #if at any point we notice that the gripper is fully inward, we've dropped the object
            elif grasped:
                gripper_command = 0
                twist_command = z_twist_command(current_pose.position.z, moveit.height)
                print("Grasping has occured, now moving up")
                gripper_positions = get_gripper_position()
                print(gripper_positions)
                all_grips.append(gripper_positions[0])

            else:
                #we basically want a twist command that gives a velocity relative to the final position we want to be in
                twist_command = z_twist_command(current_pose.position.z, moveit.down_height)

            image = None
            while image is None:
                image = camera.capture(timestep)
            ee_vel = [twist_command.twist.linear.x, twist_command.twist.linear.y, twist_command.twist.linear.z]
            ee_pos = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
            position, _, _ = get_joint_states()

            if gripper_command == 0:
                print("Closing gripper")
                send_gripper_command(gripper_command)

            pub.publish(twist_command)
            _, velocity, _ = get_joint_states()

            images.append(image)

            #appending gripper command to the full action
            full_action = ee_vel + [gripper_command]
            actions.append(full_action)
            vel_actions.append(velocity)
            qts.append(position)
            ee_qts.append(ee_pos)

            rate.sleep()

            #finally publish a zero command to stop arm from moving
            twist_command = new_twist_command()
            pub.publish(twist_command)

        grip_difference = all_grips[-1] - all_grips[0]
        if abs(grip_difference) > 0.0003 or all_grips[-1] < 0.00015:
            print("Grasp was unsuccesful, we won't record the new position")
            no_errors = False

        #=====================================
        #New Reaching Stage
        #=====================================
        fixed_random_place_pose = moveit.get_valid_distant_pose()
        if True:
            for timestep in range(reaching_timesteps):
                print("Reaching timestep: %d" % timestep)

                current_pose = moveit.get_ee_pose()
                if current_pose.position.x < moveit.lower_point.position.x:
                    twist_command = new_twist_command(x=scale_control)
                elif current_pose.position.y < moveit.lower_point.position.y:
                    twist_command = new_twist_command(y=scale_control)
                elif current_pose.position.x > moveit.upper_point.position.x:
                    twist_command = new_twist_command(x=-scale_control)
                elif current_pose.position.y > moveit.upper_point.position.y:
                    twist_command = new_twist_command(y=-scale_control)
                elif timestep < random_motion_timesteps - 4:
                    twist_command = fixed_twist_command(moveit,fixed_random_place_pose, random_noise=True)
                else:
                    twist_command = fixed_twist_command(moveit,fixed_random_place_pose, random_noise=False)

                image = None
                while image is None:
                    image = camera.capture(timestep)
                ee_vel = [twist_command.twist.linear.x, twist_command.twist.linear.y, twist_command.twist.linear.z]
                ee_pos = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
                position, _, _ = get_joint_states()

                #publish the random command, then get your current vel, append all your info, then sleep
                pub.publish(twist_command)
                _, velocity, _ = get_joint_states()

                images.append(image)

                #0 is the gripper command, since we are grasping the object
                full_action = ee_vel + [0]
                actions.append(full_action)
                vel_actions.append(velocity)
                qts.append(position)
                ee_qts.append(ee_pos)

                rate.sleep()

                #finally publish a zero command to stop arm from moving
                twist_command = new_twist_command()
                pub.publish(twist_command)
        

        #finally we want to drop whatever object we currently have and move the arm down a bit, then go in a random velocity direction
        #now we assume that the object has been succesfully grasped

        #=====================================
        #Dropping Stage
        #=====================================
        if True:
            for timestep in range(dropping_timesteps):
                print("Dropping timestep: %d" % timestep)
          
                current_pose = moveit.get_ee_pose()
                gripper_command = 0

                #made for 10 timestep dropping
                if timestep < 3:
                    twist_command = z_twist_command(current_pose.position.z, moveit.mid_height)
                elif timestep is 3:
                    gripper_command = 1
                    twist_command = new_twist_command()

                    #update position of the object assuming we just dropped it
                    if no_errors:
                        new_object_pose = moveit.get_ee_pose()
                        object_poses[fixed_random_object_index] = new_object_pose

                elif timestep > 3 and timestep < 7:
                    gripper_command = 1
                    twist_command = z_twist_command(current_pose.position.z, moveit.height)
                elif timestep > 7:
                    gripper_command = 1
                    twist_command = random_twist_command()

                image = None
                while image is None:
                    image = camera.capture(timestep)
                ee_vel = [twist_command.twist.linear.x, twist_command.twist.linear.y, twist_command.twist.linear.z]
                ee_pos = [current_pose.position.x, current_pose.position.y, current_pose.position.z]
                position, _, _ = get_joint_states()

                if gripper_command == 1:
                    send_gripper_command(gripper_command)

                pub.publish(twist_command)
                _, velocity, _ = get_joint_states()

                images.append(image)

                #appending gripper command to the full action
                full_action = ee_vel + [gripper_command]
                actions.append(full_action)
                vel_actions.append(velocity)
                qts.append(position)
                ee_qts.append(ee_pos)

                rate.sleep()

                #finally publish a zero command to stop arm from moving
                twist_command = new_twist_command()
                pub.publish(twist_command)

        send_gripper_command(1)

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
    print("Total errors: %d" % total_errors)


def main(episodes, random_motion_timesteps, grasping_timesteps, reaching_timesteps, dropping_timesteps, saveinterval, save_path,port, startepisode, num_objects):

    try:
        camera = RealCamera(port)
        processes = camera.processes
        newstdin = os.fdopen(os.dup(sys.stdin.fileno()))
        processes.append(Process(target=start_data_collection, args=(episodes, random_motion_timesteps, grasping_timesteps, reaching_timesteps,
                        dropping_timesteps, saveinterval, save_path, port, startepisode, camera, num_objects, newstdin)))

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
    main(int(args.episodes), int(args.random_motion_timesteps), int(args.grasping_timesteps), 
        int(args.reaching_timesteps), int(args.dropping_timesteps), int(args.saveinterval), args.savepath, 
        int(args.port), int(args.startepisode),int(args.num_objects))
