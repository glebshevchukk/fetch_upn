import numpy as np
from capture_joints import *

import time
import sys
import rospy
from traj_27 import *

import random
import uuid

from shell import *
from reset_fetch_for_control import *




def record_runner(samples, time_sleep):
    rospy.init_node("record_runner")
    arm = FollowTrajectoryClient("arm_controller", ["shoulder_pan_joint",
                   "shoulder_lift_joint", "upperarm_roll_joint",
                   "elbow_flex_joint", "forearm_roll_joint",
                   "wrist_flex_joint", "wrist_roll_joint"])
    #arm.move_to([1.364, -0.294, -2.948, 0.906, -0.275, -1.206, 3.086])
    states = []
    controls = []
    print("Starting to capture joints from real Fetch robot")
    for i in range(samples):
        capture_joints(states,controls)
        time.sleep(time_sleep)
        print(i)
    print("Finished capturing joints from real Fetch robot")

    states = np.array([states])
    controls = np.array([controls])
    traj = Trajectory(states,controls)
    
    save_name = '/home/fetch/dev/upn/demos/trace.pickle'
    pic = open(save_file, "wb")
    pickle.dump(traj,pic)

    print("Trajectory pickle file has been saved to ",save_file)

    return save_name

if __name__ == "__main__":
    #reset_fetch_for_control()
    print("Running record runner")
    name = record_runner(200,0.25)

    print(name)
