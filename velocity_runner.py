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
from tf.transformations import quaternion_from_euler

from gripper_client import *
from shell import *


# server = "sadigh-ws-1.stanford.edu"
# user= "glebs"
# password="robotics"

# workstation_path = '/scr/glebs/dev/upn/data/1_test/'

# sh = ShellHandler(server,user,password)


scale_control = 20
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


f_lower_x = 0.35
f_upper_x = 0.7

#mid_x = lower_x + (upper_x-lower_x)/2.

f_lower_y = -0.2
f_upper_y = 0.2

#mid_y = lower_y + (upper_y - lower_y)/2.

f_height=0.96

f_start_x = 0.6

f_start_y = 0



class MoveIt(object):
    def __init__(self):

        self.lower_point = None
        self.upper_point = None
        self.lower_opp_point = None
        self.upper_opp_point = None
        self.start_point = None


        self.robot = moveit_commander.RobotCommander()
        self.group = moveit_commander.MoveGroupCommander("arm")
        self.waypoints = []

    def get_ee_bounds(self, fixed=True):
        if fixed:

            lower_x = f_lower_x
            lower_y = f_lower_y
            upper_x = f_upper_x
            upper_y = f_upper_y
            height = f_height


        else:

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
                lower_x = second_pose.position.x
                lower_y = second_pose.position.y
                
                upper_x = first_pose.position.x
                upper_y = first_pose.position.y

            elif first_pose.position.x < second_pose.position.x and first_pose.position.y < second_pose.position.y:
                lower_x = first_pose.position.x
                lower_y = first_pose.position.y
                
                upper_x = second_pose.position.x
                upper_y = second_pose.position.y

            else:
                print("Boundaries are incorrect, please try again")
                return False

            height = f_height

        self.lower_point = Point(lower_x, lower_y,height)
        self.lower_point = Pose(position=self.lower_point, orientation=upright)

        self.lower_opp_point = Point(lower_x, upper_y, height)
        self.lower_opp_point = Pose(position=self.lower_opp_point, orientation=upright)

        self.upper_point = Point(upper_x, upper_y, height)
        self.upper_point = Pose(position=self.upper_point, orientation=upright)

        self.upper_opp_point = Point(upper_x, lower_y, height)
        self.upper_opp_point = Pose(position=self.upper_opp_point, orientation=upright)

        return True


    def get_start_point(self, fixed=True):

        if fixed:
            self.start_point = Pose(position=Point(f_start_x, f_start_y, f_height), orientation=upright)
            return True
        else:
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

    def go_to(self, pose, high=False):
        pose = copy.deepcopy(pose)
        if high:
          pose.position.z = 1.05
        self.group.clear_pose_targets()
        self.group.set_pose_target(pose)
        self.group.plan()
        self.group.go(wait=True)


    def fix_orientation(self):
        self.group.clear_pose_targets()
        pose = self.get_ee_pose()
        pose.orientation = upright
        self.group.set_pose_target(pose)
        self.group.plan()
        self.group.go(wait=True)


    def exceeds_bounds(self):

        current_pose = self.get_ee_pose()

        if current_pose.position.x < self.lower_point.position.x or current_pose.position.y < self.lower_point.position.y or current_pose.position.x > self.upper_point.position.x or current_pose.position.y > self.upper_point.position.y:
            return True
        else:
            return False

    def trace_perimeter(self):
        self.group.clear_pose_targets()
        waypoints = []

        waypoints.append(self.get_ee_pose())

        for point in [self.lower_point, self.lower_opp_point, self.upper_point, self.upper_opp_point]:
            waypoints.append(copy.deepcopy(point))

        (plan, fraction) = self.group.compute_cartesian_path(
                             waypoints,   # waypoints to follow
                             0.01,        # eef_step
                             0.0, avoid_collisions = False)         # jump_threshold

        print("Succesfully made perimeter plan")

        self.group.execute(plan, wait=True)



    def shuffle_reset(self):

        #mid_point_1 = Pose(position=Point(mid_x, lower_y,height), orientation=upright)
        #mid_point_2 = Pose(position=Point(mid_x, upper_y,height), orientation=upright)
        #mid_point_3 = Pose(position=Point(lower_x, mid_y,height), orientation=upright)
        #mid_point_4 = Pose(position=Point(upper_x, mid_y,height), orientation=upright)


        #goes to corners
        self.go_to(self.start_point, True)
        self.go_to(self.upper_point, True)
        self.go_to(self.upper_point)
        self.go_to(self.start_point)
        
        self.go_to(self.start_point, True)
        self.go_to(self.lower_point, True)
        self.go_to(self.lower_point)
        self.go_to(self.start_point)
        
        self.go_to(self.start_point, True)
        self.go_to(self.lower_opp_point, True)
        self.go_to(self.lower_opp_point)
        self.go_to(self.start_point)

        self.go_to(self.start_point, True)
        self.go_to(self.upper_opp_point, True)
        self.go_to(self.upper_opp_point)
        self.go_to(self.start_point)

        #goes to sides

        #self.go_to(self.start_point, True)
        #self.go_to(mid_point_1, True)
        #self.go_to(mid_point_1)
        #self.go_to(self.start_point)
        
        #self.go_to(self.start_point, True)
        #self.go_to(mid_point_2, True)
        #self.go_to(mid_point_2)
        #self.go_to(self.start_point)
        
        #self.go_to(self.start_point, True)
        #self.go_to(mid_point_3, True)
        #self.go_to(mid_point_3)
        #self.go_to(self.start_point)

        #self.go_to(self.start_point, True)
        #self.go_to(mid_point_4, True)
        #self.go_to(mid_point_4)
        #self.go_to(self.start_point)



        

def start_data_collection(episodes, timesteps, saveinterval, save_path,port):

    pub = rospy.Publisher('/arm_controller/cartesian_twist/command', TwistStamped)
    rospy.init_node('send_velocities')
    rate = rospy.Rate(3)
    reset_rate = rospy.Rate(0.2)
    
    camera = RealCamera(port)
    #camera.view()

    moveit = MoveIt()

    moveit.get_ee_bounds(fixed=True)

    # if not good_bounds:
    #    return

    moveit.get_start_point(fixed=True)

    # if not good_start:
    #    return   

    start_time = 0

    all_images = []
    all_actions = []
    all_qts = []

    for episode in range(episodes):

        moveit.go_to_start()
        time.sleep(2)

        images = []
        actions = []
        qts = []
        
        #move_gripper(1)
        #moveit.shuffle_reset()
        #moveit.trace_perimeter()\
        print("Starting episode %d" % episode)

       
        for timestep in range(timesteps):
            print(timestep)
      
            image = camera.capture()
            #image = np.zeros((100,100,3))
            position, velocity, effort = get_joint_states()
            current_pose = moveit.get_ee_pose()
            if current_pose.position.x < moveit.lower_point.position.x:
                twist_command = new_twist_command(x=2)
            elif current_pose.position.y < moveit.lower_point.position.y:
                twist_command = new_twist_command(y=2)
            elif current_pose.position.x > moveit.upper_point.position.x:
                twist_command = new_twist_command(x=-2)
            elif current_pose.position.y > moveit.upper_point.position.y:
                twist_command = new_twist_command(y=-2)
            else:
              twist_command = random_twist_command()

            pub.publish(twist_command)
            rate.sleep()

            images.append(image)
            actions.append(velocity)
            qts.append(position)

        final_image = camera.capture()
        images.append(final_image)

        all_images.append(images)
        all_actions.append(actions)
        all_qts.append(qts)
        print("Collected single set for episode %d" % episode)

        if (episode % saveinterval) == 0:

            time_interval = str(start_time) + '-' + str(episode)
            pickle.dump(all_images,open(save_path + '/images/images_' + time_interval + '.pickle','wb'))
            pickle.dump(all_actions,open(save_path + '/actions/actions_' + time_interval + '.pickle','wb'))
            pickle.dump(all_qts,open(save_path + '/qts/qts_' + time_interval + '.pickle','wb'))

            all_images = []
            all_actions = []
            all_qts = []

            start_time =  episode

        # if (episode % 1) == 0:

        #     send_to_workstation(save_path)

        del images
        del actions
        del qts

    camera.close()

def random_twist_command():
    global global_random_counter
    control = npr.uniform(low=-scale_control, high=scale_control, size=(2))
    twist = TwistStamped()
    #twist.header.stamp = rospy.Time.now()
    global_random_counter += 1
    twist.header.seq = global_random_counter
    twist.header.frame_id = 'base_link'
    twist.twist.linear.x = control[0]
    twist.twist.linear.y = control[1]
    #twist.twist.linear.x = 1
    #twist.twist.linear.y = 1
    twist.twist.linear.z = 0

    return twist

def new_twist_command(x=0, y=0, z=0):
    global global_random_counter
    twist = TwistStamped()
    #twist.header.stamp = rospy.Time.now()
    global_random_counter += 1
    twist.header.seq = global_random_counter
    twist.header.frame_id = 'base_link'
    twist.twist.linear.x = x
    twist.twist.linear.y = y
    #twist.twist.linear.x = 1
    #twist.twist.linear.y = 1
    twist.twist.linear.z = z

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


# def send_to_workstation(save_path):

#     all_files = glob.glob(save_path + '/*')

#     for file in all_files:

#         sh.send_file(file, save_path, workstation_path)





if __name__ == "__main__":
    args = parser.parse_args()
    start_data_collection(int(args.episodes), int(args.timesteps), int(args.saveinterval), args.savepath, int(args.port))
