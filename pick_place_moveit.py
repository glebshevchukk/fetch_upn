import moveit_commander
import time
from tf.transformations import quaternion_from_euler,euler_from_quaternion

import geometry_msgs.msg
from geometry_msgs.msg import Point,Pose, Quaternion
from std_msgs.msg import String

import roslib
roslib.load_manifest('joint_listener')
from joint_listener.srv import *
import copy
import rospy
import numpy.random as npr

joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

quat = list(quaternion_from_euler(0, 1.5707, 0))
upright = Quaternion(quat[0],quat[1],quat[2],quat[3])


f_lower_x = 0.4
f_upper_x = 0.75

f_lower_y = -0.2
f_upper_y = 0.2


f_height=1.2
f_mid_height = 0.97
f_down_height = 0.91

f_start_x = 0.42
f_start_y = 0

distant_radius = 0.05


class MoveIt(object):
    def __init__(self):

        self.lower_point = None
        self.upper_point = None
        self.lower_opp_point = None
        self.upper_opp_point = None
        self.start_point = None


        self.lower_x = None
        self.lower_y = None
        self.upper_x = None
        self.upper_y = None
        self.height = None
        self.down_height = None
        self.mid_height = None

        self.object_poses = None


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

            self.lower_x = f_lower_x
            self.lower_y = f_lower_y
            self.upper_x = f_upper_x
            self.upper_y = f_upper_y
            self.height = f_height
            self.down_height = f_down_height
            self.mid_height = f_mid_height
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


    def get_object_positions(self, num_objects,newstdin):
        sys.stdin = newstdin
        object_poses = []
        for i in range(num_objects):
            try:
              input("Please set the end effector to next object then click ENTER")
            except:
              pass        
            object_pose = self.get_ee_pose()
            object_poses.append(object_pose)

        self.object_poses = object_poses
        return object_poses


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

    def orientation_violated(self):
        pose = self.get_ee_pose()

        quat = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
        euler = euler_from_quaternion(quat)

        if euler[1] < 1.45 or euler[1] > 1.58:
            print(euler[1])
            return True
        else:
            return False


    def not_at_start(self):
        pose = self.get_ee_pose()

        for c,s in zip([pose.position.x, pose.position.y, pose.position.z],[self.start_point.position.x, self.start_point.position.y, self.start_point.position.z]):

            if c < s - 0.01 or c > s + 0.01:

                return True

        return False


    def get_ee_pose(self):
        return self.group.get_current_pose().pose

    def get_valid_distant_pose(self):
        poses = self.object_poses + [self.get_ee_pose()]
        attempts = 0
        while attempts < 20:
            random_pose = self.get_random_pose()
            success = True
            for pose in poses:

                if (random_pose.position.x - pose.position.x > distant_radius and 
                random_pose.position.y - pose.position.y > distant_radius and
                random_pose.position.z - pose.position.z > distant_radius):
                    continue
                else:
                    success=False
            if success:
                return random_pose

            attempts += 1

        return self.get_random_pose()


    def get_random_pose(self):
        x = npr.uniform(low=self.lower_x, high=self.upper_x)
        y = npr.uniform(low=self.lower_y, high=self.upper_y)
        
        pose = Point(x, y, self.height)
        pose = Pose(position=pose, orientation=upright)

        return pose

    def exceeds_bounds(self):

        current_pose = self.get_ee_pose()

        if current_pose.position.x < self.lower_point.position.x or current_pose.position.y < self.lower_point.position.y or current_pose.position.x > self.upper_point.position.x or current_pose.position.y > self.upper_point.position.y:
            return True
        else:
            return False



