import moveit_commander
import time
from tf.transformations import quaternion_from_euler,euler_from_quaternion

import geometry_msgs.msg
from geometry_msgs.msg import Point,Pose, Quaternion
from std_msgs.msg import String

import roslib
roslib.load_manifest('joint_listener')
from joint_listener.srv import ReturnJointStates, ReturnEEPose
import moveit_commander
from tf.transformations import quaternion_from_euler,euler_from_quaternion
import copy
import numpy.random as npr
import random

joint_names = ["torso_lift_joint", "shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint", "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

quat = list(quaternion_from_euler(0, 1.5707, 0))
upright = Quaternion(quat[0],quat[1],quat[2],quat[3])


f_lower_x = 0.35
f_upper_x = 0.75

f_lower_y = -0.25
f_upper_y = 0.25


f_height=0.97
upper_f_height = 1.1

f_start_x = 0.4
f_start_y = 0.15

mid_x = f_lower_y + (f_upper_x - f_lower_x)/ 2
mid_y = f_lower_y + (f_upper_y - f_lower_y)/ 2



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
        self.middle = None

        self.corner_points = None


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

        self.corner_points = [self.lower_point, self.lower_opp_point, self.upper_point, self.upper_opp_point]

        return True


    def get_start_point(self, fixed=True):

        if fixed:
            self.fixed_start_point = Pose(position=Point(f_start_x, f_start_y, f_height), orientation=upright)
            self.start_point = self.fixed_start_point
            self.middle = copy.deepcopy(self.start_point)
            self.middle.position.x = 0.55
            return True
        else:
            try:
              input("Please set the end effector to the point you want it to always start at, then click ENTER")
            except:
              pass
            if not self.exceeds_bounds():
                self.fixed_start_point = self.get_ee_pose()
                print(self.fixed_start_point)
                return True
            else:
                print("Start state is incorrect, please try again")
                return False

    

    def go_to_start(self, rand=False, corner=False):
        # if corner:
        #     print("Going to a random corner")
        #     r_index = random.randint(0,4)
        #     self.start_point = self.corner_points[r_index]

        # elif not corner and random:
        #     print("Going to a random point")
        #     self.start_point = self.get_random_pose()
        # else:
        print("Going to a fixed point")
        self.start_point = self.fixed_start_point
        self.start_point.position.z = upper_f_height


        while self.not_at_upper_start():
            self.group.clear_pose_targets()
            self.group.set_pose_target(self.start_point)
            self.group.plan()
            self.group.go(wait=True)


        self.start_point.position.z = f_height
        self.group.clear_pose_targets()
        self.group.set_pose_target(self.start_point)
        self.group.plan()
        self.group.go(wait=True)

    def go_to(self, pose, high=False):
        pose = copy.deepcopy(pose)
        if high:
          pose.position.z = 1.15
        else:
          pose.position.z = 0.97
        self.group.clear_pose_targets()
        self.group.set_pose_target(pose)
        self.group.plan()
        self.group.go(wait=True)

    def linear_go_to(self, pose, high=False):
        current_pose = self.get_ee_pose()
        pose = copy.deepcopy(pose)
        if high:
          pose.position.z = 1.2
        else:
          pose.position.z = 0.97
        self.group.clear_pose_targets()

        waypoints = [current_pose,pose]
        
        (plan, fraction) = self.group.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.1,        # eef_step
                                 0.0, avoid_collisions = False)
        self.group.execute(plan, wait=True)
        time.sleep(1)


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
        self.start_point.position.z = f_height
        for c,s in zip([pose.position.x, pose.position.y, pose.position.z],[self.start_point.position.x, self.start_point.position.y, self.start_point.position.z]):

            if c < s - 0.01 or c > s + 0.01:

                return True

        return False

    def not_at_upper_start(self):
        self.start_point.position.z = upper_f_height
        pose = self.get_ee_pose()

        for c,s in zip([pose.position.x, pose.position.y, pose.position.z],[self.start_point.position.x, self.start_point.position.y, self.start_point.position.z]):

            if c < s - 0.01 or c > s + 0.01:

                return True

        return False


    def get_ee_pose(self):
        return self.group.get_current_pose().pose

    def exceeds_bounds(self):

        current_pose = self.get_ee_pose()

        if current_pose.position.x < self.lower_point.position.x or current_pose.position.y < self.lower_point.position.y or current_pose.position.x > self.upper_point.position.x or current_pose.position.y > self.upper_point.position.y:
            return True
        else:
            return False

    def trace_perimeter(self):

    
        for point in [self.lower_point, self.lower_opp_point, self.upper_point, self.upper_opp_point, self.middle]:
            self.group.clear_pose_targets()
            waypoints = []

            waypoints.append(self.get_ee_pose())
            waypoints.append(copy.deepcopy(point))

            print("Made waypoint to next point.")

            (plan, fraction) = self.group.compute_cartesian_path(
                                 waypoints,   # waypoints to follow
                                 0.1,        # eef_step
                                 0.0, avoid_collisions = False)         # jump_threshold

            self.group.execute(plan, wait=True)
            time.sleep(2)
            print("Succesfully executed cartesian path")



    def shuffle_reset(self):

        # mid_point_1 = Pose(position=Point(mid_x, lower_y,f_height), orientation=upright)
        # mid_point_2 = Pose(position=Point(mid_x, upper_y,f_height), orientation=upright)
        # mid_point_3 = Pose(position=Point(lower_x, mid_y,f_height), orientation=upright)
        # mid_point_4 = Pose(position=Point(upper_x, mid_y,f_height), orientation=upright)


        #goes to corners

        self.linear_go_to(self.upper_point, True)
        self.linear_go_to(self.upper_point)
        self.linear_go_to(self.middle)
        
        self.linear_go_to(self.lower_point, True)
        self.linear_go_to(self.lower_point)
        self.linear_go_to(self.middle)
        
        self.linear_go_to(self.lower_opp_point, True)
        self.linear_go_to(self.lower_opp_point)
        self.linear_go_to(self.middle)

        self.linear_go_to(self.upper_opp_point, True)
        self.linear_go_to(self.upper_opp_point)
        self.linear_go_to(self.middle)

        self.linear_go_to(self.start_point)

        #goes to sides

        # self.linear_go_to(mid_point_1, True)
        # self.go_to(mid_point_1)
        # self.linear_go_to(self.middle)
        
        # self.linear_go_to(mid_point_2, True)
        # self.go_to(mid_point_2)
        # self.linear_go_to(self.middle)
        
        # self.linear_go_to(mid_point_3, True)
        # self.go_to(mid_point_3)
        # self.linear_go_to(self.middle)

        # self.linear_go_to(mid_point_4, True)
        # self.go_to(mid_point_4)
        # self.linear_go_to(self.middle)

    def get_random_pose(self):
        x = npr.uniform(low=self.lower_x, high=self.upper_x)
        y = npr.uniform(low=self.lower_y, high=self.upper_y)
        
        pose = Point(x, y, self.height)
        pose = Pose(position=pose, orientation=upright)

        return pose

    def make_new_pose(self,xp,yp):
        
        pose = Point(xp, yp, self.height)
        pose = Pose(position=pose, orientation=upright)

        return pose
