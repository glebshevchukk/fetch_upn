import sys
import rospy
import roslib
roslib.load_manifest('joint_listener')
from joint_listener.srv import *


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
    rospy.init_node("test_torso")
    max_torso_height()