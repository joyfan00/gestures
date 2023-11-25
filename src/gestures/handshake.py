#!/usr/bin/env python3

import roslib
roslib.load_manifest("gestures")

#!/usr/bin/env python3

#!/usr/bin/env python3

import argparse
import struct
import sys
import copy
import rospy
import rospkg
from gazebo_msgs.srv import (SpawnModel, DeleteModel,)
from geometry_msgs.msg import (PoseStamped, Pose, Point, Quaternion,)
from std_msgs.msg import (Header, Empty,)
from baxter_core_msgs.srv import (SolvePositionIK, SolvePositionIKRequest,)
import baxter_interface

class Handshake(object):
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        print("Initializing Node")
        self._limb_name = limb
        self._hover_distance = hover_distance
        self._verbose = verbose
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        # Define the joint angles for the starting position of the high-five
        # start_angles = {
        #     'left_w0': 0.6699952259595108,
        #     'left_w1': 1.030009435085784,
        #     'left_w2': -3.059,
        #     'left_e0': -1.189968899785275,
        #     'left_e1': 1.9400238130755056,
        #     'left_s0': -0.08000397926829805,
        #     'left_s1': -0.9999781166910
        # }
	# right_w0: 2.5
        start_angles = {
            'right_w0': 2.0,
            'right_w1': 0,
            'right_w2': 0.65,
            'right_e0': 2,
            'right_e1': 0.8,
            'right_s0': 0.15,
            'right_s1': 0.35
        }
        #start_angles = {
        #    'right_w0': 2.0,
        #    'right_w1': 0,
        #    'right_w2': 0.65,
        #    'right_e0': 2,
        #    'right_e1': 0,
        #    'right_s0': 0.75,
        #    'right_s1': 0.25
        #}

        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(0.02)

    def move_to_high_five(self, high_five_angles=None):
        # Define the joint angles for the high-five pose
        start_angles = {
            'right_w0': 2.0,
            'right_w1': 0,
            'right_w2': 0.65,
            'right_e0': 2.5,
            'right_e1': 0,
            'right_s0': 0.75,
            'right_s1': 0.15
        }

        self._guarded_move_to_joint_position(high_five_angles)
        self.gripper_open()
        rospy.sleep(0.05)

    def ik_request(self, pose):
        # Include your inverse kinematics request code here
        pass

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(0.02)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(0.1)

def main():
    print("HELLO")
    rospy.init_node("handshake")
    # rospy.wait_for_message("/robot/sim/started", Empty)
    limb = 'right'
    hover_distance = 0.15
    robot = Handshake(limb, hover_distance)
    
    # Move to the desired starting angles
    robot.move_to_start()
    rospy.sleep(0.05)
    
    # Move to the high-five pose
    # robot.move_to_high_five()
    # rospy.sleep(1.0)

    robot._rs.disable()

    # Return or continue with other actions

if __name__ == '__main__':
    sys.exit(main())
