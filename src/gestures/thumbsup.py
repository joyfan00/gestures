#!/usr/bin/env python3
import argparse
import struct
import sys
import copy

import rospy
import rospkg

from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from std_msgs.msg import (
    Header,
    Empty,
)

from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)

import baxter_interface

class thumb(object):
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        print("Running. Ctrl-c to quit")

    def move_to_thumb(self, thumb_angles=None):
        print("Moving the {0} arm to thumb pose...".format(self._limb_name))
        if not thumb_angles:
            thumb_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(thumb_angles)
        self.gripper_open()
        print("Running. Ctrl-c to quit")

    def move_to_rest(self, rest_angles=None):
        print("Moving the {0} arm to rest pose...".format(self._limb_name))
        if not rest_angles:
            rest_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(rest_angles)
        self.gripper_open()
        print("Running. Ctrl-c to quit")




    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")

    def gripper_open(self):
        self._gripper.open()
        rospy.sleep(1.0)

    def gripper_close(self):
        self._gripper.close()
        rospy.sleep(1.0)



def main():
    rospy.init_node("thumbsup")
    # Wait for the All Clear from emulator startup
    #rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15 # meters
    # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': -3,
                             'left_w1': 1.35,
                             'left_w2': -0.5,
                             'left_e0': 1.0,
                             'left_e1': 1.0,
                             'left_s0': 0.0,
                             'left_s1': 0.0}
    thumb_angles = {'left_w0': 0,
                             'left_w1': -0.5, #-1.4,
                             'left_w2': -0.0124707,
                             'left_e0':  0,
                             'left_e1': 1,
                             'left_s0': -0.8 ,
                             'left_s1':-0.5 }
    rest_angles = {'left_w0': -0.178079,
                             'left_w1': -0.0610333,
                             'left_w2': -0.0124707,
                             'left_e0': 0.000806359,
                             'left_e1': 0.491094,
                             'left_s0': 0.192483,
                             'left_s1': 1.047}






    robot= thumb(limb, hover_distance)
    robot.move_to_thumb(thumb_angles)
    robot._rs.disable()




if __name__ == '__main__':
    sys.exit(main())
