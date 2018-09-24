#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import struct
import sys
import copy
import rospkg
import random
import math
import tf_conversions

import baxter_interface

import tf

from cv_bridge import CvBridge
from matplotlib import pyplot as plt

from std_msgs.msg import String, Int32, Header, Empty
from sensor_msgs.msg import Image, CameraInfo, Range
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Point, Quaternion

from visualization_msgs.msg import Marker, MarkerArray

from image_geometry import PinholeCameraModel

from gazebo_msgs.srv import SpawnModel, DeleteModel
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

SIM = True
REAL = False


# Most of this is from PickAndPlace Baxter demo
class CenterObject():
    def __init__(self, limb, hover_distance = 0.15, verbose=True):
        self._limb_name = limb # string
        self._hover_distance = hover_distance # in meters
        self._verbose = verbose # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)
        ns = "ExternalTools/" + self._limb_name + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()
        self.block_detected = False

    def subscribe(self):
        topic = "/block_finder/" + self._limb_name + "_hand" + "/block_xy"
        self.pose_sub = rospy.Subscriber(topic, Point, self.block_xy_callback)

    def block_xy_callback(self, data):
        self.loc_x = data.x
        self.loc_y = data.y
        self.block_detected = True


    def move_to_start(self, start_angles=None):
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")


    def ik_request(self, pose):
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

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

    def _approach(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x 
        ik_pose.position.y = current_pose['position'].y 
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x 
        ik_pose.orientation.y = current_pose['orientation'].y 
        ik_pose.orientation.z = current_pose['orientation'].z 
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    def pick(self, pose):
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()


    def move_to_pose(self, pose):
        rospy.loginfo("Moving to pose x:%f, y:%f, z:%f", pose.position.x, pose.position.y, pose.position.z)
        self._approach(pose)
        self._servo_to_pose(pose)


    def move_to_above_pose(self, pose, height):
        rospy.loginfo("Moving above detected block")
        pose.position.z = height
        self.move_to_pose(pose)

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def main():
    rospy.init_node("center_object")

    limb = 'right'
    hover_distance = 0.15 # meters

    # Starting Joint angles for arm
    starting_joint_angles = {limb + "_w0": -0.6699952259595108,
                             limb + "_w1": 1.030009435085784,
                             limb + "_w2": 0.6, #-0.4999997247485215,
                             limb + "_e0": 1.189968899785275,
                             limb + "_e1": 1.9400238130755056,
                             limb + "_s0": -0.08000397926829805,
                             limb + "_s1": -0.9999781166910306}


    center_object = CenterObject(limb)
    center_object.subscribe()

    starting_orientation = Quaternion(
                            x=0,
                            y=0,
                            z=0,
                            w=0)

    center_object.gripper_open()

    # Move to the desired starting angles
    center_object.move_to_start(starting_joint_angles)

    block_choice = 0

    rate = rospy.Rate(10)

    idx = 0
    above_block = False
    while not rospy.is_shutdown():
        if(SIM):
            # First store the initial vector to the block

            rospy.loginfo("Running center_object on Simulated Baxter")
            if(center_object.block_detected):
                if(not first_location_recorded):
                    # At least one block has been found
                    rospy.loginfo("At least one block has been found")
                    

                    first_block_loc = np.array([center_object.loc_x, center_object.loc_y])
                    first_location_recorded = True

                    # Move closer TODO: Make a PID
                    """
                    # Check how far away the block is from center
                    delta_x = center_x - center_object.loc_x
                    delta_y = center_y - center_object.loc_y

                    # Adjust position by some factor of delta_x
                    motion_scaling_factor = 0.01
                    curr_position.x += (delta_x * motion_scaling_factor)
                    curr_position.y += (delta_y * motion_scaling_factor)

                    center_object.move_to_pose(Pose(position = curr_position), orientation=starting_orientation)
                    """
                elif(not second_location_recorded):
                    # Move to a random location in an attempt to find a block
                    new_block_pos_x = random.uniform(.9 -.5, .8)
                    new_block_pos_y = random.uniform(0, -.4)
                    new_block_pos_z = 0.125 # Height of table
                    new_block_position = Point(x = new_block_pos_x, y = new_block_pos_y, z = new_block_pos_z)

                    center_object.move_to_above_pose(Pose(position = new_block_position, orientation=starting_orientation), .14)

                    first_block_loc = np.array([center_object.loc_x, center_object.loc_y])

                else:
                    # Have both first and second location, can now triangulate the location of block


            else:
                rospy.loginfo("Can't find block, moving to a new location.")
                # Move to a random location in an attempt to find a block
                new_block_pos_x = random.uniform(.9 -.5, .8)
                new_block_pos_y = random.uniform(0, -.4)
                new_block_pos_z = 0.125 # Height of table
                new_block_position = Point(x = new_block_pos_x, y = new_block_pos_y, z = new_block_pos_z)

                center_object.move_to_above_pose(Pose(position = new_block_position, orientation=starting_orientation), .14)
        else:
            rospy.loginfo("Running block_picker on Real Baxter")

        rate.sleep()
    return

if __name__ == '__main__': 
    main()           