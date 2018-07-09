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

from image_geometry import PinholeCameraModel

from gazebo_msgs.srv import SpawnModel, DeleteModel
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest

REAL = False
SIM  = True

class BlockSpawner():
    def __init__(self):
        self.blocks = []

    def spawn_block(self, block_pose, block_reference, block_dim, block_color):
        if(block_dim == None):
            block_dim = random.choice(["1x1", "1x2", "1x3"])
        if(block_color == None):
            block_color = random.choice(["blue", "red", "yellow", "green"])

        rospy.loginfo("Loading a %s, %s, megablok from file", block_color, block_dim)

        block_dir = rospkg.RosPack().get_path('lego_gazebo') + "/models/"
        block_path = block_dir + "megabloks" + block_dim + "_" + block_color + "/model.sdf"

        block_xml = ''
        with open (block_path, "r") as block_file:
            block_xml = block_file.read().replace('\n', '')
        
        rospy.wait_for_service('/gazebo/spawn_sdf_model')


        if(block_pose == None): 
            block_pos_x = random.uniform(1 -.4, 1 + .4)
            block_pos_y = random.uniform(-.4, .4)
            block_pos_z = 0.7825 # Height of table

            #block_rot = random.uniform(0, math.pi)
            block_rot = -math.pi/4

            rospy.loginfo("Creating a block with random location x: %f, y: %f, z: %f, theta: %f", block_pos_x, block_pos_y, block_pos_z, block_rot * 180.0 / math.pi)
            
            block_position = Point(x = block_pos_x, y = block_pos_y, z = block_pos_z)
            block_orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, block_rot))
            print(type(block_orientation))
            block_pose = Pose(position=block_position, orientation = block_orientation)

        else:
            block_pos_x = pose.position.x
            block_pos_y = pose.position.y
            block_pos_z = pose.position.z

            rospy.loginfo("Creating a block  with set location x: %f, y: %f, z: %f", block_pos_x, block_pos_y, block_pos_z)


        try:
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
            block_handle = "block_" + str(len(self.blocks)) + "_" + block_dim + "_" + block_color

            resp_sdf = spawn_sdf(block_handle, block_xml, "/",
                                block_pose, block_reference)

        except rospy.ServiceException, e:
            rospy.logerr("Spawn SDF (BLOCK) service call failed: {0}".format(e))

        self.blocks.append(block_handle)

    def delete_blocks(self):
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
            for block in self.blocks:
                resp_delete = delete_model(block)
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))




# Most of this is from PickAndPlace Baxter demo
class BlockPicker():
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
        self.block_pose = Pose()

    def subscribe(self):
        topic = "/block_finder/left_hand/block_poses"
        self.pose_sub = rospy.Subscriber(topic, PoseArray, self.pose_callback)

    def pose_callback(self, data):
        rospy.loginfo("Block found!")
        poses = data.poses
        pose = poses[0]

        block_position = pose.position

        #block_position.x -= (.75-.6725)
        #block_position.y -= (.15 - .1265)  
        block_position.z = 0.2 
        self.block_pose = block_position
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
        self._approach(pose)
        self._servo_to_pose(pose)

    def place(self, pose):
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

def spawn_table(table_pose=Pose(position=Point(x=1.0, y=0.0, z=0.0)), table_reference_frame="world"):

    # Get Models' Path
    model_path = rospkg.RosPack().get_path('baxter_sim_examples')+"/models/"

    # Load Table SDF
    table_xml = ''
    with open (model_path + "cafe_table/model.sdf", "r") as table_file:
        table_xml=table_file.read().replace('\n', '')

    # Spawn Table SDF
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp_sdf = spawn_sdf("cafe_table", table_xml, "/",
                             table_pose, table_reference_frame)
    except rospy.ServiceException, e:
        rospy.logerr("Spawn SDF (TABLE) service call failed: {0}".format(e))
       

def delete_table():
    # This will be called on ROS Exit, deleting Gazebo models
    # Do not wait for the Gazebo Delete Model service, since
    # Gazebo should already be running. If the service is not
    # available since Gazebo has been killed, it is fine to error out
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        resp_delete = delete_model("cafe_table")
    except rospy.ServiceException, e:
        rospy.loginfo("Delete Model service call failed: {0}".format(e))

def main():
    rospy.init_node("block_picker")

    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame
    spawn_table()

    # Remove models from the scene on shutdown
    rospy.on_shutdown(delete_table)

    # Wait for the All Clear from emulator startup
    # TODO: Change this when running on real baxter
    rospy.wait_for_message("/robot/sim/started", Empty)

    limb = 'left'
    hover_distance = 0.15 # meters

    # Starting Joint angles for left arm
    starting_joint_angles = {'left_w0': 0.6699952259595108,
                             'left_w1': 1.030009435085784,
                             'left_w2': -0.4999997247485215,
                             'left_e0': -1.189968899785275,
                             'left_e1': 1.9400238130755056,
                             'left_s0': -0.08000397926829805,
                             'left_s1': -0.9999781166910306}
    block_spawner = BlockSpawner()
    for i in range(10):
        block_spawner.spawn_block(block_pose=None, block_reference = "world", block_dim = None, block_color = "blue")

    rospy.on_shutdown(block_spawner.delete_blocks)
    

    block_picker = BlockPicker(limb, hover_distance)
    block_picker.subscribe()

    # An orientation for gripper fingers to be overhead and parallel to the obj
    overhead_orientation = Quaternion(
                            x=-0.0249590815779,
                            y=0.999649402929,
                            z=0.00737916180073,
                            w=0.00486450832011)

    starting_orientation = Quaternion(
                            x=0,
                            y=0,
                            z=0,
                            w=0)
     
    # Move to the desired starting angles
    block_picker.move_to_start(starting_joint_angles)
    # Move to the desired starting angles
    
    start_position = Point()
    start_position.x = 0.2
    start_position.y = 0.0
    start_position.z = 0.2

    

    start_pose = Pose(position=start_position, orientation=starting_orientation)

    #block_picker.move_to_pose(start_pose)
    # TODO: Set above
    rate = rospy.Rate(10)

    idx = 0
    while not rospy.is_shutdown():
        if(block_picker.block_detected):
            rospy.loginfo("Block has been detected")
            # Pick up block
            
            block_picker.move_to_pose(Pose(position = block_picker.block_pose, orientation=overhead_orientation))
            block_picker.block_detected = False     

        rate.sleep()
    return

if __name__ == '__main__':
     main()
