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

from baxter_pykdl import baxter_kinematics

SIM = True
REAL = False


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
        self.block_poses = []
        self.pose = Pose()

    def subscribe(self):
        topic = "/block_finder/" + self._limb_name + "_hand" + "/block_poses"
        self.pose_sub = rospy.Subscriber(topic, PoseArray, self.pose_callback)

    def pose_callback(self, data):
        curr_block_poses = []
        poses = data.poses

        for pose in poses:
            #block_position.x -= (.75-.6725)
            #block_position.y -= (.15 - .1265)  
            pose.position.z = 0.05

            starting_orientation = Quaternion(
                            x=-0.0249590815779,
                            y=0.999649402929,
                            z=0.00737916180073,
                            w=0.00486450832011
                            )
            pose.orientation = starting_orientation
        

            curr_block_poses.append(pose)
        self.block_poses = curr_block_poses

        #TODO: uncomment!
        #self.block_detected = True



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
        print("Approaching ", pose)
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
        print("Servoing to pose: ", pose)
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


    def translate_camera(self, distance = 0.05, angle = math.pi/4):
        curr_q = self.pose.orientation

        # Get rotation matrix from current orientation
        rot = tf.transformations.quaternion_matrix([curr_q.w, curr_q.x, curr_q.y, curr_q.z])


        print(rot)
        trans_cam = np.zeros((1,4))

        # Keep same orientation, only translate in the direction of the angle by distance
        trans_cam[0, 0] = distance * math.cos(angle)
        trans_cam[0, 1] = distance * math.sin(angle)
        trans_cam[0, 2] = 0
        trans_cam[0, 3] = 1


        trans_world = np.dot(trans_cam, rot)
        print("Camera translation: ", trans_cam)

        trans_cam[0, :2] /= trans_cam[0, 3]

        new_pose = Pose()
        new_pose.position.x = self.pose.position.x + trans_cam[0, 0]
        new_pose.position.y = self.pose.position.y + trans_cam[0, 1]
        new_pose.position.z = self.pose.position.z + trans_cam[0, 2]
        """
        no_rot_orientation = Quaternion()
        no_rot_orientation.w = 1
        no_rot_orientation.x = 0
        no_rot_orientation.y = 0
        no_rot_orientation.z = 0
        """

        new_pose.orientation = self.pose.orientation
        
        self.move_to_pose(new_pose)

    def fixate_camera(self, center_pixel_loc):
        CENTER_X_PIXEL_THRESH = 10
        CENTER_Y_PIXEL_THRESH = 10
        
        curr_pixel_offset = self.current_pixel_loc - self.center_pixel_loc
        center_distance = math.hypot(curr_pixel_offset[0], curr_pixel_offset[1])

        while(center_distance > CENTER_PIXEL_THRESH):
            # last_pixel_offset = curr_pixel_offset.copy()

            # find distance from current_pixel_loc to center_pixel_loc
            #curr_pixel_offset = self.current_pixel_loc - self.center_pixel_loc

            #pitch_angle = math.pi/32 * (0.001) * curr_pixel_offset[1]
            #yaw_angle =   math.pi/32 * (0.001) * curr_pixel_offset[0]
            
            pitch_angle = math.pi/32 * (0.001) * 20
            yaw_angle =   math.pi/32 * (0.001) * 10

            fixate_quaternion = tf.transformations.quaternion_from_euler(0, pitch_angle, yaw_angle)
            
            new_pose = Pose()
            new_pose.position = self.pose.position


def spawn_table(table_pose=Pose(position=Point(x=.8, y=0.0, z=0.0)), table_reference_frame="world"):

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

    limb = 'right'
    hover_distance = 0.15 # meters

    # Starting Joint angles for left arm
    starting_joint_angles = {limb + "_w0": -0.6699952259595108,
                             limb + "_w1": 1.030009435085784,
                             limb + "_w2": 0.6, #-0.4999997247485215,
                             limb + "_e0": 1.189968899785275,
                             limb + "_e1": 1.9400238130755056,
                             limb + "_s0": -0.08000397926829805,
                             limb + "_s1": -0.9999781166910306}


    block_picker = BlockPicker(limb, hover_distance)
    block_picker.subscribe()

    baxter_kin = baxter_kinematics('right')

    

    # An orientation for gripper fingers to be overhead and parallel to the obj
    #overhead_orientation = Quaternion(
    #                        x=-0.0249590815779,
    #                        y=0.999649402929,
    #                        z=0.00737916180073,
    #                        w=0.00486450832011)

    starting_orientation = Quaternion(
                            x=-0.0249590815779,
                            y=0.999649402929,
                            z=0.00737916180073,
                            w=0.00486450832011)
                            #x=0,
                            #y=0,
                            #z=0,
                            #w=0)
    block_picker.gripper_open()
    # Move to the desired starting angles
    block_picker.move_to_start(starting_joint_angles)
    curr_kin = baxter_kin.forward_position_kinematics()

    block_picker.pose.position.x =    curr_kin[0]
    block_picker.pose.position.y =    curr_kin[1]
    block_picker.pose.position.z =    curr_kin[2]
    block_picker.pose.orientation.w = curr_kin[6]
    block_picker.pose.orientation.x = curr_kin[3]
    block_picker.pose.orientation.y = curr_kin[4]
    block_picker.pose.orientation.z = curr_kin[5]
    """
    block_picker.pose.orientation.w = 1
    block_picker.pose.orientation.x = 0
    block_picker.pose.orientation.y = 0
    block_picker.pose.orientation.z = 0
    """

    block_choice = 0

    rate = rospy.Rate(10)

    idx = 0
    above_block = False
    while not rospy.is_shutdown():
        if(SIM):
            rospy.loginfo("Running block_picker on Simulated Baxter")
            if(block_picker.block_detected):
                # At least one block has been found
                rospy.loginfo("At least one block has been found")

                if(above_block == True):
                    rospy.loginfo("Picking block up")
                    block_picker.move_to_above_pose(block_picker.block_poses[block_choice], -.14)
                    block_picker.gripper_close()
                    block_picker.move_to_above_pose(block_picker.block_poses[block_choice], .14)

                    # Only bottom left corner of table
                    new_block_pos_x = random.uniform(.9 -.5, .8)
                    new_block_pos_y = random.uniform(0, -.4)

                    new_block_pos_z = 0.125 # Height of table
                    new_block_position = Point(x = new_block_pos_x, y = new_block_pos_y, z = new_block_pos_z)

                    block_picker.move_to_above_pose(Pose(position = new_block_position, orientation=starting_orientation), .14)
                    block_picker.move_to_above_pose(Pose(position = new_block_position, orientation=starting_orientation), -.14)
                    block_picker.gripper_open()
                    block_picker.move_to_start(starting_joint_angles)

                    above_block = False
                
                else:
                    rospy.loginfo("%d blocks have been detected", len(block_picker.block_poses))
                    block_choice = int(raw_input("Which block do you want to pick up? (0-" + str(len(block_picker.block_poses))))

                    rospy.loginfo("Moving gripper to %f above block")
                    
                    block_picker.move_to_above_pose(block_picker.block_poses[block_choice], .14)
                    above_block = True

            else:
                """
                rospy.loginfo("Can't find block, moving to a new location.")
                # Move to a random location in an attempt to find a block
                new_block_pos_x = random.uniform(.9 -.5, .8)
                new_block_pos_y = random.uniform(0, -.4)
                new_block_pos_z = 0.125 # Height of table
                new_block_position = Point(x = new_block_pos_x, y = new_block_pos_y, z = new_block_pos_z)

                block_picker.move_to_above_pose(Pose(position = new_block_position, orientation=starting_orientation), .14)
                """
        
                block_picker.translate_camera()

                curr_kin = baxter_kin.forward_position_kinematics()

                block_picker.pose.position.x =    curr_kin[0]
                block_picker.pose.position.y =    curr_kin[1]
                block_picker.pose.position.z =    curr_kin[2]
                """
                block_picker.pose.orientation.w = 1
                block_picker.pose.orientation.x = 0
                block_picker.pose.orientation.y = 0
                block_picker.pose.orientation.z = 0
                """
                block_picker.pose.orientation.w = curr_kin[6]
                block_picker.pose.orientation.x = curr_kin[3]
                block_picker.pose.orientation.y = curr_kin[4]
                block_picker.pose.orientation.z = curr_kin[5

        else:
            rospy.loginfo("Running block_picker on Real Baxter")

        rate.sleep()
    return

if __name__ == '__main__': 
    main()           