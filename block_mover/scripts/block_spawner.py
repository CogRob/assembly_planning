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
import block_finder as bf

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

REAL = False
SIM  = True

class BlockSpawner():
    def __init__(self):
        self.blocks = []
        self.marker_list = MarkerArray()
        self.block_colors = ["red", "yellow", "green", "blue"]
        self.color_index = 0

    def publish(self):
        self.marker_pub  = rospy.Publisher("block_spawner/" "/block_ground_truths", MarkerArray, queue_size=10)

    def spawn_block(self, block_pose, block_reference, block_dim, block_color):
        if(block_dim == None):
            block_dim = random.choice(["1x1", "1x2", "1x3"])
        if(block_color == None):
            # Randomize color
            #block_color = random.choice(["blue", "red", "green", "yellow"])
            # Alternate color
            block_color = self.block_colors[self.color_index]
            self.color_index = (self.color_index + 1) % 4

        rospy.loginfo("Loading a %s, %s, megablok from file", block_color, block_dim)

        block_dir = rospkg.RosPack().get_path('lego_gazebo') + "/models/"
        block_path = block_dir + "megabloks" + block_dim + "_" + block_color + "/model.sdf"

        block_xml = ''
        with open (block_path, "r") as block_file:
            block_xml = block_file.read().replace('\n', '')
        
        rospy.wait_for_service('/gazebo/spawn_sdf_model')



        if(block_pose == None): 
            # Only bottom left corner of table
            block_pos_x = random.uniform(1 -.6, .7)
            block_pos_y = random.uniform(-.2, -.4)
            block_pos_z = 0.7825 # Height of table

            # Entire table
            """
            block_pos_x = random.uniform(1 -.4, 1 + .4)
            block_pos_y = random.uniform(-.4, .4)
            block_pos_z = 0.7825 # Height of table
            """

            # TODO: change to random to test how different orientations affect algo
            #block_rot = random.uniform(0, math.pi)
            #block_rot = -math.pi/4
            block_rot = 0 

            rospy.loginfo("Creating a block with random location x: %f, y: %f, z: %f, theta: %f", block_pos_x, block_pos_y, block_pos_z, block_rot * 180.0 / math.pi)
            # Only bottom right corner of table
            block_pos_x = random.uniform(1 -.45, .7)
            block_pos_y = random.uniform(-.2, -.4)
            block_pos_z = 0.7825 # Height of table
            block_position = Point(x = block_pos_x, y = block_pos_y, z = block_pos_z)
            block_orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, block_rot))

            block_pose = Pose(position=block_position, orientation = block_orientation)


        else:
            block_pos_x = block_pose.x
            block_pos_y = block_pose.y
            block_pos_z = block_pose.z
            
            block_position = Point(x = block_pos_x, y = block_pos_y, z = block_pos_z)
            block_orientation = Quaternion(*tf_conversions.transformations.quaternion_from_euler(0, 0, 0))
            block_pose = Pose(position=block_position, orientation = block_orientation)

            rospy.loginfo("Creating a block  with set location x: %f, y: %f, z: %f", block_pos_x, block_pos_y, block_pos_z)

        block_position_table = Point(block_position.x, block_position.y, -.14)
        # Create a marker to display in RViz
        curr_marker = bf.create_block_marker(frame = "world", id = len(self.marker_list.markers), position = block_position_table, orientation=block_orientation, block_type = "1x1", block_color = block_color, transparency = 0.3)

        self.marker_list.markers.append(curr_marker)

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

    def build_square_simple(self, dim, origin_pose):
        curr_pose = origin_pose
        block_dim = 0.033
        
        # Use only 1x1 blocks
        for i in range(dim):
            self.spawn_block(block_pose = curr_pose, block_reference = "world", block_dim = "1x1", block_color=None)
            curr_pose.x += block_dim
        for i in range(dim):
            self.spawn_block(block_pose = curr_pose, block_reference = "world", block_dim = "1x1", block_color=None)
            curr_pose.y -= block_dim
        for i in range(dim):
            self.spawn_block(block_pose = curr_pose, block_reference = "world", block_dim = "1x1", block_color=None)
            curr_pose.x -= block_dim
        for i in range(dim):
            self.spawn_block(block_pose = curr_pose, block_reference = "world", block_dim = "1x1", block_color=None)
            curr_pose.y += block_dim

        

    # Not yet fully implemented...
    """
    def build_wall(self, start_coord, direction, length):

        if(direction == "west" or direction=="east"):
            angle = math.pi/2
        else: 
            angle = 0

        curr_center = start_coord
        
        for i in range(length):
            blocks_left = length - i

            block_color = random.choice(COLORS_LIST)

            if(blocks_left == 1):
                block_type = random.choice("1x1", "1x2", "1x3")
                # Either use a 1x1 to fill last spot or start with a new wall with 1x2 or a 1x3
        
                if(block_type == "1x1"):
                    already_complete = 0
                    block_center, block_angle = calc_center(curr_center, direction, 1)
                    
                elif(block_type == "1x2"):
                    angle = angle + pi/2
                    already_complete =  1
                    block_center = calc_center(curr_center, angle, 1)

                elif(block_type == "1x3"):
                    angle = angle + pi/2
                    already_complete =  2
                    block_center = calc_center(curr_center, angle, 3)

                block_config.append([block_type, block_color, block_center, angle])

                return block_config, already_complete


            if(blocks_left == 2):
                block_type = random.choice("1x1", "1x2")
        
                # Either use a 1x1 to fill last spot
                if(block_type == "1x1"):
                    block_center = calc_center(curr_center, direction, 1)
                    block_config([block_type, block_color, block_center, angle])
                    continue
                    
                elif(block_type == "1x2"):
                    block_center = calc_center(curr_center, direction, 2)
                    block_config([block_type, block_color, block_center, angle])

 

            if(blocks_left == 3):
                 # Any block can be used here
                block_type = random.choice("1x1", "1x2", "1x3")

                block_config.append([block_type, block_color, block_center, angle])

    def build_structure(self, x_dim, y_dim):
        # Uses only 1x1 1x2 and 1x3 pieces
        end_coord = build_wall(start_coord = origin, direction=east, length = x_dim)
        end_coord = build_wall(start_coord = end_coord, direction=south, length = y_dim - already_complete)
        end_coord = build_wall(start_coord = end_coord, direction=west, length = x_dim - already_complete)
        end_coord = build_wall(start_coord = end_coord, direction=north, length = y_dim - already_complete - 1)

        assert(end_coord == origin)

    """

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
    rospy.init_node("block_spawner")

    # Load Gazebo Models via Spawning Services
    # Note that the models reference is the /world frame
    # and the IK operates with respect to the /base frame

    #spawn_table()

    # Remove models from the scene on shutdown
    #rospy.on_shutdown(delete_table)

    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    block_spawner = BlockSpawner()
    block_spawner.publish()

    # Spawn n number of bricks
    """
    n_bricks = 1
    for i in range(n_bricks):
        block_spawner.spawn_block(block_pose=None, block_reference = "world", block_dim = None, block_color = "blue")
    """

    # Spawn a square of blocks
    square_origin = Point()
    square_origin.x = 0.5
    square_origin.y = -0.2
    square_origin.z = 0.7825
    block_spawner.build_square_simple(1, square_origin)
    

    rate = rospy.Rate(10)

    rospy.on_shutdown(block_spawner.delete_blocks)
    while not rospy.is_shutdown():
        block_spawner.marker_pub.publish(block_spawner.marker_list)

        rate.sleep()
    return

if __name__ == '__main__':
     main()
