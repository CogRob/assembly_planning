#!/usr/bin/env python

import numpy as np
import rospy
from block_mover.msg import BlockObservation, BlockObservationArray
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

from matplotlib import pyplot as plt 

TOTAL_HASH_BITS = 24

COLOR_BITS = 4
COLOR_RED = 1
COLOR_GRN = 2
COLOR_BLU = 4

DIM_BITS = 8
DIM_1X1   = 1
DIM_1X2   = 2
DIM_1X3   = 4
DIM_1X4   = 8
DIM_2X2   = 16

HEIGHT_BITS = 4
HEIGHT_T  = 1
HEIGHT_S  = 2

BLOCK_BITS = 8

class Block():
    def __init__(self, color, dim, height, num):
        self.color = color
        self.dim = dim
        self.height = height
        self.num = 0

    def hash_block(self):
        block_hash = 0
        shift = 0

        if(self.color == "red"):
            block_hash = block_hash | (COLOR_RED << shift)
            
        elif(self.color == "grn"):
            block_hash = block_hash | (COLOR_GRN << shift)

        elif(self.color == "blu"):
            block_hash = block_hash | (COLOR_BLU << shift)

        print(hex(block_hash))


        shift += COLOR_BITS

        if(self.dim == "1X1"):
            block_hash = block_hash | (DIM_1X1 << shift)
            
        elif(self.dim == "1X2"):
            block_hash = block_hash | (DIM_1X2 << shift)

        elif(self.dim == "1X3"):
            block_hash = block_hash | (DIM_1X3 << shift)

        elif(self.dim == "1X4"):
            block_hash = block_hash | (DIM_1X4 << shift)

        elif(self.dim == "2X2"):
            block_hash = block_hash | (DIM_2X2 << shift)
        
        print(hex(block_hash))


        shift += DIM_BITS

        if(self.height == "T"):
            block_hash = block_hash | (HEIGHT_T << shift)

        elif(self.height == "S"):
            block_hash = block_hash | (HEIGHT_S << shift)

        print(hex(block_hash))

        shift += HEIGHT_BITS

        if (self.num <= 2**BLOCK_BITS - 1):
            block_hash = block_hash | (self.num << shift)
            
        print(hex(block_hash))

        return block_hash

    def __str__(self):
        return self.color + "_" + self.dim + "_" + self.height + "_" + str(self.num)

def unhash_block(block_hash):
    shift = 0

    if((block_hash & (COLOR_RED << shift)) >> shift == COLOR_RED):
        block_color = "red"
    elif((block_hash & (COLOR_GRN << shift)) >> shift == COLOR_GRN):
        block_color = "grn"
    elif((block_hash & (COLOR_BLU << shift)) >> shift == COLOR_BLU):
        block_color = "blu"

    shift += COLOR_BITS

    if((block_hash & (DIM_1X1 << shift)) >> shift == DIM_1X1):
        block_dim = "1X1"
    elif((block_hash & (DIM_1X2 << shift)) >> shift == DIM_1X2):
        block_dim = "1X2"
    elif((block_hash & (DIM_1X3 << shift)) >> shift == DIM_1X3):
        block_dim = "1X3"
    elif((block_hash & (DIM_1X4 << shift)) >> shift == DIM_1X4):
        block_dim = "1X4"
    elif((block_hash & (DIM_2X2 << shift)) >> shift == DIM_2X2):
        block_dim = "2X2"
    else:
        print("WHAT!")

    shift += DIM_BITS

    if((block_hash & (HEIGHT_T << shift)) >> shift == HEIGHT_T):
        block_height = "T"
    elif((block_hash & (HEIGHT_S << shift)) >> shift == HEIGHT_S):
        block_height = "S"

    shift += HEIGHT_BITS

    block_num = int(block_hash >> shift)

    return Block(color=block_color, dim=block_dim, height=block_height, num=block_num)

def color_int_to_str(color_int):
    if(color_int == COLOR_RED):
        return "red"
    elif(color_int == COLOR_GRN):
        return "grn"
    elif(color_int == COLOR_BLU):
        return "blu"
    else:
        return None


def dim_int_to_str(dim_int):
    if(dim_int == DIM_1X1):
        return "1X1"
    elif(dim_int == DIM_1X2):
        return "1X2"
    elif(dim_int == DIM_1X3):
        return "1X3"
    elif(dim_int == DIM_1X4):
        return "1X4"
    elif(dim_int == DIM_2X2):
        return "2X2"
    else:
        return None


block_list_1 = [
    Block(color="red", dim="1X1", height="T", num=0),
    Block(color="red", dim="1X2", height="T", num=0),
    Block(color="red", dim="1X3", height="T", num=0),
    Block(color="red", dim="2X2", height="T", num=0),
    Block(color="blu", dim="1X1", height="T", num=0),
    Block(color="blu", dim="1X2", height="T", num=0),
    Block(color="blu", dim="1X4", height="T", num=0),
    Block(color="grn", dim="1X2", height="T", num=0),
    Block(color="grn", dim="1X4", height="T", num=0),
]
map_resolution = 0.001

class StateEstimator():
    def __init__(self, table_diameter=1.2, map_resolution = map_resolution, block_list=block_list_1):
        self.map_resolution = map_resolution
        self.map_height = int(table_diameter / self.map_resolution)
        self.map_width  = int(table_diameter / self.map_resolution)

        rospy.loginfo("Spawning state estimator with dim: (w: %d, h: %d) with resolution %f m.", self.map_height, self.map_height, self.map_resolution)

        self.block_dict = {}

        self.occupancy_grid_right_hand = {}
        self.occupancy_grid_top = {}

        for block in block_list:
            block_hash = block.hash_block()
            self.occupancy_grid_right_hand[block_hash] = np.zeros((self.map_height, self.map_width))
            self.occupancy_grid_top[block_hash] = np.zeros((self.map_height, self.map_width))

            self.block_dict[block_hash] = (block.color, block.dim, block.height, block.num)

        self.occup_map = np.zeros((self.map_height, self.map_width, 4)) #[height, width, (B, G, R), orientation]

        self.display_map_right_hand = np.zeros((self.map_height, self.map_width, 3), dtype=np.uint8) #[height, width, (B, G, R), orientation]
        self.display_map_top = np.zeros((self.map_height, self.map_width, 3), dtype=np.uint8) #[height, width, (B, G, R), orientation]

        self.pub_rate = rospy.Rate(10)
        self.bridge = CvBridge()

    def subscribe(self):
        self.top_cam_obs_sub    = rospy.Subscriber("/block_finder/top/block_obs", BlockObservationArray, self.top_observation_callback)

        self.hand_cam_obs_sub   = rospy.Subscriber("/block_finder/right_hand/block_obs", BlockObservationArray, self.right_hand_observation_callback)

    def publish(self):
        self.display_map_top_pub = rospy.Publisher("/state_est/top/display_map", Image, queue_size=1)
        self.display_map_right_hand_pub = rospy.Publisher("/state_est/right_hand/display_map", Image, queue_size=1)

    def top_observation_callback(self, data):
        rospy.loginfo("Received %d observations", len(data.observations))

        self.update_occup_map(data.observations, "top")
    
    def right_hand_observation_callback(self, data):
        rospy.loginfo("Received %d observations", len(data.observations))

        self.update_occup_map(data.observations, "right_hand")

    def update_occup_map(self, observations, camera):
        for obs in observations:
            # We are only using 1 of each block and tall blocks, so height and num are defaulted
            new_block = Block(color=color_int_to_str(obs.color), dim=dim_int_to_str(obs.dim), num=0, height="T")

            block_hash = new_block.hash_block()

            pose_grid = pose_to_grid(obs.pose)

            if(camera == "top"):
                if(block_hash in self.occupancy_grid_top):
                    self.occupancy_grid_top[block_hash][pose_grid.x, pose_grid.y] += 1
                else:
                    print("Key not in dictionary!")
            elif(camera == "right_hand"):
                if(block_hash in self.occupancy_grid_right_hand):
                    self.occupancy_grid_right_hand[block_hash][pose_grid.x, pose_grid.y] += 1
                else:
                    print("Key not in dictionary!")
            else:
                print("That camera is not supported")
            
    def update_display_map(self, camera):
        if(camera == "top"):
            occupancy_grid = self.occupancy_grid_top
        elif(camera == "right_hand"):
            occupancy_grid = self.occupancy_grid_right_hand
        else:
            print("ERROR!!")
            
        display_map = np.zeros((self.map_width, self.map_height, 3), dtype=np.uint8)

        for key in occupancy_grid:
            
            # plt.subplot(3, 3, i)
            curr_occ = occupancy_grid[key]
            # Find max value of curr_occ
            max_index = np.unravel_index(np.argmax(curr_occ), curr_occ.shape)
            max_val = np.max(curr_occ)
            if(max_val > 1):

                color = self.block_dict[key][0] 
                block_type = self.block_dict[key][1] 

                if(color == "blu"):
                    color = (0, 0, 255)
                elif(color == "red"):
                    color = (255, 0, 0)
                elif(color == "grn"):
                    color = (0, 255, 0)

                font = cv2.FONT_HERSHEY_SIMPLEX
                font_size = 3
                font_thickness = 1

                cv2.putText(display_map, block_type,(max_index[0],max_index[1]), font, font_size, color, font_thickness)

                cv2.circle(display_map, (max_index[0], max_index[1]), 25, color, 3)
                #display_map[max_index[0]] = (255, 255, 255)

                #print("Max of", max_val," at ", max_index)
                #print(occupancy_grid[key])

        display_map = np.flip(display_map, 1)
        #display_map = np.flip(display_map, 0)

        if(camera == "top"):
            self.display_map_top = display_map
        elif(camera == "right_hand"):
            self.display_map_right_hand = display_map
        else:
            print("ERROR!!!")

        # Display map
        """
        plt.imshow(display_map)
        plt.show()
        """

def pose_to_grid(pose):
    grid_x = int(pose.x / map_resolution)
    grid_y = int((pose.y + .6) / map_resolution)

    print("Converted ", pose.x, pose.y," to ",  grid_x, grid_y)

    return Pose2D(x = grid_x, y = grid_y, theta = pose.theta)


def main():
    rospy.init_node('state_estimator')

    state_estimator = StateEstimator()

    state_estimator.subscribe()
    state_estimator.publish()
    
    for block in block_list_1:
        print(block)

        hashed_block = block.hash_block()

        print("Hashed block: ", hex(hashed_block))

        block = unhash_block(hashed_block)

        print(block)

    obs_count = 0
    while not rospy.is_shutdown():
        #rospy.loginfo("Publishing!")
        rospy.loginfo("%d", obs_count)

        # Sleep
        state_estimator.pub_rate.sleep()

        state_estimator.update_display_map("top")
        state_estimator.update_display_map("right_hand")
        state_estimator.display_map_top_pub.publish(state_estimator.bridge.cv2_to_imgmsg(state_estimator.display_map_top, 'rgb8'))
        state_estimator.display_map_right_hand_pub.publish(state_estimator.bridge.cv2_to_imgmsg(state_estimator.display_map_right_hand, 'rgb8'))

        obs_count += 1

    return 

if __name__ == '__main__':
     main()
