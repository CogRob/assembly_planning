#!/usr/bin/env python

import numpy as np
import rospy
from block_mover.msg import BlockObservation, BlockObservationArray


from matplotlib import pyplot as plt 

TOTAL_HASH_BITS = 16

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

BLOCK_BITS = 4

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

class StateEstimator():
    def __init__(self, table_diameter=1., map_resolution = 0.001, block_list=block_list_1):
        self.map_resolution = map_resolution
        self.map_height = int(table_diameter / self.map_resolution)
        self.map_width  = int(table_diameter / self.map_resolution)

        rospy.loginfo("Spawning state estimator with dim: (w: %d, h: %d) with resolution %f m.", self.map_height, self.map_height, self.map_resolution)

        self.occupancy_grid = {}

        for block in block_list:
            self.occupancy_grid[block.hash_block()] = np.zeros((self.map_height, self.map_width))

        self.occup_map = np.zeros((self.map_height, self.map_width, 4)) #[height, width, (B, G, R), orientation]

        self.pub_rate = rospy.Rate(10)

    def subscribe(self):
        self.top_cam_obs_sub    = rospy.Subscriber("/block_finder/top_cam/block_obs", BlockObservationArray, self.top_cam_observation_callback)

        #self.hand_cam_obs_sub   = rospy.Subscriber("/block_finder/hand_cam/block_obs", BlockObservationArray, self.hand_cam_observation_callback)

    def publish(self):
        pass

    def top_cam_observation_callback(self, data):
        rospy.loginfo("Received an observation!")

        self.update_occup_map(data.observations)

    def update_occup_map(self, observations):

        for obs in observations:
            # We are only using 1 of each block and tall blocks, so height and num are defaulted
            new_block = Block(color=obs.color, dim=obs.dim, num=0, height="T")

            block_hash = new_block.hash_block

            self.occupancy_grid[block_hash][obs.pose.x, obs.pose.y] += 1

    def show_occupancy_map(self):
        i = 1
        for key in self.occupancy_grid:
            plt.subplot(3, 3, i)
            plt.imshow(self.occupancy_grid[key])
            i += 1

        plt.show()

def main():
    rospy.init_node('state_estimator')

    state_estimator = StateEstimator()

    state_estimator.subscribe()
    state_estimator.publish()
    
    for block in block_list_1:
        print(block)

        hashed_block = block.hash_block()

        print("Hashed block: ", hashed_block)

        block = unhash_block(hashed_block)

        print(block)

    obs_count = 0
    while not rospy.is_shutdown():
        #rospy.loginfo("Publishing!")
        rospy.loginfo("%d", obs_count)

        # Sleep
        state_estimator.pub_rate.sleep()
        if(obs_count == 50):
            state_estimator.show_occupancy_map()

        obs_count += 1

    return 

if __name__ == '__main__':
     main()
