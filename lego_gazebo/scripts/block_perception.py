#!/usr/bin/env python
"""
Author: Priyam Parashar
Node to perceive blocks in Gazebo world and publish a nice topic-based
interface for the agent to read all the info
"""
import sys

import rospy
import gazebo_msgs.msg as gazebo_msg
import geometry_msgs.msg as geo_msg
import block_detector.msgs as block_msg

from metareasoning.knowledge_base import LegoLoc


class BlockPerception(object):
    """Reads model states from Gazebo and publishes topic of type TODO"""

    def __init__(self):
        super(BlockPerception, self).__init__()
        self.blocks = []  # list of blocks read in from /gazebo/model_states
        self.block_poses = []  # list of poses wrt base_link
        self.base_link = None

        # subcriber to /gazebo/model_states
        self.gazebo_model_sub = rospy.Subscriber(
            '/gazebo/model_states', gazebo_msg.ModelStates, self.update_blocks)

        # publisher to /gazebo_agent/blocks
        self.blocks_pub = rospy.Publisher(
            'blocks', block_msg.BlockObservationArray, queue_size=5)

        # define ws and inv separation
        self.x_lim = 0.435

    def check_pose_loc(self, block_pose):
        """
        Checks if block_pose belongs to ws or inv
        """
        if block_pose.position.x < self.x_lim:
            return LegoLoc.WS
        else:
            return LegoLoc.INV

    def update_blocks(self, data):
        """
        Update self.blocks and self.block_poses with new ModelStates data
        """
        if self.base_link is None:
            # one-time update of base_link
            self.update_base_link(data)
        for i, name in enumerate(data.name):
            # if the current model is a block
            list_idx = None
            try:
                list_idx = self.blocks.index(name)
                rospy.logdebug("Block %s found. Updating pose.", name)
                self.block_poses[list_idx] = data.pose[i]
            except ValueError as val_err:
                rospy.logdebug(val_err + ". Adding.")
                self.blocks.append(name)
                self.block_poses.append(data.pose[i])

    def update_base_link(self, data):
        """
        Update position and orientation of base_link for transforming all
        model states wrt it
        """
        self.base_link = geo_msg.Pose()
        for i, name in enumerate(data.name):
            if name == 'baxter':
                # we do not care about speeds right now, only pos, ori
                self.base_link.position.x = round(data.pose[i].position.x, 3)
                self.base_link.position.y = round(data.pose[i].position.y, 3)
                self.base_link.position.z = round(data.pose[i].position.z, 3)
                self.base_link.orientation = data.pose[i].orientation

    def publish_blocks(self):
        """
        Format observed blocks into ws and inv, and publish on the topic
        """
        block_obs = block_msg.BlockObservationArray()
        for i, block in self.blocks:
            # parse block-name for width, length and color
            block_props = block.split('_')[2:]
            # format current block into block_detector/BlockObservation type
            curr_block = block_msg.BlockObservation()
            curr_block.pose = self.block_poses[i]
            block_obs.width = block_props[0]
            block_obs.length = block_props[1]
            block_obs.color = block_props[2]
            block_loc = self.check_pose_loc(self.block_poses[i])
            if block_loc == LegoLoc.WS:
                # add the block to ws_obs
                block_obs.ws_obs.append(curr_block)
            else:
                # add the block to inv_obs
                block_obs.inv_obs.append(curr_block)
        self.blocks_pub(block_obs)


def main():
    rospy.init_node('block_state_publisher', log_level=rospy.DEBUG)
    rate = rospy.Rate(10)
    block_publisher = BlockPerception()
    while not rospy.is_shutdown():
        block_publisher.publish_blocks()
        rate.sleep()


if __name__ == '__main__':
    sys.exit(main())
