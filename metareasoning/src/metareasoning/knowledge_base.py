#!/usr/bin/env python
"""
Central repository of data structures and custom containers for
defining the Lego World
"""
from enum import Enum
import networkx as nx
from geometry_msgs.msg import Pose, Pose2D
import rospy
from metareasoning.utilities import calculate_pose_diff


def node2block(env_state_node):
    """
    Helper function which converts EnvState's ws-graph node to Block type
    """

    return Block(
        env_state_node['length'],
        env_state_node['width'],
        env_state_node['color'],
        pose=Pose2D(
            x=env_state_node['pose_x'],
            y=env_state_node['pose_y'],
            theta=env_state_node['pose_theta']))


# Enum for fixed primitive actions
class PrimitiveActions(Enum):
    """Object for communicating planned actions to the agent"""
    pick = 'pick'
    place = 'place'
    transport = 'transport'
    align = 'align'
    retract = 'retract'
    detect = 'detect'


class Block(object):
    """Object for defining blocks in the environment"""

    def __init__(self, length=0, width=0, color='none', pose=Pose()):
        self.length = length
        self.width = width
        self.color = color
        self.pose = pose

    def __eq__(self, other):
        if isinstance(other, Block):
            return self.length == other.length and self.width == other.width and self.color == other.color
        else:
            raise NotImplementedError

    def __str__(self):
        return self.color + "_" + str(self.width) + "x" + str(
            self.length) + str(self.pose)


class Constraints(object):
    def __init__(self, block=None, position=None, orientation=None, grip=None):
        self.position = position
        self.block = block
        self.orientation = orientation
        self.grip = grip

    def is_Block(self):
        """checking for kind of constraint"""
        if self.block is not None:
            return True
        return False


class EnvState(object):
    """List of blocks in inventory + graph describing the blocks in workspace

    Workspace Graph:
        Node - Blocks placed in the workspace
        Edge - Spatial relationships between the blocks as they are placed

    Inventory:
        {bId: [bType, bPose]}:  Dictionary of block IDs with corresponding
                                block type and that block's geometry_msgs/Pose
    """

    def __init__(self):
        self._block_cnt = 0
        self.ws_state = nx.Graph()
        self.inv_state = []

    def add_block(self, block):
        """Method to add a new block as a node to the EnvState graph"""
        # Check if the block is already in graph
        if (self.in_graph(block)):
            return
        else:
            rospy.loginfo("Adding block %s to ws graph", block)
            self.ws_state.add_node(
                self._block_cnt,
                length=block.length,
                width=block.width,
                color=block.color,
                pose_x=block.pose.x,
                pose_y=block.pose.y,
                pose_theta=block.pose.theta)

            self._block_cnt += 1

            # Check that the number of nodes has increased
            if (self.ws_state.number_of_nodes() != self._block_cnt):
                rospy.logerr("The node wasn't properly added. WTF?!?!")
            self._update_edges()

    def clear(self):
        self.inv_state = []
        self.ws_state.clear()
        self._block_cnt = 0

    def in_graph(self, block):
        # Checks if a block is already in the graph
        for node_key in self.ws_state.nodes:
            node = self.ws_state.nodes[node_key]
            node_block = node2block(node)

            if (node_block == block):
                rospy.loginfo("%s is already in the graph!", block)
                return True

        rospy.loginfo("%s is not in the graph!", block)
        return False

    def _update_edges(self):
        """Method to update edges to the latest block added"""
        base_node = self.ws_state.nodes[self._block_cnt - 1]

        base_node_pose = Pose2D(
            x=base_node['pose_x'],
            y=base_node['pose_y'],
            theta=base_node['pose_theta'])
        for idx in range(0, self._block_cnt - 1):
            target_node_pose = Pose2D(
                x=self.ws_state.nodes[idx]['pose_x'],
                y=self.ws_state.nodes[idx]['pose_y'],
                theta=self.ws_state.nodes[idx]['pose_theta'])
            pose_diff = calculate_pose_diff(base_node_pose, target_node_pose)
            self.ws_state.add_edge(
                self._block_cnt - 1,
                idx,
                del_x=pose_diff.x,
                del_y=pose_diff.y,
                del_theta=pose_diff.theta)

    def print_graph(self):
        print("Nodes:")
        for node_key in self.ws_state.nodes:
            node = self.ws_state.nodes[node_key]
            print(node)
        print("edges")
        for edge_key in self.ws_state.edges:
            print self.ws_state.edges[edge_key]


class Task(object):
    """Structure definition for a task object"""

    def __init__(self, head):
        self.name = head
        self.goal = EnvState()
