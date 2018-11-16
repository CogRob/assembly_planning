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


# Enum for methods
class NonPrimitiveActions(Enum):
    acquire = 'ACQUIRE'
    deposit = 'DEPOSIT'


# Enum for fixed primitive actions
class PrimitiveActions(Enum):
    """Object for communicating planned actions to the agent"""
    pick = 'pick'
    place = 'place'
    transport = 'transport'
    align = 'align'
    retract = 'retract'
    detect = 'detect'


# Enum for locations/stations
class LegoLoc(Enum):
    WS = 'workspace'
    INV = 'inventory'


class BlockType(object):
    """Object for describing the physical aspects of a block"""

    def __init__(self, width=0, length=0, color='none'):
        if length == 0 or width == 0:
            raise ValueError(
                "Creation of a block object requires non-zero length and width"
            )
        self.length = length
        self.width = width
        self.color = color

    def __eq__(self, other):
        if isinstance(other, BlockType):
            return self.length == other.length and \
                    self.width == other.width and self.color == other.color
        else:
            raise NotImplementedError("Cannot compare BlockType with %s",
                                      type(other))

    def __str__(self):
        return str(self.width) + "x" + str(self.length) + "_" + str(self.color)


class Block(object):
    """Object for defining blocks grounded in the environment"""

    def __init__(self, width=0, length=0, color='none', pose=Pose()):
        self.type = BlockType(width, length, color)
        self.pose = pose

    def __eq__(self, other):
        if isinstance(other, Block):
            return self.type == other.type and self.pose == other.pose
        else:
            raise NotImplementedError("Cannot compare Block with %s",
                                      type(other))

    def __str__(self):
        return str(self.type) + " at " + str(self.pose)


class FeatureType(Enum):
    """Enum for constraining types of features to be used with _detect"""
    block = 'Block'
    pose = 'Pose'


class Feature(object):
    """
    Features to be provided to _detect routine for interfacing with sensors
    """

    def __init__(self, feat_type, feature):
        if not isinstance(feature, Block) and not isinstance(feature, Pose):
            raise TypeError("Feature needs to be one of the following types: "
                            + [feat.value for feat in FeatureType])
        self.type = feat_type
        self.feature = feature


class EnvState(object):
    """State of the environment as known to the agent

    List of blocks in inventory + graph describing the blocks in workspace
    Inventory:
        {bId: [block]}:  Dictionary of block IDs with corresponding
                                block type and that block's geometry_msgs/Pose
    Workspace Graph:
        Node - Blocks placed in the workspace
        Edge - Delta-x, delta-y, delta-theta between all the nodes. The
               relationships are of absolute float type.
    """

    def __init__(self):
        self._block_cnt = 0
        self.workspace = nx.Graph()
        self.inventory = []

    def add_block(self, block):
        """Method to add a new block as a node to the EnvState graph"""
        # Check if the block is already in graph
        if (self.in_graph(block)):
            return
        else:
            rospy.logdebug("Adding block %s to ws graph", str(block))
            self.workspace.add_node(
                self._block_cnt,
                width=block.type.width,
                length=block.type.length,
                color=block.type.color,
                pose_x=block.pose.x,
                pose_y=block.pose.y,
                pose_yaw=block.pose.theta)

            self._block_cnt += 1

            # Check that the number of nodes has increased
            if (self.workspace.number_of_nodes() != self._block_cnt):
                rospy.logerr("The node wasn't properly added. WTF?!?!")
            self._update_edges()

    def clear(self):
        self.inventory = []
        self.workspace.clear()
        self._block_cnt = 0

    def in_graph(self, block):
        # Checks if a block is already in the graph
        for node_key in self.workspace.nodes:
            node = self.workspace.nodes[node_key]
            node_block = node2block(node)

            if (node_block == block):
                rospy.logdebug("%s is already in the graph!", block)
                return True

        rospy.logdebug("%s is not in the graph!", block)
        return False

    def _update_edges(self):
        """Method to update edges to the latest block added"""
        base_node = self.workspace.nodes[self._block_cnt - 1]

        base_node_pose = Pose2D(
            x=base_node['pose_x'],
            y=base_node['pose_y'],
            theta=base_node['pose_yaw'])
        for idx in range(0, self._block_cnt - 1):
            target_node_pose = Pose2D(
                x=self.workspace.nodes[idx]['pose_x'],
                y=self.workspace.nodes[idx]['pose_y'],
                theta=self.workspace.nodes[idx]['pose_yaw'])
            pose_diff = calculate_pose_diff(base_node_pose, target_node_pose)
            self.workspace.add_edge(
                self._block_cnt - 1,
                idx,
                del_x=abs(pose_diff.x),
                del_y=abs(pose_diff.y),
                del_yaw=abs(pose_diff.theta))

    def print_graph(self):
        print("Nodes:")
        for node_key in self.workspace.nodes:
            node = self.workspace.nodes[node_key]
            print(node)
        print("edges")
        for edge_key in self.workspace.edges:
            print self.workspace.edges[edge_key]


class Task(object):
    """Structure definition for a task object"""

    def __init__(self, head):
        self.name = head
        self.goal = EnvState()
