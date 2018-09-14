#!/usr/bin/env python
"""
Central repository of data structures and custom containers for
defining the Lego World
"""
from enum import Enum
from geometry_msgs.msg import Pose


# Enum for fixed primitive actions
class PrimitiveActions(Enum):
    """Object for communicating planned actions to the agent"""
    pick = 'pick'
    place = 'place'
    transport = 'transport'
    align = 'align'
    retract = 'retract'


class Block(object):
    """Object for defining blocks in the environment"""

    def __init__(self, length, width, color='none', pose=Pose()):
        self.length = length
        self.width = width
        self.color = color
        self.pose = pose

    def __eq__(self, other):
        if isinstance(other, Block):
            return self.length == other.length and self.width == other.width
        else:
            raise NotImplementedError

    def __str__(self):
        return self.color + "_" + str(self.width) + "x" + str(
            self.length) + str(self.pose)
