#!/usr/bin/env python
"""
Author: Priyam Parashar

Module for defining constraints which are inputs to the PrimitiveActions
"""
from __future__ import print_function
from enum import Enum
import tf


class ConstraintType(Enum):
    """Enum for constraining constraint types"""
    pose = 'Pose'
    motion = 'Motion'


class GripType(Enum):
    """Enum for constraining types of grips"""
    OVER = 'overhead'
    SIDE = 'side-aligned'


class PoseConstraint(object):
    """
    class for creating pose constraints
    Includes fields:
        pose:   geometry_msgs/Pose
        frame:  string
    """

    def __init__(self, pose=None, frame=None):
        # check for bad value assignment
        if pose is None or frame is None:
            raise ValueError(
                "Object of type PoseConstraint needs values of both pose and \
                frame to initialize\nUsage: \
                PoseConstraint(pose=geometry_msgs/Pose,frame=string)")
        if not self._is_valid_frame(frame):
            raise ValueError(
                "Creation of PoseConstraint requires a valid tf frame")
        self.pose = pose
        self.frame = frame

    def __str__(self):
        print("PoseType Constraint with Pose: %s, wrt Frame: %s", str(
            self.pose), self.frame)

    def _is_valid_frame(self, frame):
        transformer = tf.Transformer()
        frames = transformer.getFrameStrings()
        if frame in frames:
            return True
        return False

    def get_type(self):
        return ConstraintType.pose


class MotionConstraint(object):
    """
    Constraint of type Motion.
    Consists of fields:
        grip_type:      Enum GripType
        grip_offset:    position for gripping wrt center of the object
    """

    def __init__(self, grip_type=None, grip_offset=0):
        # check for invalid valur assignment
        if grip_type is None:
            raise ValueError(
                "Creation of MotionConstraint requires a valid grip_type \
                input. Usage: MotionConstraint(grip_type=GripType.OVER/SIDE, \
                grip_offset=double")
        self.grip_type = None
        self.grip_offset = None

    def __str__(self):
        print("MotionType Constraint with GripType: %s, GripOffset: %s",
              str(self.grip_type), str(self.grip_offset))

    def get_type(self):
        return ConstraintType.motion


class Constraint(object):
    """
    Class for constraints which link world information with primitive actions
    Consists of fields:
        constraints:    list of constraints to be applied to the action
    """

    def __init__(self, constraint=None):
        self.constraints = []

    def __str__(self):
        for constraint in self.constraints:
            print(str(constraint))
