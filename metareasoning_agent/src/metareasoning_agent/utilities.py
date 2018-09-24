"""
utility methods for manipulating ros variables
"""
import rospy
from geometry_msgs.msg import Pose2D
import math


def calculate_pose_diff(pose1, pose2):
    """
    Method to store the spatial distance between centroids of two blocks

    Input:
        pose1: type - geometry_msgs/Pose2D
        pose2: type - geometry_msgs/Pose2d
    Output:
        delta_pose: type - geometry_msgs/Point
    """
    pose_diff = Pose2D()
    pose_diff.x = math.fabs(pose1.x - pose2.x)
    pose_diff.y = math.fabs(pose1.y - pose2.y)
    pose_diff.theta = math.fabs(pose1.theta - pose2.theta)

    return pose_diff
