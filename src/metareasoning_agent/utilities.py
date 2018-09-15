"""
utility methods for manipulating ros variables
"""
import rospy
from geometry_msgs.msg import Point


def calculate_pose_diff(pose1, pose2):
    """
    Method to store the spatial distance between centroids of two blocks

    Input:
        pose1: type - geometry_msgs/Pose2D
        pose2: type - geometry_msgs/Pose2d
    Output:
        delta_pose: type - geometry_msgs/Point
    """
    pose_diff = Point()
    pose_diff.x = pose1.x - pose2.x
    pose_diff.y = pose1.y - pose2.y

    return pose_diff
