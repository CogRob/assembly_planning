"""
utility methods for manipulating ros variables
"""
import rospy
from geometry_msgs.msg import Point


def print_pose(pose_msg):
    """
    Utility function for printing geometry_msgs/Pose msgs
    """
    rospy.loginfo("x = %s", pose_msg.position.x)
    rospy.loginfo("y = %s", pose_msg.position.y)
    rospy.loginfo("z = %s", pose_msg.position.z)
    rospy.loginfo("qx = %s", pose_msg.orientation.x)
    rospy.loginfo("qy = %s", pose_msg.orientation.y)
    rospy.loginfo("qz = %s", pose_msg.orientation.z)
    rospy.loginfo("qw = %s", pose_msg.orientation.w)


def calculate_pose_diff(pose1, pose2):
    """
    Method to store the spatial distance between centroids of two blocks

    Input:
        pose1: type - geometry_msgs/Pose
        pose2: type - geometry_msgs/Pose
    Output:
        delta_pose: type - geometry_msgs/Point
    """
    pose_diff = Point()
    pose_diff.x = pose1.position.x - pose2.position.x
    pose_diff.y = pose1.position.y - pose2.position.y
    return pose_diff
