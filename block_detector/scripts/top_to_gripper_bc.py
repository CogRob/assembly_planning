#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import sys
import copy
import rospkg
import math
import tf


class TopToBaseBC():
    def __init__(self):
        orig_x = +1.21
        orig_y = -.035
        orig_z = .64
        yaw = 0
        pitch = 90
        roll = 180
        self.top_to_base_mat = np.zeros((4, 4))

        self.top_to_base_mat[0, 3] = orig_x
        self.top_to_base_mat[1, 3] = orig_y
        self.top_to_base_mat[2, 3] = orig_z

        R = tf.transformations.euler_matrix(math.radians(
            yaw), math.radians(pitch), math.radians(roll))

        self.top_to_base_mat = self.top_to_base_mat + R

        top_to_base_p_x = self.top_to_base_mat[0, 3]
        top_to_base_p_y = self.top_to_base_mat[1, 3]
        top_to_base_p_z = self.top_to_base_mat[2, 3]

        self.p_tuple = (top_to_base_p_x, top_to_base_p_y, top_to_base_p_z)

        self.q = tf.transformations.quaternion_from_matrix(
            self.top_to_base_mat)

        self.top_cam_to_base_tf_br = tf.TransformBroadcaster()


def main():
    rospy.init_node('top_to_table_bc')
    top_to_base_bc = TopToBaseBC()

    while not rospy.is_shutdown():
        top_to_base_bc.top_cam_to_base_tf_br.sendTransform(top_to_base_bc.p_tuple,
                                                           top_to_base_bc.q,
                                                           rospy.Time.now(),
                                                           "base",
                                                           "camera_link"

                                                           )

        rospy.loginfo("Sending out camera to base transform!")
        # Sleep
        rospy.sleep(0.01)

    return


if __name__ == '__main__':
    main()
