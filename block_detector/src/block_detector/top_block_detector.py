#!/usr/bin/env python

from block_detector_base import BlockDetector, line_plane_intersection, \
    create_block_marker

import numpy as np

from block_detector.msg import BlockObservation, BlockObservationArray

# ROS imports
import rospy
import tf

# ROS messages
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Quaternion, PointStamped, Pose2D

from image_geometry import PinholeCameraModel


top_cam_hsv_dict = {
    "blue": {
        "low_h": 98,
        "high_h": 118,
        "low_s": 154,
        "high_s": 255,
        "low_v": 0,
        "high_v": 255,
        "color_val": (255, 0, 0),
    },

    "green": {
        "low_h": 41,  # 31
        "high_h": 104,  # 77
        "low_s": 94,  # 89
        "high_s": 255,
        "low_v": 13,  # 71
        "high_v": 212,
        "color_val": (0, 255, 0),
    },

    "red": {
        "low_h": [0, 161],
        "high_h": [11, 180],  # 0
        "low_s": 96,
        "high_s": 255,
        "low_v": 47,
        "high_v": 255,
        "color_val": (0, 0, 255),
    }
}


class TopBlockDetector(BlockDetector):
    def __init__(self, pub_rate):
        BlockDetector.__init__(self, resolution=(640, 480),
                               allowed_circle_center=(315, 255),
                               allowed_circle_diameter=190,
                               allowed_circle_thickness=220,
                               pub_rate=pub_rate)

        self.camera = "top"

        self.blob_area_min_thresh = 40
        self.blob_area_max_thresh = 1200

        # Constant
        self.table_dist = 1.33

        # Camera reference frame origin
        self.ray_point = np.array([0, 0, 0])

        self.table_to_base_dist = -0.1
        self.store_segmented_images = True

        self.font_size = 0.5
        self.font_thickness = 1

        # The maximum pixel dimensions of block "units"
        self.one_unit_max = 22
        self.two_unit_max = 34
        self.three_unit_max = 48
        self.four_unit_max = 60

        self.morph_opening_kernel = (1, 1)
        self.morph_closing_kernel = (2, 2)

        self.block_obs_array = BlockObservationArray()

        # Divide workspace and inventory using the vertical line that splits
        # the image
        self.ws_inv_divider = self.image_width / 2

        # Get TF from top camera to base
        try:
            self.tf_listener = tf.TransformListener()
            time = rospy.Time(0)

            self.tf_listener.waitForTransform(
                "/base", "/camera_rgb_optical_frame", time, rospy.Duration(4))

            (self.trans, self.rot) = self.tf_listener.lookupTransform(
                "/base", "/camera_rgb_optical_frame", time)

        except (tf.LookupException, tf.ConnectivityException):
            rospy.loginfo("No transform from base to camera available!")

        # From base reference frame table normal and point on table
        table_plane_normal_base = PointStamped()
        table_plane_normal_base.header.frame_id = "base"

        table_plane_normal_base.header.stamp = rospy.Time(0)
        table_plane_normal_base.point.x = 0
        table_plane_normal_base.point.y = 0
        table_plane_normal_base.point.z = -1000.

        table_plane_point_base = PointStamped()
        table_plane_point_base.header.frame_id = "base"
        table_plane_point_base.header.stamp = rospy.Time(0)
        table_plane_point_base.point.x = 0
        table_plane_point_base.point.y = 0
        table_plane_point_base.point.z = self.table_to_base_dist

        # Covert to camera reference frame
        table_plane_normal_cam_p = self.tf_listener.transformPoint(
            "camera_rgb_optical_frame", table_plane_normal_base)

        table_plane_point_cam_p = self.tf_listener.transformPoint(
            "camera_rgb_optical_frame", table_plane_point_base)

        # Convert to arrays
        self.table_plane_normal_cam = \
            np.array([table_plane_normal_cam_p.point.x,
                      table_plane_normal_cam_p.point.y,
                      table_plane_normal_cam_p.point.z
                      ])

        self.table_plane_point_cam = \
            np.array([table_plane_point_cam_p.point.x,
                      table_plane_point_cam_p.point.y,
                      table_plane_point_cam_p.point.z
                      ])

        self.enable_rviz_markers = True

        self.hsv_dict = top_cam_hsv_dict

    def create_debugging_publisher(self):
        BlockDetector.create_debugging_publisher(self)

    def create_block_obs_publisher(self):
        self.block_obs_pub = rospy.Publisher(
            "block_detector/top/block_obs", BlockObservationArray,
            queue_size=10)

    def publish_block_obs(self):
        self.block_obs_pub.publish(self.block_obs_array)

    def subscribe(self):
        self.cam_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image,
                                        self.cam_callback)
        self.cam_info_sub = rospy.Subscriber(
            "/camera/rgb/camera_info", CameraInfo, self.cam_info_callback)

    def cam_callback(self, data):
        BlockDetector.cam_callback(self, data)

        self.generate_block_obs()

    def cam_info_callback(self, data):
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)

        # Unsubscribe after receiving CameraInfo first time
        self.cam_info_sub.unregister()

    def in_workspace(self, block_center):
        if(block_center[0] < self.ws_inv_divider):
            return True
        else:
            return False

    def get_block_type(self, block_length, block_width):
        if(block_length < self.one_unit_max):
            length = 1
        elif(block_length < self.two_unit_max):
            length = 2
        elif(block_length < self.three_unit_max):
            length = 3
        elif(block_length < self.four_unit_max):
            length = 4
        else:
            length = 0

        if(block_width < self.one_unit_max):
            width = 1
        elif(block_width < self.two_unit_max):
            width = 2
        else:
            width = 0

        return (length, width)

    def generate_block_obs(self):
        inv_block_obs_list = []
        ws_block_obs_list = []

        # Reset block markers list
        self.block_markers.markers = []

        for detected_block in self.detected_blocks:
            block_center = detected_block["block_center"]
            block_angle = detected_block["block_angle"]
            block_length = detected_block["block_length"]
            block_width = detected_block["block_width"]
            block_color = detected_block["color"]

            # Project pixel coordinates of detected block to a ray from
            # camera link (in camera's reference frame)
            block_ray_vector = np.array(
                self.camera_model.projectPixelTo3dRay(block_center))

            block_position_cam_arr = line_plane_intersection(
                self.table_plane_normal_cam, self.table_plane_point_cam,
                block_ray_vector, self.ray_point)

            block_position_cam = PointStamped()
            block_position_cam.header.frame_id = "camera_rgb_optical_frame"
            block_position_cam.header.stamp = rospy.Time(0)
            block_position_cam.point.x = block_position_cam_arr[0]
            block_position_cam.point.y = block_position_cam_arr[1]
            block_position_cam.point.z = block_position_cam_arr[2]

            block_position_base = self.tf_listener.transformPoint(
                "base", block_position_cam)

            # block_position_base.point.z = self.table_to_base_dist
            # rospy.loginfo("Block angle: %f", math.degrees(block_angle))
            # TODO: Check if block_angle is pitch, yaw or roll
            block_orientation_cam = tf.transformations.quaternion_from_euler(
                0, 0, block_angle)
            # Convert block_angle rotation to base reference frame
            block_orientation_base = tf.transformations.quaternion_multiply(
                # block_orientation_cam, self.rot)
                self.rot, block_orientation_cam)

            block_orientation_base = Quaternion(*block_orientation_base)
            block_orientation_base_yaw = \
                tf.transformations.euler_from_quaternion(
                    np.array([
                        block_orientation_base.x,
                        block_orientation_base.y,
                        block_orientation_base.z,
                        block_orientation_base.w,
                    ]))[2]

            # rospy.loginfo("Block angle yaw: %f", math.degrees(
            #    block_orientation_base_yaw))

            block_obs = BlockObservation(
                pose=Pose2D(x=block_position_base.point.x,
                            y=block_position_base.point.y,
                            theta=block_orientation_base_yaw),
                length=block_length,
                width=block_width,
                color=block_color)

            if(self.in_workspace(block_center)):
                ws_block_obs_list.append(block_obs)
            else:
                inv_block_obs_list.append(block_obs)

            if(self.enable_rviz_markers):
                self.block_markers.markers.append(
                    create_block_marker(frame="base",
                                        id=len(
                                            self.block_markers.markers),
                                        position=block_position_base.point,
                                        orientation=block_orientation_base,
                                        length=block_length, width=block_width,
                                        block_color=block_color,
                                        transparency=1)
                )

        self.block_obs_array.inv_obs = inv_block_obs_list
        self.block_obs_array.ws_obs = ws_block_obs_list


# Testing for TopBlockDetector
def main():
    rospy.init_node("top_block_detector")

    top_block_detector = TopBlockDetector(pub_rate=10)

    top_block_detector.subscribe()
    top_block_detector.create_block_obs_publisher()
    top_block_detector.create_debugging_publisher()

    while (not rospy.is_shutdown()):
        # For debugging
        top_block_detector.publish_debugging()
        top_block_detector.publish_markers()
        top_block_detector.publish_block_obs()

        top_block_detector.pub_rate.sleep()


if __name__ == "__main__":
    main()
