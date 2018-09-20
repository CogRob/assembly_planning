#!/usr/bin/env python

from block_detector import BlockDetector, line_plane_intersection, create_block_marker
import numpy as np

from block_mover.msg import BlockObservation, BlockObservationArray, \
    BlockPixelLoc, BlockPixelLocArray

# ROS imports
import rospy
import tf
from cv_bridge import CvBridge

# ROS messages
from sensor_msgs.msg import Image, CameraInfo, Range
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Quaternion, PointStamped, Pose2D
import tf2_ros
import tf2_geometry_msgs

from image_geometry import PinholeCameraModel


from matplotlib import pyplot as plt  # For plotting


class HandBlockDetector(BlockDetector):
    def __init__(self, pub_rate):
        BlockDetector.__init__(self, resolution=(1280, 800),
                               allowed_circle_center=(640, 400),
                               allowed_circle_diameter=450,
                               allowed_circle_thickness=255,
                               pub_rate=pub_rate)
        self.camera = "right_hand"

        self.font_size = 3.0
        self.font_thickness = 2

        self.enable_rviz_markers = False
        self.block_pixel_locs = BlockPixelLocArray()
        self.pub_rate = rospy.Rate(pub_rate)  # in Hz

    def create_block_pixel_locs_publisher(self):
        self.block_pixel_locs_pub = rospy.Publisher(
            "block_detector/hand/block_pixel_locs", BlockPixelLocArray,
            queue_size=10)
        pass

    def create_debugging_publisher(self):
        BlockDetector.create_debugging_publisher(self)

    def publish_block_pixel_locs(self):
        self.block_pixel_locs_pub.publish(self.block_pixel_locs)

    def subscribe(self):
        self.cam_sub = rospy.Subscriber("/cameras/right_hand_camera/image",
                                        Image, self.cam_callback)
        self.ir_sub = rospy.Subscriber("/robot/range/right_hand_range/state",
                                       Range, self.ir_callback)

    def cam_callback(self, data):
        BlockDetector.cam_callback(self, data)

        self.generate_pixel_locs()

    def ir_callback(self, data):
        self.ir_reading = data.range

        # 0.65 meters is the upper limit of the IR sensor on Baxter
        if (self.ir_reading > 0.65):
            self.ir_reading = 0.4

        self.blob_area_min_thresh = 400 / self.ir_reading  # TODO: Tune
        self.blob_area_max_thresh = 100000  # TODO: Tune

        self.camera_height = self.ir_reading

    # TODO: Make this based upon length and width instead of ratio like
    # TopBlockDetector, but need to tune the length and width when the
    # camera is at hover height first
    def get_block_type(self, block_length, block_width):
        block_ratio = block_length / block_width

        if(block_ratio <= 0.4):
            rospy.loginfo(
                "Block ratio is very small so it's probably not a block..")
        if(block_ratio > 0.5 and block_ratio <= 1.3):
            length = 1
            width = 1

        elif(block_ratio > 1.3 and block_ratio <= 2.3):
            length = 2
            width = 1

        elif(block_ratio > 2.3 and block_ratio <= 3.1):
            length = 3
            width = 1

        elif(block_ratio > 3.1 and block_ratio <= 5.3):
            length = 4
            width = 1

        return (length, width)

    def generate_pixel_locs(self):
        block_pixel_locs_list = []

        for detected_block in self.detected_blocks:
            block_center = detected_block["block_center"]
            block_angle = detected_block["block_angle"]
            block_length = detected_block["block_length"]
            block_width = detected_block["block_width"]
            block_color = detected_block["color"]

            # TODO: double check the x and y here!
            block_pixel_loc = BlockPixelLoc(
                x=block_center[0],
                y=block_center[1],
                theta=block_angle,
                length=block_length,
                width=block_width,
                color=block_color
            )

            block_pixel_locs_list.append(block_pixel_loc)

        self.block_pixel_locs.pixel_locs = block_pixel_locs_list

# Testing for HandBlockDetector


def main():
    rospy.init_node("hand_block_detector")

    hand_block_detector = HandBlockDetector(pub_rate=10)

    hand_block_detector.subscribe()
    hand_block_detector.create_block_pixel_locs_publisher()
    hand_block_detector.create_debugging_publisher()

    while (not rospy.is_shutdown()):
        # For debugging
        hand_block_detector.publish_debugging()
        # hand_block_detector.publish_markers()
        hand_block_detector.publish_block_pixel_locs()

        hand_block_detector.pub_rate.sleep()


if __name__ == "__main__":
    main()
