#!/usr/bin/env python
import numpy as np  # For matrix operations

import cv2  # OpenCV
import math  # For math operations

from block_mover.msg import BlockObservationArray

# ROS imports
import rospy
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, Range
from visualization_msgs.msg import MarkerArray
from image_geometry import PinholeCameraModel


from matplotlib import pyplot as plt  # For plotting

# TODO; Store all of these in a param file somewhere
BLU_LOW_HUE = 98
BLU_HIGH_HUE = 118
BLU_LOW_SAT = 154
BLU_HIGH_SAT = 255
BLU_LOW_VAL = 0
BLU_HIGH_VAL = 255

GRN_LOW_HUE = 30
GRN_HIGH_HUE = 63
GRN_LOW_SAT = 91
GRN_HIGH_SAT = 255
GRN_LOW_VAL = 0
GRN_HIGH_VAL = 255

TEAL_LOW_HUE = 90
TEAL_HIGH_HUE = 104
TEAL_LOW_SAT = 9
TEAL_HIGH_SAT = 201
TEAL_LOW_VAL = 42
TEAL_HIGH_VAL = 25

RED_LOW_HUE_1 = 0
RED_HIGH_HUE_1 = 0
RED_LOW_HUE_2 = 161
RED_HIGH_HUE_2 = 180
RED_LOW_SAT = 96
RED_HIGH_SAT = 255
RED_LOW_VAL = 0
RED_HIGH_VAL = 255

YEL_LOW_HUE = 4
YEL_HIGH_HUE = 38
YEL_LOW_SAT = 83
YEL_HIGH_SAT = 255
YEL_LOW_VAL = 182
YEL_HIGH_VAL = 255

TBL_LOW_HUE = 180
TBL_HIGH_HUE = 180
TBL_LOW_SAT = 255
TBL_HIGH_SAT = 255
TBL_LOW_VAL = 255
TBL_HIGH_VAL = 255

colors = {
    "red": {
        "low_h": [RED_LOW_HUE_1, RED_LOW_HUE_2],
        "high_h": [RED_HIGH_HUE_1, RED_HIGH_HUE_2],
        "low_s": RED_LOW_SAT,
        "high_s": RED_HIGH_SAT,
        "low_v": RED_LOW_VAL,
        "high_v": RED_HIGH_VAL,
        "color_val": (0, 0, 255)
    },
    "green": {
        "low_h": GRN_LOW_HUE,
        "high_h": GRN_HIGH_HUE,
        "low_s": GRN_LOW_SAT,
        "high_s": GRN_HIGH_SAT,
        "low_v": GRN_LOW_VAL,
        "high_v": GRN_HIGH_VAL,
        "color_val": (0, 255, 0)
    },
    "blue": {
        "low_h": BLU_LOW_HUE,
        "high_h": BLU_HIGH_HUE,
        "low_s": BLU_LOW_SAT,
        "high_s": BLU_HIGH_SAT,
        "low_v": BLU_LOW_VAL,
        "high_v": BLU_HIGH_VAL,
        "color_val": (255, 0, 0)
    },
    "yellow": {
        "low_h": YEL_LOW_HUE,
        "high_h": YEL_HIGH_HUE,
        "low_s": YEL_LOW_SAT,
        "high_s": YEL_HIGH_SAT,
        "low_v": YEL_LOW_VAL,
        "high_v": YEL_HIGH_VAL,
        "color_val": (0, 255, 255)
    },
    "teal": {
        "low_h": TEAL_LOW_HUE,
        "high_h": TEAL_HIGH_HUE,
        "low_s": TEAL_LOW_SAT,
        "high_s": TEAL_HIGH_SAT,
        "low_v": TEAL_LOW_VAL,
        "high_v": TEAL_HIGH_VAL,
        "color_val": (255, 60, 0)
    },
}


class BlockDetector(object):
    def __init__(self, resolution, allowed_circle_center,
                 allowed_circle_diameter):
        self.image_width = resolution[0]
        self.image_height = resolution[1]

        self.image_center = (int(self.image_width/2), int(self.image_height/2))

        self.allowed_circle_center = allowed_circle_center
        self.allowed_circle_diameter = allowed_circle_diameter

        self.curr_image = np.zeros((self.image_height, self.image_width, 3))

        # Converts image from CV
        self.cv_bridge = CvBridge()

        # For debugging
        self.markers = MarkerArray()
        self.ray_markers = MarkerArray()

        # The name of the camera
        self.camera = None
        self.curr_image = None
        self.pad_size = 0

        self.blob_area_min_thresh = None
        self.blob_area_max_thresh = None

        # For morphological opening and closing
        if (self.image_width <= 800):
            self.morph_opening_kernel = (2, 2)
            self.morph_closing_kernel = (2, 2)
        else:
            self.morph_opening_kernel = (7, 7)
            self.morph_closing_kernel = (13, 13)

        # The HSV thresholded images
        self.hsv_thresh_images = {
            "red":
            np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8),
            "yellow":
            np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8),
            "green":
            np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8),
            "blue":
            np.zeros((self.image_height, self.image_width, 3), dtype=np.uint8),
        }

        # The image with detected blocks surrounded by bounding boxes and
        # labeled with their dimensions
        self.bounded_image = np.zeros((self.image_height, self.image_width, 3),
                                      dtype=np.uint8)

        self.opencv3 = str.startswith(cv2.__version__, '3')

        self.camera_height = None

        self.store_segmented_images = False

        self.font = cv2.FONT_HERSHEY_SIMPLEX

    def create_debugging_publisher(self):
        # The HSV segmented images
        self.red_seg_image_pub = rospy.Publisher(
            "block_finder/" + self.camera + "/red_segmented_image",
            Image,
            queue_size=1)
        self.yellow_seg_image_pub = rospy.Publisher(
            "block_finder/" + self.camera + "/yellow_segmented_image",
            Image,
            queue_size=1)
        self.blue_seg_image_pub = rospy.Publisher(
            "block_finder/" + self.camera + "/blue_segmented_image",
            Image,
            queue_size=1)
        self.green_seg_image_pub = rospy.Publisher(
            "block_finder/" + self.camera + "/green_segmented_image",
            Image,
            queue_size=1)

        # The image with bounded boxes around detected blocks
        self.bounded_image_pub = rospy.Publisher(
            "block_finder/" + self.camera + "/bounded_image",
            Image,
            queue_size=1)

        # RVIZ markers for debugging
        self.marker_pub = rospy.Publisher(
            "block_finder/" + self.camera + "/block_markers",
            MarkerArray,
            queue_size=1)

        # Rays from the camera to detected blocks for debugging
        self.ray_marker_pub = rospy.Publisher(
            "block_finder/" + self.camera + "/image_rays",
            MarkerArray,
            queue_size=1)

    def publish_debugging(self):
        self.red_seg_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(
            self.hsv_thresh_images["red"]))
        self.green_seg_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(
            self.hsv_thresh_images["green"]))
        self.blue_seg_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(
            self.hsv_thresh_images["blue"]))
        self.bounded_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(
            self.bounded_image))

    def cam_callback(self, data):
        self.curr_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
        self.detected_blocks = self.detect_blocks()
        self.bounded_image = self.draw_detected_blocks()

    def detect_blocks(self):
        detected_blocks = []
        for color in colors:
            # Threshold based on HSV
            masked_image, hsv_mask = hsv_threshold_image(
                self.curr_image,
                color,
                h_range=(colors[color]["low_h"], colors[color]["high_h"]),
                s_range=(colors[color]["low_s"], colors[color]["high_s"]),
                v_range=(colors[color]["low_v"], colors[color]["high_v"]))

            # Morphological opening (remove small objects from the foreground)
            erode_1 = cv2.erode(
                hsv_mask,
                np.ones(self.morph_opening_kernel, np.uint8),
                iterations=1)
            dilate_1 = cv2.dilate(
                erode_1,
                np.ones(self.morph_opening_kernel, np.uint8),
                iterations=1)

            # Morphological closing (fill small holes in the foreground)
            dilate_2 = cv2.dilate(
                dilate_1,
                np.ones(self.morph_closing_kernel, np.uint8),
                iterations=1)
            erode_2 = cv2.erode(
                dilate_2,
                np.ones(self.morph_closing_kernel, np.uint8),
                iterations=1)

            if (self.store_segmented_images):
                self.hsv_thresh_images[color] = erode_2.copy()

            ret, thresh = cv2.threshold(erode_2, 100, 255, 0)

            if (self.opencv3):
                im2, contours, hierarchy = cv2.findContours(
                    thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            else:
                contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE,
                                                       cv2.CHAIN_APPROX_NONE)

            for contour in contours:
                min_area_rect = cv2.minAreaRect(contour)
                contour_area = min_area_rect[1][0] * min_area_rect[1][1]
                contour_center = min_area_rect[0]

                distance_to_center = distance(
                    contour_center, self.allowed_circle_center)

                # Check that contour area is within thresholds
                if (contour_area > self.blob_area_min_thresh
                        and contour_area < self.blob_area_max_thresh and
                        distance_to_center < self.allowed_circle_diameter):

                    detected_blocks.append((color, min_area_rect))
                else:
                    """
                    rospy.loginfo(
                        "Contour area was too small, probably not a block.")
                    """
                    pass

        return detected_blocks

    def draw_detected_blocks(self):
        image = self.curr_image.copy()
        cv2.circle(image, self.allowed_circle_center,
                   self.allowed_circle_diameter, (0, 0, 0), 1)

        for color, min_area_rect in self.detected_blocks:
            # Draw bounding box in color
            if (self.opencv3):
                bounding_box = np.int0(cv2.boxPoints(min_area_rect))
            else:
                bounding_box = np.int0(cv2.cv.BoxPoints(min_area_rect))

            cv2.drawContours(image, [bounding_box], 0,
                             colors[color]["color_val"], 2)

            cx = int(min_area_rect[0][0])
            cy = int(min_area_rect[0][1])

            # Draw center of bounding box
            cv2.circle(image, (cx, cy),
                       3, colors[color]["color_val"], 1)

            # Write the ratio
            if (bounding_box[1][0] > bounding_box[1][1]):
                block_ratio = bounding_box[1][0] / bounding_box[1][1]
            else:
                block_ratio = bounding_box[1][1] / bounding_box[1][0]

            # Calculate the block angle
            block_angle = min_area_rect[2]

            cv2.putText(image,
                        "R:" + str(round(block_ratio, 2)) +
                        " A: " + str(round(block_angle, 2)),
                        (cx, cy+10),
                        self.font, self.font_size, colors[color]["color_val"],
                        int(self.font_thickness))

            """
            rospy.loginfo("Contour angle: %f (rad) %f in (deg)",
                          math.radians(contour_angle),
                          math.degrees(contour_angle))
            """

        return image


# TODO: Move to utilities!

def distance(p0, p1):
    return math.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)


def hsv_threshold_image(image, color, h_range, s_range, v_range):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    if (color != "red"):
        hsv_mask = cv2.inRange(hsv_image,
                               np.array([h_range[0], s_range[0], v_range[0]]),
                               np.array([h_range[1], s_range[1], v_range[1]]))

    else:
        # There are 2 H ranges for red
        hsv_mask_lower = cv2.inRange(
            hsv_image, np.array([h_range[0][0], s_range[0], v_range[0]]),
            np.array([h_range[1][0], s_range[1], v_range[1]]))

        hsv_mask_higher = cv2.inRange(
            hsv_image, np.array([h_range[0][1], s_range[0], v_range[0]]),
            np.array([h_range[1][1], s_range[1], v_range[1]]))
        hsv_mask = hsv_mask_lower | hsv_mask_higher

    masked_image = cv2.bitwise_and(image, image, mask=hsv_mask)

    return masked_image, hsv_mask


class TopBlockDetector(BlockDetector):
    def __init__(self):
        BlockDetector.__init__(self, resolution=(640, 480),
                               allowed_circle_center=(322, 250),
                               allowed_circle_diameter=201)

        self.camera = "top"
        self.tf_listener = tf.TransformListener()
        self.camera_model = None

        self.blob_area_min_thresh = 40
        self.blob_area_max_thresh = 1000

        # Constant
        self.camera_height = 1.3
        self.store_segmented_images = True

        self.font_size = 0.5
        self.font_thickness = 1

        self.center_dist_thresh = 200

    def create_debugging_publisher(self):
        BlockDetector.create_debugging_publisher(self)

    def create_block_obs_publisher(self):
        self.pub_rate = rospy.Rate(1)  # in Hz
        self.block_obs_pub = rospy.Publisher(
            "block_finder/top/block_obs", BlockObservationArray, queue_size=1)

    def subscribe(self):
        self.cam_sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image,
                                        self.cam_callback)
        """
        self.cam_info_sub = rospy.Subscriber(
            "/ camera/rgb/camera_info", CameraInfo, self.cam_info_callback)
        """


class HandBlockDetector(BlockDetector):
    def __init__(self):
        BlockDetector.__init__(self, resolution=(1280, 800))
        self.camera = "right_hand"

        self.font_size = 3.0
        self.font_thickness = 2

    def create_publisher(self):
        self.pub_rate = rospy.Rate(1)  # in Hz
        pass

    def create_debugging_publisher(self):
        BlockDetector.create_debugging_publisher(self)

    def subscribe(self):
        self.cam_sub = rospy.Subscriber("/cameras/right_hand_camera/image",
                                        Image, self.cam_callback)
        self.cam_info_sub = rospy.Subscriber(
            "/cameras/right_hand_camera/camera_info_std", CameraInfo,
            self.cam_info_callback)
        self.ir_sub = rospy.Subscriber("/robot/range/right_hand_range/state",
                                       Range, self.ir_callback)

    def ir_callback(self, data):
        ir_reading = data.range

        # 0.65 meters is the upper limit of the IR sensor on Baxter
        if (self.ir_reading > 0.65):
            ir_reading = 0.4

        self.blob_area_min_thresh = 400 / ir_reading  # TODO: Tune
        self.blob_area_max_thresh = 100000  # TODO: Tune

        self.camera_height = ir_reading

    def cam_info_callback(self, data):
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        # Unsubscribe after receiving CameraInfo first time
        self.cam_info_sub.unregister()


# Testing for TopBlockDetector
def main():
    top_block_detector = TopBlockDetector()
    rospy.init_node("top_block_detector")
    top_block_detector.subscribe()
    top_block_detector.create_block_obs_publisher()
    top_block_detector.create_debugging_publisher()

    while (not rospy.is_shutdown()):
        top_block_detector.pub_rate.sleep()
        top_block_detector.publish_debugging()


if __name__ == "__main__":
    main()
