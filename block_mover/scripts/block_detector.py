#!/usr/env/python
import numpy as np

import cv2

from block_mover import Block

# ROS imports
import rospy
import tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, Range
from visualization import MarkerArray


class BlockDetector():
    def __init__(self, resolution):
        self.image_width = resolution[0]
        self.image_height = resolution[1]

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
        if(self.image_width <= 600):
            self.morph_opening_kernel = (5, 5)
            self.morph_closing_kernel = (10, 10)
        else:
            self.morph_opening_kernel = (7, 7)
            self.morph_closing_kernel = (13, 13)

        # The HSV thresholded images
        self.hsv_thresh_images = {
            "red": np.zeros((self.image_height, self.image_width, 3),
                            dtype=np.uint8),
            "yellow": np.zeros((self.image_height, self.image_width, 3),
                               dtype=np.uint8),
            "green": np.zeros((self.image_height, self.image_width, 3),
                              dtype=np.uint8),
            "blue": np.zeros((self.image_height, self.image_width, 3),
                             dtype=np.uint8),
        }

        # The image with detected blocks surrounded by bounding boxes and
        # labeled with their dimensions
        self.bounded_image = np.zeros(
            (self.image_height, self.image_width, 3), dtype=np.uint8)

        self.colors = {
            "red":      {"low_h": [RED_LOW_HUE_1, RED_LOW_HUE_2],
                         "high_h": [RED_HIGH_HUE_1, RED_HIGH_HUE_2],
                         "low_s": RED_LOW_SAT,   "high_s": RED_HIGH_SAT,
                         "low_v": RED_LOW_VAL,   "high_v": RED_HIGH_VAL,
                         "color_val": (0, 0, 255)},

            """
            "yellow":   {"low_h": YEL_LOW_HUE,   "high_h": YEL_HIGH_HUE,
                        "low_s": YEL_LOW_SAT,   "high_s": YEL_HIGH_SAT,
                        "low_v": YEL_LOW_VAL,   "high_v": YEL_HIGH_VAL,
                        "color_val": (0, 255, 255)},
            """

            "green":    {"low_h": GRN_LOW_HUE,   "high_h": GRN_HIGH_HUE,
                         "low_s": GRN_LOW_SAT,   "high_s": GRN_HIGH_SAT,
                         "low_v": GRN_LOW_VAL,   "high_v": GRN_HIGH_VAL,
                         "color_val": (0, 255, 0)},

            "blue":     {"low_h": BLU_LOW_HUE,   "high_h": BLU_HIGH_HUE,
                         "low_s": BLU_LOW_SAT,   "high_s": BLU_HIGH_SAT,
                         "low_v": BLU_LOW_VAL,   "high_v": BLU_HIGH_VAL,
                         "color_val": (255, 0, 0)},

            """
            "teal":     {"low_h": TEAL_LOW_HUE,  "high_h": TEAL_HIGH_HUE,
                        "low_s": TEAL_LOW_SAT,  "high_s": TEAL_HIGH_SAT,
                        "low_v": TEAL_LOW_VAL,  "high_v": TEAL_HIGH_VAL,
                        "color_val": (255, 60, 0)},
            """
        }

        self.opencv3 = str.startswith(cv2.__version__, '3')

        self.camera_height = None

    def publish(self):
        # Should be implemented in child class
        pass

    def publish_debugging(self):
        # The HSV segmented images
        self.red_seg_img_pub = rospy.Publisher(
            "block_finder/" + self.camera + "/red_segmented_image", Image,
            queue_size=1)
        self.yellow_seg_img_pub = rospy.Publisher(
            "block_finder/" + self.camera + "/yellow_segmented_image", Image,
            queue_size=1)
        self.blue_seg_img_pub = rospy.Publisher(
            "block_finder/" + self.camera + "/blue_segmented_image", Image,
            queue_size=1)
        self.green_seg_img_pub = rospy.Publisher(
            "block_finder/" + self.camera + "/green_segmented_image", Image,
            queue_size=1)

        # The image with bounded boxes around detected blocks
        self.rect_seg_img_pub = rospy.Publisher(
            "block_finder/" + self.camera + "/rect_segmented_image", Image,
            queue_size=1)

        # RVIZ markers for debugging
        self.marker_pub = rospy.Publisher(
            "block_finder/" + self.camera + "/block_markers", MarkerArray,
            queue_size=1)

        # Rays from the camera to detected blocks for debugging
        self.ray_marker_pub = rospy.Publisher(
            "block_finder/" + self.camera + "/image_rays", MarkerArray,
            queue_size=1)

    def cam_callback(self, data):
        self.curr_image = self.cv_bridge.imgmsg_to_cv2(data)
        self.find_blocks(self)

    def detect_blocks(self):
        for color in self.colors:
            # Threshold based on HSV
            masked_image = hsv_threshold_image(self.curr_image, color,
                                               h_range=(self.colors[color]["low_h"],
                                                        self.colors[color]["high_h"]),
                                               s_range=(self.colors[color]["low_s"],
                                                        self.colors[color]["high_s"]),
                                               v_range=(self.colors[color]["low_v"],
                                                        self.colors[color]["high_v"])
                                               )

            if(self.store_segmented_images):
                self.hsv_thresh_images[color] = masked_image.copy()

            # Morphological opening (remove small objects from the foreground)
            erode_1 = cv2.erode(hsv_mask, np.ones(
                self.morph_opening_kernel, np.uint8), iterations=1)
            dilate_1 = cv2.dilate(erode_1, np.ones(
                self.morph_opening_kernel, np.uint8), iterations=1)

            # Morphological closing (fill small holes in the foreground)
            dilate_2 = cv2.dilate(dilate_1, np.ones(
                self.morph_closing_kernel, np.uint8), iterations=1)
            erode_2 = cv2.erode(dilate_2, np.ones(
                self.morph_closing_kernel, np.uint8), iterations=1)

            ret, thresh = cv2.threshold(erode_2, 157, 255, 0)

            if(self.opencv3):
                im2, contours, hierarchy = cv2.findContours(
                    thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
            else:
                contours, hierarchy = cv2.findContours(
                    thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

            for contour in contours:
                min_area_rect = cv2.minAreaRect(contour)
                contour_area = min_area_rect[1][0] * min_area_rect[1][1]

                # Check that contour area is within thresholds
                if(contour_area > self.blob_area_min_thresh and
                   contour_area < self.blob_area_max_thresh):
                    block_angle = min_area_rect[2]
                    rospy.loginfo("Contour angle: %f (rad) %f in (deg)",
                                  math.radians(contour_angle),
                                  math.degrees(contour_angle))

                    if(self.opencv3):
                        bounding_box = np.int0(cv2.boxPoints(min_area_rect))
                    else:
                        bounding_box = np.int0(cv2.cv.BoxPoints(min_area_rect))
                else:
                    rospy.loginfo(
                        "Contour area was too small, probably not a block.")

            block_length, block_width = calc_block_type(
                min_area_rect, self.camera_height)

            detected_blocks.append(
                Block(color, center, min_area_rect, block_length, block_width,
                      block_angle))

        return detected_blocks

    def draw_detected_blocks(self, detected_blocks):
        for block in detected_blocks:
            # Draw block
            pass


def calc_block_type(min_area_rect, camera_height):

    pass


pass
# TODO: Move to utilities!


def hsv_threshold_image(image, color, h_range, s_range, v_range):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    if(color != "red"):
        hsv_mask = cv2.inRange(hsv_image,
                               np.array(
                                   n[h_range[0], s_range[0], v_range[0]]),
                               np.array(
                                   [h_range[1], s_range[1], v_range[1]])
                               )

    else:
        # There are 2 H ranges for red
        hsv_mask_lower = cv2.inRange(hsv_image,
                                     np.array(
                                         [h_range[0][0], s_range[0][0], v_range[0][0]]),
                                     np.array(
                                         [h_range[1][0], s_range[1][0], v_range[1][0]])
                                     )

        hsv_mask_higher = cv2.inRange(hsv_image,
                                      np.array(
                                          [h_range[0][0], s_range[0][0], v_range[0][0]]),
                                      np.array(
                                          [h_range[1][0], s_range[1][0], v_range[1][0]])
                                      )
        hsv_mask = hsv_mask_1 | hsv_mask_2

    masked_image = cv2.bitwise_and(image, image, mask=hsv_mask)
    return masked_image


class TopBlockDetector(BlockDetector):
    def __init__(self):
        super().__init__()

        self.camera = "top"
        self.tf_listener = tf.TransformListener()
        self.camera_model = None

        self.blob_area_min_thresh = 40
        self.blob_area_max_thresh = 1000

        # Constant
        self.camera_height = 1.3

    def publish(self):
        super().publish_debugging()

    def subscribe(self):
        super().subscribe()

        self.cam_sub = rospy.Subscriber(
            "/cameras/rgb/image_rect_color", Image,
            self.cam_callback)
        self.cam_info_sub = rospy.Subscriber("/ camera/rgb/camera_info",
                                             CameraInfo,
                                             self.cam_info_callback)


class HandBlockDetector(BlockDetector):
    def __init__(self):
        super().__init__()
        self.camera = "right_hand"

    def publish_debugging(self):
        super().publish_debugging()

    def subscribe(self):
        super().subscribe()
        self.cam_sub = rospy.Subscriber(
            "/cameras/right_hand_camera/image", Image,
            self.cam_callback)
        self.cam_info_sub = rospy.Subscriber(
            "/cameras/right_hand_camera/camera_info_std",
            CameraInfo, self.cam_info_callback)
        self.ir_sub = rospy.Subscriber(
            "/robot/range/right_hand_range/state", Range,
            self.ir_callback)

    def ir_callback(self, data):
        ir_reading = data.range

        # 0.65 meters is the upper limit of the IR sensor on Baxter
        if(self.ir_reading > 0.65):
            ir_reading = 0.4

        self.blob_area_min_thresh = 400 / ir_reading  # TODO: Tune
        self.blob_area_max_thresh = 100000  # TODO: Tune

        self.camera_height = ir_reading
