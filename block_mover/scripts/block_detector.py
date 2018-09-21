#!/usr/bin/env python
import numpy as np  # For matrix operations

import cv2  # OpenCV
import math  # For math operations

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

# TODO; Store all of these in a param file somewhere
# To the right are values that are better tuned for good lighting
# whereas the current values can still do well in bad lighting
# (lights off)
BLU_LOW_HUE = 98
BLU_HIGH_HUE = 118
BLU_LOW_SAT = 154
BLU_HIGH_SAT = 255
BLU_LOW_VAL = 0
BLU_HIGH_VAL = 255

GRN_LOW_HUE = 41  # 31
GRN_HIGH_HUE = 70  # 77
GRN_LOW_SAT = 94  # 89
GRN_HIGH_SAT = 255
GRN_LOW_VAL = 13  # 71
GRN_HIGH_VAL = 212

TEAL_LOW_HUE = 90
TEAL_HIGH_HUE = 104
TEAL_LOW_SAT = 9
TEAL_HIGH_SAT = 201
TEAL_LOW_VAL = 42
TEAL_HIGH_VAL = 25

RED_LOW_HUE_1 = 0
RED_HIGH_HUE_1 = 11  # 0
RED_LOW_HUE_2 = 161
RED_HIGH_HUE_2 = 180
RED_LOW_SAT = 96
RED_HIGH_SAT = 255
RED_LOW_VAL = 47  # 47
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
    # "yellow": {
    #    "low_h": YEL_LOW_HUE,
    #    "high_h": YEL_HIGH_HUE,
    #    "low_s": YEL_LOW_SAT,
    #    "high_s": YEL_HIGH_SAT,
    #    "low_v": YEL_LOW_VAL,
    #    "high_v": YEL_HIGH_VAL,
    #    "color_val": (0, 255, 255)
    # },
    # "teal": {
    #    "low_h": TEAL_LOW_HUE,
    #    "high_h": TEAL_HIGH_HUE,
    #    "low_s": TEAL_LOW_SAT,
    #    "high_s": TEAL_HIGH_SAT,
    #    "low_v": TEAL_LOW_VAL,
    #    "high_v": TEAL_HIGH_VAL,
    #    "color_val": (255, 60, 0)
    # },
}

TUNE_HSV_VALS = False


class BlockDetector():
    def __init__(self, resolution, allowed_circle_center,
                 allowed_circle_diameter,
                 allowed_circle_thickness, pub_rate):
        self.image_width = resolution[0]
        self.image_height = resolution[1]

        self.image_center = (int(self.image_width/2), int(self.image_height/2))

        self.allowed_circle_center = allowed_circle_center
        self.allowed_circle_diameter = allowed_circle_diameter

        self.allowed_circle_thickness = allowed_circle_thickness

        self.curr_image = np.zeros((self.image_height, self.image_width, 3))

        # Converts image from CV
        self.cv_bridge = CvBridge()

        # For debugging
        self.block_markers = MarkerArray()
        self.ray_markers = MarkerArray()

        # The name of the camera
        self.camera = None
        self.curr_image = None
        self.pad_size = 0

        self.blob_area_min_thresh = None
        self.blob_area_max_thresh = None

        # For morphological opening and closing
        if (self.image_width <= 800):
            self.morph_opening_kernel = (1, 1)
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
        self.pub_rate = rospy.Rate(pub_rate)  # In Hz

    def create_debugging_publisher(self):
        # The HSV segmented images
        self.red_seg_image_pub = rospy.Publisher(
            "block_detector/" + self.camera + "/red_segmented_image",
            Image,
            queue_size=10)
        self.yellow_seg_image_pub = rospy.Publisher(
            "block_detector/" + self.camera + "/yellow_segmented_image",
            Image,
            queue_size=10)
        self.blue_seg_image_pub = rospy.Publisher(
            "block_detector/" + self.camera + "/blue_segmented_image",
            Image,
            queue_size=10)
        self.green_seg_image_pub = rospy.Publisher(
            "block_detector/" + self.camera + "/green_segmented_image",
            Image,
            queue_size=10)

        # The image with bounded boxes around detected blocks
        self.bounded_image_pub = rospy.Publisher(
            "block_detector/" + self.camera + "/bounded_image",
            Image,
            queue_size=10)

        # RVIZ markers for debugging
        self.marker_pub = rospy.Publisher(
            "block_detector/" + self.camera + "/block_markers",
            MarkerArray,
            queue_size=10)

        # Rays from the camera to detected blocks for debugging
        self.ray_marker_pub = rospy.Publisher(
            "block_detector/" + self.camera + "/image_rays",
            MarkerArray,
            queue_size=10)

    def publish_debugging(self):
        self.red_seg_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(
            self.hsv_thresh_images["red"]))
        self.green_seg_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(
            self.hsv_thresh_images["green"]))
        self.blue_seg_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(
            self.hsv_thresh_images["blue"]))
        self.bounded_image_pub.publish(self.cv_bridge.cv2_to_imgmsg(
            self.bounded_image))

    def publish_markers(self):
        self.marker_pub.publish(self.block_markers)

    def cam_callback(self, data):
        self.curr_image = self.cv_bridge.imgmsg_to_cv2(data, "bgr8")
        if(TUNE_HSV_VALS):
            self.find_hsv_values()
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
                contour_center = (int(min_area_rect[0][0]),
                                  int(min_area_rect[0][1]))

                distance_to_center = distance(
                    contour_center, self.allowed_circle_center)

                # Check that contour area is within thresholds
                if (contour_area > self.blob_area_min_thresh
                        and contour_area < self.blob_area_max_thresh and
                        distance_to_center < self.allowed_circle_diameter):
                                # Write the ratio

                    if (min_area_rect[1][0] > min_area_rect[1][1]):
                        block_length_pix = round(min_area_rect[1][0], 0)
                        block_width_pix = round(min_area_rect[1][1], 0)
                    else:
                        block_length_pix = round(min_area_rect[1][1], 0)
                        block_width_pix = round(min_area_rect[1][0], 0)
                    # Get the block length in block units (1x1, 1x4, etc)
                    block_length, block_width = self.get_block_type(
                        block_length_pix, block_width_pix)
                    # Calculate the block angle
                    # Adding 90 because the resultant angle from min_area_rect
                    # is always [-90, 0] so we can shift it's range to
                    # [0, 90] and then convert from degrees to radians
                    if(min_area_rect[1][0] < min_area_rect[1][1]):
                        block_angle = math.radians(min_area_rect[2] + 180)
                    else:
                        block_angle = math.radians(min_area_rect[2] + 90)

                    detected_blocks.append(
                        {"color": color,
                         "min_area_rect": min_area_rect,
                         "block_center": contour_center,
                         "block_length": block_length,
                         "block_width": block_width,
                         "block_angle": block_angle
                         })

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
                   self.allowed_circle_diameter + self.allowed_circle_thickness/2, (0, 0, 0), self.allowed_circle_thickness)

        for detected_block in self.detected_blocks:
            # Draw bounding box in color
            if (self.opencv3):
                bounding_box = np.int0(cv2.boxPoints(
                    detected_block["min_area_rect"]))
            else:
                bounding_box = np.int0(cv2.cv.BoxPoints(
                    detected_block["min_area_rect"]))

            cv2.drawContours(image, [bounding_box], 0,
                             colors[detected_block["color"]]["color_val"], 2)

            # Draw center of bounding box
            cv2.circle(image, detected_block["block_center"],
                       3, colors[detected_block["color"]]["color_val"], 1)

            cv2.putText(image,
                        str(detected_block["block_width"]) + "x" +
                        str(detected_block["block_length"]),
                        # For printing angle on image uncomment:
                        # + " A: " +
                        # str(math.degrees(
                        #    round(detected_block["block_angle"], 2))),
                        detected_block["block_center"],
                        self.font, self.font_size, colors[detected_block["color"]
                                                          ]["color_val"],
                        int(self.font_thickness))

            """
            rospy.loginfo("Contour angle: %f (rad) %f in (deg)",
                          math.radians(contour_angle),
                          math.degrees(contour_angle))
            """

        return image

    def get_block_type(self, block_length, block_width):
        pass
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

    def find_hsv_values(self):
        # optional argument for trackbars

        # named ites for easy reference
        barsWindow = 'Bars'
        hl = 'H Low'
        hh = 'H High'
        sl = 'S Low'
        sh = 'S High'
        vl = 'V Low'
        vh = 'V High'

        # set up for video capture on camera 0

        # create window for the slidebars
        cv2.namedWindow(barsWindow, flags=cv2.WINDOW_AUTOSIZE)

        # create the sliders
        cv2.createTrackbar(hl, barsWindow, 0, 179, nothing)
        cv2.createTrackbar(hh, barsWindow, 0, 179, nothing)
        cv2.createTrackbar(sl, barsWindow, 0, 255, nothing)
        cv2.createTrackbar(sh, barsWindow, 0, 255, nothing)
        cv2.createTrackbar(vl, barsWindow, 0, 255, nothing)
        cv2.createTrackbar(vh, barsWindow, 0, 255, nothing)

        # set initial values for sliders
        cv2.setTrackbarPos(hl, barsWindow, 0)
        cv2.setTrackbarPos(hh, barsWindow, 179)
        cv2.setTrackbarPos(sl, barsWindow, 0)
        cv2.setTrackbarPos(sh, barsWindow, 255)
        cv2.setTrackbarPos(vl, barsWindow, 0)
        cv2.setTrackbarPos(vh, barsWindow, 255)

        while(True):
            frame = self.curr_image
            frame = cv2.GaussianBlur(frame, (5, 5), 0)

            # convert to HSV from BGR
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # read trackbar positions for all
            hul = cv2.getTrackbarPos(hl, barsWindow)
            huh = cv2.getTrackbarPos(hh, barsWindow)
            sal = cv2.getTrackbarPos(sl, barsWindow)
            sah = cv2.getTrackbarPos(sh, barsWindow)
            val = cv2.getTrackbarPos(vl, barsWindow)
            vah = cv2.getTrackbarPos(vh, barsWindow)

            # make array for final values
            HSVLOW = np.array([hul, sal, val])
            HSVHIGH = np.array([huh, sah, vah])

            # apply the range on a mask
            mask = cv2.inRange(hsv, HSVLOW, HSVHIGH)
            maskedFrame = cv2.bitwise_and(frame, frame, mask=mask)

            # display the camera and masked images
            cv2.imshow('Masked', maskedFrame)
            cv2.imshow('Camera', frame)

            # check for q to quit program with 5ms delay
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break

        # clean up our resources
        cv2.destroyAllWindows()


def nothing(x):
    pass


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


def create_block_marker(frame, id, position, orientation, length, width,
                        block_color, transparency):
    curr_marker = Marker()
    curr_marker.header.frame_id = frame

    curr_marker.type = 1  # sphere
    curr_marker.action = 0
    curr_marker.id = id
    curr_marker.frame_locked = True
    curr_marker.pose.position = position
    curr_marker.pose.orientation = orientation

    single_unit_dim = 0.03

    curr_marker.scale.x = width*single_unit_dim
    curr_marker.scale.y = length*single_unit_dim
    curr_marker.scale.z = 1.2 * single_unit_dim

    if(block_color == "red"):
        curr_marker.color.r = 1.0
        curr_marker.color.g = 0.0
        curr_marker.color.b = 0.0
    elif(block_color == "blue"):
        curr_marker.color.r = 0.0
        curr_marker.color.g = 0.0
        curr_marker.color.b = 1.0
    elif(block_color == "green"):
        curr_marker.color.r = 0.0
        curr_marker.color.g = 1.0
        curr_marker.color.b = 0.0
    elif(block_color == "yellow"):
        curr_marker.color.r = 1.0
        curr_marker.color.g = 1.0
        curr_marker.color.b = 0.0
    elif(block_color == "teal"):
        curr_marker.color.r = 0.0
        curr_marker.color.g = 0.2
        curr_marker.color.b = 1.0
    else:
        rospy.logerr(
            "Color %s doesn't have a supported marker yet! you should add one.", block_color)

    # Alpha value (transparency)
    curr_marker.color.a = transparency

    curr_marker.lifetime = rospy.Duration(0)

    return curr_marker

# From https://rosettacode.org/wiki/Find_the_intersection_of_a_line_with_a_plane


def line_plane_intersection(plane_normal, plane_point, ray_direction,
                            ray_point, epsilon=1e-6):
    n_dot_u = plane_normal.dot(ray_direction)
    if(math.fabs(n_dot_u) < epsilon):
        return None

    w = ray_point - plane_point
    si = -plane_normal.dot(w) / n_dot_u
    psi = w + si * ray_direction + plane_point

    return psi