#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import sys
import copy
import rospkg
import math
import tf

from matplotlib import pyplot as plt
from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, CameraInfo, Range
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion, Pose2D
from image_geometry import PinholeCameraModel
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from block_mover.msg import BlockObservation, BlockObservationArray, BlockPixelLoc, BlockPixelLocArray

#import baxter_interface
# Check opencv version
if(str.startswith(cv2.__version__, '3')):
    print("OpenCV3 detected.")
    OPENCV3 = True
    TOP_CAM = True
    HAND_CAM = False
else:
    print("OpenCV2 detected.")
    OPENCV3 = False
    TOP_CAM = False
    HAND_CAM = True


# To enable HSV values that were trained in gazebo
SIM = False

# Enables a tool that helps to tune HSV thresholds
TUNE_HSV_VALS = False

# TODO; Store all of these in a param file somewhere
if(TOP_CAM):
    BLU_LOW_HUE     = 106
    BLU_HIGH_HUE    = 115
    BLU_LOW_SAT     = 25
    BLU_HIGH_SAT    = 255
    BLU_LOW_VAL     = 127
    BLU_HIGH_VAL    = 255

    GRN_LOW_HUE     = 30
    GRN_HIGH_HUE    = 75
    GRN_LOW_SAT     = 15
    GRN_HIGH_SAT    = 255
    GRN_LOW_VAL     = 40
    GRN_HIGH_VAL    = 255

    TEAL_LOW_HUE     = 90
    TEAL_HIGH_HUE    = 104
    TEAL_LOW_SAT     = 9
    TEAL_HIGH_SAT    = 201
    TEAL_LOW_VAL     = 42
    TEAL_HIGH_VAL    = 255

    RED_LOW_HUE_1     = 0
    RED_HIGH_HUE_1    = 40
    RED_LOW_HUE_2     = 130
    RED_HIGH_HUE_2    = 180 
    RED_LOW_SAT     = 25
    RED_HIGH_SAT    = 255
    RED_LOW_VAL     = 0
    RED_HIGH_VAL    = 255

    YEL_LOW_HUE     = 4
    YEL_HIGH_HUE    = 38
    YEL_LOW_SAT     = 83
    YEL_HIGH_SAT    = 255
    YEL_LOW_VAL     = 182
    YEL_HIGH_VAL    = 255

    TBL_LOW_HUE     = 19
    TBL_HIGH_HUE    = 33
    TBL_LOW_SAT     = 0
    TBL_HIGH_SAT    = 255
    TBL_LOW_VAL     = 0
    TBL_HIGH_VAL    = 255


if(HAND_CAM):
    BLU_LOW_HUE     = 90
    BLU_HIGH_HUE    = 133
    BLU_LOW_SAT     = 117
    BLU_HIGH_SAT    = 255
    BLU_LOW_VAL     = 28
    BLU_HIGH_VAL    = 255

    GRN_LOW_HUE     = 37
    GRN_HIGH_HUE    = 104
    GRN_LOW_SAT     = 60
    GRN_HIGH_SAT    = 199
    GRN_LOW_VAL     = 10
    GRN_HIGH_VAL    = 255

    TEAL_LOW_HUE     = 90
    TEAL_HIGH_HUE    = 104
    TEAL_LOW_SAT     = 97
    TEAL_HIGH_SAT    = 201
    TEAL_LOW_VAL     = 42
    TEAL_HIGH_VAL    = 255

    RED_LOW_HUE_1     = 0
    RED_HIGH_HUE_1    = 10
    RED_LOW_HUE_2     = 160
    RED_HIGH_HUE_2    = 180
    RED_LOW_SAT     = 5
    RED_HIGH_SAT    = 255
    RED_LOW_VAL     = 20
    RED_HIGH_VAL    = 255

    YEL_LOW_HUE     = 10
    YEL_HIGH_HUE    = 70
    YEL_LOW_SAT     = 33
    YEL_HIGH_SAT    = 255
    YEL_LOW_VAL     = 182
    YEL_HIGH_VAL    = 255

if(SIM):
    BLU_LOW_HUE     = 90
    BLU_HIGH_HUE    = 130
    BLU_LOW_SAT     = 0
    BLU_HIGH_SAT    = 255
    BLU_LOW_VAL     = 0
    BLU_HIGH_VAL    = 255

    GRN_LOW_HUE     = 30
    GRN_HIGH_HUE    = 80
    GRN_LOW_SAT     = 0
    GRN_HIGH_SAT    = 255
    GRN_LOW_VAL     = 0
    GRN_HIGH_VAL    = 255
    
    TEAL_LOW_HUE     = 80
    TEAL_HIGH_HUE    = 105
    TEAL_LOW_SAT     = 0
    TEAL_HIGH_SAT    = 255
    TEAL_LOW_VAL     = 0
    TEAL_HIGH_VAL    = 100

    RED_LOW_HUE_1     = 0
    RED_HIGH_HUE_1    = 10
    RED_LOW_HUE_2     = 175
    RED_HIGH_HUE_2    = 180
    RED_LOW_SAT     = 100
    RED_HIGH_SAT    = 255
    RED_LOW_VAL     = 140
    RED_HIGH_VAL    = 255

    YEL_LOW_HUE     = 1
    YEL_HIGH_HUE    = 30
    YEL_LOW_SAT     = 180
    YEL_HIGH_SAT    = 255
    YEL_LOW_VAL     = 0
    YEL_HIGH_VAL    = 255

colors = {
        "red":      {   "low_h": [RED_LOW_HUE_1, RED_LOW_HUE_2],   "high_h": [RED_HIGH_HUE_1, RED_HIGH_HUE_2],
                        "low_s": RED_LOW_SAT,   "high_s": RED_HIGH_SAT,
                        "low_v": RED_LOW_VAL,   "high_v": RED_HIGH_VAL  },
        #"yellow":   {   "low_h": YEL_LOW_HUE,   "high_h": YEL_HIGH_HUE,
        #                "low_s": YEL_LOW_SAT,   "high_s": YEL_HIGH_SAT,
        #                "low_v": YEL_LOW_VAL,   "high_v": YEL_HIGH_VAL  },
        "green":    {   "low_h": GRN_LOW_HUE,   "high_h": GRN_HIGH_HUE,
                        "low_s": GRN_LOW_SAT,   "high_s": GRN_HIGH_SAT,
                        "low_v": GRN_LOW_VAL,   "high_v": GRN_HIGH_VAL  },
        "blue":     {   "low_h": BLU_LOW_HUE,   "high_h": BLU_HIGH_HUE,
                        "low_s": BLU_LOW_SAT,   "high_s": BLU_HIGH_SAT,
                        "low_v": BLU_LOW_VAL,   "high_v": BLU_HIGH_VAL  },
        #"teal":     {   "low_h": TEAL_LOW_HUE,  "high_h": TEAL_HIGH_HUE,
        #                "low_s": TEAL_LOW_SAT,  "high_s": TEAL_HIGH_SAT,
        #                "low_v": TEAL_LOW_VAL,  "high_v": TEAL_HIGH_VAL  }
}

color_vals = {
    "red":    (0, 0, 255),
    "green":  (0, 255, 0),
    "blue":   (255, 0, 0),
    "yellow": (0, 255, 255),
    "teal":   (255, 60, 0)
}

BLOCK_TYPE_1X1 = 1
BLOCK_TYPE_1X2 = 2
BLOCK_TYPE_1X3 = 4
BLOCK_TYPE_1X4 = 8
BLOCK_TYPE_2X2 = 16


COLOR_RED = 1
COLOR_GRN = 2
COLOR_BLU = 4


# From https://github.com/botforge/ColorTrackbar
def nothing(x):
    pass


# From https://github.com/botforge/ColorTrackbar
def find_hsv_values(img):
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
    cv2.namedWindow(barsWindow, flags = cv2.WINDOW_AUTOSIZE)

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
        frame = img
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
        maskedFrame = cv2.bitwise_and(frame, frame, mask = mask)

        # display the camera and masked images
        cv2.imshow('Masked', maskedFrame)
        cv2.imshow('Camera', frame)

        # check for q to quit program with 5ms delay
        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

    # clean up our resources
    cap.release()
    cv2.destroyAllWindows()


class BlockFinder():
    def __init__(self, camera):
        self.camera = camera

        self.block_poses = []
        self.markers = MarkerArray()
        self.ray_markers = MarkerArray()
        self.tf_listener = tf.TransformListener()
        self.top_camera_model = None
        self.hand_camera_model = None
        self.min_ir_depth = 0.1
        self.object_height = 0.1
        self.bridge = CvBridge()
        self.ir_reading = None

        if(self.camera == "top"):
            self.transparency = 1.

            try:
                self.tf_listener = tf.TransformListener()
                time = rospy.Time(0)
                self.tf_listener.waitForTransform("/camera_link", "/base", time, rospy.Duration(4.0))
                (trans,rot) = self.tf_listener.lookupTransform("/camera_link", "/base", time)

                self.top_to_base_mat = tf.transformations.compose_matrix(translate = trans, angles=tf.transformations.euler_from_quaternion(rot))
            except (tf.LookupException, tf.ConnectivityException):
                rospy.loginfo("No transform from base to camera available!")

        elif(self.camera == "right_hand"):
            self.transparency = 0.5

        self.pub_rate = rospy.Rate(1)
        self.seg_img = {
            "red": np.zeros((800,800,3), dtype=np.uint8),
            "yellow": np.zeros((800,800,3), dtype=np.uint8),
            "green": np.zeros((800,800,3), dtype=np.uint8),
            "blue": np.zeros((800,800,3), dtype=np.uint8),
            }
        self.rect_seg_img = np.zeros((800,800,3), dtype=np.uint8)

        # TODO: Tune!
        self.top_cam_table_dist = 1.33

        self.block_obs = []

        self.detected_blocks = 0

    def publish(self):
        self.pose_pub           = rospy.Publisher("block_finder/" + self.camera + "/block_poses", PoseArray, queue_size=1)

        # The segmented images
        self.red_seg_img_pub    = rospy.Publisher("block_finder/" + self.camera + "/red_segmented_image", Image, queue_size=1)
        self.yellow_seg_img_pub = rospy.Publisher("block_finder/" + self.camera + "/yellow_segmented_image", Image, queue_size=1)
        self.blue_seg_img_pub   = rospy.Publisher("block_finder/" + self.camera + "/blue_segmented_image", Image, queue_size=1)
        self.green_seg_img_pub  = rospy.Publisher("block_finder/" + self.camera + "/green_segmented_image", Image, queue_size=1)

        # The image with bounded boxes around detected blocks
        self.rect_seg_img_pub   = rospy.Publisher("block_finder/" + self.camera + "/rect_segmented_image", Image, queue_size=1)
        # The detected poses of markers
        self.marker_pub         = rospy.Publisher("block_finder/" + self.camera + "/block_markers", MarkerArray, queue_size=1)
        # Rays from the camera to detected blocks
        self.ray_marker_pub     = rospy.Publisher("block_finder/" + self.camera + "/image_rays", MarkerArray, queue_size=1)

        # The observations of blocks
        self.block_obs_pub      = rospy.Publisher("block_finder/" + self.camera + "/block_obs", BlockObservationArray, queue_size=1) 
        self.pixel_loc_pub      = rospy.Publisher("block_finder/" + self.camera + "/block_pixel_locs", BlockPixelLocArray, queue_size=1)

    def subscribe(self):
        if(self.camera == "right_hand"):
            # The camera in left or right hand
            self.hand_cam_sub   = rospy.Subscriber("/cameras/" + self.camera + "_camera/image", Image, self.hand_cam_callback)
            self.hand_cam_info_sub       = rospy.Subscriber("/cameras/" + self.camera + "_camera/camera_info_std", CameraInfo, self.hand_cam_info_callback)
            self.ir_sub         = rospy.Subscriber("/robot/range/" + self.camera + "_range/state", Range, self.ir_callback)
        elif(self.camera == "top"):
            # The camera above the table
            self.top_cam_sub    = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.top_cam_callback)
            self.top_cam_info_sub       = rospy.Subscriber("/camera/rgb/camera_info", CameraInfo, self.top_cam_info_callback)



        
    def hand_cam_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data)

        self.find_blocks(cv_image)
    
    def top_cam_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        self.find_blocks(cv_image)

    '''
    Thresholds camera image and stores object centroid location (x,y) in Baxter's base frame.
    '''
    def find_blocks(self, cv_image):
        block_pose_list = []
        block_marker_list = MarkerArray()
        ray_marker_list = MarkerArray()
        block_obs_list = []
        block_pixel_locs_list = []
        
        # Mask cv_image to remove baxter's grippers

        if(TUNE_HSV_VALS):
            find_hsv_values(cv_image)

        height, width, depth = cv_image.shape
        low_s = 0
        low_v = 0

        if(self.camera == "top"):        
            # Remove table from hsv image
            hsv = remove_table(cv_image)
        else: 
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        images = [hsv, hsv, hsv, hsv, hsv]
        i = 0
        ray_id = 0

        # Find table

        if(self.camera == "right_hand"):
            if(self.ir_reading != None):
                area_min_threshold = 180 / self.ir_reading
                area_max_threshold = 100000 # TODO: tune
            else:
                area_min_threshold = 180 / 0.4
                area_max_threshold = 100000 # TODO: tune

        elif(self.camera == "top"):
            area_min_threshold= 40
            area_max_threshold = 1000

        for color in colors:
            low_h = colors[color]["low_h"]
            high_h = colors[color]["high_h"]
            low_s = colors[color]["low_s"]
            high_s = colors[color]["high_s"]
            low_v = colors[color]["low_v"]
            high_v = colors[color]["high_v"]

            #Converting image to HSV format
            if color == "red":
                hsv_mask_1 = cv2.inRange(hsv, np.array([low_h[0], low_s, low_v]), np.array([high_h[0], high_s, high_v]))
                hsv_mask_2 = cv2.inRange(hsv, np.array([low_h[1], low_s, low_v]), np.array([high_h[1], high_s, high_v]))

                hsv_mask = hsv_mask_1 | hsv_mask_2

            else:
                hsv_mask = cv2.inRange(hsv, np.array([low_h, low_s, low_v]), np.array([high_h, high_s, high_v]))

            # Apply mask to original image
            masked_img = cv2.bitwise_and(cv_image, cv_image, mask=hsv_mask)

            # Store HSV masked image for the current color
            self.seg_img[color] = masked_img.copy()
            #cv2.imshow(color, masked_img)

            #Morphological opening (remove small objects from the foreground)
            erode_1 = cv2.erode(hsv_mask, np.ones((5,5), np.uint8), iterations=1)
            dilate_1 = cv2.dilate(erode_1, np.ones((5,5), np.uint8), iterations=1)

            #Morphological closing (fill small holes in the foreground)
            dilate_2 = cv2.dilate(dilate_1, np.ones((10, 10), np.uint8), iterations=1)
            erode_2 = cv2.erode(dilate_2, np.ones((10,10), np.uint8), iterations=1)

            images[i] = erode_2.copy()
            
            ret, thresh = cv2.threshold(erode_2,157,255,0)
            if(OPENCV3):
                im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

            else:
                contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

            #Draw the countours.
            #cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
            
            num_obj = len(contours) # number of objects found in current frame

            cv2.circle(cv_image, (320, 200), 5, (255, 255, 0), 1)
            cv2.circle(cv_image, (340, 20), 5, (0, 255, 255), 1)
            
            
            # At 0 Meters
            cv2.circle(cv_image, (325, 129), 5, (255, 0, 255), 1)

            # At -0.15 Meters
            cv2.circle(cv_image, (330, 94), 5, (255, 255, 0), 1)

            rospy.loginfo("Found %d %s objects", num_obj, color)

            for contour in contours:
                moms = cv2.moments(contour)
                cx = int(moms['m10']/moms['m00'])
                cy = int(moms['m01']/moms['m00'])

                if(self.camera == "top"):
                    # Block should not be outside of circle centered at 319,255 with radius 200
                    d = math.sqrt((cx - 319)**2 + (cy - 255)**2)

                    if(d > 198):
                        continue
                        
                area = cv2.contourArea(contour)
                
                if(area > area_min_threshold and area < area_max_threshold):
                    x, y, w, h = cv2.boundingRect(contour)
                    rect = cv2.minAreaRect(contour)
                    
                    #rospy.loginfo("AREA: %f", area)


                    if(OPENCV3):
                        box = cv2.boxPoints(rect)
                    
                    else:
                        box = cv2.cv.BoxPoints(rect)

                    box = np.int0(box)

                    cv2.drawContours(cv_image, [box], 0, color_vals[color] , 2)

                    cropped_img = masked_img[y-5:y+h+5, x-5:x+w+5]

                    if(self.camera == "top"):
                        cropped_img = np.flip(cropped_img, 0)
                    
                    block_angle = calc_angle(cropped_img)


                    block_ratio = calc_ratio(rect[1][1], rect[1][0])

                    block_length, block_width = calc_block_type(block_ratio)


                    if (moms['m00'] > area_min_threshold and moms['m00'] < area_max_threshold):
                        obj_found = True
                        

                        # print 'cx = ', cx
                        # print 'cy = ', cy

                        #cx = cx - self.camera_model.cx()
                        #cy = cy - self.camera_model.cy()

                        cv2.circle(cv_image,(cx,cy), 10, color_vals[color], 1)

                        # Write the block tshape
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        if(self.camera == "top"):
                            font_size = 0.5
                            font_thickness = 1
                        else:
                            font_size = 3.0
                            font_thickness = 2 

                        cv2.putText(cv_image,block_type_string(block_length, block_width),(cx,cy), font, font_size, color_vals[color], font_thickness)

                        self.pixel_loc = [cx, cy]
                        
                        if(self.camera == "right_hand"):
                            vec = np.array(self.hand_camera_model.projectPixelTo3dRay((cx, cy)))

                        
                        elif(self.camera == "top"):
                            vec = np.array(self.top_camera_model.projectPixelTo3dRay((cx, cy)))
                            new_vec = vec.copy()
                            new_vec[0] = vec[2]
                            new_vec[1] = -vec[0]
                            new_vec[2] = -vec[1]
                            vec = new_vec.copy()

                        else:
                            # Invalid camera name
                            rospy.loginfo("The camera name you passed to find blocks is invalid!")
                            return

                        # If we're using the hand camera, make sure we have a valid IR reading...
                        if(self.camera == "top" or (self.camera == "right_hand" and self.ir_reading != None)):
                            if(self.camera =="right_hand"):
                                d = self.ir_reading + 0.15
                            elif(self.camera =="top"):
                                d = self.top_cam_table_dist

                            ray_pt_1 = np.array([0,0,0])
                            ray_pt_2 = 2 * vec

                            norm_vec = np.array([0, 0, 1])

                            rospy.loginfo("Vec: %f, %f, %f", vec[0], vec[1], vec[2])
                            rospy.loginfo("Norm Vec: %f, %f, %f", norm_vec[0], norm_vec[1], norm_vec[2])

                            d_proj = d * np.dot(norm_vec, vec) / (np.linalg.norm(norm_vec) * np.linalg.norm(vec))

                            rospy.loginfo("Distance to object: %f", d)
                            rospy.loginfo("Projected distance to object: %f", d_proj)
                            
                            d_cam = d * vec

                            homog_d_cam = np.concatenate((d_cam, np.ones(1))).reshape((4,1))

                            homog_ray_pt_1 = np.concatenate((ray_pt_1, np.ones(1))).reshape((4,1))
                            homog_ray_pt_2 = np.concatenate((ray_pt_2, np.ones(1))).reshape((4,1))  
                            
                            if(self.camera == "top"):
                                camera_to_base = self.top_to_base_mat
                        
                            else:
                                # Wait for transformation from base to camera as this change as the hand_camera moves
                                self.tf_listener.waitForTransform('/base', self.camera + "_camera", rospy.Time(), rospy.Duration(4))
                                (trans, rot) = self.tf_listener.lookupTransform('/base', self.camera + "_camera", rospy.Time())

                                camera_to_base = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))

                            block_position_arr = np.dot(camera_to_base, homog_d_cam) 

                            # Create a ray from the camera to the detected block                            
                            # Transform ray to base frame
                            ray_pt_1_tf = np.dot(camera_to_base, homog_ray_pt_1)

                            ray_pt_2_tf = np.dot(camera_to_base, homog_ray_pt_2)

                            ray_marker_list.markers.append(create_ray_marker(
                                                frame="base", 
                                                id=ray_id,
                                                point1=Point(ray_pt_1_tf[0], ray_pt_1_tf[1], ray_pt_1_tf[2]),
                                                point2=Point(ray_pt_2_tf[0], ray_pt_2_tf[1], ray_pt_2_tf[2]),
                                                ray_color=color,
                                                transparency = self.transparency
                                            )
                            )

                            ray_id += 1
                            rospy.loginfo("Block position: %f, %f, %f", block_position_arr[0], block_position_arr[1], block_position_arr[2])
                            rospy.loginfo("Block type: %s", block_type_string(block_length, block_width))

                            block_position_p = Point()
                            block_position_arr_copy = block_position_arr.copy()
                            block_position_p.x = block_position_arr_copy[0]
                            block_position_p.y = block_position_arr_copy[1]
                            block_position_p.z = -.1

                            # TODO: double check that the angle is correct (What if camera rotates?)
                            # Rotation about z-axis
                            block_orientation_arr = tf.transformations.quaternion_from_euler(0, 0, block_angle)

                            block_orientation = Quaternion()
                            block_orientation.x = block_orientation_arr[0]
                            block_orientation.y = block_orientation_arr[1]
                            block_orientation.z = block_orientation_arr[2]
                            block_orientation.w = block_orientation_arr[3]

                            # Create a marker to visualize in RVIZ 
                            curr_marker = create_block_marker(frame = "base", id = len(block_marker_list.markers), position = block_position_p, orientation=block_orientation, length=block_length, width=block_width, block_color = color, transparency = self.transparency)

                            #rospy.loginfo("Adding new marker and block pose!")
                            block_marker_list.markers.append(curr_marker)
                            block_pose_list.append(Pose(position=block_position_p, orientation=block_orientation))

                            # TODO: The block angle will still be wrong. Need to transform it from the camera coordinate to the world frame
                            block_obs_list.append(BlockObservation(pose = Pose2D(x = block_position_p.x, y = block_position_p.y, theta=block_angle), color=color, length=block_length, width=block_width))
                            block_pixel_locs_list.append(BlockPixelLoc(x=cx, y=cy, theta=block_angle, color=color, length=block_length, width=block_width))

                        else:
                            rospy.loginfo("No ir_data has been recieved yet!")
                    else:
                        rospy.loginfo("Moments aren't large enough!")
                else:
                    pass
                    rospy.loginfo("Contour area is not large enough!")
        
        self.rect_seg_img = cv_image.copy()
        self.ray_markers = ray_marker_list
        self.block_markers = block_marker_list
        
        self.block_obs = block_obs_list
        self.block_pixel_locs = block_pixel_locs_list

        self.detected_blocks = len(block_obs_list)
        

    def hand_cam_info_callback(self, data):
        self.hand_camera_model = PinholeCameraModel()
        self.hand_camera_model.fromCameraInfo(data)
        self.hand_cam_info_sub.unregister() # Unsubscribe after receiving CameraInfo first time
    
    def top_cam_info_callback(self, data):
        self.top_camera_model = PinholeCameraModel()
        self.top_camera_model.fromCameraInfo(data)
        self.top_cam_info_sub.unregister() # Unsubscribe after receiving CameraInfo first time

    def ir_callback(self, data):
        self.ir_reading = data.range
        #rospy.loginfo("IR reading: %f", self.ir_reading)

        if(self.ir_reading > 65):
            #rospy.loginfo("Invalid IR reading")
            self.ir_reading = 0.4
def remove_table(cv_image):
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    hsv_mask = cv2.inRange(hsv, np.array([TBL_LOW_HUE, TBL_LOW_SAT, TBL_LOW_VAL]), np.array([TBL_HIGH_HUE, TBL_HIGH_SAT, TBL_HIGH_VAL]))
    hsv_mask = 255 - hsv_mask

    #print(hsv_mask)
    # Apply mask to original image
    hsv = cv2.bitwise_and(hsv, hsv, mask=hsv_mask)

    #plt.imshow(cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB))
    #plt.show()


    return hsv



def block_type_string(length, width):
    return str(width) + "X" + str(length)


def color_to_int(color):
    if color == "red":
        return COLOR_RED
    elif color == "green":
        return COLOR_GRN
    elif color == "blue":
        return COLOR_BLU

def find_ray_plane_intersection(ray):
    # Table plane  
    table_height = -0.1 # In baxter base reference frame

    table_normal = np.array([0, 0, 1])
    plane_point  = np.array([0, 0, -0.1])

# TODO: Need to improve this checking...
def calc_block_type(block_ratio):
    if(block_ratio <= 0.4):
        rospy.loginfo("Block ratio is very small so it's probably not a block..")
    if(block_ratio > 0.5 and block_ratio <= 1.5):
        block_type = (1, 1)

    elif(block_ratio > 1.5 and block_ratio <= 2.5):
        block_type = (2, 1)

    elif(block_ratio > 2.5 and block_ratio <= 3.5):
        block_type = (3, 1)

    elif(block_ratio > 3.5 and block_ratio <= 4.5):
        block_type = (4, 1)

    else:
        rospy.loginfo("BLOCK RATIO is %f, which doesn't fall into any of the defined ranges.. Setting to be a 1x1", block_ratio)
        block_type = (1, 1)

    return block_type

def generate_gripper_mask(hand_cam_image):
    pass

def calc_ratio(height, width):
    #rospy.loginfo("Height: %d, Width: %d", h, w)
    if(height > width):
        temp = height
        height = width
        width = temp

    rospy.loginfo("Height: %d, Width: %d", height, width)

    ratio = width / height

    return ratio


def calc_angle(cropped_image):
    # PCA
    pca_img = cropped_image.copy()

    print("Cropped image type: ", type(cropped_image))
    gray = cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY)
    
    if(gray is None):
        return 0.0

    print("GRay image type: ", type(gray))
    h, w = gray.shape

    #From a matrix of pixels to a matrix of coordinates of non-black points.
    #(note: mind the col/row order, pixels are accessed as [row, col]
    #but when we draw, it's (x, y), so have to swap here or there)
    mat = []
    for col in range(w):
        for row in range(h):
            if gray[row, col] > 0:
                mat.append([col, row])

    mat = np.array(mat).astype(np.float32) #have to convert type for PCA

    #mean (e. g. the geometrical center)
    #and eigenvectors (e. g. directions of principal components)
    m, e = cv2.PCACompute(mat, mean = None)

    #now to draw: let's scale our primary axis by 100,
    #and the secondary by 50
    center = tuple(m[0])
    endpoint1 = tuple(m[0] + e[0]*100)
    endpoint2 = tuple(m[0] + e[1]*50)
    
    """
    cv2.circle(pca_img, center, 5, 255)
    # Major Axis
    cv2.line(pca_img, center, endpoint1, 255, 4)

    # Minor Axis
    cv2.line(pca_img, center, endpoint2, 255, 4)

    plt.imshow(pca_img)
    plt.show()
    """

    # Calculate angle of major axis
    major_x = endpoint1[0] - center[0]
    major_y = endpoint1[1] - center[1]
    minor_x = endpoint2[0] - center[0]
    minor_y = endpoint2[1] - center[1]

    angle_rad_major = math.atan2(major_y, major_x)

    return angle_rad_major

def calc_center_angle_old(cont, cv_img):
    
    #img_shape = cv_img.shape

    #blank_img = np.zeros(img_shape, dtype=np.uint8)

    #cont_img = cv2.drawContours(cv_img, cont, 0, (0, 0, 255), 2)

    gray_img = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray_img, 50, 100, apertureSize = 3)


    #cv2.imshow("gray", gray_img)
    # TODO: optimize this by only performing operation on subsection of image
    lines = cv2.HoughLinesP(edges, rho=1, theta=0.017, threshold=30, minLineLength=2, maxLineGap=5) # From https://github.com/osrf/baxter_demos/blob/master/config/object_finder.yaml

    if lines is None:
        rospy.loginfo( "No lines found" )
        return None

    lines = lines.reshape(lines.shape[1], lines.shape[2])

    # Calculate the lengths of each line
    lengths = np.square(lines[:, 0]-lines[:, 2]) +\
            np.square(lines[:, 1]-lines[:, 3])
    lines = np.hstack((lines, lengths.reshape((lengths.shape[0], 1)) ))
    lines = lines[lines[:,4].argsort()]
    lines = lines[::-1] #Reverse the sorted array
    lines = lines[:, 0:4]

    longest_line = lines[0] #Get the longest line

    angle = math.atan2(longest_line[1] - longest_line[3], longest_line[0] - longest_line[2])

    rospy.loginfo("Angle: %f", angle * 180 / math.pi)


def create_block_marker(frame, id, position, orientation, length, width, block_color, transparency):
    curr_marker = Marker()
    curr_marker.header.frame_id = frame
    
    curr_marker.type = 1 # sphere
    curr_marker.action = 0
    curr_marker.id = id
    curr_marker.frame_locked = True
    curr_marker.pose.position = position
    curr_marker.pose.orientation = orientation
   
    single_unit_dim = 0.03

    curr_marker.scale.x = single_unit_dim
    curr_marker.scale.y = length*single_unit_dim 
    curr_marker.scale.z = single_unit_dim 

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
        rospy.logerr("Color %s doesn't have a supported marker yet! you should add one.", color)
    
    # Alpha value (transparency)
    curr_marker.color.a = transparency

    curr_marker.lifetime = rospy.Duration(0)

    return curr_marker



def create_ray_marker(frame, id, point1, point2, ray_color, transparency = 1):
    curr_marker = Marker()
    curr_marker.header.frame_id = frame
    
    curr_marker.type = 5 # line list
    curr_marker.action = 0
    curr_marker.id = id
    curr_marker.frame_locked = True
    #curr_marker.pose.position = position
    #curr_marker.pose.orientation = orientation

    if(ray_color == "red"):
        curr_marker.color.r = 1.0
        curr_marker.color.g = 0.0
        curr_marker.color.b = 0.0
    elif(ray_color == "blue"):
        curr_marker.color.r = 0.0
        curr_marker.color.g = 0.0
        curr_marker.color.b = 1.0
    elif(ray_color == "green"):
        curr_marker.color.r = 0.0
        curr_marker.color.g = 1.0
        curr_marker.color.b = 0.0
    elif(ray_color == "yellow"):
        curr_marker.color.r = 1.0
        curr_marker.color.g = 1.0
        curr_marker.color.b = 0.0
    elif(ray_color == "teal"):
        curr_marker.color.r = 0.0
        curr_marker.color.g = 0.2
        curr_marker.color.b = 1.0
    else:
        rospy.logerr("Color %s doesn't have a supported marker yet! you should add one.", color)
    curr_marker.scale.x = .01

    points_list = []
    #point1 = Point(x = 0, y = 0, z = 0)
    #point1 = Point(x = 0, y = 1, z = 0)

    position = Point(x = 0, y = 0, z = 0)
    orientation = Quaternion(w = 0, x = 0, y = 0, z = 0)

    curr_marker.pose.position = position
    curr_marker.pose.orientation = orientation


    points_list.append(point1)
    points_list.append(point2)
    curr_marker.points = points_list
    
    # Alpha value (transparency)
    curr_marker.color.a = transparency

    curr_marker.lifetime = rospy.Duration(0)

    return curr_marker

def main():
    camera_name = sys.argv[1]

    rospy.init_node('block_finder' + camera_name)

    block_finder = BlockFinder(camera_name)
    block_finder.subscribe()
    block_finder.publish()

    while not rospy.is_shutdown():
        if(camera_name == "top"):
            try:
                (trans,rot) = block_finder.tf_listener.lookupTransform("/base", "/camera_link", rospy.Time(0))
                block_finder.top_to_base_mat = tf.transformations.compose_matrix(translate = trans, angles=tf.transformations.euler_from_quaternion(rot))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                print("No TF from camera to base is available!")
        if(block_finder.detected_blocks > 0):
            rospy.loginfo("Publishing block location markers")

            rospy.loginfo("There are %d block markers", len(block_finder.block_markers.markers))
            block_finder.marker_pub.publish(block_finder.block_markers)

            # Publish ray from camera lens to detected object
            rospy.loginfo("There are %d ray markers", len(block_finder.ray_markers.markers))
            block_finder.ray_marker_pub.publish(block_finder.ray_markers)


            rospy.loginfo("Publishing block observations")
            block_obs_array = BlockObservationArray()
            block_obs_array.inv_obs = block_finder.block_obs

            block_finder.block_obs_pub.publish(block_obs_array)

            block_pixel_locs_array = BlockPixelLocArray()
            block_pixel_locs_array.pixel_locs = block_finder.block_pixel_locs
            block_finder.pixel_loc_pub.publish(block_pixel_locs_array)
    
        block_finder.rect_seg_img_pub.publish(block_finder.bridge.cv2_to_imgmsg(block_finder.rect_seg_img))

        block_finder.red_seg_img_pub.publish(block_finder.bridge.cv2_to_imgmsg(block_finder.seg_img["red"]))

        block_finder.green_seg_img_pub.publish(block_finder.bridge.cv2_to_imgmsg(block_finder.seg_img["green"]))

        block_finder.yellow_seg_img_pub.publish(block_finder.bridge.cv2_to_imgmsg(block_finder.seg_img["yellow"]))

        block_finder.blue_seg_img_pub.publish(block_finder.bridge.cv2_to_imgmsg(block_finder.seg_img["blue"]))

        # Sleep
        block_finder.pub_rate.sleep()

    return 

if __name__ == '__main__':
     main()
