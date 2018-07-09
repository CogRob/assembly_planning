#!/usr/bin/env python
import rospy
import cv2
import numpy as np

import argparse
import struct
import sys
import copy
import rospkg
import math

from matplotlib import pyplot as plt

from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, CameraInfo, Range
from geometry_msgs.msg import Pose, PoseArray, Point, Quaternion

from image_geometry import PinholeCameraModel

from cv_bridge import CvBridge
import baxter_interface

import tf

REAL = False
SIM  = True

if(REAL):
    BLU_LOW_HUE     = 105
    BLU_HIGH_HUE    = 120
    BLU_LOW_SAT     = 105
    BLU_HIGH_SAT    = 120
    BLU_LOW_VAL     = 105
    BLU_HIGH_VAL    = 120

    GRN_LOW_HUE     = 30
    GRN_HIGH_HUE    = 80
    GRN_LOW_SAT     = 200
    GRN_HIGH_SAT    = 255
    GRN_LOW_VAL     = 200
    GRN_HIGH_VAL    = 255

    RED_LOW_HUE     = 0
    RED_HIGH_HUE    = 10
    RED_LOW_SAT     = 0
    RED_HIGH_SAT    = 255
    RED_LOW_VAL     = 0
    RED_HIGH_VAL    = 255

    YEL_LOW_HUE     = 10
    YEL_HIGH_HUE    = 30
    YEL_LOW_SAT     = 68
    YEL_HIGH_SAT    = 147
    YEL_LOW_VAL     = 30
    YEL_HIGH_VAL    = 200

if(SIM):
    BLU_LOW_HUE     = 90
    BLU_HIGH_HUE    = 130
    BLU_LOW_SAT     = 0
    BLU_HIGH_SAT    = 255
    BLU_LOW_VAL     = 0
    BLU_HIGH_VAL    = 255

    GRN_LOW_HUE     = 30
    GRN_HIGH_HUE    = 80
    GRN_LOW_SAT     = 200
    GRN_HIGH_SAT    = 255
    GRN_LOW_VAL     = 200
    GRN_HIGH_VAL    = 255

    RED_LOW_HUE     = 0
    RED_HIGH_HUE    = 1
    RED_LOW_SAT     = 235
    RED_HIGH_SAT    = 255
    RED_LOW_VAL     = 0
    RED_HIGH_VAL    = 255

    YEL_LOW_HUE     = 10
    YEL_HIGH_HUE    = 30
    YEL_LOW_SAT     = 68
    YEL_HIGH_SAT    = 147
    YEL_LOW_VAL     = 30
    YEL_HIGH_VAL    = 200

colors = {
#        "red":      {   "low_h": RED_LOW_HUE,   "high_h": RED_HIGH_HUE,
#                        "low_s": RED_LOW_SAT,   "high_s": RED_HIGH_SAT,
#                        "low_v": RED_LOW_VAL,   "high_v": RED_HIGH_VAL  },
#        "yellow":   {   "low_h": YEL_LOW_HUE,   "high_h": YEL_HIGH_HUE,
#                        "low_s": YEL_LOW_SAT,   "high_s": YEL_HIGH_SAT,
#                        "low_v": YEL_LOW_VAL,   "high_v": YEL_HIGH_VAL  },
#        "green":    {   "low_h": GRN_LOW_HUE,   "high_h": GRN_HIGH_HUE,
#                        "low_s": GRN_LOW_SAT,   "high_s": GRN_HIGH_SAT,
#                        "low_v": GRN_LOW_VAL,   "high_v": GRN_HIGH_VAL  },
        "blue":     {   "low_h": BLU_LOW_HUE,   "high_h": BLU_HIGH_HUE,
                       "low_s": BLU_LOW_SAT,   "high_s": BLU_HIGH_SAT,
                       "low_v": BLU_LOW_VAL,   "high_v": BLU_HIGH_VAL  }
}

class BlockFinder():
    def __init__(self, limb):
        self.limb = limb
        self.block_poses = []
        self.tf_listener = tf.TransformListener()
        self.camera_model = None
        self.min_ir_depth = 0.1
        self.object_height = 0.05
        self.seg_img = None
        self.bridge = CvBridge()
        self.ir_reading = None
        
        self.pub_rate = rospy.Rate(10)
        
    def publish(self):
        self.pose_pub    = rospy.Publisher("block_finder/" + self.limb + "/block_poses", PoseArray, queue_size=10)
        self.seg_img_pub = rospy.Publisher("block_finder/" + self.limb + "/segmented_image", Image, queue_size=10)

    def subscribe(self):
        self.cam_sub    = rospy.Subscriber("/cameras/" + self.limb + "_camera/image", Image, self.cam_callback)
        self.info_sub   = rospy.Subscriber("/cameras/" + self.limb + "_camera/camera_info", CameraInfo, self.info_callback)
        self.ir_sub     = rospy.Subscriber("/robot/range/" + self.limb + "_range/state", Range, self.ir_callback)

   
    '''
    Thresholds camera image and stores object centroid location (x,y) in Baxter's base frame.
    '''
    def cam_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        #find_hsv_values(cv_image)

        height, width, depth = cv_image.shape
        low_s = 0
        low_v = 0

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        images = [hsv, hsv, hsv, hsv, hsv]
        i = 0

        for key in colors:
            low_h = colors[key]["low_h"]
            high_h = colors[key]["high_h"]
            low_s = colors[key]["low_s"]
            high_s = colors[key]["high_s"]
            low_v = colors[key]["low_v"]
            high_v = colors[key]["high_v"]

            #Converting image to HSV format
            curr_image = cv2.inRange(hsv, np.array([low_h, low_s, low_v]), np.array([high_h, high_s, high_v]))

            #Morphological opening (remove small objects from the foreground)
            curr_image = cv2.erode(curr_image, np.ones((5,5), np.uint8), iterations=1)
            curr_image = cv2.dilate(curr_image, np.ones((5,5), np.uint8), iterations=1)

            #Morphological closing (fill small holes in the foreground)
            curr_image = cv2.dilate(curr_image, np.ones((5,5), np.uint8), iterations=1)
            curr_image = cv2.erode(curr_image, np.ones((5,5), np.uint8), iterations=1)

            images[i] = curr_image.copy()
            
            ret, thresh = cv2.threshold(curr_image,157,255,0)

            contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)


            #Draw the countours.
            #cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
            for c in contours:
                rect = cv2.minAreaRect(c)
                box = cv2.cv.BoxPoints(rect)
                box = np.int0(box)
                cv2.drawContours(cv_image, [box], 0, (0, 0, 255), 2)

                angle = calc_center_angle(c, cv_image)

                #cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)

            self.seg_img = cv_image.copy()

            numobj = len(contours) # number of objects found in current frame
            rospy.loginfo("There are %d objects", numobj)
            
            if numobj > 0:
                moms = cv2.moments(contours[0])
                #print(moms)
                if moms['m00']>500:
                    cx = int(moms['m10']/moms['m00'])
                    cy = int(moms['m01']/moms['m00'])

                    print 'cx = ', cx
                    print 'cy = ', cy

                    # Shift to center of image
                    #cx = cx - self.camera_model.cx()
                    #cy = cy - self.camera_model.cy()

                    print "Found ", numobj,  " ", key, "object(s)"
                    obj_found = True

                    cam_info = rospy.wait_for_message("/cameras/"+ self.limb + "_camera" + "/camera_info", CameraInfo, timeout=None)
                    
                    vec = np.array(self.camera_model.projectPixelTo3dRay((cx, cy)))
                    if(self.ir_reading != None):
                        rospy.loginfo("IR reading is valid!")
                        d = (self.ir_reading - self.object_height)

                        print("Distance to object: ", d)
                        d_cam = d * vec

                        homog_d_cam = np.concatenate((d_cam, np.ones(1))).reshape((4,1))

                        # Wait for transformation from base to head_camera
                        self.tf_listener.waitForTransform('/world', self.limb + "_camera", rospy.Time(), rospy.Duration(4))
                        (trans, rot) = self.tf_listener.lookupTransform('/world', self.limb + "_camera", rospy.Time())

                        camera_to_base = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))

                        block_position_arr = np.dot(camera_to_base, homog_d_cam)
                        
                        block_position_p = Point()
                        block_position_p.x = block_position_arr[0]
                        block_position_p.y = block_position_arr[1]
                        block_position_p.z = block_position_arr[2]
                        
                        
                        # TODO: Need to calculate this somehow still
                        block_orientation = Quaternion()
                        block_orientation.w = 0
                        block_orientation.x = 0
                        block_orientation.y = 0
                        block_orientation.z = 0

                        rospy.loginfo("Adding new block pose!")
                        self.block_poses.append(Pose(position=block_position_p, orientation=block_orientation))

                    else:
                        rospy.loginfo("No ir_data has been recieved yet!")
        """
        i = 0
        for key in colors:
            cv2.imshow("Original", cv_image)
            cv2.imshow(key, images[i])

            cv2.waitKey()

            i += 1
        """

    def info_callback(self, data):
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.info_sub.unregister() # Unsubscribe after receiving CameraInfo first time

    def ir_callback(self, data):
        self.ir_reading = data.range
        #print("IR reading: ", self.ir_reading)
        if(self.ir_reading > 60):
            #rospy.loginfo("Invalid IR reading")
            self.ir_reading = 0.4

def calc_center_angle(cont, cv_image):
    #img_shape = cv_image.shape

    #blank_img = np.zeros(img_shape, dtype=np.uint8)

    cont_img = cv2.drawContours(cv_image, cont, 0, (0, 0, 255), 2)

    gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
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


    print("ANGLE: ", angle * 180 / math.pi)

def main():
    rospy.init_node('block_finder')

    block_finder = BlockFinder("left_hand")
    block_finder.subscribe()
    block_finder.publish()


    while not rospy.is_shutdown():
        print(block_finder.block_poses)
        if(len(block_finder.block_poses) > 0):
            print("Publishing block location")
            pose_msg = PoseArray()
            pose_msg.poses = block_finder.block_poses
            block_finder.pose_pub.publish(pose_msg)
            block_finder.seg_img_pub.publish(block_finder.bridge.cv2_to_imgmsg(block_finder.seg_img, "bgr8"))
        block_finder.pub_rate.sleep()

    return 

if __name__ == '__main__':
     main()
