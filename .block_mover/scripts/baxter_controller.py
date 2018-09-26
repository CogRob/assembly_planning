#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import *
from gazebo_msgs.msg import *
from geometry_msgs.msg import *
from sensor_msgs.msg import Image, CameraInfo
import random
import math
import tf
import numpy as np
import cv2
from skimage import feature
import pickle
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm
import collections
from cv_bridge import CvBridge

REAL = True
USE_HSV = True
bbox_buffer = 10

if(REAL):
    BLU_LOW_HUE     = 105
    BLU_HIGH_HUE    = 120
    BLU_LOW_SAT     = 0
    BLU_HIGH_SAT    = 255
    BLU_LOW_VAL     = 0
    BLU_HIGH_VAL    = 100

    GRN_LOW_HUE     = 50
    GRN_HIGH_HUE    = 80
    GRN_LOW_SAT     = 100
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
    RED_LOW_SAT     = 0
    RED_HIGH_SAT    = 255
    RED_LOW_VAL     = 0
    RED_HIGH_VAL    = 100

    YEL_LOW_HUE     = 14
    YEL_HIGH_HUE    = 24
    YEL_LOW_SAT     = 0
    YEL_HIGH_SAT    = 255
    YEL_LOW_VAL     = 0
    YEL_HIGH_VAL    = 255

colors = {

        #"red":      {   "low_h": [RED_LOW_HUE_1, RED_LOW_HUE_2],   "high_h": [RED_HIGH_HUE_1, RED_HIGH_HUE_2],
        #                "low_s": RED_LOW_SAT,   "high_s": RED_HIGH_SAT,
        #                "low_v": RED_LOW_VAL,   "high_v": RED_HIGH_VAL  },
        #"yellow":   {   "low_h": YEL_LOW_HUE,   "high_h": YEL_HIGH_HUE,
        #                "low_s": YEL_LOW_SAT,   "high_s": YEL_HIGH_SAT,
        #                "low_v": YEL_LOW_VAL,   "high_v": YEL_HIGH_VAL  },
        "green":    {   "low_h": GRN_LOW_HUE,   "high_h": GRN_HIGH_HUE,
                        "low_s": GRN_LOW_SAT,   "high_s": GRN_HIGH_SAT,
                        "low_v": GRN_LOW_VAL,   "high_v": GRN_HIGH_VAL  },
        #"blue":     {   "low_h": BLU_LOW_HUE,   "high_h": BLU_HIGH_HUE,
        #                "low_s": BLU_LOW_SAT,   "high_s": BLU_HIGH_SAT,
        #                "low_v": BLU_LOW_VAL,   "high_v": BLU_HIGH_VAL  },
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

class CameraController():
    def __init__(self):
        self.curr_pose = None
        self.limb = "right"
        self.res_w = 1920
        self.res_h = 1080
        self.center_loc = np.array([self.res_w/2, self.res_h/2]) # Location of camera center (in pixels)
        self.pixel_loc = np.array([self.res_w/2, self.res_h/2]) # Location of blob center (in pixels)

        self.orig_img = np.zeros((self.res_w, self.res_h,3), dtype=np.uint8)
        self.seg_img = np.zeros((self.res_w, self.res_h,3), dtype=np.uint8)
        self.canny_img = np.zeros((self.res_w, self.res_h,1), dtype=np.uint8)
        self.pca_img = np.zeros((self.res_w, self.res_h,3), dtype=np.uint8)
        self.hough_img = np.zeros((self.res_w, self.res_h,3), dtype=np.uint8)

        self.bridge = CvBridge()
        self.camera_centered = False
        self.blob_detected = False

        self.blob_major_angle = None
        self.blob_minor_angle = None
        self.major_len = 1
        self.minor_len = 1
        self.contour_area = -1
        self.bounding_area = -1

    def publish(self):
        self.orig_img_pub = rospy.Publisher("cam_controller/orig_img", Image, queue_size=1)
        self.seg_img_pub = rospy.Publisher("cam_controller/seg_img", Image, queue_size=1)
        self.canny_img_pub = rospy.Publisher("cam_controller/canny_img", Image, queue_size=1)
        self.pca_img_pub = rospy.Publisher("cam_controller/pca_img", Image, queue_size=1)
        self.hough_img_pub = rospy.Publisher("cam_controller/hough_img", Image, queue_size=1)

    def subscribe(self):
        self.cam_sub    = rospy.Subscriber("/free_camera/image_raw", Image, self.cam_callback)
        #self.info_sub   = rospy.Subscriber("/cameras/" + self.limb + "_camera/camera_info", CameraInfo, self.info_callback)

    def get_camera_pose(self):
        rospy.wait_for_service('/gazebo/get_model_state')
        get_model_state = rospy.ServiceProxy('gazebo/get_model_state', GetModelState)
        self.curr_state = get_model_state('camera_robot', 'world')

    def reset_camera_pose(self):
        zero_pose = Pose()
        zero_pose.position.z = 1
    
        zero_pose.orientation.w = 0.1
        zero_pose.orientation.x = 0.1
        zero_pose.orientation.y = 0.01
        zero_pose.orientation.z = 0.01


        self.set_camera_pose(zero_pose)


    def set_camera_pose(self, new_pose):
        rospy.wait_for_service('/gazebo/set_model_state')
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = 'camera_robot'
        model_state.pose.position.x = new_pose.position.x
        model_state.pose.position.y = new_pose.position.y
        model_state.pose.position.z = new_pose.position.z
        model_state.pose.orientation.x = new_pose.orientation.x
        model_state.pose.orientation.y = new_pose.orientation.y
        model_state.pose.orientation.z = new_pose.orientation.z
        model_state.pose.orientation.w = new_pose.orientation.w

        model_state.reference_frame = 'world'
        set_model_state(model_state)

        self.model_state = model_state

        #rospy.loginfo("Camera position set to %f, %f.", new_pose.position.x, new_pose.position.y)

    def move_camera_in_plane(self, direction, motion_dist=0.005):
        if(direction == None):
            return

        rospy.loginfo("Moving camera in plane in direction %f ", math.degrees(direction))

        # Orientation should remain constant, only position should change
        curr_x = self.model_state.pose.position.x
        curr_y = self.model_state.pose.position.y
        curr_z = self.model_state.pose.position.z

        cam_trans_x = motion_dist * math.cos(direction)
        cam_trans_y = motion_dist * math.sin(direction)

        trans_vec = np.array([0, -cam_trans_x, cam_trans_y,  1])

        # Rotation matrix from camera to world coordinates
        curr_orient = self.model_state.pose.orientation

        curr_quat = np.array([curr_orient.x, curr_orient.y, curr_orient.z, curr_orient.w])

        euler = tf.transformations.euler_from_quaternion(curr_quat)

        R = tf.transformations.quaternion_matrix(curr_quat)

        # Translate camera vector to world coordinates
        new_coord = np.dot(R, trans_vec)

        new_pose = Pose()
        new_pose.position.x = curr_x + new_coord[0]
        new_pose.position.y = curr_y + new_coord[1]
        new_pose.position.z = curr_z + new_coord[2]
        new_pose.orientation = self.state.pose.orientation

        self.set_camera_pose(new_pose)

    def camera_yaw(self, amount):
        new_pose = Pose()
        new_pose.position = self.model_state.pose.position # Pose should remain constant

        # Only orientation changes
        curr_orient = self.model_state.pose.orientation
        curr_orient_arr = np.array([curr_orient.x, curr_orient.y, curr_orient.z, curr_orient.w])

        # Rotation about z-axis by small amount
        yaw = tf.transformations.quaternion_from_euler(0, 0, amount)

        q_new = tf.transformations.quaternion_multiply(curr_orient_arr, yaw)

        new_pose.orientation = Quaternion()
        new_pose.orientation.x = q_new[0]
        new_pose.orientation.y = q_new[1]
        new_pose.orientation.z = q_new[2]
        new_pose.orientation.w = q_new[3]

        self.set_camera_pose(new_pose)

    def camera_pitch(self, amount):
        new_pose = Pose()
        new_pose.position = self.model_state.pose.position # Pose should remain constant

        # Only orientation changes
        curr_orient = self.model_state.pose.orientation
        curr_orient_arr = np.array([curr_orient.x, curr_orient.y, curr_orient.z, curr_orient.w])

        # Rotation about y-axis
        pitch = tf.transformations.quaternion_from_euler(0, amount, 0)

        q_new = tf.transformations.quaternion_multiply(curr_orient_arr, pitch)

        new_pose.orientation = Quaternion()
        new_pose.orientation.x = q_new[0]
        new_pose.orientation.y = q_new[1]
        new_pose.orientation.z = q_new[2]
        new_pose.orientation.w = q_new[3]

        self.set_camera_pose(new_pose)


    def camera_roll(self, amount):
        new_pose = Pose()
        new_pose.position = self.model_state.pose.position # Pose should remain constant

        # Only orientation changes
        curr_orient = self.model_state.pose.orientation
        curr_orient_arr = np.array([curr_orient.x, curr_orient.y, curr_orient.z, curr_orient.w])

        # Rotation about y-axis
        pitch = tf.transformations.quaternion_from_euler(amount, 0, 0)

        q_new = tf.transformations.quaternion_multiply(curr_orient_arr, pitch)

        new_pose.orientation = Quaternion()
        new_pose.orientation.x = q_new[0]
        new_pose.orientation.y = q_new[1]
        new_pose.orientation.z = q_new[2]
        new_pose.orientation.w = q_new[3]

        self.set_camera_pose(new_pose)



    def center_on_pixel(self):
        #rospy.loginfo("Centering camera on pixel_loc: (%f, %f)", self.pixel_loc[0], self.pixel_loc[1])

        # PID loop to center on pixel location
        center_thresh = 15.0 # Attempt to get the camera centered within 5 pixels in x and y axes
        x_centered = False
        y_centered = False

        # Calculate how far off 
        
        x_offset = self.res_w/2 - self.pixel_loc[0] 
        #rospy.loginfo("Object is off by %f pixels in x", x_offset)

        if(math.fabs(x_offset) > center_thresh):
            #rospy.loginfo("Yawing camera")
            self.camera_yaw(x_offset * math.pi / 80000)
        else:
            #rospy.loginfo("X offset of %f is within threshold %f", x_offset, center_thresh)
            x_centered = True
        
        y_offset = self.res_h/2 - self.pixel_loc[1] 
        #rospy.loginfo("Object is off by %f pixels in y", y_offset)
        
        if(math.fabs(y_offset) > center_thresh):
            #rospy.loginfo("Pitching camera")
            self.camera_pitch(-y_offset * math.pi / 80000)
        else:
            #rospy.loginfo("Y offset of %f is within threshold %f", y_offset, center_thresh)
            y_centered = True


        if(x_centered == True and y_centered == True):
            self.camera_centered = True
            self.pixel_loc = np.zeros(2)

    def find_object_axes(self, img):
       pass 



    '''
    Thresholds camera image and stores object centroid location (x,y) in Baxter's base frame.
    '''
    def cam_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.orig_img = img

        height, width, depth = img.shape
        low_s = 0
        low_v = 0

        # Converting image to HSV format
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        color = "green"

        low_h = colors[color]["low_h"]
        high_h = colors[color]["high_h"]
        low_s = colors[color]["low_s"]
        high_s = colors[color]["high_s"]
        low_v = colors[color]["low_v"]
        high_v = colors[color]["high_v"]

        if color == "red":
            hsv_mask_1 = cv2.inRange(hsv_img, np.array([low_h[0], low_s, low_v]), np.array([high_h[0], high_s, high_v]))
            hsv_mask_2 = cv2.inRange(hsv_img, np.array([low_h[1], low_s, low_v]), np.array([high_h[1], high_s, high_v]))

            hsv_mask = hsv_mask_1 | hsv_mask_2

        else:
            hsv_mask = cv2.inRange(hsv_img, np.array([low_h, low_s, low_v]), np.array([high_h, high_s, high_v]))
        
        # Apply mask to original image
        masked_img = cv2.bitwise_and(img, img, mask=hsv_mask)

        self.seg_img = masked_img

        #Morphological opening (remove small objects from the foreground)
        erode_1 = cv2.erode(hsv_mask, np.ones((5,5), np.uint8), iterations=1)
        dilate_1 = cv2.dilate(erode_1, np.ones((5,5), np.uint8), iterations=1)

        #Morphological closing (fill small holes in the foreground)
        dilate_2 = cv2.dilate(dilate_1, np.ones((10,10), np.uint8), iterations=1)
        erode_2 = cv2.erode(dilate_2, np.ones((10,10), np.uint8), iterations=1)

        ret, thresh = cv2.threshold(erode_1,157,255,0)

        im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

        # Draw the countours.
        # cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
        large_contours = []

        rect_img = img.copy()
        min_area_mask = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        min_area_mask.fill(0)

        contour_area_thresh = 10
        
        self.blob_detected = True

        if(len(contours) > 0):
            areas = map(lambda c: cv2.contourArea(c), contours)
            self.contour_area = sorted(areas)[-1]
            
            #print("There are ", len(contours), "contours")
            for c in contours:
                area = cv2.contourArea(c)
                (x, y), (MA, ma), angle = cv2.fitEllipse(c)

                major = max(MA, ma)
                minor = min(MA, ma)

                if(area > contour_area_thresh):
                    self.contour_area = area
                    large_contours.append(c)

                    # Min Area Rectangle
                    rect = cv2.minAreaRect(c)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(min_area_mask, [box], 0, (255,255,255) , -1)
                    masked_img = cv2.bitwise_and(img, img, mask=min_area_mask)

                    # Bounding Rectangle
                    x, y, w, h = cv2.boundingRect(c)
                    
                    self.bounding_area = w*h
                    cv2.rectangle(rect_img, (x, y), (x+w, y+h), color_vals[color], 2)

                    img = img[y-bbox_buffer:y+h+bbox_buffer, x-bbox_buffer:x+w+bbox_buffer]

                    # TODO: To optimize, crop the eroded image...
                    erode_cropped = erode_1[y-bbox_buffer:y+h+bbox_buffer, x-bbox_buffer:x+w+bbox_buffer]
                    cropped_w = y
                    cropped_h = x

                    break
                      
                else:
                    rospy.loginfo("No contour with area larger than %d was found.", contour_area_thresh)
                    self.blob_detected = False
                    return


            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Canny Edge Detectionz
            canny_min_thresh = 10
            canny_max_thresh = 300
            canny_img = cv2.Canny(gray, canny_min_thresh, canny_max_thresh, apertureSize = 5)
            canny_img = 255 - canny_img
            """
            #canny_img = np.logical_not(canny_img)
            gray_blur = cv2.medianBlur(gray, 7)
            canny_img = feature.canny(gray_blur, sigma=1)
            canny_img = canny_img.astype(np.uint8)
            """


            #canny_img = 255 - 255*canny_img

            canny_img = cv2.dilate(canny_img, np.ones((1,1), np.uint8), iterations=1)
            canny_img = cv2.erode(canny_img, np.ones((1,1), np.uint8), iterations=1)

            self.canny_img = canny_img

            # PCA
            pca_img = img.copy()
            h, w = gray.shape

            #From a matrix of pixels to a matrix of coordinates of non-black points.
            #(note: mind the col/row order, pixels are accessed as [row, col]
            #but when we draw, it's (x, y), so have to swap here or there)
            mat = []
            for col in range(w):
                for row in range(h):
                    if erode_cropped[row, col] > 0:
                        #print(img[row,col])
                        mat.append([col, row])

            self.canny_img = erode_cropped

            mat = np.array(mat).astype(np.float32) #have to convert type for PCA

            #mean (e. g. the geometrical center)
                #and eigenvectors (e. g. directions of principal components)
            m, e = cv2.PCACompute(mat, mean = None)

            #now to draw: let's scale our primary axis by 100,
            #and the secondary by 50
            center = tuple(m[0])
            endpoint1 = tuple(m[0] + e[0]*major/2)
            endpoint2 = tuple(m[0] + e[1]*minor/2)

            major_len = np.math.sqrt((m[0][1] - e[0][1])**2 + (m[0][0] - e[0][0])**2)
            minor_len = np.math.sqrt((m[0][1] - e[1][1])**2 + (m[0][0] - e[1][0])**2)

            # Save major and minor lengths to optimize over
            self.major_len = major
            self.minor_len = minor

            cv2.circle(pca_img, center, 5, 255)
            # Major Axis
            cv2.line(pca_img, center, endpoint1, 255, 1)

            # Minor Axis
            cv2.line(pca_img, center, endpoint2, 255, 1)

            self.pca_img = pca_img

            # Calculate angle of major axis
            major_x = endpoint1[0] - center[0]
            major_y = endpoint1[1] - center[1]
            minor_x = endpoint2[0] - center[0]
            minor_y = endpoint2[1] - center[1]

            #rospy.loginfo("x: %d, y: %d   center_x: %d, center_y: %d", x, y, center[0], center[1])
            img_center = center + np.array([x,y])
            self.pixel_loc = img_center

            angle_rad_major = math.atan2(major_y, major_x)
            angle_rad_minor = math.atan2(minor_y, minor_x)

            self.blob_major_angle = 180 - (angle_rad_major * 180 / np.pi) % 180
            self.blob_minor_angle = 180 - (angle_rad_minor * 180 / np.pi) % 180

            # Hough Transform
            hough_img = img.copy()

            """
            # Standard Hough
            lines = cv2.HoughLines(canny_img,2,np.pi/720,20)
            angles = []

            if(lines is not None):
                for rho, theta in lines[0][:100]:
                    
                    angles.append(theta)
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    #print("Image ", i, " has theta ", theta, " and rho ", rho)
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 - 1000*(a))
                    #if(math.fabs(theta - angle_rad_major) < math.pi/132):
                    cv2.line(hough_img,(x1,y1),(x2,y2),(0,0,0),1)
                    

                sorted_angles = sorted(angles)
            """

            # Probabilistic Hough
            minLineLength = 50
            maxLineGap = 10
            lines = cv2.HoughLinesP(canny_img,1,np.pi/180,10,maxLineGap,minLineLength)
            hough_p_img = img.copy()
            if(lines is not None):
                for x1,y1,x2,y2 in lines[0]:
                    cv2.line(hough_p_img,(x1,y1),(x2,y2),(0,0,0),1)
            self.hough_img = hough_p_img

            """
            harris_img = gray.copy()
            # Find Corners
            dst = cv2.cornerHarris(harris_img,2,3,0.04)

            #result is dilated for marking the corners, not important
            dst = cv2.dilate(dst,None)

            # Threshold for an optimal value, it may vary depending on the image.
            harris_img[dst>0.001*dst.max()]=0

            brisk = cv2.BRISK()
            kp_brisk = brisk.detect(gray)
            brisk_img = img.copy()

            brisk_img = cv2.drawKeypoints(img, kp_brisk, brisk_img)
            """



                
            """
            plt.subplot("231")
            plt.imshow(rect_img,cmap='gray')
            plt.title("HSV Thresholded Image")
            plt.subplot("232")
            plt.imshow(canny_img,cmap='gray')
            plt.title("Canny Image")
            plt.subplot("233")
            plt.imshow(harris_img,cmap='gray')
            plt.title("Harris Image")
            plt.subplot("234")
            plt.imshow(hough_img,cmap='gray'
            plt.title("Hough Image")
            plt.subplot("235")
            plt.imshow(hough_p_img,cmap='gray')
            plt.title("Probabilistic Hough Image")
            plt.subplot("236")
            plt.imshow(brisk_img,cmap='gray')
            plt.title("Brisk Image")
            plt.suptitle("Image " + str(i))
            plt.show()
            """
            
            """
            
            cv2.imshow("Axes", pca_img)

            cv2.waitKey(1)
            """
            """
            #plt.subplot(6,10,i + 1)
            plt.subplot(6,1,1)
            plt.imshow(orig_img, cmap='gray')
            plt.title(str(angle))


            #plt.subplot(6, 10, i+11)
            plt.subplot(6, 1, 2)
            plt.imshow(erode_cropped, cmap='gray')
            plt.title(str(angle))

            #plt.subplot(6, 10, i+21)
            plt.subplot(6, 1, 3)
            plt.imshow(canny_img, cmap='gray')
            plt.title(str(angle))

            #plt.subplot(6, 10, i+31)
            plt.subplot(6, 1, 4)
            plt.imshow(pca_img, cmap='gray')
            title = r'${:.0f}\degree%'.format(angle)
            plt.title(title)
            
            #plt.subplot(6, 10, i+41)
            plt.subplot(6, 1, 5)
            plt.imshow(hough_img, cmap='gray')
            plt.title(str(angle))

            plt.show()
            """

        else:
            self.blob_detected = False
            rospy.loginfo("No contours were found in the image.")

def info_callback(self):
    #self.camera_model = PinholeCameraModel()
    #self.camera_model.fromCameraInfo(data)
    self.info_sub.unregister() # Unsubscribe after receiving CameraInfo first time


def load_and_plot():
    #images = pickle.load(open("images.p", "rb"))
    poses = pickle.load(open("poses.p", "rb"))
    #angles = pickle.load(open("angles.p", "rb"))
    major_lens = pickle.load(open("major_lens.p", "rb"))
    minor_lens = pickle.load(open("minor_lens.p", "rb"))
    areas = pickle.load(open("areas.p", "rb"))

    plot_results(poses, major_lens, minor_lens, areas)

def plot_results(poses, major_lens, minor_lens, areas, angles):
    x = []
    y = []
    z = []
    z_1 = []
    z_2 = []
    distances = []
    area_ratio = []
    bb_areas = []
    rect_areas = []

    for areas_list in areas:
        area_ratio.append(areas_list[1] /areas_list[0])
        bb_areas.append(areas_list[1])
        rect_areas.append(areas_list[0])
    for pose in poses:
        x.append(pose.position.x)
        y.append(pose.position.y)
        z.append(pose.position.z)
        distances.append(math.sqrt((math.pow(pose.position.x, 2) + math.pow(pose.position.y, 2) + math.pow(pose.position.z, 2))))

    for major_len in major_lens:
        z_1.append(major_len)
        rospy.loginfo(major_len)

    for minor_len in minor_lens:
        z_2.append(minor_len)
        rospy.loginfo(minor_len)

    z_3 = [a / b for a, b in zip(z_1, z_2)]
    z_4 = [a / b for a, b in zip(z_2, z_1)]
    #plt.scatter(distances, areas)
    #plt.show()
    
    fig1 = plt.figure("Major")
    ax1 = fig1.add_subplot(111, projection='3d')
    ax1.scatter(x, y, z_1, c='b', marker = 'o')
    #ax1.set_zlim(2000, 2500)
    plt.show()

    fig2 = plt.figure("Minor")
    ax2 = fig2.add_subplot(111, projection='3d')
    ax2.scatter(x, y, z_2, c='r', marker = '^')
    plt.show()

    fig3 = plt.figure("Major / Minor")
    ax3 = fig3.add_subplot(111, projection='3d')
    ax3.scatter(x, y, bb_areas, c='r', marker = '^')
    plt.show()

    fig4 = plt.figure("Minor / Major")
    ax4 = fig4.add_subplot(111, projection='3d')
    ax4.scatter(x, y, z_3, c='r', marker = '^')
    plt.show()
    
    fig5 = plt.figure("Area Ratio")
    ax5 = fig5.add_subplot(111, projection='3d')
    ax5.scatter(x, y, area_ratio, c='r', marker = '^')
    plt.show()
    
    """
    fig6 = plt.figure("Angles")
    ax6 = fig6.add_subplot(111, projection='3d')
    ax6.scatter(x, y, angles, c='r', marker = '^')
    plt.show()
    """
    fig7 = plt.figure("Bounding Box Area")
    ax7 = fig7.add_subplot(111, projection='3d')
    ax7.scatter(x, y, bb_areas, c='r', marker = '^')
    plt.show()
    
    fig8 = plt.figure("Rectangle Area")
    ax8 = fig8.add_subplot(111, projection='3d')
    ax8.scatter(x, y, rect_areas, c='r', marker = '^')
    plt.show()
    fig9 = plt.figure("Path")
    ax9 = fig9.add_subplot(111, projection='3d')
    ax9.scatter(x, y, z, c='r', marker = '^')
    plt.show()
    """

    for angle in angles:
        angle = angle%90


    # Sort all data by angles
    sorted_idxs = np.argsort(angles)

    for idx in sorted_idxs[::-1]:
        curr_img = images[idx]
        curr_pose = poses[idx]
        curr_angle = angles[idx]
        print("Current angle: ", curr_angle)
        print("Current_pose: ", curr_pose)
        plt.imshow(curr_img)
        plt.show()
    """

    
def main():

    run_camera_controller = True

    if(run_camera_controller): 
        rospy.init_node('camera_controller')
        cam_ctrl = CameraController()

        cam_ctrl.reset_camera_pose()
        cam_ctrl.publish()
        cam_ctrl.subscribe()
        cam_ctrl.subscribe()
        radius = 1
        divisor = 16
        curr_step = 0


        poses = []
        images = []
        angles = []
        major_lens = []
        minor_lens = []
        areas = []
        i = 0
        sample_count = 100

        # Move to random location
        angle = curr_step * 2 * math.pi / divisor
        curr_step += 1

        cam_ctrl.get_camera_pose()

        curr_pose = cam_ctrl.model_state.pose

        new_pose = Pose()

        x = random.uniform(-.4, .4)
        #x = .75
        #y = 0
        y = random.uniform(-.4, .4)
            
        #z = random.uniform(0.2, .8)
        #z = 1
        z = math.sqrt(.5**2 - x**2 - y**2)

        if(i == sample_count - 1):
            x = 0
            y = 0
            z = .25

        """
        x = 0
        y = 0
        z = random.uniform(0.2, .8)
        """

        origin = Point(x=0, y=0, z=0) 

        new_pose.position.x = origin.x + x
        new_pose.position.y = origin.y + y
        new_pose.position.z = origin.z + z

        #rospy.loginfo("x: %f, y: %f, z: %f", x, y, z)
        # point towards center
        new_pose_q_arr = tf.transformations.quaternion_from_euler(0, math.atan2(z, math.sqrt(x*x + y*y)), math.atan2(-y, -x))
        
        # point directly down
        #new_pose_q_arr = tf.transformations.quaternion_from_euler(0,  1.5, 0)
        #rospy.loginfo("new_pose_q: %f, %f, %f, %f", new_pose_q_arr[0], new_pose_q_arr[1], new_pose_q_arr[2], new_pose_q_arr[3])
        new_pose.orientation = Quaternion(x = new_pose_q_arr[0], y = new_pose_q_arr[1], z = new_pose_q_arr[2], w = new_pose_q_arr[3])
        
        cam_ctrl.set_camera_pose(new_pose)
        prev_contour_area = 100000000
        prev_major_len = 0
        prev_minor_len = 0
        min_bounding_area = 100000000
        max_stable = 0
               
               
        increasing = 0
        decreasing = 0
        stable     = 0 

        planar_motion_dist = 0.1
        area_thresh = .01
        min_pose = None

        last_5_areas = collections.deque(maxlen=5)

        pic_num = 0
        direction = 0

        while not rospy.is_shutdown():
            if(i == 0):
                prev_bounding_area = cam_ctrl.bounding_area
            
            if(cam_ctrl.blob_detected and cam_ctrl.blob_major_angle is not None):
                cam_ctrl.orig_img_pub.publish(cam_ctrl.bridge.cv2_to_imgmsg(cam_ctrl.orig_img, "bgr8"))
                cam_ctrl.seg_img_pub.publish(cam_ctrl.bridge.cv2_to_imgmsg(cam_ctrl.seg_img, "bgr8"))
                cam_ctrl.pca_img_pub.publish(cam_ctrl.bridge.cv2_to_imgmsg(cam_ctrl.pca_img, "bgr8"))

                cam_ctrl.camera_centered = False
                

                # Center camera on object
                #rospy.loginfo("Centering Camera")
                while(cam_ctrl.camera_centered == False):
                    if cam_ctrl.blob_detected:
                        #rospy.loginfo("Blob Detected!")
                        cam_ctrl.center_on_pixel()
                    else:
                        cam_ctrl.camera_centered = True
                    rospy.sleep(0.01)
                #rospy.loginfo("Done Centering")

                prev_minor_len = cam_ctrl.minor_len

                #rospy.loginfo("Rolling Camera")
                # Align Camera with major axis of object pointing up
                while(cam_ctrl.blob_major_angle is not None and cam_ctrl.blob_major_angle%90 > 3.0):
                    #rospy.loginfo("Angle is still %f... Rolling", cam_ctrl.blob_major_angle)
                    if(cam_ctrl.blob_major_angle > 90):
                        cam_ctrl.camera_roll(-.005)
                    else:
                        cam_ctrl.camera_roll(.005)

                    rospy.sleep(0.01)

                #rospy.loginfo("Done Rolling")


                if(i < sample_count / 2):
                    rospy.loginfo("Aligning Minor Axis")


                    if(cam_ctrl.bounding_area < min_bounding_area):
                        min_bounding_area = cam_ctrl.bounding_area
                        rospy.loginfo("Reached a new minimum!")
                        min_pose = cam_ctrl.model_state.pose

                    if(cam_ctrl.bounding_area < (1 - area_thresh) * min_bounding_area):
                        increasing =  0
                        decreasing += 1
                        stable     =  0

                    elif(cam_ctrl.bounding_area > (1 + area_thresh) * min_bounding_area):
                        increasing += 1
                        decreasing = 0
                        stable = 0

                    else:
                        increasing = 0
                        decreasing = 0
                        stable += 1


                    if(increasing >= sample_count/10):
                        rospy.loginfo("*************INCREASING**************")

                        # Reverse direction
                        direction += 180
                        increasing = 0
                        stable = 0

                    if(stable >= sample_count/5):
                        rospy.loginfo("*************STABLE**************")
                        i = sample_count/2
                        continue

                    prev_bounding_area = cam_ctrl.bounding_area

                    cam_ctrl.move_camera_in_plane(math.radians(direction + cam_ctrl.blob_minor_angle))


                elif i == sample_count / 2:
                    
                    rospy.loginfo("Moving to optimal position to align with minor axis.")
                    cam_ctrl.set_camera_pose(min_pose)

                    # Reset Counters
                    increasing = 0
                    decreasing = 0
                    stable = 0
                elif i > sample_count / 2 and i < sample_count:
                    rospy.loginfo("Aligning Major Axis")
                    
                    cam_ctrl.move_camera_in_plane(math.radians(cam_ctrl.blob_major_angle))

                    if(cam_ctrl.bounding_area < min_bounding_area):
                        min_bounding_area = cam_ctrl.bounding_area
                        rospy.loginfo("Reached a new minimum!")
                        min_pose = cam_ctrl.model_state.pose
                
                else:
                    rospy.loginfo("Moving to optimal position to align with major axis.")
                    cam_ctrl.set_camera_pose(min_pose)


                


    
                """
                # Determine which planar camera motion to perform
                if(cam_ctrl.contour_area > (1 + area_thresh) * prev_contour_area):
                    rospy.loginfo("Area is %f and is increasing", cam_ctrl.contour_area)
                    if(cam_ctrl.major_len > 1.01 * prev_major_len):
                        rospy.loginfo("Major Axis is increasing")
                        cam_ctrl.move_camera_in_plane(math.radians(cam_ctrl.blob_major_angle), planar_motion_dist)
                    #if(cam_ctrl.minor_len > 1.01 * prev_minor_len):
                    #    rospy.loginfo("Minor Axis is increasing")
                    #    cam_ctrl.move_camera_in_plane(math.radians(cam_ctrl.blob_minor_angle), planar_motion_dist)
                    increasing += 1
                    decreasing += 0
                    stable = 0

                elif(cam_ctrl.contour_area < (1 - area_thresh) * prev_contour_area):
                    rospy.loginfo("Area is %f and is decreasing", cam_ctrl.contour_area)
                    if(cam_ctrl.major_len < .99 * prev_major_len):
                        # Major Axis is decreasing
                        rospy.loginfo("Major Axis is increasing")
                        cam_ctrl.move_camera_in_plane(math.radians(-cam_ctrl.blob_major_angle), planar_motion_dist)
                    #if(cam_ctrl.minor_len < .99 * prev_minor_len):
                    #    # Minor Axis is decreasing
                    #    rospy.loginfo("Minor Axis is increasing")
                    #    cam_ctrl.move_camera_in_plane(math.radians(-cam_ctrl.blob_minor_angle), planar_motion_dist)
                    increasing = 0
                    decreasing += 1
                    stable = 0

                    if(cam_ctrl.contour_area < min_contour_area):
                        min_contour_area = cam_ctrl.contour_area

                else:
                    rospy.loginfo("Area is stable at %f", cam_ctrl.contour_area)
                    increasing = 0
                    decreasing = 0
                    stable += 1

                rospy.loginfo("Stable: %d, increasing: %d, decreasing: %d", stable, increasing, decreasing)
                rospy.loginfo("Max Stable: %d at (%f, %f, %f)", max_stable, curr_pose.position.x, curr_pose.position.y, curr_pose.position.z)
                if(stable > max_stable):
                    max_stable = stable
                    max_stable_pose = curr_pose

                if (stable == 10000):
                    rospy.loginfo("Minimum contour area is: %f", min_contour_area)
                    while(cam_ctrl.camera_centered == False):
                        if cam_ctrl.blob_detected:
                            #rospy.loginfo("Blob Detected!")
                            cam_ctrl.center_on_pixel()
                    while(cam_ctrl.blob_major_angle is not None and cam_ctrl.blob_major_angle%90 > 3.0):
                        #rospy.loginfo("Angle is still %f... Rolling", cam_ctrl.blob_major_angle)
                        if(cam_ctrl.blob_major_angle > 90):
                            cam_ctrl.camera_roll(-.001)
                        else:
                            cam_ctrl.camera_roll(.001)
                    return
                    

                prev_contour_area = cam_ctrl.contour_area
                prev_major_len = prev_major_len
                prev_minor_len = prev_minor_len
                """
                
                #cam_ctrl.move_camera_in_plane(math.radians(cam_ctrl.blob_minor_angle))
                """
                if(cam_ctrl.major_len < max_major_len):
                    rospy.loginfo("Major axis length is decreasing")
                    cam_ctrl.move_camera_in_plane(math.radians(cam_ctrl.blob_major_angle), 0.008)
                """
                    
                
                #rospy.loginfo("Minor length: %f", cam_ctrl.minor_len)
                #if(cam_ctrl.minor_len <= min_minor_len):
                #    rospy.loginfo("Minor axis length is still decreasing")
                #    cam_ctrl.move_camera_in_plane(-cam_ctrl.blob_minor_angle)

                #rospy.sleep(1)
                if(i < sample_count):
                    rospy.loginfo("i is %d.", i)
                    if(cam_ctrl.blob_detected):
                        while(not cam_ctrl.camera_centered):
                            cam_ctrl.center_on_pixel()
                            cam_ctrl.orig_img_pub.publish(cam_ctrl.bridge.cv2_to_imgmsg(cam_ctrl.orig_img, "bgr8"))
                            cam_ctrl.seg_img_pub.publish(cam_ctrl.bridge.cv2_to_imgmsg(cam_ctrl.seg_img, "bgr8"))
                            cam_ctrl.canny_img_pub.publish(cam_ctrl.bridge.cv2_to_imgmsg(cam_ctrl.canny_img, "8UC1"))
                            cam_ctrl.pca_img_pub.publish(cam_ctrl.bridge.cv2_to_imgmsg(cam_ctrl.pca_img, "bgr8"))
                            cam_ctrl.hough_img_pub.publish(cam_ctrl.bridge.cv2_to_imgmsg(cam_ctrl.hough_img, "bgr8"))
                        #rospy.loginfo("Area is %d", cam_ctrl.contour_area)
                        #rospy.loginfo("Major Length is %f", cam_ctrl.major_len)
                        #rospy.loginfo("Minor Length is %f", cam_ctrl.minor_len)

                    # Camera is centered on center of blob
                    curr_pose = cam_ctrl.model_state.pose

                    x = curr_pose.position.x
                    y = curr_pose.position.y
                    z = curr_pose.position.z

                    d = math.sqrt(x**2 + y**2 + z**2)
                    # Distance to object
                    rospy.loginfo("X, Y, Z = (%f, %f, %d) Area is %f Distance to object is: %f", x, y, z, cam_ctrl.bounding_area, d)

                    major_lens.append(cam_ctrl.major_len)
                    minor_lens.append(cam_ctrl.minor_len)

                    poses.append(curr_pose)
                    images.append(cam_ctrl.orig_img)
                    angles.append([cam_ctrl.blob_major_angle, cam_ctrl.blob_minor_angle])
                    areas.append((cam_ctrl.contour_area, cam_ctrl.bounding_area))

                    i += 1
                else:
                    # Save images and poses
                    plot_results(poses, minor_lens, major_lens, areas, angles)

                    print("Saving data from run")
                    pickle.dump(poses, open("poses.p", "wb"))
                    pickle.dump(images, open("images.p", "wb"))
                    pickle.dump(angles, open("angles.p", "wb"))
                    pickle.dump(major_lens, open("major_lens.p", "wb"))
                    pickle.dump(minor_lens, open("minor_lens.p", "wb"))
                    pickle.dump(areas, open("areas.p", "wb"))

                    print("Completed Saving.")

                    return

                rospy.sleep(0.1)
            else:
                rospy.loginfo("No blobs have been detected!")

    else:
        load_and_plot()





if __name__=='__main__':
    main()