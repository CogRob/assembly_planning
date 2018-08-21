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

from cv_bridge import CvBridge

REAL = True
USE_HSV = True

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
        self.model_state = None
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

        self.blob_angle = 0

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
        self.model_state = get_model_state('camera_robot', 'world')

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

    def move_camera_in_plane(self, direction, motion_dist=0.01):
        #rospy.loginfo("Moving camera in plane in direction %f ", direction)

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
        new_pose.orientation = self.model_state.pose.orientation

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



    def center_on_pixel(self):
        #rospy.loginfo("Centering camera on pixel_loc: (%f, %f)", self.pixel_loc[0], self.pixel_loc[1])

        # PID loop to center on pixel location
        center_thresh = 10.0 # Attempt to get the camera centered within 5 pixels in x and y axes
        x_centered = False
        y_centered = False

        # Calculate how far off 
        
        x_offset = self.res_w/2 - self.pixel_loc[0] 
        #rospy.loginfo("Object is off by %f pixels in x", x_offset)

        if(math.fabs(x_offset) > center_thresh):
            #rospy.loginfo("Yawing camera")
            self.camera_yaw(x_offset * math.pi / 400000)
        else:
            #rospy.loginfo("X offset of %f is within threshold %f", x_offset, center_thresh)
            x_centered = True
        
        y_offset = self.res_h/2 - self.pixel_loc[1] 
        #rospy.loginfo("Object is off by %f pixels in y", y_offset)
        
        if(math.fabs(y_offset) > center_thresh):
            #rospy.loginfo("Pitching camera")
            self.camera_pitch(-y_offset * math.pi / 400000)
        else:
            #rospy.loginfo("Y offset of %f is within threshold %f", y_offset, center_thresh)
            y_centered = True


        if(x_centered == True and y_centered == True):
            self.camera_centered = True
            self.pixel_loc = np.zeros(2)
            


    '''
    Thresholds camera image and stores object centroid location (x,y) in Baxter's base frame.
    '''
    def cam_callback(self, data):
        img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.orig_img = img

        height, width, depth = img.shape
        low_s = 0
        low_v = 0

        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        color = "green"

        low_h = colors[color]["low_h"]
        high_h = colors[color]["high_h"]
        low_s = colors[color]["low_s"]
        high_s = colors[color]["high_s"]
        low_v = colors[color]["low_v"]
        high_v = colors[color]["high_v"]

        #Converting image to HSV format
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

        contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_NONE)

        #Draw the countours.
        #cv2.drawContours(cv_image, contours, -1, (0,255,0), 3)
        large_contours = []

        rect_img = img.copy()
        min_area_mask = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        min_area_mask.fill(0)

        contour_area_thresh = 10
        
        self.blob_detected = True

        if(len(contours) > 0):
            for c in contours:
                area = cv2.contourArea(c)

                if(area > contour_area_thresh):
                    large_contours.append(c)

                    # Min Area Rectangle
                    rect = cv2.minAreaRect(c)
                    box = cv2.cv.BoxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(min_area_mask, [box], 0, (255,255,255) , -1)
                    masked_img = cv2.bitwise_and(img, img, mask=min_area_mask)

                    # Bounding Rectangle
                    x, y, w, h = cv2.boundingRect(c)
                    cv2.rectangle(rect_img, (x, y), (x+w, y+h), color_vals[color], 2)

                    img = img[y:y+h, x:x+w]

                    # TODO: To optimize, crop the eroded image...
                    erode_cropped = erode_1[y:y+h, x:x+w]
                    cropped_w = y
                    cropped_h = x

                    break
                else:
                    rospy.loginfo("No contour with area larger than %d was found.", contour_area_thresh)
                    self.blob_detected = False
                    return


            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            
            # Canny Edge Detection
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

            mat = np.array(mat).astype(np.float32) #have to convert type for PCA

            #mean (e. g. the geometrical center)
                #and eigenvectors (e. g. directions of principal components)
            m, e = cv2.PCACompute(mat, mean = None)

            #now to draw: let's scale our primary axis by 100,
            #and the secondary by 50
            center = tuple(m[0])
            endpoint1 = tuple(m[0] + e[0]*100)
            endpoint2 = tuple(m[0] + e[1]*50)

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

            self.blob_angle = 180 - (angle_rad_major * 180 / np.pi) % 180

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



def plot_results():
    images = pickle.load("images.p")
    poses = pickle.load("poses.p")
    angles = pickle.load("angles.p")

    for i in range(len(images)):
        curr_img = images[i]
        curr_pose = poses[i]
        curr_angle = angles[i]



if __name__=='__main__':
    rospy.init_node('camera_controller')
    cam_cont = CameraController()

    cam_cont.reset_camera_pose()
    cam_cont.publish()
    cam_cont.subscribe()
    cam_cont.subscribe()
    radius = 1
    tilt = 0.0
    divisor = 16
    curr_step = 0


    poses = []
    images = []
    angles = []
    i = 0

    while not rospy.is_shutdown():
        # Move to random location
        if(curr_step%divisor == 0):
            tilt += 0.1

        angle = curr_step * 2 * math.pi / divisor
        curr_step += 1
    

        rospy.loginfo("Tilt: %f", tilt)
        rospy.loginfo("Center pixel location: %d %d", cam_cont.center_loc[0], cam_cont.center_loc[1])
        
        cam_cont.get_camera_pose()

        curr_pose = cam_cont.model_state.pose

        new_pose = Pose()

        x = random.uniform(-1, 1)
        y = random.uniform(-1, 1)
        
        z = random.uniform(0, 1)

        origin = Point(x=0, y=0, z=0) 

        new_pose.position.x = origin.x + x
        new_pose.position.y = origin.y + y
        new_pose.position.z = origin.z + z

        #rospy.loginfo("x: %f, y: %f, z: %f", x, y, z)
        new_pose_q_arr = tf.transformations.quaternion_from_euler(0, math.atan2(z, math.sqrt(x*x + y*y)), math.atan2(-y, -x))
        #rospy.loginfo("new_pose_q: %f, %f, %f, %f", new_pose_q_arr[0], new_pose_q_arr[1], new_pose_q_arr[2], new_pose_q_arr[3])
        new_pose.orientation = Quaternion(x = new_pose_q_arr[0], y = new_pose_q_arr[1], z = new_pose_q_arr[2], w = new_pose_q_arr[3])
        cam_cont.set_camera_pose(new_pose)
        cam_cont.orig_img_pub.publish(cam_cont.bridge.cv2_to_imgmsg(cam_cont.orig_img, "bgr8"))
        cam_cont.seg_img_pub.publish(cam_cont.bridge.cv2_to_imgmsg(cam_cont.seg_img, "bgr8"))
        cam_cont.pca_img_pub.publish(cam_cont.bridge.cv2_to_imgmsg(cam_cont.pca_img, "bgr8"))

        cam_cont.camera_centered = False
        """
        while(cam_cont.camera_centered == False):
            if cam_cont.blob_detected:
                cam_cont.center_on_pixel()
                rospy.loginfo("No blobs were detected")
                rospy.sleep(1)
            else:
                cam_cont.camera_centered = True
                rospy.sleep(1)
        """

        while(i < 1000):
            rospy.loginfo("i is %d.", i)
            if(cam_cont.blob_detected):
                while(not cam_cont.camera_centered):
                    cam_cont.center_on_pixel()
                    cam_cont.orig_img_pub.publish(cam_cont.bridge.cv2_to_imgmsg(cam_cont.orig_img, "bgr8"))
                    cam_cont.seg_img_pub.publish(cam_cont.bridge.cv2_to_imgmsg(cam_cont.seg_img, "bgr8"))
                    cam_cont.canny_img_pub.publish(cam_cont.bridge.cv2_to_imgmsg(cam_cont.canny_img, "8UC1"))
                    cam_cont.pca_img_pub.publish(cam_cont.bridge.cv2_to_imgmsg(cam_cont.pca_img, "bgr8"))
                    cam_cont.hough_img_pub.publish(cam_cont.bridge.cv2_to_imgmsg(cam_cont.hough_img, "bgr8"))

                # Camera is centered on center of blob
                curr_pose = cam_cont.model_state.pose

                poses.append(curr_pose)
                images.append(cam_cont.pca_img)
                angles.append(cam_cont.blob_angle)

                i += 1
                #cam_cont.move_camera_in_plane(cam_cont.blob_angle)
                break
            else:
                break

        if(i == 1000):
            # Save images and poses
            pickle.dump(poses, open("poses.p", "wb"))
            pickle.dump(images, open("images.p", "wb"))
            pickle.dump(angles, open("angles.p", "wb"))



