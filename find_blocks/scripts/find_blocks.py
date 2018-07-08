#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from matplotlib import pyplot as plt

from std_msgs.msg import String, Int32
from sensor_msgs.msg import Image, CameraInfo, Range

from image_geometry import PinholeCameraModel

from cv_bridge import CvBridge

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
    RED_LOW_SAT     = 242
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
        "red":      {   "low_h": RED_LOW_HUE,   "high_h": RED_HIGH_HUE,
                        "low_s": RED_LOW_SAT,   "high_s": RED_HIGH_SAT,
                        "low_v": RED_LOW_VAL,   "high_v": RED_HIGH_VAL  },
#        "yellow":   {   "low_h": YEL_LOW_HUE,   "high_h": YEL_HIGH_HUE,
#                        "low_s": YEL_LOW_SAT,   "high_s": YEL_HIGH_SAT,
#                        "low_v": YEL_LOW_VAL,   "high_v": YEL_HIGH_VAL  },
#        "green":    {   "low_h": GRN_LOW_HUE,   "high_h": GRN_HIGH_HUE,
#                        "low_s": GRN_LOW_SAT,   "high_s": GRN_HIGH_SAT,
#                        "low_v": GRN_LOW_VAL,   "high_v": GRN_HIGH_VAL  },
#        "blue":     {   "low_h": BLU_LOW_HUE,   "high_h": BLU_HIGH_HUE,
#                       "low_s": BLU_LOW_SAT,   "high_s": BLU_HIGH_SAT,
#                       "low_v": BLU_LOW_VAL,   "high_v": BLU_HIGH_VAL  }
}

class BlockFinder():
    def __init__(self, limb):
        self.limb = limb
        self.block_locs = []
        self.tf_listener = tf.TransformListener()
        self.camera_model = None
        self.min_ir_depth = 0.1
        self.object_height = 0.05
        
    def publish(self):
        self.handler_pub = rospy.Publisher("block_finder/" + self.limb + "/block_loc", PoseArray)
    def subscribe(self):
        self.cam_sub = rospy.Subscriber("/cameras/" + self.limb + "_camera/image", Image, self.cam_callback)
        self.info_sub = rospy.Subscriber("/cameras/" + self.limb + "_camera/camera_info", CameraInfo, self.info_callback)
        self.ir_sub = rospy.Subscriber("/robot/range/" + self.limb + "_range/state", Range, self.ir_callback)


    def info_callback(self, data):
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(data)
        self.info_sub.unregister() # Unsubscribe after receiving CameraInfo first time

    def ir_callback(self, data):
        self.ir_reading = data.range
        #print("IR reading: ", self.ir_reading)
        if(self.ir_reading > 60):
            rospy.loginfo("Invalid IR reading")
            self.ir_reading = 0.4
    
    '''
    Thresholds camera image and stores object centroid location (x,y) in Baxter's base frame.
    '''
    def cam_callback(self, data):
        self.block_locs = []
        bridge = CvBridge()

        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")

        #find_hsv_values(cv_image)

        height, width, depth = cv_image.shape
        low_s = 0
        low_v = 0

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        images = [hsv, hsv, hsv, hsv, hsv]
        i = 0

        fig = plt.figure()


        for key in colors:
            print(key)
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

            numobj = len(contours) # number of objects found in current frame
            print ('Number of objects found in the current frame: ' , numobj)

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
                    d = (self.ir_reading - self.object_height)

                    print("Distance to object: ", d)
                    d_cam = d * vec

                    homog_d_cam = np.concatenate((d_cam, np.ones(1))).reshape((4,1))

                    # TODO: Might need to multiply by distance to table from head here...


                    # Wait for transformation from base to head_camera
                    self.tf_listener.waitForTransform('/world', self.limb + "_camera", rospy.Time(), rospy.Duration(4))
                    (trans, rot) = self.tf_listener.lookupTransform('/world', self.limb + "_camera", rospy.Time())

                    camera_to_base = tf.transformations.compose_matrix(translate=trans, angles=tf.transformations.euler_from_quaternion(rot))

                    object_world = np.dot(camera_to_base, homog_d_cam)
                    div = np.divide(object_world, np.array([[0.7], [0.15], [-0.129], [1]]))

                    print("DIV: ", div)
                    
                    self.block_locs.append(Pose(position=Point(*object_world)))
                    
                    #print("World coordinate of object: ", self.block_locs[0])

                    
        """
        i = 0
        for key in colors:
            cv2.imshow("Original", cv_image)
            cv2.imshow(key, images[i])

            cv2.waitKey()

            i += 1
        """

def nothing():
    pass

def find_hsv_values(cv_image):
   cv2.namedWindow('result')

   # Starting with 100's to prevent error while masking
   h,s,v = 100,100,100

   # Creating track bar
   cv2.createTrackbar('h', 'result',0,179,nothing)
   cv2.createTrackbar('s', 'result',0,255,nothing)
   cv2.createTrackbar('v', 'result',0,255,nothing)

   while(1):
       #converting to HSV
       hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

       # get info from track bar and appy to result
       h = cv2.getTrackbarPos('h','result')
       s = cv2.getTrackbarPos('s','result')
       v = cv2.getTrackbarPos('v','result')

       # Normal masking algorithm
       lower_blue = np.array([h,s,v])
       upper_blue = np.array([180,255,255])

       mask = cv2.inRange(hsv,lower_blue, upper_blue)

       result = cv2.bitwise_and(cv_image,cv_image,mask = mask)

       cv2.imshow('result',result)

       k = cv2.waitKey(5) & 0xFF
       if k == 27:
           break


   cv2.destroyAllWindows()


'''
Creates a service that provides information about the loaction of an object.
Subscribes to left hand camera image feed
'''
def main():
    rospy.init_node('find_block')

    block_finder = BlockFinder("left_hand")
    block_finder.subscribe()
    block_finder.publish()

    rate = rospy.Rate(500)


    while not rospy.is_shutdown():
        #if len(block_finder.block_locs) > 0:
        #    print block_finder.block_locs
        rospy.spin()

if __name__ == '__main__':
     main()
