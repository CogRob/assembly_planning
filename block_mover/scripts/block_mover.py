#!/usr/bin/env python
import rospy
import rospkg
import tf
import numpy as np
import math
from apriltags2_ros.msg import AprilTagDetection, AprilTagDetectionArray
import tf_conversions as tfc


import rospy
class BlockMover:
    def __init__(self):
        self.hand_cam_to_table = [np.zeros((4,4)), np.zeros((4,4)), np.zeros((4,4))]
        self.top_cam_to_table  =  [np.zeros((4,4)), np.zeros((4,4)), np.zeros((4,4))]
        self.top_cam_to_base = np.zeros((4,4))

        self.hand_cam_to_table_stamp = None
        self.top_cam_to_table_stamp = None

        self.top_cam_to_base_stamp = None

        self.top_cam_tag_found = False
        self.hand_cam_tag_found = False




        self.tf_listener = tf.TransformListener()

    def subscribe(self):
        # April tag detections
        self.hand_cam_tags_sub = rospy.Subscriber("/hand_cam/tag_detections", AprilTagDetectionArray, self.hand_cam_tag_callback)
        self.top_cam_tags_sub = rospy.Subscriber("/top_cam/tag_detections", AprilTagDetectionArray, self.top_cam_tag_callback)

    def publish(self):
        #self.top_cam_to_base_pub = Publisher("/top_cam_to_base", )
        pass

    
    def save_transforms(self):
        # Save the transform from kinect to middle April Tag

        # Save the transform from Base to middle April Tag 
        
        pass
    
    def hand_cam_tag_callback(self, tag_detections):
        #rospy.loginfo("Received hand cam tag!")

        for detection in tag_detections.detections:
            self.hand_cam_to_table_stamp = tag_detections.header.stamp
            self.hand_cam_tag_found = True
            tag_id = detection.id[0]
            self.hand_cam_to_table[tag_id] = tfc.toMatrix(tfc.fromMsg(detection.pose.pose.pose))
            #print("Hand_to_Table[",tag_id, "] ", self.hand_cam_to_table[tag_id][0:3, 3])
        
    def top_cam_tag_callback(self, tag_detections):
        #rospy.loginfo("Received top cam tag!")

        for detection in tag_detections.detections:
            self.top_cam_to_table_stamp = tag_detections.header.stamp
            self.top_cam_tag_found = True
            tag_id = detection.id[0]
            self.top_cam_to_table[tag_id] = tfc.toMatrix(tfc.fromMsg(detection.pose.pose.pose))
            #print("Top_to_table[",tag_id, "] ", self.top_cam_to_table[tag_id][0:3, 3])

    def calc_top_cam_to_base(self):

        if(self.top_cam_tag_found and self.hand_cam_tag_found):
            time_diff = math.fabs(self.top_cam_to_table_stamp.secs - self.hand_cam_to_table_stamp.secs)

            # Make sure timestamps are within 1 sec of eachother
            if(time_diff <= 149.0):
                # Get hand_cam to base transform
                self.tf_listener.waitForTransform('right_hand_camera', '/base', rospy.Time(), rospy.Duration(4))
                (hand_cam_to_base_trans, hand_cam_to_base_rot) = self.tf_listener.lookupTransform('/right_hand_camera', 'base', rospy.Time())

                hand_cam_to_base = tf.transformations.compose_matrix(translate=hand_cam_to_base_trans, angles=hand_cam_to_base_rot)

                for i in range(3):
                    table_to_hand_cam = np.linalg.inv(self.hand_cam_to_table[i])
                    print(self.hand_cam_to_table[i])

                    top_cam_to_hand_cam = np.dot(self.top_cam_to_table[i], table_to_hand_cam)
                    #top_cam_to_hand_cam = np.dot(table_to_hand_cam, self.top_cam_to_table[i])

                    self.top_cam_to_base = np.dot(top_cam_to_hand_cam, hand_cam_to_base)

                    self.top_cam_to_base_stamp = rospy.Time.now()

                    print("Top cam to base[",i ,"]: ", self.top_cam_to_base[0:3,3], " Time hand: ", self.hand_cam_to_table_stamp.secs, " Time top: ", self.top_cam_to_table_stamp.secs)
                
            else:
               rospy.loginfo("TF Timestamps are off by %f", time_diff)
        else:
            if(self.top_cam_tag_found == False):
               rospy.loginfo("Top cam to table transform not available!")
            else:
               top_cam_to_table_age = rospy.Time.now() - self.top_cam_to_table_stamp
               rospy.loginfo("Top cam to table transform is available (%f.%f old).", top_cam_to_table_age.secs, top_cam_to_table_age.nsecs)
        
            if(self.hand_cam_tag_found == False):
               rospy.loginfo("Hand cam to table transform not available!")
            else:
               hand_cam_to_table_age = rospy.Time.now() - self.hand_cam_to_table_stamp
               rospy.loginfo("Hand cam to table transform is available (%f.%f old).", hand_cam_to_table_age.secs, hand_cam_to_table_age.nsecs)



def main():
    block_mover = BlockMover()
    rospy.init_node('block_mover', anonymous=True)

    block_mover.subscribe()
    block_mover.publish()

    while not rospy.is_shutdown():
        block_mover.calc_top_cam_to_base()
        rospy.sleep(0.1)

if __name__ == '__main__':
               rospy.loginfo("Hand cam to table transform is available!")



def main():
    block_mover = BlockMover()
    rospy.init_node('block_mover', anonymous=True)

    block_mover.subscribe()
    block_mover.publish()

    while not rospy.is_shutdown():
        block_mover.calc_top_cam_to_base()
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()