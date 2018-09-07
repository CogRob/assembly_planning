#!/usr/bin/env python
import rospy
import rospkg
import tf
import numpy as np
import math
from apriltags2_ros import AprilTagDetection, AprilTagDetectionArray
from geometry import tf_conversions as tfc


import rospy
class BlockMover:
    def __init__(self):
        self.hand_cam_to_table = None
        self.top_cam_to_table = None
        self.top_cam_to_base = None

        self.hand_cam_to_table_stamp = None
        self.top_cam_to_table_stamp = None
        self.top_cam_to_base_stamp





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
        self.hand_cam_to_table = tfc.toMatrix(tag_detections[0].pose)
        self.hand_cam_to_table_stamp = tag_detections.header.stamp
        
    def top_cam_tag_callback(self, tag_detections):
        self.top_cam_to_table = tfc.toMatrix(tag_detections[0].pose)
        self.top_cam_to_table_stamp = tag_detections.header.stamp



    def calc_top_cam_to_base(self):
        if(self.top_cam_to_table != None and self.hand_cam_to_table != None):
            time_diff = math.fabs(self.top_cam_to_table_stamp.sec - self.hand_cam_to_table_stamp)

            # Make sure timestamps are within 1 sec of eachother
            if(time_diff < 1.0):
                # Get hand_cam to base transform
                self.tf_listener.waitForTransform('right_hand_camera', '/base', rospy.Time(), rospy.Duration(4))
                (hand_cam_to_base_trans, hand_cam_to_base_rot) = self.tf_listener.lookupTransform('/right_hand_camera', 'base', rospy.Time())

                # Get base to hand_cam transform
                #self.tf_listener.waitForTransform('/base', 'right_hand_camera', rospy.Time(), rospy.Duration(4))
                #(trans, rot) = self.tf_listener.lookupTransform('/base', 'right_hand_camera', rospy.Time())

                hand_cam_to_base = tf.transfomations.compose_matrix(translate=hand_cam_to_base_trans, angles=hand_cam_to_base_rot)
                table_to_hand_cam = np.linalg.inverse(self.hand_cam_to_table)

                top_cam_to_hand_cam = np.dot(self.top_cam_to_table, table_to_hand_cam)

                self.top_cam_to_base = np.dot(top_cam_to_hand_cam, hand_cam_to_base)
                self.top_cam_to_base_stamp = rospy.Time.now()

                print("Top cam to base: ", self.top_cam_to_base)
                
            else:
               rospy.loginfo("TF Timestamps are off by %f", time_diff)
        else:
           if(self.top_cam_to_table_tf == None):
               rospy.loginfo("Top cam to table transform is still None!")
           if(self.hand_cam_to_table_tf == None):
               rospy.loginfo("Hand cam to table transform is still None!")



def main():
    block_mover = BlockMover()
    rospy.init_node('block_mover', anonymous=True)

    block_mover.subscribe()
    block_mover.publish()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()