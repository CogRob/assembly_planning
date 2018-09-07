#!/usr/bin/env python
from message_filters import TimeSynchronizer, ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, CameraInfo
import rospy
class ImageSynchronizer:
    def __init__(self):
        self.image_sub = Subscriber("/cameras/right_hand_camera/image", Image)
        self.info_sub = Subscriber("/cameras/right_hand_camera/camera_info_std", CameraInfo)
    
    def subscribe(self):
        #self.ats = ApproximateTimeSynchronizer([self.image_sub, self.info_sub], queue_size=5, slop=0.5)
        #self.ats.registerCallback(self.gotImage)
        
        self.ts = TimeSynchronizer([self.image_sub, self.info_sub], queue_size=5,)
        self.ts.registerCallback(self.gotImage)
    
    def publish(self):
        self.image_pub  = rospy.Publisher("/synch_camera/image_rect",  Image, queue_size=10)
        self.info_pub   = rospy.Publisher("/synch_camera/camera_info", CameraInfo, queue_size=10)
    
    def gotImage(self, image, camerainfo):
        assert image.header.stamp == camerainfo.header.stamp
        rospy.loginfo("got an Image and CameraInfo")
        
        self.image_pub.publish(image)
        self.info_pub.publish(camerainfo)

def main():
    im_synch = ImageSynchronizer()
    rospy.init_node('image_synch', anonymous=True)

    im_synch.subscribe()
    im_synch.publish()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)

if __name__ == '__main__':
    main()