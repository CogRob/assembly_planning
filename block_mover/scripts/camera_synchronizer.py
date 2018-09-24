#!/usr/bin/env python

from message_filters import ApproximateTimeSynchronizer, TimeSynchronizer, Subscriber
from sensor_msgs.msg import Image, CameraInfo
import rospy
import sys


class CameraSynchronizer:
    def __init__(self, camera_name):
        rospy.logerr("Initializing!")
        self.camera_name = camera_name

        if(self.camera_name == "right_hand"):
            self.image_sub = Subscriber(
                "/cameras/right_hand_camera/image", Image)
            self.info_sub = Subscriber(
                "/cameras/right_hand_camera/camera_info_std", CameraInfo)

        elif(self.camera_name == "top"):
            rospy.loginfo("Initializing top camera!")
            self.image_sub = Subscriber(
                "/camera/rgb/image_raw", Image)
            self.info_sub = Subscriber(
                "/camera/rgb/camera_info", CameraInfo)

    def subscribe(self):
        # self.ats = ApproximateTimeSynchronizer(
        #    [self.image_sub, self.info_sub], queue_size=5, slop=0.5)
        # self.ats.registerCallback(self.gotImage)

        self.ts = TimeSynchronizer(
            [self.image_sub, self.info_sub], queue_size=5,)
        self.ts.registerCallback(self.gotImage)

    def publish(self):
        self.image_pub = rospy.Publisher(
            "/" + self.camera_name + "/synch_camera/image_rect",
            Image, queue_size=10)
        self.info_pub = rospy.Publisher(
            "/" + self.camera_name + "/synch_camera/camera_info",
            CameraInfo, queue_size=10)

    def gotImage(self, image, camerainfo):
        rospy.loginfo("Got an image!")
        print("GOT AN IMAGE")
        assert image.header.stamp == camerainfo.header.stamp
        rospy.loginfo("got an Image and CameraInfo")

        self.image_pub.publish(image)
        self.info_pub.publish(camerainfo)


def main():
    camera_name = sys.argv[1]
    im_synch = CameraSynchronizer(camera_name)
    rospy.init_node(camera_name + 'image_synch')

    im_synch.subscribe()
    im_synch.publish()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)


if __name__ == '__main__':
    main()
