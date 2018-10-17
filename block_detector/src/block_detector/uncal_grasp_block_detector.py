#!/usr/bin/env python

from block_detector_base import BlockDetector

from block_detector.msg import BlockPixelLoc, BlockPixelLocArray

# ROS imports
import rospy

# ROS messages
from sensor_msgs.msg import Image, Range


# TODO; Store all of these in a param file somewhere


class UncalGraspBlockDetector(BlockDetector):
    def __init__(self, pub_rate, sim):
        self.camera_topic = "/cameras/right_hand_camera/image"

        img_msg = rospy.wait_for_message(self.camera_topic, Image)

        self.res_w = img_msg.width
        self.res_h = img_msg.height

        BlockDetector.__init__(self, resolution=(self.res_w, self.res_h),
                               allowed_circle_center=(
                                   self.res_w/2, self.res_h/2),
                               allowed_circle_diameter=450,
                               allowed_circle_thickness=255,
                               pub_rate=pub_rate)

        self.camera = "right_hand"

        self.font_size = 3.0
        self.font_thickness = 2

        self.morph_opening_kernel = (7, 7)
        self.morph_closing_kernel = (13, 13)

        self.enable_rviz_markers = False
        self.block_pixel_locs = BlockPixelLocArray()
        self.pub_rate = rospy.Rate(pub_rate)  # in Hz
        self.sim = sim
        if(self.sim):
            self.hsv_dict = {
                # "blue": {
                #    "low_h": 90,
                #    "high_h": 133,
                #    "low_s": 0,
                #    "high_s": 255,
                #    "low_v": 0,
                #    "high_v": 255,
                #    "color_val": (255, 0, 0),
                # },

                "green": {
                    "low_h": 35,
                    "high_h": 179,
                    "low_s": 0,
                    "high_s": 255,
                    "low_v": 0,
                    "high_v": 255,
                    "color_val": (0, 255, 0),
                },
                # "red": {
                #    "low_h": [0, 160],
                #    "high_h": [10, 180],
                #    "low_s": 0,
                #    "high_s": 255,
                #    "low_v": 0,
                #    "high_v": 255,
                #    "color_val": (0, 0, 255),
                # }
            }
        else:
            self.hsv_dict = {
                "blue": {
                    "low_h": 90,
                    "high_h": 133,
                    "low_s": 117,
                    "high_s": 225,
                    "low_v": 28,
                    "high_v": 255,
                    "color_val": (255, 0, 0),
                },

                "green": {
                    "low_h": 37,
                    "high_h": 104,
                    "low_s": 60,
                    "high_s": 199,
                    "low_v": 10,
                    "high_v": 255,
                    "color_val": (0, 255, 0),
                },

                "red": {
                    "low_h": [0, 160],
                    "high_h": [10, 180],
                    "low_s": 30,
                    "high_s": 255,
                    "low_v": 20,
                    "high_v": 255,
                    "color_val": (0, 0, 255),
                }
            }

    def create_block_pixel_locs_publisher(self):
        self.block_pixel_locs_pub = rospy.Publisher(
            "block_detector/uncal_grasp/block_pixel_locs", BlockPixelLocArray,
            queue_size=10)
        pass

    def create_debugging_publisher(self):
        BlockDetector.create_debugging_publisher(self)

    def publish_block_pixel_locs(self):
        self.block_pixel_locs_pub.publish(self.block_pixel_locs)

    def subscribe(self):
        self.cam_sub = rospy.Subscriber("/cameras/right_hand_camera/image",
                                        Image, self.cam_callback)
        self.ir_sub = rospy.Subscriber("/robot/range/right_hand_range/state",
                                       Range, self.ir_callback)

    def cam_callback(self, data):
        BlockDetector.cam_callback(self, data)

        self.generate_pixel_locs()

        # To tune hsv thresholds, uncomment
        # self.find_hsv_values()

    def ir_callback(self, data):
        self.ir_reading = data.range

        # 0.65 meters is the upper limit of the IR sensor on Baxter
        if (self.ir_reading > 0.65):
            self.ir_reading = 0.4

        if(self.ir_reading != 0):
            self.blob_area_min_thresh = 400 / self.ir_reading  # TODO: Tune
        else:
            self.blob_area_min_thresh = 400  # TODO: Tune

        self.blob_area_max_thresh = 100000  # TODO: Tune

        self.camera_height = self.ir_reading

    # TODO: Make this based upon length and width instead of ratio like
    # TopBlockDetector, but need to tune the length and width when the
    # camera is at hover height first
    def get_block_type(self, block_length, block_width):
        return (int(block_length), int(block_width))

    def generate_pixel_locs(self):
        block_pixel_locs_list = []

        for detected_block in self.detected_blocks:
            block_center = detected_block["block_center"]
            block_angle = detected_block["block_angle"]
            block_length = detected_block["block_length"]
            block_width = detected_block["block_width"]
            block_color = detected_block["color"]

            # TODO: double check the x and y here!
            block_pixel_loc = BlockPixelLoc(
                x=block_center[0],
                y=block_center[1],
                theta=block_angle,
                length=block_length,
                width=block_width,
                color=block_color
            )

            block_pixel_locs_list.append(block_pixel_loc)

        self.block_pixel_locs.pixel_locs = block_pixel_locs_list


# Testing for HandBlockDetector

def main():
    rospy.init_node("uncal_grasp_block_detector")

    hand_block_detector = UncalGraspBlockDetector(pub_rate=10, sim=True)

    hand_block_detector.subscribe()
    hand_block_detector.create_block_pixel_locs_publisher()
    hand_block_detector.create_debugging_publisher()

    while (not rospy.is_shutdown()):
        # For debugging
        hand_block_detector.publish_debugging()
        # hand_block_detector.publish_markers()
        hand_block_detector.publish_block_pixel_locs()

        hand_block_detector.pub_rate.sleep()


if __name__ == "__main__":
    main()
