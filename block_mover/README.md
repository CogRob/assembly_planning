

block_mover components

Apriltags2_ros - ros package that detects april tags in an image and returns the pose of the tag w.r.t to the camera

synchronize_camera - ros script to synchronize camera_info and images over network with latency

block_finder  - ros node to find blocks in an image.

camera_calibrator - ros node that outputs a tf from the overhead kinect to baxter’s base to align their reference frames. Only runs if motion in the base has been detected

* To use:
First calibrate both cameras (Kinect and Baxter's right hand camera) using ROS camera_calibration package.

* I used a 8x10 chessboard with 0.0225 m squares

* To run, run the following command 
$ rosrun camera_calibration cameracalibrator.py -p 'chessboard' -c /camera/rgb -s 10x8 -q 0.0225



## Extrinsic Calibration Setup
#### T1
$ roscore

#### T2
$ rosbag play baxter_2_cam.bag


### For top camera:
#### T3
$ rosrun block_mover synch_camera.py "top"
#### T4
$ roslaunch apriltags2_ros continuous_detection.launch camera_name:=/synch_camera/top/ image_topic:=/image_rect camera_frame:=/tf_static/camera_rgb_optical_frame

### For hand camera:
#### T5
$ rosrun block_mover synch_camera.py "right_hand"
#### T6
$ roslaunch apriltags2_ros continuous_detection.launch camera_name:=/synch_camera/right_hand image_topic:=/image_rect camera_frame:=/tf/right_hand_camera

#### T7 RVIZ
$ rosrun rviz rviz 

#### Open apriltags.rviz config.