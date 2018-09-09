block_mover components

Apriltags2_ros - ros package that detects april tags in an image and returns the pose of the tag w.r.t to the camera

synchronize_camera - ros script to synchronize camera_info and images over network with latency

block_finder  - ros node to find blocks in an image.

camera_calibrator - ros node that outputs a tf from the overhead kinect to baxter’s base to align their reference frames. Only runs if motion in the base has been detected