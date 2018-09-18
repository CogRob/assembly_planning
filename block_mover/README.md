

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


Results of Kinect:
('D = ', [0.12901548737858895, -0.0976214488138105, 0.04467423672011915, 0.05023453204900971, 0.0])
('K = ', [640.0488679517078, 0.0, 388.72401633581785, 0.0, 613.0261892946805, 296.54390594388144, 0.0, 0.0, 1.0])
('R = ', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
('P = ', [649.9735717773438, 0.0, 409.96939727506833, 0.0, 0.0, 635.33251953125, 309.3424960302218, 0.0, 0.0, 0.0, 1.0, 0.0])
None
# oST version 5.0 parameters


[image]

width
640

height
480

[narrow_stereo]

camera matrix
640.048868 0.000000 388.724016
0.000000 613.026189 296.543906
0.000000 0.000000 1.000000

distortion
0.129015 -0.097621 0.044674 0.050235 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
649.973572 0.000000 409.969397 0.000000
0.000000 635.332520 309.342496 0.000000
0.000000 0.000000 1.000000 0.000000



Results of Right hand Camera

('D = ', [-0.00580122522379253, -0.015772840097667544, 0.0012825141370990728, 0.0024796076315615487, 0.0])
('K = ', [445.8264532700091, 0.0, 648.7442426351295, 0.0, 415.8229406323779, 366.93550568725846, 0.0, 0.0, 1.0])
('R = ', [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
('P = ', [-121.93794250488281, 0.0, 1278.1032029572089, 0.0, 0.0, 777.3639526367188, -9.610130516863933, 0.0, 0.0, 0.0, 1.0, 0.0])
None
# oST version 5.0 parameters


[image]

width
1280

height
800

[narrow_stereo]

camera matrix
445.826453 0.000000 648.744243
0.000000 415.822941 366.935506
0.000000 0.000000 1.000000

distortion
-0.005801 -0.015773 0.001283 0.002480 0.000000

rectification
1.000000 0.000000 0.000000
0.000000 1.000000 0.000000
0.000000 0.000000 1.000000

projection
-121.937943 0.000000 1278.103203 0.000000
0.000000 777.363953 -9.610131 0.000000
0.000000 0.000000 1.000000 0.000000
