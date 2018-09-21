# lego_building

### TODO
### Bug Fixes
* Fix the block angle issues DONE
* Fix Erosion DONE

### Making Code More Robust
* Lighting issues
    * Either move baxter to corner of lab and mount Kinect on head OR
    * Diffuse the Fluorescent lights in lab center Baxter with tarp DONE

* Create a debugging flag to remove all components not necessary for functionality to improve framerate of block finder



### Code Organization
* Split block_finder into two modules: top_block_detector.py and hand_block_detector.py (These can both inherit from a BlockDetector Class) DONE
* Split camera_controller.py (James' project) from block_finder/ mover code IN PROGRESS
* Split code for simulation and code for robot better

### Ease of Use Improvements
* Create a launch file for top and hand BlockDetectors DONE
* top_block_detector.launch: Launches OpenNI2 driver, Kinect to Baxter's base TF broadcaster (top_to_gripper_bc), and runs top camera block detector
Launch openni2 to publish ASUS camera data
```
roslaunch openni2_launch openni2.launch
```
Publish the TF from extrinsic calibration
```
roslaunch easy_handeye publish.launch
```
Run top camera block detector
```
rosrun block_mover top_block_detector.py

* hand_block_detector.launch:
Set Baxter's hand camera resolution to 1280x800 
```
rosrun baxter_tools camera_tools.py -o "right_hand_camera" -r 1280x800
```
Run hand camera block detector
```
rosrun block_mover hand_block_detector.py
```

* Add a functionality to switch the workspace and the inventory so we don't need to reset inventory after each mission

* Add instructions for use to this README

### Nice to Haves
* Make RViz Block markers Look like blocks 





