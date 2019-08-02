# Installation Instructions for UR5 in Gazebo with RF85 Gripper

1. Download UR's moveit-config, kinematics and gazebo
    ``` bash
    sudo apt-get install \
       ros-kinetic-ur-gazebo \
    ros-kinetic-ur5-moveit-config \
    ros-kinetic-ur-kinematics
    ```
2. Go to `~/catkin_ws/src` type:
    ``` bash
    git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
    git clone https://github.com/crigroup/robotiq.git
    ```
3. Copy the `robotiq_description` from robotiq to the src and delete the rest
4. Go to `robotiq_description/urdf` and open `robotiq_85_gripper.transmission.xacro`.
5. Replace all `libgazebo_mimic_joint_plugin.so` to `libroboticsgroup_gazebo_mimic_joint_plugin.so`
6. Type `roscd ur_gazebo/controller` and create a file called:
  `gripper_controller_robotiq.yaml` with following content
  ``` yaml
  gripper_controller:
   type: position_controllers/JointTrajectoryController
   joints:
      - robotiq_85_left_knuckle_joint
   constraints:
       goal_time: 0.6
       stopped_velocity_tolerance: 0.05
       robotiq_85_left_knuckle_joint: {trajectory: 0.1, goal: 0.1}
   stop_trajectory_duration: 0.5
   state_publish_rate:  25
   action_monitor_rate: 10
   ```
7. Type `roscd ur_description/robot.urdf` and add following content to robot tag
  ``` xml
   <!-- gripper -->
   <xacro:include filename="$(find robotiq_description)/urdf/robotiq_85_gripper.urdf.xacro" />
   <!-- include camera -->
   <xacro:include filename="/path/to/camera.urdf.xacro" />
   <!-- include arm camera -->
   <xacro:include filename="/path/to/arm_camera.urdf.xacro" />
   <xacro:robotiq_85_gripper prefix="" parent="ee_link" >
     <origin xyz="0 0 0" rpy="0 0 0"/>
   </xacro:robotiq_85_gripper>
   <!-- camera -->
   <xacro:camera/>
   <!-- arm camera -->
   <xacro:arm_camera/>
   <joint name="arm_camera_joint" type="fixed">
     <parent link="ee_link"/>
     <child link = "arm_camera__link"/>
     <origin xyz="0.02 0.0 0.05" rpy="0.0 0.0 0.0" />
   </joint>
   <joint name="world_camera" type="fixed">
     <parent link="world" />
     <child link = "camera__link" />
     <origin xyz="0.75 -0.4 1.65" rpy="0.0 0.7 2.6" />
   </joint>
  ```
8. Run `roslaunch ur_gazebo ur5.launch` for testing whether the gripper is on the arm.

# Configuring MoveIt to work with Robotiq RF85 Gripper

1. Type `roscd ur5_moveit_config/config` and open "controllers.yaml" and add following
content at the end:
  ``` yaml
   - name: "gripper_controller"
     action_ns: follow_joint_trajectory
     type: FollowJointTrajectory
     joints:
       - robotiq_85_left_knuckle_joint
  ```
2. Open `ur5.srdf` file and add content:
  ``` xml
<joint name="robotiq_85_left_knuckle_joint" /> to the group "manipulator"
  ```
3. Add following group to robot
  ``` xml
 <group name="gripper">
     <link name="robotiq_85_base_link"/>
     <link name="robotiq_85_left_inner_knuckle_link"/>
     <link name="robotiq_85_left_finger_tip_link"/>
     <link name="robotiq_85_left_knuckle_link"/>
     <link name="robotiq_85_left_finger_link"/>
     <link name="robotiq_85_right_inner_knuckle_link"/>
     <link name="robotiq_85_right_finger_tip_link"/>
     <link name="robotiq_85_right_knuckle_link"/>
     <link name="robotiq_85_right_finger_link"/>
 </group>
  ```
4. Add tag 
  ``` xml
<joint name="robotiq_85_left_knuckle_joint" value="0" />
  ```
  to
  ``` xml
<group_state name="home" group="manipulator">
<group_state name="up" group="manipulator">
  ```
5. Add the end_effector to:
  ``` xml
<end_effector name="moveit_ee" parent_link="ee_link" group="gripper" parent_group="endeffector" />
  ```                                                                                        
  and disable the collisions depending on your need.
  
6. Type:
``` bash
roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true
```
There should be (maybe warnings) but no errors

Written by: [Jiaming](https://github.com/jih189)
