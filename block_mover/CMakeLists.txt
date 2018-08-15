
cmake_minimum_required(VERSION 2.8.3)
project(block_mover)

find_package(catkin REQUIRED COMPONENTS
  rospy
  rospack
  baxter_core_msgs
  baxter_gazebo
  baxter_interface
  baxter_tools
  baxter_tools
  gazebo_ros
  gazebo_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
  rospy
  rospack
  baxter_core_msgs
  baxter_gazebo
  baxter_interface
  baxter_tools
  baxter_tools
  gazebo_ros
  gazebo_msgs
  visualization_msgs
)


#############
## Install ##
#############

install(PROGRAMS
   scripts/block_finder.py
   scripts/block_picker.py
   scripts/block_spawner.py
   scripts/camera_controller.py
   scripts/edge_finder.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir launch models)
   install(DIRECTORY ${dir}/
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

