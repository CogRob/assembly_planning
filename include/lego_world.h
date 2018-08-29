#ifndef LEGO_WORLD_H
#define LEGO_WORLD_H

#include <vector>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <iostream>

enum ObjectOccupancy {INVENTORY, WORKSPACE };
enum ObjectColor {}

struct EnvState {
  /* data */
  std::vector<ObjectDesc> object_list;
  std::vector<ObjectState> object_state_list;
};

struct ObjectDesc {
  int length;
  int width;
  ObjectColor color;
};

struct ObjectState
{
  geometry_msgs::Pose2D pose;
  ObjectOccupancy location;
};

#endif /* LEGO_WORLD_H */