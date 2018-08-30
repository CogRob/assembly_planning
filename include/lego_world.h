#ifndef LEGO_WORLD_H
#define LEGO_WORLD_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <iostream>
#include <vector>

enum ObjectOccupancy { INVENTORY, WORKSPACE, UNKNOWN };
enum ObjectColor { RED, GREEN, BLUE, YELLOW, GREY };

struct EnvState {
  /* data */
  std::vector<ObjectDesc> object_list;
  std::vector<ObjectState> object_state_list;
};

struct ObjectDesc {
  ObjectDesc() : length(0), width(0), color(ObjectColor::GREY) {}
  int length;
  int width;
  ObjectColor color;
};

struct ObjectState {
  ObjectState()
      : pose(geometry_msgs::Pose2D()),
        pose_set(false),
        location(ObjectOccupancy::UNKNOWN) {}
  geometry_msgs::Pose2D pose;
  bool pose_set;
  ObjectOccupancy location;
};

#endif /* LEGO_WORLD_H */