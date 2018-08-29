#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <iostream>

enum PrimitiveActions { DETECT, ALIGN, TRANSPORT, RETRACT, PICKUP, PLACE };
enum ConstraintType { POSE, OBJECT, BOTH };

struct EnvState {
  /* data */
};

struct ObjectDesc {
  /* data */
};