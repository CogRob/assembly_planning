#ifndef LEGO_WORLD_H
#define LEGO_WORLD_H

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <iostream>
#include <set>
#include <unordered_map>

// environment-specific constants
enum ObjectOccupancy { INVENTORY, WORKSPACE, UNKNOWN };
enum ObjectColor { RED, GREEN, BLUE, YELLOW, GREY };

// planner-specific constants
enum PrimitiveActions { DETECT, ALIGN, TRANSPORT, RETRACT, PICKUP, PLACE };
enum ConstraintType { POSE, OBJECT, BOTH, NONE };
enum PlanStatus { COMPLETE, PARTIAL, UNKNOWN };

// environment description data structures
struct EnvState {
  std::unordered_map<ObjectType, std::vector<ObjectInstance>> object_list;
  std::set<ObjectType> workspace, inventory;
  // TODO: write a comparison where keys are compared and if all keys exist then
};

struct ObjectType {
  ObjectType() : length(0), width(0), color(ObjectColor::GREY) {}
  unsigned int length;
  unsigned int width;
  ObjectColor color;

  bool operator==(const ObjectType& rhs) const {
    return ((length == rhs.length) & (width == rhs.width) &
            (color == rhs.color));
  }
  bool operator<(const ObjectType& rhs) const {
    return (width < rhs.width) ||
           (!(width < rhs.width) && (length < rhs.length)) &&
               (!(width < rhs.width) && !(length < rhs.length) &&
                (color < rhs.color));
  }
};

struct ObjectInstance {
  ObjectInstance()
      : id(0),
        pose(geometry_msgs::Pose2D()),
        location(ObjectOccupancy::UNKNOWN) {}
  unsigned int id;
  geometry_msgs::Pose2D pose;
  ObjectOccupancy location;

  // == operator: Only compares object-level info
  bool operator==(const ObjectInstance& rhs) const {
    return (this->location == rhs.location);
  }
};
struct AgentState {
  AgentState() : gripper_state(false) {}
  bool gripper_state;
  geometry_msgs::Pose gripper_pose;
  ObjectType obj_in_gripper;  // depends on state of gripper

  // == operator: Only compares object-level info
  bool operator==(const AgentState& rhs) const {
    bool tmp = this->gripper_state == rhs.gripper_state;
    if (tmp == true && this->gripper_state == 0) {
      tmp = tmp & this->obj_in_gripper == rhs.obj_in_gripper;
    }
    return tmp;
  }
};

struct AgentInternalState {
  PlanGoal current_goal;
  Expectation current_expectation;
};

// planner data structures
struct ActionConstraints {
  ConstraintType type;
  geometry_msgs::Pose2D goal_pose;  // in the table's local frame of reference
  // environment goal pose is converted to world/agent's frame by the
  // lower-level motion planner TODO: in agent.cc
  ObjectType goal_object;
};

struct PlanGoal {
  PrimitiveActions action;
  ActionConstraints env_constraint;
};

struct PreConditions {
  // we assume all pre-conditions are at the abstract level of objects
  // and not at the scope of world pose <--> object relationships
  EnvState env_pc;
  AgentState agent_pc;
};

// meta-reasoner data structures
struct Expectation {
  /* data */
};

#endif /* LEGO_WORLD_H */