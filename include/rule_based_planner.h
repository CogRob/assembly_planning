#ifndef RULE_PLANNER_H
#define RULE_PLANNER_H

#include <iostream>
#include "lego_world.h"
#include "meta_reasoner.h"

enum PrimitiveActions { DETECT, ALIGN, TRANSPORT, RETRACT, PICKUP, PLACE };
enum ConstraintType { POSE, OBJECT, BOTH };
enum PlanStatus { COMPLETE, PARTIAL, UNKNOWN };

struct ActionConstraints {
  ConstraintType type;
  geometry_msgs::PoseStamped goal_pose;
  ObjectDesc goal_object;
};

struct PlanGoal {
  PrimitiveActions action;
  ActionConstraints constraint;
};

struct PreConditions {
  // encodes the pre-conditions for the actions and methods
};

class RulePlanner {
 private:
  friend class MetaReasoner;

 public:
  RulePlanner(/* args */);
  ~RulePlanner();

  PlanGoal get_goal();
  PreConditions get_pre_conditions();
  PlanStatus make_plan(std::string task_goal, EnvState current_state);
  PreConditions get_diff();
  // meta-reasoner would need to get the affected plan
};

RulePlanner::RulePlanner(/* args */) {}

RulePlanner::~RulePlanner() {}

#endif /* RULE_PLANNER_H */