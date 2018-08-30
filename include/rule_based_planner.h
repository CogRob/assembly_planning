#ifndef RULE_PLANNER_H
#define RULE_PLANNER_H

#include <iostream>
#include <string>
#include <vector>
#include "lego_world.h"
#include "meta_reasoner.h"

enum PrimitiveActions { DETECT, ALIGN, TRANSPORT, RETRACT, PICKUP, PLACE };
enum ConstraintType { POSE, OBJECT, BOTH, NONE };
enum PlanStatus { COMPLETE, PARTIAL, UNKNOWN };

struct ActionConstraints {
  ConstraintType type;
  geometry_msgs::Pose2D goal_pose;  // in the table's local frame of reference
  // environment goal pose is converted to world/agent's frame by the
  // lower-level motion planner TODO: in agent.cc
  ObjectDesc goal_object;
};

struct PlanGoal {
  PrimitiveActions action;
  ActionConstraints env_constraint;
};

struct PreConditions {
  EnvState env_pre_condition;
  AgentState agent_pre_condition;
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