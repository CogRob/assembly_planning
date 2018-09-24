#ifndef RULE_PLANNER_H
#define RULE_PLANNER_H

#include <iostream>
#include <string>
#include <vector>
#include "lego_world.hh"
#include "meta_reasoner.hh"

class RulePlanner {
 private:
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