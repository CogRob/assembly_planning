#ifndef META_AGENT_H
#define META_AGENT_H

#include <iostream>
#include "lego_world.h"
#include "meta_reasoner.h"
#include "rule_based_planner.h"

struct AgentState {
  AgentState() : gripper_state(false) {}
  bool gripper_state;
  geometry_msgs::Pose gripper_pose;
  ObjectDesc obj_in_gripper;  // depends on state of gripper
  PlanGoal current_goal;
};

// TODO:
class Agent {
 private:
  /*  data  */
  AgentState current_phys_state;
  struct AgentInternalState {
    PlanGoal current_goal;
    Expectation current_expectation;
  } current_int_state;
  /*  methods */
  // primitive actions
  bool detect();
  bool align(ActionConstraints);
  bool retract();
  bool transport(ActionConstraints);
  bool pickup();
  bool place();
  friend class MetaReasoner;

 public:
  AgentState get_state();
  Expectation get_expectations();
  Agent(/* args */);
  ~Agent();
};

Agent::Agent(/* args */) {}

Agent::~Agent() {}

#endif /* META_AGENT_H */