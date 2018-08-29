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
  AgentGoal current_goal;
};

struct AgentExpectation {
  /* data */
};

// TODO
class Agent {
 private:
  /*  data  */
  AgentExpectation current_expectation;
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
  AgentExpectation get_expectations();
  Agent(/* args */);
  ~Agent();
};

Agent::Agent(/* args */) {}

Agent::~Agent() {}

#endif /* META_AGENT_H */