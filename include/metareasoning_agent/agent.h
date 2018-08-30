#ifndef META_AGENT_H
#define META_AGENT_H

#include <iostream>
#include "lego_world.h"
#include "meta_reasoner.h"
#include "rule_based_planner.h"

// TODO:
class Agent {
 private:
  /*  data  */
  AgentState current_phys_state;
  AgentInternalState current_int_state;
  /*  methods */
  // primitive actions
  bool detect();
  bool align(ActionConstraints);
  bool retract();
  bool transport(ActionConstraints);
  bool pickup();
  bool place();

 public:
  AgentState get_state();
  Expectation get_expectations();
  Agent(/* args */);
  ~Agent();
};

Agent::Agent(/* args */) {}

Agent::~Agent() {}

#endif /* META_AGENT_H */