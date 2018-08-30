#ifndef META_REASONER_h
#define META_REASONER_h

#include "lego_world.h"

class MetaReasoner {
 private:
  /* data */
  EnvState real_env_state;
  AgentState real_phys_state, exp_phys_state;
  AgentInternalState real_int_state, exp_int_state;
  PreConditions curr_goal_conditions, next_goal_conditions;
  Expectation curr_goal_expectations, next_goal_expectations;
  bool mismatch_flag;
  // need something to store the name of method-->action trace where
  // expectations mismatched, additionally would also need a data structure
  // to store if symbols mismatched or occupancy-grid, and if former which
  // symbols?

 public:
  MetaReasoner(/* args */);
  ~MetaReasoner();
  void PlanMonitor();
};

MetaReasoner::MetaReasoner(/* args */) {}

MetaReasoner::~MetaReasoner() {}

#endif