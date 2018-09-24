#ifndef META_AGENT_H
#define META_AGENT_H

#include <baxter_core_msgs/SolvePositionIK.h>
#include <baxter_core_msgs/SolvePositionIKRequest.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <metareasoning_agent/lego_world.hh>
#include <metareasoning_agent/meta_reasoner.hh>
#include <metareasoning_agent/rule_based_planner.hh>

// TODO: this is the interfacing layer between agent-agnostic planner and the
// agent, i.e. Baxter
class Agent {
 private:
  /*  data  */
  AgentState current_phys_state;
  AgentInternalState current_int_state;
  /*  methods */
  // primitive actions
  bool actuation_result;
  bool detect();
  bool align(ActionConstraints);
  bool retract();
  bool transport(ActionConstraints);
  bool pickup();
  bool place();

 public:
  AgentState get_state();
  Expectation get_expectations();
  void executor(PrimitiveActions, ActionConstraints);
  Agent(/* args */);
  ~Agent();
};

Agent::Agent(/* args */) {}

Agent::~Agent() {}

#endif /* META_AGENT_H */