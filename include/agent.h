#include <iostream>
#include "lego_world.h"

struct ActionConstraints {
  ConstraintType type;
  geometry_msgs::PoseStamped goal_pose;
  ObjectDesc goal_object;
};

struct AgentState {
  AgentState() : gripper_state(false) {}
  bool gripper_state;
  geometry_msgs::Pose gripper_pose;
  AgentGoal current_goal;
  AgentExpectation expectation_dash;
};

struct AgentGoal {
  PrimitiveActions action;
  ActionConstraints constraint;
};

struct AgentExpectation {
  /* data */
};

// TODO
class Agent {
 private:
 public:
  Agent(/* args */);
  ~Agent;
};

Agent::Agent(/* args */) {}

Agent::~Agent() {}