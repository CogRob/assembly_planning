#include "meta_reasoner.h"
#include <iostream>

void MetaReasoner::PlanMonitor() {
  // code to match real_phys_state with next_goal_conditions.agent_pc
  mismatch_flag = (real_phys_state == next_goal_conditions.agent_pc);
  // code to match real_env_state with next_goal_conditions.env_pc
  mismatch_flag =
      mismatch_flag & (real_env_state == next_goal_conditions.env_pc);
  return;
}