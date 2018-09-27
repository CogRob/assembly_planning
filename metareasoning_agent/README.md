# Meta-reasoning Enabled Block Building

Top-level directory for the Block building project which uses meta-reasoning
to:

# Milestones:
1. Learn corrections from human users for discrepancy resolution
2. Use experience to model self-inability and ask help from human 
3. Use meta-reasoning to generalize corrections to more instances of the
 discrepancy

# Todo:
- [ ] Use navigator buttons to switch from LEARN to EXECUTE mode
- [ ] Use navigator OK button to trigger capture_state()
- [ ] Write new NNPlanner for above Observation
- [ ] Write Learner which only storess sequence and pose of blocks
- [ ] Write Monitor which compares state after method execution to expectation
- [ ] Write service for rendering scene in Gazebo when monitor detects fault
- [ ] Write method to relay to above service the last state and tried action/
method which broke the task
- [ ] Write detector for detecting learnt sub-structure in new task
- [ ] Write Kinesthetic Learner which learns trajectory and other state info
when human teaches kinesthetically


## Future

1. Enable collaboration
