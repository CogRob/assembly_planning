#!/usr/bin/env python
"""
Borrowed parts from: RSDK Inverse Kinematics Pick and Place Example

Creates an object of type Agent, initializes and tests various primitive
action implementations
"""
from metareasoning_agent import agent
from metareasoning_agent import knowledge_base as kb

import rospy
from geometry_msgs.msg import Point


def main():
    # create a rosnode
    rospy.init_node("agent_test", log_level=rospy.DEBUG)

    # create an agent with the intent to control right arm
    test_agent = agent.Agent('right')   #pylint: disable=C0103

    # TEST various primitive actions #
    # First update map
    test_agent.update_block_locations()

    inv_block_count = len(test_agent.inv_state)    
    print("There are ", inv_block_count, " blocks in view of top camera.")

    # Failsafe  TODO: remove later
    if(inv_block_count != 8):
        rospy.loginfo("There are not 8 blocks, something is wrong... Exiting!")

    i = 0
    for block in test_agent.inv_state:
        print("Block ", i, ": ", str(block))
        i += 1


    block_num = input("Which block would you like to pick up?")

    rospy.loginfo("Picking up %d", block_num)

    picked_block = test_agent.inv_state[block_num]

    block_location = picked_block.pose
    
    response = input("Block location is " + str(block_location) + ". Is this reasonable?")

    rospy.loginfo("Moving block...")

    transport_constraint = Point()  # pylint: disable=C0103

    transport_constraint.x = block_location.x
    transport_constraint.y = block_location.y
    transport_constraint.z = 0
    test_agent.executor(kb.PrimitiveActions.transport, transport_constraint)


    transport_constraint.x = block_location.x
    transport_constraint.y = block_location.y
    transport_constraint.z = -0.28
    test_agent.executor(kb.PrimitiveActions.transport, transport_constraint)
    
    # test pick
    rospy.logdebug("testing %s...", kb.PrimitiveActions.pick)
    test_agent.executor(kb.PrimitiveActions.pick)

    transport_constraint.x = block_location.x
    transport_constraint.y = block_location.y
    transport_constraint.z = -0.28
    test_agent.executor(kb.PrimitiveActions.transport, transport_constraint)
    
    # rospy.logdebug("testing %s with object constraint",
    #                kb.PrimitiveActions.transport)
    # # TODO: add an object to the env of Agent
    # transport_constraint = kb.Block()
    # # TODO: fill in the object constraint
    # agent.executor(kb.PrimitiveActions.transport, transport_constraint)

    # test place
    rospy.logdebug("testing %s...", kb.PrimitiveActions.place)
    test_agent.executor(kb.PrimitiveActions.place)

    # test align
if __name__ == '__main__':
    main()
