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
    test_agent.subscribe()

    # TEST various primitive actions #
    while(True):
        rospy.loginfo("Moving to beginning pose...")

        transport_constraint = Point()  # pylint: disable=C0103
 
        transport_constraint.x = .5
        transport_constraint.y = -.75
        transport_constraint.z = 0
        test_agent.executor(kb.PrimitiveActions.transport, transport_constraint)

        # Get pixel locations from perception module
        test_agent.update_block_locations()

        inv_block_count = len(test_agent.inv_state)    
        print("There are ", inv_block_count, " blocks in view of top camera.")

        # Print out the blocks
        i = 0
        for block in test_agent.inv_state:
            print("Block ", i, ": ", str(block))
            i += 1


        block_num = input("Which block would you like to pick up?")
        chosen_block = test_agent.inv_state[block_num]

        block_location = chosen_block.pose
        response = input("Block location is " + str(block_location) + ". Is this reasonable?")

        rospy.loginfo("Moving above block %d", block_num)

        block_color = chosen_block.color
        block_length = chosen_block.length
        block_width = chosen_block.width

        transport_constraint = Point()  # pylint: disable=C0103
        transport_constraint.x = block_location.x
        transport_constraint.y = block_location.y
        transport_constraint.z = 0
        
        test_agent.executor(kb.PrimitiveActions.transport, transport_constraint)
        
        rospy.sleep(1)

        # Align gripper so that it is perpindicular to major axis
        # _______
        # |     |
        #
        #    B
        #    B
        #    B
        
        rospy.loginfo("Aligning gripper with blocks major_axis.")
        test_agent.extern_align(block_color, block_length, block_width, block_location.theta, "major")
        
        # Align gripper so that it is perpindicular to minor axis
        # _______
        # |     |
        #
        #   BBB
        #   
        # rospy.loginfo("Aligning gripper with blocks minor axis.")
        # test_agent.extern_align(block_color, block_length, block_width, block_location.theta, "major")

        rospy.sleep(5)

        # Retrieve the current pose of end effector
        curr_pose = test_agent.get_current_pose()

        new_x = curr_pose['position'].x
        new_y = curr_pose['position'].y
        new_z = -0.28

        transport_constraint.x = new_x
        transport_constraint.y = new_y
        transport_constraint.z = new_z

        test_agent.executor(kb.PrimitiveActions.transport, transport_constraint)
        
        # test pick (grasp block)
        rospy.logdebug("testing %s...", kb.PrimitiveActions.pick)
        test_agent.executor(kb.PrimitiveActions.pick)

        # Block should be in gripper now. Lift above the table 
        transport_constraint.x = new_x
        transport_constraint.y = new_y
        transport_constraint.z = -.15
        test_agent.executor(kb.PrimitiveActions.transport, transport_constraint)

        # Set it back down 
        transport_constraint.x = new_x
        transport_constraint.y = new_y
        transport_constraint.z = -.28
        test_agent.executor(kb.PrimitiveActions.transport, transport_constraint)

        # test place (release block)
        rospy.logdebug("testing %s...", kb.PrimitiveActions.place)
        test_agent.executor(kb.PrimitiveActions.place)

        # Raise gripper back above the table
        transport_constraint.x = new_x
        transport_constraint.y = new_y
        transport_constraint.z = 0
        test_agent.executor(kb.PrimitiveActions.transport, transport_constraint)

        rospy.sleep(10)

        rospy.loginfo("Completed pick and place, starting loop again...")

if __name__ == '__main__':
    main()
