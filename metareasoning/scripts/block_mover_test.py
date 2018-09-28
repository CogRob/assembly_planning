#!/usr/bin/env python
"""
Borrowed parts from: RSDK Inverse Kinematics Pick and Place Example

Creates an object of type Agent, initializes and tests various primitive
action implementations
"""
from metareasoning import agent
from metareasoning import knowledge_base as kb

import rospy
from geometry_msgs.msg import Point, Pose


def main():
    # create a rosnode
    rospy.init_node("block_mover_test", log_level=rospy.DEBUG)

    # TEST various primitive actions #
    while (True):
        rospy.loginfo("Moving to beginning pose...")
        # create an agent with the intent to control right arm
        test_agent = agent.Agent('right')  #pylint: disable=C0103
        test_agent.subscribe()

        # Transport block constraint

        transport_constraint = test_agent.get_current_pose()  # pylint: disable=C0103

        transport_constraint.position.x = .5
        transport_constraint.position.y = -.75
        transport_constraint.position.z = 0
        test_agent.executor(kb.PrimitiveActions.transport,
                            transport_constraint)

        # Get pixel locations from perception module
        test_agent.update_block_locations()

        inv_block_count = len(test_agent.inventory)
        print("There are ", inv_block_count, " blocks in view of top camera.")

        # Print out the blocks
        i = 0
        for block in test_agent.inventory:
            print("Block ", i, ": ", str(block))
            i += 1

        block_num = input("Which block would you like to pick up?")
        chosen_block = test_agent.inventory[block_num]

        block_location = chosen_block.pose
        #response = input("Block location is " + str(block_location) + ". Is this reasonable?")

        rospy.loginfo("Moving above block %d", block_num)

        block_color = chosen_block.color
        block_length = chosen_block.length
        block_width = chosen_block.width

        transport_constraint = test_agent.get_current_pose()  # pylint: disable=C0103

        z_curr = -0.16
        transport_constraint.position.x = block_location.x
        transport_constraint.position.y = block_location.y
        transport_constraint.position.z = z_curr

        test_agent.executor(kb.PrimitiveActions.transport,
                            transport_constraint)
        #waiting = input("Waiting... press enter when ready")
        rospy.sleep(1)

        # Align gripper so that it is perpindicular to major axis
        # _______
        # |     |
        #
        #    B
        #    B
        #    B

        rospy.loginfo("Aligning gripper with blocks major_axis.")
        new_pose = test_agent.extern_align(block_color, block_length,
                                           block_width, "major")

        # Align gripper so that it is perpindicular to minor axis
        # _______
        # |     |
        #
        #   BBB
        #
        # rospy.loginfo("Aligning gripper with blocks minor axis.")
        # test_agent.extern_align(block_color, block_length, block_width, block_location.theta, "major")

        rospy.sleep(10)

        transport_constraint = new_pose  # pylint: disable=C0103

        while (z_curr >= -0.28):
            transport_constraint.position.z = z_curr

            z_curr -= 0.04

            test_agent.executor(kb.PrimitiveActions.transport,
                                transport_constraint)

        # Retrieve the current pose of end effector
        transport_constraint = test_agent.get_current_pose()

        # test pick (grasp block)
        rospy.logdebug("testing %s...", kb.PrimitiveActions.pick)
        test_agent.executor(kb.PrimitiveActions.pick)

        rospy.sleep(1)

        # Block should be in gripper now. Lift above the table
        transport_constraint = test_agent.get_current_pose()
        transport_constraint.position.z = -.15
        test_agent.executor(kb.PrimitiveActions.transport,
                            transport_constraint)

        # Set it back down
        transport_constraint = test_agent.get_current_pose()

        transport_constraint.position.z = -0.28
        test_agent.executor(kb.PrimitiveActions.transport,
                            transport_constraint)

        # test place (release block)
        rospy.logdebug("testing %s...", kb.PrimitiveActions.place)
        test_agent.executor(kb.PrimitiveActions.place)

        # Raise gripper back above the table
        transport_constraint.position.z = 0
        test_agent.executor(kb.PrimitiveActions.transport,
                            transport_constraint)

        rospy.sleep(10)

        rospy.loginfo("Completed pick and place, starting loop again...")


if __name__ == '__main__':
    main()
