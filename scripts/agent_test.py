#!/usr/bin/env python
"""
Borrowed parts from: RSDK Inverse Kinematics Pick and Place Example

Creates an object of type Agent, initializes and tests various primitive
action implementations
"""
from metareasoning_agent import agent
from metareasoning_agent import knowledge_base as kb

import rospy
from geometry_msgs.msg import Point, Pose



def main():
    # create a rosnode
    rospy.init_node("agent_test", log_level=rospy.DEBUG)

    # create an agent with the intent to control right arm
    test_agent = agent.Agent('right')   #pylint: disable=C0103
    test_agent.subscribe()

    fake_action_list = [
        {'action': kb.PrimitiveActions.detect,       
        'constraints': agent.Constraints(),
        'desc': "Detecting block locations with top camera" 
        },
        {'action': kb.PrimitiveActions.transport,    
         'constraints': agent.Constraints(block=kb.Block(length=4, width=1, color="green"), 
                                          position=None, 
                                          orientation=0),
         'desc': "Moving towards 1x4 green",
        },

        {'action': kb.PrimitiveActions.align,        
         'constraints': agent.Constraints(block=kb.Block(length=4, width=1, color="green"),
                                          position=None,
                                          orientation=0),
         'desc': "Aligning with 1x4 green",
        },

        {'action': kb.PrimitiveActions.pick,         
         'constraints': agent.Constraints(),
         'desc': "Picking 1x4 green"
        },
        
        {'action': kb.PrimitiveActions.retract,         
         'constraints': agent.Constraints(),
         'desc': "Retracting..."
        },

        {'action': kb.PrimitiveActions.transport,
         'constraints': agent.Constraints(block=None,
                                          position=Point(x=0.75, y = -0.3, z = 0.0),
                                          orientation=0),
         'desc': "Moving 1x4 green at x=0, y=-0.3, z=-0.16",
        },

        {'action': kb.PrimitiveActions.align, 
         'constraints': agent.Constraints(block=None, 
                                          position=None, 
                                          orientation=0),
         'desc': "Aligning 1x4 green wiht block_orientation 0",
        },

        {'action': kb.PrimitiveActions.place, 
         'constraints': agent.Constraints(),
         'desc': "Placing 1x4 green",
        },
        
        {'action': kb.PrimitiveActions.retract,         
         'constraints': agent.Constraints(),
         'desc': "Retracting..."
        },
        
        {'action': kb.PrimitiveActions.transport, 
         'constraints': agent.Constraints(block=None, 
                                          position=Point(x=0.75, y=-0.65, z=0.0),
                                          orientation=0),
         'desc': "Moving back to starting pose",
        },

        None,
        None
    ] 
    i = 0
    none_count = 0

    while(True):
        action_dict = fake_action_list[i]
        
        if(action_dict is None):
            none_count += 1
        else: 
            rospy.loginfo("Executing action: %s", action_dict['desc'])
            action = action_dict['action']
            constraints = action_dict['constraints']

            test_agent.executor(action, constraints)
            rospy.sleep(0.1)
        
        i += 1

        if(none_count == 2):
            rospy.loginfo("Training session complete!")
       

    """
    # TEST various primitive actions #
    while(True):

        rospy.loginfo("Moving to beginning pose...")

        # Transport block constraint

 
        transport_constraint = test_agent.get_current_pose()  # pylint: disable=C0103

        transport_constraint.position.x = .5
        transport_constraint.position.y = -.75
        transport_constraint.position.z = 0
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
            

        test_agent.executor(kb.PrimitiveActions.transport, transport_constraint)
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
        new_pose = test_agent.extern_align(block_color, block_length, block_width, "major")

        
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

        while(z_curr >= -0.28):
            transport_constraint.position.z = z_curr
            
            z_curr -= 0.04

            test_agent.executor(kb.PrimitiveActions.transport, transport_constraint)

        # Retrieve the current pose of end effector
        transport_constraint = test_agent.get_current_pose()
        
        # test pick (grasp block)
        rospy.logdebug("testing %s...", kb.PrimitiveActions.pick)
        test_agent.executor(kb.PrimitiveActions.pick)

        rospy.sleep(1)

        # Block should be in gripper now. Lift above the table 
        transport_constraint = test_agent.get_current_pose()
        transport_constraint.position.z = -.15
        test_agent.executor(kb.PrimitiveActions.transport, transport_constraint)

        # Set it back down 
        transport_constraint = test_agent.get_current_pose()

        transport_constraint.position.z = -0.28
        test_agent.executor(kb.PrimitiveActions.transport, transport_constraint)

        # test place (release block)
        rospy.logdebug("testing %s...", kb.PrimitiveActions.place)
        test_agent.executor(kb.PrimitiveActions.place)

        # Raise gripper back above the table
        transport_constraint.position.z = 0
        test_agent.executor(kb.PrimitiveActions.transport, transport_constraint)

        rospy.sleep(10)

        rospy.loginfo("Completed pick and place, starting loop again...")
    """

if __name__ == '__main__':
    main()
