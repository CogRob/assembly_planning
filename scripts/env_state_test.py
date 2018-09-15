#!/usr/bin/env python
from metareasoning_agent import agent
from metareasoning_agent import knowledge_base as kb

import rospy
from geometry_msgs.msg import Point


def generate_pick_and_place(block, location, pick_orientation, place_orientation):
    resting_position = resting_position = Point(x=0.75, y=-.65, z=0.0)
    return [
        {'action': kb.PrimitiveActions.detect,
            'constraints': agent.Constraints(),
            'desc': "Detecting block locations with top camera"
         },

        {'action': kb.PrimitiveActions.transport,
            'constraints': agent.Constraints(block=block,
                                             position=None,
                                             orientation=pick_orientation),
            'desc': "Moving towards " + str(block),
         },

        {'action': kb.PrimitiveActions.align,
            'constraints': agent.Constraints(block=block,
                                             position=None,
                                             orientation=pick_orientation),
            'desc': "Aligning with " + str(block) + " with orientation " + str(pick_orientation)
         },

        {'action': kb.PrimitiveActions.pick,
            'constraints': agent.Constraints(),
            'desc': "Picking " + str(block),
         },

        {'action': kb.PrimitiveActions.retract,
            'constraints': agent.Constraints(),
            'desc': "Retracting..."
         },

        {'action': kb.PrimitiveActions.transport,
            'constraints': agent.Constraints(block=None,
                                             position=location,
                                             orientation=place_orientation),
            'desc': "Moving " + str(block) + " to " + str(location)
         },

        {'action': kb.PrimitiveActions.align,
            'constraints': agent.Constraints(block=None,
                                             position=None,
                                             orientation=place_orientation),
            'desc': "Aligning " + str(block) + "with orientation " + str(place_orientation),
         },

        {'action': kb.PrimitiveActions.place,
            'constraints': agent.Constraints(),
            'desc': "Placing " + str(block)
         },

        {'action': kb.PrimitiveActions.retract,
            'constraints': agent.Constraints(),
            'desc': "Retracting..."
         },

        {'action': kb.PrimitiveActions.transport,
            'constraints': agent.Constraints(block=None,
                                             position=resting_position,
                                             orientation=0),
            'desc': "Moving back to starting pose",
         },
        None,
    ]


def main():
    # create a rosnode
    rospy.init_node("agent_test", log_level=rospy.DEBUG)

    # create an agent with the intent to control right arm
    test_agent = agent.Agent('right')  # pylint: disable=C0103
    test_agent.subscribe()

    green_1x4 = kb.Block(length=4, width=1, color="green")
    green_1x4_final_position = Point(x=0.735, y=-0.287, z=0.0)
    blue_1x2 = kb.Block(length=2, width=1, color="blue")
    blue_1x2_final_position = Point(x=0.738, y=-0.240, z=0.0)

    green_1x4_pnp = generate_pick_and_place(
        green_1x4, green_1x4_final_position, 0, 0)

    blue_1x2_pnp = generate_pick_and_place(
        blue_1x2, blue_1x2_final_position, 0, 1)

    fake_action_list = green_1x4_pnp + blue_1x2_pnp + [None]

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

        if(none_count == 1):
            rospy.loginfo("Finished block PNP!")
            rospy.loginfo("Adding new block to WS EnvState")

        if(none_count == 2):
            rospy.loginfo("Task complete")


if __name__ == "__main__":
    main()
