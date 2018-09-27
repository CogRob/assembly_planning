#!/usr/bin/env python
from metareasoning import agent
from metareasoning import knowledge_base as kb

import rospy
from geometry_msgs.msg import Point, Pose2D

import networkx.algorithms.isomorphism as iso
import networkx as nx

import math


def generate_pick_and_place(block, location, pick_orientation,
                            place_orientation):
    resting_position = Point(x=0.75, y=-.65, z=0.0)

    return [
        {
            'action': kb.PrimitiveActions.detect,
            'constraints': agent.Constraints(),
            'desc': "Detecting block locations with top camera"
        },
        {
            'action':
            kb.PrimitiveActions.transport,
            'constraints':
            agent.Constraints(
                block=block, position=None, orientation=pick_orientation),
            'desc':
            "Moving towards " + str(block),
        },
        {
            'action':
            kb.PrimitiveActions.align,
            'constraints':
            agent.Constraints(
                block=block, position=None, orientation=pick_orientation),
            'desc':
            "Aligning with " + str(block) + " with orientation " +
            str(pick_orientation)
        },
        {
            'action': kb.PrimitiveActions.pick,
            'constraints': agent.Constraints(),
            'desc': "Picking " + str(block),
        },
        {
            'action': kb.PrimitiveActions.retract,
            'constraints': agent.Constraints(),
            'desc': "Retracting..."
        },
        {
            'action':
            kb.PrimitiveActions.transport,
            'constraints':
            agent.Constraints(
                block=None, position=location, orientation=place_orientation),
            'desc':
            "Moving " + str(block) + " to " + str(location)
        },
        {
            'action':
            kb.PrimitiveActions.align,
            'constraints':
            agent.Constraints(
                block=None, position=None, orientation=place_orientation),
            'desc':
            "Aligning " + str(block) + "with orientation " +
            str(place_orientation),
        },
        {
            'action': kb.PrimitiveActions.place,
            'constraints': agent.Constraints(),
            'desc': "Placing " + str(block)
        },
        {
            'action': kb.PrimitiveActions.retract,
            'constraints': agent.Constraints(),
            'desc': "Retracting...",
        },
        {
            'action':
            kb.PrimitiveActions.transport,
            'constraints':
            agent.Constraints(
                block=None, position=resting_position, orientation=0),
            'desc':
            "Moving back to starting pose",
        },
    ]


def main():
    # create a rosnode
    rospy.init_node("agent_test", log_level=rospy.DEBUG)

    # create an agent with the intent to control right arm
    test_agent = agent.Agent('right')  # pylint: disable=C0103
    test_agent.subscribe()

    #    B
    # _ BB
    #   BB
    #    B Actual Pyramid
    """
    green_1x4 = kb.Block(length=4, width=1, color="green")
    green_1x4_final_position = Point(x=0.735, y=-0.287, z=0.0)
    blue_1x2 = kb.Block(length=2, width=1, color="blue")
    blue_1x2_final_position = Point(x=0.738, y=-0.240, z=0.0)

    green_1x4_pnp = generate_pick_and_place(
        green_1x4, green_1x4_final_position, 0, 0)

    blue_1x2_pnp = generate_pick_and_place(
        blue_1x2, blue_1x2_final_position, 0, 1)

    fake_action_list = green_1x4_pnp + blue_1x2_pnp + [None]

    green_1x4_desired = Pose2D(x=0.729, y=-0.273, theta=0.0)
    blue_1x2_desired = Pose2D(x=0.723, y=-0.321, theta=math.pi/2)

    green_1x4_block = kb.Block(length=4, width=1, color="green",
                               pose=green_1x4_desired)
    blue_1x2_block = kb.Block(
        length=2, width=1, color="blue", pose=blue_1x2_desired)
    """

    # BBB
    #  B
    #  B
    #  B
    #  B
    # BBB  Barbell
    green_1x2 = kb.Block(length=2, width=1, color="green")
    green_1x2_final_position = Point(x=0.554, y=-0.239, z=0.0)

    blue_1x4 = kb.Block(length=4, width=1, color="blue")
    blue_1x4_final_position = Point(x=.637, y=-0.242, z=0.0)

    red_1x2 = kb.Block(length=2, width=1, color="red")
    red_1x2_final_position = Point(x=0.728, y=-0.247, z=0.0)

    green_1x2_pnp = generate_pick_and_place(green_1x2,
                                            green_1x2_final_position, 0, 1)

    blue_1x4_pnp = generate_pick_and_place(blue_1x4, blue_1x4_final_position,
                                           0, 0)

    red_1x2_pnp = generate_pick_and_place(red_1x2, red_1x2_final_position, 0,
                                          1)

    fake_action_list = green_1x2_pnp + red_1x2_pnp + blue_1x4_pnp

    green_1x2_desired = Pose2D(x=0.807, y=-0.245, theta=math.pi / 2)
    blue_1x4_desired = Pose2D(x=0.729, y=-0.241, theta=0)
    red_1x2_desired = Pose2D(x=0.653, y=-0.232, theta=math.pi / 2)

    green_1x2_block = kb.Block(
        length=4, width=1, color="green", pose=green_1x2_desired)
    blue_1x4_block = kb.Block(
        length=4, width=1, color="blue", pose=blue_1x4_desired)
    red_1x2_block = kb.Block(
        length=2, width=1, color="red", pose=red_1x2_desired)

    expectation = kb.EnvState()
    expectation.add_block(green_1x2_block)
    expectation.add_block(blue_1x4_block)
    expectation.add_block(red_1x2_block)

    errortolerancexy = 0.010
    errortolerancetheta = 0.087

    node_condition1 = iso.categorical_node_match(['length', 'width'], [1, 1])
    node_condition2 = iso.numerical_edge_match(['pose_x', 'pose_y'], [0, 0],
                                               rtol=errortolerancexy)
    node_condition3 = iso.numerical_edge_match(['pose_theta'], [0],
                                               rtol=errortolerancetheta)

    i = 0
    none_count = 0
    while (True):
        action_dict = fake_action_list[i]

        if (action_dict is None):
            none_count += 1
        else:
            rospy.loginfo("Executing action: %s", action_dict['desc'])
            action = action_dict['action']
            constraints = action_dict['constraints']

            test_agent.executor(action, constraints)
            rospy.sleep(0.1)

        i += 1

        if (none_count == 1):
            rospy.loginfo("Finished block PNP!")
            rospy.loginfo("Adding new block to WS EnvState")

        if (none_count == 2):
            rospy.loginfo("Task complete")
            break

    expectation.print_graph()
    test_agent.get_ws().print_graph()
    result1 = nx.is_isomorphic(
        expectation.ws_state,
        test_agent.get_ws().ws_state,
        node_match=node_condition1)
    result2 = nx.is_isomorphic(
        expectation.ws_state,
        test_agent.get_ws().ws_state,
        node_match=node_condition2)
    result3 = nx.is_isomorphic(
        expectation.ws_state,
        test_agent.get_ws().ws_state,
        node_match=node_condition3)
    result = result1 and result2 and result3

    print("Result 1: ", result1, " and Result 2: ", result2, " Result 3: ",
          result3, " and sum ", result)


if __name__ == "__main__":
    main()
