#!/usr/bin/env python
"""
Borrowed parts from: RSDK Inverse Kinematics Pick and Place Example

Creates an object of type Agent, initializes and tests various primitive
action implementations
"""
from metareasoning_agent import agent
from metareasoning_agent import lego_world as lw

import rospy
from geometry_msgs.msg import Point

if __name__ == '__main__':
    # create a rosnode
    rospy.init_node("agent_test", log_level=rospy.DEBUG)

    # create an agent with the intent to control right arm
    agent = agent.Agent('right')  # pylint: disable=C0103

    # TEST various primitive actions #

    # test going to a pose with fixed quaternion for end-effector
    rospy.logdebug("testing %s with Pose constraint",
                   lw.PrimitiveActions.transport)
    # fill in the Pose constraint
    transport_constraint = Point()  # pylint: disable=C0103
    transport_constraint.x = 0.82
    transport_constraint.y = -0.28
    transport_constraint.z = 0
    agent.executor(lw.PrimitiveActions.transport, transport_constraint)
    rospy.sleep(5)
    transport_constraint.x = 0.77
    transport_constraint.y = 0.05
    transport_constraint.z = -0.10
    agent.executor(lw.PrimitiveActions.transport, transport_constraint)
    # rospy.logdebug("testing %s with object constraint",
    #                lw.PrimitiveActions.transport)
    # # TODO: add an object to the env of Agent
    # transport_constraint = lw.Block()
    # # TODO: fill in the object constraint
    # agent.executor(lw.PrimitiveActions.transport, transport_constraint)

    # test pick
    rospy.logdebug("testing %s...", lw.PrimitiveActions.pick)
    agent.executor(lw.PrimitiveActions.pick)

    # test place
    rospy.logdebug("testing %s...", lw.PrimitiveActions.place)
    agent.executor(lw.PrimitiveActions.place)

    # test align
