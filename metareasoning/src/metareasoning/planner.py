#!/usr/bin/env python
# Author: Priyam Parashar
# Simplified planner for creating plan given required change in the workspace

import networkx as nx
import rospy
from geometry_msgs.msg import Pose2D
from metareasoning.knowledge_base import (
    NonPrimitiveActions,
    PrimitiveActions,
    Block,
    Constraint,
    node2block,
)
from metareasoning.utilities import calculate_pose_diff


# routine definitions: global so that both planner and learner can access them
def acquireroutine(constraint):  # pylint: disable=no-self-use
    """
    Definition for the high-level method acquire

    input
    :block: of type knowledge_base.Block

    returns
    :action_plan: a list of tuples, where 1st element is the name of a
    PrimitiveAction and 2nd is the related constraint
    """
    action_plan = []
    action_plan.append((PrimitiveActions.transport, constraint))
    action_plan.append((PrimitiveActions.align, constraint))
    action_plan.append((PrimitiveActions.pick, Constraint()))
    action_plan.append((PrimitiveActions.retract, Constraint()))
    return action_plan


def depositroutine(constraint):  # pylint: disable=no-self-use
    """
    Definition for the high-level method deposit

    input
    :block: of type knowledge_base.Block

    returns
    :action_plan: a list of tuples, where 1st element is the name of a
    PrimitiveAction and 2nd is the related constraint
    """
    action_plan = []
    action_plan.append((PrimitiveActions.transport, constraint))
    action_plan.append((PrimitiveActions.align, constraint))
    action_plan.append((PrimitiveActions.place, Constraint()))
    action_plan.append((PrimitiveActions.retract, Constraint()))
    return action_plan


class DeltaPlanner(object):
    """
    Given change in state, returns a list of ordered actions and constraints.
    """

    def __init__(self):
        self._plan_result = False
        self._plan = None
        self._action_plan = []
        self._state_diff = None
        # FIXME only for A's execution
        self._hard_coded_plan = []
        # method2action db
        self._method2action_db = {}
        self._method2action_db[NonPrimitiveActions.acquire] = acquireroutine
        self._method2action_db[NonPrimitiveActions.deposit] = depositroutine

    def _compute_diff(self, curr, reqd):
        rospy.loginfo(list(curr.workspace.nodes(data=True)))
        rospy.loginfo(list(reqd.workspace.nodes(data=True)))
        difference = reqd.workspace.copy()
        # FIXME difference needs to be computed on the type and pose of a block
        remove_nodes = []
        for reqd_node, reqd_data in difference.nodes(data=True):
            for curr_node, curr_data in curr.workspace.nodes(data=True):
                # get block types from both nodes
                reqd_block = node2block(difference.nodes[reqd_node])
                curr_block = node2block(curr.workspace.nodes[curr_node])
                # get poses from both nodes
                reqd_pose = Pose2D(reqd_data['pose_x'], reqd_data['pose_y'],
                                   reqd_data['pose_theta'])
                curr_pose = Pose2D(curr_data['pose_x'], curr_data['pose_y'],
                                   curr_data['pose_theta'])
                pose_diff = calculate_pose_diff(reqd_pose, curr_pose)
                if reqd_block == curr_block and pose_diff.x < 0.1 and \
                   pose_diff.y < 0.1 and pose_diff.theta < 0.1:
                    remove_nodes.append(reqd_node)
                    break
        difference.remove_nodes_from(n for n in remove_nodes)
        node = difference.nodes[list(difference.nodes())[0]]
        self._state_diff = node2block(node)

    def _find_plan(self):
        # use self._state_diff of type Block to get BlockType and BlockPose
        rospy.loginfo("Finding plan for: %s", str(self._state_diff))
        block_type = self._state_diff
        block_pose = self._state_diff.pose
        constraint1 = Constraint(block=block_type, orientation=0)
        if abs(abs(self._state_diff.pose.theta) - 1.5) < 0.3:
            constraint2 = Constraint(position=block_pose, orientation=0)
        else:
            constraint2 = Constraint(position=block_pose, orientation=1)

        self._plan = [(NonPrimitiveActions.acquire, constraint1),
                      (NonPrimitiveActions.deposit, constraint2)]
        self._plan_result = True

    def _plan2action(self, plan):
        action_plan = []
        for (method, constraint) in plan:
            action_plan.extend(self._method2action_db[method](constraint))
        return action_plan

    def plan(self, curr_state, next_state):
        self._compute_diff(curr_state, next_state)
        self._find_plan()
        if self._plan_result:
            return self._plan2action(self._plan)
        else:
            return False

    def get_hard_coded_plan(self):
        """
        One-time use method for sending back actions which write A
        """
        if self._hard_coded_ptr < len(self._hard_coded_plan):
            current_plan = self._hard_coded_plan[self._hard_coded_ptr]
            self._hard_coded_ptr += 1
            return self._plan2action(current_plan)
        return None

    def setup_hard_coded_plan(self, task):
        """
        One-time use method for populating actions which write A
        """
        if task == 'A':
            constraintpose4blue = Constraint()
            block4blue = Block(4, 1, 'blue')
            constraint4blue = Constraint(block=block4blue, orientation=0)
            pose4blue = Pose2D(x=0.73, y=-0.18, theta=0.0)
            # place 1x4xblue
            constraintpose4blue.set_pose(pose4blue, 0)
            self._hard_coded_plan.append(
                [(NonPrimitiveActions.acquire, constraint4blue),
                 (NonPrimitiveActions.deposit, constraintpose4blue)])
            # place 1x4xgreen
            block4green = Block(4, 1, 'green')
            constraint4green = Constraint(block=block4green, orientation=0)
            pose4green = Pose2D(x=0.73, y=-0.07, theta=0)
            constraintpose4green = Constraint(
                position=pose4green, orientation=0)
            self._hard_coded_plan.append(
                [(NonPrimitiveActions.acquire, constraint4green),
                 (NonPrimitiveActions.deposit, constraintpose4green)])
            # place 1x2xred
            block2red = Constraint(block=Block(2, 1, 'red'), orientation=0)
            pose2red = Constraint(
                position=Pose2D(x=0.80, y=-0.13, theta=0), orientation=1)
            self._hard_coded_plan.append(
                [(NonPrimitiveActions.acquire, block2red),
                 (NonPrimitiveActions.deposit, pose2red)])
            # place 1x2xblue
            block2blue = Constraint(block=Block(2, 1, 'blue'), orientation=0)
            pose2blue = Constraint(
                position=Pose2D(x=0.73, y=-0.13, theta=0), orientation=1)
            self._hard_coded_plan.append(
                [(NonPrimitiveActions.acquire, block2blue),
                 (NonPrimitiveActions.deposit, pose2blue)])
        elif task == '||':
            blue_1x4 = Constraint(block=Block(4, 1, 'blue'), orientation=0)
            pose_blue_1x4 = Constraint(
                position=Pose2D(x=0.73, y=-0.15), orientation=0)
            self._hard_coded_plan.append(
                [(NonPrimitiveActions.acquire, blue_1x4),
                 (NonPrimitiveActions.deposit, pose_blue_1x4)])
            green_1x4 = Constraint(block=Block(4, 1, 'green'), orientation=0)
            pose_green_1x4 = Constraint(
                position=Pose2D(x=0.73, y=-0.12), orientation=0)
            self._hard_coded_plan.append(
                [(NonPrimitiveActions.acquire, green_1x4),
                 (NonPrimitiveActions.deposit, pose_green_1x4)])
        self._hard_coded_ptr = 0
