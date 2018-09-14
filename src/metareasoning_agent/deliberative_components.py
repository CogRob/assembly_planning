#!/usr/bin/env python
"""
Class for rule-based planner for supporting meta-reasoning and agent class
"""

import logging
from geometry_msgs.msg import Pose
from metareasoning_agent.knowledge_base import PrimitiveActions, Block

MODULE_LOGGER = logging.getLogger(
    'metareasoning_agent.deliberative_components')


def check_availability(plans, blocks):
    """method to check availability of blocks against plan requirements

    input:
        plan
            a list of plans which are dicts of form block: pose
        blocks
            a list of blocks available to the agent currently
    returns:
        ranked availabilities
            a list of tuples (plan_idx, boolean availability, length of plan)
    """
    result_list = {}
    for (idx, plan) in enumerate(plans):
        result_list[idx] = [0, len(plan)]
        for reqd_block in plan:
            if reqd_block in blocks:
                result_list[idx][0] += 1
        if result_list[idx][0] == result_list[idx][1]:
            result_list[idx][0] = True
    return result_list


class Planner(object):  # pylint: disable=too-many-instance-attributes
    """Planner class which houses all decomposition rules and task-plans

    Important interfacing functions:
    * setup(task, multiple = bool):
        'task' is of form string to start with. setup resets all
        internal variables, and finds the corresponding
        plan to task. If found the result = True, otherwise False. setup
        sends the retrieved plan for decomposition to _plan_decomposition()
        which decomposes and stores the sequence of primitive actions in list.
        A pointer, self_next_action, is initialized which takes care of which
        time-step the execution is at. 'multiple' is a boolean which describes
        if we just need the most optimal decomposition of the task or if a
        decomposition tree should be constructed to be traversed for
        exploration. It is stored into self._multiple_mode
    * get_action(task):
        'task' is matched against the planned task to do a weak
        handshake. First it checks if the plan is terminated or not. If not the
        function returns self._action_plan[self._next_action] and increments
        self._next_action. If the plan is terminated it sends a None object
        indicating end of the plan. When in multiple mode, the end of all plans
        is notified by sending two None objects at the end
    * get_plan(task):
        Does the same weak handshake as get_action and returns
        self._action_plan if it passes. Mostly used to interface with
        meta-reasoner

    Important internal functions:
    * _mission_decomposition():
        decomposes task into a tree of possible block combinations
    * _plan_decomposition():
        returns the optimal decomposition or stores all
        possible decompositions as a tree/list to be accessed by
        get_next_action() in multiple mode. Also takes care of branching and
        explored information for multiple tree traversal. Traversals are ranked
        by two main things. If decomposition rules already has a learnt
        q-values, choose the one with max first. Each decomposition can also be
        introspected for number of actions, least number of actions to be
        ranked first. Feature precedence: Q-value > |action| .
    """

    def __init__(self, debug):
        self._debug = debug
        self._logger = logging.getLogger(
            'metareasoning_agent.deliberative_components.Planner')
        # database init
        self._method_db = {}
        self._mission2method_db = {}
        self._block_list = []
        # state-ful variables
        self._mission_plan = []
        self._action_plan = []
        self._mission_result = False
        self._result = False
        self._task = None
        self._multiple_mode = False
        self._action_pointer = -1

    # internal databases
    def _populate_mission_rules(self):
        """
        Decompose from form to block tree
        """
        self._mission2method_db['task1'] = []
        # append the 1x2 + 1x4 mission
        pose_1x2 = Pose()
        pose_1x4 = Pose()
        # TODO: fill these damned Poses
        self._mission2method_db['task1'].append([(Block(1, 2), pose_1x2),
                                                 (Block(1, 4), pose_1x4)])
        pose_1x1 = Pose()
        # TODO: fill these damned Poses
        # pose_1x2 = Pose()
        # pose_1x4 = Pose()
        plan = []
        plan.append((Block(1, 1), pose_1x1))
        plan.append((Block(1, 2), pose_1x2))
        plan.append((Block(1, 2), pose_1x2))
        plan.append((Block(1, 1), pose_1x1))
        self._mission2method_db['task1'].append(plan)

    def _populate_method(self):
        """
        Protected method to fill in base methods
        The planning hierarchy goes:
            Mission input --> Mission Decomposition based on availability of
            blocks --> Methods which can enable the goal state --> Action Plan
        """
        # TODO: this needs to be a block to block(s) mapping where the output
        # should be an ordered dictionary with each entry as the sub-goal block
        # and its attribute, i.e. Pose
        # 1. 1x4 to 2x2,2x2 1x1,1x3 1x3,1x1
        # 2. 1x3 to 1x1,1x1,1x1 1x1,1x2 1x2,1x1
        # 3. 1x2 to 1x1,1x1
        # * Would love it if we can do reverse mapping as well in the same
        # format
        # TODO: block to block mapping is kind of like a function
        # where you access the required Pose of the Block and then
        # map it to a list of blocks whose poses are a function of the
        # input pose
        self._method_db['acquire'] = self._acquireroutine
        self._method_db['deposit'] = self._depositroutine
        # self._rule_db['nudge'] = self._nudgeroutine

    # routine definitions
    def _acquireroutine(self, block):  # pylint: disable=no-self-use
        """
        Definition for the high-level method acquire

        input
        :block: of type knowledge_base.Block

        returns
        :action_plan: a list of tuples, where 1st element is the name of a
        PrimitiveAction and 2nd is the related constraint
        """
        action_plan = []
        action_plan.append((PrimitiveActions.transport, block))
        action_plan.append((PrimitiveActions.align, block))
        action_plan.append((PrimitiveActions.pick, None))
        action_plan.append((PrimitiveActions.retract, None))
        return action_plan

    def _depositroutine(self, b_pose):  # pylint: disable=no-self-use
        """
        Definition for the high-level method deposit

        input
        :block: of type knowledge_base.Block

        returns
        :action_plan: a list of tuples, where 1st element is the name of a
        PrimitiveAction and 2nd is the related constraint
        """
        action_plan = []
        action_plan.append((PrimitiveActions.transport, b_pose))
        action_plan.append((PrimitiveActions.align, b_pose))
        action_plan.append((PrimitiveActions.place, None))
        action_plan.append((PrimitiveActions.retract, None))
        return action_plan

    # internal decomposition functions
    def _mission_decomposition(self):
        """
        Decomposes mission into a list of (block,pose) tuples

        uses:
            _task - mission input
            _block_list - to check availability for decomposition
            _mission2method_db - to traverse through possible decompositions

        returns:
            True/False - if a decomposition is found/not found
        """
        if self._task in self._mission2method_db:
            plans = self._mission2method_db[self._task]
            if plans is not None:
                ranked_valid_plans = check_availability(
                    plans, self._block_list)
                lenmin = 1000
                minid = -1
                for key in ranked_valid_plans:
                    if ranked_valid_plans[key][0] is True and\
                       ranked_valid_plans[key][1] < lenmin:
                        lenmin = ranked_valid_plans[key][1]
                        minid = key
                if minid != -1:
                    self._mission_plan = plans[minid]
                    return True
        return False

    def _plan_decomposition(self):
        """
        Use inputs from self._task and self._mission_plan to decompose it into
        a tree or list (depending upon if self._multiple_mode is True or False
        respectively)
        """
        pass

    # external interface functions
    def setup(self, task, multiple=False):
        """Extracts plan for task and decomposes
        :return: 1 if plan is foumd, 0 if not
        """
        self._task = task
        self._multiple_mode = multiple
        # dependent on block availability
        self._mission_result = self._mission_decomposition()
        # just a general decomposition
        self._result = self._plan_decomposition()
        if self._result is True:
            self._action_pointer = 0
        return self._result

    def get_action(self):
        """Return the next action from the plan created for task
        """
        if not (self._multiple_mode):
            if self._action_pointer == len(self._action_plan) - 1:
                return None
            self._action_pointer += 1
        else:
            # TODO: here will be checking if all plans have been executed, if
            # not then reset _action_pointer and continue as usual, but if the
            # plans have ended then send another None and be done with it
            pass
        return self._action_plan[self._action_pointer - 1]

    def get_plan(self, task):
        """Interface for meta-reasoner"""
        if self._task == task:
            return self._action_plan
        return None


class Learner(object):
    """Reinforcement learner with q-learning algorithm"""

    def __init__(self, debug=False):
        self._logger = logging.getLogger(
            'metareasoning_agent.deliberative_components.Learner')
