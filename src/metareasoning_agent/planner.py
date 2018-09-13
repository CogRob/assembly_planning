#!/usr/bin/env python
"""
Class for rule-based planner for supporting meta-reasoning and agent class
"""

from metareasoning_agent.knowledge_base import PrimitiveActions, Block


class Planner(object):
    """Planner class which houses all decomposition rules and task-plans

    Important interfacing functions:
    * make_plan(task, multiple = bool): 'task' is of form string to start with.
        make_plan resets all internal variables, and finds the corresponding
        plan to task. If found the result = True, otherwise False. make_plan
        sends the retrieved plan for decomposition to _plan_decomposition()
        which decomposes and stores the sequence of primitive actions in a list.
        A pointer, self_next_action, is initialized which takes care of which
        time-step the execution is at. 'multiple' is a boolean which describes
        if we just need the most optimal decomposition of the task or if a
        decomposition tree should be constructed to be traversed for
        exploration. It is stored into self._multiple_mode
    * get_action(task): 'task' is matched against the planned task to do a weak
        handshake. First it checks if the plan is terminated or not. If not the
        function returns self._action_plan[self._next_action] and increments
        self._next_action. If the plan is terminated it sends a None object
        indicating end of the plan. When in multiple mode, the end of all plans
        is notified by sending two None objects at the end
    * get_plan(task): Does the same weak handshake as get_action and returns
        self._action_plan if it passes. Mostly used to interface with
        meta-reasoner

    Important internal functions:
    * _plan_decomposition(): returns the optimal decomposition or stores all
        possible decompositions as a tree/list to be accessed by
        get_next_action() in multiple mode. Also takes care of branching and
        explored information for multiple tree traversal. Traversals are ranked
        by two main things. If decomposition rules already has a learnt
        q-values, choose the one with max first. Each decomposition can also be
        introspected for number of actions, least number of actions to be ranked
        first. Feature precedence: Q-value > |action| .
    """

    def __init__(self):
        self._pc = PrimitiveActions()
        self._plan_db = {}
        self._method_db = {}
        self._block_list = []
        self._block_attr = []  # block attributes should be the Pose or Pose2D
        self._result = True
        self._task = None

    def _populate_method(self):
        """
        Protected method to fill in the method DB
        """
        # TODO: this needs to be a block to block(s) mapping where the output
        # should be an ordered dictionary with each entry as the sub-goal block
        # and its attribute, i.e. Pose
        # 1. 1x4 to 2x2,2x2 1x1,1x3 1x3,1x1
        # 2. 1x3 to 1x1,1x1,1x1 1x1,1x2 1x2,1x1
        # 3. 1x2 to 1x1,1x1
        # * Would love it if we can do reverse mapping as well in the same
        # format

    def _populate_plans(self):
        """Fills in the task: plan dictionary
        :returns: a filled _plan_db

        """
        pass

    def make_plan(self, task):
        """Extracts plan for task and decomposes
        :return: 1 if plan is foumd, 0 if not
        """
        self._result = True
        # TODO: write the extraction and decomposition routine

    def next_action(self):
        """Return the next action from the plan created for task
        """
        pass

    def get_all_plans(self, task):
        """Returns a list of all valid alternate plans for input: task
        """
        # TODO: this will be sort of a recursive call to make_plan or a third
        # function which has the functionality to keep finding different plan
        # decompositions as if from a tree or something
        pass
