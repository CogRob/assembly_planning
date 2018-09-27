#!/usr/bin/env python

from collections import deque
from itertools import permutations
import logging
from Queue import PriorityQueue
import networkx as nx
import networkx.algorithms.isomorphism as iso
from geometry_msgs.msg import Pose
from metareasoning.knowledge_base import (Block, Constraints, EnvState,
                                          PrimitiveActions)


class Learner(object):
    """Reinforcement learner with q-learning algorithm"""

    def __init__(self, debug=False):
        # get logger
        self._logger = logging.getLogger(__name__)
        # constants
        self._methods = ['a1d1', 'a2d1', 'a1d2', 'a2d2']
        self._errortolerancexy = 0.010
        self._errortolerancetheta = 0.087
        self._gamma = 0.75
        # book-keeping
        self._expectation = EnvState()
        self._blocks = EnvState()
        self._mission_ptr = -1
        self._method_ptr = -1
        self._curr_block_ptr = -1
        self._reward_acc = 0
        # TODO: make constraint = Pose, GripOrientation1or2, Block
        # to be updated by DelLayer during the switch
        self._task = None
        self._missions = None
        self._method_plans = None
        self._action_plan = None
        # search-based max-reward-learning
        # q-tables
        self._mission_table = None  # 1xlen(_missions)
        self._method_table = None  # len(_missions)x4 (constraint permutation)
        # results
        self._mission_idx = -1
        self._method_idx = -1
        # method function bindings
        self._method_db = {}
        self._method_db['acquire'] = acquireroutine
        self._method_db['deposit'] = depositroutine

    def _plan_decomposition(self):
        # TODO: use routines at top to decompose method plans received as input
        pass

    def _get_current_expectation(self):
        """Creates expectation state from current mission sub-goal

        every block-level step of the mission, this function adds to the global
        _expectation variable so that global expectation is compared with
        current global state
        """
        item = self._missions[self._mission_ptr][self._curr_block_ptr]
        self._expectation.add_block(item[0])

    def setup(self, task, mission_db, method_db):
        """
        Setup call for DelLayer passing required info from Planner
        """
        self._task = task
        self._missions = mission_db
        self._method_plans = method_db
        # TODO: setup the q-tables
        self._mission_table = [0] * len(self._missions)
        self._method_table = {}
        for i in range(len(self._missions)):
            self._method_table[i] = [0] * 4
        # setup the ptrs
        self._mission_ptr = 0
        self._method_ptr = 0
        self._reward_acc = 0

    def get_action(self, task):
        """
        Call from DelLayer to pass next_action to Agent
        """
        # TODO: how does this relate to multiple plans in Planner?
        pass

    def update(self, state):
        """incoming interface function for updating state"""
        self._blocks = state
        self._get_current_expectation()  # TODO: implement
        self._curr_block_ptr += 1
        self._reward_acc += self._calculate_reward()

    def _calculate_reward(self):
        """
        Calculates the shaped reward for current timestep using state and task

        state is EnvState from Agent.py. The comparison needs to be done
        between the current graph and intended graph

        FIXME: task right now is hand-coded, need to include in meta-reasoner
        """
        # nx does not support different tolerances for different edges
        # therefore the comparison is a two-step process for xy and theta
        node_condition = iso.categorical_node_match(['length', 'width'],
                                                    [1, 1])
        edge_condition1 = iso.numerical_edge_match(['delx', 'dely'], [0, 0],
                                                   rtol=self._errortolerancexy)
        edge_condition2 = iso.numerical_edge_match(
            ['deltheta'], [0], rtol=self._errortolerancetheta)
        result1 = nx.is_isomorphic(
            self._expectation.ws_state,
            self._blocks.ws_state,
            edge_match=edge_condition1,
            node_match=node_condition)
        result2 = nx.is_isomorphic(
            self._expectation.ws_state,
            self._blocks.ws_state,
            edge_match=edge_condition2,
            node_match=node_condition)
        result = result1 and result2
        if result:
            shaped_reward = self._gamma * len(
                self._missions[self._mission_ptr]) - (self._mission_ptr + 1)
            return (-1 + shaped_reward)
        else:
            return -1
