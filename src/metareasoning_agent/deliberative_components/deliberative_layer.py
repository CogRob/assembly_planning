#!/usr/bin/env python
"""
Class definition for deliberative layer which is just a communication MUX over
planner and learner class
Author: Priyam Parashar, CogRob
Date: 09/13/2018
"""

from enum import Enum
from metareasoning_agent.deliberative_components import Planner, Learner
import logging

module_logger = logging.getLogger('metareasoning_agent.deliberative_layer')


class DelControl(Enum):
    """
    Enum for various modes available for the deliberative layer
    """
    PLAN = 'planner'
    LEARN = 'learner'


class DeliberativeLayer(object):
    """Communication MUX over learner and planner"""

    def __init__(self, args):
        self._logger = logging.getLogger(
            'metareasoning_agent.deliberative_layer.DLayer')
        self._components = {}
        self._components[DelControl.PLAN] = Planner(True)
        self._components[DelControl.LEARN] = Learner(True)
        self._control = DelControl.PLAN

    def get_action(self, task):
        """
        Interface action wrapper which connects with planner and learner
        """
        action = self._components[self._control].get_action()
        self._logger.debug('Next action coming from %s is %s', self._control,
                           action)

        return action

    def switch_control(self, mode, task):
        """
        Interface function used to switch mode of the deliberative layer

        :mode: str
            which mode is the deliberative layer in
        :task: mission-representation
            mission that planner needs to plan for or learner needs to learn
        """
        self._control = mode
        self._logger.debug(
            'Deliberative-level control has been switched to %s',
            self._control)

    def setup(self, task):
        """
        Interface function that calls to setup of other deliberative_components
        """
        self._logger.debug('Setting up component %s with task %s',
                           self._control, task)
        result = self._components[self._control].setup(task)
        return result

    def get_plan(self, task):
        """
        Interface function for meta-reasoner: returns intended action plan

        Only works if the planner mode is on, otherwise just returns a None
        """
        if self._control == DelControl.LEARN:
            return None
        else:
            return self._components[self._control].get_plan(task)

    def update_state(self, state):
        """Interface function for communicating state to components"""
        self._components[self._control].update_state(state)
