#!/usr/bin/env python
"""
Class definition for deliberative layer which is just a communication MUX over
planner and learner class
Author: Priyam Parashar, CogRob
Date: 09/13/2018
"""

from metareasoning_agent.deliberative_components import Planner, Learner


class DeliberativeLayer(object):
    """Deliberative class for MUXing communication b/w learner and planner"""

    def __init__(self, args):
        self._planner = Planner(True)
        self._learner = Learner(True)
        self._control = 'planner'
