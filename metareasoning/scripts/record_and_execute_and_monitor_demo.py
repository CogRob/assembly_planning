#!/usr/bin/env python
# Author: Priyam Parashar
# Demo script for recording a block arrangement, creating expectations and
# executing it on a Baxter

import sys
import matplotlib.pyplot as plt
from enum import Enum
import pickle as pkl

import rospy
import networkx as nx
from metareasoning.knowledge_base import EnvState, Block
from block_detector.msg import BlockObservationArray

from baxter_interface import (
    Navigator, )


class RobotModes(Enum):
    learn = 'LEARN'
    execute = 'EXECUTE'
    think = 'THINK'


class RecordAndExecute(object):
    """
    Records block placement sequence and plays it back
    """

    def __init__(self, arm, mode=RobotModes.learn, lights=True):
        """
        @type arm: str
        @param arm: arm which will execute
        @type mode: RobotModes
        @param mode: specifies which mode to start the robot in
        @type lights: bool
        @param lights: if lights should be activated
        """
        self._arm = arm
        # control input and output
        self._nav = Navigator('%s' % (arm, ))
        # state
        self._mode = mode
        self._num_states = 0
        self._curr_env_state = EnvState()
        self.state_list = []
        self._task_expectation = None
        self._task_loaded = False
        # state input and output
        rospy.Subscriber('/block_detector/top/block_obs',
                         BlockObservationArray, self.get_state)
        rospy.loginfo("Robot started in %s mode", self._mode)

    def _switch_mode_lights(self):
        if self._mode == RobotModes.learn:
            self._nav.inner_led = True
            self._nav.outer_led = False
        elif self._mode == RobotModes.execute:
            self._nav.inner_led = False
            self._nav.outer_led = True

    def next_cycle(self):
        # manage mode change
        if self._nav.button2 == 1:
            if self._mode == RobotModes.learn:
                self._mode = RobotModes.execute
            else:
                self._mode = RobotModes.learn
            self._switch_mode_lights()
            rospy.logdebug(self._mode)
            rospy.sleep(1.0)
        # manage capture state
        if self._mode == RobotModes.learn and self._nav.button0 == 1:
            rospy.logdebug("capture state")
            self.save_state()
            rospy.sleep(1.0)
        # manage execution mode
        if self._mode == RobotModes.execute and not self._task_loaded:
            rospy.loginfo("Saving expectations")
            with open('a_expectation.pkl', 'w') as f:
                pkl.dump(self._task_expectation, f)
            # TODO pickle the expectation
        else:
            rospy.loginfo("Executing learnt task")
            # TODO get plan for current milestone
            # execute
        # manage end of rosnode
        if self._nav.button1 == 1:
            rospy.signal_shutdown("End of Demo")

    def exit(self):
        self._nav.inner_led = False
        self._nav.outer_led = False

    def get_state(self, data):
        # look at the message description and save: block_type, poses
        self._curr_env_state.clear()
        count = 0
        for block_obs in data.inv_obs:
            block = Block(block_obs.length, block_obs.width, block_obs.color,
                          block_obs.pose)
            self._curr_env_state.inv_state.append(block)
            count += 1
        for block_obs in data.ws_obs:
            block = Block(block_obs.length, block_obs.width, block_obs.color,
                          block_obs.pose)
            self._curr_env_state.add_block(block)
            count += 1
        self._curr_env_state._block_cnt = count
        rospy.logdebug("EnvState updated. Block #: %d", count)

    def save_state(self):
        state_snapshot = self._curr_env_state
        self.state_list.append(state_snapshot)
        self._task_expectation.append(state_snapshot)
        self._num_states += 1
        nx.draw(
            state_snapshot.ws_state,
            pos=nx.spring_layout(state_snapshot.ws_state))
        plt.savefig('new_fig' + str(self._num_states) + '.jpeg')
        plt.clf()
        rospy.logdebug("EnvState saved. Block #: %d",
                       state_snapshot._block_cnt)

    def set_mode(self, mode):
        self._mode = mode
        self._switch_mode_lights()
        rospy.loginfo("Robot is in %s mode", mode)

    def set_expectation(self, expectations):
        self._task_expectation = expectations
        self._task_loaded = True


def main():
    rospy.init_node('record_and_execute', disable_signals=True)
    rexec = RecordAndExecute('right')
    rospy.on_shutdown(rexec.exit)
    expectations = None
    # read stored expectations
    try:
        with open('a_expectation.pkl', 'r') as f:
            expectations = pkl.load(f)
        rospy.loginfo("Found the expectation file")
        rexec.state_list = expectations
        rexec.set_expectation(expectations)
        rexec.set_mode(RobotModes.execute)
    except IOError:
        rospy.loginfo("Expectations not found. Please help the robot!")
    while not rospy.is_shutdown():
        rexec.next_cycle()


if __name__ == '__main__':
    sys.exit(main())
