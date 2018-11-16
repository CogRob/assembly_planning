#!/usr/bin/env python
# Author: Priyam Parashar
# Demo script for recording a block arrangement, creating expectations and
# executing it on a Baxter

import sys
import matplotlib.pyplot as plt
from enum import Enum
import pickle as pkl
import threading
import copy

import rospy
from rospkg import RosPack
import networkx as nx
from metareasoning.knowledge_base import EnvState, Block
from metareasoning.planner import DeltaPlanner
from metareasoning.agent import Agent
from metareasoning.baxter_scripts import JointRecorder
from block_detector.msg import BlockObservationArray

from std_msgs.msg import (
    Header,
    Empty,
)
from baxter_interface import (
    DigitalIO,
    Navigator,
)
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)


class RobotModes(Enum):
    learn = 'LEARN'
    execute = 'EXECUTE'
    teach = 'TEACH'


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
        # control input and output
        self._agent = Agent(arm)
        self._agent.subscribe()
        self._arm_nav = Navigator('%s' % (arm, ))
        self._torso_nav = Navigator('torso_%s' % (arm, ))
        self._planner = DeltaPlanner()
        self._teach_button = DigitalIO('%s_upper_button' % (arm, ))  # dash
        # record right arm, record in end-effector space, save to 'trajectory'
        # file at 10 Hz
        self._recorder = JointRecorder(
            arm='right', mode='end', filename='trajectory', rate=10)

        # state
        self._mode = mode
        self._num_states = 0
        self._curr_env_state = EnvState()
        self.state_list = []
        self._task_expectation = []
        self._adapted_plan = []
        self._task_loaded = False  # if expectations are loaded
        self._task_saved = False  # if loaded expectations are saved
        self._execution = False  # whether loaded task is executed or not
        self._plan_loaded = False
        self._teach_start = False
        self._teach_done = False

        # state input and output
        rospy.Subscriber('/block_detector/top/block_obs',
                         BlockObservationArray, self.get_state)
        self._teach_button.state_changed.connect(self.teach_routine)

        # uncomment if any files need to be read
        # rp = RosPack()
        # path = rp.get_path('metareasoning')
        # read stored expectations
        # try:
        #     with open(path + '/data/a_expectation.pkl', 'rb') as f:
        #         self._task_expectation = pkl.load(f)
        #     rospy.loginfo("Found the expectation file")
        #     self.state_list = self._task_expectation
        #     for state in self.state_list:
        #         print str(state)
        #     self.set_mode(RobotModes.execute)
        #     self._task_loaded = True
        # except IOError:
        #     rospy.loginfo("Expectations not found. Please help the robot!")
        # read adapted plan
        # try:
        #     with open(path + '/data/adapted_plan.pkl', 'rb') as f:
        #         self._adapted_plan = pkl.load(f)
        #     rospy.loginfo("Found the adapted plan")
        #     self.set_mode(RobotModes.execute)
        #     self._plan_loaded = True
        # except IOError:
        #     rospy.loginfo("Expectations not found. Please help the robot!")
        rospy.loginfo("Robot started in %s mode", self._mode)
        self._switch_mode_lights()

    def _switch_mode_lights(self):
        if self._mode == RobotModes.learn:
            self._arm_nav.inner_led = True
            self._arm_nav.outer_led = False
        elif self._mode == RobotModes.execute:
            self._arm_nav.inner_led = False
            self._arm_nav.outer_led = True
        elif self._mode == RobotModes.teach:
            self._arm_nav.inner_led = True
            self._arm_nav.outer_led = True

    def next_cycle(self):
        # manage mode change
        if self._arm_nav.button2 == 1:
            if self._mode == RobotModes.learn:
                self._mode = RobotModes.execute
            else:
                self._mode = RobotModes.learn
            self._switch_mode_lights()
            rospy.logdebug(self._mode)
            rospy.sleep(1.0)
        # manage learn mode
        if self._mode == RobotModes.learn:
            if self._arm_nav.button0 == 1:
                self._task_loaded = True
                rospy.loginfo("capture state")
                self.save_state()
                rospy.sleep(1.0)
        # manage execution mode
        if self._mode == RobotModes.execute:
            if self._plan_loaded:
                rospy.loginfo("executing the plan")
                for i in range(len(self._adapted_plan.plan.points)):
                    point = dict(
                        zip(self._agent._limb.joint_names(),
                            self._adapted_plan.plan.points[i].positions))
                    rospy.loginfo(point)
                    self._agent._limb.move_to_joint_positions(point)
                    rospy.sleep(0.5)
                rospy.loginfo("done executing")
                self._plan_loaded = False
            if self._task_loaded and not self._task_saved:
                rospy.loginfo("Saving expectations")
                with open('a_expectation.pkl', 'wb') as f:
                    pkl.dump(self._task_expectation, f)
                self._task_saved = True
            if self._task_loaded and not self._execution:
                self.execute()
        if self._mode == RobotModes.teach:
            pass
        # manage end of rosnode
        if self._arm_nav.button1 == 1:
            rospy.signal_shutdown("End of Demo")
        # manage RENDER
        if self._torso_nav.button0 == 1:
            self._render()

    def exit(self):
        self._arm_nav.inner_led = False
        self._arm_nav.outer_led = False

    def get_state(self, data):
        # look at the message description and save: block_type, poses
        self._curr_env_state.clear()
        count = 0
        for block_obs in data.inv_obs:
            block = Block(block_obs.length, block_obs.width, block_obs.color,
                          block_obs.pose)
            self._curr_env_state.inventory.append(block)
            count += 1
        for block_obs in data.ws_obs:
            block = Block(block_obs.length, block_obs.width, block_obs.color,
                          block_obs.pose)
            self._curr_env_state.add_block(block)
            count += 1
        self._curr_env_state._block_cnt = count
        rospy.logdebug("EnvState updated. Block #: %d", count)

    def save_state(self):
        state_snapshot = copy.deepcopy(self._curr_env_state)
        self.state_list.append(state_snapshot)
        self._task_expectation.append(state_snapshot)
        self._num_states += 1
        nx.draw(state_snapshot.workspace)
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

    def execute(self):
        debug = False
        if debug:
            rospy.loginfo("Preparing to execute the hard-coded task")
            self._planner.setup_hard_coded_plan('A')
            action_plan = self._planner.get_hard_coded_plan()
            while action_plan is not None:
                for action, constraint in action_plan:
                    print str(action) + ', ' + str(constraint)
                    self._agent.executor(action, constraint)
                action_plan = self._planner.get_hard_coded_plan()
        else:
            rospy.loginfo("Executing demonstrated task")
            ptr = 0
            while ptr < len(self.state_list) - 1:
                action_plan = self._planner.plan(self.state_list[ptr],
                                                 self.state_list[ptr + 1])
                for action, constraint in action_plan:
                    rospy.loginfo("%s, %s", str(action), str(constraint))
                    self._agent.executor(action, constraint)
                ptr += 1
        rospy.loginfo("Done executing the plan")
        self._agent.move_to_start()
        self._execution = True

    def teach_routine(self, value):
        """
        Record the trajectory taught by human user
        """
        self.set_mode(RobotModes.teach)
        if not self._teach_start:
            self._teach_start = True
            self._teach_done = False
        elif self._teach_start and not self._teach_done:
            self._teach_start = False
            self._teach_done = True
            self._recorder.stop()
        if self._teach_start:
            process = threading.Thread(target=self._recorder.record)
            process.daemon = True
            process.start()


def main():
    rospy.init_node('record_and_execute')
    rexec = RecordAndExecute('right')
    rospy.on_shutdown(rexec.exit)
    while not rospy.is_shutdown():
        rexec.next_cycle()


if __name__ == '__main__':
    sys.exit(main())
