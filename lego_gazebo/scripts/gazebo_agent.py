#!/usr/bin/env python
"""
Script to programatically:
    1. Insert and delete blocks models in Gazebo
    2. Actuate Baxter to pick and place blocks
    3. Randomize pick and place variables as seeded from recorded
       demonstrations
"""
import logging
import math
from metareasoning.knowledge_base import Block, EnvState
from metareasoning.constraints import (
    Constraint,
    PoseConstraint,
    MotionConstraint,
)
from metareasoning.agent_sim import Agent
import metareasoning.planner as planner
import rospkg
import rospy
import sys

from geometry_msgs.msg import (
    Pose,
    Point,
    Pose2D,
    Quaternion,
)
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from std_msgs.msg import (
    Empty, )
from tf.transformations import quaternion_from_euler

import baxter_interface


class AgentSim(Agent):
    """Simulated Baxter agent + Gazebo world management for execution"""

    def __init__(self, limb, hover_distance=0.15, verbose=True):
        # sets up manipulation variables
        super(AgentSim, self).__init__(limb, hover_distance)

        # world knowledge
        self._real_table_ht = 0.81
        self._gazebo_table_ht = 0.755
        self._model_dict = {}
        self.world_state = EnvState()
        self.table_pose = Pose(
            position=Point(
                x=1.0, y=0.0, z=self._real_table_ht - self._gazebo_table_ht))
        self.table_reference_frame = "world"
        self.block_num = 0
        self._populate_dictionary()

    def _populate_dictionary(self):
        self._model_dict['1x1'] = 'megabloks1x1/model.sdf'
        self._model_dict['1x2'] = 'megabloks1x2/model.sdf'
        self._model_dict['1x3'] = 'megabloks1x3/model.sdf'

    def load_gazebo_models(self):
        # Get Models' Path
        model_path = rospkg.RosPack().get_path('lego_gazebo') + "/models/"
        # Load Table SDF
        table_xml = ''
        with open(model_path + "cafe_table/model.sdf", "r") as table_file:
            table_xml = table_file.read().replace('\n', '')
        # Spawn Table SDF
        rospy.wait_for_service('/gazebo/spawn_sdf_model')
        try:
            spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model',
                                           SpawnModel)
            resp_sdf = spawn_sdf("cafe_table", table_xml, "/", self.table_pose,
                                 self.table_reference_frame)
        except rospy.ServiceException, e:
            rospy.logerr("Spawn SDF service call failed: {0}".format(e))
        # Load Blocks SDF as described in the EnvState graph
        block_reference_frame = 'world'
        self.block_num = 0
        rospy.loginfo(
            "Nodes in state: " + str(self.world_state.workspace.nodes()))
        for node, data in self.world_state.workspace.nodes(data=True):
            block_xml = ''
            # read the block model SDF
            with open(model_path + self._model_dict[str(data['width']) + 'x' +
                                                    str(data['length'])],
                      'r') as block_file:
                block_xml = block_file.read().replace('\n', '')
            block_quat = []
            if data['pose_theta'] == 1:
                block_quat = quaternion_from_euler(0, 0, math.pi / 2)
            else:
                block_quat = quaternion_from_euler(0, 0, 0)
            block_pose = Pose(
                position=Point(
                    x=data['pose_x'],
                    y=data['pose_y'],
                    z=self._real_table_ht + 0.1),
                orientation=Quaternion(
                    x=block_quat[0],
                    y=block_quat[1],
                    z=block_quat[2],
                    w=block_quat[3]))
            rospy.loginfo("Pose of block: " + str(block_pose))
            # get formatted name for the model
            block_model_name = self._generate_model_name(
                self.block_num, data['width'], data['length'], data['color'])

            # spawn the block sdf
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            try:
                spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model',
                                               SpawnModel)
                resp_sdf = spawn_sdf(block_model_name, block_xml, "/",
                                     block_pose, block_reference_frame)
            except rospy.ServiceException, e:
                rospy.logerr("Spawn SDF service call failed: {0}".format(e))
            self.block_num += 1

    def delete_gazebo_models(self):
        # This will be called on ROS Exit, deleting Gazebo models
        # Do not wait for the Gazebo Delete Model service, since
        # Gazebo should already be running. If the service is not
        # available since Gazebo has been killed, it is fine to error out
        try:
            delete_model = rospy.ServiceProxy('/gazebo/delete_model',
                                              DeleteModel)
            resp_delete = delete_model("cafe_table")
            for i in range(self.block_num):
                resp_delete = delete_model("block" + str(i))
        except rospy.ServiceException, e:
            rospy.loginfo("Delete Model service call failed: {0}".format(e))

    def _generate_model_name(self, block_num, width, length, color):
        return 'block_' + block_num + '_' + width + '_' + length + '_' + color


def main():
    rospy.init_node('digital_twin')
    # Wait for the All Clear from emulator startup
    rospy.wait_for_message("/robot/sim/started", Empty)

    # constants for the simulation pipeline
    limb = 'left'
    hover_distance = 0.15  # meters

    # create a simulation agent
    sim = AgentSim(limb, hover_distance)
    # Move to the desired starting angles
    sim.move_to_start()
    # populate the graph
    block1 = Block(1, 3, 'red', pose=Pose2D(x=1.0, y=0.0))
    sim.world_state.add_block(block1)
    block2 = Block(1, 3, 'blue', pose=Pose2D(x=1.0, y=0.2, theta=1))
    sim.world_state.add_block(block2)
    sim.load_gazebo_models()
    rospy.on_shutdown(sim.delete_gazebo_models)
    while not rospy.is_shutdown():
        continue
        # TODO: generate a placement pose
        # logging.info("Picking...")
        # pick_constraint = Constraint(block=block1)
        # plan = planner.acquireroutine(pick_constraint)
        # for action, constraint in plan:
        #     sim.executor(action, constraint)
        # logging.info("Placing...")
        # place_constraint = Constraint(position=Pose2D(x=1.0, y=1.0))
        # plan = planner.depositroutine(place_constraint)
        # for action, constraints in plan:
        #     sim.executor(action, constraint)
    return 0


if __name__ == '__main__':
    sys.exit(main())
