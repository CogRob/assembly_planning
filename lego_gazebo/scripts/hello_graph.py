#!/usr/bin/env python
"""
Script to programatically:
    1. Insert and delete blocks models in Gazebo
"""
import logging
from metareasoning_agent.knowledge_base import Block, EnvState
import networkx as nx
import rospkg
import rospy
import sys

from baxter_sim_example import PickAndPlace

from geometry_msgs.msg import (
    Pose,
    Point,
    Pose2D,
)
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)
from std_msgs.msg import (
    Empty, )


class GazeboSimulator(object):
    """ TODO: docstring for GazeboSimulator"""

    def __init__(self):
        super(GazeboSimulator, self).__init__()
        self._real_table_ht = 0.81
        self._gazebo_table_ht = 0.755
        self._model_dict = {}
        self.state = EnvState()
        self.table_pose = Pose(
            position=Point(
                x=1.0, y=0.0, z=self._real_table_ht - self._gazebo_table_ht))
        self.table_reference_frame = "world"
        self.block_num = 0
        self._populate_dictionary()

    def _populate_dictionary(self):
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
        for node, data in self.state.workspace.nodes(data=True):
            block_xml = ''
            # read the block model SDF
            with open(model_path + self._model_dict[str(data['length']) + 'x' +
                                                    str(data['width'])],
                      'r') as block_file:
                block_xml = block_file.read().replace('\n', '')
            block_pose = Pose(
                position=Point(
                    x=data['pose_x'], y=data['pose_y'], z=self._real_table_ht))
            # spawn the block sdf
            rospy.wait_for_service('/gazebo/spawn_sdf_model')
            try:
                spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model',
                                               SpawnModel)
                resp_sdf = spawn_sdf("block" + str(self.block_num), block_xml,
                                     "/", block_pose, block_reference_frame)
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


def main():
    rospy.init_node('hello_graph')
    robot = False
    if robot:
        # Wait for the All Clear from emulator startup
        rospy.wait_for_message("/robot/sim/started", Empty)

        limb = 'left'
        hover_distance = 0.15  # meters
        # Starting Joint angles for left arm
        starting_joint_angles = {
            'left_w0': 0.6699952259595108,
            'left_w1': 1.030009435085784,
            'left_w2': -0.4999997247485215,
            'left_e0': -1.189968899785275,
            'left_e1': 1.9400238130755056,
            'left_s0': -0.08000397926829805,
            'left_s1': -0.9999781166910306
        }
        pnp = PickAndPlace(limb, hover_distance)
        # Move to the desired starting angles
        pnp.move_to_start(starting_joint_angles)
    sim = GazeboSimulator()
    # TODO: populate the graph
    block1 = Block(1, 2)
    block2 = Block(1, 3)
    block1.pose = Pose2D(x=1.0, y=1.0)
    block2.pose = Pose2D(x=1.0, y=0.0)
    sim.state.add_block(block1)
    sim.state.add_block(block2)
    sim.load_gazebo_models()
    rospy.on_shutdown(sim.delete_gazebo_models)
    while not rospy.is_shutdown():
        logging.info('Looping, you should be seeing the graph')
    return 0


if __name__ == '__main__':
    sys.exit(main())
