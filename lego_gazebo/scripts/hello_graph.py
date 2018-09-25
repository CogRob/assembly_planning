#!/bin/env python
import logging
from metareasoning_agent.knowledge_base import Block, EnvState
import networkx as nx
import rospkg
import rospy
import sys

from gazebo_msgs.msg import (
    Pose,
    Point,
)
from gazebo_msgs.srv import (
    SpawnModel,
    DeleteModel,
)


class GazeboSimulator(object):
    """ TODO: docstring for GazeboSimulator"""

    def __init__(self, arg):
        super(GazeboSimulator, self).__init__()
        self.state = EnvState()
        self.table_pose = Pose(position=Point(x=1.0, y=0.0, z=0.0))
        self.table_reference_frame = "world"
        self._model_dict = {}
        self._populate_dictionary()
        self.block_num = 0

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
        for node in self.state.ws_state.nodes():
            block_xml = ''
            # read the block model SDF
            with open(model_path + self._model_dict[str(node['width']) + 'x' +
                                                    str(node['length'])],
                      'r') as block_file:
                block_xml = block_file.read().replace('\n', '')
            block_pose = Pose(
                position=Point(x=node['pose_x'], y=node['pose_y'], z=0.7825))
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
    sim = GazeboSimulator()
    # TODO: populate the graph
    block1 = Block(1, 2)
    block2 = Block(1, 3)
    block1.pose = Pose(position=Point(x=1.0, y=1.0, z=0.07825))
    block2.pose = Pose(position=Point(x=1.0, y=0.0, z=0.07825))
    sim.state.add_block(block1)
    sim.state.add_block(block2)
    sim.load_gazebo_models()
    rospy.on_shutdown(sim.delete_gazebo_models)
    while not rospy.is_shutdown():
        logging.info('Looping, you should be seeing the graph')
    return 0


if __name__ == '__main__':
    sys.exit(main())
