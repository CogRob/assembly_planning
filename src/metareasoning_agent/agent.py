#!/usr/bin/env python
"""
 Author: Priyam Parashar
 Agent class which interfaces with Planner and MetaReasoning. Also actuates
 Baxter's hardware depending upon the planned action.
"""
from __future__ import print_function

import copy
import struct

import baxter_interface
import rospy
from baxter_core_msgs.srv import SolvePositionIK, SolvePositionIKRequest
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from std_msgs.msg import Header
import networkx as nx
import tf
import math
import numpy as np

from block_mover.msg import BlockObservationArray, BlockObservation, BlockPixelLocArray, BlockPixelLoc


from metareasoning_agent.knowledge_base import Block, PrimitiveActions
from metareasoning_agent.utilities import print_pose, calculate_pose_diff


class EnvState(object):
    """List of blocks in inventory + graph describing the blocks in workspace

    Workspace Graph:
        Node - Blocks placed in the workspace
        Edge - Spatial relationships between the blocks as they are placed

    Inventory:
        {bId: [bType, bPose]}:  Dictionary of block IDs with corresponding
                                block type and that block's geometry_msgs/Pose
    """

    def __init__(self):
        self._block_cnt = 0
        self.ws_state = nx.Graph()

        self.inv_state = []

    def add_block(self, block_type, block_pose):
        """Method to add a new block as a node to the EnvState graph"""
        self.state.add_node(
            self._block_cnt + 1, bType=block_type, bPose=block_pose)
        self._block_cnt += 1
        #self._update_edges()

    def _update_edges(self):
        """Method to update edges to the latest block added"""
        base_node_pose = self.env.nodes[self._block_cnt - 1]['bPose']
        for idx in range(0, self._block_cnt - 1):
            target_node_pose = self.state.nodes[idx]['bPose']
            pose_diff = calculate_pose_diff(base_node_pose, target_node_pose)
            self.state.add_edge(self._block_cnt - 1, idx, object=pose_diff)

class Agent(object):
    """
    Interface layer between Baxter's hardware, planning and meta-reasoning
    Attributes:
        _limb_name:         name of the robot limb to be controlled
        _limb:              interface object for the limb to be controlled
        _hover_distance:    distance above the object where the gripper should
                            hover first
        _gripper:           interface object for the gripper of limb to be
                            controlled
        _iksvc:             ServiceProxy for sending requests to the Cartesian
                            to IKSolver service
        _rs:                robot object, to monitor and control hardware
    """

    # TODO: Need to add methods which can "push" updates to meta-reasoning
    # TODO: Perception class which interfaces James' stuff with my code
    def __init__(self, limb, hover_distance=0.15, verbose=True):
        # Baxter specific variables
        self._limb_name = limb
        self._hover_distance = hover_distance  # in meters
        self._verbose = verbose  # bool
        self._limb = baxter_interface.Limb(limb)
        self._gripper = baxter_interface.Gripper(limb)

        # Baxter system setup
        namespace = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(namespace, SolvePositionIK)
        rospy.wait_for_service(namespace, 5.0)
        #   verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

        # cognition variables
        self._ws = EnvState()
        self._inventory = {}
        # TODO: initialize the inventory dictionary with all kinds of block
        # types which can exist, assign each to an empty list

        # planning interface variables
        self._actions = {
            PrimitiveActions.pick: self._pick,
            PrimitiveActions.place: self._place,
            PrimitiveActions.retract: self._retract,
            PrimitiveActions.transport: self._transport,
            PrimitiveActions.align: self._align
        }
        self._overhead_orientation = Quaternion(
            x = 0.707,
            y = 0.707,
            z = 0.0,
            w = 0.0
        )

        # x=-0.0249590815779,
        # y=0.999649402929,
        # z=0.00737916180073,
        # w=0.00486450832011)
        self._start_angles = {
            'right_s0': 0.4398689909261424,
            'right_s1': -0.6872233929726653,
            'right_w0': -0.0337475773334791,
            'right_w1': 1.271670073157008,
            'right_w2': -1.6409759478404213,
            'right_e0': 0.018407769454624964,
            'right_e1': 0.8709175923219437
        }

        # Meta-reasoning level information
        self._actuation_results = True

        # move to starting position
        self.move_to_start(self._start_angles)

        self.curr_pose = None
        self.pixel_locs = []

    def update_block_locations(self):
        rospy.loginfo("Updating block locations. Waiting for BlockObservationArray....")

        block_obs_msg = rospy.wait_for_message("/block_finder/top/block_obs", BlockObservationArray)

        rospy.loginfo("Received %d block observations", len(block_obs_msg.inv_obs))
        block_locations = []
        
        for block_obs in block_obs_msg.inv_obs:
            block_locations.append(Block(length=block_obs.length, width=block_obs.width, color=block_obs.color, pose=block_obs.pose))

        self.inv_state = block_locations


    def subscribe(self):
        hand_cam_pix_sub = rospy.Subscriber("/block_finder/right_hand/block_pixel_locs", BlockPixelLocArray, self.hand_cam_pixel_locs_callback)

    def hand_cam_pixel_locs_callback(self, data):
        rospy.loginfo("New pixel location data has arrived")
        self.pixel_locs = data.pixel_locs


    # BAXTER-specific methods
    def move_to_start(self, start_angles=None):
        """Move to start_angles joint configuration before task execution"""

        rospy.logdebug("Moving the %s arm to start pose...", self._limb_name)
        if not start_angles:
            start_angles = dict(
                zip(self._limb._joint_names[self._limb_name], [0] * 7))  # pylint: disable=W0212
        self._guarded_move_to_joint_position(start_angles)
        self._gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        """Returns joint angle configuration for desired end-effector pose"""
        rospy.loginfo("Moving to Pos: x:%f, y:%f, z%f", pose.position.x, pose.position.y, pose.position.z)
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), exceptn:
            rospy.logerr("Service call failed: %s", (exceptn, ))
            return False

        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                                   resp.result_type)
        limb_joints = {}
        if resp_seeds[0] != resp.RESULT_INVALID:
            seed_str = {
                ikreq.SEED_USER: 'User Provided Seed',
                ikreq.SEED_CURRENT: 'Current Joint Angles',
                ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
            }.get(resp_seeds[0], 'None')
            if self._verbose:
                print(
                    "IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".
                    format((seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(
                zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr(
                "No Joint Angles provided for move_to_joint_positions. Staying put."
            )

    def _gripper_open(self):
        self._gripper.open()

    def _gripper_close(self):
        self._gripper.close()

    def _servo_to_pose(self, pose):
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)

    # Methods for primitive actions

    def _transport(self, pose):
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _align(self, block_color, block_length, block_width, block_angle):
        # When align gets called, height should be at 0
        pixel_dist_thresh = 5
        pixel_dist = 0
            
        # At 0.0 meters the location of center pixel that will result in optimal grasp
        upper_pixel_center_x = 325
        upper_pixel_center_y = 129

        # At -0.15 meters the location of center pixel that will result in optimal grasp
        lower_pixel_center_x = 330
        lower_pixel_center_y = 94

        rospy.loginfo("Rotating gripper to %f", math.degrees(block_angle))
        self._rotate_gripper(block_angle)

        # Align at 0.0 meters 
        while(True):
            if len(self.pixel_locs) > 0:
                rospy.loginfo("There are some blocks...")
            else:
                rospy.loginfo("There are no blocks...")

            for pixel_loc in self.pixel_locs:
                rospy.loginfo("Block color (from locs): %s W: %d, L: %d", pixel_loc.color, pixel_loc.width, pixel_loc.length)
                rospy.loginfo("Block color: %s W: %d, L: %d", block_color, block_width, block_length)

                if(pixel_loc.color == block_color and pixel_loc.width == block_width and pixel_loc.length == block_length):
                    # We found the block we want to allign with
                    pixel_x_dist = pixel_loc.x - lower_pixel_center_x
                    pixel_y_dist = pixel_loc.y - lower_pixel_center_y
                    pixel_dist = math.sqrt(pixel_x_dist**2 + pixel_y_dist**2)

                    rospy.loginfo("Pixel distance is: %f", pixel_x_dist)
                    if(pixel_x_dist > 100):
                        # Don't move... something must be wrong
                        return
                    if(pixel_y_dist > 100):
                        # Don't move... something must be wrong
                        return

                    if(pixel_dist > pixel_dist_thresh):
                        motion_angle = math.atan2(pixel_loc.y - lower_pixel_center_y, pixel_loc.x - lower_pixel_center_x)
                        """

                        if(pixel_x_dist < 0 and pixel_y_dist > 0):
                            # Q2
                            motion_angle += math.pi
                        elif(pixel_x_dist , 0 and pixel_y_dist < 0):
                            # Q3
                            motion_angle += math.pi
                        """

                        rospy.loginfo("Pixel distance still larger than threshold. Moving in direction %f", motion_angle)
                        self.move_camera_in_plane(motion_angle, motion_dist = 0.01)
                        rospy.sleep(20)
                        continue
                    else:
                        rospy.loginfo("Reached within %f of goal", pixel_dist)
                        rospy.sleep(10)
                        return
                else:
                    continue


    # TODO: For testing purposes to give tester access to private _align(), delete later!
    def extern_align(self, block_color, block_length, block_width, block_angle):
        self._align(block_color, block_length, block_width, block_angle)


    def _retract(self):
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)

    def _pick(self):
        # close gripper
        self._gripper_close()
        # retract to clear object
        self._retract()

    def _place(self):
        # open the gripper
        self._gripper_open()
        # retract to clear object
        self._retract()



    def _rotate_gripper(self, angle):
        # NOTE: angle in radians!
        q_rot = tf.transformations.quaternion_from_euler(angle, 0, 0)
        curr_q = self._overhead_orientation
        curr_q_arr = np.array([curr_q.w, curr_q.x, curr_q.y, curr_q.z])

        q_new = tf.transformations.quaternion_multiply(q_rot, curr_q_arr)
        
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z
        ik_pose.orientation.x = q_new[1]
        ik_pose.orientation.y = q_new[2]
        ik_pose.orientation.z = q_new[3]
        ik_pose.orientation.w = q_new[0]

        joint_angles = self.ik_request(ik_pose)
        self._guarded_move_to_joint_position(joint_angles)

    def get_current_state(self):
        return self._limb.endpoint_pose()

    def move_camera_in_plane(self, direction, motion_dist=0.01):
        if(direction is None):
            return

        rospy.loginfo("Moving camera in plane in direction %f ", math.degrees(direction))

        current_pose = self._limb.endpoint_pose()

        ik_pose = Pose()
        
        ik_pose.orientation.x = current_pose['orientation'].x 
        ik_pose.orientation.y = current_pose['orientation'].y 
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        ik_pose_q = (ik_pose.orientation.x,
                     ik_pose.orientation.y,
                     ik_pose.orientation.z,
                     ik_pose.orientation.w)

        ik_pose_euler = tf.transformations.euler_from_quaternion(ik_pose_q)
        rospy.loginfo("Roll: %f Pitch: %f Yaw %f", ik_pose_euler[0], ik_pose_euler[1], ik_pose_euler[2])

        ik_pose.position.x = current_pose['position'].x + motion_dist*math.cos(direction) # + ik_pose_euler[2])
        ik_pose.position.y = current_pose['position'].y + motion_dist*math.sin(direction) # + ik_pose_euler[2])
        ik_pose.position.z = current_pose['position'].z

        joint_angles = self.ik_request(ik_pose)
        self._guarded_move_to_joint_position(joint_angles)

        return ik_pose.position.x, ik_pose.position.y, ik_pose.position.z

    # PLANNING interface
    def executor(self, action, constraints=None):
        """Executor: Interfaces with planner, receives action and actuates Baxter"""
        # if the action does not require constraint, check constraint == None
        if action is PrimitiveActions.pick or \
        action is PrimitiveActions.place or \
        action is PrimitiveActions.retract:
            if constraints != None:
                rospy.logerr("%s should not be passed any arguments", action)
            else:
                self._actions[action]()
        else:
            if action == PrimitiveActions.transport:
                if isinstance(constraints, Block):
                    # TODO: need a way to distinguish between whether the block
                    # that the gripper is to be transported to is in INV or WS
                    rospy.logdebug("Not Implemented")
                else:
                    pose_constraint = Pose()
                    pose_constraint.position = constraints
                    pose_constraint.orientation = self._overhead_orientation
                    print_pose(pose_constraint)
                    self._actions[action](pose_constraint)
            else:
                rospy.logdebug("Not Implemented")
        return


#def test():
#    """
#    Borrowed parts from: RSDK Inverse Kinematics Pick and Place Example
#
#    Creates an object of type Agent, initializes and tests various implementations
#    """
#    # create a rosnode
#    rospy.init_node("agent_test")
#
#    # create an agent with the intent to control right arm
#    agent = Agent('right')
#
#    # test various primitive actions
#    for i in range(0, len(agent.pactions)):
#        rospy.logdebug("testing %s...", agent.pactions[i])
#        agent.executor(agent.pactions[i])
