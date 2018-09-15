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
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point
from std_msgs.msg import Header
import networkx as nx
import tf
import math
import numpy as np

from block_mover.msg import BlockObservationArray, BlockObservation, BlockPixelLocArray, BlockPixelLoc

from metareasoning_agent.knowledge_base import Block, PrimitiveActions
from metareasoning_agent.utilities import calculate_pose_diff


class Constraints(object):
    def __init__(self, block=None, position=None, orientation=None):
        self.block = block
        self.position = position
        self.orientation = orientation

        if((self.block is not None and self.orientation is None) or \
           (self.position is not None and self.orientation is None) or \
           (self.position is not None and self.block is not None)):
            rospy.logerr(
                "The constraint you are attempting to make is not valid")

    def is_block_constraints(self):
        return (self.block is not None) and (self.orientation is not None)

    def is_position_constraints(self):
        return (self.position is not None) and (self.orientation is not None)

    def is_empty_constraints(self):
        return self.block is None and self.position is None and self.orientation is None

    def is_orientation_constraints(self):
        return self.block is None and self.position is None and self.orientation is not None


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
        self._update_edges()

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
    def __init__(self, limb, hover_distance_= 0.0, verbose=True):
        # Baxter specific variables
        self._limb_name = limb
        self._hover_distance = hover_distance_  # in meters
        self._table_distance = -0.11
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
            PrimitiveActions.align: self._align,
            PrimitiveActions.detect: self._detect,
        }

        self._overhead_orientation = Quaternion(x=0, y=1, z=0, w=0)

        self._start_position = Point(x=0.5, y=-0.75, z=0)

        self._start_pose = Pose(
            position=self._start_position,
            orientation=self._overhead_orientation)

        self._start_angles = self.ik_request(self._start_pose)

        # Meta-reasoning level information
        self._actuation_results = True

        # move to starting position
        self.move_to_start(self._start_angles)

        self.curr_pose = None
        self.pixel_locs = []

    def subscribe(self):
        hand_cam_pix_sub = rospy.Subscriber(
            "/block_finder/right_hand/block_pixel_locs", BlockPixelLocArray,
            self.hand_cam_pixel_locs_callback)

    def hand_cam_pixel_locs_callback(self, data):
        self.pixel_locs = data.pixel_locs

    def _get_updated_pixel_locs(self):
        rospy.loginfo(
            "Waiting for updated block pixel locations from hand camera...")

        # Update the block pixel locations
        block_pixel_locs = rospy.wait_for_message(
            "/block_finder/right_hand/block_pixel_locs",
            BlockPixelLocArray).pixel_locs

        rospy.loginfo("Block pixel locations updated.")

        return block_pixel_locs

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
        rospy.loginfo("Moving to Pos: x:%f, y:%f, z%f", pose.position.x,
                      pose.position.y, pose.position.z)
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
    def _transport(self, block=None, position=None):
        overhead_pose = Pose()

        if (block is not None and position is None):
            rospy.loginfo(
                "Finding the correct block to transport to above. Looking for a %dx%d %s block",
                block.width, block.length, block.color)
            # TODO: We shouldn't have to go through the list of blocks everytime, store in a dictionary in future
            # Find the location of the block to move towards

            for block_loc in self.inv_state:
                rospy.loginfo("Checking block: %dx%d %s", block_loc.width,
                              block_loc.length, block_loc.color)
                # Requested block should have same color, width and length
                if (block_loc.color == block.color
                        and block_loc.width == block.width
                        and block_loc.length == block.length):
                    print("Location to go to is %f %f", block_loc.pose.x,
                          block_loc.pose.y)
                    overhead_pose.position = Point(
                        x=block_loc.pose.x,
                        y=block_loc.pose.y,
                        z=self._hover_distance)
                    overhead_pose.orientation = self._overhead_orientation

        elif (position is not None and block is None):
            overhead_pose.position = position
            overhead_pose.orientation = self._overhead_orientation

        else:
            rospy.loginfo("One of block or position should be None")
            return

        approach = copy.deepcopy(overhead_pose)
        # approach with a pose the hover-distance above the requested pose
        approach.position.z = self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)

    def _align(self, orientation, block=None):
        if block is None:
            # This is a place align. Adjust gripper to orientation and return the new pose
            return self._rotate_gripper(orientation)

        else:
            # This is a pick align. Adjust gripper to orientation and return the new pose
            block_pixel_locs = self._get_updated_pixel_locs()
            block_angle = None

            cam_center_x = 640
            cam_center_y = 400
            rospy.loginfo("Finding correct block")
            while (block_angle is None):
                # First align with blocks major or minor axis
                for pixel_loc in block_pixel_locs:
                    print(pixel_loc)
                    # Requested block should have same color, width and length
                    if (pixel_loc.color == block.color
                            and pixel_loc.width == block.width
                            and pixel_loc.length == block.length):
                        # Also ensure that block is within 100 pixels of camera frame center
                        block_center_distance = math.sqrt(
                            (pixel_loc.x - cam_center_x)**2 +
                            (pixel_loc.y - cam_center_y)**2)
                        if (block_center_distance > 400):
                            rospy.loginfo(
                                "This is not the block you are looking for... It is too far from the center..."
                            )
                            continue
                        else:
                            block_angle = pixel_loc.theta

                block_pixel_locs = self._get_updated_pixel_locs()

            if (block_angle is None):
                rospy.loginfo("No matching block was found! Can't align")
                return None
            else:
                rospy.loginfo(
                    "Aligning with %s colored %dx%d block with angle %f about with orientation %f",
                    block.color, block.width, block.length, block_angle,
                    orientation)

            rospy.loginfo(
                "Sleeping for 1 second before rotating... Check block angle to ensure it is correct"
            )
            rospy.sleep(1)

            # When align gets called, height should be at 0
            pixel_dist_thresh = 3
            pixel_dist = 0

            # At 0.0 meters the location of center pixel that will result in optimal grasp
            # 1280 -> 960
            # 800  -> 600 * 3/4
            # y + 100
            # x + 240
            # Possible new values:
            upper_pixel_center_x = 645
            upper_pixel_center_y = 329
            #upper_pixel_center_x = 325
            #upper_pixel_center_y = 129

            # At -0.15 meters the location of center pixel that will result in optimal grasp
            # Possible new values:
            #lower_pixel_center_x = 650
            #lower_pixel_center_y = 294

            # At -0.16 meters the location of center pixel that will result in optimal grasp
            # Possible new values:
            lower_pixel_center_x = 660
            lower_pixel_center_y = 285
            #lower_pixel_center_x = 330
            #lower_pixel_center_y = 94

            #pixel_center_x = upper_pixel_center_x
            #pixel_center_y = upper_pixel_center_y
            pixel_center_x = lower_pixel_center_x
            pixel_center_y = lower_pixel_center_y
            cam_center_x = 640
            cam_center_y = 400

            # Rotate block_angle by 90 so that gripper will be perpendicular to blocks major axis
            if (orientation == 1):
                rotation_angle = block_angle + math.pi / 2
            # Just rotate by block angle
            elif (orientation == 0):
                rotation_angle = block_angle
            else:
                rospy.logerr(
                    "Rotation should either be a 1 for 90 degree rotation or a 0 for no rotation"
                )

            # Clamp block angle between pi and -pi
            if (rotation_angle > math.pi):
                rospy.loginfo("Rotation angle > math.pi")
                rotation_angle -= (2 * math.pi)
            elif (rotation_angle < -math.pi):
                rospy.loginfo("Rotation angle < -math.pi")
                rotation_angle += (2 * math.pi)
            # TODO: Uncomment to enable initial rotation of gripper

            rospy.loginfo("Rotating gripper by %f degrees",
                          math.degrees(rotation_angle))
            new_pose = self._rotate_gripper(block_angle)

            block_pixel_locs = self._get_updated_pixel_locs()

            # Align at 0.0 meters
            while (True):
                if len(block_pixel_locs) > 0:
                    rospy.loginfo(
                        "There are %d blocks in view of hand camera.",
                        len(block_pixel_locs))
                else:
                    rospy.loginfo(
                        "There are no blocks in view of hand camera.")

                # Go through each pixel location to find the requested block
                for pixel_loc in block_pixel_locs:
                    # Requested block should have same color, width and length
                    if (pixel_loc.color == block.color
                            and pixel_loc.width == block.width
                            and pixel_loc.length == block.length):
                        """
                        # For debugging
                        rospy.loginfo("Block color (from locs): %s W: %d, L: %d", pixel_loc.color, pixel_loc.width, pixel_loc.length)
                        rospy.loginfo("Block color: %s W: %d, L: %d", block_color, block_width, block_length)
                        rospy.loginfo("Block pixel location: x: %f y:%f", pixel_loc.x, pixel_loc.y)
                        """

                        # We found the block we want to allign with
                        pixel_x_dist = pixel_center_x - pixel_loc.x
                        pixel_y_dist = pixel_center_y - pixel_loc.y

                        # Overall distance
                        pixel_dist = math.sqrt(
                            pixel_x_dist**2 + pixel_y_dist**2)
                        rospy.loginfo("Pixel distance is: %f", pixel_dist)

                        if (pixel_dist > 200):
                            rospy.loginfo(
                                "Pixel distance is greater that 200. This probably isn't the correct block..."
                            )
                            continue

                        # TODO uncomment after testing as a failsafe
                        """
                        if(pixel_dist > 100):
                            rospy.loginfo("Pixel distance is off by 100! Something is wrong!")
                            return
                        """

                        if (pixel_dist > pixel_dist_thresh):
                            rospy.loginfo("X_DIST: %f, Y_DIST: %f",
                                          pixel_x_dist, pixel_y_dist)
                            in_frame_angle = math.atan2(
                                -pixel_y_dist, pixel_x_dist)

                            motion_angle = in_frame_angle + rotation_angle

                            rospy.loginfo("Within frame angle is: %f",
                                          math.degrees(in_frame_angle))
                            rospy.loginfo("Rotation angle is %f",
                                          math.degrees(rotation_angle))
                            rospy.loginfo("Baxter's motion angle is %f",
                                          math.degrees(motion_angle))

                            # TODO: tune motion distance and possibly implement a PID that moves proportionally to the distance from goal
                            new_pose = self.move_camera_in_plane(
                                motion_angle + math.pi, motion_dist=.008)
                            rospy.sleep(.5)
                        # TODO uncomment after testing once motion is requested
                        else:

                            rospy.loginfo("Reached within %f of goal",
                                          pixel_dist)
                            return new_pose

                    else:
                        continue

                block_pixel_locs = self._get_updated_pixel_locs()

    # TODO: For testing purposes to give tester access to private _align(), delete later!
    def extern_align(self, block_color, block_length, block_width, axis):
        return self._align(block_color, block_length, block_width, axis)

    def _retract(self):
        self._ascend()

    def _pick(self):
        self._descend()

        # Close the gripper
        self._gripper_close()

    def _place(self):
        self._descend()

        # open the gripper
        self._gripper_open()

    def _detect(self):
        rospy.loginfo(
            "Updating block locations. Waiting for BlockObservationArray...")

        block_obs_msg = rospy.wait_for_message("/block_finder/top/block_obs",
                                               BlockObservationArray)

        rospy.loginfo("Received %d inventory block observations",
                      len(block_obs_msg.inv_obs))
        rospy.loginfo("Received %d workspace block observations",
                      len(block_obs_msg.ws_obs))

        inv_block_locations = []
        ws_block_locations = []

        # Go through inventory block locations
        for block_obs in block_obs_msg.inv_obs:
            inv_block_locations.append(
                Block(
                    length=block_obs.length,
                    width=block_obs.width,
                    color=block_obs.color,
                    pose=block_obs.pose))

        # Go through workspace block locations
        for block_obs in block_obs_msg.ws_obs:
            ws_block_locations.append(
                Block(
                    length=block_obs.length,
                    width=block_obs.width,
                    color=block_obs.color,
                    pose=block_obs.pose))

        self.inv_state = inv_block_locations
        self.ws_state = ws_block_locations

    def _rotate_gripper(self, angle):
        # NOTE: angle in radians!
        q_rot = tf.transformations.quaternion_from_euler(angle, 0, 0)

        curr_pose = self.get_current_pose()

        curr_q_arr = np.array([
            curr_pose.orientation.w, curr_pose.orientation.x,
            curr_pose.orientation.y, curr_pose.orientation.z
        ])

        q_new = tf.transformations.quaternion_multiply(q_rot, curr_q_arr)

        curr_pose.orientation.x = q_new[1]
        curr_pose.orientation.y = q_new[2]
        curr_pose.orientation.z = q_new[3]
        curr_pose.orientation.w = q_new[0]

        joint_angles = self.ik_request(curr_pose)
        self._guarded_move_to_joint_position(joint_angles)

        return curr_pose

    def _ascend(self):
        rospy.loginfo("Ascending to %f", self._hover_distance)
        curr_pose = self.get_current_pose()
        curr_z = curr_pose.position.z

        # Go up until the hover distance is reached
        while (curr_z <= self._hover_distance):
            curr_z += 0.04

            curr_pose.position.z = curr_z
            joint_angles = self.ik_request(curr_pose)

            self._guarded_move_to_joint_position(joint_angles)

    def _descend(self):
        rospy.loginfo("Descending to %f", self._table_distance)
        curr_pose = self.get_current_pose()
        curr_z = curr_pose.position.z

        if (np.fabs(curr_z - self._hover_distance) > 0.01):
            rospy.logerr("z_position should be within 1 cm of %f meters when descend is called, but is %f", self._hover_distance, curr_z)

        while (curr_z >= self._table_distance):
            curr_z -= 0.02
            curr_pose.position.z = curr_z

            joint_angles = self.ik_request(curr_pose)

            self._guarded_move_to_joint_position(joint_angles)

    def get_current_pose(self):
        current_pose = self._limb.endpoint_pose()

        ik_pose = Pose()

        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        return ik_pose

    def move_camera_in_plane(self, direction, motion_dist=0.005):
        if (direction is None):
            return

        rospy.loginfo("Moving camera in plane in direction %f ",
                      math.degrees(direction))

        current_pose = self._limb.endpoint_pose()

        ik_pose = Pose()

        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w

        ik_pose_q = (ik_pose.orientation.x, ik_pose.orientation.y,
                     ik_pose.orientation.z, ik_pose.orientation.w)

        ik_pose_euler = tf.transformations.euler_from_quaternion(ik_pose_q)
        rospy.loginfo("Roll: %f Pitch: %f Yaw %f", ik_pose_euler[0],
                      ik_pose_euler[1], ik_pose_euler[2])

        ik_pose.position.x = current_pose['position'].x + motion_dist * math.sin(
            direction)  # + ik_pose_euler[2])
        ik_pose.position.y = current_pose['position'].y - motion_dist * math.cos(
            direction)  # + ik_pose_euler[2])
        ik_pose.position.z = current_pose['position'].z

        joint_angles = self.ik_request(ik_pose)
        self._guarded_move_to_joint_position(joint_angles)

        return ik_pose

    # PLANNING interface
    def executor(self, action, constraints=Constraints()):
        """Executor: Interfaces with planner, receives action and actuates Baxter"""
        # if the action does not require constraint, check constraint == None
        if action is PrimitiveActions.pick or \
        action is PrimitiveActions.place or \
        action is PrimitiveActions.retract or \
        action is PrimitiveActions.detect:
            if not constraints.is_empty_constraints():
                rospy.logerr("%s should not be passed any arguments", action)
            else:
                self._actions[action]()
        else:
            if action == PrimitiveActions.transport:
                # We want to move above a block
                if constraints.is_block_constraints():
                    rospy.loginfo("Constraint is a block constraint")
                    # TODO: need a way to distinguish between whether the block
                    # that the gripper is to be transported to is in INV or WS
                    # NOTE: I don't think a block will ever get _transport called
                    # on it if unless it is in the invent

                    self._actions[action](constraints.block, None)
                elif (constraints.is_position_constraints()):
                    rospy.loginfo("Constraint is a position constraint")
                    pose_constraint = constraints.position
                    self._actions[action](None, pose_constraint)
                else:
                    rospy.logerr("%s must be passed arguments", action)
            elif action == PrimitiveActions.align:
                if (constraints.is_block_constraints()):
                    block_constraint = constraints.block
                    orientation_constraint = constraints.orientation
                    # We have a block that we wish to align with
                    self._actions[action](orientation_constraint,
                                          block_constraint)
                else:
                    rospy.logerr("%s must be passed arguments", action)
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
