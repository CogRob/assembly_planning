#!/usr/bin/env python
"""
Customized versions of utility examples created by Rethink Robotics for Baxter
"""

import rospy

import baxter_interface

from baxter_interface import CHECK_VERSION


class JointRecorder(object):
    def __init__(self,
                 arm='both',
                 mode='arm',
                 gripper=False,
                 filename='joint_recorder',
                 rate=10):
        """
        Records joint data to a file at a specified rate.
        """
        self._filename = filename
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = False
        self._arm = arm
        self._gripper = gripper

        if arm == 'left' or arm == 'both':
            self._limb_left = baxter_interface.Limb("left")
            if gripper:
                self._gripper_left = baxter_interface.Gripper(
                    "left", CHECK_VERSION)
                self._io_left_upper = baxter_interface.DigitalIO(
                    'left_upper_button')
        if arm == 'right' or arm == 'both':
            self._limb_right = baxter_interface.Limb("right")
            if gripper:
                self._gripper_right = baxter_interface.Gripper(
                    "right", CHECK_VERSION)
                self._io_right_upper = baxter_interface.DigitalIO(
                    'right_upper_button')

        # Verify Grippers Have No Errors and are Calibrated
        if gripper:
            if self._gripper_left.error():
                self._gripper_left.reset()
            if self._gripper_right.error():
                self._gripper_right.reset()
            if (not self._gripper_left.calibrated()
                    and self._gripper_left.type() != 'custom'):
                self._gripper_left.calibrate()
            if (not self._gripper_right.calibrated()
                    and self._gripper_right.type() != 'custom'):
                self._gripper_right.calibrate()

    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    def stop(self):
        """
        Stop recording.
        """
        self._done = True
        rospy.loginfo("Recording stopped.")

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def record(self):
        """
        Records the current joint positions to a csv file if outputFilename was
        provided at construction this function will record the latest set of
        joint angles in a csv format.
        This function does not test to see if a file exists and will overwrite
        existing files.
        """
        if self._filename:
            rospy.loginfo("Recording trajectory...")
            with open(self._filename, 'w') as f:
                f.write('time')
                if self._arm == 'left' or self._arm == 'both':
                    joints_left = self._limb_left.joint_names()
                    f.write(',' + ','.join([j for j in joints_left]))
                    if self._gripper:
                        f.write(',left_gripper')
                if self._arm == 'right' or self._arm == 'both':
                    joints_right = self._limb_right.joint_names()
                    f.write(',' + ','.join([j for j in joints_right]))
                    if self._gripper:
                        f.write(',right_gripper')
                f.write('\n')

                while not self.done():
                    # Look for gripper button presses
                    if self._gripper:
                        pass
                        # TODO fix for upper to be used for both open/close
                        # if self._io_left_lower.state:
                        #     self._gripper_left.open()
                        # elif self._io_left_upper.state:
                        #     self._gripper_left.close()
                        # if self._io_right_lower.state:
                        #     self._gripper_right.open()
                        # elif self._io_right_upper.state:
                        #     self._gripper_right.close()

                    f.write("%f" % (self._time_stamp(), ))

                    if self._arm == 'left' or self._arm == 'both':
                        angles_left = [
                            self._limb_left.joint_angle(j) for j in joints_left
                        ]
                        f.write(',' + ','.join([str(x) for x in angles_left]))

                        if self._gripper:
                            f.write(',' + str(self._gripper_left.position()))

                    if self._arm == 'right' or self._arm == 'both':
                        angles_right = [
                            self._limb_right.joint_angle(j)
                            for j in joints_right
                        ]
                        f.write(',' + ','.join([str(x) for x in angles_right]))
                        if self._gripper:
                            f.write(',' + str(self._gripper_right.position()))

                    f.write('\n')
                    self._rate.sleep()