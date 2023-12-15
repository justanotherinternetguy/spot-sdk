# Copyright (c) 2022 Boston Dynamics, Inc.  All rights reserved.
#
# Downloading, reproducing, distributing or otherwise using the SDK Software
# is subject to the terms and conditions of the Boston Dynamics Software
# Development Kit License (20191101-BDSDK-SL).

"""Tutorial to show how to use the Boston Dynamics API"""
from __future__ import print_function

import argparse
import os
import sys
import time
import math
import random

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.client.image import ImageClient
from bosdyn.api import arm_command_pb2, geometry_pb2
from bosdyn.client import math_helpers
from bosdyn.client.frame_helpers import GRAV_ALIGNED_BODY_FRAME_NAME, ODOM_FRAME_NAME, get_a_tform_b
from bosdyn.client.robot_command import (RobotCommandBuilder, RobotCommandClient,
                                         block_until_arm_arrives, blocking_stand)
from bosdyn.client.robot_state import RobotStateClient


# C:\Users\alexr\AppData\Local\Programs\Python\Python311\python.exe python\examples\hello_spot\hello_spot.py 192.168.80.3
def hello_spot(config):
    """A simple example of using the Boston Dynamics API to command a Spot robot."""

    # The Boston Dynamics Python library uses Python's logging module to
    # generate output. Applications using the library can specify how
    # the logging information should be output.
    bosdyn.client.util.setup_logging(config.verbose)

    # The SDK object is the primary entry point to the Boston Dynamics API.
    # create_standard_sdk will initialize an SDK object with typical default
    # parameters. The argument passed in is a string identifying the client.
    sdk = bosdyn.client.create_standard_sdk('HelloSpotClient')

    # A Robot object represents a single robot. Clients using the Boston
    # Dynamics API can manage multiple robots, but this tutorial limits
    # access to just one. The network address of the robot needs to be
    # specified to reach it. This can be done with a DNS name
    # (e.g. spot.intranet.example.com) or an IP literal (e.g. 10.0.63.1)
    robot = sdk.create_robot(config.hostname)

    # Clients need to authenticate to a robot before being able to use it.
    bosdyn.client.util.authenticate(robot)

    # Establish time sync with the robot. This kicks off a background thread to establish time sync.
    # Time sync is required to issue commands to the robot. After starting time sync thread, block
    # until sync is established.
    robot.time_sync.wait_for_sync()

    # Verify the robot is not estopped and that an external application has registered and holds
    # an estop endpoint.
    assert not robot.is_estopped(), "Robot is estopped. Please use an external E-Stop client, " \
                                    "such as the estop SDK example, to configure E-Stop."

    # Only one client at a time can operate a robot. Clients acquire a lease to
    # indicate that they want to control a robot. Acquiring may fail if another
    # client is currently controlling the robot. When the client is done
    # controlling the robot, it should return the lease so other clients can
    # control it. The LeaseKeepAlive object takes care of acquiring and returning
    # the lease for us.

    lease_client = robot.ensure_client(bosdyn.client.lease.LeaseClient.default_service_name)
    with bosdyn.client.lease.LeaseKeepAlive(lease_client, must_acquire=True, return_at_exit=True):
        # Now, we are ready to power on the robot. This call will block until the power
        # is on. Commands would fail if this did not happen. We can also check that the robot is
        # powered at any point.
        robot.logger.info("Powering on robot... This may take several seconds.")
        robot.power_on(timeout_sec=20)
        assert robot.is_powered_on(), "Robot power on failed."
        robot.logger.info("Robot powered on.")

        # Tell the robot to stand up. The command service is used to issue commands to a robot.
        # The set of valid commands for a robot depends on hardware configuration. See
        # RobotCommandBuilder for more detailed examples on command building. The robot
        # command service requires timesync between the robot and the client.
        robot.logger.info("Commanding robot to stand...")
        command_client = robot.ensure_client(RobotCommandClient.default_service_name)
        blocking_stand(command_client, timeout_sec=10)
        robot.logger.info("Robot standing.")
        time.sleep(3)



        # while True:
        #     distance = float(input("Distance (meters): "))
        #     velocity = float(input("Velocity (m/s): "))
        #     cmd = RobotCommandBuilder.synchro_velocity_command(v_x=velocity, v_y=0, v_rot=0)
        #     command_client.robot_command(command=cmd, end_time_secs=time.time() + distance / velocity)
        #     robot.logger.info("Finished Moving")
        #     time.sleep(1)

        def calculateAngle(p1, p2):
            # p1 is your point, p2 is other point
            # First value of coordinate needs to be x value and second value needs to be y value
            angle = math.degrees(math.atan2(p2[1] - p1[1], p2[0] - p1[0]))
            return angle


        while True:
            x = float(input("Move to: Relative X Position (meters): "))
            y = float(input("Move to: Relative Y Position (meters): "))
            rotation_velocity = math.radians(float(input("Rotation Velocity (degrees/s): ")))
            shortest_angle_in_radians = math.radians(calculateAngle((0, 0), (x, y)))
            rotation_velocity = math.copysign(rotation_velocity, shortest_angle_in_radians)
            cmd_rotate = RobotCommandBuilder.synchro_velocity_command(v_x=0, v_y=0, v_rot=rotation_velocity)

            # If angle greater than 90 degrees, first turn 90 degrees then turn the rest of the way
            if abs(shortest_angle_in_radians) > math.pi/2:
                ninetyDeg_in_radians = math.copysign((math.pi / 2), shortest_angle_in_radians)
                first_turn_duration = ninetyDeg_in_radians / rotation_velocity
                command_client.robot_command(command=cmd_rotate, end_time_secs=time.time() + first_turn_duration)
                time.sleep(first_turn_duration)
                shortest_angle_in_radians -= ninetyDeg_in_radians

            time.sleep(0.1)
            # Turn to angle
            turn_duration = shortest_angle_in_radians/rotation_velocity
            command_client.robot_command(command=cmd_rotate, end_time_secs=time.time() + turn_duration)
            time.sleep(turn_duration)

            # velocity = float(input("Velocity X (m/s): ")) # use line below
            translational_velocity = float(input("Translational Velocity (movement x, y) (m/s): "))
            distance = math.dist((0, 0), (x, y))
            # Move to point
            cmd_move = RobotCommandBuilder.synchro_velocity_command(v_x=translational_velocity, v_y=0, v_rot=0)
            move_duration = distance/translational_velocity
            command_client.robot_command(command=cmd_move, end_time_secs=time.time() + move_duration)

            robot.logger.info("Finished Moving")
            time.sleep(1)

        # Movement to coords with both types of motion
        # while True:
        #     x = float(input("Relative X Position: "))
        #     y = float(input("Relative Y Position: "))
        #     velocity = float(input("Velocity (m/s): "))
        #     cmd_x = RobotCommandBuilder.synchro_velocity_command(v_x=velocity, v_y=0, v_rot=0)
        #     cmd_y = RobotCommandBuilder.synchro_velocity_command(v_x=0, v_y=velocity, v_rot=0)
        #     cmd = RobotCommandBuilder.build_synchro_command(cmd_y, cmd_x)
        #     command_client.robot_command(command=cmd, end_time_secs=time.time() + max(x/velocity, y/velocity))
        #     robot.logger.info("Finished Moving")
        #     time.sleep(1)

        # Now tell the robot to stand taller, using the same approach of constructing
        # a command message with the RobotCommandBuilder and issuing it with
        # robot_command.
        cmd = RobotCommandBuilder.synchro_stand_command(body_height=0.1)
        command_client.robot_command(cmd)
        robot.logger.info("Robot standing tall.")
        time.sleep(3)

        cm = RobotCommandBuilder.synchro_velocity_command(v_x=0.9, v_y=0, v_rot=0) # MOVEMENT YAY
        command_client.robot_command(command=cm, end_time_secs=time.time() + 1)
        robot.logger.info("Robot moving forward.")
        time.sleep(3)

        assert robot.has_arm(), "Robot requires an arm to run this code."

        command_client.robot_command(RobotCommandBuilder.arm_ready_command())
        robot.logger.info("arm ready")
        time.sleep(3)

        x, y, z = 0.5, 0.5, 0
        command_client.robot_command(RobotCommandBuilder.arm_gaze_command(x=x, y=y, z=z, frame_name="body")) # odom, vision, body, flat_body, gpe (https://dev.bostondynamics.com/docs/concepts/geometry_and_frames)
        robot.logger.info(f"arm gazing at {x}, {y}, {z}")
        time.sleep(3)

        command_client.robot_command(RobotCommandBuilder.claw_gripper_open_command())
        robot.logger.info("claw open")
        time.sleep(3)

        command_client.robot_command(RobotCommandBuilder.claw_gripper_close_command())
        robot.logger.info("claw open")
        time.sleep(3)


        command_client.robot_command(RobotCommandBuilder.arm_stow_command())
        robot.logger.info("arm stowed")
        time.sleep(3)


        # Log a comment.
        # Comments logged via this API are written to the robots test log. This is the best way
        # to mark a log as "interesting". These comments will be available to Boston Dynamics
        # devs when diagnosing customer issues.
        log_comment = "HelloSpot tutorial user comment."
        robot.operator_comment(log_comment)
        robot.logger.info('Added comment "%s" to robot log.', log_comment)

        # Power the robot off. By specifying "cut_immediately=False", a safe power off command
        # is issued to the robot. This will attempt to sit the robot before powering off.
        robot.power_off(cut_immediately=False, timeout_sec=20)
        assert not robot.is_powered_on(), "Robot power off failed."
        robot.logger.info("Robot safely powered off.")



def main(argv):
    """Command line interface."""
    parser = argparse.ArgumentParser()
    bosdyn.client.util.add_base_arguments(parser)
    """
    parser.add_argument(
        '-s', '--save', action='store_true', help=
        'Save the image captured by Spot to the working directory. To chose the save location, use --save_path instead.'
    )
    parser.add_argument(
        '--save-path', default=None, nargs='?', help=
        'Save the image captured by Spot to the provided directory. Invalid path saves to working directory.'
    )
    """
    options = parser.parse_args(argv)
    try:
        hello_spot(options)
        return True
    except Exception as exc:  # pylint: disable=broad-except
        logger = bosdyn.client.util.get_logger()
        logger.error("Hello, Spot! threw an exception: %r", exc)
        return False


if __name__ == '__main__':
    if not main(sys.argv[1:]):
        sys.exit(1)
