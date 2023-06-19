from __future__ import print_function

import argparse
import os
import sys
import time
import random

import bosdyn.client
import bosdyn.client.lease
import bosdyn.client.util
import bosdyn.geometry
from bosdyn.client.image import ImageClient
from bosdyn.client.robot_command import RobotCommandBuilder, RobotCommandClient, blocking_stand

from bosdyn.api import *
from bosdyn.api.docking import * 
from bosdyn.api.graph_nav import *
from bosdyn.api.mission import *
from bosdyn.mission import *


if __name__ == "__main__":
    power_on = nodes_pb2.BosdynPowerRequest(service_name='power',

                                        host='localhost',
                                        request=power_pb2.PowerCommandRequest.REQUEST_ON)

    power_on_mission = nodes_pb2.Node(name='Just power on')
    power_on_mission.impl.Pack(power_on)

    request = basic_command_pb2.StandCommand.Request()
    mobility_command = mobility_command_pb2.MobilityCommand.Request(stand_request=request)
    robot_command = robot_command_pb2.RobotCommand(mobility_command=mobility_command)
    stand = nodes_pb2.BosdynRobotCommand(service_name='robot-command',

                                 host='localhost',
                                 command=robot_command)

    stand_mission = nodes_pb2.Node(name='Just stand')
    stand_mission.impl.Pack(stand)

    request = basic_command_pb2.SitCommand.Request()
    mobility_command = mobility_command_pb2.MobilityCommand.Request(sit_request=request)
    robot_command = robot_command_pb2.RobotCommand(mobility_command=mobility_command)
    sit = nodes_pb2.BosdynRobotCommand(service_name='robot-command',
                               host='localhost', command=robot_command)

    sit_mission = nodes_pb2.Node(name='Just sit')
    sit_mission.impl.Pack(sit)

    sequence = nodes_pb2.Sequence()
    sequence.children.add().CopyFrom(power_on_mission)
    sequence.children.add().CopyFrom(stand_mission)
    sequence.children.add().CopyFrom(sit_mission)

    mission = nodes_pb2.Node(name='Power on then stand then sit')
    mission.impl.Pack(sequence)


