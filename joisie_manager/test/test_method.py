# Imports stolen from task manager
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import String, Bool, Header
from vision_msgs.msg import Detection2D
from terrawarden_interfaces.msg import DroneTelemetry, DroneWaypoint, ArmStatus
from geometry_msgs.msg import PoseStamped, Pose
from builtin_interfaces.msg import Time
import tf2_ros
from std_srvs.srv import Empty
import cv2
import numpy as np
import os
from collections.abc import Callable
from collections import deque
import math
from enum import Enum, auto
import time
import pytest
from ../joisie_manager.taskmanager import *

from .test_helpers import *

@pytest.fixture
def manager():
    # Runs before tests
    rclpy.init()
    manager = TaskManagerNode()
    yield manager
    # Runs after tests
    rclpy.shutdown()

# HELPERS FOR INSTANTIATING TASK MANAGER AND MOCKING TELEMETRY

def get_state_setter(task_manager):
    return get_publisher(task_manager, String, "joisie_set_state")


# # Function name starts with "test_"
# @pytest.mark.skip(reason="not implemented")
# def test_math(manager):
#     assert 2 + 2 == 5   # This should fail for most mathematical systems\

@pytest.mark.skip(reason="not implemented")
def test_send_waypoint_NED(manager):
    assert 2 + 2 == 5  

# @pytest.mark.skip(reason="not implemented")
def test_offset_point_FLU(manager):

    FLUpoint = create_point(1,1,1)

    FLUoffset = create_point(0, 0, 0)
    assert manager.offsetPointFLU(FLUpoint, FLUoffset) == create_point(1, 1, 1) # + none

    FLUoffset = create_point(0.5, 0, 0)
    assert manager.offsetPointFLU(FLUpoint, FLUoffset) == create_point(1.5, 1, 1) # + X

    FLUoffset = create_point(0, 0.5, 0)
    assert manager.offsetPointFLU(FLUpoint, FLUoffset) == create_point(1, 1.5, 1) # + Y

    FLUoffset = create_point(0, 0, 0.5)
    assert manager.offsetPointFLU(FLUpoint, FLUoffset) == create_point(1, 1, 1.5) # + Z

    FLUoffset = create_point(0.5, 0.5, 0.5)
    assert manager.offsetPointFLU(FLUpoint, FLUoffset) == create_point(1.5, 1.5, 1.5) # + all

    FLUoffset = create_point(-0.5, 0, 0)
    assert manager.offsetPointFLU(FLUpoint, FLUoffset) == create_point(.5, 1, 1) # - X

    FLUoffset = create_point(0, -0.5, 0)
    assert manager.offsetPointFLU(FLUpoint, FLUoffset) == create_point(1, .5, 1) # - Y

    FLUoffset = create_point(0, 0, -0.5)
    assert manager.offsetPointFLU(FLUpoint, FLUoffset) == create_point(1, 1, .5) # - Z

    FLUoffset = create_point(-0.5, -0.5, -0.5)
    assert manager.offsetPointFLU(FLUpoint, FLUoffset) == create_point(0.5, 0.5, 0.5) # - all

# @pytest.mark.skip(reason="not implemented")
def test_FLU_to_NED(manager):
    # make publisher
    pub = get_publisher(manager, DroneTelemetry, "drone_telemetry_topic")

    dronePoint = create_point(0,0,0)
    msg = mock_telemetry(dronePoint) #only care about position here
    
    pub.publish(msg)

    FLUoffsetPoint = create_point(1, 2, 3)
    assert manager.FLU2NED(FLUoffsetPoint, 0) = create_point(1, 2, 3) # basic, no heading incorporated
    assert manager.FLU2NED(FLUoffsetPoint, 90.0) = create_point(2, 1, 3) #90 degree turn

@pytest.mark.skip(reason="not implemented")
def test_FLU_to_NED_quaternion(manager):
    # make publisher
    pub = get_publisher(manager, DroneTelemetry, "drone_telemetry_topic")

    dronePoint = create_point(0,0,0)
    msg = mock_telemetry(dronePoint) #only care about position here
    
    pub.publish(msg)

    FLUoffsetPoint = create_point(1, 2, 3)
    assert manager.FLU2NED_quaternion(FLUoffsetPoint) == create_point(1, 2, 3) # basic, no heading incorporated

    # turn drone 90 degress
    assert manager.FLU2NED_quaternion(FLUoffsetPoint) == create_point(2, 1, 3) #90 degree turn
 

def test_is_in_range_NED(manager):
    # make publisher
    pub = get_publisher(manager, DroneTelemetry, "drone_telemetry_topic")

    dronePoint = create_point(0,0,0)
    msg = mock_telemetry(dronePoint) #only care about position here
    
    pub. publish(msg)

    refRange = create_point(1.2, 0, 1.2)
    assert manager.isInRangeNED(refRange, 1.1, 1.5) == False #out of range +X
    assert manager.isInRangeNED(refRange, 1.5, 1.1) == False #out of range +Z
    assert manager.isInRangeNED(refRange, 1.5, 1.5) == True #in range X, Y, Z

    refRange = create_point(0, 1.2, 0)
    assert manager.isInRangeNED(refRange, 1.1, 1) == False #out of range +Y
    assert manager.isInRangeNED(refRange, 1.5, 1.5) == True #in range X, Y, Z

@pytest.mark.skip(reason="not implemented")
def test_check_for_errors(manager):
    assert 2 + 2 == 5  

@pytest.mark.skip(reason="not implemented")
def test_check_failsafe(manager):
    assert 2 + 2 == 5  

# @pytest.mark.skip(reason="not used - not priority")
# def test_quaternion_to_heading(manager):
#     assert 2 + 2 == 5  

