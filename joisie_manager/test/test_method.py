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
from ..joisie_manager.taskmanager import *

from .test_helpers import *

@pytest.fixture
def rclpy_sucks():
    # Runs before tests
    rclpy.init()
    yield
    # Runs after tests
    rclpy.shutdown()

# HELPERS FOR INSTANTIATING TASK MANAGER AND MOCKING TELEMETRY

def get_state_setter(task_manager):
    return get_publisher(task_manager, String, "joisie_set_state")


# Function name starts with "test_"
@pytest.mark.skip(reason="not implemented")
def test_math(rclpy_sucks):
    assert 2 + 2 == 5   # This should fail for most mathematical systems\

@pytest.mark.skip(reason="not implemented")
def test_send_waypoint_NED(rclpy_sucks):
    assert 2 + 2 == 5  

@pytest.mark.skip(reason="not implemented")
def test_offset_point_FLU(rclpy_sucks):
    assert 2 + 2 == 5  

@pytest.mark.skip(reason="not implemented")
def test_FLU_to_NED(rclpy_sucks):
    assert 2 + 2 == 5  

@pytest.mark.skip(reason="not implemented")
def test_FLU_to_NED_quaternion(rclpy_sucks):
    assert 2 + 2 == 5  

def test_is_in_range_NED(rclpy_sucks):
    manager = TaskManagerNode()

    dronePoint = [0,0,0]
    mock_telemetry(dronePoint) #only care about position here
    # make publisher
    # publish mock_telemetry

    refRange = Point(x=1.2,y=0.,z=1.2)
    assert manager.isInRangeNED(refRange, 1.1, 1.5) == False #out of range X
    assert manager.isInRangeNED(refRange, 1.5, 1.1) == False #out of range Z
    assert manager.isInRangeNED(refRange, 1.5, 1.5) == True #in range X, Y, Z

    refRange = Point(x=0., y=1.2, z=0.)
    assert manager.isInRangeNED(refRange, 1.1, 1) == False #out of range Y
    assert manager.isInRangeNED(refRange, 1.5, 1.5) == True #in range X, Y, Z



@pytest.mark.skip(reason="not implemented")
def test_quaternion_to_heading():
    assert 2 + 2 == 5  

@pytest.mark.skip(reason="not implemented")
def test_check_for_errors():
    assert 2 + 2 == 5  

@pytest.mark.skip(reason="not implemented")
def test_check_failsafe():
    assert 2 + 2 == 5  

