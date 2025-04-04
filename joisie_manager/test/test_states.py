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


# HELPERS FOR INSTANTIATING TASK MANAGER AND MOCKING TELEMETRY
from .test_helpers import *

@pytest.fixture
def manager():
    # Runs before tests
    rclpy.init()
    manager = TaskManagerNode()
    yield manager
    # Runs after tests
    rclpy.shutdown()

# Function name starts with "test_"
def test_math():
    assert 2 + 2 == 5   # This should fail for most mathematical systems

# class State(Enum):
#     STARTUP = "STARTUP"
#     FAILSAFE = "FAILSAFE"
#     LANDING = "LAND"

#     WAITING = "WAIT"
#     HOLD = "HOLD"

#     SEARCHING = "SEARCH"
#     NAVIGATING = "NAV"
#     GRASPING = "GRASP"
#     DEPOSITING = "DEPOSIT"



# state_publisher = get_publisher(manager, String, 'state_setter_topic')
# state_msg = String("HOLD")
# state_publisher.publish(state_msg)
    
def state_change_condition(manager, new_state):
        # Allow manager to run and process incoming info
        # While waiting for the state change
        def condition():
            rclpy.spin_once(manager)
            return manager.state == new_state
        # return callable version
        return condition


def test_STARTUP_to_HOLD(manager):
    #initiate publishers
    telemetry_pub = get_publisher(manager, DroneTelemetry, 'drone_telemetry_topic')
    state_publisher = get_publisher(manager, String, 'state_setter_topic')

    #fake messages
    telemetry_msg = mock_telemetry((1.,-1.,0.5)) # fake message of drone being at xyz position (1,0,0)
    state_msg = String("HOLD") # fake drone in hold position

    #publishes messages
    telemetry_pub.publish(telemetry_msg) # Need a telemetry message to tell the drone it is in offboard mode and flying
    state_publisher.publish(state_msg) # tell the drone it's holding

    assert timeout_condition(5, state_change_condition(manager, State.HOLD))

def test_STARTUP_to_GRASPING(manager):
    # manager = TaskManagerNode()
    telemetry_pub = get_publisher(manager, DroneTelemetry, 'drone_telemetry_topic')
    telemetry_msg = mock_telemetry()

    # state_publisher = get_publisher(manager, String, 'state_setter_topic')
    # state_msg = String("HOLD")
    # state_publisher.publish(state_msg)
    
    # Need a telemetry message to tell the drone it is in offboard mode and flying
    telemetry_pub.publish(telemetry_msg)

    assert timeout_condition(5, state_change_condition(manager, State.HOLD))

def test_STARTUP_to_SEARCHING(manager):

    assert 2 == 2

def test_HOLD_to_STARTUP(manager):
    assert 2 == 2

def test_SEARCHING_to_WAITING_to_NAVIGATING():
    assert 2 == 2

def test_NAVIGATING_to_WAITING_to_GRASPING():
    assert 2 == 2

def test_GRASPING_behavior():
    assert 2 == 2

def test_SEARCHING_behavior():
    assert 2 == 2

def test_NAVIGATING_behavior():
    assert 2 == 2

    