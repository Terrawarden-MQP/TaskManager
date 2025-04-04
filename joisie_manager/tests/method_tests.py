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

from joisie_manager.taskmanager import *


# HELPERS FOR INSTANTIATING TASK MANAGER AND MOCKING TELEMETRY
from test_helpers import *

def get_state_setter(task_manager):
    return get_publisher(task_manager, String, "joisie_set_state")

def test_send_waypoint_NED():
    assert 2 + 2 == 5  

def test_offset_point_FLU():
    assert 2 + 2 == 5  

def test_FLU_to_NED():
    assert 2 + 2 == 5  

def test_FLU_to_NED_quaternion():
    assert 2 + 2 == 5  

def test_is_in_range_NED():
    assert 2 + 2 == 5  

def test_quaternion_to_heading():
    assert 2 + 2 == 5  

def test_check_for_errors():
    assert 2 + 2 == 5  

def test_check_failsafe():
    assert 2 + 2 == 5  

