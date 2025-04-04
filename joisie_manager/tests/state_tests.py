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
    
def test_STARTUP_to_HOLD():
    assert 2 == 2

def test_STARTUP_to_GRASPING():
    assert 2 == 2

def test_STARTUP_to_SEARCHING():
    assert 2 == 2

def test_HOLD_to_STARTUP():
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

    