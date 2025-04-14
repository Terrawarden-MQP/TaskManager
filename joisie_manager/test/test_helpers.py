from __future__ import annotations
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

def get_publisher(task_manager: TaskManagerNode, message_type: type, ros_param_name: str):
    return task_manager.create_publisher(message_type,
        task_manager.get_parameter(ros_param_name).value, 10)

def mock_telemetry(pos: tuple[float, float, float] = (0.,0.,0.), 
                   vel: tuple[float, float, float] = (0.,0.,0.),
                   gps: NavSatFix = NavSatFix(),
                   altitude_above_ground: float = 1.,
                   heading_degrees: float = 0.,
                   battery_percentage: float = 60.,
                   error: str = "",
                   is_flying: bool = True,
                   is_offboard: bool = True,
                   has_rc_link: bool = True,
                   timestamp_seconds: float = 0.):
    """
        Helper function for retrieving a mocked telemetry input so you don't need
        To specify every parameter while writing unit tests.

        Use: 
        telemetry_msg = mock_telemetry(pos=(x,y,z), timestamp=Time(sec=1, nanosec=5e-8))
    """
    # Convert pos from tuple to PoseStamped
    pos = PoseStamped(pose=Pose(position=Point(x=float(pos[0]),y=float(pos[1]),z=float(pos[2]))))
    # Split time into seconds and fractional seconds
    sec, sec_frac = math.modf(timestamp_seconds)
    pos.header.stamp = Time(sec=int(sec), nanosec=int(sec_frac*1e9))
    vel = PoseStamped(pose=Pose(position=Point(x=float(vel[0]),y=float(vel[1]),z=float(vel[2]))))
    
    return DroneTelemetry(pos=pos, 
                          vel=vel, 
                          gps=gps, 
                          altitude_above_ground=float(altitude_above_ground),
                          heading_degrees=float(heading_degrees),
                          battery_percentage=float(battery_percentage),
                          error=error,
                          is_flying=is_flying,
                          is_offboard=is_offboard,
                          has_rc_link=has_rc_link)

def timeout_condition(timeout_s: float, condition: Callable[[], bool]):
    """Simple test helper for putting a timeout on a condition occuring"""
    start_time = time.time()
    did_condition_occur = False
    while time.time() <= start_time + float(timeout_s):
        did_condition_occur = condition()
        if did_condition_occur:
            return True
    return False

def create_point(x: float, y: float, z: float):
    point = Point(x=float(x), y=float(y), z=float(z))
    return point

# to run tests: use 
def test_math():
    assert 2 + 2 == 5   # This should fail for most mathematical systems\


