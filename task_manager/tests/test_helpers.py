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

from task_manager.taskmanager import *

def get_publisher(task_manager: TaskManagerNode, message_type, ros_param_name: str):
    return task_manager.create_publisher(message_type,
        task_manager.get_parameter(ros_param_name).value, 10)

def mock_telemetry(pos: tuple[float, float, float] = (0,0,0), 
                   vel: tuple[float, float, float] = (0,0,0),
                   gps: NavSatFix = NavSatFix(),
                   altitude_above_ground: float = 1,
                   heading_degrees: float = 0,
                   battery_percentage: float = 60,
                   error: str = "",
                   is_flying: bool = True,
                   is_offboard: bool = True,
                   has_rc_link: bool = True,
                   timestamp_seconds: float = 0):
    """
        Helper function for retrieving a mocked telemetry input so you don't need
        To specify every parameter while writing unit tests.

        Use: 
        telemetry_msg = mock_telemetry(pos=(x,y,z), timestamp=Time(sec=1, nanosec=5e-8))
    """
    # Convert pos from tuple to PoseStamped
    pos = PoseStamped(pose=Pose(x=pos[0],y=pos[1],z=pos[2]))
    # Split time into seconds and fractional seconds
    sec, sec_frac = math.modf(timestamp_seconds)
    pos.header.stamp = Time(sec=int(sec), nanosec=int(sec_frac*1e9))
    vel = PoseStamped(pose=Pose(x=vel[0],y=vel[1],z=vel[2]))
    
    return DroneTelemetry(pos=pos, 
                          vel=vel, 
                          gps=gps, 
                          altitude_above_ground=altitude_above_ground,
                          heading_degrees=heading_degrees,
                          battery_percentage=battery_percentage,
                          error=error,
                          is_flying=is_flying,
                          is_offboard=is_offboard,
                          has_rc_link=has_rc_link)