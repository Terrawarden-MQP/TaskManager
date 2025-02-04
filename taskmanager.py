# ROS2 imports 
import rclpy
from rclpy.node import Node

# CV Bridge and message imports
from sensor_msgs.msg import Image
from std_msgs.msg import String
from vision_msgs.msg import ObjectHypothesisWithPose, BoundingBox2D, Detection2D, Detection2DArray
from cv_bridge import CvBridge, CvBridgeError

import cv2
import numpy as np
import os

from enum import Enum, auto

class State(Enum):
    HOLD = auto()
    SEARCHING = auto()
    NAVIGATING = auto()
    GRASPING = auto()
    DEPOSITING = auto()

class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('trt_detection_node')

        # Create a subscriber to the Image topic
        # topic can be changed through parameters in ros2 launch command
        self.declare_parameter('topic', 'image')
        topic_name = self.get_parameter('topic')
        self.subscription = self.create_subscription(Image, topic_name.value, self.listener_callback, 10)
        self.detection_publisher = self.create_publisher(Detection2DArray, 'trt_detection', 10)

        # C

    def onReceive(self, ros_msg):
        #
        pass

    def hold(self):
        pass
    
    def search(self):
        # flies in whatever search pattern we decide upon
        # when an object is detected, move to navigate state
        pass

    def navigate(self):
        # navigates drone to be within range of object
        # if object is within range, move to grasp state
        # if object is lost, return to search state
        pass

    def grasp(self):
        # generates and exectues object grasp
        pass
    
    def deposit(self):
        # drops the object
        #  pass this for now
        pass

    def checkForErrors(self):
        pass

    def main(self):
        while True:
            pass