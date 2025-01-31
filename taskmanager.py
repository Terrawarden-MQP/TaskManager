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

class TRTDetectionNode(Node):

    def __init__(self):
        super().__init__('trt_detection_node')

        # Create a subscriber to the Image topic
        self.declare_parameter('topic', 'image')
        topic_name = self.get_parameter('topic')
        self.subscription = self.create_subscription(Image, topic_name.value, self.listener_callback, 10)
        self.detection_publisher = self.create_publisher(Detection2DArray, 'trt_detection', 10)

        # C