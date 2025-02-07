# ROS2 imports 
import rclpy
from rclpy.node import Node

# CV Bridge and message imports
from sensor_msgs.msg import Image
from std_msgs.msg import String, Boolean
from vision_msgs.msg import Detection2D
from geometry_msgs.msg import Point2D, PoseWithCovariance, PoseStamped

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

        ### TOPIC DECLARATION - ALL PARAMETERIZED THROUGH ROS2 LAUNCH

        # Topic for receiving raw image from camera
        self.declare_parameter('image_topic', 'image')
        # Topic for receiving Detection from LiveDetect Node
        self.declare_parameter('detection_topic', 'joisie_detection')
        # Topic for sending detected object centroid to VBM
        self.declare_parameter('centroid_topic', 'joisie_detected_centroid')
        # Topic for receiving Telemetry from Drone Node
        # self.declare_parameter('telemetry', 'joisie_telemetry')
        # Topic for sending target pose to drone
        self.declare_parameter('drone_pose_topic', 'joisie_target_pose')
        # Topic for receiving grasp from VBM
        self.declare_parameter('vbm_grasp_topic', 'joisie_grasp_read')
        # Topic for sending grasp to arm
        self.declare_parameter('arm_grasp_topic', 'joisie_grasp_send')
        # CustomArmMsg from Arm Node
        self.declare_parameter('arm_status_topic', 'joisie_arm_status')
        # Topic for InRangeOfObj Service Call to Arm Node
        self.declare_parameter('arm_service_topic', 'joisie_arm_inrange_service')

        self.declare_parameter('state_setter_topic', 'joisie_set_state')

        self.state_setter_subscriber = self.create_subscription(String, 
                                            self.get_parameter('state_setter_topic').value, 
                                            self.receive_desired_state, 10)
        self.image_subscriber = self.create_subscription(Image, 
                                                    self.get_parameter('image_topic').value, 
                                                    self.receive_raw_image, 10)
        self.detection_subscriber = self.create_subscription(Detection2D, 
                                                    self.get_parameter('detection_topic').value, 
                                                    self.receive_detection, 10)
        # self.telemetry_subscriber = self.create_subscription(Telemetry, 
        #                                             self.get_parameter('telemetry_topic').value, 
        #                                             self.receive_detection, 10)
        self.grasp_subscriber = self.create_subscription(PoseStamped, 
                                                    self.get_parameter('vbm_grasp_topic').value, 
                                                    self.receive_grasp, 10)
        # self.arm_status_subscriber = self.create_subscription(DynaArmStatus, 
        #                                             self.get_parameter('arm_status_topic').value, 
        #                                             self.receive_grasp, 10)

        self.drone_publisher = self.create_publisher(PoseStamped, 
                                                    self.get_parameter('drone_pose_topic').value, 10)
        self.centroid_publisher = self.create_publisher(Point2D, 
                                                    self.get_parameter('centroid_topic').value, 10)
        self.grasp_publisher = self.create_publisher(PoseStamped, 
                                                    self.get_parameter('arm_grasp_topic').value, 10)

        self.in_range_service = self.create_client(Boolean, self.get_parameter('arm_grasp_topic').value)

        self.state = State.HOLD

    def receive_desired_state(self, ros_msg: String):
        bindings = {
            "HOLD": State.HOLD,
            "SEARCH": State.SEARCHING,
            "NAV": State.NAVIGATING,
            "GRASP": State.GRASPING,
            "DEPOSIT": State.DEPOSITING
        }
        try:
            string = ros_msg.data
            self.state = bindings[string.upper()]
        except:
            print("No matching state for string:", string)

    def receive_detection(self, ros_msg: Detection2D):
        self.detection = ros_msg

    def receive_raw_image(self, ros_msg: Image):
        self.raw_image = ros_msg

    # def receive_telemetry(self, ros_msg: Telemetry):
    #     self.telemetry = ros_msg

    def receive_grasp(self, ros_msg: PoseStamped):
        self.raw_grasp = ros_msg

    # def receive_arm_status(self, ros_msg: DynaArmStatus):
    #     self.arm_status = ros_msg



    # define vars



    def hold(self):  
        drone_hover()
        
        # listen for desiredState
        # if there is desiredState, state = desiredState
        if self.desired_state:
            self.State = desired_state
            self.desired_state = None
        pass

    def grasp(self):
        drone_hover()
        
        # calculate grasp
        # check livedetect information
        # if bounding box size is within 'pickup' range AND bounding box centroid is within 'pickup' range
            # execute grasp
        pass
    
    def search(self):
        # move drone to next position in search pattern
        # if new/different data from LiveDetect
            # state = navigate
        pass

    def navigate(self):
        # move drone to next position in navigation sequence
        # check livedetect information
        # if bounding box size is within 'pickup' range AND bounding box centroid is within 'pickup' range
            # state = grasp
        # if no object is detected for X out of Y LiveDetect messages
            # state = search
        pass

    def approachPoint(self, FLUpoint, range, height):
        # calculate where drone needs to be to be within X range of a point,
        approachPointFLU = 

        # convert FLU approach point to NED
        approachPointNED = FLU2NED(approachPointFLU)

        # move drone to that point
        send_drone_waypoint(approachPointNED)
        pass
    
    def deposit(self):
        # drops the object
        #  pass this for now
        pass

    def checkForErrors(self):
        # Read most recent Telemetry and ArmStatus data
        # Set mode to State.HOLD if any errors
        pass

    def process_detection(self):
        # TODO kay+mark fill in
        pass

    def process_grasp(self):
        # TODO kay+mark fill in
        # request grasp calculation
        pass

    def cam2FLU(self, point):
        # fill in using tf2 funcitionality
        # taylor (+ jakub?)
        # read the docs
        pass

    def FLU2NED(self, point):
        # fill in using tf2 funcitionality
        # taylor + jakub
        # read the docs
        pass

    def drone_hover(self):
        # pull drone current position via telemetry
        currentPos = self.telemetry.position

        # TODO convert coords here if needed

        # send waypoint
        send_drone_waypoint(currentPos)
        pass
    
    def send_drone_waypoint(self, point):
        # give drone NED coordinate to navigate to
        pass

    def main(self):
        while True:

            new_state = self.state
            
            if self.state == State.HOLD:
                new_state = self.hold()
            elif self.state == State.SEARCHING:
                # self.process_detection()
                new_state = self.search()
            elif self.state == State.NAVIGATING:
                # self.process_detection()
                # self.process_grasp()
                new_state = self.navigate()
            elif self.state == State.GRASPING:
                # self.process_detection()
                # self.process_grasp()
                new_state = self.grasp()
            elif self.state == State.DEPOSITING:
                new_state = self.deposit()
            
            if self.checkForErrors():
                new_state = State.HOLD

            self.state = new_state