# ROS2 imports 
import rclpy
from rclpy.node import Node

# CV Bridge and message imports
from sensor_msgs.msg import Image
from std_msgs.msg import String, Boolean
from vision_msgs.msg import Detection2D
from geometry_msgs.msg import Point2D, PoseWithCovariance, PoseStamped
import tf2_ros
import transformations

import cv2
import numpy as np
import os
import math

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

        self.lastSetpointNED = None #TODO replacce

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

    # -----


    def hold(self):  
        droneHover()
        
        # listen for desiredState
        # if there is desiredState, state = desiredState
        if self.desired_state:
            self.State = desired_state
            self.desired_state = None
        pass

    def grasp(self):
        droneHover()
        
        # calculate grasp
        # check livedetect information
        # if bounding box size is within 'pickup' range AND bounding box centroid is within 'pickup' range
            # execute grasp
        pass
    
    # -----
    def cam2FLU(self, point):
        # fill in using tf2 funcitionality
        # TODO - Mark
        # read the docs
        pass

    def sendWaypointNED(self, NEDpoint: list[float, float, float], heading=None):
        # give drone NED coordinate to navigate to
        
        # keep the current heading if not given
        if not heading:
            heading = self.telemetry.attitude
        else:
            pass
            heading = self.quaternion2head(heading)       
        
        # send waypoint by creating a PoseStamped message
                
        pass
    
    def search(self):
        # move drone to next position in search pattern
        # if new/different data from LiveDetect
            # state = navigate
        pass

    def navigate(self):
        # perform approach sequence
        CAM_2D = [self.detection.bbox.center.x, self.detection.bbox.center.y]
        FLU_pos = self.offsetPointFLU(self.cam2FLU(CAM_2D), [0, 0, 0])
        NED_pos = self.FLU2NED(FLU_pos, self.quaternion2head(self.telemetry.attitude))
        self.sendWaypointNED(NED_pos)

        # check livedetect information
        # if bounding box size is within 'pickup' range AND bounding box centroid is within 'pickup' range
            # state = grasp
        # if no object is detected for X out of Y LiveDetect messages
            # state = search
        pass
    
    def isInPosition(self, FLUpoint: list[float, float, float], tolerance: float) -> bool:
        """
        FLUpoint is goal position
        tolerance is acceptable difference between drone position and given position
        return TRUE if drone is in position
        """
        # does this even need to exist?
        # because if the drone is in a certain FLU position, that position will be [0,0,0]
        return

    def offsetPointFLU(self, FLUpoint: list[float, float, float], FLUoffset: list[float, float, float]) -> list[float, float, float]:
        """
        Approach the drone to a point in FLU coordinates
        FLU = [forward, left, up] in meters
        FLUoffset = to offset the point in the local FLU drone frame                
        """
        
        # calculate where drone needs to be to be within X range of a point,
        return [FLUpoint[0] + FLUoffset[0], FLUpoint[1] + FLUoffset[1], FLUpoint[2] + FLUoffset[2]]

    def FLU2NED(self, FLUoffsetPoint: list[float, float, float], yaw) -> list[float, float, float]:
        """
        yaw is Heading in degrees 0 to 360
        """
        # fill in using tf2 funcitionality
        # taylor + jakub
        # take in a local offset in meters and a yaw in degrees, convert to the NED frame, and return the NED coordinates
    
        """Convert a local FLU offset in meters to NED coordinates
        Returns the offset in NED, not the global NED"""
        # convert yaw to radians
        yaw_rad = yaw * (pi/180)   
        
        # convert the offset to rotated FLU
        rotated_flu_x = FLUoffsetPoint[0] * math.cos(yaw_rad) + FLUoffsetPoint[1] * math.sin(yaw_rad)
        rotated_flu_y = FLUoffsetPoint[0] * math.sin(yaw_rad) - FLUoffsetPoint[1] * math.cos(yaw_rad)
        rotated_flu_z = FLUoffsetPoint[2]
        
        # convert to NED coordinates
        offset_ned = [rotated_flu_x, rotated_flu_y, -rotated_flu_z]  
        return [self.lastSetpointNED[0] + offset_ned[0], self.lastSetpointNED[1] + offset_ned[1], self.lastSetpointNED[2] + offset_ned[2]]

    def droneHover(self):   
        sendWaypointNED(self.lastSetpointNED)

    def isInPosNED(self, NEDpoint: list[float, float, float], tolerance: float) -> bool:
        for i in range(len(NEDpoint)):
            if (NEDpoint(i) - tolerance <= lastSetpointNED(i)) and (lastSetpointNED(i) <= NEDpoint(i) + tolerance):
                continue
            else: return False
        
        return True

    def quaternion2head(self, quat):
        """Get the equivalent yaw-pitch-roll angles aka. intrinsic Tait-Bryan angles following the z-y'-x'' convention

        Returns:
            yaw:    rotation angle around the z-axis in radians, in the range `[-pi, pi]`
            pitch:  rotation angle around the y'-axis in radians, in the range `[-pi/2, pi/2]`
            roll:   rotation angle around the x''-axis in radians, in the range `[-pi, pi]` 

        The resulting rotation_matrix would be R = R_x(roll) R_y(pitch) R_z(yaw)

        Note: 
            This feature only makes sense when referring to a unit quaternion. Calling this method will implicitly normalise the Quaternion object to a unit quaternion if it is not already one.
        """
        
        qw = quat.w
        qx = quat.x
        qy = quat.y
        qz = quat.z

        yaw = atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
        pitch = asin(-2.0*(qx*qz - qw*qy))
        roll = atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)

        return yaw, pitch, roll
    
    def heading_to_px4_yaw(self, heading: float) -> float:
        """Convert a heading in degrees [0, 360) to a PX4 yaw in radians (-pi, pi]
        Heading is in degrees, yaw is in radians"""
        heading = heading % 360.0
        if heading > 180.0:
            heading = heading - 360.0
        return heading * (math.pi / 180.0) # heading is backwards 
    
    def px4_yaw_to_heading(self, yaw: float) -> float:
        """Convert a PX4 yaw in radians (-pi, pi] to a heading in degrees [0, 360)
        Yaw is in radians, heading is in degrees"""
        return (yaw * (180.0 / math.pi)) % 360.0

    # -----
    
    def deposit(self):
        # drops the object
        #  pass this for now
        pass

    def checkForErrors(self):
        # Read most recent Telemetry and ArmStatus data
        # Set mode to State.HOLD if any errors
        pass

    # -----

    def processDetection(self):
        # TODO kay+mark fill in
        pass

    def processGrasp(self):
        # TODO kay+mark fill in
        # request grasp calculation
        pass

    # -----




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