# ROS2 imports 
import rclpy
from rclpy.node import Node

# CV Bridge and message imports
from sensor_msgs.msg import Image
from std_msgs.msg import String, Boolean
from vision_msgs.msg import Detection2D
from geometry_msgs.msg import Pose2D, Point, PoseWithCovariance, PoseStamped
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
        self.centroid_publisher = self.create_publisher(Pose2D, 
                                                    self.get_parameter('centroid_topic').value, 10)
        self.grasp_publisher = self.create_publisher(PoseStamped, 
                                                    self.get_parameter('arm_grasp_topic').value, 10)

        self.in_range_service = self.create_client(Boolean, self.get_parameter('arm_grasp_topic').value)


        self.declare_parameter('debug', '0b00000')
        debug = self.get_parameter('debug').value
        self.debug_publish  = bool(debug & 0b10000)
        self.debug_drone    = bool(debug & 0b01000)
        self.debug_detect   = bool(debug & 0b00100)
        self.debug_vbm      = bool(debug & 0b00010)
        self.debug_arm      = bool(debug & 0b00001)

        # INITIAL STATE
        self._state = State.HOLD

        # STORE PREVIOUS MESSAGE SENT PER TOPIC
        self.lastSentMessages = {}
        self.received_new = {
            self.state_setter_subscriber.topic: False,
            self.image_subscriber.topic: False,
            self.detection_subscriber.topic: False,
            # self.telemetry_subscriber.topic: False,
            self.grasp_subscriber.topic: False,
            # self.arm_status_subscriber.topic: False,
        }

        self.lastSetpointNED = None #TODO replace

    #
    #### SET UP INCOMING MESSAGES AS PROPERTIES SO WE CAN KEEP TRACK OF WHAT
    #### NEW INFORMATION HAS / HASN'T BEEN ACCESSED
    #

    @property
    def state(self) -> State:
        self.received_new[self.state_setter_subscriber.topic] = False
        return self._state

    @state.setter
    def state_setter(self, value) -> State:
        self._state = value
    
    def receive_desired_state(self, ros_msg: String):
        self.received_new[self.state_setter_subscriber.topic] = True
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

    @property
    def detection(self) -> Detection2D:
        self.received_new[self.detection_subscriber.topic] = False
        return self._detection

    @detection.setter
    def receive_detection(self, ros_msg: Detection2D):
        self.received_new[self.detection_subscriber.topic] = True
        self._detection = ros_msg

    @property
    def raw_image(self) -> Image:
        self.received_new[self.image_subscriber.topic] = False
        return self._raw_image

    @raw_image.setter
    def receive_raw_image(self, ros_msg: Image):
        self.received_new[self.image_subscriber.topic] = True
        self._raw_image = ros_msg

    # @property
    # def telemetry(self) -> Telemetry:
    #     self.received_new[self.telemetry_subscriber.topic] = False
    #     return self._telemetry

    # @telemetry.setter
    # def receive_telemetry(self, ros_msg: Telemetry):
        # self.received_new[self.telemetry_subscriber.topic] = True
    #     self._telemetry = ros_msg

    @property
    def raw_grasp(self) -> PoseStamped:
        self.received_new[self.grasp_subscriber.topic] = False
        return self._raw_grasp

    @raw_grasp.setter
    def receive_grasp(self, ros_msg: PoseStamped):
        self.received_new[self.grasp_subscriber.topic] = True
        self._raw_grasp = ros_msg

    # @property
    # def arm_status(self) -> DynaArmStatus:
    #     self.received_new[self.arm_status_subscriber.topic] = False
    #     return self._arm_status
    
    # @arm_status.setter
    # def receive_arm_status(self, ros_msg: DynaArmStatus):
    #     self.received_new[self.arm_status_subscriber.topic] = True
    #     self._arm_status = ros_msg

    def publish_helper(self, publisher, message):
        # PUBLISHER HELPER SO WE DON"T PUBLISH DUPLICATE MESSAGES

        # ONLY PASS IF THE MESSAGE IS THE SAME 
        # ROS MESSAGES ARE SET UP TO BE EQUAL IF HEADERS+CONTENT ARE IDENTICAL
        if self.is_outgoing_new_msg(publisher, message):
            # DEBUG REPORTS ALL OUTGOING MESSAGES

            # Node name, topic, message, extra debug info (if present)
            self.debug(self.debug_publish, f"Topic - {publisher.topic}\tMessage - {message}")

            publisher.publish(message)
            self.lastSentMessages[publisher.topic] = message

    def is_outgoing_new_msg(self, publisher, message):
        # RETURNS TRUE IF THIS IS A NEW OUTGOING MESSAGE
        return not (publisher.topic in self.lastSentMessages 
                    and self.lastSentMessages[publisher.topic] == message)
    
    def is_new_data_from_subscriber(self, subscriber):
        # RETURNS TRUE IF THERE WAS A NEW MESSAGE RECEIVED ON THIS SUBSCRIBER
        return self.received_new[subscriber.topic]

    def debug(self, if_debug, string):
        if if_debug:
            # Node name, topic, message, extra debug info (if present)
            self.get_logger().info(string)

    # define vars

    # -----


    def hold(self):  
        self.droneHover()
        
        # listen for desiredState
        # if there is desiredState, state = desiredState
        if self.desired_state:
            self.State = self.desired_state
            self.desired_state = None
        pass

    def grasp(self):
        self.droneHover()
        
        # check livedetect information
        # if bounding box size is within 'pickup' range AND bounding box centroid is within 'pickup' range
            # execute grasp
        # calculate grasp
        pass
    
    # -----
    def cam2FLU(self, point):
        # fill in using tf2 funcitionality
        # taylor - get help (or at least a buddy)
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
        self.sendWaypointNED(self.lastSetpointNED)

    def isInPosNED(self, NEDpoint: list[float, float, float], tolerance: float) -> bool:
        for i in range(len(NEDpoint)):
            if (NEDpoint(i) - tolerance <= self.lastSetpointNED(i)) and (self.lastSetpointNED(i) <= NEDpoint(i) + tolerance):
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

        yaw = math.atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz)
        pitch = math.asin(-2.0*(qx*qz - qw*qy))
        roll = math.atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)

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
        if self.is_new_data_from_subscriber(self.detection_subscriber):
            # DETECTION CONFIDENCE - USEFUL FOR DEBUG
            probability = self.detection.results[0].score
            # FULL BOUNDING BOX OF DETECTED OBJECT
            bounding_box = self.detection.bbox

            self.debug(self.debug_detect, f"Detected object at {bounding_box.center} with probability {probability}")

            # PUBLISH MESSAGE USING HELPER FUNCTION (WITH EXTRA DEBUG TERM FOR DISPLAYING PROBABILITY)
            self.publish_helper(self.centroid_publisher, bounding_box.center)

            return bounding_box
        
        # RETURN FALSE IF NO NEW INFO FROM SUBSCRIBER
        return False

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