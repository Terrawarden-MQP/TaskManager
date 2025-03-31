# ROS2 imports 
import rclpy
from rclpy.node import Node

# CV Bridge and message imports
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Header
from vision_msgs.msg import Detection2D
from terrawarden_interfaces.msg import DroneTelemetry, DroneWaypoint, ArmStatus
from geometry_msgs.msg import Pose2D, Point, PointStamped, PoseWithCovariance, PoseStamped
from builtin_interfaces.msg import Time
import tf2_ros
from std_srvs.srv import Empty


# import transformations
import cv2
import numpy as np
import os
from collections.abc import Callable
from collections import deque
import math
from enum import Enum, auto
import time

class State(Enum):
    STARTUP = "STARTUP"
    FAILSAFE = "FAILSAFE"
    LANDING = "LAND"

    WAITING = "WAIT"
    HOLD = "HOLD"

    SEARCHING = "SEARCH"
    NAVIGATING = "NAV"
    GRASPING = "GRASP"
    DEPOSITING = "DEPOSIT"
    
# launch using the: ros2 launch joisie_manager all_nodes manager_debug:=0b11000
# manually set state: ros2 topic pub -1 /joisie_set_state std_msgs/msg/String "{data: 'SEARCH'}"

class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('task_manager')
        
        # drone flight parameters dictionary
        self.droneParams = {
            "fastMaxLinVel_m_s": 3.0,
            "fastMaxAngVel_deg_s": 45.0,
            "fastMaxLinAccel_m_s2": 0.5,
            "fastMaxZVel_m_s": 1.2,
            "slowMaxLinVel_m_s": 0.5,
            "slowMaxAngVel_deg_s": 30.0,
            "slowMaxLinAccel_m_s2": 0.25,
            "slowMaxZVel_m_s": 0.5,
            "precisionMaxLinVel_m_s": 0.3,
            "precisionMaxAngVel_deg_s": 20.0,
            "precisionMaxLinAccel_m_s2": 0.3,
            "precisionMaxZVel_m_s": 0.3,
        }

        ### TOPIC DECLARATION - ALL PARAMETERIZED THROUGH ROS2 LAUNCH

        ### DRONE TOPICS ------------------------------------------------------
        # param_name_topic_tuples = [
        #     ## Topic for sending target pose to drone
        #     ('drone_pose_topic', 'drone/waypoint'),
        #     ## Topic for receiving Telemetry from Drone Node
        #     ('drone_telemetry_topic', 'drone/telemetry')
        # ]
        self.declareParameter('drone_pose_topic', 'drone/waypoint')
        self.declareParameter('drone_telemetry_topic', 'drone/telemetry')

        # self.declareParameter('telemetry', 'joisie_telemetry')

        # Topic for sending target pose to drone
        # self.declareParameter('drone_pose_topic', 'joisie_target_pose')

        ### DETECTION TOPICS --------------------------------------------------
        
        # Topic for receiving Detection from LiveDetect Node
        self.declareParameter('detection_topic', 'joisie_detection')

        # Topic for sending detected object centroid to VBM
        self.declareParameter('centroid_topic', 'joisie_detected_centroid')

        # Topic for receiving 3D point from VBM extract_cluster
        self.declareParameter('vbm_extract_topic', 'joisie_extract_centroid')

        # Topic for receiving grasp from VBM optimal_grasp
        self.declareParameter('vbm_grasp_topic', 'joisie_grasp_read')

        ### ARM TOPICS --------------------------------------------------------

        # Topic for sending grasp to arm
        self.declareParameter('arm_grasp_topic', 'joisie_grasp_send')

        # Topic for sending grasp to arm WITH TRAJECTORY
        self.declareParameter('traj_arm_grasp_topic', 'joisie_grasp_send')

        # CustomArmMsg from Arm Node
        self.declareParameter('arm_status_topic', 'joisie_arm_status')

        # Topic for InRangeOfObj Service Call to Arm Node
        self.declareParameter('arm_service_topic', 'joisie_arm_inrange_service')

        # Topic for Stow Service Call to Arm Node
        self.declareParameter('arm_stow_service_topic', 'stow_arm')

        # Topic for Unstow Service Call to Arm Node
        self.declareParameter('arm_unstow_service_topic', 'unstow_arm')

        ### STATE TOPICS --------------------------------------------------
        
        # Topic to send state information
        self.declareParameter('state_topic','joisie_state')

        self.declareParameter('state_setter_topic', 'joisie_set_state')
        
        # -----
        # SERVICES

        # self.in_range_service = self.create_client(Bool, self.get_parameter('arm_grasp_topic').value) # TODO fix or delete

        # -----
        # DEBUG
        self.declareParameter('overrideErrors',False)
        self.declareParameter('debug', 0b11111)
        self.overrideErrors = self.get_parameter('overrideErrors').value
        debug = self.get_parameter('debug').value
        self.debugPublish  = bool(debug & 0b10000)
        self.debugDrone    = bool(debug & 0b01000)
        self.debugDetect   = bool(debug & 0b00100)
        self.debugVBM      = bool(debug & 0b00010)
        self.debugArm      = bool(debug & 0b00001)
        self.get_logger().info(f"Debug Flags: Publish: {self.debugPublish}, Drone {self.debugDrone},"+
                               f" Detection {self.debugDetect}, VBM {self.debugVBM}, Arm {self.debugArm}")
        # INITIAL STATE
        self._state = State.STARTUP
        self._recievedState = State.HOLD
        self._telemetry = DroneTelemetry()
        self.telemetry_queue = deque()
        self._detection = Detection2D()
        self._extract_pt = PointStamped()
        self._raw_grasp = PoseStamped()
        # self._arm_status = DynaArmStatus()    

        # STORE PREVIOUS MESSAGE SENT PER TOPIC
        self.lastSentMessage = {}
        self.msgTimeout = 3
        
        # failsafe variables
        self.hasFailsafed = False
        
        # hold state variables
        self.holdNEDPoint = Point()
        self.holdHeading = 0.0

        # search state tracking
        # TOOD: add zig-zag GPS pattern
            # for now it could use the hold state pos tracking, but I will keep it like this for the future
        self.searchStartTime = None
        self.searchStartHeading = None
        self.entryPoint = None
        self.borderPoints = []

        # waiting, stowing, unstowing state variables
        self.nextState = State.HOLD
        self._wait_time = 0
        self._wait_start_time = 0

        # There's no need for an on_shutdown method here
        # Drone Node listens for a heartbeat from Task Manager, 
        # and will failsafe if no message is received for enough time
        # rclpy.get_default_context().on_shutdown(self.on_shutdown)

    # SUBSCRIBERS

        self.stateSetterSubscriber = self.create_subscription(String, 
                                            self.get_parameter('state_setter_topic').value, 
                                            self.recieveDesiredState, 10)
        
        self.detectionSubscriber = self.create_subscription(Detection2D, 
                                                    self.get_parameter('detection_topic').value, 
                                                    self.getSetter("detection"), 10)
        
        self.telemetrySubscriber = self.create_subscription(DroneTelemetry, 
                                                    self.get_parameter('drone_telemetry_topic').value, 
                                                    self.getSetter("telemetry"), 10)
        
        self.extractSubscriber = self.create_subscription(PointStamped,
                                                    self.get_parameter('vbm_extract_topic').value,
                                                    self.getSetter("extract_pt"), 10)
        
        self.graspSubscriber = self.create_subscription(PoseStamped,
                                                    self.get_parameter('vbm_grasp_topic').value, 
                                                    self.getSetter("raw_grasp"), 10)

        self.armStatusSubscriber = self.create_subscription(ArmStatus, 
                                                    self.get_parameter('arm_status_topic').value, 
                                                    self.getSetter("arm_status"), 10)

    ### STOW ARM SERVICE CLIENT

        self.stowArmClient = self.create_client(Empty, self.get_parameter('arm_stow_service_topic').value)
        self.unstowArmClient = self.create_client(Empty, self.get_parameter('arm_unstow_service_topic').value)

    # PUBLISHERS

        self.dronePublisher = self.create_publisher(DroneWaypoint, 
                                                    self.get_parameter('drone_pose_topic').value, 10)
        # self.centroidPublisher = self.create_publisher(Pose2D, # this publisher is redundant and not used
        #                                             self.get_parameter('centroid_topic').value, 10)
        self.graspPublisher = self.create_publisher(PoseStamped, 
                                                    self.get_parameter('arm_grasp_topic').value, 10)
        self.trajGraspPublisher = self.create_publisher(PoseStamped, 
                                                    self.get_parameter('traj_arm_grasp_topic').value, 10)
        self.statePublisher = self.create_publisher(String,
                                                    self.get_parameter('state_topic').value, 10)
                
        # new needs to exist for the properties but needs the subscribers to exist as well
        self.received_new = {
            self.stateSetterSubscriber.topic: False,
            # self.image_subscriber.topic: False,
            self.detectionSubscriber.topic: False,
            self.telemetrySubscriber.topic: False,
            self.extractSubscriber.topic: False,
            self.graspSubscriber.topic: False,
            self.armStatusSubscriber.topic: False,
        }  

        # initializes the drone state switching loop timer at the bottom of the file
        self.initLoop()


# ----- PROPERTIES
    
    #### SET UP INCOMING MESSAGES AS PROPERTIES SO WE CAN KEEP TRACK OF WHAT
    #### NEW INFORMATION HAS / HASN'T BEEN ACCESSED
    
    # When python initializes properties, it hides the setters so that they get called when you do "self.property ="
    # This function retrieves the setter manually for any of the properties for the purposes of subscriber callbacks
    def getSetter(self, property):
        return lambda value: getattr(TaskManagerNode, property).fset(self, value)

    @property
    def state(self) -> State:
        return self._state

    @state.setter
    def state(self, value) -> State:
        if value not in State:
            raise ValueError(f"Invalid state: {value}")
        self._state = value

    
    @property
    def recievedState(self) -> State:
        self.received_new[self.stateSetterSubscriber.topic] = False
        return self._recievedState

    @recievedState.setter
    def recievedState(self, value) -> State:
        if value not in State:
            raise ValueError(f"Invalid state: {value}")
        self._recievedState = value
    
    def recieveDesiredState(self, ros_msg: String) -> None:
        self.received_new[self.stateSetterSubscriber.topic] = True
        try:
            string = ros_msg.data
            self.recievedState = State(string)
            self.debug(True, f"State changed queued from incoming message: {string, self._recievedState}")
        except ValueError:
            self.debug(True, f"[WARNING] No matching state for string: {string}")

    @property
    def detection(self) -> Detection2D:
        self.received_new[self.detectionSubscriber.topic] = False
        return self._detection

    @detection.setter
    def detection(self, ros_msg: Detection2D) -> None:
        self.received_new[self.detectionSubscriber.topic] = True
        self._detection = ros_msg

    @property
    def telemetry(self) -> DroneTelemetry:
        self.received_new[self.telemetrySubscriber.topic] = False
        return self._telemetry

    @telemetry.setter
    def telemetry(self, ros_msg: DroneTelemetry) -> None:
        self.received_new[self.telemetrySubscriber.topic] = True
        self._telemetry = ros_msg

        #                               Timestamp              Message
        self.telemetry_queue.append((ros_msg.pos.header.stamp, ros_msg))

        compare_times = lambda t1, t2: abs(t1.sec + t1.nanosec/1e9 - t2.sec - t2.nanosec/1e9)
        TELEMETRY_QUEUE_TIME = 5 # SECONDS
        
        while True:
            msg_time = self.telemetry_queue[-1][0]
            # Compares old message time with newest message time
            if compare_times(ros_msg.pos.header.stamp, msg_time) > TELEMETRY_QUEUE_TIME:
                # Remove old message if it is more than TELEMETRY_QUEUE_TIME seconds older than the newest message
                self.telemetry_queue.pop()
            else:
                break

    @property
    def raw_grasp(self) -> PoseStamped:
        self.received_new[self.graspSubscriber.topic] = False
        return self._raw_grasp

    @raw_grasp.setter
    def raw_grasp(self, ros_msg: PoseStamped) -> None:
        self.received_new[self.graspSubscriber.topic] = True
        self._raw_grasp = ros_msg


    @property
    def extract_pt(self) -> PointStamped:
        self.received_new[self.extractSubscriber.topic] = False
        return self._extract_pt
    
    @extract_pt.setter
    def extract_pt(self, ros_msg: PointStamped) -> None:
        self.received_new[self.extractSubscriber.topic] = True
        self._extract_pt = ros_msg


    @property
    def wait_time(self) -> float:
        return self._wait_time
    
    @wait_time.setter
    def wait_time(self, val: float) -> None:
        self._wait_start_time = time.time()
        self._wait_time = val

    @property
    def arm_status(self) -> ArmStatus:
        self.received_new[self.armStatusSubscriber.topic] = False
        return self._arm_status
    
    @arm_status.setter
    def arm_status(self, ros_msg: ArmStatus):
        self.received_new[self.armStatusSubscriber.topic] = True
        self._arm_status = ros_msg

# ----- HELPER FNs

    def publish_helper(self, publisher, message) -> None:
        """EXISTS SO WE DON'T PUBLISH DUPLICATE MESSAGES
        ONLY PASS IF THE MESSAGE IS THE SAME 
        ROS MESSAGES ARE SET UP TO BE EQUAL IF HEADERS+CONTENT ARE IDENTICAL"""

        if self.is_outgoing_new_msg(publisher, message):
            # DEBUG REPORTS ALL OUTGOING MESSAGES
            # Node name, topic, message, extra debug info (if present)
            self.debug(self.debugPublish, f"Topic - {publisher.topic}\tMessage - {message}")

            publisher.publish(message)
            self.lastSentMessage[publisher.topic] = {"msg": message, "time": time.time()}

    def is_outgoing_new_msg(self, publisher, message) -> bool:
        '''RETURNS TRUE IF THIS IS A NEW OUTGOING MESSAGE'''

        return not (publisher.topic in self.lastSentMessage 
                    and self.lastSentMessage[publisher.topic]["msg"] == message
                    # Additionally, send message if previous message is more than {self.msgTimeout} seconds old
                    and self.lastSentMessage[publisher.topic]["time"] > time.time() - self.msgTimeout)
    
    def is_new_data_from_subscriber(self, subscriber):
        '''RETURNS TRUE IF THERE WAS A NEW MESSAGE RECEIVED ON THIS SUBSCRIBER'''
        return self.received_new[subscriber.topic]

    def get_last_sent_message(self, publisher):
        '''
        returns last sent message under publisher topic
        if there is no last sent message w/ that topic, returns None
        '''

        if not publisher.topic in self.lastSentMessage:
            return None
            
        return self.lastSentMessage[publisher.topic]["msg"]

    def debug(self, if_debug:bool, string:String) -> None:
        '''
        if_debug is a boolean - if true, string gets printed to ros logger
        '''
        if if_debug:
            # Node name, topic, message, extra debug info (if present)
            self.get_logger().info(string)

    def get_telemetry_closest_to_time(self, msg_time: Time):
        '''
            Finds the telemetry message sent closest to the parameter msg_time
            TaskManager stores the last 10 seconds of Telemetry messages in self.telemetry_queue
        '''

        if len(self.telemetry_queue) == 0:
            return self.telemetry

        # Compute time difference in seconds
        compare_times = lambda t1, t2: abs(t1.sec + t1.nanosec/1e9 - t2.sec - t2.nanosec/1e9)

        # default is the most recent message                       index \/
        closest = (compare_times(self.telemetry_queue[-1][0], msg_time), -1)

        for i in range(len(self.telemetry_queue)):

            telemetry_time = self.telemetry_queue[i][0]
            # if message is closer in time than previous closest
            if compare_times(telemetry_time,msg_time) < closest[0]:
                # store it and it's corresponding index instead
                closest = (compare_times(telemetry_time, msg_time), i)

        # ONLY INCLUDE THE MESSAGE CONTENT (queue stores [timestamp, message])
        return self.telemetry_queue[closest[1]][1]
            
    def saveDroneHoldPose(self):
        self.holdNEDPoint = self.telemetry.pos.pose.position
        self.holdHeading = self.telemetry.heading_degrees
        
    def retrieveDroneHoldPose(self):
        return self.holdNEDPoint, self.holdHeading


# ----- STOW FNs
    def stow_arm(self):
        # Call the stow_arm service
        self.get_logger().info('Calling stow_arm service...')
        request = Empty.Request()
        future = self.stowArmClient.call_async(request)
        # Internal function for service debug messages
        def service_debug_message(response):
            try:
                response.result()
                return "Stow Service Success"
            except:
                return "Stow Service Failure"

        future.add_done_callback(lambda response: self.debug(self.debugArm, service_debug_message(response)))
    
    def unstow_arm(self):
        # Call the unstow_arm service
        self.get_logger().info('Calling unstow_arm service...')
        request = Empty.Request()
        future = self.unstowArmClient.call_async(request)
        # Internal function for service debug messages
        def service_debug_message(response):
            try:
                response.result()
                return "Unstow Service Success"
            except:
                return "Unstow Service Failure"

        future.add_done_callback(lambda response: self.debug(self.debugArm, service_debug_message(response)))
    

# ----- DRONE HELPERS

    def sendWaypointNED(self, NEDpoint: Point, heading:float=None, max_ang_vel_deg_s:float=None, max_lin_vel_m_s:float=None, max_z_vel_m_s:float=None, max_lin_accel_m_s2:float=None):
        """
        NEDpoint is the waypoint to send to the drone in [N,E,D] format
        heading is OPTIONAL, does not change if left out
        max_ang_vel_deg_s, max_lin_vel_m_s, max_z_vel_m_s, and max_lin_accel_m_s2 if not specified are left at precision (slow)
        """ 
        
        # send waypoint by creating a PoseStamped message
        waypoint_msg = DroneWaypoint()

        # Set the pose data 
        waypoint_msg.ned_pos = NEDpoint
        
        if heading:
            waypoint_msg.heading_degrees = heading
        else:
            waypoint_msg.heading_degrees = self.telemetry.heading_degrees

        # Set the velocity and acceleration data
        if max_ang_vel_deg_s: waypoint_msg.max_ang_vel_deg_s = max_ang_vel_deg_s
        else: waypoint_msg.max_ang_vel_deg_s = self.droneParams["precision_max_ang_vel_deg_s"]
        
        if max_lin_vel_m_s: waypoint_msg.max_lin_vel_m_s = max_lin_vel_m_s
        else: waypoint_msg.max_lin_vel_m_s = self.droneParams["precision_max_lin_vel_m_s"]
            
        if max_z_vel_m_s: waypoint_msg.max_z_vel_m_s = max_z_vel_m_s
        else: waypoint_msg.max_z_vel_m_s = self.droneParams["precision_max_z_vel_m_s"]
            
        if max_lin_accel_m_s2: waypoint_msg.max_lin_accel_m_s2 = max_lin_accel_m_s2
        else: waypoint_msg.max_lin_accel_m_s2 = self.droneParams["precision_max_lin_accel_m_s2"]     
    
        self.publish_helper(self.dronePublisher, waypoint_msg)  
    
    
    def offsetPointFLU(self, FLUpoint: Point, FLUoffset: Point) -> Point:
        """
        Approach the drone to a point in FLU coordinates
        
        FLU = [forward, left, up] in METERS
        FLUoffset = to offset the point in the local FLU drone frame in F, L, U
        returns a Point              
        """
        
        return Point(x=FLUpoint.x + FLUoffset.x, y=FLUpoint.y + FLUoffset.y, z=FLUpoint.z + FLUoffset.z)

    def FLU2NED(self, FLUoffsetPoint: Point, heading_deg: float = None) -> Point:
        """
        Convert a local FLU offset in meters to NED coordinates
        yaw is Heading in DEGREES 0 to 360
        Returns the offset as a Point in NED
        """

        # if no heading is given, keep current heading
        if heading_deg is None:
            heading_deg = self.telemetry.heading_degrees

        # convert yaw to radians
        yaw_rad = heading_deg * (math.pi/180)   
        
        # convert the offset to rotated FLU
        rotated_flu_x = FLUoffsetPoint.x * math.cos(yaw_rad) + FLUoffsetPoint.y * math.sin(yaw_rad)
        rotated_flu_y = FLUoffsetPoint.x * math.sin(yaw_rad) - FLUoffsetPoint.y * math.cos(yaw_rad)
        rotated_flu_z = FLUoffsetPoint.z
        
        # convert to NED coordinates
        offset_ned = Point(x=rotated_flu_x, y=rotated_flu_y, z=-rotated_flu_z)
        
        known_position = self.telemetry.pos.pose.position

        return Point(x=known_position.x + offset_ned.x, 
                    y=known_position.y + offset_ned.y, 
                    z=known_position.z + offset_ned.z)
        
    def FLU2NED_quaternion(self, FLUoffsetPoint: Point, timestamp: Time = None) -> Point:
        """
        Convert a local FLU offset in meters to global NED coordinates
        Accepts a ROS2 timestamp, if specified grabs the quaternion at that point in time to counteract the camera processing delay
        Apply the current drone NED rotation quaternion to figure out the new point XYZ in the NED frame
        """

        # https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
        def multiply_quaternion(q1: np.array, q2: np.array) -> np.array:
            r0, r1, r2, r3 = q1
            s0, s1, s2, s3 = q2
            
            return np.array([
                r0*s0 - r1*s1 - r2*s2 - r3*s3,
                r0*s1 + r1*s0 - r2*s3 + r3*s2,
                r0*s2 + r1*s3 + r2*s0 - r3*s1,
                r0*s3 - r1*s2 + r2*s1 + r3*s0
            ])

        # do it in numpy, convert FLU to FRD
        point_np = np.array([0, FLUoffsetPoint.x, -FLUoffsetPoint.y, -FLUoffsetPoint.z])
        
        # Get the current orientation quaternion
        # Quaternion rotation from FRD body frame to reference frame
        # use the last position from telemetry and updage it with what the camera sees
        if timestamp is None:
            quat = self.telemetry.pos.pose.orientation
            last_known_position = self.telemetry.pos.pose.position
        else:
            # get the closest telemetry message to the timestamp
            telemetry = self.get_telemetry_closest_to_time(timestamp)
            quat = telemetry.pos.pose.orientation
            last_known_position = telemetry.pos.pose.position
            
        quat_np = np.array([quat.w, quat.x,  quat.y, quat.z])
        quat_np_conjugate = np.array([quat.w, -quat.x, -quat.y, -quat.z])
        
        # quaternion multiplication is commutative - perform the passive rotation
        rotated_point = multiply_quaternion(multiply_quaternion(quat_np, point_np), quat_np_conjugate)
        
        # extract the rotated point
        NED_offset_point = Point(x=rotated_point[1], y=rotated_point[2], z=rotated_point[3])
        
        return Point(x=last_known_position.x + NED_offset_point.x, 
                    y=last_known_position.y + NED_offset_point.y, 
                    z=last_known_position.z + NED_offset_point.z)
         

    def droneHover(self) -> None:   
        """
        Sends last waypoint to drone as new waypoint, essentially tells it to stay where it is.    
        """
        
        last_setpoint, last_heading = self.retrieveDroneHoldPose()

        # sends message
        self.sendWaypointNED(last_setpoint, last_heading)
        self.debug(self.debugDrone, f'drone hovering @ {last_setpoint} heading {last_heading}')
        

    def isInRangeNED(self, NEDpoint: Point, toleranceXY: float, toleranceZ: float) -> bool:
        """
        Returns true if drone's location is at given NEDpoint plus or minus given tolerance
        Otherwise returns false
        """

        last_received = self.telemetry.pos.pose

        # if there is no waypoint, false
        if last_received is None:
            return False
        
        self.debug(self.debugVBM,f'NED point from VBM: ({NEDpoint.x},{NEDpoint.y},{NEDpoint.z})')
        self.debug(self.debugDrone, f'diffX, diffY, diffZ ({abs(last_received.position.x - NEDpoint.x)}, {abs(last_received.position.y - NEDpoint.y)}, {abs(last_received.position.z - NEDpoint.z)})')
        
        # if difference between current point and set point is greater than tolerance, false
        if abs(last_received.position.x - NEDpoint.x) > toleranceXY: return False
        if abs(last_received.position.y - NEDpoint.y) > toleranceXY: return False
        if abs(last_received.position.z - NEDpoint.z) > toleranceZ: return False
        
        return True

    def quaternion2head(self, quat):
        """
        Get the equivalent yaw-pitch-roll angles aka. intrinsic Tait-Bryan angles following the z-y'-x'' convention

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
        """
        Convert a heading in degrees [0, 360) to a PX4 yaw in radians (-pi, pi]
        Heading is in degrees, yaw is in radians
        """
        heading = heading % 360.0
        
        if heading > 180.0:
            heading = heading - 360.0
            
        return heading * (math.pi / 180.0) # heading is backwards 
    
    def px4_yaw_to_heading(self, yaw: float) -> float:
        """
        Convert a PX4 yaw in radians (-pi, pi] to a heading in degrees [0, 360)
        Yaw is in radians, heading is in degrees
        """
        return (yaw * (180.0 / math.pi)) % 360.0

# ----- ARM HELPERS

    def openGripper(self):
        '''send ROSmsg to arm control node to open gripper'''
        self.publish_helper(self.gripper_publisher, True)

    
    def closeGripper(self):
        '''send ROSmsg to arm control node to close gripper'''
        self.publish_helper(self.gripper_publisher, False)


    def sendArmToPoint(self, poseStampedMsg, trajectory:bool=True):
        '''send ROSmsg to arm control node with a point'''

        # send posestamped to different topics depending wether trajectory is true or false
        if trajectory:
            self.publish_helper(self.trajGraspPublisher, poseStampedMsg)
        else:
            self.publish_helper(self.graspPublisher, poseStampedMsg)

    def set_wait(self, nextState: State, wait_time_s: float = 0.5, wait_until_fn: Callable[[None], bool] = None) -> State:
        '''
        nextState is the desired state after wait time
        wait_time_s is time IN SECONDS
        wait_time_fn is a function()->bool. Overrides wait_time if not included in function
        does not command the drone in any way, shape, or form
        '''
        # Default is time-based
        def default_wait_fn():
            return time.time() > self._wait_start_time + self.wait_time
        if wait_until_fn is None:
            wait_until_fn = default_wait_fn

        self.wait_time = wait_time_s
        self.nextState = nextState
        self.wait_until_fn = wait_until_fn

        return State.WAITING
    
# ----- STATE FUNCTIONS (not helpers!)

    def wait(self) -> State:
        '''
        this loops when wait is current state
        no inputs, outputs State
        does not command the drone in any way, shape, or form
        '''
        if self.wait_until_fn():
            return self.nextState
        return State.WAITING
    
# ----- STARTUP

    def startup(self) -> State:
        '''
        this loops when startup is current state
        checks drone telemetry and switches to HOLD if drone is offboard and armed
        '''
        
        # check if drone is flying and offboard
        if self.is_new_data_from_subscriber(self.telemetrySubscriber):
            if self.telemetry.is_flying == True and self.telemetry.is_offboard == True:
                self.debug(self.debugDrone, "Drone is offboard and armed, switching to HOLD")
                return State.HOLD                
        
        return State.STARTUP
    
# ----- FAILSAFE

    def failsafe(self) -> State:
        '''
        upon entering this state keep slowly returning to home armed position and land
        used only in case the RC link has been lost to prevent runaway of the system
        '''

        # when it is within a meter of the home position in XY only
        if self.isInRangeNED(Point(x=0, y=0, z=-10), 2.0, 10000.0):
            self.debug(self.debugDrone, "Drone failsafe is in range of home position, switching to LANDING")    
            
            # send the setpoint to go down the current above ground altitude (+ 2 extra meters for good measure)
            # the PX4 autopilot should detect the landing, and disarm the drone automatically
            current_NED_pos = self.telemetry.pos.pose.position
            # e.g.: the altitude above ground is 10, so the drone should descend down (10+2) meters to land on the ground    
            point_to_land = self.FLU2NED(Point(x=0, y=0, z=-(self.telemetry.altitude_above_ground_m + 2)))
            self.sendWaypointNED(point_to_land, 0)
            return State.LANDING
        
        return State.FAILSAFE
    
# ----- LANDING

    def land(self) -> State:
        '''
        land the drone by issuing a PX4 command to land
        this is a failsafe state, so it will keep trying to land until it is on the ground
        '''
        
        # self.sendWaypointNED(Point(x=0, y=0, z=0), 0, self.droneParams["precision_max_ang_vel_deg_s"], self.droneParams["precision_max_lin_vel_m_s"], self.droneParams["precision_max_z_vel_m_s"], self.droneParams["precision_max_lin_accel_m_s2"])                   
        
        return State.LANDING
    
    def hold(self) -> State:  
        '''
        this loops when hold is current state
        no inputs, outputs State
        '''

        self.droneHover()
        # listening for desired state happens async from this'

    
        # if not self.arm_status.is_stowed:
        #     self.stow_arm()
        
        # the only time it can go back to startup is, if it is in hold and the offboard mode is deactivated
        if self.telemetry.is_offboard == False:
            self.debug(self.debugDrone, "Drone is not in offboard, switching to STARTUP")            
            return State.STARTUP

        return State.HOLD

# ----- GRASP

    def grasp(self) -> State:
        '''
        this loops when grasp is current state
        no inputs, outputs State
        '''

        self.droneHover()
        
        # calculate grasp
        # generate posestamped message from grasp
        # TODO if refresh rate gets to live rate, use this instead
        # if self.is_new_data_from_subscriber(self.graspSubscriber):
        #     # self.raw_grasp
        #     self.sendArmToPoint(self.raw_grasp)

        
        if self.is_new_data_from_subscriber(self.extractSubscriber):
            pt = PoseStamped()
            pt.header = self.extract_pt.header
            pt.pose.position = self.extract_pt.point
            self.sendArmToPoint(pt)
        
        return State.GRASPING # TODO - what state makes sense to move into?

# ----- SEARCH

    def search(self) -> State:
        """
        Move drone to next position in search pattern
        No inputs, no outputs
        """
        # TODO - make this a zig-zags GPS assisted search patters for outside
                
        # just slowly spin for now, slowly
        rpm = 2.0    
        degrees_per_second = rpm * 360.0 / 60.0
        
        self.debug(self.debugDrone, f'spinning, RPM: {rpm}') 

        # desired heading based on elapsed time
        current_time = self.get_clock().now()
        elapsed = (current_time - self.searchStartTime).nanoseconds / 1e9
        desired_heading = (self.searchStartHeading + elapsed * degrees_per_second) % 360.0

        # send waypoint maintaining position but rotating
        self.sendWaypointNED(
            self.retrieveDroneHoldPose()[0],  # NED position
            heading=desired_heading,
            max_ang_vel_deg_s=degrees_per_second * 1.2  # 20% for smoothness
        )

        # detection found, transition states
        if self.is_new_data_from_subscriber(self.extractSubscriber):

            self.debug(self.debugDrone, f'found object at ({self.extract_pt.point}), changing to HOLD') 

            return self.set_wait(State.NAVIGATING, 5) # Move to grasp after 5 seconds 
        
        self.debug(self.debugDrone, f'nothing found, continue SEARCHING') 

        return State.SEARCHING

# ----- NAVIGATE

    def navigate(self) -> State:
        """ 
        Perform approach sequence
        No inputs, output State
        """
        
        #TODO: if object is lost for 10s + and you hover at position, go back to search
        
        # if new extracted pt, recalculate approach
        if self.is_new_data_from_subscriber(self.extractSubscriber):
            
            self.debug(self.debugVBM, f'new point extracted, recalculating approach') 

            # convert that 3D point to NED, offset it above and towards the drone a bit
            
            FLU_pos = self.offsetPointFLU(self.extract_pt.point, Point(x=-0.707, y=0., z=0.707))
            NED_pos = self.FLU2NED_quaternion(FLU_pos, self.extract_pt.header.stamp) 
                    
            # calculate heading needed to turn towards point
            diff_north = NED_pos.x - self.telemetry.pos.pose.position.x
            diff_east = NED_pos.y - self.telemetry.pos.pose.position.y
            px4_yaw_rad = math.atan2(diff_east, diff_north) 
            heading_deg = self.px4_yaw_to_heading(px4_yaw_rad)            
            
            self.sendWaypointNED(NED_pos, heading_deg, self.droneParams["precision_max_ang_vel_deg_s"], self.droneParams["precision_max_lin_vel_m_s"], self.droneParams["precision_max_z_vel_m_s"], self.droneParams["precision_max_lin_accel_m_s2"])    

            # if the new waypoint is within a certain distance of the robot, switch to grasping state
            if self.isInRangeNED(NED_pos, 0.1, 0.1):  #TODO: ROS-tunable params
                self.debug(self.debugVBM, f'within range of object, begin GRASPING') 
                # return State.GRASPING

                # if self.arm_status.is_stowed:
                #     self.unstow_arm()

                def check_arm_position_unstowed():
                    return self.arm_status.at_setpoint and not self.arm_status.is_stowed

                # return self.set_wait(State.GRASPING, check_arm_position_unstowed) # Move to grasp state when unstowed
                return self.set_wait(State.GRASPING, 5) # Move to grasp after 5 seconds

        # stay in navigation state
        self.debug(self.debugDrone, f'out of range, continue NAVIGATING') 

        return State.NAVIGATING
    
    def deposit(self) -> State:
        '''
        deposit state function
        no inputs, returns a State
        '''
        # drops the object
        #  pass this for now
        return State.DEPOSITING # TODO
    
# --- ERROR CHECKING AND FAILSAFES

    def checkForErrors(self) -> bool:
        '''
        checks for ground clearence beneath the drone and arm errors messages
        prints errors + wardnings to ros logger
        
        returns True if there is error
        '''
        if self.overrideErrors:
            return False
        # Read most recent Telemetry and ArmStatus data
        # Set mode to State.HOLD if any errors
        if self.is_new_data_from_subscriber(self.telemetrySubscriber):
            
            # check that we have more than 0.4m of ground clearance beneath the drone
            if self.telemetry.altitude_above_ground < 0.4:
                # check that we are not too low
                self.debug(self.debugDrone, "Ground Too Close, Abort")
                return True          
            
            if self.telemetry.altitude_above_ground < 0.8:
                # check that we are not too low
                self.debug(self.debugDrone, "Ground Proximity Warning")
                return False     
            
            # check that offboard mode has been deactivated
            if self.telemetry.is_offboard == False:            
                self.debug(self.debugDrone, "Not in Offboard Mode")
                return True   
                
        return False # TODO
    
    def checkFailSafe(self) -> bool:
        '''
        checks for failsafe condition of RC link lost    
        returns True if there is error
        
        the RC control can always switch off offboard mode and take over manually
        '''
        # If we're ignoring errors, ignore failsafe as well
        if self.overrideErrors:
            return False
        
        if self.is_new_data_from_subscriber(self.telemetrySubscriber):
            # check that we have a good RC link
            if self.telemetry.has_rc_link == False:
                # check that we are not too low
                self.debug(self.debugDrone, "No RC Link")
                return True        
            
            # if battery is below 10%
            if self.telemetry.battery_percentage < 10:
                # check that we are not too low
                self.debug(self.debugDrone, "Battery Low")
                return True
        return False

# --- STATE MACHINE

    def state_transitions(self, old_state, new_state):
            
        # just entering the HOLD for the first time
        if new_state == State.HOLD and old_state != State.HOLD:
            # save the POSE from telemetry to hold at
            self.saveDroneHoldPose()
            # if not self.arm_status.is_stowed:
            #     self.stow_arm()

        if new_state == State.GRASPING and old_state != State.GRASPING:
            self.saveDroneHoldPose()

        elif new_state == State.FAILSAFE and old_state != State.FAILSAFE:

            # send a point to the drone to return to home position and 0.5m above the current flight level
            current_position = self.telemetry.pos.pose.position                
            self.sendWaypointNED(Point(x=0, y=0, z=(current_position.z - 0.5)))
            self.hasFailsafed = True

            # if not self.arm_status.is_stowed:
            #     self.stow_arm()
            
        # TODO:
        elif new_state != State.FAILSAFE and old_state == State.FAILSAFE:
            self.hasFailsafed = False

        elif new_state == State.SEARCHING and old_state != State.SEARCHING:
            # save the current position so we can spin around it
            self.searchStartTime = self.get_clock().now()
            self.searchStartHeading = self.telemetry.heading_degrees
            self.saveDroneHoldPose()
            self.debug(self.debugDrone, "Starting search pattern")

# ----- MAIN LOOP
    
    def main_loop(self):
        state_msg = String()
        state_msg.data = self.state.value
        self.publish_helper(self.statePublisher, state_msg)

        new_state = self.state
        
        # match self.state:
        #     case State.STARTUP:
        #         new_state = self.startup()

        #     etc.

        if self.state == State.STARTUP:
            new_state = self.startup()    
        elif self.state == State.FAILSAFE:
            new_state = self.failsafe()
        elif self.state == State.LANDING:
            new_state = self.land()
        elif self.state == State.HOLD:
            new_state = self.hold()
        elif self.state == State.SEARCHING:
            new_state = self.search()
        elif self.state == State.NAVIGATING:
            new_state = self.navigate()
        elif self.state == State.GRASPING:
            new_state = self.grasp()    
        elif self.state == State.DEPOSITING:
            new_state = self.deposit()
        elif self.state == State.WAITING:
            new_state = self.wait()
            
        if self.is_new_data_from_subscriber(self.stateSetterSubscriber):
            # Use new state from message if there's an incoming state
            self.debug(self.debugPublish,f"Updating state from received message: {self.recievedState}")
            new_state = self.recievedState            
            
        # if we are in normal operation, check for potential errors and failsafes
            # this still leaves the state machine fully running and states switchable using SSH
            # the RC control can always switch off offboard mode and take over manually
        if self.hasFailsafed == False:    
            if self.checkForErrors():
                self.debug(True, "ERRORS FOUND - MOVING TO HOLD STATE")
                new_state = State.HOLD
                
            if self.checkFailSafe():
                self.debug(True, "FAILSAFE TRIGGERED - MOVING TO FAILSAFE STATE")     
                new_state = State.FAILSAFE
        
        # any state transition behavior and set state
        self.state_transitions(self.state, new_state)     
        self.state = new_state

    def initLoop(self):
        '''
        Times the main loop to run at 20Hz (every 0.05 seconds)
        '''
        
        self.timer = self.create_timer(0.05, self.main_loop)
        self.debug(True, "Task Manager initialized with 20Hz timer")

def main(args=None):
    rclpy.init(args=args)

    manager_node = TaskManagerNode()
    
    # # Now actually allow ROS to process callbacks
    rclpy.spin(manager_node)

    manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
