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
        self.declareParameter('armStatus_topic', 'joisie_armStatus')

        # Topic for InRangeOfObj Service Call to Arm Node
        self.declareParameter('arm_service_topic', 'joisie_arm_inrange_service')

        # Topic for Stow Service Call to Arm Node
        self.declareParameter('arm_stow_service_topic', 'stowArm')

        # Topic for Unstow Service Call to Arm Node
        self.declareParameter('arm_unstow_service_topic', 'unstowArm')

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
        self._extractPoint = PointStamped()
        self._rawGrasp = PoseStamped()
        # self._armStatus = DynaArmStatus()    

        # STORE PREVIOUS MESSAGE SENT PER TOPIC
        self.lastSentMsg = {}
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
        self._waitTime = 0
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
                                                    self.getSetter("extractPoint"), 10)
        
        self.graspSubscriber = self.create_subscription(PoseStamped,
                                                    self.get_parameter('vbm_grasp_topic').value, 
                                                    self.getSetter("rawGrasp"), 10)

        self.armStatusSubscriber = self.create_subscription(ArmStatus, 
                                                    self.get_parameter('armStatus_topic').value, 
                                                    self.getSetter("armStatus"), 10)

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

        compareTimes = lambda t1, t2: abs(t1.sec + t1.nanosec/1e9 - t2.sec - t2.nanosec/1e9)
        TELEMETRY_QUEUE_TIME = 5 # SECONDS
        
        while True:
            msgTime = self.telemetry_queue[-1][0]
            # Compares old message time with newest message time
            if compareTimes(ros_msg.pos.header.stamp, msgTime) > TELEMETRY_QUEUE_TIME:
                # Remove old message if it is more than TELEMETRY_QUEUE_TIME seconds older than the newest message
                self.telemetry_queue.pop()
            else:
                break

    @property
    def rawGrasp(self) -> PoseStamped:
        self.received_new[self.graspSubscriber.topic] = False
        return self._rawGrasp

    @rawGrasp.setter
    def rawGrasp(self, ros_msg: PoseStamped) -> None:
        self.received_new[self.graspSubscriber.topic] = True
        self._rawGrasp = ros_msg


    @property
    def extractPoint(self) -> PointStamped:
        self.received_new[self.extractSubscriber.topic] = False
        return self._extractPoint
    
    @extractPoint.setter
    def extractPoint(self, ros_msg: PointStamped) -> None:
        self.received_new[self.extractSubscriber.topic] = True
        self._extractPoint = ros_msg


    @property
    def waitTime(self) -> float:
        return self._waitTime
    
    @waitTime.setter
    def waitTime(self, val: float) -> None:
        self._wait_start_time = time.time()
        self._waitTime = val

    @property
    def armStatus(self) -> ArmStatus:
        self.received_new[self.armStatusSubscriber.topic] = False
        return self._armStatus
    
    @armStatus.setter
    def armStatus(self, ros_msg: ArmStatus):
        self.received_new[self.armStatusSubscriber.topic] = True
        self._armStatus = ros_msg

# ----- HELPER FNs

    def publishHelper(self, publisher, message) -> None:
        """Exists so we don't publish duplicate messages 
            only pass is the message is the same
            ROS messages are equal if the headers AND content are identical

        Args:
            publisher (ROS Publisher): the publisher the message is being sent through
            message (ROS message): the message being sent
        """        

        if self.isOutgoingNewMsg(publisher, message):
            # DEBUG REPORTS ALL OUTGOING MESSAGES
            # Node name, topic, message, extra debug info (if present)
            self.debug(self.debugPublish, f"Topic - {publisher.topic}\tMessage - {message}")

            publisher.publish(message)
            self.lastSentMsg[publisher.topic] = {"msg": message, "time": time.time()}

    def isOutgoingNewMsg(self, publisher, message) -> bool:
        """checks if outgoing message is new

        Args:
            publisher (ROS publisher): the publisher the message is being sent through
            message (ROS message): the message being sent

        Returns:
            bool: TRUE if message is not equal to last message sent by publisher
        """

        return not (publisher.topic in self.lastSentMsg 
                    and self.lastSentMsg[publisher.topic]["msg"] == message
                    # Additionally, send message if previous message is more than {self.msgTimeout} seconds old
                    and self.lastSentMsg[publisher.topic]["time"] > time.time() - self.msgTimeout)
    
    def isNewDataFromSubscriber(self, subscriber) -> bool:
        """checks if new message was recieved on the subscriber

        Args:
            subscriber (ROS subscriber): the subscriber recieving information

        Returns:
            bool: TRUE if new message recieved on subscriber
        """

        return self.received_new[subscriber.topic]

    def getLastSentMsg(self, publisher):
        """gets last message sent under a publisher's topic

        Args:
            publisher (ROS publisher): publisher with desired topic to check

        Returns:
            ROS message: returns the last sent ros message in publisher.topic
            None: returns None if there is no message under publisher.topic
        """

        if not publisher.topic in self.lastSentMsg:
            return None
            
        return self.lastSentMsg[publisher.topic]["msg"]

    def debug(self, ifDebug:bool, string:String) -> None:
        """toggles wether to print to ros logger or not

        Args:
            ifDebug (bool): if TRUE, string gets printed to ros logger
            string (String): the desired info to print to the ros logger
        """

        if ifDebug:
            # Node name, topic, message, extra debug info (if present)
            self.get_logger().info(string)

    def get_telemetry_closest_to_time(self, msgTime: Time):
        '''
            Finds the telemetry message sent closest to the parameter msgTime
            TaskManager stores the last 10 seconds of Telemetry messages in self.telemetry_queue
        '''

        if len(self.telemetry_queue) == 0:
            return self.telemetry

        # Compute time difference in seconds
        compareTimes = lambda t1, t2: abs(t1.sec + t1.nanosec/1e9 - t2.sec - t2.nanosec/1e9)

        # default is the most recent message                       index \/
        closest = (compareTimes(self.telemetry_queue[-1][0], msgTime), -1)

        for i in range(len(self.telemetry_queue)):

            telemetryTime = self.telemetry_queue[i][0]
            # if message is closer in time than previous closest
            if compareTimes(telemetryTime,msgTime) < closest[0]:
                # store it and it's corresponding index instead
                closest = (compareTimes(telemetryTime, msgTime), i)

        # ONLY INCLUDE THE MESSAGE CONTENT (queue stores [timestamp, message])
        return self.telemetry_queue[closest[1]][1]
            
    def saveDroneHoldPose(self) -> None:
        """
        saves drone's held position
        """        

        self.holdNEDPoint = self.telemetry.pos.pose.position
        self.holdHeading = self.telemetry.heading_degrees
        
    def getDroneHoldPose(self):
        """gets saved drone hold pose

        Returns:
            Point: point the drone is set to hold at
            Float: heading, in degrees, between 0 and 360
        """        
        return self.holdNEDPoint, self.holdHeading


# ----- STOW FNs
    def stowArm(self) -> None: 
        """
        Calls stowArm service to stow the arm
        """        

        # Call the stowArm service
        self.get_logger().info('Calling stowArm service...')
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
    
    def unstowArm(self) -> None:
        """
        special routine to un-stow the arm, sending it to the zero position
        """

        # Call the unstowArm service
        self.get_logger().info('Calling unstowArm service...')
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

    def sendWaypointNED(self, NEDpoint: Point, heading:float=None, maxAngVel_deg_s:float=None, maxLinVel_m_s:float=None, maxZVel_m_s:float=None, maxLinAccel_m_s2:float=None):
        """wrapper to publish drone target waypoint
        maxAngVel_deg_s, maxLinVel_m_s, maxZVel_m_s, and maxLinAccel_m_s2 if not specified are left at precision (slow)

        Args:
            NEDpoint (Point): waypoint, in NED coordinates
            heading (float, optional): desired heading for drone motion. Defaults to None.
            maxAngVel_deg_s (float, optional): maximum angular velocity, in degrees/second. Defaults to None.
            maxLinVel_m_s (float, optional): maximum linear velocity in m/s. Defaults to None.
            maxZVel_m_s (float, optional): maximum ascent/descent velocity in m/s. Defaults to None.
            maxLinAccel_m_s2 (float, optional): maximum linear acceleration in m/s^2. Defaults to None.
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
        if maxAngVel_deg_s: waypoint_msg.maxAngVel_deg_s = maxAngVel_deg_s
        else: waypoint_msg.maxAngVel_deg_s = self.droneParams["precision_maxAngVel_deg_s"]
        
        if maxLinVel_m_s: waypoint_msg.maxLinVel_m_s = maxLinVel_m_s
        else: waypoint_msg.maxLinVel_m_s = self.droneParams["precision_maxLinVel_m_s"]
            
        if maxZVel_m_s: waypoint_msg.maxZVel_m_s = maxZVel_m_s
        else: waypoint_msg.maxZVel_m_s = self.droneParams["precision_maxZVel_m_s"]
            
        if maxLinAccel_m_s2: waypoint_msg.maxLinAccel_m_s2 = maxLinAccel_m_s2
        else: waypoint_msg.maxLinAccel_m_s2 = self.droneParams["precision_maxLinAccel_m_s2"]     
    
        self.publishHelper(self.dronePublisher, waypoint_msg)  
    
    
    def offsetPointFLU(self, FLUpoint: Point, FLUoffset: Point) -> Point:
        """Approach the drone to a point in FLU coordinates

        Args:
            FLUpoint (Point): [forward, left, up] in METERS
            FLUoffset (Point): desired offset as [delta(forward), delta(left), delta(up)]

        Returns:
            Point: a point the desired offset away from the given FLUpoint
        """
        
        return Point(x=FLUpoint.x + FLUoffset.x, y=FLUpoint.y + FLUoffset.y, z=FLUpoint.z + FLUoffset.z)

    def FLUtoNED(self, FLUpoint: Point, heading_deg: float = None) -> Point:
        """transforms FLU coordinates to NED coordinates, assuming the drone is flat.

        Args:
            FLUpoint (Point): FLU point to be transformed
            heading_deg (float, optional): heading, in degrees, from 0 to 360. Defaults to None, in which case current heading is kept

        Returns:
            Point: point equivalent to given FLUpoint, in the NED coordinate frame
        """        
        
        # if no heading is given, keep current heading
        if heading_deg is None:
            heading_deg = self.telemetry.heading_degrees

        # convert yaw to radians
        yaw_rad = heading_deg * (math.pi/180)   
        
        # convert the offset to rotated FLU
        rotatedFLU_x = FLUpoint.x * math.cos(yaw_rad) + FLUpoint.y * math.sin(yaw_rad)
        rotatedFLU_y = FLUpoint.x * math.sin(yaw_rad) - FLUpoint.y * math.cos(yaw_rad)
        rotatedFLU_z = FLUpoint.z
        
        # convert to NED coordinates
        offsetNED = Point(x=rotatedFLUx, y=rotatedFLU_y, z=-rotatedFLU_z)
        
        knownPosition = self.telemetry.pos.pose.position

        return Point(x=knownPosition.x + offsetNED.x, 
                    y=knownPosition.y + offsetNED.y, 
                    z=knownPosition.z + offsetNED.z)
        
    def FLUtoNEDquaternion(self, FLUpoint: Point, timestamp: Time = None) -> Point:
        """apply current NED rotation quaternion to given FLUpoint to calculate new point in the NED frame, accounting for the drone rotation

        Args:
            FLUpoint (Point): given FLU point
            timestamp (Time, optional): time at which the calulation is run - this is used to get the last known position. Defaults to None.

        Returns:
            Point: point equivalent to FLUpoint in the NED frame
        """

        # https://danceswithcode.net/engineeringnotes/quaternions/quaternions.html
        def multiplyQuaternion(q1: np.array, q2: np.array) -> np.array:
            r0, r1, r2, r3 = q1
            s0, s1, s2, s3 = q2
            
            return np.array([
                r0*s0 - r1*s1 - r2*s2 - r3*s3,
                r0*s1 + r1*s0 - r2*s3 + r3*s2,
                r0*s2 + r1*s3 + r2*s0 - r3*s1,
                r0*s3 - r1*s2 + r2*s1 + r3*s0
            ])

        # do it in numpy, convert FLU to FRD
        pointNP = np.array([0, FLUpoint.x, -FLUpoint.y, -FLUpoint.z])
        
        # Get the current orientation quaternion
        # Quaternion rotation from FRD body frame to reference frame
        # use the last position from telemetry and updage it with what the camera sees
        if timestamp is None:
            quat = self.telemetry.pos.pose.orientation
            lastKnownPosition = self.telemetry.pos.pose.position
        else:
            # get the closest telemetry message to the timestamp
            telemetry = self.get_telemetry_closest_to_time(timestamp)
            quat = telemetry.pos.pose.orientation
            lastKnownPosition = telemetry.pos.pose.position
            
        quatNP = np.array([quat.w, quat.x,  quat.y, quat.z])
        quatNPConjugate = np.array([quat.w, -quat.x, -quat.y, -quat.z])
        
        # quaternion multiplication is commutative - perform the passive rotation
        rotatedPoint = multiplyQuaternion(multiplyQuaternion(quatNP, pointNP), quatNPConjugate)
        
        # extract the rotated point
        NEDoffsetPoint = Point(x=rotatedPoint[1], y=rotatedPoint[2], z=rotatedPoint[3])
        
        return Point(x=lastKnownPosition.x + NEDoffsetPoint.x, 
                    y=lastKnownPosition.y + NEDoffsetPoint.y, 
                    z=lastKnownPosition.z + NEDoffsetPoint.z)
         

    def droneHover(self) -> None:   
        """
        Sends last waypoint to drone as new waypoint, essentially tells it to stay where it is.    
        """
        
        lastSetpoint, lastHeading = self.retrieveDroneHoldPose()

        # sends message
        self.sendWaypointNED(lastSetpoint, lastHeading)
        self.debug(self.debugDrone, f'drone hovering @ {lastSetpoint} heading {lastHeading}')
        

    def isInRangeNED(self, NEDpoint: Point, toleranceXY: float, toleranceZ: float) -> bool:
        """checks if drone is at a given NEDpoint within a given tolerance

        Args:
            NEDpoint (Point): point being checked against current drone position
            toleranceXY (float): tolerance in the X/Y direction
            toleranceZ (float): tolerance in the Z direction


        Returns:
            bool: TRUE if drone's location is within tolerance of the given NEDpoint, FALSE otherwise
        """

        lastRecieved = self.telemetry.pos.pose

        # if there is no waypoint, false
        if lastRecieved is None:
            return False
        
        self.debug(self.debugVBM,f'NED point from VBM: ({NEDpoint.x},{NEDpoint.y},{NEDpoint.z})')
        self.debug(self.debugDrone, f'diffX, diffY, diffZ ({abs(lastRecieved.position.x - NEDpoint.x)}, {abs(lastRecieved.position.y - NEDpoint.y)}, {abs(lastRecieved.position.z - NEDpoint.z)})')
        
        # if difference between current point and set point is greater than tolerance, false
        if abs(lastRecieved.position.x - NEDpoint.x) > toleranceXY: return False
        if abs(lastRecieved.position.y - NEDpoint.y) > toleranceXY: return False
        if abs(lastRecieved.position.z - NEDpoint.z) > toleranceZ: return False
        
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

    
    #Unused
    def heading_to_px4_yaw(self, heading: float) -> float
        """
        Convert a heading in degrees [0, 360) to a PX4 yaw in radians (-pi, pi]
        Heading is in degrees, yaw is in radians
        """
        heading = heading % 360.0
        
        if heading > 180.0:
            heading = heading - 360.0
            
        return heading * (math.pi / 180.0) # heading is backwards 
    
    def px4YawToHeading(self, yaw: float) -> float:
        """convert PX4 yaw in radians to (-pi, pi] ot a heading in degrees [0, 360)]

        Args:
            yaw (float): heading from the PX4, in Radians

        Returns:
            float: heading in degrees
        """

        return (yaw * (180.0 / math.pi)) % 360.0

# ----- ARM HELPERS

    def openGripper(self) -> None:
        """send ROSmsg to arm control node to open gripper
        """

        self.publishHelper(self.gripper_publisher, True)

    
    def closeGripper(self) -> None:
        """send ROSmsg to arm control node to close gripper
        """

        self.publishHelper(self.gripper_publisher, False)


    def sendArmToPoint(self, poseStampedMsg, trajectory:bool=True) -> None:
        """wrapper to send the arm end effector to a specific point

        Args:
            poseStampedMsg (PoseStamped): end effector pose to be transmitted
            trajectory (bool, optional): determines wether or not trajectory control is used. Defaults to True.
        """         

        # send posestamped to different topics depending wether trajectory is true or false
        if trajectory:
            self.publishHelper(self.trajGraspPublisher, poseStampedMsg)
        else:
            self.publishHelper(self.graspPublisher, poseStampedMsg)
            
# ----- STATE FUNCTIONS (not helpers!)

    def setWait(self, nextState: State, waitTime_s: float = 0.5, waitUntilFn: Callable[[None], bool] = None) -> State:
        """sets the wait time between states

        Args:
            nextState (State): desired state to transition to after wait period over
            waitTime_s (float, optional): time to wait, in seconds. Defaults to 0.5.
            waitUntilFn (Callable[[None], bool], optional): waits until given function returns TRUE. Defaults to None.

        Returns:
            State: the WAITING state
        """

        # Default is time-based
        def defaultWaitFn():
            return time.time() > self._wait_start_time + self.waitTime
        if waitUntilFn is None:
            waitUntilFn = defaultWaitFn

        self.waitTime = waitTime_s
        self.nextState = nextState
        self.waitUntilFn = waitUntilFn

        return State.WAITING

    def wait(self) -> State:
        """this loops when WAIT is the current state
        does not command the drone in any way, shape, or form

        Returns:
            State: either next desired state OR WAITING state
        """

        if self.waitUntilFn():
            return self.nextState
        return State.WAITING
    
# ----- STARTUP

    def startup(self) -> State:
        """this loops when startup is the current state

        Returns:
            State: either STARTUP or, if drone is ready, HOLD
        """
        
        # check if drone is flying and offboard
        if self.isNewDataFromSubscriber(self.telemetrySubscriber):
            if self.telemetry.is_flying == True and self.telemetry.is_offboard == True:
                self.debug(self.debugDrone, "Drone is offboard and armed, switching to HOLD")
                return State.HOLD                
        
        return State.STARTUP
    
# ----- FAILSAFE

    def failsafe(self) -> State:
        """upon entering this state keep slowly returning to home armed position and land
        used only in case the RC link has been lost to prevent runaway of the system

        Returns:
            State: LANDING if within a meter of the home position in XY, FAILSAFE otherwise
        """


        '''
        upon entering this state keep slowly returning to home armed position and land
        used only in case the RC link has been lost to prevent runaway of the system
        '''

        # when it is within a meter of the home position in XY only
        if self.isInRangeNED(Point(x=0, y=0, z=-10), 2.0, 10000.0):
            self.debug(self.debugDrone, "Drone failsafe is in range of home position, switching to LANDING")    
            
            # send the setpoint to go down the current above ground altitude (+ 2 extra meters for good measure)
            # the PX4 autopilot should detect the landing, and disarm the drone automatically
            currentNEDpos = self.telemetry.pos.pose.position
            # e.g.: the altitude above ground is 10, so the drone should descend down (10+2) meters to land on the ground    
            pointToLand = self.FLUtoNED(Point(x=0, y=0, z=-(self.telemetry.altitude_above_ground_m + 2)))
            self.sendWaypointNED(pointToLand, 0)
            return State.LANDING
        
        return State.FAILSAFE
    
# ----- LANDING

    def land(self) -> State:
        """lands the drone by issuing a PX4 command to land, 
        this is a failsafe state, so it will keep trying to land until it is on the ground

        Returns:
            State: LANDING
        """
        
        # self.sendWaypointNED(Point(x=0, y=0, z=0), 0, self.droneParams["precision_maxAngVel_deg_s"], self.droneParams["precision_maxLinVel_m_s"], self.droneParams["precision_maxZVel_m_s"], self.droneParams["precision_maxLinAccel_m_s2"])                   
        
        return State.LANDING
    
    def hold(self) -> State:  
        """this loops when hold is current state

        Returns:
            State: STARTUP only is offboard mode is deactivated, otherwise HOLD
        """

        self.droneHover()
        # listening for desired state happens async from this'

    
        # if not self.armStatus.is_stowed:
        #     self.stowArm()
        
        # the only time it can go back to startup is, if it is in hold and the offboard mode is deactivated
        if self.telemetry.is_offboard == False:
            self.debug(self.debugDrone, "Drone is not in offboard, switching to STARTUP")            
            return State.STARTUP

        return State.HOLD

# ----- GRASP

    def grasp(self) -> State:
        """this loops when GRASPING is current state

        Returns:
            State: GRASPING
        """        

        '''
        this loops when grasp is current state
        no inputs, outputs State
        '''

        self.droneHover()
        
        # calculate grasp
        # generate posestamped message from grasp
        # TODO if refresh rate gets to live rate, use this instead
        # if self.isNewDataFromSubscriber(self.graspSubscriber):
        #     # self.rawGrasp
        #     self.sendArmToPoint(self.rawGrasp)

        
        if self.isNewDataFromSubscriber(self.extractSubscriber):
            pt = PoseStamped()
            pt.header = self.extractPoint.header
            pt.pose.position = self.extractPoint.point
            self.sendArmToPoint(pt)
        
        return State.GRASPING # TODO - what state makes sense to move into?

# ----- SEARCH

    def search(self) -> State:
        """this loops when SEARCHING is active state

        Returns:
            State: SEARCHING, or, if object found, NAVIGATING
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
            maxAngVel_deg_s=degrees_per_second * 1.2  # 20% for smoothness
        )

        # detection found, transition states
        if self.isNewDataFromSubscriber(self.extractSubscriber):

            self.debug(self.debugDrone, f'found object at ({self.extractPoint.point}), changing to HOLD') 

            return self.setWait(State.NAVIGATING, 5) # Move to grasp after 5 seconds 
        
        self.debug(self.debugDrone, f'nothing found, continue SEARCHING') 

        return State.SEARCHING

# ----- NAVIGATE

    def navigate(self) -> State:
        """this loops when NAVIGATE is active state

        Returns:
            State: NAVIGATING, or, if within range of object, GRASPING
        """

        """ 
        Perform approach sequence
        No inputs, output State
        """
        
        #TODO: if object is lost for 10s + and you hover at position, go back to search
        
        # if new extracted pt, recalculate approach
        if self.isNewDataFromSubscriber(self.extractSubscriber):
            
            self.debug(self.debugVBM, f'new point extracted, recalculating approach') 

            # convert that 3D point to NED, offset it above and towards the drone a bit
            
            FLU_pos = self.offsetPointFLU(self.extractPoint.point, Point(x=-0.707, y=0., z=0.707))
            NED_pos = self.FLUtoNEDquaternion(FLU_pos, self.extractPoint.header.stamp) 
                    
            # calculate heading needed to turn towards point
            diff_north = NED_pos.x - self.telemetry.pos.pose.position.x
            diff_east = NED_pos.y - self.telemetry.pos.pose.position.y
            px4_yaw_rad = math.atan2(diff_east, diff_north) 
            heading_deg = self.px4YawToHeading(px4_yaw_rad)            
            
            self.sendWaypointNED(NED_pos, heading_deg, self.droneParams["precision_maxAngVel_deg_s"], self.droneParams["precision_maxLinVel_m_s"], self.droneParams["precision_maxZVel_m_s"], self.droneParams["precision_maxLinAccel_m_s2"])    

            # if the new waypoint is within a certain distance of the robot, switch to grasping state
            if self.isInRangeNED(NED_pos, 0.1, 0.1):  #TODO: ROS-tunable params
                self.debug(self.debugVBM, f'within range of object, begin GRASPING') 
                # return State.GRASPING

                # if self.armStatus.is_stowed:
                #     self.unstowArm()

                def check_arm_position_unstowed():
                    return self.armStatus.at_setpoint and not self.armStatus.is_stowed

                # return self.setWait(State.GRASPING, check_arm_position_unstowed) # Move to grasp state when unstowed
                return self.setWait(State.GRASPING, 5) # Move to grasp after 5 seconds

        # stay in navigation state
        self.debug(self.debugDrone, f'out of range, continue NAVIGATING') 

        return State.NAVIGATING
    
    def deposit(self) -> State:
        """this loops when DEPOSITING is active state
        INCOMPLETE

        Returns:
            State: _description_
        """

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
        if self.isNewDataFromSubscriber(self.telemetrySubscriber):
            
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
        
        if self.isNewDataFromSubscriber(self.telemetrySubscriber):
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

    def stateTransitions(self, oldState, newState):
            
        # just entering the HOLD for the first time
        if newState == State.HOLD and oldState != State.HOLD:
            # save the POSE from telemetry to hold at
            self.saveDroneHoldPose()
            # if not self.armStatus.is_stowed:
            #     self.stowArm()

        if newState == State.GRASPING and oldState != State.GRASPING:
            self.saveDroneHoldPose()

        elif newState == State.FAILSAFE and oldState != State.FAILSAFE:

            # send a point to the drone to return to home position and 0.5m above the current flight level
            currentPosition = self.telemetry.pos.pose.position                
            self.sendWaypointNED(Point(x=0, y=0, z=(currentPosition.z - 0.5)))
            self.hasFailsafed = True

            # if not self.armStatus.is_stowed:
            #     self.stowArm()
            
        # TODO:
        elif newState != State.FAILSAFE and oldState == State.FAILSAFE:
            self.hasFailsafed = False

        elif newState == State.SEARCHING and oldState != State.SEARCHING:
            # save the current position so we can spin around it
            self.searchStartTime = self.get_clock().now()
            self.searchStartHeading = self.telemetry.heading_degrees
            self.saveDroneHoldPose()
            self.debug(self.debugDrone, "Starting search pattern")

# ----- MAIN LOOP
    
    def main_loop(self):
        stateMsg = String()
        stateMsg.data = self.state.value
        self.publishHelper(self.statePublisher, stateMsg)

        newState = self.state
        
        # match self.state:
        #     case State.STARTUP:
        #         newState = self.startup()

        #     etc.

        if self.state == State.STARTUP:
            newState = self.startup()    
        elif self.state == State.FAILSAFE:
            newState = self.failsafe()
        elif self.state == State.LANDING:
            newState = self.land()
        elif self.state == State.HOLD:
            newState = self.hold()
        elif self.state == State.SEARCHING:
            newState = self.search()
        elif self.state == State.NAVIGATING:
            newState = self.navigate()
        elif self.state == State.GRASPING:
            newState = self.grasp()    
        elif self.state == State.DEPOSITING:
            newState = self.deposit()
        elif self.state == State.WAITING:
            newState = self.wait()
            
        if self.isNewDataFromSubscriber(self.stateSetterSubscriber):
            # Use new state from message if there's an incoming state
            self.debug(self.debugPublish,f"Updating state from received message: {self.recievedState}")
            newState = self.recievedState            
            
        # if we are in normal operation, check for potential errors and failsafes
            # this still leaves the state machine fully running and states switchable using SSH
            # the RC control can always switch off offboard mode and take over manually
        if self.hasFailsafed == False:    
            if self.checkForErrors():
                self.debug(True, "ERRORS FOUND - MOVING TO HOLD STATE")
                newState = State.HOLD
                
            if self.checkFailSafe():
                self.debug(True, "FAILSAFE TRIGGERED - MOVING TO FAILSAFE STATE")     
                newState = State.FAILSAFE
        
        # any state transition behavior and set state
        self.stateTransitions(self.state, newState)     
        self.state = newState

    def initLoop(self):
        '''
        Times the main loop to run at 20Hz (every 0.05 seconds)
        '''
        
        self.timer = self.create_timer(0.05, self.main_loop)
        self.debug(True, "Task Manager initialized with 20Hz timer")

def main(args=None):
    rclpy.init(args=args)

    managerNode = TaskManagerNode()
    
    # # Now actually allow ROS to process callbacks
    rclpy.spin(managerNode)

    managerNode.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
