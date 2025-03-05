# ROS2 imports 
import rclpy
from rclpy.node import Node

# CV Bridge and message imports
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Header
from vision_msgs.msg import Detection2D
from terrawarden_interfaces.msg import DroneTelemetry, DroneWaypoint
from geometry_msgs.msg import Pose2D, Point, PointStamped, PoseWithCovariance, PoseStamped
from builtin_interfaces.msg import Time
import tf2_ros
from collections import deque

# import transformations
import cv2
import numpy as np
import os
import math
from enum import Enum, auto
import time

class State(Enum):
    HOLD = "HOLD"
    SEARCHING = "SEARCH"
    NAVIGATING = "NAV"
    GRASPING = "GRASP"
    DEPOSITING = "DEPOSIT"
    WAITING = "WAIT"
    
# launch using the ros2 launch joisie_manager all_nodes
# ros2 topic pub -1 /joisie_manager/joisie_set_state std_msgs/msg/String "{data: 'SEARCH'}"

class TaskManagerNode(Node):

    def __init__(self):
        super().__init__('task_manager')
        
        # drone flight parameters dictionary
        self.drone_params = {
            "fast_max_lin_vel_m_s": 8.0,
            "fast_max_ang_vel_deg_s": 90.0,
            "fast_max_lin_accel_m_s2": 1.0,
            "fast_max_z_vel_m_s": 2.0,
            "precision_max_lin_vel_m_s": 1.0,
            "precision_max_ang_vel_deg_s": 30.0,
            "precision_max_lin_accel_m_s2": 0.4,
            "precision_max_z_vel_m_s": 0.5,
        }

        ### TOPIC DECLARATION - ALL PARAMETERIZED THROUGH ROS2 LAUNCH

        ### DRONE TOPICS ------------------------------------------------------

        # Topic for sending target pose to drone
        self.declare_parameter('drone_pose_topic', 'drone/waypoint')
        self.declare_parameter('drone_telemetry_topic', 'drone/telemetry')

        ## Topic for receiving Telemetry from Drone Node
        # self.declare_parameter('telemetry', 'joisie_telemetry')

        # Topic for sending target pose to drone
        # self.declare_parameter('drone_pose_topic', 'joisie_target_pose')

        ### DETECTION TOPICS --------------------------------------------------
        
        # Topic for receiving Detection from LiveDetect Node
        self.declare_parameter('detection_topic', 'joisie_detection')

        # Topic for sending detected object centroid to VBM
        self.declare_parameter('centroid_topic', 'joisie_detected_centroid')

        # Topic for receiving 3D point from VBM extract_cluster
        self.declare_parameter('vbm_extract_topic', 'joisie_extract_centroid')

        # Topic for receiving grasp from VBM optimal_grasp
        self.declare_parameter('vbm_grasp_topic', 'joisie_grasp_read')

        ### ARM TOPICS --------------------------------------------------------

        # Topic for sending grasp to arm
        self.declare_parameter('arm_grasp_topic', 'joisie_grasp_send')

        # Topic for sending grasp to arm WITH TRAJECTORY
        self.declare_parameter('traj_arm_grasp_topic', 'joisie_grasp_send')

        # CustomArmMsg from Arm Node
        self.declare_parameter('arm_status_topic', 'joisie_arm_status')

        # Topic for InRangeOfObj Service Call to Arm Node
        self.declare_parameter('arm_service_topic', 'joisie_arm_inrange_service')

        ### STATE TOPICS --------------------------------------------------
        
        # Topic to send state information
        self.declare_parameter('state_topic','joisie_state')

        self.declare_parameter('state_setter_topic', 'joisie_set_state')
        
        # -----
        # SERVICES

        # self.in_range_service = self.create_client(Bool, self.get_parameter('arm_grasp_topic').value) # TODO fix or delete

        # -----


        self.declare_parameter('debug', 0b11111)
        debug = self.get_parameter('debug').value
        self.debug_publish  = bool(debug & 0b10000)
        self.debug_drone    = bool(debug & 0b01000)
        self.debug_detect   = bool(debug & 0b00100)
        self.debug_vbm      = bool(debug & 0b00010)
        self.debug_arm      = bool(debug & 0b00001)
        self.get_logger().info(f"Debug Flags: Publish: {self.debug_publish}, Drone {self.debug_drone},"+
                               f" Detection {self.debug_detect}, VBM {self.debug_vbm}, Arm {self.debug_arm}")
        # INITIAL STATE
        self._state = State.HOLD
        self._telemetry = DroneTelemetry()
        self.telemetry_queue = deque()
        self._detection = Detection2D()
        self._extract_pt = PointStamped()
        self._raw_grasp = PoseStamped()
        # self._arm_status = DynaArmStatus()

        # STORE PREVIOUS MESSAGE SENT PER TOPIC
        self.last_sent_messages = {}
        self.msg_timeout = 3

        # search state tracking
        self.search_start_time = None
        self.search_start_heading = None

        # waiting state variables
        self.next_state = State.HOLD
        self._wait_time = 0
        self._wait_start_time = 0

        # -----
    # SUBSCRIBERS

        self.state_setter_subscriber = self.create_subscription(String, 
                                            self.get_parameter('state_setter_topic').value, 
                                            self.receive_desired_state, 10)
        
        self.detection_subscriber = self.create_subscription(Detection2D, 
                                                    self.get_parameter('detection_topic').value, 
                                                    self.get_setter("detection"), 10)
        
        self.telemetry_subscriber = self.create_subscription(DroneTelemetry, 
                                                    self.get_parameter('drone_telemetry_topic').value, 
                                                    self.get_setter("telemetry"), 10)
        
        self.extract_subscriber = self.create_subscription(PointStamped,
                                                    self.get_parameter('vbm_extract_topic').value,
                                                    self.get_setter("extract_pt"), 10)
        
        self.grasp_subscriber = self.create_subscription(PoseStamped,
                                                    self.get_parameter('vbm_grasp_topic').value, 
                                                    self.get_setter("raw_grasp"), 10)

        # self.arm_status_subscriber = self.create_subscription(DynaArmStatus, 
        #                                             self.get_parameter('arm_status_topic').value, 
        #                                             self.arm_status, 10)

        # -----
    # PUBLISHERS

        self.drone_publisher = self.create_publisher(DroneWaypoint, 
                                                    self.get_parameter('drone_pose_topic').value, 10)
        self.centroid_publisher = self.create_publisher(Pose2D, 
                                                    self.get_parameter('centroid_topic').value, 10)
        self.grasp_publisher = self.create_publisher(PoseStamped, 
                                                    self.get_parameter('arm_grasp_topic').value, 10)
        self.traj_grasp_publisher = self.create_publisher(PoseStamped, 
                                                    self.get_parameter('traj_arm_grasp_topic').value, 10)
        self.state_publisher = self.create_publisher(String,
                                                    self.get_parameter('state_topic').value, 10)
                
        # received_new needs to exist for the properties but needs the subscribers to exist as well
        self.received_new = {
            self.state_setter_subscriber.topic: False,
            # self.image_subscriber.topic: False,
            self.detection_subscriber.topic: False,
            self.telemetry_subscriber.topic: False,
            self.extract_subscriber.topic: False,
            self.grasp_subscriber.topic: False,
            # self.arm_status_subscriber.topic: False,
        }  

        self.init_loop()


# ----- PROPERTIES
    
    #### SET UP INCOMING MESSAGES AS PROPERTIES SO WE CAN KEEP TRACK OF WHAT
    #### NEW INFORMATION HAS / HASN'T BEEN ACCESSED
    
    # When python initializes properties, it hides the setters so that they get called when you do "self.property ="
    # This function retrieves the setter manually for any of the properties for the purposes of subscriber callbacks
    def get_setter(self, property):
        return lambda value: getattr(TaskManagerNode, property).fset(self, value)

    @property
    def state(self) -> State:
        self.received_new[self.state_setter_subscriber.topic] = False
        return self._state

    @state.setter
    def state(self, value) -> State:
        if value not in State:
            raise ValueError(f"Invalid state: {value}")
        self._state = value
    
    def receive_desired_state(self, ros_msg: String) -> None:
        self.received_new[self.state_setter_subscriber.topic] = True
        try:
            string = ros_msg.data
            self.state = State(string)
            self.debug(True, f"State changed due to incoming message: {string}")
        except:
            self.debug(True, f"[WARNING] No matching state for string: {string}")

    @property
    def detection(self) -> Detection2D:
        self.received_new[self.detection_subscriber.topic] = False
        return self._detection

    @detection.setter
    def detection(self, ros_msg: Detection2D) -> None:
        self.received_new[self.detection_subscriber.topic] = True
        self._detection = ros_msg
        

    @property
    def telemetry(self) -> DroneTelemetry:
        self.received_new[self.telemetry_subscriber.topic] = False
        return self._telemetry

    @telemetry.setter
    def telemetry(self, ros_msg: DroneTelemetry) -> None:
        self.received_new[self.telemetry_subscriber.topic] = True
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
        self.received_new[self.grasp_subscriber.topic] = False
        return self._raw_grasp

    @raw_grasp.setter
    def raw_grasp(self, ros_msg: PoseStamped) -> None:
        self.received_new[self.grasp_subscriber.topic] = True
        self._raw_grasp = ros_msg


    @property
    def extract_pt(self) -> PointStamped:
        self.received_new[self.extract_subscriber.topic] = False
        return self._extract_pt
    
    @extract_pt.setter
    def extract_pt(self, ros_msg: PointStamped) -> None:
        self.received_new[self.extract_subscriber.topic] = True
        self._extract_pt = ros_msg


    @property
    def wait_time(self) -> float:
        return self._wait_time
    
    @wait_time.setter
    def wait_time(self, val: float) -> None:
        self._wait_start_time = time.time()
        self._wait_time = val

    # @property
    # def arm_status(self) -> DynaArmStatus:
    #     self.received_new[self.arm_status_subscriber.topic] = False
    #     return self._arm_status
    
    # @arm_status.setter
    # def arm_status(self, ros_msg: DynaArmStatus):
    #     self.received_new[self.arm_status_subscriber.topic] = True
    #     self._arm_status = ros_msg

# ----- HELPER FNs

    def publish_helper(self, publisher, message) -> None:
        """EXISTS SO WE DON'T PUBLISH DUPLICATE MESSAGES
        ONLY PASS IF THE MESSAGE IS THE SAME 
        ROS MESSAGES ARE SET UP TO BE EQUAL IF HEADERS+CONTENT ARE IDENTICAL"""

        if self.is_outgoing_new_msg(publisher, message):
            # DEBUG REPORTS ALL OUTGOING MESSAGES
            # Node name, topic, message, extra debug info (if present)
            self.debug(self.debug_publish, f"Topic - {publisher.topic}\tMessage - {message}")

            publisher.publish(message)
            self.last_sent_messages[publisher.topic] = {"msg": message, "time": time.time()}

    def is_outgoing_new_msg(self, publisher, message) -> bool:
        '''RETURNS TRUE IF THIS IS A NEW OUTGOING MESSAGE'''

        return not (publisher.topic in self.last_sent_messages 
                    and self.last_sent_messages[publisher.topic]["msg"] == message
                    # Additionally, send message if previous message is more than {self.msg_timeout} seconds old
                    and self.last_sent_messages[publisher.topic]["time"] > time.time() - self.msg_timeout)
    
    def is_new_data_from_subscriber(self, subscriber):
        '''RETURNS TRUE IF THERE WAS A NEW MESSAGE RECEIVED ON THIS SUBSCRIBER'''
        return self.received_new[subscriber.topic]

    def get_last_sent_message(self, publisher):
        '''
        returns last sent message under publisher topic
        if there is no last sent message w/ that topic, returns None
        '''

        if not publisher.topic in self.last_sent_messages:
            return None
            
        return self.last_sent_messages[publisher.topic]["msg"]

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
            

# ----- WAIT

    def set_wait(self, next_state: State, wait_time_s: float) -> State:
        '''
        next_state is the desired state after wait time
        wait_time_s is time IN SECONDS
        '''
        
        self.wait_time = wait_time_s
        self.next_state = next_state
        
        return State.WAITING

    def wait(self) -> State:
        '''
        this loops when wait is current state
        no inputs, outputs State
        '''
        if time.time() > self._wait_start_time + self.wait_time:
            return self.next_state
        return State.WAITING

# ----- HOLD

    def hold(self) -> State:  
        '''
        this loops when hold is current state
        no inputs, outputs State
        '''

        self.droneHover()
        # listening for desired state happens async from this'

        return State.HOLD

# ----- GRASP

    def grasp(self) -> State:
        '''
        this loops when grasp is current state
        no inputs, outputs State
        '''

        self.droneHover()
        
        # calculate grasp
        # generate posestamped message from grastp
        if self.is_new_data_from_subscriber(self.grasp_subscriber):
            self.raw_grasp
            self.sendArmToPoint(self.raw_grasp)
        
        return State.GRASPING # TODO - what state makes sense to move into?
    
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
        else: waypoint_msg.max_ang_vel_deg_s = self.drone_params["precision_max_ang_vel_deg_s"]
        
        if max_lin_vel_m_s: waypoint_msg.max_lin_vel_m_s = max_lin_vel_m_s
        else: waypoint_msg.max_lin_vel_m_s = self.drone_params["precision_max_lin_vel_m_s"]
            
        if max_z_vel_m_s: waypoint_msg.max_z_vel_m_s = max_z_vel_m_s
        else: waypoint_msg.max_z_vel_m_s = self.drone_params["precision_max_z_vel_m_s"]
            
        if max_lin_accel_m_s2: waypoint_msg.max_lin_accel_m_s2 = max_lin_accel_m_s2
        else: waypoint_msg.max_lin_accel_m_s2 = self.drone_params["precision_max_lin_accel_m_s2"]     
    
        self.publish_helper(self.drone_publisher, waypoint_msg)  
    
    
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
        Returns the offset as a Point in NED, not the global NED
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
        
        last_setpoint = self.get_last_sent_message(self.drone_publisher).ned_pos

        return Point(x=last_setpoint.x + offset_ned.x, 
                    y=last_setpoint.y + offset_ned.y, 
                    z=last_setpoint.z + offset_ned.z)
        
    def FLU2NED_quaternion(self, FLUoffsetPoint: Point, timestamp: Time = None) -> Point:
        """
        Convert a local FLU offset in meters to global NED coordinates
        Accepts a ROS2 timestamp in nanoseconds since epoch??? and if specified grabs the quaternion at that point in time
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
        if timestamp is None:
            quat = self.telemetry.pos.pose.orientation
        else:
            # get the closest telemetry message to the timestamp
            telemetry = self.get_telemetry_closest_to_time(timestamp)
            quat = telemetry.pos.pose.orientation
            
        quat_np = np.array([quat.w, quat.x,  quat.y, quat.z])
        quat_np_conjugate = np.array([quat.w, -quat.x, -quat.y, -quat.z])
        
        # quaternion multiplication is commutative - perform the passive rotation
        rotated_point = multiply_quaternion(multiply_quaternion(quat_np, point_np), quat_np_conjugate)
        
        # extract the rotated point
        NED_offset_point = Point(x=rotated_point[1], y=rotated_point[2], z=rotated_point[3])
        
        #TODO: may need to use the last_NED_pos from drone telemetry vs the last one that was sent out
        last_setpoint = self.get_last_sent_message(self.drone_publisher).ned_pos

        return Point(x=last_setpoint.x + NED_offset_point.x, 
                    y=last_setpoint.y + NED_offset_point.y, 
                    z=last_setpoint.z + NED_offset_point.z)
         

    def droneHover(self) -> None:   
        """
        Sends last waypoint to drone as new waypoint, essentially tells it to stay where it is.    
        """
        # finds last waypoint / current position
        if (self.get_last_sent_message(self.drone_publisher) is None):
            telemetry_point = self.telemetry.pos.pose.position
            last_setpoint = Point(x=telemetry_point.x, y=telemetry_point.y, z=telemetry_point.z)
        else:
            last_setpoint = self.get_last_sent_message(self.drone_publisher).ned_pos

        # sends message
        self.sendWaypointNED(Point(x=last_setpoint.x, y=last_setpoint.y, z=last_setpoint.z))
        self.debug(self.debug_drone, f'drone hovering @ {last_setpoint}')
        

    def isInRangeNED(self, NEDpoint: Point, toleranceXY: float, toleranceZ: float) -> bool:
        """
        Returns true if drone's location is at given NEDpoint plus or minus given tolerance
        Otherwise returns false
        """

        last_received = self.telemetry.pos.pose

        # if there is no waypoint, false
        if last_received is None:
            return False
        
        self.debug(self.debug_vbm,f'NED point from VBM: ({NEDpoint.x},{NEDpoint.y},{NEDpoint.z})')
        self.debug(self.debug_drone, f'diffX, diffY, diffZ ({abs(last_received.position.x - NEDpoint.x)}, {abs(last_received.position.y - NEDpoint.y)}, {abs(last_received.position.z - NEDpoint.z)})')
        
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
            self.publish_helper(self.traj_grasp_publisher, poseStampedMsg)
        else:
            self.publish_helper(self.grasp_publisher, poseStampedMsg)


# ----- SEARCH

    def search(self) -> State:
        """
        Move drone to next position in search pattern
        No inputs, no outputs
        """
        # TODO - make this a real search pattern
                
        # just slowly spin for now, slowly
        rpm = 2.0    
        degrees_per_second = rpm * 360.0 / 60.0
        
        self.debug(self.debug_drone, f'spinning, RPM: {rpm}') 

        # init
        if self.search_start_time is None:
            self.search_start_time = self.get_clock().now()
            self.search_start_heading = self.telemetry.heading_degrees
            self.debug(self.debug_drone, "Starting search pattern")

        # desired heading based on elapsed time
        current_time = self.get_clock().now()
        elapsed = (current_time - self.search_start_time).nanoseconds / 1e9
        desired_heading = (self.search_start_heading + elapsed * degrees_per_second) % 360.0

        # send waypoint maintaining position but rotating
        if self.get_last_sent_message(self.drone_publisher) is not None:
            self.sendWaypointNED(
                self.get_last_sent_message(self.drone_publisher).ned_pos,   
                heading=desired_heading,
                max_ang_vel_deg_s=degrees_per_second * 1.2  # 20% for smoothness
            )
        # else:
        #     self.debug(self.debug_drone, f"Drone publisher history None; {self.last_sent_messages}")

        # detection found, transition states
        if self.is_new_data_from_subscriber(self.detection_subscriber):
            
            # reset search timer, heading
            self.search_start_time = None   
            self.search_start_heading = None

            self.debug(self.debug_drone, f'found object, changing to NAVIGATING') 

            return State.NAVIGATING
        
        self.debug(self.debug_drone, f'nothing found, continue SEARCHING') 

        return State.SEARCHING

# ----- NAVIGATE

    def navigate(self) -> State:
        """ 
        Perform approach sequence
        No inputs, output State
        """
        
        #TODO: if object is lost for 10s + and you hover at position, go back to search
        
        # if new extracted pt, recalculate approach
        if self.is_new_data_from_subscriber(self.extract_subscriber):
            
            self.debug(self.debug_vbm, f'new point extracted, recalculating approach') 

            # convert that 3D point to NED, offset it above and towards the drone a bit
            
            FLU_pos = self.offsetPointFLU(self.extract_pt.point, Point(x=-0.5, y=0., z=0.5))
            NED_pos = self.FLU2NED_quaternion(FLU_pos, self.extract_pt.header.stamp) 
                    
            # calculate heading needed to turn towards point
            diff_north = NED_pos.x - self.telemetry.pos.pose.position.x
            diff_east = NED_pos.y - self.telemetry.pos.pose.position.y
            px4_yaw_rad = math.atan2(diff_east, diff_north) 
            heading_deg = self.px4_yaw_to_heading(px4_yaw_rad)            
            
            self.sendWaypointNED(NED_pos, heading_deg, self.drone_params["precision_max_ang_vel_deg_s"], self.drone_params["precision_max_lin_vel_m_s"], self.drone_params["precision_max_z_vel_m_s"], self.drone_params["precision_max_lin_accel_m_s2"])    

            # if the new waypoint is within a certain distance of the robot, switch to grasping state
            if self.isInRangeNED(NED_pos, 0.5, 0.2):  #TODO: ROS-tunable params
                self.debug(self.debug_vbm, f'within range of object, begin GRASPING') 

                return self.set_wait(State.GRASPING, 3) # Move to grasp after 3 seconds

        # stay in navigation state
        self.debug(self.debug_drone, f'out of range, continue NAVIGATING') 

        return State.NAVIGATING

# -----
    
    def deposit(self) -> State:
        '''
        deposit state function
        no inputs, returns a State
        '''
        # drops the object
        #  pass this for now
        return State.DEPOSITING # TODO

    def checkForErrors(self) -> bool:
        '''
        checks for ground clearence beneath the drone
        prints errors + wardnings to ros logger
        
        returns True if there is error
        '''

        # Read most recent Telemetry and ArmStatus data
        # Set mode to State.HOLD if any errors
        if self.is_new_data_from_subscriber(self.telemetry_subscriber):
            
            # check that we have more than 0.35m of ground clearance beneath the drone
            if self.telemetry.altitude_above_ground < 0.35:
                # check that we are not too low
                self.debug(self.debug_drone, "Ground Too Close, Abort")
                return False          
            
            if self.telemetry.altitude_above_ground < 0.6:
                # check that we are not too low
                self.debug(self.debug_drone, "Ground Proximity Warning")
                return False
                
        return False # TODO

# ----- MAIN LOOP

    def init_loop(self):
        '''
        Times the main loop to run at 20Hz (every 0.05 seconds)
        '''
        
        self.timer = self.create_timer(0.05, self.main_loop)
        self.debug(True, "Task Manager initialized with 20Hz timer")
    
    def main_loop(self):
        state_msg = String()
        state_msg.data = self.state.value
        self.publish_helper(self.state_publisher, state_msg)

        new_state = self.state
        
        if self.state == State.HOLD:
            new_state = self.hold()
        elif self.state == State.SEARCHING:
            new_state = self.search()
        elif self.state == State.NAVIGATING:
            new_state = self.navigate()
        elif self.state == State.GRASPING:
            new_state = self.grasp()
            # self.debug(self.debug_drone, f'state action is HOLD (debug 2/28)') 
            # new_state = self.hold()
        elif self.state == State.DEPOSITING:
            new_state = self.deposit()
            # self.debug(self.debug_drone, f'state action is HOLD (debug 2/28)') 
            # new_state = self.hold()
        elif self.state == State.WAITING:
            new_state = self.wait()
            
        if self.checkForErrors():
            self.debug(True, "ERRORS FOUND - MOVING TO HOLD STATE")
            new_state = State.HOLD

        # Directly set _state to avoid setter loop
        if not self.is_new_data_from_subscriber(self.state_setter_subscriber):
            self.state = new_state


def main(args=None):
    rclpy.init(args=args)

    manager_node = TaskManagerNode()
    
    # # Now actually allow ROS to process callbacks
    rclpy.spin(manager_node)

    manager_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
