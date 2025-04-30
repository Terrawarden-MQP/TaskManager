from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
# https://docs.ros.org/en/rolling/Tutorials/Intermediate/Launch/Using-ROS2-Launch-For-Large-Projects.html#writing-launch-files
def generate_launch_description():
    return LaunchDescription([
        # Task Manager arguments
        DeclareLaunchArgument("image_topic", default_value="/camera/camera/color/image_raw", description="Topic for receiving raw image from camera"),
        DeclareLaunchArgument("detection_topic", default_value="joisie_detection", description="Topic for receiving Detection from LiveDetect Node"),
        # DeclareLaunchArgument("centroid_topic", default_value="joisie_detected_centroid", description="Topic for sending detected object centroid to VBM"), # SEE coord_topic
        DeclareLaunchArgument("drone_pose_topic", default_value="joisie_target_pose", description="Topic for sending target pose to drone"),
        DeclareLaunchArgument("vbm_extract_topic",default_value="joisie_extract_centroid", description="Topic for receiving 3D Point from VBM extract_cluster"),
        # DeclareLaunchArgument("vbm_grasp_topic", default_value="joisie_grasp_read", description="Topic for receiving grasp from VBM"), # SEE pos_topic
        DeclareLaunchArgument('pos_topic', default_value='joisie_grasp_read', description='Grasp pose topic'),
        DeclareLaunchArgument("arm_grasp_topic", default_value="joisie_grasp_send", description="Topic for sending grasp to arm"),
        DeclareLaunchArgument("arm_status_topic", default_value="joisie_arm_status", description="CustomArmMsg from Arm Node"),
        DeclareLaunchArgument("arm_service_topic", default_value="joisie_arm_inrange_service", description="Topic for InRangeOfObj Service Call to Arm Node"),
        DeclareLaunchArgument("state_setter_topic", default_value="joisie_set_state", description="Topic for setting states"),
        DeclareLaunchArgument("state_topic",default_value="joisie_state",description="Topic to publish task manager state information"),
        DeclareLaunchArgument("manager_debug",default_value="0b11111",description="Flags for selecting which sections of code to debug"),
        DeclareLaunchArgument("override_errors",default_value="false",description="Flag for overriding error checking (useful for ground testing). If true, changes all wait times to 0.5"),
        
        Node(
            package="joisie_manager",
            namespace="joisie_manager",
            executable="task_manager",
            parameters=[{
                # "image_topic": LaunchConfiguration("image_topic"),
                "detection_topic": LaunchConfiguration("detection_topic"),
                "centroid_topic": LaunchConfiguration("coord_topic"),
                "drone_pose_topic": LaunchConfiguration("drone_pose_topic"),
                "vbm_extract_topic": LaunchConfiguration("vbm_extract_topic"),
                "vbm_grasp_topic": LaunchConfiguration("pos_topic"),
                "arm_grasp_topic": LaunchConfiguration("arm_grasp_topic"),
                "arm_status_topic": LaunchConfiguration("arm_status_topic"),
                "arm_service_topic": LaunchConfiguration("arm_service_topic"),
                "state_setter_topic": LaunchConfiguration("state_setter_topic"),
                "state_topic": LaunchConfiguration("state_topic"),
                "manager_debug": LaunchConfiguration("manager_debug"),
                "override_errors": LaunchConfiguration("override_errors"),
            }]
        ),
    ])