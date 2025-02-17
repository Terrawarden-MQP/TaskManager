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
        DeclareLaunchArgument('show_cv', default_value="false", description='Display Raw OpenCV Output'),
        DeclareLaunchArgument("image_topic", default_value="image", description="Topic for receiving raw image from camera"),
        DeclareLaunchArgument("detection_topic", default_value="joisie_detection", description="Topic for receiving Detection from LiveDetect Node"),
        DeclareLaunchArgument("centroid_topic", default_value="joisie_detected_centroid", description="Topic for sending detected object centroid to VBM"),
        DeclareLaunchArgument("drone_pose_topic", default_value="joisie_target_pose", description="Topic for sending target pose to drone"),
        DeclareLaunchArgument("vbm_grasp_topic", default_value="joisie_grasp_read", description="Topic for receiving grasp from VBM"),
        DeclareLaunchArgument("arm_grasp_topic", default_value="joisie_grasp_send", description="Topic for sending grasp to arm"),
        DeclareLaunchArgument("arm_status_topic", default_value="joisie_arm_status", description="CustomArmMsg from Arm Node"),
        DeclareLaunchArgument("arm_service_topic", default_value="joisie_arm_inrange_service", description="Topic for InRangeOfObj Service Call to Arm Node"),
        DeclareLaunchArgument("state_setter_topic", default_value="joisie_set_state", description="Topic for setting states"),
        # PREVENT CAMERA MESSAGES FROM BEING SENT OVER NETWORK
        ExecuteProcess(
            cmd=['bash', '-c', 'export ROS_LOCALHOST_ONLY=1'], # Command to execute
            shell=True, # Run in a shell
            name='shell_command', # Name for the process
            output='screen' # Display output in the terminal
        ),
        IncludeLaunchDescription(
            # package="realsense2_camera",
            # launch="rs_launch.py",
            FindPackageShare('realsense2_camera').find('realsense2_camera') + '/launch/rs_launch.py',
            launch_arguments=
                {
                    "depth_module.depth_profile":"480,270,5",
                    "depth_module.exposure":"8000",
                    "enable_sync":"true",
                    "pointcloud.enable":"true",
                    "enable_color":"true",
                    "initial_reset":"true",
                    "rgb_camera.color_profile":"1280,720,15",
                    "align_depth.enable":"true"
                }.items() 
        ),
        Node(
            package="joisie_vision",
            namespace="joisie_vision",
            executable="trt_detection",
            parameters=[{
                    "topic":"/camera/camera/color/image_raw",
                    # "show": LaunchConfiguration('show_cv')
            }]
        ),
        Node(
            package="joisie_manager",
            namespace="joisie_manager",
            executable="trt_detection_node",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
                "detection_topic": LaunchConfiguration("detection_topic"),
                "centroid_topic": LaunchConfiguration("centroid_topic"),
                "drone_pose_topic": LaunchConfiguration("drone_pose_topic"),
                "vbm_grasp_topic": LaunchConfiguration("vbm_grasp_topic"),
                "arm_grasp_topic": LaunchConfiguration("arm_grasp_topic"),
                "arm_status_topic": LaunchConfiguration("arm_status_topic"),
                "arm_service_topic": LaunchConfiguration("arm_service_topic"),
                "state_setter_topic": LaunchConfiguration("state_setter_topic"),
            }]
        )
    ])