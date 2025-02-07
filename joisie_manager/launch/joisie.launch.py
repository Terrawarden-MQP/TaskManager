from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
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
                }.items() 
        ),
        Node(
            package="joisie_vision",
            namespace="joisie_vision",
            executable="trt_detection",
            parameters=[
                {
                    "topic":"/camera/camera/color/image_raw"
                }
            ]
        )
    ])