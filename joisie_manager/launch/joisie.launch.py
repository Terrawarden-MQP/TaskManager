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
        # Detection arguments
        DeclareLaunchArgument('show_cv', default_value="false", description='Display Raw OpenCV Output'),
        # Task Manager arguments
        DeclareLaunchArgument("image_topic", default_value="image", description="Topic for receiving raw image from camera"),
        DeclareLaunchArgument("detection_topic", default_value="joisie_detection", description="Topic for receiving Detection from LiveDetect Node"),
        # DeclareLaunchArgument("centroid_topic", default_value="joisie_detected_centroid", description="Topic for sending detected object centroid to VBM"), # SEE coord_topic
        DeclareLaunchArgument("drone_pose_topic", default_value="joisie_target_pose", description="Topic for sending target pose to drone"),
        DeclareLaunchArgument("vbm_extract_topic",default_value="joisie_extract_centroid", description="Topic for receiving 3D Point from VBM extract_cluster"),
        # DeclareLaunchArgument("vbm_grasp_topic", default_value="joisie_grasp_read", description="Topic for receiving grasp from VBM"), # SEE pos_topic
        DeclareLaunchArgument("arm_grasp_topic", default_value="joisie_grasp_send", description="Topic for sending grasp to arm"),
        DeclareLaunchArgument("arm_status_topic", default_value="joisie_arm_status", description="CustomArmMsg from Arm Node"),
        DeclareLaunchArgument("arm_service_topic", default_value="joisie_arm_inrange_service", description="Topic for InRangeOfObj Service Call to Arm Node"),
        DeclareLaunchArgument("state_setter_topic", default_value="joisie_set_state", description="Topic for setting states"),
        DeclareLaunchArgument("state_topic",default_value="joisie_state",description="Topic to publish task manager state information"),
        # VBM arguments
        DeclareLaunchArgument('log_level', default_value='INFO', description='Log verbosity level'),
        DeclareLaunchArgument('cluster_topic', default_value='/detected_cluster', description='Cluster topic name'),
        DeclareLaunchArgument('pointcloud_topic', default_value='/camera/camera/depth/color/points', description='Pointcloud topic name'),
        DeclareLaunchArgument('coord_topic', default_value='/joisie_vision/detected_object_centroid', description='2D centroid target coordinates topic'),
        DeclareLaunchArgument('camera_info_topic_depth', default_value='/camera/camera/aligned_depth_to_color/camera_info', description='Camera depth image info topic'),
        DeclareLaunchArgument('camera_info_topic_color', default_value='/camera/camera/color/camera_info', description='Camera color image info topic'),
        DeclareLaunchArgument('camera_depth_topic', default_value='/camera/camera/aligned_depth_to_color/image_raw', description='Camera depth image topic'),
        DeclareLaunchArgument('visualize', default_value='false', description='Enable visualization in RViz of filters and normals'),
        DeclareLaunchArgument('crop_radius', default_value='0.2', description='Crop box radius'),
        DeclareLaunchArgument('sor_mean_k', default_value='50', description='SOR mean K'),
        DeclareLaunchArgument('sor_stddev_mul_thresh', default_value='1.0', description='SOR stddev multiplier threshold'),
        DeclareLaunchArgument('voxel_leaf_size', default_value='0.01', description='Voxel leaf size'),
        DeclareLaunchArgument('ransac_max_iterations', default_value='1000', description='RANSAC max iterations'),
        DeclareLaunchArgument('ransac_distance_threshold', default_value='0.005', description='RANSAC distance threshold'),
        DeclareLaunchArgument('header_frame', default_value='camera_color_optical_frame', description='Frame of reference for the camera point cloud color'),
        DeclareLaunchArgument('header_frame_drone', default_value='drone_frame', description='Frame of reference for the drone'),
        DeclareLaunchArgument('header_frame_depth', default_value='camera_depth_optical_frame', description='Frame of reference for the camera point cloud depth'),
        DeclareLaunchArgument('cluster_tolerance', default_value='0.02', description='Cluster tolerance'),
        DeclareLaunchArgument('min_cluster_size', default_value='100', description='Minimum cluster size'),
        DeclareLaunchArgument('max_cluster_size', default_value='25000', description='Maximum cluster size'),
        DeclareLaunchArgument('target_point_tolerance', default_value='0.02', description='Target point tolerance'),
        DeclareLaunchArgument('curvature', default_value='0.01', description='Curvature value for edge detection'),
        DeclareLaunchArgument('normal_search_radius', default_value='0.03', description='Normal search radius'),
        DeclareLaunchArgument('min_search_threshold', default_value='0.035', description='Minimum search threshold'),
        DeclareLaunchArgument('max_search_threshold', default_value='0.08', description='Maximum search threshold'),
        DeclareLaunchArgument('select_stability_metric', default_value='1', description='1: maximum minimum svd, 2: maximum volume ellipsoid in wrench space,\
                               3: isotropy index, 4: maximum minimum svd with abs for numeric stability, 5: weighing (1) and (2) equally'),
        DeclareLaunchArgument('variance_neighbors', default_value='4', description='Grasp uncertainty variance neighbors to search'),
        DeclareLaunchArgument('variance_threshold', default_value='0.2', description='Grasp uncertainty variance threshold'),
        DeclareLaunchArgument('pos_topic', default_value='joisie_grasp_read', description='Grasp pose topic'),
        
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
                    "show": LaunchConfiguration('show_cv')
            }]
        ),
        Node(
            package="joisie_manager",
            namespace="joisie_manager",
            executable="trt_detection_node",
            parameters=[{
                "image_topic": LaunchConfiguration("image_topic"),
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
            }]
        ),
        Node(
            package='grasp_vision_cpp',
            executable='extract_cluster',
            name='extract_cluster',
            parameters=[{
                'cluster_topic': LaunchConfiguration('cluster_topic'),
                'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
                'coord_topic': LaunchConfiguration('coord_topic'),
                'centroid_topic': LaunchConfiguration('vbm_extract_topic'),
                'camera_info_topic_depth': LaunchConfiguration('camera_info_topic_depth'),
                'camera_info_topic_color': LaunchConfiguration('camera_info_topic_color'),
                'camera_depth_topic': LaunchConfiguration('camera_depth_topic'),
                'visualize': LaunchConfiguration('visualize'),
                'crop_radius': LaunchConfiguration('crop_radius'),
                'sor_mean_k': LaunchConfiguration('sor_mean_k'),
                'sor_stddev_mul_thresh': LaunchConfiguration('sor_stddev_mul_thresh'),
                'voxel_leaf_size': LaunchConfiguration('voxel_leaf_size'),
                'ransac_max_iterations': LaunchConfiguration('ransac_max_iterations'),
                'ransac_distance_threshold': LaunchConfiguration('ransac_distance_threshold'),
                'header_frame': LaunchConfiguration('header_frame'),
                'header_frame_drone': LaunchConfiguration('header_frame_drone'),
                'cluster_tolerance': LaunchConfiguration('cluster_tolerance'),
                'min_cluster_size': LaunchConfiguration('min_cluster_size'),
                'max_cluster_size': LaunchConfiguration('max_cluster_size'),
                'target_point_tolerance': LaunchConfiguration('target_point_tolerance'),
                'curvature': LaunchConfiguration('curvature'),
                'normal_search_radius': LaunchConfiguration('normal_search_radius'),
                'state_topic': LaunchConfiguration('state_topic',)
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        Node(
            package='grasp_vision_cpp',
            executable='optimal_grasp',
            name='optimal_grasp',
            parameters=[{
                'cluster_topic': LaunchConfiguration('cluster_topic'),
                'normal_search_radius': LaunchConfiguration('normal_search_radius'),
                'min_search_threshold': LaunchConfiguration('min_search_threshold'),
                'max_search_threshold': LaunchConfiguration('max_search_threshold'),
                'visualize': LaunchConfiguration('visualize'),
                'select_stability_metric': LaunchConfiguration('select_stability_metric'),
                'variance_neighbors': LaunchConfiguration('variance_neighbors'),
                'variance_threshold': LaunchConfiguration('variance_threshold'),
                'pos_topic': LaunchConfiguration('pos_topic'),
            }],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')]
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_tf_pub",
            # Pitch rotation 30 deg + translation
            arguments = ['--x', '0.1', '--y', '0', '--z', '-0.16', '--yaw', '0', '--pitch', '0.523599', '--roll', '0', '--frame-id', 'drone_frame', '--child-frame-id', 'camera_link']
        )
    ])