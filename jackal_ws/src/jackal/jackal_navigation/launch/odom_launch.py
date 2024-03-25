from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    odom_launch = FindPackageShare('jackal_navigation')
    realsense2_camera = FindPackageShare('realsense2_camera')
    remappings_visual=[
          ('rgb/image', '/camera/color/image_raw'),
          ('rgb/camera_info', '/camera/color/camera_info'),
          ('depth/image', '/camera/aligned_depth_to_color/image_raw')]
    remappings_lidar=[
          ('scan', '/front/scan'),
          ]
    visual_odom_config = PathJoinSubstitution(
        [odom_launch, 'config', 'visual_odom.yaml'])
    lidar_odom_config = PathJoinSubstitution(
        [odom_launch, 'config', 'lidar_odom.yaml'])    
    realsense = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [realsense2_camera ,
                     'launch',
                     'rs_launch.py'])),
            launch_arguments={'enable_rgbd': 'true',
                              'enable_sync': 'true',
                              'align_depth.enable':'true',
                              'enable_color' : 'true',
                              'enable_depth' : 'true',
                              'depth_module.profile' : '640x480x30',
                              'rgb_camera.profile' : '640x480x30',
                              'publish_tf' : 'false'
                             }.items())
    
    visual_odom = Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            namespace='visual',
            output='screen',
            remappings=remappings_visual,
            parameters=[
              visual_odom_config
            ],
        )
    lidar_odom = Node(
            package='rtabmap_odom',
            executable='icp_odometry',
            namespace='lidar',
            output='screen',
            remappings=remappings_lidar,
            parameters=[
              lidar_odom_config
            ],
        )

    ld = LaunchDescription()
    ld.add_action(realsense)
    ld.add_action(visual_odom)
    ld.add_action(lidar_odom)
    
    
    return ld
