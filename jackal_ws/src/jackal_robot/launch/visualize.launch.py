
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

import os

def generate_launch_description():

    robot_description_command_arg = DeclareLaunchArgument(
        'robot_description_command',
        default_value=[
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('jackal_description'), 'urdf', 'jackal.urdf.xacro']
            )
        ]
    )

    robot_description_content = ParameterValue(
        Command(LaunchConfiguration('robot_description_command')),
        value_type=str
    )

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])
    pkg_dir = '/home/administrator/jackal_ws/src/jackal_robot'
    ld = LaunchDescription()
    ld.add_action(robot_description_command_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(
        Node(
          package='rviz2', executable='rviz2', name="rviz2", output='screen',
          arguments=['-d', [os.path.join(pkg_dir, 'config', 'jackal.rviz')]]
        ),
    )
    return ld
