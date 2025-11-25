import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    # locate the skootr.xacro in your description package
    desc_share = get_package_share_directory('skootr_description')
    xacro_path = os.path.join(desc_share, 'urdf', 'skootr.xacro')

    # convert XACRO to URDF on the fly
    robot_description = {'robot_description': Command(['xacro ', xacro_path])}

    return LaunchDescription([
        # start Gazebo with ROS factory plugin
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'),
        # publish the robot_state
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[robot_description]
        ),
        # spawn the robot into Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-topic', 'robot_description',
                '-entity', 'skootr',
                '-x', '0', '-y', '0', '-z', '0.1'
            ],
            output='screen'
        ),
    ])
