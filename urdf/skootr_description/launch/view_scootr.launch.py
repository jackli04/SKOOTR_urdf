import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, FindExecutable


# def generate_launch_description():
#     # Locate the skootr_description package and xacro file
#     pkg_share = get_package_share_path('skootr_description')
#     xacro_file = pkg_share / 'urdf' / 'skootr.xacro'

#     # Process the xacro to generate the robot description parameter
#     robot_description = ParameterValue(
#         Command(['xacro',' ', str(xacro_file)]),
#         value_type=str
#     )

#     # Define the robot_state_publisher node
#     robot_state_publisher_node = Node(
#         package='robot_state_publisher',
#         executable='robot_state_publisher',
#         output='screen',
#         parameters=[{'robot_description': robot_description}]
#     )

#     # Create and return the launch description
#     return LaunchDescription([
#         robot_state_publisher_node
#     ])


def generate_launch_description():
    # Locate your package & xacro
    pkg_share = get_package_share_path('skootr_description')
    xacro_file = pkg_share / 'urdf' / 'skootr.xacro'

    # Convert xacro → robot_description parameter
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', str(xacro_file)
    ])
    robot_description = {'robot_description': robot_description_content}

    # Launch arguments
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        choices=['true', 'false'],
        description='Enable joint_state_publisher_gui'
    )
    rviz_arg = DeclareLaunchArgument(
        'rvizconfig',
        default_value=str(pkg_share / 'rviz' / 'urdf.rviz'),
        description='Absolute path to RViz config file'
    )

    # joint_state_publisher_gui (only if gui=="true")
    jsp_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        parameters=[robot_description],
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    # robot_state_publisher
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )

    return LaunchDescription([
        gui_arg,
        rviz_arg,
        jsp_node,
        rsp_node,
        rviz_node
    ])
