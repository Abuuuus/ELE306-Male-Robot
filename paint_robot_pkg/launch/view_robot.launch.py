import os
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Path to your xacro file
    xacro_file = "/home/rocotics/ros2_ws/src/paint_robot_pkg/urdf/paint_robot.xacro"

    # Convert xacro to URDF
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # Define the static transform publisher node (from world to base_link)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
        output="screen"
    )

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),
        # Static transform (world to base_link)
        static_tf,
        # RViz node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])
