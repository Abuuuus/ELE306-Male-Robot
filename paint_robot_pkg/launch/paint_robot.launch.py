from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

import os, time
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    package_name = 'paint_robot_pkg'

    package_path = os.path.join(
        get_package_share_directory(package_name))
    xacro_file = os.path.join(package_path,
                              'urdf/',
                              'paint_robot.xacro')
    

    
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    my_mobile_robot_description = doc.toxml()
    params = {'robot_description': my_mobile_robot_description, 'use_sim_time': True, 'paused':True}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # node_joint_state_publisher_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui'
    # )

    node_tf = Node(package = "tf2_ros", 
                       executable = "static_transform_publisher",
                       arguments = ["0", "0", "0", "0", "0", "0", "map", "base_link"])

    # node_rviz = Node(
    #     package='rviz2',
    #     namespace='',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d' + os.path.join(get_package_share_directory(package_name), 'config', 'config.rviz')]
        
    # )
    # Include the Gazebo launch file, provided by the gazebo_ros package
    start_gazebo = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            )

# Spawn the robot using gazebo_ros package.
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                    arguments=['-topic', 'robot_description',
                                '-entity', 'my_mobile_robot'],
                    output='screen')

    # joint_state_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner.py",
    #     arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    # )

    # diff_drive_spawner = Node(
    #    package="controller_manager",
    #    executable="spawner.py",
    #    arguments=["diffbot_base_controller", "--controller-manager", "/controller_manager"],
    # ) 

    joint_state_node = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'start','joint_state_broadcaster'],
    output='screen'
    )

    robot_manipulator_node = ExecuteProcess(
    cmd=['ros2', 'control', 'load_controller', '--set-state', 'start','robot_manipulator_controller'],
    output='screen'
    )

    


    return LaunchDescription([
        node_robot_state_publisher,
        # node_joint_state_publisher_gui,
        node_tf,
        # node_rviz,
        start_gazebo,
        spawn_entity,
        # joint_state_broadcaster_spawner,
        # diff_drive_spawner,
        joint_state_node,
        robot_manipulator_node
    ])
