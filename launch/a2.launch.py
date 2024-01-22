#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_setup(context, *args, **kwargs):

    #Argumments
    worldFileName = 'bugWorld.world'
    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    
    
    #Packages
    pkg_share_dir = get_package_share_directory('assignment2')
    pkg_prefix_dir = get_package_prefix('assignment2')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    #pkg_turtlebot3_cartographer = get_package_share_directory('turtlebot3_cartographer')

    #Paths
    world_file_path = os.path.join(pkg_share_dir, 'worlds', worldFileName)
    
    #Launch Gazebo Server and Client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    #Launch Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot3_gazebo, 'launch', 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    #Launch Cartographer
    cartographer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share_dir, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    #Launch Referee
    referee_node = Node(
        package='assignment2',
        executable='tracker.py',
        name='tracker',
        output='screen',
    )

    return [gzserver_cmd, gzclient_cmd, robot_state_publisher, cartographer, referee_node]


def generate_launch_description():

    #Argumments Declaration
   
    #Launch Description Declaration
    ld = LaunchDescription()

    #Add Actions

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
