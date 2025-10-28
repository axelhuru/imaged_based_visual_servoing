# Copyright (C) 2021 Bosch LLC CR, North America. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

from ament_index_python.packages import get_package_share_directory
import launch
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros
import os
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

use_real = LaunchConfiguration('use_real', default='false')

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    pkg_share = launch_ros.substitutions.FindPackageShare(package='panda_ros2_gazebo').find('panda_ros2_gazebo')
    default_model_path = os.path.join(pkg_share, "description", "models")
    default_urdf_path = os.path.join(default_model_path, "panda", "panda.urdf.xacro")
    default_rviz_config_path = os.path.join(pkg_share, 'rviz/rviz.rviz')
    
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={"verbose": "true", "world": os.path.join(pkg_share, "description", "worlds", "panda.world")
            }.items(),
    )
    
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        ),
        launch_arguments={"verbose": "true"}.items(),
    )
    
    robot_description_content = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]), value_type=str)
    robot_description = {"robot_description": robot_description_content}

    velocity_controller_config = PathJoinSubstitution(
        [FindPackageShare("panda_ros2_gazebo"), "config", "cartesian_velocity_controller.yaml"]
    )

    control_node = launch_ros.actions.Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, velocity_controller_config],
        output="both",
        remappings=[("~/robot_description", "/robot_description")],
        condition=IfCondition(use_real),
    )

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='panda',  
        output='both',
        parameters=[{
            'use_sim_time': use_sim_time, 
            'robot_description': robot_description_content, 
            'publish_frequency': 200.0
        }]
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        condition=IfCondition(gui),
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    spawn_entity = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-topic", "/panda/robot_description", "-entity", "panda"],  
        output="screen",
    )

    spawn_ball = launch_ros.actions.Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity", "red_ball", 
            "-file", os.path.join(default_model_path, "red_ball/model.sdf"), 
            "-x", "0.5", 
            "-y", "0.0", 
            "-z", "0.2"
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/panda/controller_manager"],
        output="screen",
    )

    cartesian_velocity_controller_spawner = launch_ros.actions.Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cartesian_velocity_controller", "-c", "/panda/controller_manager"],
        output="screen",
    )

    ball_detector_node = launch_ros.actions.Node(
        package="visual_servoing",
        executable="ball_detector",
        name="ball_detector",
        output="screen",
    )

    ibvs_controller_node = launch_ros.actions.Node(
        package="ibvs_controller",
        executable="ibvs_controller",
        name="ibvs_controller",
        output="screen",
    )

    gazebo_model_path = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value=[default_model_path])
    gazebo_media_path = SetEnvironmentVariable(name='GAZEBO_MEDIA_PATH', value=[default_model_path])
    
    delay_spawn_ball_after_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[spawn_ball],
        )
    )

    delay_rviz_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        )
    )

    delay_cartesian_velocity_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[cartesian_velocity_controller_spawner],
        )
    )

    delay_visual_servoing_after_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=cartesian_velocity_controller_spawner,
            on_exit=[ball_detector_node, ibvs_controller_node],
        )
    )
    
    delay_joint_state_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
    launch_description = launch.LaunchDescription([
        gazebo_model_path,
        gazebo_media_path,
        launch.actions.DeclareLaunchArgument(
            'use_real',
            default_value='false',
            description='Use real hardware (false for sim)'),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        launch.actions.DeclareLaunchArgument(
            name='gui', 
            default_value='true',
            description='Flag to enable RViz'),
        launch.actions.DeclareLaunchArgument(
            name='model', 
            default_value=default_urdf_path,
            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(
            name='rvizconfig', 
            default_value=default_rviz_config_path,
            description='Absolute path to rviz config file'),
        control_node,
        robot_state_publisher_node,
        gzclient,
        gzserver,
        spawn_entity,
        delay_spawn_ball_after_entity,
        delay_joint_state_broadcaster_after_spawn,
        delay_rviz_after_joint_state_broadcaster,
        delay_cartesian_velocity_controller_after_joint_state_broadcaster,
        delay_visual_servoing_after_controller,
    ])

    return launch_description
