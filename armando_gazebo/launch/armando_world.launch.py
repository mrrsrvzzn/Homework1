from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit

import os

def generate_launch_description():

    # Argomento per abilitare/disabilitare GUI di Gazebo
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Enable Gazebo GUI'
    )

    # Percorso al mondo vuoto di Gazebo
    gz_launch = IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']),
        launch_arguments={
            'gui': LaunchConfiguration('gui'),
            'pause': 'true',
            'gz_args': ['-r ', 'empty.sdf'],
        }.items(),
    )
    
    # lancio lo state publisher e rviz
    armando_display_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare('armando_description'), 
            'launch',
            'armando_display.launch.py'
        ])
    )
    
    # Nodo per spawnare il robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='urdf_spawner',
        arguments=['-topic', '/robot_description', '-entity', 'armando_robot', '-z', '0.0', '-unpause'],
        output='screen'
    )
    
    # Nodo per spawnare il broadcaster
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{'use_sim_time': True}],
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )  

    # Nodo per spawnare il position controller
    position_controller = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{'use_sim_time': True}],
        arguments=["position_controller", "--controller-manager", "/controller_manager"],  
    )
    
    # Nodo per spawnare il trajectory controller
    trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        parameters=[{'use_sim_time': True}],
        arguments=["trajectory_controller", "--controller-manager", "/controller_manager", "--inactive"],  
    )
    
    #Launch the ros2 controllers after the model spawns in Gazebo 
    # Lista di tutti i controller da lanciare
    controllers = [joint_state_broadcaster, position_controller, trajectory_controller]

    # Event handler unico
    delay_controllers = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=controllers
        )
    )
    
    camera_bridge= Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '/camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
            '--ros-args', 
            '-r', '/camera:=/videocamera',
        ]
    )   
    

    return LaunchDescription([
        gui_arg,
        gz_launch,
        armando_display_launch,
        spawn_robot,
        delay_controllers,
        camera_bridge 
    ])

