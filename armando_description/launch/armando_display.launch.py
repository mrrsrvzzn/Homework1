from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os


def generate_launch_description():
    
    urdf_name = 'arm.urdf.xacro'
    urdf = os.path.join(
    	get_package_share_directory('armando_description'),
    	'urdf',
    	urdf_name)
    robot_desc = {"robot_description": Command(['xacro ', urdf])}
    	
    rviz_name = 'armando_display_withcam.rviz'
    rviz_config_file = os.path.join( get_package_share_directory('armando_description'), 
    	'rviz', 
    	rviz_name )
    
    nodes_to_start = []
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_desc, {'use_sim_time': True}]
    )
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output="screen",
        parameters=[{'use_sim_time': True}]
    )
    
    rviz_node = Node( package="rviz2", 
        executable="rviz2", 
        name="rviz2", 
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': True}], 
        output="screen" )
    
    
    nodes_to_start = [
        robot_state_publisher_node,  
        joint_state_publisher,
        rviz_node
    ]
        
    return LaunchDescription(nodes_to_start) 
