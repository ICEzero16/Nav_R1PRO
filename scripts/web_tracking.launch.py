"""Add path tracking to an existing localization and global planner session."""
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('r1pro_waypoint_nav')) / 'config/nav2.yaml')
    raw = '/r1pro_nav/nav2/cmd_vel_raw'
    smooth = '/r1pro_nav/nav2/cmd_vel_smoothed'
    nodes = [
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             parameters=[config], remappings=[('cmd_vel',raw)]),
        Node(package='nav2_velocity_smoother', executable='velocity_smoother', name='velocity_smoother',
             parameters=[config], remappings=[('cmd_vel',raw),('cmd_vel_smoothed',smooth)]),
        Node(package='nav2_collision_monitor', executable='collision_monitor', name='collision_monitor',
             parameters=[config]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager', name='web_tracking_manager',
             parameters=[{'autostart':True,'node_names':['controller_server','velocity_smoother','collision_monitor']}])
    ]
    return LaunchDescription(nodes)
