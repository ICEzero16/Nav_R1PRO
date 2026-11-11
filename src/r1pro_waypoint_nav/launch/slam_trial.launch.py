"""Experimental mapping only. No Nav2 controller or hardware command bridge."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    share=get_package_share_directory('r1pro_waypoint_nav')
    return LaunchDescription([
        DeclareLaunchArgument('start_lidar',default_value='false'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','sensors.launch.py')),
            launch_arguments={'start_lidar':LaunchConfiguration('start_lidar'),'publish_tf':'false',
                              'calibration_confirmed':'false','temporal_filter':'true'}.items()),
        Node(package='robot_localization',executable='ekf_node',name='r1pro_ekf',output='screen',
            parameters=[os.path.join(share,'config','ekf_planar_candidate.yaml'),{'publish_tf':True,'use_sim_time':False}],
            remappings=[('odometry/filtered','/r1pro_nav/odom_fused')]),
        Node(package='r1pro_waypoint_nav',executable='slam_manager',output='screen',
            parameters=[{'slam_config':os.path.join(share,'config','slam.yaml')}])])
