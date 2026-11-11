import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=get_package_share_directory('r1pro_waypoint_nav')
    livox=get_package_share_directory('livox_ros_driver2')
    return LaunchDescription([
        DeclareLaunchArgument('start_lidar',default_value='false',description='Start only if no Livox driver exists'),
        DeclareLaunchArgument('calibration_confirmed',default_value='false'),
        DeclareLaunchArgument('publish_tf',default_value='false'),
        DeclareLaunchArgument('temporal_filter',default_value='false',description='Requires timestamped odom TF; mapping only'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(livox,'launch_ROS2','msg_MID360_launch.py')),
                                 condition=IfCondition(LaunchConfiguration('start_lidar'))),
        Node(package='r1pro_waypoint_nav',executable='adapters',output='screen',parameters=[
            os.path.join(share,'config','adapters.yaml'),
            {'calibration_confirmed':ParameterValue(LaunchConfiguration('calibration_confirmed'),value_type=bool),
             'publish_tf':ParameterValue(LaunchConfiguration('publish_tf'),value_type=bool)}]),
        Node(package='r1pro_waypoint_nav',executable='self_filter',output='screen',parameters=[os.path.join(share,'config','self_filter.yaml')]),
        Node(package='r1pro_waypoint_nav',executable='chassis_imu',output='screen',parameters=[os.path.join(share,'config','chassis_imu.yaml')]),
        Node(package='pointcloud_to_laserscan',executable='pointcloud_to_laserscan_node',name='r1pro_scan',
             parameters=[os.path.join(share,'config','scan.yaml')],
             remappings=[('cloud_in','/r1pro_nav/points_denoised'),('scan',PythonExpression([
                 "'/r1pro_nav/scan_raw' if '",LaunchConfiguration('temporal_filter'),"'.lower() == 'true' else '/r1pro_nav/scan'"]))],output='screen'),
        Node(package='r1pro_waypoint_nav',executable='temporal_scan',output='screen',
             condition=IfCondition(LaunchConfiguration('temporal_filter')),
             parameters=[os.path.join(share,'config','temporal_scan.yaml')])])
