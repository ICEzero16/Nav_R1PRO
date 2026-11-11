import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    if LaunchConfiguration('calibration_confirmed').perform(context).lower()!='true':
        raise RuntimeError('First validate wheel velocity units/axes and lidar extrinsics; calibration_confirmed is false')
    share=get_package_share_directory('r1pro_waypoint_nav')
    return [Node(package='slam_toolbox',executable='async_slam_toolbox_node',name='slam_toolbox',
                 parameters=[os.path.join(share,'config','slam.yaml')],output='screen')]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('calibration_confirmed',default_value='false'),OpaqueFunction(function=start)])
