import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    if LaunchConfiguration('calibration_confirmed').perform(context).lower()!='true':
        raise RuntimeError('Navigation preview requires validated calibration and running odom TF')
    map_file=os.path.abspath(LaunchConfiguration('map').perform(context))
    if not os.path.isfile(map_file):raise RuntimeError('Existing occupancy map YAML required')
    share=get_package_share_directory('r1pro_waypoint_nav');params=os.path.join(share,'config','nav2.yaml')
    nodes=[]
    specs=[('nav2_map_server','map_server','map_server'),('nav2_amcl','amcl','amcl'),
           ('nav2_controller','controller_server','controller_server'),('nav2_planner','planner_server','planner_server'),
           ('nav2_behaviors','behavior_server','behavior_server'),('nav2_bt_navigator','bt_navigator','bt_navigator')]
    # Every velocity producer is routed to a private, non-hardware topic.
    remaps=[('cmd_vel','/r1pro_nav/cmd_vel'),('navigate_to_pose','/r1pro_nav/navigate_to_pose'),
            ('navigate_through_poses','/r1pro_nav/navigate_through_poses')]
    for package,exe,name in specs:
        extra=[{'yaml_filename':map_file}] if name=='map_server' else []
        nodes.append(Node(package=package,executable=exe,name=name,parameters=[params]+extra,remappings=remaps,output='screen'))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='r1pro_nav_lifecycle',
        parameters=[{'autostart':True,'node_names':[s[2] for s in specs]}],output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map'),DeclareLaunchArgument('calibration_confirmed',default_value='false'),OpaqueFunction(function=start)])
