"""Measured-body Nav2; all commands remain private preview outputs."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,OpaqueFunction,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def start(context):
    share=get_package_share_directory('r1pro_waypoint_nav')
    map_file=os.path.abspath(LaunchConfiguration('map').perform(context))
    if not os.path.isfile(map_file):raise RuntimeError('Map YAML does not exist')
    localization=LaunchConfiguration('localization').perform(context)
    if localization not in ('amcl','external'):raise ValueError('localization must be amcl or external')
    params=os.path.join(share,'config','nav2.yaml');nodes=[]
    if LaunchConfiguration('start_sensors').perform(context).lower()=='true':
        nodes.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(share,'launch','sensors.launch.py')),
            launch_arguments={'start_lidar':LaunchConfiguration('start_lidar'),'publish_tf':'false',
                              'calibration_confirmed':'false','temporal_filter':'true'}.items()))
        nodes.append(Node(package='robot_localization',executable='ekf_node',name='r1pro_ekf',
            parameters=[os.path.join(share,'config','ekf_planar_candidate.yaml'),{'publish_tf':True}],
            remappings=[('odometry/filtered','/r1pro_nav/odom_fused')]))
    specs=[('nav2_map_server','map_server','map_server')]
    if localization=='amcl':specs.append(('nav2_amcl','amcl','amcl'))
    if LaunchConfiguration('start_planning').perform(context).lower() == 'true':
        specs.extend([('nav2_planner','planner_server','planner_server'),('nav2_controller','controller_server','controller_server'),
            ('nav2_velocity_smoother','velocity_smoother','velocity_smoother'),
            ('nav2_collision_monitor','collision_monitor','collision_monitor'),('nav2_bt_navigator','bt_navigator','bt_navigator')])
    for package,exe,name in specs:
        extra=[];remaps=[]
        if name=='map_server':extra=[{'yaml_filename':map_file}]
        if name=='controller_server':remaps=[('cmd_vel','/r1pro_nav/nav2/cmd_vel_raw')]
        if name=='velocity_smoother':remaps=[('cmd_vel','/r1pro_nav/nav2/cmd_vel_raw'),('cmd_vel_smoothed','/r1pro_nav/nav2/cmd_vel_smoothed')]
        if name=='bt_navigator':
            extra=[{'default_nav_to_pose_bt_xml':os.path.join(share,'behavior_trees','navigate_measured_body.xml'),
                    'default_nav_through_poses_bt_xml':os.path.join(share,'behavior_trees','navigate_through_measured_body.xml')}]
            # Humble's navigator creates its own action nodes; use the verified
            # standard action names, not ineffective parent-node remappings.
        nodes.append(Node(package=package,executable=exe,name=name,parameters=[params]+extra,remappings=remaps,output='screen'))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',name='r1pro_nav_lifecycle',
        parameters=[{'autostart':True,'node_names':[s[2] for s in specs]}],output='screen'))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map'),DeclareLaunchArgument('localization',default_value='amcl'),
        DeclareLaunchArgument('start_sensors',default_value='false'),DeclareLaunchArgument('start_lidar',default_value='false'),
        DeclareLaunchArgument('start_planning',default_value='true'),OpaqueFunction(function=start)])
