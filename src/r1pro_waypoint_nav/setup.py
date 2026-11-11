from setuptools import setup
from glob import glob
setup(name='r1pro_waypoint_nav', version='0.1.0', packages=['r1pro_waypoint_nav'],
      package_data={'r1pro_waypoint_nav':['libcrop_cuda.so','libknn_mean.so']},
      data_files=[('share/ament_index/resource_index/packages', ['resource/r1pro_waypoint_nav']),
                  ('share/r1pro_waypoint_nav', ['package.xml']),
                  ('share/r1pro_waypoint_nav/launch', glob('launch/*.py')),
                  ('share/r1pro_waypoint_nav/config', glob('config/*.yaml')+glob('config/*.json')),
                  ('share/r1pro_waypoint_nav/behavior_trees', glob('behavior_trees/*.xml'))],
      entry_points={'console_scripts': ['adapters = r1pro_waypoint_nav.adapters:main',
                                       'chassis_imu = r1pro_waypoint_nav.chassis_imu:main',
                                       'self_filter = r1pro_waypoint_nav.self_filter:main',
                                       'temporal_scan = r1pro_waypoint_nav.temporal_scan:main',
                                       'slam_manager = r1pro_waypoint_nav.slam_manager:main',
                                       'waypoint = r1pro_waypoint_nav.waypoint:main']})
