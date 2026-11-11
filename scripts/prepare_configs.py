"""Derive configs from this machine's installed Humble versions; preserve defaults."""
from pathlib import Path
import yaml
root=Path(__file__).resolve().parents[1]/'src/r1pro_waypoint_nav/config'
slam=yaml.safe_load(Path('/opt/ros/humble/share/slam_toolbox/config/mapper_params_online_async.yaml').read_text())
slam['slam_toolbox']['ros__parameters'].update(base_frame='base_link',odom_frame='odom',map_frame='map',
    scan_topic='/r1pro_nav/scan',minimum_travel_distance=.10,minimum_travel_heading=.10,
    minimum_time_interval=.1,max_laser_range=12.0,transform_timeout=.3)
(root/'slam.yaml').write_text(yaml.safe_dump(slam,sort_keys=False))
nav=yaml.safe_load(Path('/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml').read_text())
def replace(v):
    if isinstance(v,dict):
        for k,a in list(v.items()):
            if k in ('base_frame_id','robot_base_frame'):v[k]='base_link'
            elif k=='use_sim_time':v[k]=False
            elif k=='odom_topic':v[k]='/r1pro_nav/odom'
            elif k=='scan_topic' or (k=='topic' and a=='/scan'):v[k]='/r1pro_nav/scan'
            else:replace(a)
    elif isinstance(v,list):
        for a in v:replace(a)
replace(nav)
nav['amcl']['ros__parameters']['robot_model_type']='nav2_amcl::OmniMotionModel'
c=nav['controller_server']['ros__parameters'];c['controller_frequency']=10.0
c['min_x_velocity_threshold']=.001;c['min_y_velocity_threshold']=.001;c['min_theta_velocity_threshold']=.001
f=c['FollowPath'];f.update(min_vel_x=-.05,max_vel_x=.05,min_vel_y=-.05,max_vel_y=.05,
    max_vel_theta=.15,max_speed_xy=.05,min_speed_xy=0.0,min_speed_theta=0.0,
    acc_lim_x=.1,acc_lim_y=.1,acc_lim_theta=.2,decel_lim_x=-.1,decel_lim_y=-.1,decel_lim_theta=-.2,
    vx_samples=7,vy_samples=7,vtheta_samples=15)
for costmap in ('local_costmap','global_costmap'):
    p=nav[costmap][costmap]['ros__parameters']
    p.pop('robot_radius',None)
    # Candidate envelope only: verify whole-body footprint before any hardware integration.
    p['footprint']='[[0.45,0.40],[0.45,-0.40],[-0.45,-0.40],[-0.45,0.40]]'
    p['footprint_padding']=.05
nav['behavior_server']['ros__parameters'].update(max_rotational_vel=.15,min_rotational_vel=.05,rotational_acc_lim=.2)
(root/'nav2.yaml').write_text(yaml.safe_dump(nav,sort_keys=False))
print('Generated Humble configs:',root)
