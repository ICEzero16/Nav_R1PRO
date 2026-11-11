"""Generate Humble Nav2 settings from measured whole-body geometry."""
import copy,json,math


def navigation_profile(geometry):
    if geometry.get('frame')!='base_link' or geometry.get('box_count',0)<1:raise ValueError('Full base_link geometry required')
    low=geometry['low'];high=geometry['high']
    if not all(math.isfinite(v) for v in low+high) or not (low[0]<0<high[0] and low[1]<0<high[1]):raise ValueError('Invalid geometry')
    xmin,ymin=[math.floor(v*100)/100 for v in low[:2]]
    xmax,ymax=[math.ceil(v*100)/100 for v in high[:2]]
    pad=.03;polygon=[[xmax,ymax],[xmin,ymax],[xmin,ymin],[xmax,ymin]]
    radius=math.ceil(max(math.hypot(x+math.copysign(pad,x),y+math.copysign(pad,y)) for x,y in polygon)*100)/100
    return dict(footprint=polygon,footprint_padding=pad,global_radius=radius,
        inflation_radius=round(radius+.25,2),cost_scaling_factor=3.,
        min_obstacle_height=.05,max_obstacle_height=math.ceil((high[2]+.1)*10)/10,
        description_sha256=geometry['description_sha256'],geometry_stamp_ns=geometry['stamp_ns'],
        posture_policy='fixed_measured_envelope',linear_speed=.20,yaw_speed=.12)


def configure_nav2(defaults,profile):
    nav=copy.deepcopy(defaults)
    def replace(obj):
        if isinstance(obj,dict):
            for key,value in list(obj.items()):
                if key=='use_sim_time':obj[key]=False
                elif key in ('base_frame_id','robot_base_frame'):obj[key]='base_link'
                elif key=='odom_topic':obj[key]='/r1pro_nav/odom_fused'
                else:replace(value)
        elif isinstance(obj,list):
            for value in obj:replace(value)
    replace(nav)
    amcl=nav['amcl']['ros__parameters'];amcl.update(robot_model_type='nav2_amcl::OmniMotionModel',
        scan_topic='/r1pro_nav/scan_raw',laser_min_range=.3,laser_max_range=12.,
        update_min_d=.05,update_min_a=.05,transform_tolerance=.3,min_particles=300,max_particles=1200,
        set_initial_pose=False,always_reset_initial_pose=True)
    c=nav['controller_server']['ros__parameters'];c.update(controller_frequency=10.,odom_topic='/r1pro_nav/odom_fused',
        min_x_velocity_threshold=.001,min_y_velocity_threshold=.001,min_theta_velocity_threshold=.001)
    c['progress_checker'].update(plugin='nav2_controller::PoseProgressChecker',required_movement_radius=.05,
                                 required_movement_angle=.10,movement_time_allowance=20.)
    c['general_goal_checker'].update(xy_goal_tolerance=.05,yaw_goal_tolerance=.10)
    speed=profile['linear_speed'];yaw_speed=profile['yaw_speed']
    accel=.12;decel=.20
    f=c['FollowPath'];f.update(min_vel_x=-speed,max_vel_x=speed,min_vel_y=-speed,max_vel_y=speed,
        max_speed_xy=speed,max_vel_theta=yaw_speed,min_speed_xy=0.,min_speed_theta=0.,
        acc_lim_x=accel,acc_lim_y=accel,acc_lim_theta=.10,decel_lim_x=-decel,decel_lim_y=-decel,decel_lim_theta=-.10,
        vx_samples=5,vy_samples=5,vtheta_samples=11,sim_time=2.,linear_granularity=.025,angular_granularity=.025,
        trans_stopped_velocity=.01,xy_goal_tolerance=.05,debug_trajectory_details=False,
        critics=['RotateToGoal','Oscillation','ObstacleFootprint','GoalOrientation','PathDist','GoalDist'])
    # Orient toward the final goal during translation. All sampled rotations
    # still pass the footprint collision critic before orientation scoring.
    f['GoalOrientation.class']='r1pro_nav2_critics::GoalOrientationCritic'
    f['GoalOrientation.scale']=8.0
    for key in tuple(f):
        if key.startswith('Twirling.'):
            del f[key]
    for key in tuple(f):
        if key.startswith(('PathAlign.', 'GoalAlign.')):
            del f[key]
    f.pop('BaseObstacle.scale',None);f['ObstacleFootprint.scale']=.02
    for name in ('local_costmap','global_costmap'):
        p=nav[name][name]['ros__parameters'];p.pop('voxel_layer',None);p.pop('robot_radius',None)
        p.update(footprint=json.dumps(profile['footprint']),footprint_padding=profile['footprint_padding'],
                 transform_tolerance=.3,resolution=.05,track_unknown_space=True,
                 plugins=['static_layer','obstacle_layer','inflation_layer'],always_send_full_costmap=False)
        if name=='global_costmap':
            p.pop('footprint');p['footprint_padding']=0.;p['robot_radius']=profile['global_radius']
            p.update(update_frequency=2.,publish_frequency=1.)
        else:p.update(width=4,height=4,rolling_window=True,update_frequency=10.,publish_frequency=2.)
        p['static_layer']=dict(plugin='nav2_costmap_2d::StaticLayer',map_topic='/map',map_subscribe_transient_local=True)
        p['obstacle_layer']=dict(plugin='nav2_costmap_2d::ObstacleLayer',enabled=True,observation_sources='body_filtered_cloud',
            body_filtered_cloud=dict(topic='/r1pro_nav/points_filtered',data_type='PointCloud2',marking=True,clearing=True,
                min_obstacle_height=profile['min_obstacle_height'],max_obstacle_height=profile['max_obstacle_height'],
                obstacle_min_range=.0,obstacle_max_range=4.,raytrace_min_range=.0,raytrace_max_range=5.,
                observation_persistence=0.,expected_update_rate=.6))
        p['inflation_layer']=dict(plugin='nav2_costmap_2d::InflationLayer',inflation_radius=profile['inflation_radius'],
            cost_scaling_factor=profile['cost_scaling_factor'],inflate_unknown=False,inflate_around_unknown=False)
    nav['planner_server']['ros__parameters'].update(expected_planner_frequency=1.,planner_plugins=['GridBased'],
        GridBased=dict(plugin='nav2_smac_planner/SmacPlanner2D',tolerance=.10,allow_unknown=False,max_iterations=100000,
                       max_on_approach_iterations=1000,max_planning_time=1.,cost_travel_multiplier=2.,use_final_approach_orientation=False))
    nav['velocity_smoother']['ros__parameters']=dict(use_sim_time=False,smoothing_frequency=20.,scale_velocities=True,
        feedback='OPEN_LOOP',max_velocity=[speed,speed,yaw_speed],min_velocity=[-speed,-speed,-yaw_speed],
        max_accel=[accel,accel,.10],max_decel=[-decel,-decel,-.10],odom_topic='/r1pro_nav/odom_fused',
        odom_duration=.1,deadband_velocity=[0.,0.,0.],velocity_timeout=.3)
    polygon=[[x+math.copysign(.11,x),y+math.copysign(.11,y)] for x,y in profile['footprint']]
    nav['collision_monitor']={'ros__parameters':dict(use_sim_time=False,base_frame_id='base_link',odom_frame_id='odom',
        cmd_vel_in_topic='/r1pro_nav/nav2/cmd_vel_smoothed',cmd_vel_out_topic='/r1pro_nav/nav2/cmd_vel_safe',
        transform_tolerance=.3,source_timeout=.6,base_shift_correction=True,stop_pub_timeout=2.,
        polygons=['BodyStop'],BodyStop=dict(type='polygon',points=[v for p in polygon for v in p],
            action_type='stop',max_points=3,visualize=True,polygon_pub_topic='/r1pro_nav/nav2/stop_polygon',enabled=True),
        observation_sources=['cloud'],cloud=dict(type='pointcloud',topic='/r1pro_nav/points_filtered',enabled=True,
            min_height=profile['min_obstacle_height'],max_height=profile['max_obstacle_height']))}
    return nav
