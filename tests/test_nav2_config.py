import json
from pathlib import Path
import numpy as np
import pytest
import yaml
from r1pro_waypoint_nav.nav2_config import navigation_profile,configure_nav2


def geometry():
    return dict(frame='base_link',box_count=46,low=[-.39703,-.33748,0.],high=[.25957,.33748,1.5041],
                description_sha256='test',stamp_ns=1)


def test_footprint_covers_asymmetric_whole_body_and_padded_turn():
    p=navigation_profile(geometry());f=np.array(p['footprint'])
    assert f[:,0].min()<=-.39703 and f[:,0].max()>=.25957
    assert f[:,1].min()<=-.33748 and f[:,1].max()>=.33748
    padded=f+np.sign(f)*p['footprint_padding']
    assert np.linalg.norm(padded,axis=1).max()<=p['global_radius']
    assert p['global_radius']==.57 and p['inflation_radius']==.82


def test_invalid_geometry_rejected():
    g=geometry();g['low'][0]=.1
    with pytest.raises(ValueError):navigation_profile(g)


def test_installed_configuration_dynamic_obstacles_and_low_speed():
    p=Path('/opt/ros/humble/share/nav2_bringup/params/nav2_params.yaml')
    if not p.exists():pytest.skip('Humble configuration is on robot host')
    cfg=configure_nav2(yaml.safe_load(p.read_text()),navigation_profile(geometry()))
    assert cfg['amcl']['ros__parameters']['scan_topic']=='/r1pro_nav/scan_raw'
    for key in ('local_costmap','global_costmap'):
        layer=cfg[key][key]['ros__parameters']['obstacle_layer']['body_filtered_cloud']
        assert layer['topic']=='/r1pro_nav/points_filtered' and layer['data_type']=='PointCloud2'
        assert layer['min_obstacle_height']==.05 and layer['max_obstacle_height']>=1.5
    c=cfg['controller_server']['ros__parameters']
    assert c['odom_topic']=='/r1pro_nav/odom_fused'
    assert c['FollowPath']['max_speed_xy']==.20 and c['FollowPath']['trans_stopped_velocity']<.05
    assert c['progress_checker']['required_movement_radius']<c['FollowPath']['max_speed_xy']*c['progress_checker']['movement_time_allowance']
    assert 'ObstacleFootprint' in c['FollowPath']['critics']
    assert not cfg['planner_server']['ros__parameters']['GridBased']['allow_unknown']
    assert cfg['collision_monitor']['ros__parameters']['cmd_vel_out_topic'].endswith('/cmd_vel_safe')
