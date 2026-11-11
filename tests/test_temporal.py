import math
import numpy as np
import pytest
from r1pro_waypoint_nav.temporal_core import TemporalConfig,TemporalFilter

N=720
ANGLES=-math.pi+np.arange(N)*2*math.pi/N


def scene(pose=(0.,0.,0.), circle=None):
    """Analytic rays against a square room and optional round moving object."""
    origin=np.array(pose[:2]);theta=ANGLES+pose[2]
    direction=np.column_stack((np.cos(theta),np.sin(theta)))
    with np.errstate(divide='ignore',invalid='ignore'):
        bounds=np.where(direction>0,5.,-5.)
        distances=(bounds-origin)/direction
        distances=np.where(np.abs(direction)<1e-12,np.inf,distances)
    ranges=distances.min(axis=1)
    labels=np.zeros(N,dtype=int)
    if circle is not None:
        center=np.asarray(circle[:2]);radius=circle[2]
        delta=center-origin
        along=direction@delta
        disc=radius**2-(delta@delta-along**2)
        hit=along-np.sqrt(np.maximum(disc,0))
        mask=(disc>=0)&(hit>.3)&(hit<ranges)
        ranges[mask]=hit[mask];labels[mask]=1
    return ranges,labels


def run(f,r,stamp,pose=(0.,0.,0.)):
    return f.process(r,-math.pi,2*math.pi/N,.3,12.,stamp,pose)


def test_static_room_confirms_after_span_not_point_count():
    f=TemporalFilter();r,_=scene()
    for i in range(4):
        keep,_=run(f,r,1+i*.1)
        assert not keep.any()
    keep,stats=run(f,r,1.4)
    assert keep.all()
    assert stats['moving_candidates']==0


def test_same_frame_dense_object_does_not_self_confirm():
    f=TemporalFilter();r,_=scene(circle=(2.,0.,.3))
    keep,_=run(f,r,1.)
    assert not keep.any()
    with pytest.raises(ValueError):run(f,r,1.)
    assert len(f.history)==1


def test_single_frame_noise_rejected_without_corrupting_walls():
    f=TemporalFilter();r,_=scene()
    for i in range(6):run(f,r,1+i*.1)
    noisy=r.copy();noisy[300]=1.
    keep,_=run(f,noisy,1.6)
    assert not keep[300]
    assert keep.sum()>=N-4
    keep,_=run(f,r,1.7)
    assert keep[300]


def test_motion_compensation_preserves_stationary_room():
    f=TemporalFilter()
    for i in range(20):
        pose=(i*.025,0.,i*.01);r,_=scene(pose)
        keep,_=run(f,r,1+i*.1,pose)
        if i>6:assert keep.sum()/N>.94


def test_moving_circle_is_rejected_static_room_retained():
    f=TemporalFilter();moving_total=kept_moving=wall_total=kept_wall=0
    for i in range(40):
        r,label=scene(circle=(2.,-1.2+.06*i,.25))
        keep,stats=run(f,r,1+i*.1)
        if i>=8:
            moving_total+=sum(label==1);kept_moving+=sum(keep&(label==1))
            wall_total+=sum(label==0);kept_wall+=sum(keep&(label==0))
    assert kept_moving/moving_total<.05
    assert kept_wall/wall_total>.95


def test_static_compact_obstacle_is_not_permanently_removed():
    f=TemporalFilter();r,label=scene(circle=(2.,0.,.2))
    for i in range(10):keep,stats=run(f,r,1+i*.1)
    assert keep[label==1].all()
    assert stats['moving_candidates']==0


def test_previously_stationary_object_starts_moving():
    f=TemporalFilter()
    for i in range(12):
        r,label=scene(circle=(2.,-.5,.25));keep,_=run(f,r,1+i*.1)
    assert keep[label==1].all()
    for i in range(1,18):
        r,label=scene(circle=(2.,-.5+i*.06,.25));keep,_=run(f,r,2.1+i*.1)
        if i>=7:assert keep[label==1].mean()<.05


def test_moving_object_can_become_stationary_again():
    f=TemporalFilter()
    for i in range(12):
        r,label=scene(circle=(2.,-.6+i*.06,.25));run(f,r,1+i*.1)
    for i in range(1,35):
        r,label=scene(circle=(2.,.06,.25));keep,_=run(f,r,2.1+i*.1)
    assert keep[label==1].all()


def test_sparse_returns_can_confirm_across_unobserved_frames():
    f=TemporalFilter();r,_=scene()
    for i in range(7):
        data=r.copy() if i%2==0 else np.full(N,np.nan)
        keep,_=run(f,data,1+i*.1)
    assert keep.all()


def test_empty_current_scan_never_publishes_historical_obstacles():
    f=TemporalFilter();r,_=scene()
    for i in range(8):run(f,r,1+i*.1)
    keep,_=run(f,np.full(N,np.inf),1.8)
    assert not keep.any()


def test_spikes_do_not_fragment_static_surface_into_moving_clusters():
    f=TemporalFilter();rng=np.random.default_rng(72)
    for i in range(30):
        r,label=scene(circle=(2.,0.,.2))
        noise=rng.choice(N,25,replace=False);r[noise]=rng.uniform(.4,1.2,25)
        keep,stats=run(f,r,1+i*.1)
        if i>=10:
            static=np.ones(N,dtype=bool);static[noise]=False
            assert keep[static].mean()>.98
            assert stats['moving_candidates']==0


def test_egomotion_does_not_label_static_compact_obstacle_moving():
    f=TemporalFilter()
    for i in range(20):
        pose=(i*.015,i*.01,i*.01);r,label=scene(pose,circle=(2.,0.,.2))
        keep,stats=run(f,r,1+i*.1,pose)
        if i>7:assert keep[label==1].mean()>.85


def test_gap_and_odom_reset_require_new_confirmation():
    f=TemporalFilter();r,_=scene()
    for i in range(6):run(f,r,1+i*.1)
    keep,stats=run(f,r,2.2)
    assert not keep.any();assert stats['reset_reason']=='scan_gap'
    for i in range(1,6):run(f,r,2.2+i*.1)
    keep,stats=run(f,r,2.8,(3.,0.,0.))
    assert not keep.any();assert stats['reset_reason']=='odom_jump'


def test_unknown_and_occlusion_do_not_clear_history():
    f=TemporalFilter();r,_=scene()
    run(f,r,1.)
    unknown=np.full(N,np.inf)
    run(f,unknown,1.1)
    assert f.history[0]['active'].all()
    occluder=np.full(N,1.)
    run(f,occluder,1.2)
    assert f.history[0]['active'].all()


def test_real_farther_returns_invalidate_old_object_votes():
    f=TemporalFilter();r,label=scene(circle=(2.,0.,.25))
    run(f,r,1.)
    r,_=scene();_,stats=run(f,r,1.1)
    assert stats['invalidated_history_points']>=sum(label==1)-2


def test_disappeared_object_never_replayed_and_memory_bounded():
    f=TemporalFilter();r,label=scene(circle=(2.,0.,.2))
    for i in range(8):run(f,r,1+i*.1)
    r,_=scene();keep,_=run(f,r,1.8)
    assert np.all(r[keep]>4.)
    for i in range(100):run(f,r,2+i*.1)
    assert len(f.history)<=f.cfg.max_frames
    assert len(f.tracks)<=f.cfg.max_tracks


def test_invalid_configuration_and_measurements():
    with pytest.raises(ValueError):TemporalConfig(min_hits=20)
    with pytest.raises(ValueError):TemporalConfig(window_sec=-1.)
    f=TemporalFilter()
    for bad_pose in [(0.,0.,float('nan')),(0.,0.)]:
        with pytest.raises(ValueError):run(f,np.ones(N),1.,bad_pose)
    keep,_=run(f,np.full(N,np.nan),1.)
    assert not keep.any()


def test_ros_mask_preserves_unknown_and_message_geometry():
    from sensor_msgs.msg import LaserScan
    from r1pro_waypoint_nav.temporal_scan import masked_scan
    msg=LaserScan();msg.ranges=[1.,2.,float('inf')];msg.header.frame_id='livox_frame';msg.header.stamp.sec=5
    msg.angle_min=-1.;msg.angle_increment=.1;msg.range_min=.3;msg.range_max=12.
    out=masked_scan(msg,np.array([True,False,False]))
    assert out.ranges[0]==1. and math.isnan(out.ranges[1]) and math.isnan(out.ranges[2])
    assert out.header==msg.header and out.angle_increment==msg.angle_increment
    assert msg.ranges[1]==2. and math.isinf(msg.ranges[2])


def test_node_missing_tf_drops_without_publishing_or_using_latest():
    import time
    from collections import deque
    from types import SimpleNamespace as S
    from sensor_msgs.msg import LaserScan
    from tf2_ros import TransformException
    from r1pro_waypoint_nav.temporal_scan import TemporalScan
    msg=LaserScan();msg.header.stamp.sec=1;msg.header.frame_id='livox_frame'
    calls=[]
    def lookup(target,source,stamp):
        calls.append(stamp.nanoseconds)
        raise TransformException('no transform')
    dummy=S(pending=deque([(msg,time.monotonic()-1)]),dropped=0,stats={},fixed_frame='odom',
            buffer=S(lookup_transform=lookup),get_clock=lambda:S(now=lambda:S(nanoseconds=1_100_000_000)))
    TemporalScan.tick(dummy)
    assert calls==[1_000_000_000] and dummy.dropped==1 and not dummy.pending
    assert dummy.stats['reason']=='missing_timestamped_odom_tf'


def test_node_stale_scan_does_not_lookup_tf_or_publish():
    import time
    from collections import deque
    from types import SimpleNamespace as S
    from sensor_msgs.msg import LaserScan
    from r1pro_waypoint_nav.temporal_scan import TemporalScan
    msg=LaserScan();msg.header.stamp.sec=1
    dummy=S(pending=deque([(msg,time.monotonic())]),dropped=0,stats={},
            get_clock=lambda:S(now=lambda:S(nanoseconds=3_000_000_000)))
    TemporalScan.tick(dummy)
    assert dummy.dropped==1 and dummy.stats['reason']=='stale_or_invalid_scan'


def test_ros_tick_publishes_only_current_confirmed_beams_without_ros_start():
    import time
    from collections import deque
    from types import SimpleNamespace as S
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import TransformStamped
    from r1pro_waypoint_nav.temporal_scan import TemporalScan
    transform=TransformStamped();transform.transform.rotation.w=1.
    sent=[];clock=[1_050_000_000]
    dummy=S(pending=deque(),dropped=0,stats={},fixed_frame='odom',filter=TemporalFilter(),geometry=None,
            buffer=S(lookup_transform=lambda *args:transform),last_ok=0.,
            get_clock=lambda:S(now=lambda:S(nanoseconds=clock[0])),pub=S(publish=sent.append),
            rejected=S(get_subscription_count=lambda:0))
    ranges,_=scene()
    for i in range(7):
        msg=LaserScan();msg.header.stamp.sec=1;msg.header.stamp.nanosec=i*100_000_000
        msg.header.frame_id='livox_frame';msg.angle_min=-math.pi;msg.angle_increment=2*math.pi/N
        msg.range_min=.3;msg.range_max=12.;msg.ranges=ranges.tolist()
        clock[0]=1_050_000_000+i*100_000_000
        dummy.pending.append((msg,time.monotonic()));TemporalScan.tick(dummy)
    assert len(sent)==7 and np.isnan(sent[0].ranges).all()
    assert np.isfinite(sent[-1].ranges).all() and sent[-1].header==msg.header
    assert dummy.stats['ready'] and dummy.dropped==0
