from collections import deque
import numpy as np
import pytest
from r1pro_waypoint_nav.frame_queue import take_latest_ready
from r1pro_waypoint_nav.knn_backend import native_library,neighbor_means
from r1pro_waypoint_nav.outliers import post_filter_mask


def test_queue_uses_newest_ready_and_discards_older_backlog():
    q=deque([(1,10.),(2,10.1),(3,10.2)])
    chosen,dropped,skipped,_=take_latest_ready(q,lambda m:m*10,lambda m:True,10.3,(LookupError,))
    assert chosen==(3,10.2,30) and dropped==skipped==2 and not q


def test_missing_newest_tf_keeps_newest_for_retry_and_uses_older_ready():
    q=deque([(1,10.),(2,10.1),(3,10.2)])
    def prepare(m):
        if m==3:raise LookupError('waiting for stamped TF')
        return m
    chosen,dropped,skipped,_=take_latest_ready(q,prepare,lambda m:True,10.3,(LookupError,))
    assert chosen[0]==2 and dropped==skipped==1 and list(q)==[(3,10.2)]


def test_all_missing_tf_waits_then_expires_without_output():
    q=deque([(1,10.)])
    def prepare(m):raise LookupError('no TF')
    assert take_latest_ready(q,prepare,lambda m:True,10.1,(LookupError,))[0] is None
    assert len(q)==1
    result=take_latest_ready(q,prepare,lambda m:True,10.3,(LookupError,))
    assert result[0] is None and result[1]==1 and not q


def test_stale_newer_frame_does_not_evict_valid_frame():
    q=deque([(1,10.),(2,10.1)])
    chosen,dropped,_,_=take_latest_ready(q,lambda m:m,lambda m:m==1,10.2,(LookupError,))
    assert chosen[0]==1 and dropped==1 and not q


def test_self_filter_selects_complete_stamped_frame_and_preserves_records():
    import time
    from array import array
    from types import SimpleNamespace as S,MethodType
    from sensor_msgs.msg import PointCloud2,PointField
    from tf2_ros import TransformException
    from r1pro_waypoint_nav.self_filter import SelfFilter
    from r1pro_waypoint_nav.crop_backend import cpu_crop
    records=np.array([[1.,0.,0.,19.],[2.,0.,0.,37.]],dtype='<f4').tobytes()
    def cloud(ns):
        m=PointCloud2();m.header.frame_id='lidar';m.header.stamp.sec=1;m.header.stamp.nanosec=ns
        m.height=1;m.width=2;m.point_step=16;m.row_step=32
        m.fields=[PointField(name=name,offset=i*4,datatype=7,count=1) for i,name in enumerate(('x','y','z','intensity'))]
        m.data=array('B',records);return m
    frames=[cloud(i*100_000_000) for i in range(3)];sent=[];lookups=[]
    def pose(target,source,stamp):
        lookups.append(stamp.nanoseconds)
        if stamp.nanoseconds==1_200_000_000:raise TransformException('newest TF not ready')
        return np.eye(4)
    silent=S(get_subscription_count=lambda:0)
    model=dict(name='body',frame='base_link',source='collision',matrix=np.eye(4),low=np.full(3,-.1),high=np.full(3,.1))
    dummy=S(models=[model],pending=deque((m,time.monotonic()) for m in frames),dropped=0,pose=pose,
        get_clock=lambda:S(now=lambda:S(nanoseconds=1_250_000_000)),padding=.025,cropper=cpu_crop,
        deck_bounds=np.array([]),
        box_pub=silent,marker_pub=silent,filtered=silent,removed=silent,outliers=silent,
        denoised=S(get_subscription_count=lambda:1,publish=sent.append),
        outlier_enabled=False,outlier_k=16,outlier_std=3.,outlier_options=dict(method='range'),crop_backend='cpu',xml_hash='test')
    dummy.prepare_frame=MethodType(SelfFilter.prepare_frame,dummy)
    SelfFilter.tick(dummy)
    assert len(sent)==1 and sent[0].header==frames[1].header
    assert bytes(sent[0].data)==records and sent[0].fields==frames[1].fields
    assert dummy.dropped==1 and dummy.queue_superseded==1 and len(dummy.pending)==1
    assert dummy.pending[0][0] is frames[2]
    assert 1_000_000_000 not in lookups  # Superseded frame is never transformed.
    assert dummy.stats['ready'] and dummy.stats['output_age_ms']==150.
    assert all(v>=0 for v in dummy.stats['stage_ms'].values())


@pytest.mark.skipif(native_library() is None,reason='native library not built on this host')
@pytest.mark.parametrize('k',[2,16,128])
def test_native_exact_distances_and_masks(monkeypatch,k):
    rng=np.random.default_rng(92)
    clouds=[rng.normal(size=(1000,3)),np.zeros((200,3)),
            np.column_stack((np.linspace(-3,3,500),np.zeros((500,2)))),
            np.vstack((rng.normal(0,.01,(500,3))+[2.,0.,0.],rng.normal(size=(200,3))*4))]
    for p in clouds:
        monkeypatch.setenv('R1PRO_KNN_BACKEND','scipy');ref,_=neighbor_means(p,k)
        monkeypatch.setenv('R1PRO_KNN_BACKEND','native');new,_=neighbor_means(p,k)
        np.testing.assert_allclose(new,ref,rtol=2e-14,atol=2e-14)
        for method in ('range','global'):
            options=dict(method=method,min_bin_points=max(64,k+1))
            initial=rng.random(len(p))>.02
            monkeypatch.setenv('R1PRO_KNN_BACKEND','scipy');a=post_filter_mask(p,initial,True,k,3.,**options)
            monkeypatch.setenv('R1PRO_KNN_BACKEND','native');b=post_filter_mask(p,initial,True,k,3.,**options)
            assert np.array_equal(a[0],b[0]) and np.array_equal(a[1],b[1])
