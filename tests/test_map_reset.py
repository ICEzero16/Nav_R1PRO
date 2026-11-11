import math
from pathlib import Path
import sys
import struct
import zlib
import numpy as np
import pytest
from r1pro_waypoint_nav.map_anchor import MapAnchor,compose,inverse
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'web'))
from map_data import MapStore,png_bytes


def test_reset_preserves_nonzero_translation_rotation_and_later_motion():
    a=MapAnchor();odom=(3.,-2.,.6);old_correction=(1.2,-.8,.4)
    a.observe(compose(old_correction,odom),odom)
    before=a.map_from_odom
    a.begin_reset()
    # Robot moves while a fresh, internally zero-based SLAM graph starts.
    moved=(3.2,-1.9,.7);new_local=(0.,0.,0.)
    a.observe(new_local,moved)
    assert np.allclose(a.map_from_odom,before)
    assert np.allclose(compose(a.map_from_slam,new_local),compose(before,moved))
    a.observe((.1,0.,.02),(3.3,-1.9,.72))
    assert a.map_from_slam is not None


def test_multiple_resets_preserve_frame_not_robot_origin():
    a=MapAnchor();odom=(2.,4.,-.5);a.observe((3.,6.,.1),odom)
    for local in [(10.,-3.,2.),(-4.,5.,-.7),(0.,0.,0.)]:
        before=a.map_from_odom;a.begin_reset();a.observe(local,odom)
        assert np.allclose(a.map_from_odom,before)
        assert np.linalg.norm(compose(a.map_from_slam,local)[:2])>1


def test_no_reset_before_alignment():
    with pytest.raises(ValueError):MapAnchor().begin_reset()


def test_reset_not_ready_until_new_native_map_exists():
    import time
    from types import SimpleNamespace as S
    from r1pro_waypoint_nav.slam_manager import SlamManager
    now=time.monotonic()
    dummy=S(state='mapping',native_map=None,last_map=object(),anchor=S(map_from_slam=(0,0,0)),scan_seen=now,odom_seen=now)
    assert not SlamManager.ready(dummy)
    dummy.native_map=object()
    assert SlamManager.ready(dummy)


def test_se2_inverse():
    for p in [(1.,2.,.3),(-4.,.1,-2.9)]:assert np.allclose(compose(p,inverse(p)),(0,0,0))


def test_png_orientation_and_unknown():
    png=png_bytes(np.array([[0,100],[-1,0]]));offset=8;data=b''
    while offset<len(png):
        length=struct.unpack('!I',png[offset:offset+4])[0];kind=png[offset+4:offset+8]
        if kind==b'IDAT':data+=png[offset+8:offset+8+length]
        offset+=length+12
    assert zlib.decompress(data)==bytes([0,157,246,0,246,0])


def test_map_revisions_and_empty_reset_do_not_change_origin():
    s=MapStore();origin=(3.,-2.,.5)
    for i in range(5):s.put(2,2,.05,origin,[0,100,-1,0] if i<4 else [-1]*4,'live','test')
    assert len(s.images)==3 and s.meta['known_cells']==0
    assert s.meta['origin']==list(origin) and s.meta['revision']==5


def test_bad_map_does_not_replace_last_valid():
    s=MapStore();s.put(1,1,.05,(0,0,0),[0],'live','test')
    with pytest.raises(ValueError):s.put(1,1,.05,(0,0,0),[255],'live','test')
    assert s.meta['revision']==1


def test_saved_map_is_explicit_and_never_live(tmp_path):
    folder=tmp_path/'stationary_trial_test';folder.mkdir()
    (folder/'map.pgm').write_bytes(b'P5\n2 2\n255\n'+bytes([205,254,0,254]))
    (folder/'map.yaml').write_text('image: map.pgm\nresolution: 0.05\norigin: [2, -1, 0.4]\nfree_thresh: 0.25\n')
    s=MapStore();assert s.load_saved(tmp_path)
    assert s.meta['source']=='saved' and s.meta['origin']==[2,-1,.4]
    assert s.meta['known_cells']==3  # Legacy gray=205 stays unknown.
