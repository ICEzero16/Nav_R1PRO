import math
from pathlib import Path
import struct
import tempfile
from types import SimpleNamespace as S
import unittest
import numpy as np
from r1pro_waypoint_nav.box_geometry import build_boxes,box_mask,origin,unpack_cloud,stl_bounds,invalid_points
import xml.etree.ElementTree as E


class BoxTests(unittest.TestCase):
    def test_invalid_points_multidimensional(self):
        points=np.array([[0,0,0],[1,2,3],[float('nan'),1,2],[float('inf'),0,0],[.01,0,0]])
        self.assertEqual(invalid_points(points).tolist(),[True,False,True,True,True])
    def test_rotated_box_preserves_nearby_obstacle(self):
        t=origin(E.fromstring('<origin xyz="1 2 0" rpy="0 0 1.57079632679"/>'))
        pts=np.array([[1,2.4,0],[1.3,2,0],[1,3,0]])
        mask,_=box_mask(pts,t,np.array([-.5,-.1,-.1]),np.array([.5,.1,.1]))
        self.assertEqual(mask.tolist(),[True,False,False])
    def test_dynamic_pose_changes_selection(self):
        pts=np.array([[0,0,0],[1,0,0]])
        a=np.eye(4);b=np.eye(4);b[0,3]=1
        self.assertEqual(box_mask(pts,a,np.full(3,-.1),np.full(3,.1))[0].tolist(),[True,False])
        self.assertEqual(box_mask(pts,b,np.full(3,-.1),np.full(3,.1))[0].tolist(),[False,True])
    def test_collision_origin_and_visual_not_double_counted(self):
        xml='<robot><link name="a"><collision><origin xyz="1 2 3"/><geometry><box size="2 4 6"/></geometry></collision><visual><geometry><box size="99 99 99"/></geometry></visual></link></robot>'
        b=build_boxes(xml,lambda p:p);self.assertEqual(len(b),1);np.testing.assert_allclose(b[0]['low'],[-1,-2,-3]);np.testing.assert_allclose(b[0]['matrix'][:3,3],[1,2,3])
    def test_finger_swept_bound(self):
        xml='<robot><link name="left_gripper_finger"><collision><geometry><box size=".02 .02 .1"/></geometry></collision></link><joint name="finger" type="prismatic"><origin xyz="0 .1 0"/><parent link="gripper"/><child link="left_gripper_finger"/><axis xyz="0 1 0"/><limit lower="0" upper=".05"/></joint></robot>'
        b=build_boxes(xml,lambda p:p)[0]['fallback'];self.assertEqual(b['frame'],'gripper');np.testing.assert_allclose(b['high'],[.01,.06,.05])
    def test_binary_stl_scale(self):
        with tempfile.TemporaryDirectory() as td:
            p=Path(td)/'a.stl';p.write_bytes(b' '*80+struct.pack('<I',1)+struct.pack('<12fH',0,0,1,0,0,0,1,0,0,0,2,0,0))
            low,high=stl_bounds(p,np.array([2,3,1]));np.testing.assert_allclose(high,[2,6,0])
    def test_padded_big_endian_cloud_keeps_extra_bytes(self):
        records=[struct.pack('>fffI',float(i),2.,3.,100+i) for i in range(4)]
        m=S(width=2,height=2,point_step=16,row_step=36,is_bigendian=True,data=records[0]+records[1]+b'PAD!'+records[2]+records[3]+b'PAD!',fields=[S(name=k,offset=i*4,datatype=7) for i,k in enumerate('xyz')])
        raw,xyz=unpack_cloud(m);np.testing.assert_allclose(xyz[:,0],[0,1,2,3]);self.assertEqual(raw[[0,3]].tobytes(),records[0]+records[3])
    def test_invalid_mesh_fails_instead_of_omitting_link(self):
        with self.assertRaises(ValueError):build_boxes('<robot><link name="a"><collision><geometry><mesh filename="a.dae"/></geometry></collision></link></robot>',lambda x:x)


if __name__=='__main__':unittest.main()
