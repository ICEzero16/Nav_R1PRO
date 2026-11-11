import math
import unittest
from r1pro_waypoint_nav.core import integrate,OdomIntegrator,validate_points


class CoreTests(unittest.TestCase):
    def test_sideways_in_rotated_body(self):
        x,y,a=integrate(0,0,math.pi/2,0,1,0,1)
        self.assertAlmostEqual(x,-1);self.assertAlmostEqual(y,0)
    def test_arc(self):
        x,y,a=integrate(0,0,0,1,0,1,math.pi/2)
        self.assertAlmostEqual(x,1);self.assertAlmostEqual(y,1)
    def test_gap_latches_and_does_not_invent_distance(self):
        o=OdomIntegrator();self.assertTrue(o.update(1,(1,0,0)))
        self.assertTrue(o.update(1.1,(1,0,0)))
        self.assertFalse(o.update(2,(1,0,0)))
        self.assertFalse(o.update(2.1,(1,0,0)));self.assertAlmostEqual(o.pose[0],.1)
    def test_backward_stamp(self):
        o=OdomIntegrator();o.update(2,(0,0,0));self.assertFalse(o.update(1,(0,0,0)))
    def test_nan(self):
        o=OdomIntegrator();self.assertFalse(o.update(1,(float('nan'),0,0)))
    def test_wrong_map(self):
        with self.assertRaises(ValueError):validate_points({'map_id':'old','frame_id':'map','points':{}},'new')
    def test_invalid_waypoint(self):
        with self.assertRaises(ValueError):validate_points({'map_id':'a','frame_id':'map','points':{'x':{'x':0,'y':0,'yaw':float('nan')}}},'a')


if __name__=='__main__':unittest.main()
