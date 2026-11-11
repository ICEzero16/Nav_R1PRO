import math
import unittest
from r1pro_waypoint_nav.scan_gate import ScanGate


class ScanGateTests(unittest.TestCase):
    def test_pure_rotation_in_both_directions(self):
        for direction in (-1, 1):
            gate=ScanGate();gate.observe_accepted((0,0,0),1)
            accepted=0
            for i in range(1,61):
                pose=(0,0,direction*math.radians(i))
                if gate.wants(pose):
                    accepted+=1;gate.observe_accepted(pose,i+1)
            self.assertGreaterEqual(accepted,9)

    def test_stationary_jitter_does_not_grow_graph(self):
        gate=ScanGate();gate.observe_accepted((0,0,0),1)
        self.assertFalse(gate.wants((.003,-.004,.02)))
        self.assertTrue(gate.wants((.11,0,0)))

    def test_wraparound(self):
        gate=ScanGate();gate.observe_accepted((0,0,math.radians(179)),1)
        self.assertFalse(gate.wants((0,0,math.radians(-179))))
        self.assertTrue(gate.wants((0,0,math.radians(-170))))

    def test_rejected_scan_does_not_advance_gate(self):
        gate=ScanGate();gate.observe_accepted((0,0,0),1)
        self.assertTrue(gate.wants((0,0,.12)))
        self.assertTrue(gate.wants((0,0,.12)))
        self.assertFalse(gate.observe_accepted((0,0,.12),.5))
        self.assertTrue(gate.wants((0,0,.12)))
        gate.observe_accepted((0,0,.12),2)
        self.assertFalse(gate.wants((0,0,.12)))

    def test_invalid_and_first_pose(self):
        gate=ScanGate();self.assertTrue(gate.wants((0,0,0)))
        self.assertFalse(gate.wants((float('nan'),0,0)))
        with self.assertRaises(ValueError):ScanGate(distance=0)


if __name__=='__main__':unittest.main()
