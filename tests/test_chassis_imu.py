import unittest,math,copy
from sensor_msgs.msg import Imu
from r1pro_waypoint_nav.chassis_imu import adapt_yaw,ChassisImu

class ImuTests(unittest.TestCase):
    def test_yaw_units_sign_and_input_unchanged(self):
        raw=Imu();raw.header.stamp.sec=123;raw.angular_velocity.z=-180.
        before=copy.deepcopy(raw);out=adapt_yaw(raw,max_rate=4.)
        self.assertAlmostEqual(out.angular_velocity.z,math.pi)
        self.assertEqual(raw,before);self.assertEqual(out.header.stamp,raw.header.stamp)
        self.assertEqual(out.header.frame_id,'base_link')
    def test_unverified_channels_unavailable(self):
        raw=Imu();raw.angular_velocity.x=9.;raw.linear_acceleration.z=-1.
        out=adapt_yaw(raw)
        self.assertEqual(out.orientation_covariance[0],-1.)
        self.assertEqual(out.linear_acceleration_covariance[0],-1.)
        self.assertEqual(out.angular_velocity_covariance[0],1e6)
        self.assertEqual(out.angular_velocity.x,0.)
    def test_invalid_and_limits(self):
        for z in [float('nan'),float('inf'),1000.]:
            raw=Imu();raw.angular_velocity.z=z
            with self.assertRaises(ValueError):adapt_yaw(raw)
        for options in [dict(scale=0),dict(variance=0),dict(bias_raw=float('nan'))]:
            with self.assertRaises(ValueError):adapt_yaw(Imu(),**options)
    def test_bias(self):
        raw=Imu();raw.angular_velocity.z=.1
        self.assertEqual(adapt_yaw(raw,bias_raw=.1).angular_velocity.z,0.)
    def test_stale_duplicate_future_rejected_without_ros_start(self):
        from types import SimpleNamespace as S
        sent=[];dummy=S(last_stamp=0,last_valid=0,accepted=0,rejected=0,reason='',max_age=.15,
            options={},pub=S(publish=sent.append),get_clock=lambda:S(now=lambda:S(nanoseconds=10_000_000_000)))
        for sec,nano in [(9,0),(11,0),(10,0),(10,0)]:
            msg=Imu();msg.header.stamp.sec=sec;msg.header.stamp.nanosec=nano
            ChassisImu.receive(dummy,msg)
        self.assertEqual(len(sent),1);self.assertEqual(dummy.rejected,3)

if __name__=='__main__':unittest.main()
