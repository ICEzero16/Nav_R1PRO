"""Planar-only adapter for this HDAS build. Never claims full 3-D calibration."""
import math
import copy
import json
import time
import signal
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from std_msgs.msg import String


def adapt_yaw(raw,scale=-math.pi/180.,bias_raw=0.,variance=.0025,max_rate=2.):
    if not all(math.isfinite(v) for v in (scale,bias_raw,variance,max_rate)) or scale==0 or variance<=0 or max_rate<=0:
        raise ValueError('Invalid IMU adapter configuration')
    z=raw.angular_velocity.z
    if not math.isfinite(z):raise ValueError('Nonfinite gyro Z')
    rate=(z-bias_raw)*scale
    if abs(rate)>max_rate:raise ValueError('Gyro Z exceeds configured rate limit')
    out=Imu();out.header=copy.deepcopy(raw.header);out.header.frame_id='base_link'
    # This is a virtual planar measurement in base_link, not a relabeled raw IMU.
    out.angular_velocity.z=rate
    out.angular_velocity_covariance=[1e6,0.,0.,0.,1e6,0.,0.,0.,variance]
    out.orientation.w=1.;out.orientation_covariance[0]=-1.
    out.linear_acceleration_covariance[0]=-1.
    return out


class ChassisImu(Node):
    def __init__(self):
        super().__init__('r1pro_chassis_imu')
        for key,value in dict(yaw_scale=-math.pi/180.,yaw_bias_raw=0.,yaw_variance=.0025,max_rate=2.,max_age=.15).items():
            self.declare_parameter(key,value)
        self.options=dict(scale=self.get_parameter('yaw_scale').value,bias_raw=self.get_parameter('yaw_bias_raw').value,
            variance=self.get_parameter('yaw_variance').value,max_rate=self.get_parameter('max_rate').value)
        adapt_yaw(Imu(),**self.options)
        self.max_age=float(self.get_parameter('max_age').value)
        if not math.isfinite(self.max_age) or not 0<self.max_age<=1:raise ValueError('Invalid max_age')
        self.last_stamp=0;self.last_valid=0.;self.accepted=0;self.rejected=0;self.reason='waiting_for_chassis_imu'
        self.pub=self.create_publisher(Imu,'/r1pro_nav/imu_chassis_planar',qos_profile_sensor_data)
        self.status=self.create_publisher(String,'/r1pro_nav/imu_chassis/status',10)
        self.create_subscription(Imu,'/hdas/imu_chassis',self.receive,qos_profile_sensor_data)
        self.create_timer(1.,self.report)
    def receive(self,msg):
        stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
        age=(self.get_clock().now().nanoseconds-stamp)/1e9
        try:
            if stamp<=0 or not -.02<=age<=self.max_age:raise ValueError('stale_or_future_timestamp')
            if stamp<=self.last_stamp:raise ValueError('nonmonotonic_timestamp')
            out=adapt_yaw(msg,**self.options)
        except ValueError as e:
            self.rejected+=1;self.reason=str(e);return
        self.pub.publish(out);self.last_stamp=stamp;self.last_valid=time.monotonic();self.accepted+=1;self.reason='ok'
    def report(self):
        self.status.publish(String(data=json.dumps(dict(mode='yaw_rate_only',ready=time.monotonic()-self.last_valid<.5,
            accepted=self.accepted,rejected=self.rejected,reason=self.reason,full_xyz_calibrated=False,
            scale_basis='HDAS static inspection plus chassis rotation tests',**self.options))))


def main():
    # Keep the context alive until spin exits: the default asynchronous ROS
    # SIGINT handler can invalidate an in-flight take_message on Humble.
    from rclpy.signals import SignalHandlerOptions
    signal.signal(signal.SIGINT,signal.default_int_handler)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=None
    try:node=ChassisImu();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
