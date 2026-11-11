"""No hardware command publishers. All speed outputs are preview-only."""
import copy
import json
import math
import time
import signal
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped, TwistWithCovarianceStamped, TransformStamped
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from .core import OdomIntegrator, FeedbackWarmup


class Adapters(Node):
    def __init__(self):
        super().__init__('r1pro_nav_adapters')
        defaults = dict(calibration_confirmed=False, publish_tf=False,
                        imu_units_confirmed=False, imu_accel_scale=9.80665,
                        imu_gyro_scale=1.0, imu_accel_variance=.04,
                        imu_gyro_variance=.0025, max_gap=.2, max_age=.3,
                        feedback_warmup_samples=25)
        for k,v in defaults.items(): self.declare_parameter(k,v)
        self.confirmed = self.get_parameter('calibration_confirmed').value
        self.tf_enabled = self.get_parameter('publish_tf').value
        if self.tf_enabled and not self.confirmed:
            raise ValueError('Publishing odom TF requires calibration_confirmed=true')
        self.odom = OdomIntegrator(self.get_parameter('max_gap').value)
        self.warmup = FeedbackWarmup(self.get_parameter('feedback_warmup_samples').value,
                                     self.get_parameter('max_gap').value)
        self.last_feedback = self.last_imu = self.last_scan = None
        self.last_command = None
        self.command = (0.,0.,0.)
        self.samples = 0
        self.feedback_rejected = 0
        self.feedback_rejection_reason = ''
        self.odom_pub = self.create_publisher(Odometry, '/r1pro_nav/odom', 10)
        self.velocity_pub = self.create_publisher(TwistWithCovarianceStamped, '/r1pro_nav/chassis_twist', 10)
        self.imu_pub = self.create_publisher(Imu, '/r1pro_nav/imu', 10)
        self.status_pub = self.create_publisher(String, '/r1pro_nav/status', 10)
        self.preview_pub = self.create_publisher(TwistStamped, '/r1pro_nav/preview/target_speed_chassis', 1)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(TwistStamped, '/motion_control/chassis_speed', self.feedback, qos_profile_sensor_data)
        self.create_subscription(Imu, '/livox/imu', self.imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/r1pro_nav/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Twist, '/r1pro_nav/cmd_vel', self.cmd, 1)
        self.create_timer(.05, self.tick)
        self.create_timer(1., self.status)
        self.get_logger().info('PREVIEW ONLY: no hardware speed/brake publishers; odometry is provisional until calibrated')

    def fresh_stamp(self, stamp):
        age = self.get_clock().now().nanoseconds/1e9 - (stamp.sec+stamp.nanosec/1e9)
        return -.02 <= age <= self.get_parameter('max_age').value

    def feedback(self, msg):
        if not self.fresh_stamp(msg.header.stamp):
            if self.odom.last is None:self.warmup.reset()
            self.feedback_rejected+=1;self.feedback_rejection_reason='stale_or_future_feedback'
            return
        v = (msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z)
        if not all(math.isfinite(a) for a in v):
            self.feedback_rejected+=1;self.feedback_rejection_reason='nonfinite_feedback';return
        if math.hypot(*v[:2]) > 1.5 or abs(v[2]) > 3.:
            self.odom.fault = 'invalid_or_excessive_velocity'
            return
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        if self.odom.last is not None and stamp<=self.odom.last:
            self.feedback_rejected+=1;self.feedback_rejection_reason='old_or_duplicate_feedback';return
        if not self.warmup.observe(stamp, time.monotonic()):
            return
        # Velocity becomes observable again after a gap, even if cumulative
        # wheel displacement across that gap cannot be recovered.
        measurement = TwistWithCovarianceStamped()
        measurement.header = copy.deepcopy(msg.header)
        measurement.header.frame_id = 'base_link'
        measurement.twist.twist = copy.deepcopy(msg.twist)
        for i, variance in enumerate((.01, .01, 1e6, 1e6, 1e6, .02)):
            measurement.twist.covariance[i * 7] = variance
        self.velocity_pub.publish(measurement)
        if not self.odom.update(stamp,v): return
        self.last_feedback=time.monotonic(); self.samples += 1
        x,y,yaw=self.odom.pose
        out=Odometry(); out.header=copy.deepcopy(msg.header);out.header.frame_id='odom'
        out.child_frame_id='base_link';out.pose.pose.position.x=x;out.pose.pose.position.y=y
        out.pose.pose.orientation.z=math.sin(yaw/2);out.pose.pose.orientation.w=math.cos(yaw/2)
        out.twist=measurement.twist
        # Conservative placeholders, not calibrated uncertainty estimates.
        for i,value in enumerate([.05+self.samples*.0001,.05+self.samples*.0001,1e6,1e6,1e6,.1+self.samples*.0001]):out.pose.covariance[i*7]=value
        self.odom_pub.publish(out)
        if self.tf_enabled:
            tf=TransformStamped();tf.header=out.header;tf.child_frame_id=out.child_frame_id
            tf.transform.translation.x=x;tf.transform.translation.y=y
            tf.transform.rotation=out.pose.pose.orientation;self.tf.sendTransform(tf)

    def imu(self, msg):
        if not self.fresh_stamp(msg.header.stamp): return
        self.last_imu=time.monotonic()
        if not self.get_parameter('imu_units_confirmed').value:return
        out=copy.deepcopy(msg)
        # This adapter is for Livox IMU only, not the differently mounted chassis IMU.
        if out.header.frame_id != 'livox_frame':return
        for field,scale_name,var_name in [('linear_acceleration','imu_accel_scale','imu_accel_variance'),('angular_velocity','imu_gyro_scale','imu_gyro_variance')]:
            vec=getattr(out,field);scale=self.get_parameter(scale_name).value
            if not all(math.isfinite(getattr(vec,k)) for k in ('x','y','z')):return
            for k in ('x','y','z'):setattr(vec,k,getattr(vec,k)*scale)
            cov=[0.]*9
            for i in (0,4,8):cov[i]=self.get_parameter(var_name).value
            setattr(out,field+'_covariance',cov)
        # Driver identity quaternion is not an observed global orientation.
        out.orientation_covariance=[-1.]+[0.]*8
        self.imu_pub.publish(out)

    def scan(self,msg):
        if self.fresh_stamp(msg.header.stamp) and any(math.isfinite(v) and msg.range_min <= v <= msg.range_max for v in msg.ranges):
            self.last_scan=time.monotonic()

    def cmd(self,msg):
        v=(msg.linear.x,msg.linear.y,msg.angular.z)
        if not all(math.isfinite(a) for a in v):self.last_command=None;return
        norm=math.hypot(*v[:2]);scale=min(1.,.05/norm) if norm else 1.
        self.command=(v[0]*scale,v[1]*scale,max(-.15,min(.15,v[2])))
        self.last_command=time.monotonic()

    def tick(self):
        now=time.monotonic()
        if self.last_feedback is not None and now-self.last_feedback > self.get_parameter('max_gap').value:
            self.odom.fault='feedback_timeout' # Latched: lost displacement cannot be recovered.
        ready=(not self.odom.fault and self.last_feedback is not None and self.last_scan is not None and now-self.last_scan < .5)
        v=self.command if ready and self.last_command is not None and now-self.last_command < .3 else (0.,0.,0.)
        out=TwistStamped();out.header.stamp=self.get_clock().now().to_msg();out.header.frame_id='base_link'
        out.twist.linear.x,out.twist.linear.y,out.twist.angular.z=v
        self.preview_pub.publish(out)

    def status(self):
        now=time.monotonic()
        self.status_pub.publish(String(data=json.dumps(dict(mode='preview_only',calibration_confirmed=self.confirmed,
            odometry_fault=self.odom.fault,odometry_samples=self.samples,
            feedback_warmup_count=self.warmup.count,feedback_warmup_ready=self.odom.last is not None,
            feedback_rejected=self.feedback_rejected,feedback_rejection_reason=self.feedback_rejection_reason,
            feedback_age=None if self.last_feedback is None else now-self.last_feedback,
            scan_age=None if self.last_scan is None else now-self.last_scan,
            imu_age=None if self.last_imu is None else now-self.last_imu,
            imu_output_enabled=self.get_parameter('imu_units_confirmed').value))))


def main():
    rclpy.init();n=None
    try:n=Adapters();rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException):pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if n:n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
