"""Mapping-only temporal filter. Raw scan remains available for immediate avoidance."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import copy
import json
import math
import signal
import time
from collections import deque
from dataclasses import asdict
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException
from .temporal_core import TemporalConfig,TemporalFilter
from .box_geometry import tf_matrix


def masked_scan(msg, selection):
    """Preserve geometry, timestamp and beam indices; reject with NaN."""
    out=copy.deepcopy(msg)
    values=np.array(msg.ranges,dtype=np.float32)
    values[~selection]=np.nan
    out.ranges=values.tolist()
    out.intensities=[]  # No calibrated intensity semantics in projected cloud.
    return out


class TemporalScan(Node):
    def __init__(self):
        super().__init__('r1pro_temporal_scan')
        params=asdict(TemporalConfig())
        for name,value in params.items():
            self.declare_parameter(name,value)
            params[name]=self.get_parameter(name).value
        self.filter=TemporalFilter(TemporalConfig(**params))
        self.declare_parameter('fixed_frame','odom')
        self.fixed_frame=self.get_parameter('fixed_frame').value
        if self.fixed_frame!='odom':raise ValueError('Use continuous odom frame, not loop-corrected map')
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.pending=deque(maxlen=3);self.dropped=0;self.last_ok=0.;self.geometry=None
        self.stats=dict(ready=False,reason='waiting_for_scan_and_odom_tf')
        self.pub=self.create_publisher(LaserScan,'/r1pro_nav/scan',qos_profile_sensor_data)
        self.rejected=self.create_publisher(LaserScan,'/r1pro_nav/scan_rejected',qos_profile_sensor_data)
        self.status_pub=self.create_publisher(String,'/r1pro_nav/temporal_scan/status',10)
        self.create_subscription(LaserScan,'/r1pro_nav/scan_raw',self.receive,qos_profile_sensor_data)
        self.create_timer(.02,self.tick)
        self.create_timer(.5,self.status)

    def receive(self,msg):
        if len(self.pending)==self.pending.maxlen:self.dropped+=1
        self.pending.append((msg,time.monotonic()))

    def tick(self):
        if not self.pending:return
        msg,received=self.pending[0]
        stamp=Time.from_msg(msg.header.stamp)
        age=(self.get_clock().now().nanoseconds-stamp.nanoseconds)/1e9
        if stamp.nanoseconds<=0 or not -.02<=age<=.6:
            self.pending.popleft();self.dropped+=1
            self.stats=dict(ready=False,reason='stale_or_invalid_scan');return
        try:
            transform=self.buffer.lookup_transform(self.fixed_frame,msg.header.frame_id,stamp)
        except TransformException:
            if time.monotonic()-received>.25:
                self.pending.popleft();self.dropped+=1
                self.stats=dict(ready=False,reason='missing_timestamped_odom_tf')
            return  # Never substitute latest TF or assume the robot is stationary.
        self.pending.popleft();start=time.monotonic()
        try:
            matrix=tf_matrix(transform.transform)
            if not np.isfinite(matrix).all() or matrix[2,2]<math.cos(.15):
                raise ValueError('Scan plane not approximately horizontal')
            geometry=(msg.header.frame_id,len(msg.ranges),msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max)
            if self.geometry is not None and geometry!=self.geometry:self.filter.reset()
            self.geometry=geometry
            pose=(matrix[0,3],matrix[1,3],math.atan2(matrix[1,0],matrix[0,0]))
            keep,stats=self.filter.process(msg.ranges,msg.angle_min,msg.angle_increment,msg.range_min,msg.range_max,
                                           stamp.nanoseconds/1e9,pose)
            self.pub.publish(masked_scan(msg,keep))
            if self.rejected.get_subscription_count():
                valid=np.isfinite(msg.ranges)&(np.asarray(msg.ranges)>=msg.range_min)&(np.asarray(msg.ranges)<msg.range_max)
                self.rejected.publish(masked_scan(msg,valid&~keep))
            self.last_ok=time.monotonic()
            self.stats=dict(stats,ready=stats['confirmed']>0,reason='ok' if stats['confirmed'] else 'no_confirmed_returns',
                            processing_ms=round((self.last_ok-start)*1000,3))
        except ValueError as exc:
            self.dropped+=1;self.stats=dict(ready=False,reason=str(exc))

    def status(self):
        stats=dict(self.stats,dropped_frames=self.dropped,
                   last_success_age=None if not self.last_ok else time.monotonic()-self.last_ok)
        if not self.last_ok or time.monotonic()-self.last_ok>1:stats.update(ready=False,reason='no_recent_processed_scan')
        self.status_pub.publish(String(data=json.dumps(stats)))


def main():
    signal.signal(signal.SIGINT,signal.default_int_handler)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=None
    try:node=TemporalScan();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
