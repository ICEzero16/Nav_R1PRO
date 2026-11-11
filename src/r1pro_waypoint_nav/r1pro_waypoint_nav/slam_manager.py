"""Own one SLAM child; reset its graph while retaining the external map frame.

Never restarts EKF, sensors or a process it did not create. No motion publishers.
Native SLAM uses private generation topics and does not publish TF. This node
is the sole publisher of public /map and map->odom in the managed pipeline.
"""
import copy
import hashlib
from array import array
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy,qos_profile_sensor_data
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import PoseWithCovarianceStamped,TransformStamped
from nav_msgs.msg import OccupancyGrid,Odometry
from nav_msgs.srv import GetMap
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer,TransformListener,TransformBroadcaster,TransformException
from ament_index_python.packages import get_package_prefix,get_package_share_directory
from .map_anchor import MapAnchor,compose,pose_tuple
from .scan_gate import ScanGate


class SlamManager(Node):
    def __init__(self):
        super().__init__('r1pro_slam_manager')
        self.declare_parameter('slam_config',str(Path(get_package_share_directory('r1pro_waypoint_nav'))/'config/slam.yaml'))
        self.declare_parameter('scan_topic','/r1pro_nav/scan')
        self.declare_parameter('keyframe_distance',.1)
        self.declare_parameter('keyframe_heading',.05)
        self.declare_parameter('session_root',str(Path(os.environ.get('R1PRO_NAV_WS','.'))/'reports/slam_sessions'))
        self.session=uuid.uuid4().hex[:12]
        self.directory=Path(self.get_parameter('session_root').value)/self.session;self.directory.mkdir(parents=True,exist_ok=True)
        self.anchor=MapAnchor();self.generation=0;self.state='starting';self.error=''
        self.child=None;self.child_log=None;self.stop_at=0.;self.stop_stage=0;self.started=time.monotonic()
        self.native_subs=[];self.native_map=None;self.last_map=None;self.pending_pose=None;self.scan_relay=None;self.private_prefix=''
        self.scan_seen=0.;self.odom_seen=0.;self.odom_stamp=None;self.reset_count=0
        self.scan_gate=ScanGate(self.get_parameter('keyframe_distance').value,self.get_parameter('keyframe_heading').value)
        self.relayed_scans=0;self.accepted_scans=0;self.map_messages=0;self.map_content_updates=0;self.map_digest=None
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self);self.broadcaster=TransformBroadcaster(self)
        self.latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.map_pub=self.create_publisher(OccupancyGrid,'/map',self.latched)
        self.status_pub=self.create_publisher(String,'/r1pro_nav/slam/status',self.latched)
        self.create_subscription(LaserScan,self.get_parameter('scan_topic').value,self.scan,qos_profile_sensor_data)
        self.create_subscription(Odometry,'/r1pro_nav/odom_fused',self.odom,qos_profile_sensor_data)
        self.create_service(Trigger,'/r1pro_nav/slam/reset_map',self.reset_map)
        self.create_service(GetMap,'/r1pro_nav/slam/get_map',self.get_map)
        self.create_timer(.05,self.tick);self.create_timer(.5,self.publish_status)

    def scan(self,msg):
        if any(math.isfinite(r) and msg.range_min<=r<msg.range_max for r in msg.ranges):
            self.scan_seen=time.monotonic()
            # A stationary robot may produce only one accepted SLAM pose. Wait
            # for discovery before feeding its first scan so that pose isn't lost.
            if self.state in ('aligning','mapping') and self.scan_relay and time.monotonic()-self.started>2 and self.scan_relay.get_subscription_count()>0 and self.count_publishers(self.private_prefix+'/pose')>0:
                try:
                    tf=self.buffer.lookup_transform('odom','base_link',Time.from_msg(msg.header.stamp))
                except TransformException:return
                p=tf.transform.translation;q=tf.transform.rotation
                pose=(p.x,p.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
                if self.scan_gate.wants(pose):
                    self.scan_relay.publish(msg);self.relayed_scans+=1

    def odom(self,msg):
        if msg.header.frame_id=='odom' and msg.child_frame_id=='base_link':
            age=(self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
            if -.1<=age<.4:self.odom_seen=time.monotonic();self.odom_stamp=msg.header.stamp

    def ready(self):
        now=time.monotonic()
        return self.state=='mapping' and self.native_map is not None and self.last_map is not None and self.anchor.map_from_slam is not None and now-self.scan_seen<1 and now-self.odom_seen<.5

    def status_data(self):
        return dict(state=self.state,ready=self.ready(),session=self.session,generation=self.generation,
                    reset_count=self.reset_count,error=self.error,origin_policy='preserve_map_frame',
                    relayed_scans=self.relayed_scans,accepted_scans=self.accepted_scans,
                    map_messages=self.map_messages,map_content_updates=self.map_content_updates,
                    map_from_odom=self.anchor.map_from_odom,map_from_slam=self.anchor.map_from_slam)

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self.status_data())))

    def persist_anchor(self):
        path=self.directory/f'generation_{self.generation}_anchor.json'
        path.write_text(json.dumps(self.status_data(),indent=2))

    def start_child(self):
        for subscription in self.native_subs:self.destroy_subscription(subscription)
        self.native_subs=[];self.native_map=None;self.pending_pose=None
        self.scan_gate=ScanGate(self.scan_gate.distance,self.scan_gate.heading)
        self.relayed_scans=0;self.accepted_scans=0;self.map_messages=0;self.map_content_updates=0;self.map_digest=None
        generation=self.generation
        prefix=f'/r1pro_nav/slam_private/s{self.session}/g{generation}'
        self.private_prefix=prefix
        if self.scan_relay:self.destroy_publisher(self.scan_relay)
        self.scan_relay=self.create_publisher(LaserScan,prefix+'/scan',qos_profile_sensor_data)
        self.native_subs.append(self.create_subscription(PoseWithCovarianceStamped,prefix+'/pose',
            lambda msg:self.accept_pose(msg,generation),10))
        self.native_subs.append(self.create_subscription(OccupancyGrid,prefix+'/map',
            lambda msg:self.accept_map(msg,generation),self.latched))
        exe=Path(get_package_prefix('slam_toolbox'))/'lib/slam_toolbox/async_slam_toolbox_node'
        command=[str(exe),'--ros-args','--params-file',self.get_parameter('slam_config').value,
                 '-r','__node:=slam_toolbox','-p','map_frame:=slam_local_map','-p','odom_frame:=odom',
                 '-p','base_frame:=base_link','-p','transform_publish_period:=0.0',
                 # Humble's outer shouldProcessScan ignores heading and gates
                 # only on distance. Apply our OR gate above; disable both native
                 # motion thresholds so they cannot reject pure rotation again.
                 '-p','minimum_travel_distance:=0.0','-p','minimum_travel_heading:=0.0',
                 '-p','min_laser_range:=0.31','-p','use_sim_time:=false',
                 '-p','scan_topic:='+prefix+'/scan',
                 '-p','map_name:='+prefix+'/map','-r','pose:='+prefix+'/pose']
        self.child_log=(self.directory/f'generation_{generation}.log').open('w')
        self.child=subprocess.Popen(command,stdout=self.child_log,stderr=subprocess.STDOUT,start_new_session=True)
        self.state='aligning';self.started=time.monotonic();self.publish_status()

    def accept_pose(self,msg,generation):
        if generation==self.generation and self.state in ('aligning','mapping'):self.pending_pose=msg

    def accept_map(self,msg,generation):
        if generation!=self.generation or self.state not in ('aligning','mapping'):return
        self.map_messages+=1
        geometry=(msg.info.width,msg.info.height,msg.info.resolution,pose_tuple(msg.info.origin))
        digest=hashlib.sha256(repr(geometry).encode()+array('b',msg.data).tobytes()).hexdigest()
        if digest!=self.map_digest:self.map_content_updates+=1;self.map_digest=digest
        self.native_map=msg
        if self.anchor.map_from_slam is not None:self.publish_map()

    def publish_map(self):
        if self.native_map is None:return
        out=copy.deepcopy(self.native_map);out.header.frame_id='map'
        origin=compose(self.anchor.map_from_slam,pose_tuple(out.info.origin))
        out.info.origin.position.x=origin[0];out.info.origin.position.y=origin[1]
        out.info.origin.orientation.x=0.;out.info.origin.orientation.y=0.
        out.info.origin.orientation.z=math.sin(origin[2]/2);out.info.origin.orientation.w=math.cos(origin[2]/2)
        self.last_map=out;self.map_pub.publish(out)

    def reset_map(self,request,response):
        if not self.ready():
            response.success=False;response.message='建图未就绪、正在重置或扫描/里程计已过期';return response
        # Snapshot before mutating anything. An I/O error must not clear the map.
        try:
            snapshot=dict(status=self.status_data(),width=self.last_map.info.width,height=self.last_map.info.height,
                          resolution=self.last_map.info.resolution,origin=pose_tuple(self.last_map.info.origin),
                          data=list(self.last_map.data))
            (self.directory/f'before_reset_{self.generation}.json').write_text(json.dumps(snapshot))
        except OSError as exc:
            response.success=False;response.message='重置前快照保存失败: '+str(exc);return response
        self.anchor.begin_reset();self.state='resetting';self.reset_count+=1
        self.pending_pose=None;self.native_map=None
        blank=copy.deepcopy(self.last_map);blank.data=[-1]*len(blank.data)
        blank.header.stamp=self.get_clock().now().to_msg();self.last_map=blank;self.map_pub.publish(blank)
        self.stop_at=time.monotonic();self.stop_stage=0
        if self.child and self.child.poll() is None:self.child.send_signal(signal.SIGINT)
        self.publish_status()
        response.success=True;response.message='已开始重置，保留 map 坐标原点；等待重新建图';return response

    def get_map(self,request,response):
        if self.last_map is not None:response.map=self.last_map
        return response

    def tick(self):
        now=time.monotonic()
        if self.state=='starting' and now-self.started>2:
            conflicts=[n for n in self.get_node_names() if n=='slam_toolbox']
            others=[i for i in self.get_publishers_info_by_topic('/map') if i.node_name!=self.get_name()]
            if conflicts or others or self.buffer.can_transform('map','odom',Time()):
                self.state='error';self.error='Existing SLAM/map publisher or map->odom TF; refusing duplicate ownership';self.publish_status();return
            try:self.start_child()
            except Exception as exc:self.state='error';self.error=str(exc);self.publish_status()
        if self.state=='resetting':
            if self.child is None or self.child.poll() is not None:
                if self.child_log:self.child_log.close()
                self.generation+=1
                try:self.start_child()
                except Exception as exc:self.state='error';self.error=str(exc);self.publish_status()
            elif now-self.stop_at>8 and self.stop_stage==0:
                self.child.terminate();self.stop_stage=1
            elif now-self.stop_at>11 and self.stop_stage==1:
                self.child.kill();self.stop_stage=2
        elif self.child is not None and self.child.poll() is not None and self.state not in ('error','starting'):
            self.state='error';self.error='Owned SLAM process exited';self.publish_status()
        if self.pending_pose is not None and self.state in ('aligning','mapping'):
            msg=self.pending_pose
            try:
                tf=self.buffer.lookup_transform('odom','base_link',Time.from_msg(msg.header.stamp))
                q=tf.transform.rotation;t=tf.transform.translation
                odom=(t.x,t.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
                first=self.anchor.map_from_slam is None
                self.anchor.observe(pose_tuple(msg.pose.pose),odom)
                if self.scan_gate.observe_accepted(odom,msg.header.stamp.sec+msg.header.stamp.nanosec/1e9):self.accepted_scans+=1
                self.state='mapping';self.pending_pose=None;self.error=''
                if first:self.persist_anchor();self.publish_map();self.publish_status()
            except TransformException:pass
        if self.state=='aligning' and now-self.started>30:
            self.error='等待有效扫描与对应时间的里程计 TF，尚未完成地图对齐'
        # Preserve correction through reset, but never fake fresh odometry.
        if self.anchor.map_from_odom is not None and self.odom_stamp is not None and now-self.odom_seen<.5 and self.state!='error':
            x,y,yaw=self.anchor.map_from_odom
            tf=TransformStamped();tf.header.stamp=self.odom_stamp;tf.header.frame_id='map';tf.child_frame_id='odom'
            tf.transform.translation.x=x;tf.transform.translation.y=y
            tf.transform.rotation.z=math.sin(yaw/2);tf.transform.rotation.w=math.cos(yaw/2)
            self.broadcaster.sendTransform(tf)

    def close(self):
        if self.child and self.child.poll() is None:
            self.child.send_signal(signal.SIGINT)
            try:self.child.wait(timeout=8)
            except subprocess.TimeoutExpired:
                self.child.terminate()
                try:self.child.wait(timeout=3)
                except subprocess.TimeoutExpired:self.child.kill();self.child.wait(timeout=3)
        if self.child_log:self.child_log.close()


def main():
    signal.signal(signal.SIGINT,signal.default_int_handler)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=None
    try:node=SlamManager();rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node:node.close();node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
