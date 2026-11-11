"""Closed-loop planar tracking check with a simulated base in a separate ROS domain."""
import math
import sys
import os
import signal
import struct
import subprocess
import time
from array import array
from pathlib import Path

if os.environ.get('ROS_DOMAIN_ID') != '178' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
    raise RuntimeError('Run only in the isolated local test domain')

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist
from nav2_msgs.action import FollowPath
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import PointCloud2, PointField
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, DurabilityPolicy
from tf2_ros import TransformBroadcaster

root = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(root/'web'))
from path_speed import PathSpeedProfile
rclpy.init()
node = rclpy.create_node('simulated_omni_base')
tf = TransformBroadcaster(node)
odom = node.create_publisher(Odometry,'/r1pro_nav/odom_fused',10)
cloud = node.create_publisher(PointCloud2,'/r1pro_nav/points_filtered',10)
maps = node.create_publisher(OccupancyGrid,'/map',QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
grid=OccupancyGrid()
grid.header.frame_id='map'
grid.info.width=grid.info.height=160
grid.info.resolution=.05
grid.info.origin.position.x=grid.info.origin.position.y=-4.
grid.info.origin.orientation.w=1.
grid.data=[0]*25600
maps.publish(grid)
pose=[0.,0.,0.]
speed_profile=None
speed_limits=node.create_publisher(SpeedLimit,'/speed_limit',1)
cmd=Twist()
last=time.monotonic()
received=0.
def receive(m):
    global received,cmd
    cmd=m
    received=time.monotonic()
node.create_subscription(Twist,'/r1pro_nav/nav2/cmd_vel_safe',receive,10)
def tick():
    global last
    now=time.monotonic()
    dt=min(now-last,.10)
    last=now
    v=cmd if now-received<.25 else Twist()
    a=pose[2]
    if speed_profile is not None:
        limit=speed_profile.limit(*pose[:2],.1)
        speed_limits.publish(SpeedLimit(percentage=False,speed_limit=0. if limit>=.1-1e-6 else limit))
    pose[0]+=(math.cos(a)*v.linear.x-math.sin(a)*v.linear.y)*dt
    pose[1]+=(math.sin(a)*v.linear.x+math.cos(a)*v.linear.y)*dt
    pose[2]+=v.angular.z*dt
    stamp=node.get_clock().now().to_msg()
    for parent,child in [('map','odom'),('odom','base_link')]:
        t=TransformStamped()
        t.header.stamp=stamp;t.header.frame_id=parent;t.child_frame_id=child
        t.transform.rotation.w=1.
        if child=='base_link':
            t.transform.translation.x,t.transform.translation.y=pose[:2]
            t.transform.rotation.z=math.sin(pose[2]/2)
            t.transform.rotation.w=math.cos(pose[2]/2)
        tf.sendTransform(t)
    o=Odometry();o.header=t.header;o.child_frame_id='base_link'
    o.pose.pose.orientation=t.transform.rotation;o.pose.pose.position.x,o.pose.pose.position.y=pose[:2]
    o.twist.twist=v;odom.publish(o)
    m=PointCloud2();m.header.frame_id='base_link';m.header.stamp=stamp
    m.width=m.height=1;m.point_step=m.row_step=12
    m.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate('xyz')]
    m.data=array('B',struct.pack('<fff',3.,3.,.4));cloud.publish(m)
node.create_timer(.04,tick)
def wait(f,seconds):
    rclpy.spin_until_future_complete(node,f,timeout_sec=seconds)
    assert f.done(),'timed out'
    return f.result()
def run(dx,dy,angle):
    global speed_profile
    start=pose.copy()
    c,s=math.cos(start[2]),math.sin(start[2])
    x,y=start[0]+c*dx-s*dy,start[1]+s*dx+c*dy
    yaw=start[2]+math.radians(angle)
    path=NavPath();path.header.frame_id='map'
    for i in range(21):
        p=PoseStamped();p.header.frame_id='map'
        p.pose.position.x=start[0]+(x-start[0])*i/20
        p.pose.position.y=start[1]+(y-start[1])*i/20
        a=yaw if i==20 else math.atan2(y-start[1],x-start[0])
        p.pose.orientation.z=math.sin(a/2);p.pose.orientation.w=math.cos(a/2)
        path.poses.append(p)
    speed_profile=PathSpeedProfile([(p.pose.position.x,p.pose.position.y) for p in path.poses])
    h=wait(client.send_goal_async(FollowPath.Goal(path=path,controller_id='FollowPath',goal_checker_id='general_goal_checker')),5)
    assert h.accepted
    result=h.get_result_async()
    deadline=time.monotonic()+50
    peak_yaw=0.;sideways=0.;peak_w=0.;simultaneous=0
    while not result.done() and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        if math.hypot(x-pose[0],y-pose[1])>.11:
            peak_yaw=max(peak_yaw,abs(math.degrees(pose[2]-start[2])))
            sideways=max(sideways,math.copysign(1,dy)*cmd.linear.y)
            peak_w=max(peak_w,abs(cmd.angular.z))
            if math.hypot(cmd.linear.x,cmd.linear.y)>.015 and abs(cmd.angular.z)>.025:
                simultaneous+=1
    dist=math.hypot(x-pose[0],y-pose[1]);err=abs(math.degrees(math.remainder(pose[2]-yaw,math.tau)))
    print('relative_goal',dx,dy,angle,'status',result.result().status if result.done() else 'timeout','xy_error_m',dist,'yaw_error_deg',err,'travel_yaw_deg',peak_yaw,'travel_peak_w',peak_w,'peak_lateral_mps',sideways,flush=True)
    assert result.done() and result.result().status==4 and dist<.05 and err<6
    assert peak_yaw>8 and sideways>.012 and simultaneous>5
client=ActionClient(node,FollowPath,'/follow_path')
with (root/'reports/holonomic_test.log').open('w') as log:
    proc=subprocess.Popen(['ros2','launch',str(root/'scripts/web_tracking.launch.py')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    try:
        deadline=time.monotonic()+25
        while time.monotonic()<deadline and not client.server_is_ready():rclpy.spin_once(node,timeout_sec=.05)
        end=time.monotonic()+3
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        assert node.count_publishers('/motion_target/target_speed_chassis')==0
        run(0.,.45,85)
        run(-.32,-.32,-55)
    finally:
        proc.send_signal(signal.SIGINT)
        try:proc.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()
