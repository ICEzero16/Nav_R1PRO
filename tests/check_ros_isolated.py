"""Synthetic checks in an isolated ROS domain. Never run in robot domain 79."""
import os
if os.environ.get('ROS_DOMAIN_ID') != '178':raise RuntimeError('Use ROS_DOMAIN_ID=178 for isolated test')
import json
import math
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist,TwistStamped,TransformStamped
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import LaserScan
from lifecycle_msgs.srv import GetState
from std_msgs.msg import String

root=Path(__file__).resolve().parents[1];root.joinpath('reports').mkdir(exist_ok=True)
rclpy.init();n=rclpy.create_node('isolated_nav_check');subs=[];preview=[];status=[]
subs.append(n.create_subscription(TwistStamped,'/r1pro_nav/preview/target_speed_chassis',lambda m:preview.append((m.twist.linear.x,m.twist.linear.y,m.twist.angular.z)),10))
subs.append(n.create_subscription(String,'/r1pro_nav/status',lambda m:status.append(json.loads(m.data)),10))
fb=n.create_publisher(TwistStamped,'/motion_control/chassis_speed',10)
scan=n.create_publisher(LaserScan,'/r1pro_nav/scan',10)
cmd=n.create_publisher(Twist,'/r1pro_nav/cmd_vel',10)
report={}
def stop(p):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:p.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
def spin(seconds,send_feedback=True,send_command=False):
    end=time.monotonic()+seconds;next_send=0
    while time.monotonic()<end:
        if time.monotonic()>=next_send:
            next_send=time.monotonic()+.04
            if send_feedback:
                f=TwistStamped();f.header.stamp=n.get_clock().now().to_msg();fb.publish(f)
                s=LaserScan();s.header.stamp=f.header.stamp;s.range_min=.1;s.range_max=10.;s.ranges=[1.]*20;scan.publish(s)
            if send_command:
                c=Twist();c.linear.x=1.;c.linear.y=1.;c.angular.z=2.;cmd.publish(c)
        rclpy.spin_once(n,timeout_sec=.01)
try:
    with (root/'reports/isolated_adapters.log').open('w') as log:
        p=subprocess.Popen(['ros2','run','r1pro_waypoint_nav','adapters'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            spin(4,True,True)
            report['clamp']=bool(preview and abs(math.hypot(*preview[-1][:2])-.05)<1e-6 and abs(preview[-1][2]-.15)<1e-6)
            spin(.5,True,False);report['command_timeout_zero']=bool(preview and preview[-1]==(0.,0.,0.))
            spin(1.5,False,False);report['feedback_fault_latched']=bool(status and status[-1]['odometry_fault']=='feedback_timeout')
            spin(.5,True,True);report['fault_blocks_preview']=bool(preview and preview[-1]==(0.,0.,0.))
            report['no_hardware_publishers']=all(n.count_publishers(t)==0 for t in ['/motion_target/target_speed_chassis','/motion_target/brake_mode'])
        finally:stop(p)
    # Validate Nav2 plugins and lifecycle startup on an artificial map, with no goals.
    testdir=root/'reports/synthetic_map';testdir.mkdir(exist_ok=True)
    pixels=bytes(0 if x in (0,99) or y in (0,99) else 254 for y in range(100) for x in range(100))
    (testdir/'map.pgm').write_bytes(b'P5\n100 100\n255\n'+pixels)
    (testdir/'map.yaml').write_text('image: map.pgm\nresolution: 0.1\norigin: [-5.0, -5.0, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')
    broadcaster=TransformBroadcaster(n)
    def synthetic_tf():
        for parent,child in [('map','odom'),('odom','base_link')]:
            t=TransformStamped();t.header.stamp=n.get_clock().now().to_msg()
            t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.
            broadcaster.sendTransform(t)
    tf_timer=n.create_timer(.05,synthetic_tf)
    with (root/'reports/isolated_nav2.log').open('w') as log:
        p=subprocess.Popen(['ros2','launch','r1pro_waypoint_nav','navigation_preview.launch.py','calibration_confirmed:=true','map:='+str(testdir/'map.yaml')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        try:
            clients={name:n.create_client(GetState,'/'+name+'/get_state') for name in ['map_server','amcl','controller_server','planner_server','behavior_server','bt_navigator']}
            states={};deadline=time.monotonic()+35
            while time.monotonic()<deadline and len(states)<6:
                for name,client in clients.items():
                    if name in states or not client.service_is_ready():continue
                    f=client.call_async(GetState.Request());rclpy.spin_until_future_complete(n,f,timeout_sec=.5)
                    if f.done() and f.result().current_state.id==3:states[name]='active'
                rclpy.spin_once(n,timeout_sec=.1)
            report['nav2_all_active']=len(states)==6
            report['nav2_active_nodes']=states
            report['nav2_no_hardware_publishers']=all(n.count_publishers(t)==0 for t in ['/motion_target/target_speed_chassis','/motion_target/brake_mode'])
        finally:stop(p)
finally:
    n.destroy_node();rclpy.shutdown()
report['passed']=all(v for k,v in report.items() if isinstance(v,bool))
(root/'reports/isolated_checks.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
if not report['passed']:raise SystemExit(1)
