"""Bounded real-slam_toolbox/HTTP reset integration test on isolated domain 189.
Only synthetic LaserScan, odom and TF. Never starts robot hardware or EKF.
"""
import os
os.environ['ROS_DOMAIN_ID']='189'
os.environ['ROS_LOCALHOST_ONLY']='1'
os.environ['OPENBLAS_NUM_THREADS']='1'
import json
import math
from pathlib import Path
import signal
import subprocess
import time
import urllib.request
import urllib.error
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from ament_index_python.packages import get_package_prefix


def main():
    root=Path(__file__).resolve().parents[1];dest=root/'reports'/'managed_reset_test';dest.mkdir(parents=True,exist_ok=True)
    rclpy.init();node=Node('synthetic_reset_test');tfpub=TransformBroadcaster(node);static=StaticTransformBroadcaster(node)
    tf=TransformStamped();tf.header.frame_id='base_link';tf.child_frame_id='livox_frame';tf.transform.rotation.w=1.
    tf.header.stamp=node.get_clock().now().to_msg();static.sendTransform(tf)
    scan_pub=node.create_publisher(LaserScan,'/r1pro_nav/scan',qos_profile_sensor_data)
    odom_pub=node.create_publisher(Odometry,'/r1pro_nav/odom_fused',qos_profile_sensor_data)
    latest={};cleared=[]
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    def map_cb(msg):
        latest['map']=msg
        if len(msg.data) and all(v==-1 for v in msg.data):cleared.append(msg.info.origin)
    subs=[node.create_subscription(String,'/r1pro_nav/slam/status',lambda m:latest.update(status=json.loads(m.data)),qos),
          node.create_subscription(OccupancyGrid,'/map',map_cb,qos)]
    processes=[];logs=[];report={};failure=None
    def request(path,data=None,token=None):
        headers={}
        if data is not None:headers={'Origin':'http://127.0.0.1:8099','Content-Type':'application/json','X-Reset-Token':token or ''}
        req=urllib.request.Request('http://127.0.0.1:8099'+path,None if data is None else json.dumps(data).encode(),headers)
        with urllib.request.urlopen(req,timeout=5) as response:return response.status,json.load(response)
    try:
        manager=Path(get_package_prefix('r1pro_waypoint_nav'))/'lib/r1pro_waypoint_nav/slam_manager'
        for name,command in [('manager',[str(manager),'--ros-args','-p','session_root:='+str(dest/'sessions')]),
                              ('web',['python3','-u','web/server.py','--port','8099'])]:
            log=(dest/(name+'.log')).open('w');logs.append(log)
            processes.append(subprocess.Popen(command,cwd=root,stdout=log,stderr=subprocess.STDOUT,start_new_session=True))
        started=time.monotonic();next_scan=0.;reset_sent=False;completed=False;before=None;next_debug=0.
        while time.monotonic()-started<65:
            now=time.monotonic()
            if now>=next_scan:
                next_scan=now+.1;stamp=node.get_clock().now().to_msg()
                # Nonzero robot position and heading remain unchanged throughout reset.
                x,y,yaw=2.,-1.,.3
                tf=TransformStamped();tf.header.stamp=stamp;tf.header.frame_id='odom';tf.child_frame_id='base_link'
                tf.transform.translation.x=x;tf.transform.translation.y=y;tf.transform.rotation.z=math.sin(yaw/2);tf.transform.rotation.w=math.cos(yaw/2);tfpub.sendTransform(tf)
                odom=Odometry();odom.header=tf.header;odom.child_frame_id='base_link';odom.pose.pose.position.x=x;odom.pose.pose.position.y=y
                odom.pose.pose.orientation=tf.transform.rotation;odom_pub.publish(odom)
                scan=LaserScan();scan.header.stamp=stamp;scan.header.frame_id='livox_frame';scan.angle_min=-math.pi;scan.angle_increment=2*math.pi/720
                scan.angle_max=scan.angle_min+719*scan.angle_increment;scan.range_min=.3;scan.range_max=12.;scan.scan_time=.1
                angles=-math.pi+np.arange(720)*2*math.pi/720+yaw;directions=np.column_stack((np.cos(angles),np.sin(angles)))
                with np.errstate(divide='ignore'):
                    distances=(np.where(directions>0,5.,-5.)-np.array([x,y]))/directions
                distances[np.abs(directions)<1e-12]=np.inf;scan.ranges=distances.min(axis=1).astype(float).tolist();scan_pub.publish(scan)
            rclpy.spin_once(node,timeout_sec=.01)
            status=latest.get('status',{})
            if now>=next_debug:
                next_debug=now+8
                print('TEST STATUS',status,'map_received','map' in latest,flush=True)
            if status.get('state')=='error':raise RuntimeError(status)
            if any(p.poll() is not None for p in processes):raise RuntimeError('Test-owned process exited')
            if not reset_sent and status.get('ready') and time.monotonic()-started>10:
                _,web=request('/api/map')
                if not web['reset_allowed']:
                    if now+1>=next_debug:print('WEB STATUS',web.get('slam'),web.get('slam_fresh'),web.get('map',{}).get('source'),flush=True)
                    continue
                before=list(status['map_from_odom']);old_origin=latest['map'].info.origin
                # Invalid CSRF token must not clear anything.
                try:request('/api/map/reset',dict(confirm=True,session=status['session'],generation=0),'wrong')
                except urllib.error.HTTPError as exc:assert exc.code==403
                else:raise AssertionError('CSRF was accepted')
                code,response=request('/api/map/reset',dict(confirm=True,session=status['session'],generation=0),web['reset_token'])
                assert code==202 and response['accepted'];reset_sent=True;report['http_reset_response']=response
            if reset_sent and status.get('generation')==1 and status.get('ready'):
                after=status['map_from_odom'];assert np.allclose(before,after,atol=1e-6)
                assert cleared,'No empty map published'
                assert abs(cleared[0].position.x-old_origin.position.x)<1e-9
                assert abs(cleared[0].position.y-old_origin.position.y)<1e-9
                assert 'map' in latest and any(v>=0 for v in latest['map'].data)
                report.update(map_from_odom_before=before,map_from_odom_after=after,robot_odom=[2.,-1.,.3],
                              empty_map_seen=True,new_map_seen=True,generation=1,csrf_rejected=True,origin_preserved=True)
                completed=True;break
        assert completed,'Timed out before reset and new map were verified'
    except Exception as exc:failure=f'{type(exc).__name__}: {exc}'
    finally:
        for proc in reversed(processes):
            if proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:proc.wait(timeout=14)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
        for log in logs:log.close()
        node.destroy_node();rclpy.shutdown()
        report.update(failure=failure,domain=189,synthetic_data_only=True,motion_commands_sent=False,
                      last_status=latest.get('status'),map_received='map' in latest,
                      owned_processes_exited=all(p.poll() is not None for p in processes))
        (dest/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
    return 1 if failure else 0


if __name__=='__main__':raise SystemExit(main())
