"""Profile one explicitly authorized rotation trial; never issue motion directly."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import json
import argparse
from pathlib import Path
import signal
import subprocess
import time
import numpy as np
import psutil
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, LaserScan
from geometry_msgs.msg import TwistStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--translate-roundtrip',action='store_true')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1]
    out=root/'reports'/('rotation_profile_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir()
    rclpy.init();node=rclpy.create_node('r1pro_rotation_profiler_readonly')
    samples={};statuses={};cpu=[];subs=[];sessions=set();processes={};motion=[]
    def receive(key,msg):
        now=time.time();stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        samples.setdefault(key,[]).append([now,1000*(now-stamp)])
    def speed(msg):
        if abs(msg.twist.angular.z)>.02 or abs(msg.twist.linear.x)>.005:motion.append(time.time())
    def status(key,msg):
        d=json.loads(msg.data);statuses.setdefault(key,[]).append([time.time(),d])
        if key=='slam' and d.get('session'):
            prefix=f"/r1pro_nav/slam_private/s{d['session']}/g{d['generation']}"
            if prefix not in sessions:
                sessions.add(prefix)
                subs.append(node.create_subscription(PoseWithCovarianceStamped,prefix+'/pose',lambda m:receive('accepted_pose',m),10))
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    for kind,topic,key in [(PointCloud2,'/hdas/lidar_chassis_left','lidar'),(PointCloud2,'/r1pro_nav/points_denoised','denoised'),(LaserScan,'/r1pro_nav/scan_raw','scan_raw'),(LaserScan,'/r1pro_nav/scan','scan_confirmed'),(OccupancyGrid,'/map','map')]:
        subs.append(node.create_subscription(kind,topic,lambda m,k=key:receive(k,m),latched if key=='map' else qos_profile_sensor_data))
    for topic,key in [('/r1pro_nav/self_filter/status','filter'),('/r1pro_nav/temporal_scan/status','temporal'),('/r1pro_nav/slam/status','slam')]:
        subs.append(node.create_subscription(String,topic,lambda m,k=key:status(k,m),latched if key=='slam' else 10))
    subs.append(node.create_subscription(TwistStamped,'/motion_control/chassis_speed',speed,qos_profile_sensor_data))
    command=['python3','-u',str(root/'scripts/rotate_and_map.py')]
    if args.translate_roundtrip:command.append('--translate-roundtrip')
    proc=subprocess.Popen(command,cwd=root)
    started=time.monotonic();last_cpu=0.
    try:
        while proc.poll() is None:
            rclpy.spin_once(node,timeout_sec=.01)
            if time.monotonic()-started>150:
                proc.send_signal(signal.SIGINT);raise RuntimeError('Profile trial timed out; requesting guarded stop')
            if time.monotonic()-last_cpu<1:continue
            last_cpu=time.monotonic()
            for p in psutil.process_iter(['pid','cmdline']):
                cmd=' '.join(p.info['cmdline'] or [])
                if p.pid==os.getpid() or not any(x in cmd for x in ('/self_filter','/temporal_scan','/slam_manager','async_slam_toolbox_node','pointcloud_to_laserscan_node','/ekf_node','web/server.py','livox_ros_driver2_node')):continue
                try:
                    if p.pid not in processes:
                        processes[p.pid]=p;p.cpu_percent();continue
                    p=processes[p.pid]
                    cpu.append(dict(t=time.time(),pid=p.pid,command=cmd,cpu=p.cpu_percent(),rss_mb=p.memory_info().rss/1048576,threads=p.num_threads()))
                except (psutil.NoSuchProcess,psutil.AccessDenied):pass
    finally:
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT);proc.wait(timeout=30)
        def summary(rows):
            if not rows:return None
            duration=rows[-1][0]-rows[0][0]
            return dict(count=len(rows),fps=(len(rows)-1)/duration if duration>0 else 0,
                        age_ms_p50=float(np.percentile([x[1] for x in rows],50)),age_ms_p95=float(np.percentile([x[1] for x in rows],95)))
        window=(min(motion),max(motion)) if motion else None
        result=dict(trial_exit=proc.returncode,motion_window=window,
                    streams={k:summary(v) for k,v in samples.items()},
                    during_motion={k:summary([x for x in v if window and window[0]<=x[0]<=window[1]]) for k,v in samples.items()},
                    processing={},processes={})
        for key in ('filter','temporal'):
            rows=[d for t,d in statuses.get(key,[]) if window and window[0]<=t<=window[1] and 'processing_ms' in d]
            if rows:result['processing'][key]={name:float(np.percentile([d['processing_ms'] for d in rows],pct)) for name,pct in [('ms_p50',50),('ms_p95',95)]}
        for pid in {x['pid'] for x in cpu}:
            rows=[x for x in cpu if x['pid']==pid and window and window[0]<=x['t']<=window[1]]
            if rows:result['processes'][pid]=dict(command=rows[0]['command'],cpu_mean=float(np.mean([x['cpu'] for x in rows])),cpu_peak=max(x['cpu'] for x in rows),rss_mb=max(x['rss_mb'] for x in rows),threads=max(x['threads'] for x in rows))
        if statuses.get('slam'):result['slam_final']=statuses['slam'][-1][1]
        (out/'summary.json').write_text(json.dumps(result,indent=2))
        (out/'samples.json').write_text(json.dumps(dict(streams=samples,statuses=statuses,cpu=cpu,motion=motion)))
        print('PROFILE',out,flush=True)
        node.destroy_node();rclpy.shutdown()
    return proc.returncode


if __name__=='__main__':raise SystemExit(main())
