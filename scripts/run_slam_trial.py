"""One bounded stationary mapping trial; stop owned processes on every exit."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import json,math,signal,subprocess,time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import LaserScan,Imu
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException
from rclpy.time import Time
from slam_toolbox.srv import SerializePoseGraph
from r1pro_waypoint_nav.motion_guard import MotionGuard,load_config,pipeline_issues

def main():
    root=Path(__file__).resolve().parents[1]
    cfg=load_config(root/'src/r1pro_waypoint_nav/config/rotation_guard.yaml');guard=MotionGuard(cfg);guard_events=[];last_guard='run'
    dest=root/'maps'/('stationary_trial_'+time.strftime('%Y%m%d_%H%M%S'));dest.mkdir(parents=True)
    rclpy.init();n=Node('r1pro_slam_trial_observer');buf=Buffer();listener=TransformListener(buf,n)
    latest={};counts={};seen={};subscriptions=[];proc=None;log=None;failure=None;tf_ok=False;map_saved=False;graph_saved=False
    def receive(msg,key):
        latest[key]=msg;counts[key]=counts.get(key,0)+1
        if isinstance(msg,LaserScan) and not any(math.isfinite(r) and msg.range_min<=r<msg.range_max for r in msg.ranges):return
        seen[key]=time.monotonic()
    aliases={'imu':'/r1pro_nav/imu_chassis_planar','scan':'/r1pro_nav/scan','fused':'/r1pro_nav/odom_fused',
        'adapters':'/r1pro_nav/status','filter':'/r1pro_nav/self_filter/status','temporal':'/r1pro_nav/temporal_scan/status','slam':'/r1pro_nav/slam/status'}
    def health():
        now=time.monotonic()
        status={key:json.loads(latest[topic].data) for key,topic in aliases.items() if topic in latest and isinstance(latest[topic],String)}
        immediate,diagnostic,hard=pipeline_issues(now,{key:seen[topic] for key,topic in aliases.items() if topic in seen},status,cfg)
        if now-seen.get('/motion_control/chassis_speed',0)>cfg.wheel_timeout_sec:immediate.append('stale_wheel')
        return immediate,diagnostic,hard
    for topic,typ in [('/r1pro_nav/odom_fused',Odometry),('/r1pro_nav/scan',LaserScan),('/r1pro_nav/imu_chassis_planar',Imu),
                      ('/motion_control/chassis_speed',TwistStamped),('/r1pro_nav/status',String),('/r1pro_nav/self_filter/status',String),
                      ('/r1pro_nav/temporal_scan/status',String),('/r1pro_nav/slam/status',String)]:
        subscriptions.append(n.create_subscription(typ,topic,lambda m,t=topic:receive(m,t),qos_profile_sensor_data))
    subscriptions.append(n.create_subscription(OccupancyGrid,'/map',lambda m:receive(m,'/map'),
        QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)))
    try:
        end=time.monotonic()+5
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
        names=n.get_node_names()
        if any(x in names for x in ['r1pro_nav_adapters','r1pro_self_filter','r1pro_chassis_imu','r1pro_temporal_scan','r1pro_slam_manager','r1pro_ekf','slam_toolbox']):raise RuntimeError('Existing trial component detected; refusing duplicate startup')
        if n.count_publishers('/hdas/lidar_chassis_left')>0:raise RuntimeError('Existing lidar detected; refusing ownership conflict')
        if n.count_publishers('/motion_target/target_speed_chassis')>0:raise RuntimeError('Speed command publisher present')
        if time.monotonic()-seen.get('/motion_control/chassis_speed',0)>.2:raise RuntimeError('Missing fresh chassis feedback')
        if buf.can_transform('odom','base_link',Time()):raise RuntimeError('Existing odom TF detected')
        log=(dest/'launch.log').open('w')
        proc=subprocess.Popen(['ros2','launch','r1pro_waypoint_nav','slam_trial.launch.py','start_lidar:=true'],cwd=root,
            stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        start=time.monotonic();deadline=start+40
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.03)
            elapsed=time.monotonic()-start
            if proc.poll() is not None:raise RuntimeError('Launch exited early')
            if elapsed>12:
                level=guard.update(time.monotonic(),*health())
                if level!=last_guard:
                    guard_events.append(dict(elapsed=elapsed,level=level,reason=guard.reason,
                        statuses={key:json.loads(latest[topic].data) for key,topic in aliases.items() if topic in latest and isinstance(latest[topic],String)}));last_guard=level
                if level=='abort':raise RuntimeError(guard.reason)
        if not guard.may_move or any(health()[i] for i in (0,2)):raise RuntimeError('Mapping not healthy at end of trial')
        tf=buf.lookup_transform('map','base_link',Time());tf_ok=True
        if n.get_clock().now().nanoseconds/1e9-(tf.header.stamp.sec+tf.header.stamp.nanosec/1e9)>1:
            raise RuntimeError('Map to base TF stale')
        if '/map' not in latest:raise RuntimeError('No occupancy map received')
        msg=latest['/map'];grid=np.array(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
        if not np.any(grid>=0):raise RuntimeError('Map contains only unknown cells')
        pixels=np.full(grid.shape,205,dtype=np.uint8);pixels[(grid>=0)&(grid<=25)]=254;pixels[grid>=65]=0
        (dest/'map.pgm').write_bytes(f'P5\n{msg.info.width} {msg.info.height}\n255\n'.encode()+np.flipud(pixels).tobytes())
        p=msg.info.origin.position;q=msg.info.origin.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        (dest/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=float(msg.info.resolution),origin=[p.x,p.y,yaw],negate=0,occupied_thresh=.65,free_thresh=.196,mode='trinary')))
        map_saved=True
        client=n.create_client(SerializePoseGraph,'/slam_toolbox/serialize_map')
        if not client.wait_for_service(timeout_sec=3):raise RuntimeError('Pose graph save service absent')
        req=SerializePoseGraph.Request();req.filename=str(dest/'posegraph')
        future=client.call_async(req);rclpy.spin_until_future_complete(n,future,timeout_sec=8)
        graph_saved=bool(future.done() and future.result() and future.result().result==0)
        if not graph_saved:raise RuntimeError('Pose graph save failed')
        if '/r1pro_nav/slam/status' in latest:
            (dest/'posegraph.anchor.json').write_text(latest['/r1pro_nav/slam/status'].data)
    except Exception as e:failure=str(e)
    finally:
        if proc is not None and proc.poll() is None:
            # ros2 launch forwards SIGINT to children itself. Avoid double SIGINT.
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
        if log:log.close()
        summaries={key:json.loads(latest[key].data) for key in ['/r1pro_nav/status','/r1pro_nav/self_filter/status','/r1pro_nav/temporal_scan/status','/r1pro_nav/slam/status'] if key in latest}
        report=dict(failure=failure,counts=counts,tf_map_to_base=tf_ok,map_saved=map_saved,posegraph_saved=graph_saved,
            guard_events=guard_events,
            status=summaries,stationary_trial=True,motion_commands_sent=False,owned_launch_exited=proc is None or proc.poll() is not None)
        if '/map' in latest:
            m=latest['/map'];report['map']=dict(width=m.info.width,height=m.info.height,resolution=m.info.resolution,known_cells=sum(v>=0 for v in m.data))
        (dest/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2));print('OUTPUT',dest)
        n.destroy_node();rclpy.shutdown()
    return 1 if failure else 0

if __name__=='__main__':raise SystemExit(main())
