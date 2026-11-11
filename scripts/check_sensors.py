"""Bounded real-sensor smoke test. Starts preview adapters and lidar only."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosidl_runtime_py.utilities import get_message

root=Path(__file__).resolve().parents[1];(root/'reports').mkdir(exist_ok=True)
rclpy.init();n=rclpy.create_node('r1pro_nav_smoke_check')
for _ in range(30):rclpy.spin_once(n,timeout_sec=.1)
hardware=['/motion_target/target_speed_chassis','/motion_target/brake_mode']
before={t:n.count_publishers(t) for t in hardware}
if any('livox_lidar_publisher'==name for name,ns in n.get_node_names_and_namespaces()):
    raise RuntimeError('Existing Livox driver found; do not start duplicate sensor session')
topics={'/r1pro_nav/odom':'nav_msgs/msg/Odometry','/r1pro_nav/scan':'sensor_msgs/msg/LaserScan',
        '/r1pro_nav/status':'std_msgs/msg/String','/r1pro_nav/preview/target_speed_chassis':'geometry_msgs/msg/TwistStamped'}
count={t:0 for t in topics};last={};subs=[]
imu_count=[0]
subs.append(n.create_subscription(get_message('sensor_msgs/msg/Imu'),'/r1pro_nav/imu',lambda msg:imu_count.__setitem__(0,imu_count[0]+1),qos_profile_sensor_data))
def callback(msg,topic):
    count[topic]+=1
    if hasattr(msg,'data'):last[topic]=json.loads(msg.data)
    elif hasattr(msg,'ranges'):
        import math
        last[topic]={'frame':msg.header.frame_id,'finite_ranges':sum(math.isfinite(v) for v in msg.ranges)}
    elif hasattr(msg,'child_frame_id'):last[topic]={'frame':msg.header.frame_id,'child':msg.child_frame_id}
    else:last[topic]={'vx':msg.twist.linear.x,'vy':msg.twist.linear.y,'wz':msg.twist.angular.z}
for t,typ in topics.items():subs.append(n.create_subscription(get_message(typ),t,lambda m,t=t:callback(m,t),qos_profile_sensor_data))
log=(root/'reports/sensors_smoke.log').open('w')
p=subprocess.Popen(['ros2','launch','r1pro_waypoint_nav','sensors.launch.py','start_lidar:=true'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:
    end=time.monotonic()+25
    while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
    after={t:n.count_publishers(t) for t in hardware}
    status=last.get('/r1pro_nav/status',{})
    checks={'all_outputs_received':all(c>5 for c in count.values()),
        'hardware_publishers_unchanged':before==after,
        'imu_unconfirmed_output_absent':imu_count[0]==0 and status.get('imu_output_enabled') is False,
        'preview_only':status.get('mode')=='preview_only',
        'no_odometry_fault':status.get('odometry_fault')=='',
        'valid_scan':last.get('/r1pro_nav/scan',{}).get('finite_ranges',0)>10,
        'preview_zero':all(v==0 for v in last.get('/r1pro_nav/preview/target_speed_chassis',{'missing':1}).values())}
    report={'checks':checks,'counts':count,'last':last,'hardware_before':before,'hardware_after':after,'passed':all(checks.values())}
    (root/'reports/sensors_smoke.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
finally:
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:p.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGTERM);p.wait(timeout=5)
    log.close();n.destroy_node();rclpy.shutdown()
if not report['passed']:raise SystemExit(1)
