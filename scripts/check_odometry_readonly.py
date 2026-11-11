"""Bounded read-only feedback capture. No command, odometry or TF publishers."""
import argparse
import json
import time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from tf2_ros import Buffer,TransformListener,TransformException
from rclpy.time import Time
from r1pro_waypoint_nav.core import OdomIntegrator

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds',type=float,default=12)
    parser.add_argument('--label',default='stationary')
    args=parser.parse_args()
    if not 1<=args.seconds<=60:parser.error('duration must be 1..60 seconds')
    rclpy.init();node=Node('r1pro_readonly_odometry_check')
    buffer=Buffer();listener=TransformListener(buffer,node)
    rows=[];imus={};subscriptions=[]
    def stamp(s):return s.sec+s.nanosec/1e9
    def feedback(msg):
        rows.append(dict(stamp=stamp(msg.header.stamp),arrival=time.monotonic(),
            age=node.get_clock().now().nanoseconds/1e9-stamp(msg.header.stamp),frame=msg.header.frame_id,
            velocity=[msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z]))
    subscriptions.append(node.create_subscription(TwistStamped,'/motion_control/chassis_speed',feedback,qos_profile_sensor_data))
    def imu(msg,topic):
        imus.setdefault(topic,[]).append(dict(stamp=stamp(msg.header.stamp),frame=msg.header.frame_id,
            accel=[msg.linear_acceleration.x,msg.linear_acceleration.y,msg.linear_acceleration.z],
            gyro=[msg.angular_velocity.x,msg.angular_velocity.y,msg.angular_velocity.z]))
    for topic in ['/hdas/imu_chassis','/livox/imu']:
        subscriptions.append(node.create_subscription(Imu,topic,lambda msg,t=topic:imu(msg,t),qos_profile_sensor_data))
    transforms={}
    try:
        end=time.monotonic()+args.seconds
        while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.05)
        for frame in ['livox_frame','imu_chassis_link','odom']:
            try:
                t=buffer.lookup_transform('base_link',frame,Time()).transform
                transforms[frame]=dict(translation=[t.translation.x,t.translation.y,t.translation.z],
                    quaternion=[t.rotation.x,t.rotation.y,t.rotation.z,t.rotation.w])
            except TransformException as e:transforms[frame]=dict(unavailable=str(e))
    finally:
        node.destroy_node();rclpy.shutdown()
    summary=dict(label=args.label,count=len(rows),duration_s=args.seconds,read_only=True,calibration_confirmed=False,tf=transforms)
    if len(rows)>1:
        velocity=np.array([r['velocity'] for r in rows]);stamps=np.array([r['stamp'] for r in rows]);dt=np.diff(stamps)
        integrator=OdomIntegrator()
        for r in rows:integrator.update(r['stamp'],r['velocity'])
        summary.update(frame_ids=sorted(set(r['frame'] for r in rows)),finite=bool(np.isfinite(velocity).all()),
            feedback_hz=float((len(rows)-1)/(stamps[-1]-stamps[0])) if stamps[-1]>stamps[0] else None,
            max_gap_s=float(dt.max()),nonpositive_intervals=int((dt<=0).sum()),
            age_range_s=[min(r['age'] for r in rows),max(r['age'] for r in rows)],
            mean_velocity=velocity.mean(0).tolist(),max_abs_velocity=np.abs(velocity).max(0).tolist(),
            integrated_pose=list(integrator.pose),integration_fault=integrator.fault)
    else:summary['reason']='insufficient_feedback'
    for topic,samples in imus.items():
        a=np.array([r['accel'] for r in samples]);g=np.array([r['gyro'] for r in samples])
        summary[topic]=dict(count=len(samples),frames=sorted(set(r['frame'] for r in samples)),
            mean_acceleration=a.mean(0).tolist(),mean_accel_norm=float(np.linalg.norm(a,axis=1).mean()),mean_gyro=g.mean(0).tolist())
    path=Path(__file__).resolve().parents[1]/'reports'/('odometry_readonly_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
    path.write_text(json.dumps(dict(summary=summary,feedback=rows,imu=imus),indent=2,allow_nan=False))
    print(json.dumps(summary,indent=2));print(path)

if __name__=='__main__':main()
