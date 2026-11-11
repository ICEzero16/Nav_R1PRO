"""Four small operator-authorized motions; no navigation/lidar services."""
import argparse,json,math,time
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Imu
from hdas_msg.msg import ControllerSignalStamped,FeedbackStatus
from r1pro_waypoint_nav.core import OdomIntegrator

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trial',choices=['all','left','right','ccw','cw'],default='all')
    args=parser.parse_args()
    rclpy.init();n=Node('r1pro_bounded_axis_test');pub=None
    mode=[None,0.];feedback=[];imus=[];errors=[];active=[None];results=[]
    def mode_cb(m):mode[:]=[int(m.data.mode),time.monotonic()]
    def feedback_cb(m):
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        v=[m.twist.linear.x,m.twist.linear.y,m.twist.angular.z]
        age=n.get_clock().now().nanoseconds/1e9-stamp
        if not all(math.isfinite(x) for x in v) or not -.02<=age<=.15:return
        feedback.append(dict(t=time.monotonic(),stamp=stamp,v=v))
        if active[0] is not None:active[0].update(stamp,v)
    def imu_cb(m):
        imus.append(dict(t=time.monotonic(),stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9,
            gyro=[m.angular_velocity.x,m.angular_velocity.y,m.angular_velocity.z],
            accel=[m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z]))
    def error_cb(m):errors.extend([dict(name=e.name,code=e.error_code) for e in m.errors if e.error_code])
    subs=[n.create_subscription(ControllerSignalStamped,'/controller',mode_cb,qos_profile_sensor_data),
          n.create_subscription(TwistStamped,'/motion_control/chassis_speed',feedback_cb,qos_profile_sensor_data),
          n.create_subscription(Imu,'/hdas/imu_chassis',imu_cb,qos_profile_sensor_data),
          n.create_subscription(FeedbackStatus,'/hdas/feedback_status_chassis',error_cb,qos_profile_sensor_data)]
    def send(v):
        m=TwistStamped();m.header.stamp=n.get_clock().now().to_msg();m.header.frame_id='base_link'
        m.twist.linear.x,m.twist.linear.y,m.twist.angular.z=v;pub.publish(m)
    def halt():
        if pub is None:return
        end=time.monotonic()+2;next_send=0
        while time.monotonic()<end:
            if time.monotonic()>=next_send:send((0.,0.,0.));next_send=time.monotonic()+.05
            rclpy.spin_once(n,timeout_sec=.005)
    def stationary():
        return (len(feedback)>=20 and time.monotonic()-feedback[-1]['t']<.15 and
                all(math.hypot(*r['v'][:2])<.01 and abs(r['v'][2])<.03 for r in feedback[-10:]))
    failure=None
    try:
        end=time.monotonic()+8
        while time.monotonic()<end:
            rclpy.spin_once(n,timeout_sec=.02)
            if len(feedback)>=80 and len(imus)>=100 and mode[0] is not None:break
        if mode[0]!=5 or time.monotonic()-mode[1]>.2 or errors or not stationary():raise RuntimeError('Preflight not ready')
        if n.count_publishers('/motion_target/target_speed_chassis'):raise RuntimeError('Competing speed publisher')
        pub=n.create_publisher(TwistStamped,'/motion_target/target_speed_chassis',qos_profile_sensor_data)
        end=time.monotonic()+3
        while not pub.get_subscription_count() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
        if not pub.get_subscription_count():raise RuntimeError('No controller subscriber')
        for name,v,duration in [('left',(0.,.03,0.),1.),('right',(0.,-.03,0.),1.),
                                ('ccw',(0.,0.,.10),1.5),('cw',(0.,0.,-.10),1.5)]:
            if args.trial!='all' and name!=args.trial:continue
            if not stationary():raise RuntimeError('Not stationary before '+name)
            baseline=[r['gyro'] for r in imus if time.monotonic()-r['t']<1.]
            bias=np.mean(baseline,axis=0).tolist() if baseline else None
            active[0]=OdomIntegrator(.15);start=time.monotonic();next_send=start;reason='duration_limit';count=0
            while time.monotonic()-start<duration:
                now=time.monotonic()
                if mode[0]!=5 or now-mode[1]>.2:reason='ECU_mode_lost';break
                if errors:reason='motor_error';break
                if now-feedback[-1]['t']>.15:reason='feedback_timeout';break
                if active[0].fault:reason=active[0].fault;break
                current=feedback[-1]['v'];x,y,yaw=active[0].pose
                if math.hypot(*current[:2])>.08 or abs(current[2])>.25:reason='speed_limit';break
                if math.hypot(x,y)>.05 or abs(yaw)>.20:reason='pose_limit';break
                if n.count_publishers('/motion_target/target_speed_chassis')>1:reason='competing_controller';break
                if now>=next_send:send(v);count+=1;next_send=now+.05
                rclpy.spin_once(n,timeout_sec=.005)
            halt();end=time.monotonic();stopped=stationary()
            record=dict(name=name,command=v,command_duration_s=duration,stop_reason=reason,nonzero_messages=count,
                feedback_pose=active[0].pose,fault=active[0].fault,stopped=stopped,imu_bias_raw=bias,
                feedback=[r for r in feedback if start<=r['t']<=end],imu=[r for r in imus if start<=r['t']<=end])
            results.append(record);active[0]=None
            print(json.dumps({k:value for k,value in record.items() if k not in ['feedback','imu']}),flush=True)
            if reason!='duration_limit' or not stopped or record['fault']:raise RuntimeError('Aborted after '+name)
    except Exception as e:failure=str(e)
    finally:
        halt()
        report=dict(failure=failure,trials=results,errors=errors,stopped=stationary(),physical_axes_confirmed=False,metric_scale_calibrated=False)
        path=Path(__file__).resolve().parents[1]/'reports'/('axis_trials_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
        path.write_text(json.dumps(report,indent=2));print('REPORT',path,'failure',failure,'stopped',report['stopped'],flush=True)
        n.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
