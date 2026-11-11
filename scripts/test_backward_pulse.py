"""ONE authorized 0.03 m/s backward pulse, <=1 s. No automatic retries."""
import json,math,time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from hdas_msg.msg import ControllerSignalStamped
from r1pro_waypoint_nav.core import OdomIntegrator

def main():
    rclpy.init();node=Node('r1pro_single_backward_test');rows=[];last=None;velocity=None;integrator=OdomIntegrator(.15)
    pub=None;started=None;reason='preflight';sent=0;mode=[None,0.]
    def controller(msg):mode[:]=[int(msg.data.mode),time.monotonic()]
    node.create_subscription(ControllerSignalStamped,'/controller',controller,qos_profile_sensor_data)
    def feedback(msg):
        nonlocal last,velocity
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age=node.get_clock().now().nanoseconds/1e9-stamp
        v=(msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z)
        if not all(math.isfinite(x) for x in v) or not -.02<=age<=.15:return
        last=time.monotonic();velocity=v
        if started is not None:integrator.update(stamp,v)
        rows.append(dict(stamp=stamp,t=last,v=v,age=age))
    node.create_subscription(TwistStamped,'/motion_control/chassis_speed',feedback,qos_profile_sensor_data)
    def send(vx):
        msg=TwistStamped();msg.header.stamp=node.get_clock().now().to_msg();msg.header.frame_id='base_link';msg.twist.linear.x=vx;pub.publish(msg)
    try:
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
            if (len(rows)>=50 and last is not None and time.monotonic()-last<.15
                    and mode[0] is not None and time.monotonic()-mode[1]<.2):break
        if node.count_publishers('/motion_target/target_speed_chassis')!=0:raise RuntimeError('Another speed controller exists')
        if mode[0]!=5 or time.monotonic()-mode[1]>.2:raise RuntimeError('Fresh ECU mode 5 required')
        if len(rows)<20 or last is None or time.monotonic()-last>.15:raise RuntimeError('Feedback missing/stale')
        if any(math.hypot(*r['v'][:2])>.01 or abs(r['v'][2])>.03 for r in rows[-20:]):raise RuntimeError('Robot not stationary')
        pub=node.create_publisher(TwistStamped,'/motion_target/target_speed_chassis',qos_profile_sensor_data)
        deadline=time.monotonic()+2
        while pub.get_subscription_count()==0 and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.02)
        if pub.get_subscription_count()==0:raise RuntimeError('No control subscriber')
        started=time.monotonic();reason='one_second_limit';next_send=started
        while time.monotonic()-started<1.:
            now=time.monotonic()
            if mode[0]!=5 or now-mode[1]>.2:reason='ECU_mode_lost';break
            if now-last>.15:reason='feedback_timeout';break
            if integrator.fault:reason=integrator.fault;break
            if velocity[0]>.015 or abs(velocity[1])>.025 or abs(velocity[2])>.10 or abs(velocity[0])>.08:reason='unexpected_velocity';break
            if math.hypot(*integrator.pose[:2])>=.04 or abs(integrator.pose[2])>.035:reason='displacement_limit';break
            if node.count_publishers('/motion_target/target_speed_chassis')>1:reason='competing_controller';break
            if now>=next_send:send(-.03);sent+=1;next_send=now+.05
            rclpy.spin_once(node,timeout_sec=.005)
    except Exception as e:reason=str(e)
    finally:
        elapsed=0 if started is None else time.monotonic()-started
        if pub is not None:
            deadline=time.monotonic()+2;next_send=0
            while time.monotonic()<deadline:
                if time.monotonic()>=next_send:send(0.);next_send=time.monotonic()+.05
                rclpy.spin_once(node,timeout_sec=.005)
        fresh=last is not None and time.monotonic()-last<.15
        stopped=bool(fresh and velocity is not None and math.hypot(*velocity[:2])<.01 and abs(velocity[2])<.03)
        result=dict(stop_reason=reason,nonzero_messages=sent,command_window_s=elapsed,command_vx=-.03,
            feedback_pose_including_stop=integrator.pose,integration_fault=integrator.fault,final_velocity=velocity,
            stopped_feedback=stopped,physical_direction_confirmed=False)
        path=Path(__file__).resolve().parents[1]/'reports'/('backward_pulse_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
        path.write_text(json.dumps(dict(summary=result,feedback=rows),indent=2))
        print(json.dumps(result,indent=2));print(path)
        node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
