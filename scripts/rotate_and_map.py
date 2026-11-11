"""Authorized +60/-60 degree turns with continuous mapping and bounded cleanup.

No extra corrective turns. Bounded recovery continues the remaining angle.
A separate 20 Hz zero-on-expiry sender
stops refreshing nonzero commands when the control loop or feedback stalls.
"""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import argparse,json,math,signal,subprocess,threading,time
from pathlib import Path
import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import OccupancyGrid,Odometry
from sensor_msgs.msg import LaserScan,Imu
from std_msgs.msg import String
from hdas_msg.msg import ControllerSignalStamped,FeedbackStatus
from slam_toolbox.srv import SerializePoseGraph
from r1pro_waypoint_nav.core import OdomIntegrator
from r1pro_waypoint_nav.rotation_control import angular_command
from r1pro_waypoint_nav.translation_control import linear_command,bounded_velocity,translation_fault
from r1pro_waypoint_nav.motion_guard import MotionGuard,load_config,pipeline_issues
from dataclasses import asdict


class VelocitySender:
    def __init__(self,node,publisher,healthy,ttl=.2):
        self.node=node;self.pub=publisher;self.healthy=healthy;self.lock=threading.Lock()
        self.value=(0.,0.);self.deadline=0.;self.running=True;self.error=None;self.nonzero=0;self.ttl=ttl
        self.thread=threading.Thread(target=self.run,name='bounded_velocity_watchdog',daemon=True);self.thread.start()
    def request(self,w,vx=0.):
        value=bounded_velocity(w,vx)
        with self.lock:self.value=value;self.deadline=time.monotonic()+self.ttl
    def run(self):
        try:
            while self.running:
                with self.lock:value=self.value if time.monotonic()<self.deadline and self.healthy() else (0.,0.)
                msg=TwistStamped();msg.header.stamp=self.node.get_clock().now().to_msg();msg.header.frame_id='base_link';msg.twist.angular.z=value[0];msg.twist.linear.x=value[1]
                self.pub.publish(msg)
                if any(value):self.nonzero+=1
                time.sleep(.05)
        except Exception as exc:self.error=repr(exc)
    def close(self):
        self.request(0.);self.running=False;self.thread.join(timeout=2)
        msg=TwistStamped();msg.header.stamp=self.node.get_clock().now().to_msg();msg.header.frame_id='base_link';self.pub.publish(msg)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--translate-roundtrip',action='store_true',help='After turns, explicitly authorized backward 20 cm then forward 20 cm')
    args=parser.parse_args()
    root=Path(__file__).resolve().parents[1];dest=root/'maps'/('rotation_trial_'+time.strftime('%Y%m%d_%H%M%S'));dest.mkdir(parents=True)
    cfg=load_config(root/'src/r1pro_waypoint_nav/config/rotation_guard.yaml')
    interrupted=[False]
    def interrupt(*args):interrupted[0]=True
    signal.signal(signal.SIGINT,interrupt);signal.signal(signal.SIGTERM,interrupt)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);n=Node('r1pro_authorized_rotation_mapping')
    seen={};counts={};latest={};feedback=[];errors=[];mode=[None,0.];active=[None];gyro=[None];raw_imu=[];trials=[]
    fault_seen=[0.];failure=None;sender=None;proc=None;log=None;map_saved=False;graph_saved=False;pub=None
    guard_events=[];status_events=[];previous_status={};rejected={'wheel':0,'imu':0}
    def receive(msg,key):
        now=time.monotonic();counts[key]=counts.get(key,0)+1;latest[key]=msg
        if key=='scan' and not any(math.isfinite(r) and msg.range_min<=r<msg.range_max for r in msg.ranges):return
        seen[key]=now
        if isinstance(msg,String):
            data=json.loads(msg.data);signature=(data.get('ready'),data.get('reason'),data.get('state'),data.get('odometry_fault'))
            if signature!=previous_status.get(key):
                status_events.append(dict(t=now,key=key,status=data));previous_status[key]=signature
    def wheel(msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9
        age=n.get_clock().now().nanoseconds/1e9-stamp;v=(msg.twist.linear.x,msg.twist.linear.y,msg.twist.angular.z)
        if not all(math.isfinite(x) for x in v) or not -.02<=age<=cfg.feedback_max_age_sec or (feedback and stamp<=feedback[-1]['stamp']):
            rejected['wheel']+=1;return
        receive(msg,'wheel');feedback.append(dict(t=time.monotonic(),stamp=stamp,v=v))
        if active[0] is not None:active[0].update(stamp,v)
    def imu(msg):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec/1e9;w=msg.angular_velocity.z
        age=n.get_clock().now().nanoseconds/1e9-stamp
        if not math.isfinite(w) or not -.02<=age<=cfg.feedback_max_age_sec or (raw_imu and stamp<=raw_imu[-1]['stamp']):
            rejected['imu']+=1;return
        receive(msg,'imu');raw_imu.append(dict(t=time.monotonic(),stamp=stamp,w=w))
        if gyro[0] is not None:
            g=gyro[0]
            if g['last'] is not None:
                dt=stamp-g['last']
                if not 0<dt<=.2:g['fault']='imu_timestamp_gap';g['gaps']+=1
                else:g['angle']+=(w+g['w'])*.5*dt
            g['last']=stamp;g['w']=w
    def controller(msg):mode[:]=[int(msg.data.mode),time.monotonic()]
    def faults(msg):
        fault_seen[0]=time.monotonic()
        errors.extend(dict(name=e.name,code=int(e.error_code)) for e in msg.errors if e.error_code)
    subscriptions=[n.create_subscription(TwistStamped,'/motion_control/chassis_speed',wheel,qos_profile_sensor_data),
        n.create_subscription(ControllerSignalStamped,'/controller',controller,qos_profile_sensor_data),
        n.create_subscription(FeedbackStatus,'/hdas/feedback_status_chassis',faults,qos_profile_sensor_data),
        n.create_subscription(Imu,'/r1pro_nav/imu_chassis_planar',imu,qos_profile_sensor_data)]
    for topic,typ,key in [('/r1pro_nav/odom_fused',Odometry,'fused'),('/r1pro_nav/scan',LaserScan,'scan'),
        ('/r1pro_nav/status',String,'adapters'),('/r1pro_nav/self_filter/status',String,'filter'),
        ('/r1pro_nav/temporal_scan/status',String,'temporal')]:
        subscriptions.append(n.create_subscription(typ,topic,lambda m,k=key:receive(m,k),qos_profile_sensor_data))
    latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
    subscriptions.append(n.create_subscription(String,'/r1pro_nav/slam/status',lambda m:receive(m,'slam'),latched))
    subscriptions.append(n.create_subscription(OccupancyGrid,'/map',lambda m:receive(m,'map'),latched))
    def hardware_ok():
        now=time.monotonic()
        return not interrupted[0] and not errors and mode[0]==5 and now-mode[1]<cfg.controller_timeout_sec and now-seen.get('wheel',0)<cfg.wheel_timeout_sec and now-fault_seen[0]<cfg.motor_status_timeout_sec
    def stationary():
        return len(feedback)>=20 and time.monotonic()-seen.get('wheel',0)<cfg.wheel_timeout_sec and all(math.hypot(*f['v'][:2])<.01 and abs(f['v'][2])<.02 for f in feedback[-15:])
    def statuses():return {k:json.loads(latest[k].data) for k in ('adapters','filter','temporal','slam') if k in latest}
    def health_issues():
        immediate,diagnostic,hard=pipeline_issues(time.monotonic(),seen,statuses(),cfg)
        if 'map' not in latest or not any(v>=0 for v in latest['map'].data):immediate.append('no_map')
        return immediate,diagnostic,hard
    def mapping_health():
        return '; '.join(reason for group in health_issues() for reason in group)
    def halt(seconds=2.):
        if sender is None:return
        sender.request(0.);end=time.monotonic()+seconds
        while time.monotonic()<end:
            sender.request(0.);rclpy.spin_once(n,timeout_sec=.01)
    def save():
        nonlocal map_saved,graph_saved
        if 'map' not in latest:return
        msg=latest['map'];grid=np.array(msg.data,dtype=np.int16).reshape(msg.info.height,msg.info.width)
        pixels=np.full(grid.shape,205,dtype=np.uint8);pixels[(grid>=0)&(grid<=25)]=254;pixels[grid>=65]=0
        (dest/'map.pgm').write_bytes(f'P5\n{msg.info.width} {msg.info.height}\n255\n'.encode()+np.flipud(pixels).tobytes())
        p=msg.info.origin.position;q=msg.info.origin.orientation;yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        (dest/'map.yaml').write_text(yaml.safe_dump(dict(image='map.pgm',resolution=float(msg.info.resolution),origin=[p.x,p.y,yaw],negate=0,occupied_thresh=.65,free_thresh=.196,mode='trinary')));map_saved=True
        if 'slam' in latest:(dest/'posegraph.anchor.json').write_text(latest['slam'].data)
        client=n.create_client(SerializePoseGraph,'/slam_toolbox/serialize_map')
        if not client.wait_for_service(timeout_sec=2):return
        req=SerializePoseGraph.Request();req.filename=str(dest/'posegraph');future=client.call_async(req)
        end=time.monotonic()+8
        while not future.done() and time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
        graph_saved=bool(future.done() and future.result() and future.result().result==0)
    try:
        end=time.monotonic()+6
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
        if not hardware_ok() or not stationary():raise RuntimeError('Hardware preflight not ready')
        if n.count_publishers('/motion_target/target_speed_chassis'):raise RuntimeError('Competing speed publisher')
        if any(name in n.get_node_names() for name in ('r1pro_slam_manager','slam_toolbox','r1pro_ekf','r1pro_nav_adapters','r1pro_self_filter','r1pro_temporal_scan')):
            raise RuntimeError('Existing mapping components: ownership conflict')
        if n.count_publishers('/hdas/lidar_chassis_left'):raise RuntimeError('Existing lidar: ownership conflict')
        log=(dest/'launch.log').open('w');proc=subprocess.Popen(['ros2','launch','r1pro_waypoint_nav','slam_trial.launch.py','start_lidar:=true'],cwd=root,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        print('PHASE mapping_warmup',str(dest),flush=True)
        deadline=time.monotonic()+40;stable_since=None
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.01)
            if interrupted[0] or errors or proc.poll() is not None:raise RuntimeError('Warmup interrupted or failed')
            healthy=hardware_ok() and stationary() and not mapping_health()
            if healthy:
                if stable_since is None:stable_since=time.monotonic()
                if time.monotonic()-stable_since>2:break
            else:stable_since=None
        else:raise RuntimeError('Mapping not stable: '+mapping_health())
        if n.count_publishers('/motion_target/target_speed_chassis'):raise RuntimeError('Competing speed publisher')
        pub=n.create_publisher(TwistStamped,'/motion_target/target_speed_chassis',qos_profile_sensor_data)
        deadline=time.monotonic()+3
        while pub.get_subscription_count()==0 and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.02)
        if not pub.get_subscription_count():raise RuntimeError('No controller subscriber')
        sender=VelocitySender(n,pub,hardware_ok,cfg.command_ttl_sec)
        initial_slam=statuses().get('slam',{})
        initial_accepted=initial_slam.get('accepted_scans',0)
        initial_content=initial_slam.get('map_content_updates',0)
        plan=[('ccw','yaw',math.pi/3),('cw','yaw',-math.pi/3)]
        if args.translate_roundtrip:plan.extend([('backward','x',-.2),('forward','x',.2)])
        for name,axis,target in plan:
            rotating=axis=='yaw'
            # Brief diagnostic jitter between legs must not discard the task.
            ready_deadline=time.monotonic()+cfg.max_pause_sec;ready_since=None
            while time.monotonic()<ready_deadline:
                sender.request(0.);rclpy.spin_once(n,timeout_sec=.01)
                if interrupted[0] or errors or mode[0]!=5:raise RuntimeError('Hardware fault before '+name)
                if health_issues()[2]:raise RuntimeError(mapping_health())
                if hardware_ok() and stationary() and not mapping_health():
                    if ready_since is None:ready_since=time.monotonic()
                    if time.monotonic()-ready_since>=cfg.recovery_stable_sec:break
                else:ready_since=None
            else:raise RuntimeError('Not ready before '+name+': '+mapping_health())
            active[0]=OdomIntegrator(cfg.wheel_max_gap_sec);gyro[0]=dict(last=None,w=0.,angle=0.,fault='',offset=0.,gaps=0,rebases=0)
            start=time.monotonic();previous_time=start;command=0.;reason='target_reached';last_print=start
            guard=MotionGuard(cfg);command_time=0.;guard_state='run';expected_session=(statuses()['slam'].get('session'),statuses()['slam'].get('generation'))
            print('PHASE',name,'target_deg' if rotating else 'target_m',math.degrees(target) if rotating else target,flush=True)
            try:
                while True:
                    rclpy.spin_once(n,timeout_sec=.005);now=time.monotonic()
                    dt=max(.0001,now-previous_time);previous_time=now
                    if interrupted[0] or errors or mode[0]!=5:raise RuntimeError('hardware_fault_or_control_mode_changed')
                    if proc.poll() is not None:raise RuntimeError('mapping_launch_exited')
                    if sender.error:raise RuntimeError(sender.error)
                    if active[0].fault:raise RuntimeError(active[0].fault)
                    if n.count_publishers('/motion_target/target_speed_chassis')!=1:raise RuntimeError('competing_controller')
                    x,y,angle=active[0].pose;v=feedback[-1]['v'];direction=1 if target>0 else -1
                    if rotating:
                        if math.hypot(*v[:2])>cfg.max_linear_speed or abs(v[2])>cfg.max_yaw_speed or math.hypot(x,y)>cfg.max_translation:raise RuntimeError('unexpected_translation_or_speed')
                        if direction*angle<-cfg.reverse_angle_rad:raise RuntimeError('unexpected_rotation_direction')
                        if abs(angle)>abs(target)+math.radians(cfg.angle_margin_deg):raise RuntimeError('angle_limit')
                    else:
                        fault=translation_fault((x,y,angle),v,target)
                        if fault:raise RuntimeError(fault)
                    if now-start>cfg.max_turn_wall_sec or command_time>cfg.max_command_time_sec:raise RuntimeError('turn_time_limit')
                    progress=angle if rotating else x
                    if command_time>cfg.no_response_sec and abs(progress)<(math.radians(cfg.no_response_deg) if rotating else .005):raise RuntimeError('no_motion_response')
                    immediate,diagnostic,hard=health_issues()
                    if not hardware_ok():immediate.append('hardware_stream_stale')
                    if guard.state=='pause' and not stationary():immediate.append('waiting_for_stopped_feedback')
                    current_session=(statuses().get('slam',{}).get('session'),statuses().get('slam',{}).get('generation'))
                    if current_session!=expected_session:hard.append('map_session_changed_during_turn')
                    # Missing auxiliary gyro samples cannot be integrated. Hold
                    # still and establish a new comparison offset from valid
                    # primary wheel odometry; report the gap, never invent gyro data.
                    if gyro[0]['fault']:
                        if guard.state=='pause' and stationary() and now-seen.get('imu',0)<cfg.feedback_max_age_sec:
                            gyro[0]['offset']=angle-gyro[0]['angle'];gyro[0]['fault']='';gyro[0]['rebases']+=1
                            guard_events.append(dict(t=now,event='imu_comparison_rebased_while_stopped',wheel_angle=angle,imu_observed_angle=gyro[0]['angle']))
                        else:immediate.append('imu_gap_waiting_for_stationary_recovery')
                    aligned_imu=gyro[0]['angle']+gyro[0]['offset']
                    if not gyro[0]['fault'] and now-seen.get('imu',0)<cfg.imu_timeout_sec:
                        if rotating and direction*aligned_imu<-cfg.reverse_angle_rad:hard.append('unexpected_imu_rotation_direction')
                        if not rotating and abs(aligned_imu)>.06:hard.append('unexpected_imu_yaw_during_translation')
                        if abs(angle-aligned_imu)>max(cfg.imu_difference_rad,cfg.imu_difference_ratio*abs(angle)):hard.append('wheel_imu_disagreement')
                    level=guard.update(now,immediate,diagnostic,hard)
                    if level!=guard_state:
                        guard_events.append(dict(t=now,leg=name,level=level,reason=guard.reason,wheel_deg=math.degrees(angle),
                            ages={k:now-v for k,v in seen.items()},status=statuses()));guard_state=level
                        print('GUARD',name,level,guard.reason,flush=True)
                    if level=='abort':raise RuntimeError(guard.reason)
                    if not guard.may_move:
                        sender.request(0.);command=0.;continue
                    if abs(target)-direction*progress<=(math.radians(.4) if rotating else .003):break
                    command_time+=dt
                    command=(angular_command if rotating else linear_command)(progress,target,command,dt)
                    sender.request(command) if rotating else sender.request(0.,vx=command)
                    if now-last_print>=2:
                        last_print=now;print('PROGRESS',name,'wheel_deg' if rotating else 'travel_cm',round(math.degrees(angle) if rotating else x*100,2),'imu_deg',round(math.degrees(gyro[0]['angle']),2),flush=True)
            except Exception as exc:reason=str(exc);raise
            finally:
                halt(2.5)
                record=dict(name=name,axis=axis,target_deg=math.degrees(target) if rotating else None,target_m=None if rotating else target,wheel_deg=math.degrees(active[0].pose[2]),imu_deg=math.degrees(gyro[0]['angle']),
                            imu_comparison_deg=math.degrees(gyro[0]['angle']+gyro[0]['offset']),imu_complete=gyro[0]['gaps']==0,
                            imu_gaps=gyro[0]['gaps'],imu_rebases=gyro[0]['rebases'],pause_count=guard.pauses,paused_sec=guard.total_pause,
                            command_time_sec=command_time,
                            translation_m=list(active[0].pose[:2]),elapsed_s=time.monotonic()-start,stop_reason=reason,stopped=stationary(),
                            odometry_fault=active[0].fault,imu_fault=gyro[0]['fault'])
                trials.append(record);print('TRIAL',json.dumps(record),flush=True);active[0]=None;gyro[0]=None
            completion_ok=abs(record['wheel_deg']-record['target_deg'])<=1.5 if rotating else abs(record['translation_m'][0]-target)<=.01 and abs(record['translation_m'][1])<=.025 and abs(math.radians(record['wheel_deg']))<=.06
            if not record['stopped'] or record['odometry_fault'] or not completion_ok:
                raise RuntimeError('Motion completion out of tolerance; no automatic retry')
        print('PHASE stopped_map_settle',flush=True)
        end=time.monotonic()+7;settle_guard=MotionGuard(cfg)
        while time.monotonic()<end:
            sender.request(0.);rclpy.spin_once(n,timeout_sec=.01)
            if not stationary() or not hardware_ok():raise RuntimeError('Post-turn hardware check failed')
            if settle_guard.update(time.monotonic(),*health_issues())=='abort':raise RuntimeError('Post-turn health check failed: '+settle_guard.reason)
        end_health=health_issues()
        if not settle_guard.may_move or end_health[0] or end_health[2]:raise RuntimeError('Map not healthy at save: '+mapping_health())
        save()
        if not map_saved or not graph_saved:raise RuntimeError('Map or pose graph save incomplete')
        final_slam=statuses().get('slam',{})
        if final_slam.get('accepted_scans',0)-initial_accepted<2:
            raise RuntimeError('Rotation finished but SLAM accepted fewer than two new keyframes')
        if final_slam.get('map_content_updates',0)<=initial_content:
            raise RuntimeError('Rotation finished but map content did not change; repeated publications are not mapping progress')
    except Exception as exc:failure=f'{type(exc).__name__}: {exc}';print('ABORT',failure,flush=True)
    finally:
        halt(2.)
        stopped=stationary()
        if sender:sender.close()
        if not map_saved:
            try:save()
            except Exception as exc:print('Partial map save failed:',repr(exc),flush=True)
        if proc and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(proc.pid,signal.SIGTERM)
                try:proc.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait(timeout=3)
        if log:log.close()
        report=dict(failure=failure,trials=trials,errors=errors,stopped=stopped,motion_messages=sender.nonzero if sender else 0,
                    guard_config=asdict(cfg),guard_events=guard_events,status_events=status_events,rejected_messages=rejected,
                    counts=counts,map_saved=map_saved,posegraph_saved=graph_saved,owned_launch_exited=proc is None or proc.poll() is not None,
                    final_feedback=feedback[-1] if feedback else None,
                    final_status={k:json.loads(latest[k].data) for k in ('adapters','filter','temporal','slam') if k in latest})
        if 'map' in latest:
            m=latest['map'];report['map']=dict(width=m.info.width,height=m.info.height,resolution=m.info.resolution,known_cells=sum(v>=0 for v in m.data))
        (dest/'report.json').write_text(json.dumps(report,indent=2));(dest/'measurements.json').write_text(json.dumps(dict(feedback=feedback,imu=raw_imu)))
        print('REPORT',str(dest),'failure',failure,'stopped',stopped,flush=True)
        n.destroy_node();rclpy.shutdown()
    return 1 if failure else 0


if __name__=='__main__':raise SystemExit(main())
