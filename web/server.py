#!/usr/bin/env python3
"""Loopback map dashboard. Explicit map-reset endpoint; no motion publishers."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import signal
import secrets
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data,QoSProfile,DurabilityPolicy,ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, LaserScan, Imu
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from std_srvs.srv import Trigger
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException
from map_data import MapStore
from plan_preview import PlanPreview
from web_drive import WebDrive


class Dashboard(Node):
    def __init__(self, enable_drive=False):
        super().__init__('r1pro_map_dashboard')
        self.lock=threading.Lock();self.data={};self.rates={};self.seen={};self.trail=deque(maxlen=1200)
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.cloud_last={};self.last_trail=0.
        self.maps=MapStore();self.maps.load_saved(Path(__file__).resolve().parents[1]/'maps')
        self.reset_token=secrets.token_urlsafe(32);self.reset_future=None;self.reset_lock=threading.Lock()
        self.reset_client=self.create_client(Trigger,'/r1pro_nav/slam/reset_map')
        self.preview = PlanPreview(self)
        self.drive = WebDrive(self) if enable_drive else None
        geometry_file = Path(__file__).resolve().parents[1] / 'reports/navigation_geometry.json'
        self.footprint = json.loads(geometry_file.read_text())['footprint']
        self.grid = None
        self.map_trail=deque(maxlen=1200);self.slam_status={};self.slam_seen=0.;self.robot_map=None
        latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid,'/map',self.occupancy,latched)
        self.create_subscription(String,'/r1pro_nav/slam/status',self.slam_state,latched)
        self.create_timer(.2,self.map_pose)
        self.subscriptions_owned=[]
        self.sensor_requested=0.
        self.sensor_specs=[
            (PointCloud2,'/r1pro_nav/points_denoised',self.cloud),
            (PointCloud2,'/r1pro_nav/points_filtered',lambda m:self.cloud(m,'cloud_tf')),
            (PointCloud2,'/r1pro_nav/points_outliers',lambda m:self.cloud(m,'cloud_outliers')),
            (PointCloud2,'/hdas/lidar_chassis_left',lambda m:self.cloud(m,'cloud_raw')),
            (PointCloud2,'/r1pro_nav/points_removed',lambda m:self.cloud(m,'cloud_removed')),
            (LaserScan,'/r1pro_nav/scan',self.scan),
            (Imu,'/livox/imu',lambda m:self.imu(m,'livox_imu')),
            (Imu,'/hdas/imu_chassis',lambda m:self.imu(m,'chassis_imu')),
            (TwistStamped,'/motion_control/chassis_speed',self.speed),
            (Odometry,'/r1pro_nav/odom',self.odom),
            (String,'/r1pro_nav/status',self.status),
            (String,'/r1pro_nav/self_filter/status',lambda m:self.json_message(m,'filter')),
            (String,'/r1pro_nav/self_filter/boxes',lambda m:self.json_message(m,'boxes'))]
        self.create_timer(.5,self.sensor_subscriptions)

    def sensor_subscriptions(self):
        # Map-only viewing must not decode five extra pointcloud streams.
        wanted=time.monotonic()-self.sensor_requested<10
        if wanted and not self.subscriptions_owned:
            self.subscriptions_owned=[self.create_subscription(t,p,cb,qos_profile_sensor_data) for t,p,cb in self.sensor_specs]
        elif not wanted and self.subscriptions_owned:
            for sub in self.subscriptions_owned:self.destroy_subscription(sub)
            self.subscriptions_owned=[]

    def occupancy(self,msg):
        if msg.header.frame_id!='map':return
        q=msg.info.origin.orientation;p=msg.info.origin.position
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        try:
            with self.lock:
                self.maps.put(msg.info.width,msg.info.height,msg.info.resolution,(p.x,p.y,yaw),msg.data,
                    'live','实时地图',msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
                self.grid = msg.data
        except ValueError as exc:self.get_logger().warning(str(exc))

    def slam_state(self,msg):
        try:status=json.loads(msg.data)
        except ValueError:return
        with self.lock:
            if status.get('session')!=self.slam_status.get('session') or (status.get('state')=='resetting' and self.slam_status.get('state')!='resetting'):
                self.map_trail.clear()
            self.slam_status=status;self.slam_seen=time.monotonic()

    def map_pose(self):
        try:
            tf=self.buffer.lookup_transform('map','base_link',Time())
            age=self.get_clock().now().nanoseconds/1e9-(tf.header.stamp.sec+tf.header.stamp.nanosec/1e9)
            if not -.1<=age<.8:raise ValueError('stale')
            p=tf.transform.translation;q=tf.transform.rotation
            pose=dict(x=p.x,y=p.y,yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
            with self.lock:
                self.robot_map=pose
                # A disk map belongs to an older session; never overlay current TF on it.
                if self.maps.meta and self.maps.meta['source']=='live':self.map_trail.append([p.x,p.y])
        except (TransformException,ValueError):
            with self.lock:self.robot_map=None

    def map_snapshot(self):
        with self.lock:
            current=self.maps.snapshot();status=dict(self.slam_status)
            fresh=time.monotonic()-self.slam_seen<2
            live=current is not None and current['source']=='live'
            allowed=bool(live and fresh and status.get('ready') and self.reset_client.service_is_ready())
            return dict(map=current,slam=status,slam_fresh=fresh,reset_allowed=allowed,
                        drive=self.drive.snapshot() if self.drive else None,
                        footprint=self.footprint,planner_ready=self.preview.client.server_is_ready(),
                        reset_token=self.reset_token,robot=self.robot_map if live else None,
                        trail=list(self.map_trail) if live else [],server_time=time.time())

    def clear_map_trail(self):
        with self.lock:
            self.map_trail.clear()
        return 200,dict(message='行驶轨迹已清除，后续轨迹将从当前位置重新记录。')

    def request_reset(self,body):
        with self.reset_lock:
            state=self.map_snapshot()
            if not state['reset_allowed']:return 409,dict(error='当前没有可重置的实时建图，或数据已过期。')
            if body.get('session')!=state['slam'].get('session') or body.get('generation')!=state['slam'].get('generation'):
                return 409,dict(error='地图会话已变化，请刷新后重试。')
            if self.reset_future is not None and not self.reset_future.done():return 409,dict(error='上一次重置请求仍在处理中，请查看状态。')
            future=self.reset_client.call_async(Trigger.Request());self.reset_future=future
            done=threading.Event();future.add_done_callback(lambda f:done.set())
            if not done.wait(3):return 202,dict(accepted=True,message='请求已提交，正在等待建图管理器回复；请勿重复操作。')
            try:result=future.result()
            except Exception:return 503,dict(error='重置接口连接失败，请检查建图状态。')
            if result.success:return 202,dict(accepted=True,message=result.message)
            return 409,dict(error=result.message)

    def mark(self,key,msg=None):
        now=time.monotonic()
        with self.lock:
            q=self.rates.setdefault(key,deque(maxlen=400));q.append(now)
            age=None
            if msg is not None and hasattr(msg,'header'):
                age=self.get_clock().now().nanoseconds/1e9-(msg.header.stamp.sec+msg.header.stamp.nanosec/1e9)
            self.seen[key]=(now,age)

    def store(self,key,value):
        with self.lock:self.data[key]=value

    def transform(self,frame,stamp):
        if frame=='base_link':return lambda p:p,'base_link'
        try:
            t=self.buffer.lookup_transform('base_link',frame,Time.from_msg(stamp))
            q=t.transform.rotation;a=t.transform.translation
            x,y,z,w=q.x,q.y,q.z,q.w
            m=((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
               (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
               (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))
            return lambda p:[sum(m[i][j]*p[j] for j in range(3))+[a.x,a.y,a.z][i] for i in range(3)],'base_link'
        except TransformException:return lambda p:p,frame

    def cloud(self,msg,key='cloud'):
        self.mark(key,msg)
        now=time.monotonic()
        if now-self.cloud_last.get(key,0)<.18:return
        self.cloud_last[key]=now
        fields={f.name:f for f in msg.fields}
        if not all(k in fields and fields[k].datatype==7 for k in ('x','y','z')):return
        total=msg.width*msg.height
        if not total:
            self.store(key,dict(points=[],frame=msg.header.frame_id,total=0,received_at=time.time()));return
        if msg.point_step<=0:return
        convert,frame=self.transform(msg.header.frame_id,msg.header.stamp)
        points=[];endian='>' if msg.is_bigendian else '<'
        for i in range(0,total,max(1,math.ceil(total/2400))):
            row,col=divmod(i,msg.width);offset=row*msg.row_step+col*msg.point_step
            try:p=[struct.unpack_from(endian+'f',msg.data,offset+fields[k].offset)[0] for k in ('x','y','z')]
            except struct.error:continue
            if all(math.isfinite(v) for v in p) and .05 < sum(v*v for v in p) < 900:
                points.append([round(v,3) for v in convert(p)])
        self.store(key,dict(points=points,frame=frame,total=total,received_at=time.time()))

    def scan(self,msg):
        self.mark('scan',msg);convert,frame=self.transform(msg.header.frame_id,msg.header.stamp)
        points=[]
        for i,r in enumerate(msg.ranges):
            if math.isfinite(r) and msg.range_min<=r<=msg.range_max:
                a=msg.angle_min+i*msg.angle_increment
                points.append([round(v,3) for v in convert([r*math.cos(a),r*math.sin(a),0.])])
        self.store('scan',dict(points=points,frame=frame,rays=len(msg.ranges)))

    def imu(self,msg,key):
        self.mark(key,msg)
        a=msg.linear_acceleration;g=msg.angular_velocity
        self.store(key,dict(accel=[a.x,a.y,a.z],gyro=[g.x,g.y,g.z],frame=msg.header.frame_id or '未填写',units='unverified'))

    def speed(self,msg):
        self.mark('speed',msg)
        self.store('speed',dict(vx=msg.twist.linear.x,vy=msg.twist.linear.y,wz=msg.twist.angular.z))

    def odom(self,msg):
        self.mark('odom',msg);p=msg.pose.pose.position;q=msg.pose.pose.orientation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        now=time.monotonic()
        with self.lock:
            self.data['odom']=dict(x=p.x,y=p.y,yaw=yaw,frame=msg.header.frame_id)
            if now-self.last_trail>.2:
                self.trail.append([round(p.x,4),round(p.y,4),round(yaw,4),time.time()]);self.last_trail=now

    def status(self,msg):
        self.mark('status')
        try:self.store('status',json.loads(msg.data))
        except ValueError:pass

    def json_message(self,msg,key):
        self.mark(key)
        try:self.store(key,json.loads(msg.data))
        except ValueError:pass

    def snapshot(self):
        now=time.monotonic()
        self.sensor_requested=now
        with self.lock:
            health={}
            for key,(seen,age) in self.seen.items():
                times=[t for t in self.rates[key] if now-t<3]
                hz=(len(times)-1)/(times[-1]-times[0]) if len(times)>1 else 0.
                fresh=now-seen<(2.5 if key=='status' else .8) and (age is None or -.05<=age<=.5)
                health[key]=dict(age=round(now-seen,3),stamp_age=age,hz=round(hz,1),fresh=fresh)
            return dict(server_time=time.time(),mode='read_only',health=health,data=dict(self.data),trail=list(self.trail))


def sanitize(value):
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:sanitize(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [sanitize(v) for v in value]
    return value


def main():
    p=argparse.ArgumentParser();p.add_argument('--port',type=int,default=8088);p.add_argument('--enable-drive',action='store_true');args=p.parse_args()
    signal.signal(signal.SIGINT,signal.default_int_handler)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO);node=Dashboard(args.enable_drive)
    worker=threading.Thread(target=rclpy.spin,args=(node,),daemon=True);worker.start()
    root=Path(__file__).resolve().parent
    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*args):pass
        def do_GET(self):
            path=self.path.split('?')[0]
            if path=='/api/state':body=json.dumps(sanitize(node.snapshot()),allow_nan=False,separators=(',',':')).encode();mime='application/json'
            elif path=='/api/map':body=json.dumps(sanitize(node.map_snapshot()),allow_nan=False,separators=(',',':')).encode();mime='application/json'
            elif path.startswith('/api/maps/') and path.endswith('.png'):
                try:revision=int(path.rsplit('/',1)[1][:-4])
                except ValueError:self.send_error(404);return
                with node.lock:body=node.maps.images.get(revision)
                if body is None:self.send_error(410);return
                mime='image/png'
            elif path in ('/','/map.html','/map.js','/map.css','/map_geometry.js','/index.html','/app.js','/style.css'):
                file=root/('map.html' if path=='/' else path[1:]);body=file.read_bytes()
                mime={'html':'text/html','js':'application/javascript','css':'text/css'}[file.suffix[1:]]
            else:self.send_error(404);return
            self.send_response(200);self.send_header('Content-Type',mime+'; charset=utf-8')
            self.send_header('Content-Length',str(len(body)));self.send_header('Cache-Control','no-store')
            self.send_header('X-Content-Type-Options','nosniff')
            self.send_header('Content-Security-Policy',"default-src 'self'; style-src 'self'; script-src 'self'; connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'")
            self.end_headers()
            try:self.wfile.write(body)
            except (BrokenPipeError,ConnectionResetError):pass
        def do_POST(self):
            if self.path not in ('/api/map/reset','/api/map/trail/clear','/api/plan','/api/plan/cancel','/api/drive/start','/api/drive/stop'):self.send_error(404);return
            host=self.headers.get('Host','')
            if self.headers.get('Origin') not in ('http://'+host,'https://'+host) or host.split(':')[0] not in ('127.0.0.1','localhost'):
                self.send_error(403);return
            if not secrets.compare_digest(self.headers.get('X-Reset-Token',''),node.reset_token):self.send_error(403);return
            try:
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=2048 or self.headers.get('Content-Type','').split(';')[0]!='application/json':raise ValueError()
                body=json.loads(self.rfile.read(length))
                if not isinstance(body,dict) or body.get('confirm') is not True:raise ValueError()
            except (ValueError,TypeError):self.send_error(400);return
            try:
                if self.path.startswith('/api/drive/'):
                    if not node.drive:
                        status,payload=409,{'error':'网页处于仅预览模式'}
                    elif self.path.endswith('/start') and body.get('execute') is True:
                        status,payload=node.drive.start(str(body['ticket']))
                    elif self.path.endswith('/stop'):
                        status,payload=node.drive.cancel(str(body['ticket']))
                    else:
                        status,payload=400,{'error':'需要明确确认执行'}
                elif self.path == '/api/map/trail/clear':
                    status,payload=node.clear_map_trail()
                elif self.path == '/api/plan':
                    status,payload=node.preview.run(body)
                elif self.path == '/api/plan/cancel':
                    status,payload=node.preview.cancel(str(body['id']))
                else:
                    status,payload=node.request_reset(body)
            except (ValueError,KeyError,TypeError,OverflowError):
                status,payload=400,{'error':'请求参数无效'}
            except Exception as exc:
                node.get_logger().error(str(exc))
                status,payload=503,{'error':'服务暂时不可用'}
            encoded=json.dumps(payload).encode()
            self.send_response(status);self.send_header('Content-Type','application/json; charset=utf-8')
            self.send_header('Content-Length',str(len(encoded)));self.send_header('Cache-Control','no-store');self.end_headers()
            try:self.wfile.write(encoded)
            except (BrokenPipeError,ConnectionResetError):pass
    server=ThreadingHTTPServer(('127.0.0.1',args.port),Handler)
    print(f'R1PRO map dashboard: http://127.0.0.1:{args.port}',flush=True)
    try:server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN);server.server_close()
        if node.drive:
            node.drive.cancel()
            time.sleep(.65)
        if rclpy.ok():rclpy.shutdown()
        worker.join(timeout=3);node.destroy_node()


if __name__=='__main__':main()
