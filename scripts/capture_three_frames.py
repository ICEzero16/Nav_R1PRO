"""Capture exactly three PointCloud2 messages; stop owned lidar before offline filtering."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import copy
import json
from pathlib import Path
import signal
import subprocess
import time
from array import array
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.serialization import serialize_message
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer,TransformListener,TransformException
from ament_index_python.packages import get_package_share_directory
from r1pro_waypoint_nav.box_geometry import build_boxes,tf_matrix,unpack_cloud,invalid_points,box_mask,corners

root=Path(__file__).resolve().parents[1]
out=root/'captures'/time.strftime('three_frames_%Y%m%d_%H%M%S');out.mkdir(parents=True)
rclpy.init();node=Node('capture_three_frames_only');buffer=Buffer();listener=TransformListener(buffer,node)
client=node.create_client(GetParameters,'/robot_state_publisher/get_parameters')
driver=None;sub=None;frames=[];stamps=set();stop_requested=False;log=None
def stop_driver():
    global stop_requested
    if driver is not None and driver.poll() is None and not stop_requested:
        stop_requested=True;driver.send_signal(signal.SIGINT)
def resolve(uri):
    if uri.startswith('package://'):
        pkg,rel=uri[10:].split('/',1);return Path(get_package_share_directory(pkg))/rel
    if uri.startswith('file://'):return Path(uri[7:])
    if Path(uri).is_absolute():return Path(uri)
    raise ValueError(uri)
def receive(msg):
    if len(frames)>=3:return
    stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
    if stamp in stamps:return
    stamps.add(stamp);frames.append(msg)
    print('CAPTURED',len(frames),'stamp',stamp,'points',msg.width*msg.height,flush=True)
    if len(frames)==3:stop_driver()
def pose(target,source,stamp):
    if target==source:return np.eye(4)
    return tf_matrix(buffer.lookup_transform(target,source,stamp).transform)

try:
    if subprocess.run(['pgrep','-f','/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node'],capture_output=True).returncode==0:
        raise RuntimeError('An existing lidar driver is running; refusing unbounded shared capture')
    if not client.wait_for_service(timeout_sec=5):raise RuntimeError('robot_description service unavailable')
    req=GetParameters.Request();req.names=['robot_description'];f=client.call_async(req)
    rclpy.spin_until_future_complete(node,f,timeout_sec=5)
    if not f.done():raise RuntimeError('robot_description timed out')
    xml=f.result().values[0].string_value;models=build_boxes(xml,resolve)
    (out/'robot_description.urdf').write_text(xml)
    sub=node.create_subscription(PointCloud2,'/hdas/lidar_chassis_left',receive,qos_profile_sensor_data)
    log=(out/'capture_driver.log').open('w')
    driver=subprocess.Popen(['ros2','launch','livox_ros_driver2','msg_MID360_launch.py'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    deadline=time.monotonic()+20
    while len(frames)<3 and time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        if driver.poll() is not None:break
    stop_driver()
    node.destroy_subscription(sub);sub=None
    if driver.poll() is None:
        try:driver.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(driver.pid,signal.SIGTERM);driver.wait(timeout=5)
    print('LIDAR_STOPPED',driver.returncode,flush=True)
    if len(frames)!=3:raise RuntimeError('Expected 3 frames, received '+str(len(frames)))
    for i,msg in enumerate(frames,1):(out/f'frame_{i}_raw.cdr').write_bytes(serialize_message(msg))
    # Only TF reception continues briefly. No further lidar subscription or driver remains.
    snapshots=[];deadline=time.monotonic()+5
    for msg in frames:
        stamp=Time.from_msg(msg.header.stamp)
        while True:
            rclpy.spin_once(node,timeout_sec=.02)
            try:
                base_from_cloud=pose('base_link',msg.header.frame_id,stamp);placed=[]
                for model in models:
                    shape=model;swept=False
                    try:t=pose(msg.header.frame_id,shape['frame'],stamp)
                    except TransformException:
                        if 'fallback' not in model:raise
                        shape=model['fallback'];swept=True;t=pose(msg.header.frame_id,shape['frame'],stamp)
                    placed.append(dict(name=model['name'],frame=model['frame'],swept=swept,matrix=(t@shape['matrix']).tolist(),low=shape['low'].tolist(),high=shape['high'].tolist()))
                snapshots.append(dict(base_from_cloud=base_from_cloud.tolist(),boxes=placed));break
            except TransformException:
                if time.monotonic()>deadline:raise
    (out/'tf_snapshots.json').write_text(json.dumps(snapshots))
finally:
    stop_driver()
    if driver is not None and driver.poll() is None:
        try:driver.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(driver.pid,signal.SIGKILL);driver.wait(timeout=3)
    if log:log.close()
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()

# Offline processing begins only after ROS shutdown and driver exit.
visual=[];summary=[];padding=.025
for i,(msg,snapshot) in enumerate(zip(frames,snapshots),1):
    raw,xyz=unpack_cloud(msg);invalid=invalid_points(xyz);inside=np.zeros(len(xyz),dtype=bool);boxes=[]
    base=np.asarray(snapshot['base_from_cloud'])
    for shape in snapshot['boxes']:
        mask,world=box_mask(xyz,np.asarray(shape['matrix']),np.asarray(shape['low'])-padding,np.asarray(shape['high'])+padding);inside|=mask
        boxes.append(dict(name=shape['name'],swept=shape['swept'],corners=np.round(world@base[:3,:3].T+base[:3,3],5).tolist()))
    kept=(~inside)&(~invalid);removed=inside&(~invalid)
    for name,selection in [('filtered',kept),('removed',removed)]:
        cloud=copy.deepcopy(msg);cloud.height=1;cloud.width=int(selection.sum());cloud.row_step=cloud.width*cloud.point_step;cloud.is_dense=True
        cloud.data=array('B',raw[selection].tobytes());(out/f'frame_{i}_{name}.cdr').write_bytes(serialize_message(cloud))
    base_xyz=xyz@base[:3,:3].T+base[:3,3]
    np.savez_compressed(out/f'frame_{i}_points.npz',xyz=xyz,base_xyz=base_xyz,kept=kept,removed=removed,invalid=invalid)
    info=dict(frame=i,stamp_sec=msg.header.stamp.sec,stamp_nanosec=msg.header.stamp.nanosec,input=len(xyz),kept=int(kept.sum()),removed=int(removed.sum()),invalid=int(invalid.sum()),boxes=len(boxes),padding_m=padding)
    assert info['input']==info['kept']+info['removed']+info['invalid']
    summary.append(info)
    # Display sampled environment but all removed points; full data remains in NPZ/CDR.
    environment=base_xyz[kept];environment=environment[::max(1,int(np.ceil(len(environment)/3500)))]
    visual.append(dict(**info,environment=np.round(environment,3).tolist(),self_points=np.round(base_xyz[removed],3).tolist(),model=boxes))
(out/'visualization_data.json').write_text(json.dumps(dict(frames=visual,coordinate_frame='base_link',capture_mode='exactly_three_frames_lidar_stopped'),separators=(',',':')))
(out/'summary.json').write_text(json.dumps(dict(frames=summary,captured_frames=len(frames),driver_stopped=True,ros_shutdown_before_filter=True),indent=2))
print('OUTPUT',out,flush=True)
print(json.dumps(summary),flush=True)
