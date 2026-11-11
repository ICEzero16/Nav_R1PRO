import argparse
import json
import math
import os
from pathlib import Path
import tempfile
import time
import yaml
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException
from .core import validate_points


def atomic_save(path, data):
    path.parent.mkdir(parents=True,exist_ok=True)
    fd,tmp=tempfile.mkstemp(dir=path.parent,prefix='.waypoints-')
    try:
        with os.fdopen(fd,'w') as f:yaml.safe_dump(data,f,allow_unicode=True,sort_keys=False)
        os.replace(tmp,path)
    finally:
        if os.path.exists(tmp):os.unlink(tmp)


def main():
    parser=argparse.ArgumentParser(description='Named waypoints; go defaults to printing the goal only.')
    parser.add_argument('command',choices=['list','save','delete','go'])
    parser.add_argument('name',nargs='?')
    parser.add_argument('--file',required=True)
    parser.add_argument('--map-id',required=True,help='Use a distinct ID for every map revision')
    parser.add_argument('--replace',action='store_true')
    parser.add_argument('--send-preview',action='store_true',help='Send to Nav2; the configured stack routes velocity only to preview topics')
    parser.add_argument('--action-name',default='/navigate_to_pose',help='Verified Humble NavigateToPose action name')
    parser.add_argument('--timeout',type=float,default=120.)
    args=parser.parse_args()
    if not 0 < args.timeout <= 600:parser.error('timeout must be in (0,600] seconds')
    if args.command!='list' and not args.name:parser.error('name required')
    path=Path(args.file).expanduser()
    data=yaml.safe_load(path.read_text()) if path.exists() else {'map_id':args.map_id,'frame_id':'map','points':{}}
    validate_points(data,args.map_id)
    if args.command=='list':print(yaml.safe_dump(data,allow_unicode=True));return
    if args.command=='delete':
        if args.name not in data['points']:raise ValueError('Unknown waypoint')
        del data['points'][args.name];atomic_save(path,data);return
    if args.command=='go':
        point=data['points'][args.name]
        if not args.send_preview:print(json.dumps({'dry_run':True,'map_id':args.map_id,'goal':point}));return
    rclpy.init();n=Node('r1pro_waypoint_client');buffer=Buffer();listener=TransformListener(buffer,n)
    goal_handle=None
    try:
        deadline=time.monotonic()+5;tf=None
        while time.monotonic()<deadline:
            rclpy.spin_once(n,timeout_sec=.1)
            try:
                tf=buffer.lookup_transform('map','base_link',Time())
                age=n.get_clock().now().nanoseconds/1e9-(tf.header.stamp.sec+tf.header.stamp.nanosec/1e9)
                if not 0 <= age <= .5:tf=None;continue
                break
            except TransformException:pass
        if tf is None:raise RuntimeError('Fresh map -> base_link transform unavailable')
        if args.command=='save':
            if args.name in data['points'] and not args.replace:raise ValueError('Point exists; use --replace explicitly')
            q=tf.transform.rotation;p=tf.transform.translation
            point={'x':p.x,'y':p.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}
            data['points'][args.name]=point;validate_points(data,args.map_id);atomic_save(path,data)
            print('Saved',args.name,point);return
        client=ActionClient(n,NavigateToPose,args.action_name)
        if not client.wait_for_server(timeout_sec=5):raise RuntimeError('Preview Nav2 action unavailable')
        pose=PoseStamped();pose.header.frame_id='map';pose.header.stamp=n.get_clock().now().to_msg()
        pose.pose.position.x=float(point['x']);pose.pose.position.y=float(point['y'])
        pose.pose.orientation.z=math.sin(point['yaw']/2);pose.pose.orientation.w=math.cos(point['yaw']/2)
        goal=NavigateToPose.Goal();goal.pose=pose
        future=client.send_goal_async(goal)
        rclpy.spin_until_future_complete(n,future,timeout_sec=5)
        if not future.done():raise RuntimeError('Goal acknowledgement timed out; inspect preview server before retry')
        goal_handle=future.result()
        if not goal_handle.accepted:raise RuntimeError('Goal rejected')
        result=goal_handle.get_result_async();deadline=time.monotonic()+args.timeout
        while not result.done() and time.monotonic()<deadline:rclpy.spin_once(n,timeout_sec=.2)
        if not result.done():raise TimeoutError('Navigation timeout; cancellation requested below')
        print('Nav2 result status:',result.result().status,'(4=success, 5=canceled, 6=aborted)')
        goal_handle=None
    except KeyboardInterrupt:print('Interrupted; requesting cancellation')
    finally:
        if goal_handle is not None and goal_handle.accepted:
            cancel=goal_handle.cancel_goal_async();rclpy.spin_until_future_complete(n,cancel,timeout_sec=3)
            print('Cancellation acknowledgement:',bool(cancel.done() and cancel.result().goals_canceling))
        n.destroy_node()
        if rclpy.ok():rclpy.shutdown()
