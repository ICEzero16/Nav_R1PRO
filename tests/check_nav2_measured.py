"""Isolated-domain Nav2 test on saved map; synthetic sensors, no hardware bridge."""
import os
if os.environ.get('ROS_DOMAIN_ID')!='178':raise RuntimeError('Only ROS_DOMAIN_ID=178 is permitted')
import json,math,signal,subprocess,time
from pathlib import Path
from array import array
import rclpy
from rclpy.action import ActionClient,get_action_names_and_types
from rclpy.qos import QoSProfile,DurabilityPolicy,ReliabilityPolicy
from geometry_msgs.msg import TransformStamped,Twist,PoseStamped
from nav_msgs.msg import Odometry,OccupancyGrid
from sensor_msgs.msg import PointCloud2,PointField
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import ComputePathToPose,NavigateToPose
from tf2_ros import TransformBroadcaster

root=Path(__file__).resolve().parents[1];report={};process=None;goal_handle=None
dest=root/'reports/nav2_measured_check';dest.mkdir(exist_ok=True)
rclpy.init();node=rclpy.create_node('isolated_nav2_measured_check')
tf=TransformBroadcaster(node);commands=[];maps=[];obstacle=[False]
cloud=node.create_publisher(PointCloud2,'/r1pro_nav/points_filtered',10)
odom=node.create_publisher(Odometry,'/r1pro_nav/odom_fused',10)
sub=node.create_subscription(Twist,'/r1pro_nav/nav2/cmd_vel_safe',lambda m:commands.append((time.monotonic(),m.linear.x,m.linear.y,m.angular.z)),10)
latched=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL,reliability=ReliabilityPolicy.RELIABLE)
msub=node.create_subscription(OccupancyGrid,'/global_costmap/costmap',maps.append,latched)
def tick():
    stamp=node.get_clock().now().to_msg()
    for parent,child in [('map','odom'),('odom','base_link')]:
        t=TransformStamped();t.header.stamp=stamp;t.header.frame_id=parent;t.child_frame_id=child;t.transform.rotation.w=1.;tf.sendTransform(t)
    o=Odometry();o.header.stamp=stamp;o.header.frame_id='odom';o.child_frame_id='base_link';o.pose.pose.orientation.w=1.;odom.publish(o)
    import struct
    points=[(.3,.01*i,.3) for i in range(8)] if obstacle[0] else [(2.,2.,.3)]
    m=PointCloud2();m.header.stamp=stamp;m.header.frame_id='base_link';m.height=1;m.width=len(points);m.point_step=12;m.row_step=m.width*12
    m.fields=[PointField(name=k,offset=i*4,datatype=7,count=1) for i,k in enumerate(('x','y','z'))]
    m.data=array('B',b''.join(struct.pack('<fff',*p) for p in points));cloud.publish(m)
timer=node.create_timer(.05,tick)
def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)
def wait(future,seconds=10):
    rclpy.spin_until_future_complete(node,future,timeout_sec=seconds)
    if not future.done():raise RuntimeError('Action/service timeout')
    return future.result()
def pose(x,y):
    p=PoseStamped();p.header.frame_id='map';p.header.stamp=node.get_clock().now().to_msg();p.pose.position.x=x;p.pose.position.y=y;p.pose.orientation.w=1.;return p
try:
    log=(dest/'launch.log').open('w')
    process=subprocess.Popen(['ros2','launch','r1pro_waypoint_nav','navigation.launch.py','localization:=external',
        'map:='+str(root/'maps/rotation_trial_20261110_204505/map.yaml')],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    names=['map_server','planner_server','controller_server','velocity_smoother','collision_monitor','bt_navigator']
    clients={name:node.create_client(GetState,'/'+name+'/get_state') for name in names};states=set();end=time.monotonic()+40
    while time.monotonic()<end and len(states)<len(names):
        for name,c in clients.items():
            if name not in states and c.service_is_ready():
                f=c.call_async(GetState.Request());rclpy.spin_until_future_complete(node,f,timeout_sec=.3)
                if f.done() and f.result().current_state.id==3:states.add(name)
        spin(.1)
    report['active_nodes']=sorted(states)
    assert len(states)==len(names),'Lifecycle startup failed'
    spin(3)
    planner=ActionClient(node,ComputePathToPose,'/compute_path_to_pose')
    assert planner.wait_for_server(timeout_sec=3)
    req=ComputePathToPose.Goal();req.start=pose(0.,0.);req.goal=pose(.5,.2);req.use_start=True;req.planner_id='GridBased'
    handle=wait(planner.send_goal_async(req));assert handle.accepted
    result=wait(handle.get_result_async())
    report['insufficient_clearance_goal_rejected']=result.status==6 and not result.result.path.poses
    assert report['insufficient_clearance_goal_rejected']
    req.goal=pose(-.5,.2)
    handle=wait(planner.send_goal_async(req));assert handle.accepted
    result=wait(handle.get_result_async())
    report['planner_status']=result.status;report['path_points']=len(result.result.path.poses)
    assert result.status==4 and len(result.result.path.poses)>2
    (dest/'path.json').write_text(json.dumps([[p.pose.position.x,p.pose.position.y] for p in result.result.path.poses]))
    actions=dict(get_action_names_and_types(node));report['actions']=actions
    print('ACTION_GRAPH',json.dumps(actions),flush=True)
    action_name='/r1pro_nav/navigate_to_pose' if '/r1pro_nav/navigate_to_pose' in actions else '/navigate_to_pose'
    report['navigation_action']=action_name
    navigator=ActionClient(node,NavigateToPose,action_name)
    deadline=time.monotonic()+3
    while not navigator.server_is_ready() and time.monotonic()<deadline:spin(.05)
    assert navigator.server_is_ready()
    goal=NavigateToPose.Goal();goal.pose=pose(-.5,.2);goal_handle=wait(navigator.send_goal_async(goal));assert goal_handle.accepted
    spin(3)
    profile=json.loads((root/'src/r1pro_waypoint_nav/config/navigation_profile.json').read_text())
    report['bounded_private_commands']=bool(commands) and all(math.hypot(x,y)<=profile['linear_speed']+.0001 and abs(w)<=profile['yaw_speed']+.0001 for _,x,y,w in commands)
    report['nonzero_private_commands']=sum(math.hypot(x,y)>1e-4 or abs(w)>1e-4 for _,x,y,w in commands)
    assert report['bounded_private_commands'] and report['nonzero_private_commands']>0
    obstacle[0]=True;spin(1.2)
    recent=[c for c in commands if c[0]>time.monotonic()-.4]
    report['collision_stop']=bool(recent) and all(abs(x)+abs(y)+abs(w)<1e-8 for _,x,y,w in recent)
    assert report['collision_stop']
    wait(goal_handle.cancel_goal_async(),3);goal_handle=None
    report['hardware_publishers']=node.count_publishers('/motion_target/target_speed_chassis')
    assert report['hardware_publishers']==0
    assert maps
    m=maps[-1];report['inflated_map_cells']=sum(0<v<100 for v in m.data)
    report['unknown_cells']=sum(v<0 for v in m.data)
    assert report['inflated_map_cells']>0 and report['unknown_cells']>0
    (dest/'costmap.json').write_text(json.dumps(dict(width=m.info.width,height=m.info.height,resolution=m.info.resolution,
        origin=[m.info.origin.position.x,m.info.origin.position.y],data=list(m.data))))
    report['passed']=True
finally:
    if goal_handle:
        try:wait(goal_handle.cancel_goal_async(),3)
        except Exception:pass
    if process and process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:process.wait(timeout=10)
        except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGTERM);process.wait(timeout=5)
    (dest/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2),flush=True)
    node.destroy_node();rclpy.shutdown()
