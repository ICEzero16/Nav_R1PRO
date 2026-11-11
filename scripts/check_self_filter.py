"""Check timestamp-matched real clouds, byte preservation, and box membership."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import json
import time
from pathlib import Path
from collections import OrderedDict,Counter
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener
from r1pro_waypoint_nav.box_geometry import unpack_cloud,invalid_points,tf_matrix

rclpy.init();n=rclpy.create_node('self_filter_validation');buffer=Buffer();listener=TransformListener(buffer,n)
frames=OrderedDict();status=[];subs=[]
def receive(msg,key):
    if key=='boxes':
        value=json.loads(msg.data);stamp=round(value['stamp']*1e9)
    else:value=msg;stamp=msg.header.stamp.sec*1000000000+msg.header.stamp.nanosec
    # JSON seconds are rounded to sub-microsecond precision. Match by nearest microsecond.
    stamp=round(stamp/1000)
    frames.setdefault(stamp,{})[key]=value
    while len(frames)>40:frames.popitem(last=False)
for key,topic,typ in [('raw','/hdas/lidar_chassis_left',PointCloud2),('kept','/r1pro_nav/points_filtered',PointCloud2),('removed','/r1pro_nav/points_removed',PointCloud2),('boxes','/r1pro_nav/self_filter/boxes',String)]:
    subs.append(n.create_subscription(typ,topic,lambda m,key=key:receive(m,key),qos_profile_sensor_data))
subs.append(n.create_subscription(String,'/r1pro_nav/self_filter/status',lambda m:status.append(json.loads(m.data)),10))
end=time.monotonic()+12
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.05)
results=[]
for stamp,items in list(frames.items())[-20:]:
    if set(items)!={'raw','kept','removed','boxes'}:continue
    raw=items['raw'];kept=items['kept'];removed=items['removed'];boxes=items['boxes']['boxes']
    rb,xyz=unpack_cloud(raw);kb,kxyz=unpack_cloud(kept)
    if removed.width:db,dxyz=unpack_cloud(removed)
    else:db=np.empty((0,raw.point_step),dtype=np.uint8);dxyz=np.empty((0,3))
    t=tf_matrix(buffer.lookup_transform('base_link',raw.header.frame_id,Time.from_msg(raw.header.stamp)).transform)
    def membership(points,epsilon):
        pts=points@t[:3,:3].T+t[:3,3];inside=np.zeros(len(pts),dtype=bool)
        for box in boxes:
            c=np.asarray(box['corners']);axes=np.asarray([c[4]-c[0],c[2]-c[0],c[1]-c[0]])
            extent=np.linalg.norm(axes,axis=1);local=(pts-c[0])@(axes/extent[:,None]).T
            inside|=np.all((local>=-epsilon)&(local<=extent+epsilon),axis=1)
        return inside
    valid=~invalid_points(xyz)
    original=Counter(bytes(row) for row in rb[valid]);outputs=Counter(bytes(row) for row in kb)+Counter(bytes(row) for row in db)
    fields=lambda m:[(f.name,f.offset,f.datatype,f.count) for f in m.fields]
    results.append(dict(input=raw.width*raw.height,kept=kept.width,removed=removed.width,invalid=int((~valid).sum()),
        records_preserved=original==outputs,fields_preserved=fields(raw)==fields(kept)==fields(removed),
        stamps_preserved=raw.header==kept.header==removed.header,
        removed_inside_boxes=bool(membership(dxyz,1e-4).all()),kept_outside_boxes=bool((~membership(kxyz,-1e-4)).all())))
    if len(results)>=5:break
report={'frames':results,'latest_status':status[-1] if status else {},'hardware_speed_publishers':n.count_publishers('/motion_target/target_speed_chassis'),'hardware_brake_publishers':n.count_publishers('/motion_target/brake_mode')}
report['passed']=len(results)>=3 and all(all(v for k,v in row.items() if isinstance(v,bool)) for row in results) and report['hardware_speed_publishers']==0 and report['hardware_brake_publishers']==0
path=Path(__file__).resolve().parents[1]/'reports/self_filter_validation.json';path.write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))
n.destroy_node();rclpy.shutdown()
if not report['passed']:raise SystemExit(1)
