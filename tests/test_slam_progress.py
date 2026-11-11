"""Exercise manager callbacks without creating a ROS node or communicating."""
import math
from types import SimpleNamespace as S
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from r1pro_waypoint_nav.slam_manager import SlamManager
from r1pro_waypoint_nav.scan_gate import ScanGate
import time


def test_republished_map_is_not_content_progress():
    obj=S(generation=0,state='mapping',map_messages=0,map_content_updates=0,
          map_digest=None,anchor=S(map_from_slam=None))
    msg=OccupancyGrid();msg.info.width=2;msg.info.height=1
    msg.info.resolution=.05;msg.info.origin.orientation.w=1.;msg.data=[-1,0]
    SlamManager.accept_map(obj,msg,0)
    msg.header.stamp.sec=10
    SlamManager.accept_map(obj,msg,0)
    assert obj.map_messages==2 and obj.map_content_updates==1
    msg.data=[100,0]
    SlamManager.accept_map(obj,msg,0)
    assert obj.map_content_updates==2
    SlamManager.accept_map(obj,msg,1)
    assert obj.map_messages==3


def test_manager_relays_pure_rotation_but_not_stationary_scans():
    published=[];gate=ScanGate();gate.observe_accepted((0,0,0),1)
    q=S(x=0.,y=0.,z=0.,w=1.)
    transform=S(transform=S(translation=S(x=0.,y=0.),rotation=q))
    obj=S(state='mapping',started=time.monotonic()-5,scan_gate=gate,
          relayed_scans=0,private_prefix='/offline',count_publishers=lambda _:1,
          scan_relay=S(get_subscription_count=lambda:1,publish=published.append),
          buffer=S(lookup_transform=lambda *args:transform))
    scan=LaserScan();scan.range_min=.1;scan.range_max=10.;scan.ranges=[1.]
    SlamManager.scan(obj,scan)
    assert not published
    q.z=math.sin(.12/2);q.w=math.cos(.12/2)
    SlamManager.scan(obj,scan)
    assert len(published)==1 and obj.relayed_scans==1
    gate.observe_accepted((0,0,.12),2)
    SlamManager.scan(obj,scan)
    assert len(published)==1
