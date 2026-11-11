"""Check alignment of time-matched laser endpoints against the current occupancy grid."""
import math
import time
import numpy as np
from scipy.ndimage import distance_transform_edt
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import LaserScan
from rclpy.time import Time
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformException


class MapAlignment:
    def __init__(self, node):
        self.node = node
        self.score = self.sample_time = 0.
        self.latest = None
        self.revision = None
        node.create_subscription(LaserScan, '/r1pro_nav/scan_raw', self.receive, qos_profile_sensor_data)
        node.create_timer(.4, self.measure)

    def receive(self, scan):
        self.latest = scan

    def measure(self):
        scan = self.latest
        if scan is None:
            return
        with self.node.lock:
            meta, grid = self.node.maps.meta, self.node.grid
        if grid is None or meta['source'] != 'live':
            return
        stamp = Time.from_msg(scan.header.stamp)
        if abs((self.node.get_clock().now()-stamp).nanoseconds) > 500_000_000:
            return
        try:
            t = self.node.buffer.lookup_transform('map',scan.header.frame_id,stamp).transform
        except TransformException:
            return
        if self.revision != meta['revision']:
            occupancy = np.asarray(grid).reshape(meta['height'],meta['width'])
            self.dist = distance_transform_edt(occupancy < 65) * meta['resolution']
            self.revision = meta['revision']
        r = np.asarray(scan.ranges)[::3]
        a = scan.angle_min + np.arange(len(r))*3*scan.angle_increment
        ok = np.isfinite(r) & (r>=scan.range_min) & (r<scan.range_max)
        r,a = r[ok],a[ok]
        if len(r)<20:
            return
        q=t.rotation
        xyz=Rotation.from_quat([q.x,q.y,q.z,q.w]).apply(np.c_[r*np.cos(a),r*np.sin(a),r*0])
        ox,oy,angle=meta['origin']
        dx=xyz[:,0]+t.translation.x-ox
        dy=xyz[:,1]+t.translation.y-oy
        c,s=math.cos(angle),math.sin(angle)
        x=np.floor((c*dx+s*dy)/meta['resolution']).astype(int)
        y=np.floor((-s*dx+c*dy)/meta['resolution']).astype(int)
        inside=(x>=0)&(x<meta['width'])&(y>=0)&(y<meta['height'])
        self.score=float(np.count_nonzero(self.dist[y[inside],x[inside]]<=.20)/len(x))
        self.sample_time=time.monotonic()
