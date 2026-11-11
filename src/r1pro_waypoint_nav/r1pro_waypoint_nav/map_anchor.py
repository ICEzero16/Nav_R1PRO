"""Planar frame algebra for clearing SLAM without redefining map (0,0)."""
import math


def compose(a,b):
    x,y,t=a;u,v,s=b;c=math.cos(t);d=math.sin(t)
    return (x+c*u-d*v,y+d*u+c*v,math.atan2(math.sin(t+s),math.cos(t+s)))


def inverse(a):
    x,y,t=a;c=math.cos(t);s=math.sin(t)
    return (-c*x-s*y,s*x-c*y,-t)


def pose_tuple(p):
    q=p.orientation
    result=(p.position.x,p.position.y,math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)))
    if not all(math.isfinite(v) for v in result):raise ValueError('Nonfinite pose')
    return result


class MapAnchor:
    def __init__(self):
        self.map_from_odom=None
        self.map_from_slam=None
        self.retained=None

    def begin_reset(self):
        if self.map_from_odom is None:raise ValueError('Map alignment unavailable')
        self.retained=self.map_from_odom
        self.map_from_slam=None

    def observe(self,slam_from_base,odom_from_base):
        if self.map_from_slam is None:
            self.map_from_slam=(0.,0.,0.) if self.retained is None else compose(
                compose(self.retained,odom_from_base),inverse(slam_from_base))
        self.map_from_odom=compose(compose(self.map_from_slam,slam_from_base),inverse(odom_from_base))
        return self.map_from_odom
