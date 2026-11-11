"""Low-speed angle controller for an explicitly authorized bounded turn."""
import math


def angular_command(angle,target,previous,dt):
    if not all(math.isfinite(v) for v in (angle,target,previous,dt)) or dt<=0:
        raise ValueError('Invalid rotation state')
    direction=1. if target>0 else -1.
    remaining=abs(target)-direction*angle
    if remaining<=math.radians(.4):return 0.
    desired=direction*min(.12,max(.025,.8*remaining))
    step=.10*min(dt,.1)
    return max(previous-step,min(previous+step,desired))
