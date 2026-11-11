"""Bounded straight-line controller for an explicitly authorized 20 cm test."""
import math


def linear_command(position,target,previous,dt):
    if not all(math.isfinite(v) for v in (position,target,previous,dt)) or dt<=0 or abs(target)>.200001:
        raise ValueError('Invalid bounded translation state')
    direction=1. if target>0 else -1.
    remaining=abs(target)-direction*position
    if remaining<=.003:return 0.
    desired=direction*min(.04,max(.008,.8*remaining))
    step=.04*min(dt,.1)
    return max(previous-step,min(previous+step,desired))


def bounded_velocity(w,vx=0.):
    if not all(math.isfinite(v) for v in (w,vx)) or abs(w)>.12001 or abs(vx)>.040001 or (w!=0 and vx!=0):
        raise ValueError('Unsafe command')
    return w,vx


def translation_fault(pose,velocity,target):
    x,y,yaw=pose;vx,vy,w=velocity;direction=1 if target>0 else -1
    if not all(math.isfinite(v) for v in (*pose,*velocity)):return 'invalid_translation_feedback'
    if math.hypot(vx,vy)>.05 or abs(w)>.10:return 'unexpected_translation_speed'
    if abs(y)>.025 or abs(yaw)>.06:return 'unexpected_translation_drift'
    if direction*x<-.005:return 'unexpected_translation_direction'
    if abs(x)>abs(target)+.02:return 'translation_distance_limit'
    return ''
