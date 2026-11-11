"""Bounded, causal scan persistence and compact-cluster motion rejection.

No ROS dependency. All matching is in odom; output contains current returns only.
Unknown/occluded rays never supply free-space evidence. This is not semantic
person detection and does not erase measurements already stored by SLAM.
"""
from collections import deque
from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import cKDTree
from scipy.optimize import linear_sum_assignment


@dataclass
class TemporalConfig:
    window_sec: float = 1.2
    max_frames: int = 12
    min_hits: int = 3
    min_span_sec: float = .4
    min_frame_interval: float = .06
    max_gap_sec: float = .5
    match_radius: float = .06
    range_radius_scale: float = .005
    max_match_radius: float = .12
    clear_margin: float = .20
    cluster_gap: float = .20
    cluster_range_scale: float = .01
    cluster_min_points: int = 3
    cluster_max_width: float = 1.5
    max_tracks: int = 128
    track_timeout: float = .4
    track_max_speed: float = 2.5
    motion_min_span: float = .25
    motion_min_displacement: float = .12
    motion_min_speed: float = .20
    motion_hold_sec: float = 1.0
    max_scan_points: int = 4096

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError('Expected positive finite parameter: '+name)
        for name in ('max_frames','min_hits','cluster_min_points','max_tracks','max_scan_points'):
            if not isinstance(getattr(self,name),int):raise ValueError(name+' must be integer')
        if not 2 <= self.min_hits <= self.max_frames:raise ValueError('Invalid min_hits')
        if not self.min_frame_interval <= self.min_span_sec < self.window_sec:raise ValueError('Invalid time window')
        if self.max_match_radius < self.match_radius:raise ValueError('Invalid match radii')


class TemporalFilter:
    def __init__(self, config=None):
        self.cfg=config or TemporalConfig()
        self.reset()

    def reset(self):
        self.history=deque(maxlen=self.cfg.max_frames)
        self.tracks={}
        self.next_id=0
        self.last_stamp=None
        self.last_pose=None

    @staticmethod
    def _tree(frame, active_only=False):
        key='active_tree' if active_only else 'all_tree'
        if frame.get(key) is None:
            points=frame['points'][frame['active']] if active_only else frame['points']
            frame[key]=cKDTree(points)
        return frame[key]

    def _clusters(self, points, indices, ranges, beam_count, increment):
        if not len(points):return []
        # Isolated spikes must not fragment a wall into apparently moving
        # compact clusters. Keep their temporal votes separate from tracking.
        adjacent=np.linalg.norm(np.diff(points,axis=0),axis=1)
        close=adjacent<=self.cfg.cluster_gap+self.cfg.cluster_range_scale*np.minimum(ranges[:-1],ranges[1:])
        coherent=np.r_[False,close]|np.r_[close,False]
        selected=np.flatnonzero(coherent)
        points=points[selected];indices=indices[selected];ranges=ranges[selected]
        if not len(points):return []
        gaps=np.linalg.norm(np.diff(points,axis=0),axis=1)
        limits=self.cfg.cluster_gap+self.cfg.cluster_range_scale*np.minimum(ranges[:-1],ranges[1:])
        cuts=np.flatnonzero((np.diff(indices)>3)|(gaps>limits))+1
        groups=list(np.split(np.arange(len(points)),cuts))
        # A 360 degree scan may split one object at its angular seam.
        full_circle=abs(beam_count*increment-2*math.pi)<2*increment
        if len(groups)>1 and full_circle and indices[0]==0 and indices[-1]==beam_count-1:
            limit=self.cfg.cluster_gap+self.cfg.cluster_range_scale*min(ranges[0],ranges[-1])
            if np.linalg.norm(points[0]-points[-1])<=limit:
                groups=[np.concatenate((groups[-1],groups[0]))]+groups[1:-1]
        return [selected[g] for g in groups if len(g)>=self.cfg.cluster_min_points and
                np.linalg.norm(np.ptp(points[g],axis=0))<=self.cfg.cluster_max_width]

    def _motion(self, points, indices, ranges, beam_count, increment, stamp):
        c=self.cfg
        stationary=np.zeros(len(points),dtype=bool)
        references=[frame for frame in self.history if c.motion_min_span<=stamp-frame['stamp']<=.7 and len(frame['points'])]
        if references and len(points):
            distances,_=self._tree(references[0]).query(points,k=1,workers=1)
            stationary=distances<=np.minimum(.06,.035+.002*ranges)
        self.tracks={k:t for k,t in self.tracks.items() if stamp-t['last']<=c.track_timeout}
        groups=sorted(self._clusters(points,indices,ranges,beam_count,increment),key=len,reverse=True)[:c.max_tracks]
        centers=np.array([points[g].mean(axis=0) for g in groups]).reshape(-1,2)
        sizes=np.array([np.linalg.norm(np.ptp(points[g],axis=0)) for g in groups])
        assignments={}
        ids=list(self.tracks)
        if ids and len(centers):
            old=np.array([self.tracks[k]['center'] for k in ids])
            distance=np.linalg.norm(old[:,None,:]-centers[None,:,:],axis=2)
            gate=np.array([c.track_max_speed*(stamp-self.tracks[k]['last'])+.12 for k in ids])
            size_error=np.abs(np.array([self.tracks[k]['size'] for k in ids])[:,None]-sizes)
            cost=np.where((distance<=gate[:,None])&(size_error<.5),distance+size_error*.2,1e6)
            rows,cols=linear_sum_assignment(cost)
            for row,col in zip(rows,cols):
                if cost[row,col]<1e6:assignments[int(col)]=ids[row]
        point_tracks=np.full(len(points),-1,dtype=np.int64)
        moving=np.zeros(len(points),dtype=bool)
        for i,group in enumerate(groups):
            key=assignments.get(i)
            if key is None:
                if len(self.tracks)>=c.max_tracks:continue
                key=self.next_id;self.next_id+=1
                self.tracks[key]=dict(samples=deque(maxlen=c.max_frames),moving_until=-1.)
            track=self.tracks[key]
            samples=track['samples']
            while samples and stamp-samples[0][0]>.7:samples.popleft()
            samples.append((stamp,centers[i].copy()))
            track.update(center=centers[i],last=stamp,size=sizes[i])
            if len(samples)>=3:
                span=stamp-samples[0][0]
                displacement=float(np.linalg.norm(centers[i]-samples[0][1]))
                # Changing visible fragments/centroids alone are not motion:
                # most of the actual surface must also leave its old position.
                if span>=c.motion_min_span and displacement>=c.motion_min_displacement and displacement/span>=c.motion_min_speed and stationary[group].mean()<.8:
                    track['moving_until']=stamp+c.motion_hold_sec
            point_tracks[group]=key
            if stamp<=track['moving_until']:moving[group]=True
        moving_ids={k for k,t in self.tracks.items() if stamp<=t['moving_until']}
        return moving,point_tracks,moving_ids

    def process(self, ranges, angle_min, angle_increment, range_min, range_max, stamp, pose):
        """pose=(odom x,y,yaw) of scan frame at *this scan's* timestamp.

        Returns keep mask (same beam positions) and diagnostics. Caller must
        encode rejected beams as NaN, never max-range or reconstructed history.
        """
        c=self.cfg
        ranges=np.asarray(ranges,dtype=np.float64)
        pose=np.asarray(pose,dtype=np.float64)
        if ranges.ndim!=1 or not 0<len(ranges)<=c.max_scan_points:raise ValueError('Invalid scan size')
        if pose.shape!=(3,) or not np.isfinite(pose).all():raise ValueError('Invalid pose')
        if not all(math.isfinite(v) for v in (stamp,angle_min,angle_increment,range_min,range_max)):
            raise ValueError('Invalid scan metadata')
        if stamp<=0 or angle_increment<=0 or not 0<range_min<range_max:raise ValueError('Invalid scan metadata')
        reset_reason=''
        if self.last_stamp is not None:
            dt=stamp-self.last_stamp
            if dt<c.min_frame_interval-1e-8:raise ValueError('Nonmonotonic or too-close scan timestamp')
            yaw_step=math.atan2(math.sin(pose[2]-self.last_pose[2]),math.cos(pose[2]-self.last_pose[2]))
            if dt>c.max_gap_sec:reset_reason='scan_gap'
            elif np.linalg.norm(pose[:2]-self.last_pose[:2])>2.*dt+.15 or abs(yaw_step)>3.*dt+.15:
                reset_reason='odom_jump'
            if reset_reason:self.reset()
        self.last_stamp=stamp;self.last_pose=pose.copy()
        while self.history and stamp-self.history[0]['stamp']>c.window_sec:self.history.popleft()
        valid=np.isfinite(ranges)&(ranges>=range_min)&(ranges<range_max)
        indices=np.flatnonzero(valid)
        angles=angle_min+indices*angle_increment+pose[2]
        points=np.column_stack((ranges[indices]*np.cos(angles),ranges[indices]*np.sin(angles)))+pose[:2]
        moving,point_tracks,moving_ids=self._motion(points,indices,ranges[indices],len(ranges),angle_increment,stamp)
        support=np.ones(len(points),dtype=np.int16)
        oldest=np.full(len(points),stamp)
        radius=np.minimum(c.max_match_radius,c.match_radius+c.range_radius_scale*ranges[indices])
        invalidated=0
        full_circle=abs(len(ranges)*angle_increment-2*math.pi)<2*angle_increment
        moving_keys=list(moving_ids)
        # Historical points only vote. They are never replayed into the scan.
        for frame in self.history:
            active=frame['active']
            before=int(np.count_nonzero(active))
            if moving_ids:active &= ~np.isin(frame['tracks'],moving_keys)
            delta=frame['points']-pose[:2]
            distance=np.linalg.norm(delta,axis=1)
            bearing=np.arctan2(delta[:,1],delta[:,0])-pose[2]
            bearing=(bearing-angle_min)%(2*math.pi)
            beam=np.rint(bearing/angle_increment).astype(int)
            near=np.column_stack((beam-1,beam,beam+1))
            inside=np.ones(len(beam),dtype=bool) if full_circle else (near.min(axis=1)>=0)&(near.max(axis=1)<len(ranges))
            near=near%len(ranges) if full_circle else np.clip(near,0,len(ranges)-1)
            # Three actual farther returns required: no clearing from inf/NaN,
            # out-of-FOV, an occluder, or a filtered endpoint.
            clear=inside & np.all(valid[near] & (ranges[near]>distance[:,None]+c.clear_margin),axis=1)
            invalidated+=int(np.count_nonzero(active&clear));active &= ~clear
            # Active bits only transition True -> False. A count change means
            # cached geometry must be rebuilt before it can vote again.
            if np.count_nonzero(active)!=before:frame['active_tree']=None
            if not active.any() or not len(points):continue
            distances,_=self._tree(frame,active_only=True).query(points,k=1,workers=1)
            matched=distances<=radius
            support+=matched
            oldest[matched]=np.minimum(oldest[matched],frame['stamp'])
        accepted=(support>=c.min_hits)&(stamp-oldest>=c.min_span_sec-1e-8)&~moving
        keep=np.zeros(len(ranges),dtype=bool);keep[indices]=accepted
        self.history.append(dict(stamp=stamp,points=points,tracks=point_tracks,active=~moving))
        return keep,dict(input_valid=int(valid.sum()),confirmed=int(accepted.sum()),
            moving_candidates=int(moving.sum()),pending=int((~accepted&~moving).sum()),
            history_frames=len(self.history),tracks=len(self.tracks),invalidated_history_points=invalidated,
            reset_reason=reset_reason,method='temporal_support_and_compact_cluster_motion')
