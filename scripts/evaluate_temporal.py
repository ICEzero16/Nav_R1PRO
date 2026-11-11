"""Deterministic synthetic replay, no ROS initialization or sensor acquisition."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
os.environ['OMP_NUM_THREADS']='1'
import json
import math
import time
from pathlib import Path
import numpy as np
from r1pro_waypoint_nav.temporal_core import TemporalFilter


def scan(pose, center, rng, noise_count):
    n=720
    angles=-math.pi+np.arange(n)*2*math.pi/n+pose[2]
    direction=np.column_stack((np.cos(angles),np.sin(angles)))
    origin=np.array(pose[:2])
    with np.errstate(divide='ignore',invalid='ignore'):
        distance=(np.where(direction>0,5.,-5.)-origin)/direction
    distance=np.where(np.abs(direction)<1e-12,np.inf,distance)
    ranges=distance.min(axis=1);labels=np.zeros(n,dtype=int)
    # A small stationary post must survive the filter, as well as the walls.
    for xy,radius,label in [((-1.5,1.),.15,3),(center,.25,1)]:
        if xy is None:continue
        delta=np.asarray(xy)-origin;along=direction@delta
        discriminant=radius**2-(delta@delta-along**2)
        hit=along-np.sqrt(np.maximum(discriminant,0.))
        mask=(discriminant>=0)&(hit>.3)&(hit<ranges)
        ranges[mask]=hit[mask];labels[mask]=label
    indices=rng.choice(n,noise_count,replace=False)
    ranges[indices]=rng.uniform(.4,1.2,size=noise_count);labels[indices]=2
    return ranges,labels,direction


def main():
    root=Path(__file__).resolve().parents[1]
    dest=root/'reports'/'temporal_offline';dest.mkdir(parents=True,exist_ok=True)
    results={};examples={}
    for name,speed,ego,noise in [('static_with_noise',0.,False,25),('moving_with_noise',.5,True,25),('slow_motion_limit',.03,False,0)]:
        f=TemporalFilter();rng=np.random.default_rng(20261110)
        counts={key:[0,0] for key in ('wall','object','noise','static_post')}
        elapsed=[];last={}
        for i in range(60):
            pose=(i*.01,0.,i*.005) if ego else (0.,0.,0.)
            center=(2.,-1.5+speed*i*.1)
            ranges,labels,directions=scan(pose,center,rng,noise)
            start=time.perf_counter()
            keep,last=f.process(ranges,-math.pi,2*math.pi/720,.3,12.,1+i*.1,pose)
            elapsed.append((time.perf_counter()-start)*1000)
            if i>=10:
                for label,key in enumerate(('wall','object','noise','static_post')):
                    counts[key][0]+=int(sum(labels==label));counts[key][1]+=int(sum(keep&(labels==label)))
            if i==35:
                points=directions*ranges[:,None]+np.array(pose[:2])
                examples[name]=dict(points=points.tolist(),labels=labels.tolist(),keep=keep.tolist(),pose=pose)
        results[name]=dict(observations_after_warmup={key:dict(input=v[0],kept=v[1],kept_fraction=None if not v[0] else v[1]/v[0]) for key,v in counts.items()},
            processing_ms=dict(median=float(np.median(elapsed)),p95=float(np.percentile(elapsed,95)),max=float(max(elapsed))),
            last_status=last,object_speed_mps=speed,robot_moving=ego)
    report=dict(data_source='synthetic analytic 2D rays, not robot recordings',frames_per_case=60,beams=720,seed=20261110,
                worker_count=1,results=results,scope='core algorithm timing only; excludes existing CUDA/SOR/TF/ROS overhead')
    (dest/'report.json').write_text(json.dumps(report,indent=2))
    (dest/'example_frames.json').write_text(json.dumps(examples))
    print(json.dumps(report,indent=2))
    assert results['moving_with_noise']['observations_after_warmup']['object']['kept_fraction']<.1
    assert results['static_with_noise']['observations_after_warmup']['noise']['kept_fraction']<.05
    assert results['static_with_noise']['observations_after_warmup']['object']['kept_fraction']>.98
    assert results['static_with_noise']['observations_after_warmup']['wall']['kept_fraction']>.98
    assert results['moving_with_noise']['observations_after_warmup']['wall']['kept_fraction']>.95
    assert results['moving_with_noise']['observations_after_warmup']['static_post']['kept_fraction']>.95


if __name__=='__main__':main()
