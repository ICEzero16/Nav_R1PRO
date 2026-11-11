"""Offline A/B benchmark on saved clouds and deterministic scan sequences."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import importlib.util
import json
from pathlib import Path
import sys
import time
import numpy as np
from r1pro_waypoint_nav.outliers import post_filter_mask
from r1pro_waypoint_nav.temporal_core import TemporalFilter

root=Path(__file__).resolve().parents[1]
def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path);mod=importlib.util.module_from_spec(spec)
    sys.modules[name]=mod;spec.loader.exec_module(mod);return mod

old=load('baseline_outliers',root/'reports/filter_baseline/outliers.py')
old_temporal=load('baseline_temporal',root/'reports/filter_baseline/temporal_core.py')
scene=load('benchmark_scene',root/'tests/test_temporal.py')
result=dict(offline=True,services_started=False,compute_workers=1,clouds=[],temporal=[])
def stats(values):return dict(median_ms=float(np.median(values)),p95_ms=float(np.percentile(values,95)))
for frame in range(1,4):
    p=np.load(root/f'captures/three_frames_20261108_215505/frame_{frame}_points.npz')
    xyz=p['xyz'];initial=p['kept'];elapsed={'before':[],'after':[]}
    for i in range(14):
        outputs={}
        # Alternate order so warmed CPU/cache does not always favor one side.
        for name,fn in ([('before',old.post_filter_mask),('after',post_filter_mask)] if i%2==0 else [('after',post_filter_mask),('before',old.post_filter_mask)]):
            start=time.perf_counter();outputs[name]=fn(xyz,initial,True,16,3.,method='range')
            if i>=2:elapsed[name].append((time.perf_counter()-start)*1000)
        for mask in (0,1):assert np.array_equal(outputs['before'][mask],outputs['after'][mask])
        for a,b in zip(outputs['before'][2]['bins'],outputs['after'][2]['bins']):
            if a['threshold_m'] is not None:assert np.isclose(a['threshold_m'],b['threshold_m'],rtol=1e-12,atol=1e-12)
    row=dict(frame=frame,points=len(xyz),kept=int(outputs['after'][0].sum()),masks_identical=True,
             before=stats(elapsed['before']),after=stats(elapsed['after']))
    result['clouds'].append(row);print('CLOUD',json.dumps(row),flush=True)

for kind in ('static','rotating','moving_object'):
    inputs=[];rng=np.random.default_rng(451)
    for i in range(80):
        pose=(0.,0.,i*.012) if kind=='rotating' else (0.,0.,0.)
        circle=(2.,-1.2+.04*i,.25) if kind=='moving_object' else (2.,0.,.2)
        ranges,_=scene.scene(pose,circle=circle)
        if i%5==0:ranges[rng.choice(len(ranges),10,replace=False)]=np.nan
        inputs.append((ranges,1+i*.1,pose))
    elapsed={'before':[],'after':[]}
    for repeat in range(3):
        filters={'before':old_temporal.TemporalFilter(),'after':TemporalFilter()}
        for i,(ranges,stamp,pose) in enumerate(inputs):
            outputs={}
            for name in (('before','after') if repeat%2==0 else ('after','before')):
                start=time.perf_counter();outputs[name]=filters[name].process(ranges,-np.pi,2*np.pi/len(ranges),.3,12.,stamp,pose)
                if i>=15:elapsed[name].append((time.perf_counter()-start)*1000)
            assert np.array_equal(outputs['before'][0],outputs['after'][0]),(kind,i)
            assert outputs['before'][1]==outputs['after'][1],(kind,i)
    row=dict(scene=kind,frames=80,repeats=3,masks_and_stats_identical=True,before=stats(elapsed['before']),after=stats(elapsed['after']))
    result['temporal'].append(row);print('TEMPORAL',json.dumps(row),flush=True)

out=root/'reports'/('filter_engineering_'+time.strftime('%Y%m%d_%H%M%S')+'.json')
out.write_text(json.dumps(result,indent=2));print('RESULT',out,flush=True)
