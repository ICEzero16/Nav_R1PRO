"""Bounded offline regression/benchmark. Never initialize ROS or drivers."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import json,time
from pathlib import Path
import numpy as np
from r1pro_waypoint_nav.box_geometry import corners,box_mask,invalid_points
from r1pro_waypoint_nav.crop_backend import prepare_boxes,cpu_crop,CudaCrop
from r1pro_waypoint_nav.outliers import post_filter_mask

root=Path(__file__).resolve().parents[1]
capture=root/'captures/three_frames_20261108_215505'
snapshots=json.loads((capture/'tf_snapshots.json').read_text())
gpu=CudaCrop()
def original(xyz,posed):
    worlds=[corners(s['low']-.025,s['high']+.025)@t[:3,:3].T+t[:3,3] for _,s,t in posed]
    union=np.concatenate(worlds)
    nearby=np.flatnonzero((~invalid_points(xyz))&np.all((xyz>=union.min(0))&(xyz<=union.max(0)),axis=1))
    p=xyz[nearby];hit=np.zeros(len(p),dtype=bool)
    for _,s,t in posed:hit|=box_mask(p,t,s['low']-.025,s['high']+.025)[0]
    result=np.zeros(len(xyz),dtype=bool);result[nearby]=hit
    return result
results=[]
try:
    # Explicit faces, empty input/boxes, duplicate points, invalid coordinates;
    # resize both persistent buffers, repeat calls, rotated boxes.
    rng=np.random.default_rng(17)
    for n in [0,1,200,12000,20]:
        xyz=rng.normal(size=(n,3)).astype('f4')
        if n>10:xyz[:7]=[[0,0,0],[1,1,1],[-1,-1,-1],[1.00001,0,0],[np.nan,0,0],[np.inf,0,0],[0,0,0]]
        for count in [0,1,50]:
            posed=[]
            for j in range(count):
                t=np.eye(4);a=j*.13;t[:2,:2]=[[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]
                posed.append(({},dict(low=np.full(3,-1.),high=np.full(3,1.)),t))
            boxes,_=prepare_boxes(posed,0)
            assert np.array_equal(cpu_crop(xyz,boxes),gpu(xyz,boxes))
    for i,snapshot in enumerate(snapshots,1):
        data=np.load(capture/f'frame_{i}_points.npz');xyz=data['xyz'];invalid=data['invalid']
        posed=[({},dict(low=np.array(b['low']),high=np.array(b['high'])),np.array(b['matrix'])) for b in snapshot['boxes']]
        times={k:[] for k in ['original_cpu','optimized_cpu','cuda']};pipeline={k:[] for k in times}
        expected=np.load(capture/'outliers_range_k16_s3'/f'frame_{i}_masks.npz')['denoised']
        for repeat in range(7):
            for name in (list(times) if repeat%2==0 else list(reversed(times))):
                start=time.perf_counter()
                if name=='original_cpu':inside=original(xyz,posed)
                else:
                    boxes,_=prepare_boxes(posed,.025)
                    inside=(gpu if name=='cuda' else cpu_crop)(xyz,boxes)
                crop_ms=(time.perf_counter()-start)*1000
                kept=~inside&~invalid
                assert np.array_equal(kept,data['kept']),name
                final,rejected,_=post_filter_mask(xyz,kept,True,16,3.,method='range')
                total_ms=(time.perf_counter()-start)*1000
                assert np.array_equal(final,expected),name
                if repeat:
                    times[name].append(crop_ms);pipeline[name].append(total_ms)
        results.append(dict(frame=i,identical_masks=True,crop_median_ms={k:round(float(np.median(v)),3) for k,v in times.items()},
            filter_median_ms={k:round(float(np.median(v)),3) for k,v in pipeline.items()}))
finally:gpu.close()
report=dict(frames=results,cpu_workers=1,services_started=False,cuda_includes='box preparation, input conversion, host/device transfers, synchronization and returned mask',
    excludes='TF lookups, ROS transport, output cloud packing and visualization',synthetic_cases=15)
(root/'reports/cuda_crop_validation.json').write_text(json.dumps(report,indent=2))
print(json.dumps(report,indent=2))
