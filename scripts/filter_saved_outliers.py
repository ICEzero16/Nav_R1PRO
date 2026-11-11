"""Offline only: reuse the three captured frames, never initialize ROS or start a process."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
from array import array
import copy
import argparse
import json
from pathlib import Path
import numpy as np
from rclpy.serialization import deserialize_message,serialize_message
from sensor_msgs.msg import PointCloud2
from r1pro_waypoint_nav.box_geometry import unpack_cloud
from r1pro_waypoint_nav.outliers import post_filter_mask

root=Path(__file__).resolve().parents[1]
capture=root/'captures/three_frames_20261108_215505'
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--method',choices=['global','range'],default='range')
args=parser.parse_args()
multiplier=3. if args.method=='range' else 2.
out=capture/('outliers_range_k16_s3' if args.method=='range' else 'outliers_k16_s2');out.mkdir(exist_ok=True)
prior=json.loads((capture/'visualization_data.json').read_text());results=[];visual=[]
for i in range(1,4):
    stored=np.load(capture/f'frame_{i}_points.npz');xyz=stored['xyz'];base_xyz=stored['base_xyz'];initial=stored['kept']
    final,rejected,info=post_filter_mask(xyz,initial,True,16,multiplier,method=args.method)
    old_final,old_rejected,old_info=post_filter_mask(xyz,initial,True,16,2.)
    if args.method=='range':assert not (old_final&~final).any()
    info.update(legacy_removed=int(old_rejected.sum()),restored_points=int((old_rejected&final).sum()))
    ranges=np.linalg.norm(xyz,axis=1)
    for entry in info.get('bins',[]):
        in_bin=(ranges>=entry['near_m']) & (ranges<(entry['far_m'] if entry['far_m'] is not None else np.inf))
        entry['legacy_removed']=int((old_rejected&in_bin).sum())
    assert np.array_equal(final|rejected,initial) and not (final&rejected).any()
    msg=deserialize_message((capture/f'frame_{i}_raw.cdr').read_bytes(),PointCloud2);raw,_=unpack_cloud(msg)
    for name,selection in [('denoised',final),('outliers',rejected)]:
        cloud=copy.deepcopy(msg);cloud.height=1;cloud.width=int(selection.sum());cloud.row_step=cloud.width*cloud.point_step;cloud.is_dense=True
        cloud.data=array('B',raw[selection].tobytes());(out/f'frame_{i}_{name}.cdr').write_bytes(serialize_message(cloud))
        decoded=deserialize_message((out/f'frame_{i}_{name}.cdr').read_bytes(),PointCloud2)
        assert decoded.header==msg.header and decoded.fields==msg.fields and bytes(decoded.data)==raw[selection].tobytes()
    np.savez_compressed(out/f'frame_{i}_masks.npz',denoised=final,outliers=rejected)
    info.update(frame=i,original_input=len(xyz),body_points=int(stored['removed'].sum()),invalid_points=int(stored['invalid'].sum()),records_preserved=True)
    results.append(info)
    env=base_xyz[final];env=env[::max(1,int(np.ceil(len(env)/3500)))]
    p=prior['frames'][i-1]
    visual.append(dict(frame=i,stamp_sec=p['stamp_sec'],stamp_nanosec=p['stamp_nanosec'],input=int(initial.sum()),kept=int(final.sum()),removed=int(rejected.sum()),invalid=0,boxes=46,padding_m=.025,
        model=p['model'],environment=np.round(env,3).tolist(),self_points=np.round(base_xyz[rejected],3).tolist(),sor=info))
(out/'summary.json').write_text(json.dumps(dict(frames=results,mean_k=16,std_multiplier=multiplier,method=args.method,offline_only=True,services_started=False),indent=2))
(out/'visualization_data.json').write_text(json.dumps(dict(frames=visual,coordinate_frame='base_link',capture_mode='offline_outlier_comparison'),separators=(',',':')))
print(json.dumps(results,indent=2));print('OUTPUT',out)
