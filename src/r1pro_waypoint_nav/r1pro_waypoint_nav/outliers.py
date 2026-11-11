"""Single-pass statistical outlier removal on already self-filtered XYZ."""
import math
import time
import numpy as np
from .knn_backend import neighbor_means


def statistical_inliers(points,mean_k=16,std_multiplier=2.0):
    if isinstance(mean_k,bool) or not isinstance(mean_k,(int,np.integer)) or not 2<=mean_k<=128:
        raise ValueError('mean_k must be an integer within 2..128')
    if isinstance(std_multiplier,bool) or not math.isfinite(std_multiplier) or std_multiplier<=0:
        raise ValueError('std_multiplier must be finite and positive')
    xyz=np.asarray(points,dtype=np.float64)
    if xyz.ndim!=2 or xyz.shape[1]!=3 or not np.isfinite(xyz).all():
        raise ValueError('SOR input must contain only finite N x 3 XYZ')
    start=time.perf_counter();n=len(xyz)
    info=dict(method='statistical_single_pass',mean_k=int(mean_k),std_multiplier=float(std_multiplier),input_points=n)
    if n<=mean_k:
        info.update(reason='insufficient_neighbors_preserved',removed_points=0,kept_points=n,processing_ms=0.)
        return np.ones(n,dtype=bool),info
    averages,info['neighbor_backend']=neighbor_means(xyz,mean_k)
    mean=float(averages.mean());std=float(averages.std(ddof=1));threshold=mean+std_multiplier*std
    keep=averages<=threshold+max(1e-12,abs(threshold)*1e-12)
    info.update(reason='ok',neighbor_mean_m=mean,neighbor_std_m=std,threshold_m=threshold,
        removed_points=int((~keep).sum()),kept_points=int(keep.sum()),processing_ms=round((time.perf_counter()-start)*1000,3))
    return keep,info


def range_statistical_inliers(points,mean_k=16,std_multiplier=3.0,
        range_edges=(2.,5.,10.,20.),min_bin_points=64,global_floor_std=2.0):
    """Sensor-frame XYZ only: global neighbors, range-local thresholds, one pass.

    Empty/small bins are preserved. Thresholds never tighten with range and
    never fall below the legacy global SOR floor. No point records are changed.
    """
    xyz=np.asarray(points,dtype=np.float64)
    # Reuse validation without constructing a second tree.
    statistical_inliers(np.empty((0,3)),mean_k,std_multiplier)
    statistical_inliers(np.empty((0,3)),mean_k,global_floor_std)
    edges=np.asarray(range_edges,dtype=float)
    if edges.ndim!=1 or not len(edges) or not np.isfinite(edges).all() or edges[0]<=0 or not (np.diff(edges)>0).all():
        raise ValueError('range_edges must be finite, positive and strictly increasing')
    if isinstance(min_bin_points,bool) or not isinstance(min_bin_points,(int,np.integer)) or min_bin_points<mean_k+1:
        raise ValueError('min_bin_points must be an integer >= mean_k + 1')
    if xyz.ndim!=2 or xyz.shape[1]!=3 or not np.isfinite(xyz).all():
        raise ValueError('Range SOR input must contain finite N x 3 sensor-frame XYZ')
    start=time.perf_counter();n=len(xyz);keep=np.ones(n,dtype=bool)
    info=dict(method='range_statistical_single_pass',mean_k=int(mean_k),std_multiplier=float(std_multiplier),
        range_origin='input_cloud_sensor_origin',range_edges_m=edges.tolist(),min_bin_points=int(min_bin_points),
        global_floor_std=float(global_floor_std),input_points=n,bins=[])
    if n<=mean_k:
        info.update(reason='insufficient_neighbors_preserved',removed_points=0,kept_points=n,processing_ms=0.)
        return keep,info
    averages,info['neighbor_backend']=neighbor_means(xyz,mean_k)
    floor=float(averages.mean()+global_floor_std*averages.std(ddof=1))
    groups=np.searchsorted(edges,np.linalg.norm(xyz,axis=1),side='right')
    threshold=floor
    for index in range(len(edges)+1):
        selection=groups==index;count=int(selection.sum())
        entry=dict(near_m=0. if index==0 else float(edges[index-1]),
            far_m=float(edges[index]) if index<len(edges) else None,input_points=count)
        if count<min_bin_points:
            entry.update(reason='insufficient_bin_points_preserved',threshold_m=None,removed_points=0)
        else:
            values=averages[selection];mean=float(values.mean());std=float(values.std(ddof=1))
            threshold=max(threshold,mean+std_multiplier*std)
            local=values<=threshold+max(1e-12,abs(threshold)*1e-12);keep[selection]=local
            entry.update(reason='ok',neighbor_mean_m=mean,neighbor_std_m=std,
                threshold_m=threshold,removed_points=int((~local).sum()))
        info['bins'].append(entry)
    info.update(reason='ok',global_floor_m=floor,removed_points=int((~keep).sum()),
        kept_points=int(keep.sum()),processing_ms=round((time.perf_counter()-start)*1000,3))
    return keep,info


def post_filter_mask(xyz,self_kept,enabled=True,mean_k=16,std_multiplier=2.0,
        method='global',range_edges=(2.,5.,10.,20.),min_bin_points=64,global_floor_std=2.0):
    """Return masks in ORIGINAL record order, so all per-point fields survive."""
    xyz=np.asarray(xyz);self_kept=np.asarray(self_kept)
    if self_kept.dtype!=bool or self_kept.shape!=(len(xyz),):raise ValueError('Expected one boolean mask per original point')
    if enabled:
        if method=='global':
            local,info=statistical_inliers(xyz[self_kept],mean_k,std_multiplier)
        elif method=='range':
            local,info=range_statistical_inliers(xyz[self_kept],mean_k,std_multiplier,range_edges,min_bin_points,global_floor_std)
        else:raise ValueError('method must be global or range')
    else:
        local=np.ones(int(self_kept.sum()),dtype=bool)
        info=dict(method='disabled',reason='disabled',input_points=int(self_kept.sum()),removed_points=0,kept_points=int(self_kept.sum()),processing_ms=0.)
    final=np.zeros(len(xyz),dtype=bool);final[self_kept]=local
    rejected=self_kept&~final
    return final,rejected,info
