"""Prepared OBB crop; CPU single thread or persistent CUDA context. No ROS."""
import ctypes as C
from pathlib import Path
import numpy as np
from .box_geometry import corners,invalid_points


def prepare_boxes(posed, padding):
    records=[];worlds=[]
    for model,shape,t in posed:
        margin=shape.get('padding',padding)
        low=shape['low']-margin;high=shape['high']+margin
        world=corners(low,high)@t[:3,:3].T+t[:3,3]
        worlds.append(world)
        records.append(np.concatenate([t[:3,:3].ravel(),t[:3,3],low,high,world.min(0),world.max(0)]))
    return np.asarray(records,dtype=np.float64).reshape(-1,24),worlds


def cpu_crop(xyz,boxes):
    inside=np.zeros(len(xyz),dtype=bool)
    if not len(boxes):return inside
    # Only one original-to-nearby index, no per-box index gather/scatter.
    nearby=np.flatnonzero((~invalid_points(xyz))&np.all((xyz>=boxes[:,18:21].min(0))&(xyz<=boxes[:,21:24].max(0)),axis=1))
    points=xyz[nearby].astype(np.float64);hits=np.zeros(len(points),dtype=bool)
    for b in boxes:
        broad=(~hits)&np.all((points>=b[18:21])&(points<=b[21:24]),axis=1)
        if not broad.any():continue
        local=(points[broad]-b[9:12])@b[:9].reshape(3,3)
        hits[broad]=np.all((local>=b[12:15]-1e-7)&(local<=b[15:18]+1e-7),axis=1)
    inside[nearby]=hits
    return inside


class CudaCrop:
    def __init__(self):
        self.lib=C.CDLL(str(Path(__file__).with_name('libcrop_cuda.so')))
        lib=self.lib
        lib.crop_create.restype=C.c_void_p
        lib.crop_destroy.argtypes=[C.c_void_p]
        lib.crop_run.argtypes=[C.c_void_p,C.c_void_p,C.c_int,C.c_void_p,C.c_int,C.c_void_p]
        lib.crop_run.restype=C.c_int
        lib.crop_error.restype=C.c_char_p
        self.context=lib.crop_create()
        if not self.context:raise RuntimeError(lib.crop_error().decode())
    def __call__(self,xyz,boxes):
        xyz=np.ascontiguousarray(xyz,dtype=np.float32);boxes=np.ascontiguousarray(boxes,dtype=np.float64)
        out=np.empty(len(xyz),dtype=np.uint8)
        status=self.lib.crop_run(self.context,xyz.ctypes.data,len(xyz),boxes.ctypes.data,len(boxes),out.ctypes.data)
        if status:raise RuntimeError(self.lib.crop_error().decode())
        return out.view(np.bool_)
    def close(self):
        if getattr(self,'context',None):self.lib.crop_destroy(self.context);self.context=None
    def __del__(self):self.close()
