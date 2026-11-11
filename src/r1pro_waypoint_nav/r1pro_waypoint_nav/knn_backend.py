"""Exact nearest-neighbor means, one worker, with portable SciPy fallback."""
import ctypes as C
from functools import lru_cache
import os
from pathlib import Path
import numpy as np
from scipy.spatial import cKDTree


@lru_cache(maxsize=1)
def native_library():
    try:
        lib=C.CDLL(str(Path(__file__).resolve().with_name('libknn_mean.so')))
    except OSError:
        return None
    lib.knn_mean.argtypes=[C.c_void_p,C.c_size_t,C.c_int,C.c_void_p]
    lib.knn_mean.restype=C.c_int
    return lib


def neighbor_means(xyz,k):
    # Caller has validated finite N x 3 coordinates and k < N.
    backend=os.environ.get('R1PRO_KNN_BACKEND','auto')
    if backend not in ('auto','native','scipy'):raise ValueError('Invalid kNN backend')
    lib=None if backend=='scipy' else native_library()
    if lib is not None:
        points=np.ascontiguousarray(xyz,dtype=np.float64)
        out=np.empty(len(points),dtype=np.float64)
        code=lib.knn_mean(points.ctypes.data,len(points),k,out.ctypes.data)
        if code:raise RuntimeError('Native kNN failed: '+str(code))
        return out,'native_exact_single_thread'
    if backend=='native':raise RuntimeError('Build libknn_mean.so first')
    distances,_=cKDTree(xyz).query(xyz,k=k+1,workers=1)
    return distances[:,1:].mean(axis=1),'scipy_exact_single_thread'
