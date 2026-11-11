"""Occupancy map validation, PNG encoding and last-saved-map fallback (no ROS)."""
from collections import OrderedDict
import math
from pathlib import Path
import struct
import time
import zlib
import numpy as np
import yaml


def png_bytes(grid):
    values=np.asarray(grid,dtype=np.int16)
    gray=np.full(values.shape,157,dtype=np.uint8)
    known=values>=0
    gray[known]=np.round(246*(1-values[known]/100.)).astype(np.uint8)
    rows=np.flipud(gray)
    raw=b''.join(b'\x00'+row.tobytes() for row in rows)
    def chunk(kind,data):return struct.pack('!I',len(data))+kind+data+struct.pack('!I',zlib.crc32(kind+data)&0xffffffff)
    return b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR',struct.pack('!2I5B',rows.shape[1],rows.shape[0],8,0,0,0,0))+chunk(b'IDAT',zlib.compress(raw,3))+chunk(b'IEND',b'')


class MapStore:
    """Caller holds its lock. Image revisions are immutable to prevent tearing."""
    def __init__(self):self.meta=None;self.images=OrderedDict();self.revision=0;self.received=0.

    def put(self,width,height,resolution,origin,data,source,label,stamp=None):
        if width<=0 or height<=0 or width*height>4_000_000:raise ValueError('Map size out of bounds')
        if not math.isfinite(resolution) or resolution<=0 or len(origin)!=3 or not all(math.isfinite(v) for v in origin):raise ValueError('Invalid map geometry')
        grid=np.asarray(data,dtype=np.int16)
        if grid.size!=width*height or np.any((grid<-1)|(grid>100)):raise ValueError('Invalid occupancy data')
        image=png_bytes(grid.reshape(height,width));self.revision+=1;self.received=time.monotonic()
        self.images[self.revision]=image
        while len(self.images)>3:self.images.popitem(last=False)
        self.meta=dict(revision=self.revision,width=width,height=height,resolution=resolution,origin=list(origin),
                       frame='map',source=source,label=label,stamp=stamp,known_cells=int(sum(grid.ravel()>=0)),
                       image_url=f'/api/maps/{self.revision}.png')

    def snapshot(self):
        if self.meta is None:return None
        return dict(self.meta,age=time.monotonic()-self.received)

    def load_saved(self,root):
        candidates=sorted(Path(root).glob('*_trial_*/map.yaml'),key=lambda p:p.stat().st_mtime,reverse=True)
        for path in candidates:
            try:
                spec=yaml.safe_load(path.read_text())
                image=(path.parent/spec['image']).resolve()
                if image.parent!=path.parent.resolve():continue
                # Our saved maps use binary PGM with a one-byte max value.
                with image.open('rb') as stream:
                    if stream.readline().strip()!=b'P5':continue
                    line=stream.readline()
                    while line.startswith(b'#'):line=stream.readline()
                    width,height=map(int,line.split())
                    if stream.readline().strip()!=b'255' or width*height>4_000_000:continue
                    pixels=np.frombuffer(stream.read(),dtype=np.uint8).reshape(height,width)
                prob=pixels/255. if spec.get('negate',0) else 1-pixels/255.
                grid=np.full((height,width),-1,dtype=np.int16)
                grid[prob>spec.get('occupied_thresh',.65)]=100;grid[prob<spec.get('free_thresh',.25)]=0
                # Earlier workspace exports used gray=205 with free_thresh=.25,
                # which would otherwise misclassify their unknown cells as free.
                grid[pixels==205]=-1
                self.put(width,height,float(spec['resolution']),spec['origin'],np.flipud(grid),'saved',path.parent.name,path.stat().st_mtime)
                return True
            except (ValueError,KeyError,OSError,TypeError):continue
        return False
