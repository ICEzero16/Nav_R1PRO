"""Timestamp-aligned articulated robot box self-filter. No motion publishers."""
import os
os.environ['OPENBLAS_NUM_THREADS']='1'
import copy
from array import array
import hashlib
import json
import signal
import time
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from rcl_interfaces.srv import GetParameters
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from tf2_ros import Buffer,TransformListener,TransformException
from ament_index_python.packages import get_package_share_directory
from .box_geometry import build_boxes,tf_matrix,box_mask,unpack_cloud,corners,invalid_points
from .outliers import post_filter_mask
from .crop_backend import prepare_boxes,cpu_crop,CudaCrop
from .deck_sparse import sparse_deck_indices
from .frame_queue import take_latest_ready


class SelfFilter(Node):
    def __init__(self):
        super().__init__('r1pro_self_filter')
        self.declare_parameter('padding',.025)
        self.padding=float(self.get_parameter('padding').value)
        if not 0<=self.padding<=.10:raise ValueError('padding must be within 0..0.10 m')
        self.declare_parameter('base_top_patch_bounds', Parameter.Type.DOUBLE_ARRAY)
        bounds=self.get_parameter_or('base_top_patch_bounds').value
        self.base_top_patch=None
        if bounds:
            extent=np.asarray(bounds,dtype=float)
            if extent.shape!=(6,) or not np.isfinite(extent).all() or np.any(extent[:3]>=extent[3:]):
                raise ValueError('base_top_patch_bounds requires [xmin,ymin,zmin,xmax,ymax,zmax]')
            self.base_top_patch=dict(name='base_link:top_patch',frame='base_link',source='measured',
                                     low=extent[:3],high=extent[3:],matrix=np.eye(4),padding=0.)
        self.declare_parameter('deck_sparse_bounds',Parameter.Type.DOUBLE_ARRAY)
        self.deck_bounds=np.asarray(self.get_parameter_or('deck_sparse_bounds').value or [],dtype=float)
        self.deck_radius=self.declare_parameter('deck_neighbor_radius',.04).value
        self.deck_min_points=self.declare_parameter('deck_min_points',4).value
        if self.deck_bounds.size and (self.deck_bounds.shape!=(6,) or not np.isfinite(self.deck_bounds).all()
                                      or np.any(self.deck_bounds[:3]>=self.deck_bounds[3:])):
            raise ValueError('Invalid deck_sparse_bounds')
        if not 0 < self.deck_radius <= .2 or not 2 <= self.deck_min_points <= 32:
            raise ValueError('Invalid deck neighborhood')
        self.declare_parameter('outlier_enabled',True)
        self.declare_parameter('outlier_mean_k',16)
        self.declare_parameter('outlier_std_multiplier',3.0)
        self.declare_parameter('outlier_method','range')
        self.declare_parameter('outlier_range_edges',[2.0,5.0,10.0,20.0])
        self.declare_parameter('outlier_min_bin_points',64)
        self.declare_parameter('outlier_global_floor_std',2.0)
        self.outlier_enabled=self.get_parameter('outlier_enabled').value
        self.outlier_k=self.get_parameter('outlier_mean_k').value
        self.outlier_std=self.get_parameter('outlier_std_multiplier').value
        self.outlier_options=dict(method=self.get_parameter('outlier_method').value,
            range_edges=self.get_parameter('outlier_range_edges').value,
            min_bin_points=self.get_parameter('outlier_min_bin_points').value,
            global_floor_std=self.get_parameter('outlier_global_floor_std').value)
        post_filter_mask(np.empty((0,3)),np.zeros(0,dtype=bool),True,
            self.outlier_k,self.outlier_std,**self.outlier_options) # Validate before processing.
        self.declare_parameter('crop_backend','cpu')
        self.crop_backend=self.get_parameter('crop_backend').value
        if self.crop_backend not in ('cpu','cuda'):raise ValueError('crop_backend must be cpu or cuda')
        self.cropper=CudaCrop() if self.crop_backend=='cuda' else cpu_crop
        self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
        self.models=[];self.pending=deque(maxlen=4);self.future=None;self.xml_hash='';self.dropped=0
        self.last_received=0.;self.last_ok=0.;self.stats=dict(ready=False,reason='waiting_for_robot_description')
        self.service=self.create_client(GetParameters,'/robot_state_publisher/get_parameters')
        self.filtered=self.create_publisher(PointCloud2,'/r1pro_nav/points_filtered',qos_profile_sensor_data)
        self.removed=self.create_publisher(PointCloud2,'/r1pro_nav/points_removed',qos_profile_sensor_data)
        self.denoised=self.create_publisher(PointCloud2,'/r1pro_nav/points_denoised',qos_profile_sensor_data)
        self.outliers=self.create_publisher(PointCloud2,'/r1pro_nav/points_outliers',qos_profile_sensor_data)
        self.status_pub=self.create_publisher(String,'/r1pro_nav/self_filter/status',10)
        self.box_pub=self.create_publisher(String,'/r1pro_nav/self_filter/boxes',10)
        self.marker_pub=self.create_publisher(MarkerArray,'/r1pro_nav/self_filter/markers',10)
        self.create_subscription(PointCloud2,'/hdas/lidar_chassis_left',self.receive,qos_profile_sensor_data)
        self.create_timer(1.,self.description)
        self.create_timer(.02,self.tick)
        self.create_timer(.5,self.publish_status)
        self.last_description_request=0.

    def description(self):
        if self.future is not None:
            if not self.future.done():return
            try:
                xml=self.future.result().values[0].string_value;digest=hashlib.sha256(xml.encode()).hexdigest()
                if digest!=self.xml_hash:
                    self.models=build_boxes(xml,self.resolve)
                    if self.base_top_patch is not None:
                        self.models.append(self.base_top_patch)
                    self.xml_hash=digest;self.pending.clear()
                    self.get_logger().info(f'Fitted {len(self.models)} collision-local boxes; padding={self.padding:.3f} m')
            except Exception as e:
                self.models=[];self.stats=dict(ready=False,reason='robot_description_error: '+str(e))
                self.get_logger().error(self.stats['reason'])
            self.future=None
        if (not self.models or time.monotonic()-self.last_description_request>30) and self.service.service_is_ready():
            request=GetParameters.Request();request.names=['robot_description'];self.future=self.service.call_async(request)
            self.last_description_request=time.monotonic()

    @staticmethod
    def resolve(uri):
        if uri.startswith('package://'):
            package,relative=uri[len('package://'):].split('/',1)
            return Path(get_package_share_directory(package))/relative
        if uri.startswith('file://'):return Path(uri[7:])
        if Path(uri).is_absolute():return Path(uri)
        raise ValueError('Unresolvable mesh URI: '+uri)

    def receive(self,msg):
        self.last_received=time.monotonic()
        if len(self.pending)==self.pending.maxlen:self.dropped+=1
        self.pending.append((msg,time.monotonic()))

    def pose(self,target,source,stamp):
        if target==source:return np.eye(4)
        return tf_matrix(self.buffer.lookup_transform(target,source,stamp).transform)

    def prepare_frame(self,msg):
        stamp=Time.from_msg(msg.header.stamp);posed=[];fallbacks=[]
        base_from_cloud=self.pose('base_link',msg.header.frame_id,stamp)
        cache={}
        for model in self.models:
            shape=model
            try:
                if shape['frame'] not in cache:cache[shape['frame']]=self.pose(msg.header.frame_id,shape['frame'],stamp)
            except TransformException:
                if 'fallback' not in model:raise
                shape=model['fallback'];fallbacks.append(model['frame'])
                if shape['frame'] not in cache:cache[shape['frame']]=self.pose(msg.header.frame_id,shape['frame'],stamp)
            posed.append((model,shape,cache[shape['frame']]@shape['matrix']))
        return stamp,base_from_cloud,posed,fallbacks

    def tick(self):
        if not self.models or not self.pending:return
        start=time.monotonic();clock_ns=self.get_clock().now().nanoseconds
        def fresh(msg):
            ns=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
            return ns>0 and -.02<=(clock_ns-ns)/1e9<=.6
        selected,dropped,superseded,error=take_latest_ready(self.pending,self.prepare_frame,fresh,start,(TransformException,ValueError))
        self.dropped+=dropped
        self.queue_superseded=getattr(self,'queue_superseded',0)+superseded
        if selected is None:
            if dropped:self.stats=dict(ready=False,reason='cloud_timestamp_invalid_or_stale' if error=='stale_or_invalid_timestamp' else 'incomplete_timestamped_tf',detail=error[:220])
            return
        msg,received,(stamp,base_from_cloud,posed,fallbacks)=selected
        tf_done=time.monotonic()
        try:
            raw,xyz=unpack_cloud(msg);invalid=invalid_points(xyz)
            unpack_done=time.monotonic()
            prepared,worlds=prepare_boxes(posed,self.padding)
            inside=self.cropper(xyz,prepared)
            crop_done=time.monotonic()
            boxes=[];markers=MarkerArray()
            boxes_needed=self.box_pub.get_subscription_count()>0
            markers_needed=self.marker_pub.get_subscription_count()>0
            for index,(model,shape,t) in enumerate(posed):
                if not (boxes_needed or markers_needed):continue
                world=worlds[index]
                base_corners=world@base_from_cloud[:3,:3].T+base_from_cloud[:3,3]
                boxes.append(dict(name=model['name'],frame=model['frame'],source=model['source'],swept=model['frame'] in fallbacks,
                    corners=np.round(base_corners,5).tolist(),size=np.round(shape['high']-shape['low']+2*shape.get('padding',self.padding),5).tolist()))
                if not markers_needed:continue
                marker=Marker();marker.header=copy.deepcopy(msg.header);marker.header.frame_id='base_link';marker.ns='self_filter_boxes';marker.id=index
                marker.type=Marker.LINE_LIST;marker.action=Marker.ADD;marker.pose.orientation.w=1.;marker.scale.x=.006
                marker.color.r=.95 if model['frame'] in fallbacks else .3;marker.color.g=.75;marker.color.b=.9;marker.color.a=.8
                for a,b in [(i,i^(1<<k)) for i in range(8) for k in range(3) if i<(i^(1<<k))]:
                    for v in (base_corners[a],base_corners[b]):marker.points.append(Point(x=float(v[0]),y=float(v[1]),z=float(v[2])))
                marker.lifetime.sec=1;markers.markers.append(marker)
            keep=~inside & ~invalid
            deck_start=time.monotonic()
            deck_sparse = np.empty(0,dtype=np.intp)
            if self.deck_bounds.size:
                deck_sparse = sparse_deck_indices(xyz,keep,base_from_cloud,self.deck_bounds,
                                                  self.deck_radius,self.deck_min_points)
                keep[deck_sparse] = False
            # xyz is still in the original lidar frame, NOT base_link/map.
            sor_start=time.monotonic()
            final,outliers,outlier_info=post_filter_mask(xyz,keep,self.outlier_enabled,self.outlier_k,self.outlier_std,**self.outlier_options)
            outliers[deck_sparse] = True
            sor_done=time.monotonic()
            for publisher,selection in [(self.filtered,keep),(self.removed,inside & ~invalid),(self.denoised,final),(self.outliers,outliers)]:
                if publisher.get_subscription_count()==0:continue
                out=PointCloud2();out.header=copy.deepcopy(msg.header);out.fields=copy.deepcopy(msg.fields);out.is_bigendian=msg.is_bigendian
                out.height=1;out.width=int(selection.sum());out.point_step=msg.point_step;out.row_step=out.width*out.point_step
                out.is_dense=True;out.data=array('B');out.data.frombytes(memoryview(np.ascontiguousarray(raw[selection])));publisher.publish(out)
            self.last_ok=time.monotonic()
            self.stats=dict(ready=True,reason='ok',input_points=len(xyz),output_points=int(keep.sum()),
                deck_sparse_points=len(deck_sparse),self_points=int((inside&~invalid).sum()),invalid_points=int(invalid.sum()),box_count=len(posed),
                denoised_points=int(final.sum()),outlier_points=int(outliers.sum()),outlier_enabled=self.outlier_enabled,outlier=outlier_info,crop_backend=self.crop_backend,
                padding_m=self.padding,swept_fingers=fallbacks,processing_ms=round((self.last_ok-start)*1000,2),description_sha256=self.xml_hash)
            self.stats['stage_ms']=dict(queue_wait=round((start-received)*1000,3),tf_prepare=round((tf_done-start)*1000,3),
                unpack=round((unpack_done-tf_done)*1000,3),crop=round((crop_done-unpack_done)*1000,3),
                visualization=round((deck_start-crop_done)*1000,3),deck_sparse=round((sor_start-deck_start)*1000,3),
                outlier=round((sor_done-sor_start)*1000,3),
                publish=round((self.last_ok-sor_done)*1000,3))
            self.stats['queue_superseded']=self.queue_superseded
            self.stats['output_age_ms']=round((self.get_clock().now().nanoseconds-stamp.nanoseconds)/1e6,3)
            if boxes_needed:self.box_pub.publish(String(data=json.dumps(dict(frame='base_link',stamp=stamp.nanoseconds/1e9,padding=self.padding,boxes=boxes))))
            if markers_needed:self.marker_pub.publish(markers)
        except Exception as e:
            self.dropped+=1;self.stats=dict(ready=False,reason='filter_error',detail=str(e));self.get_logger().error(str(e))

    def publish_status(self):
        s=dict(self.stats);s['dropped_frames']=self.dropped
        s['last_success_age']=None if not self.last_ok else round(time.monotonic()-self.last_ok,3)
        if not self.last_received or time.monotonic()-self.last_received>1:s.update(ready=False,reason='waiting_for_cloud')
        elif not self.last_ok or time.monotonic()-self.last_ok>1:s['ready']=False
        self.status_pub.publish(String(data=json.dumps(s)))


def main():
    rclpy.init();node=None
    try:node=SelfFilter();rclpy.spin(node)
    except (KeyboardInterrupt,ExternalShutdownException):pass
    finally:
        signal.signal(signal.SIGINT,signal.SIG_IGN)
        if node:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
