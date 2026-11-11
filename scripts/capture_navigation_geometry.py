"""Bounded read-only robot-description and timestamped TF geometry snapshot."""
import hashlib,json,time
from pathlib import Path
import numpy as np
from scipy.spatial import ConvexHull
import rclpy
from rclpy.time import Time
from rcl_interfaces.srv import GetParameters
from tf2_ros import Buffer,TransformListener,TransformException
from r1pro_waypoint_nav.box_geometry import build_boxes,corners,tf_matrix
from r1pro_waypoint_nav.self_filter import SelfFilter


def main():
    root=Path(__file__).resolve().parents[1]
    rclpy.init();n=rclpy.create_node('r1pro_navigation_geometry_readonly')
    buffer=Buffer();listener=TransformListener(buffer,n)
    try:
        client=n.create_client(GetParameters,'/robot_state_publisher/get_parameters')
        if not client.wait_for_service(timeout_sec=3):raise RuntimeError('Robot description unavailable')
        req=GetParameters.Request();req.names=['robot_description'];f=client.call_async(req)
        rclpy.spin_until_future_complete(n,f,timeout_sec=3)
        if not f.done():raise RuntimeError('Robot description timeout')
        xml=f.result().values[0].string_value;models=build_boxes(xml,SelfFilter.resolve)
        end=time.monotonic()+4
        while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.02)
        ns=n.get_clock().now().nanoseconds-200_000_000;stamp=Time(nanoseconds=ns)
        all_points=[];boxes=[];cache={'base_link':np.eye(4)}
        for model in models:
            shape=model;fallback=False
            try:
                if shape['frame'] not in cache:cache[shape['frame']]=tf_matrix(buffer.lookup_transform('base_link',shape['frame'],stamp).transform)
            except TransformException:
                if 'fallback' not in model:raise
                shape=model['fallback'];fallback=True
                if shape['frame'] not in cache:cache[shape['frame']]=tf_matrix(buffer.lookup_transform('base_link',shape['frame'],stamp).transform)
            matrix=cache[shape['frame']]@shape['matrix']
            points=corners(shape['low'],shape['high'])@matrix[:3,:3].T+matrix[:3,3]
            all_points.extend(points)
            boxes.append(dict(name=model['name'],frame=model['frame'],fallback=fallback,low=points.min(0).tolist(),high=points.max(0).tolist()))
        points=np.asarray(all_points);hull=points[ConvexHull(points[:,:2]).vertices,:2]
        report=dict(frame='base_link',stamp_ns=ns,description_sha256=hashlib.sha256(xml.encode()).hexdigest(),
                    box_count=len(boxes),low=points.min(0).tolist(),high=points.max(0).tolist(),
                    size=(points.max(0)-points.min(0)).tolist(),footprint=hull.tolist(),
                    circumscribed_radius=float(np.linalg.norm(hull,axis=1).max()),boxes=boxes)
        path=root/'reports/navigation_geometry.json';path.write_text(json.dumps(report,indent=2))
        print(json.dumps({k:v for k,v in report.items() if k!='boxes'},indent=2))
    finally:n.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
