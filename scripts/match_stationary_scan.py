"""Compare two stationary scan batches against the saved R1PRO occupancy grid."""

import argparse
import json
import math
import time
import warnings
from pathlib import Path

import numpy as np
import yaml
from PIL import Image
from scipy.ndimage import distance_transform_edt, map_coordinates
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation


def capture():
    import rclpy
    from geometry_msgs.msg import TwistWithCovarianceStamped
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from sensor_msgs.msg import LaserScan
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()
    node = rclpy.create_node("stationary_scan_match_capture")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    samples = []
    speeds = []

    def record_speed(msg):
        v = msg.twist.twist
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        speeds.append((stamp, math.hypot(v.linear.x, v.linear.y), abs(v.angular.z)))

    def record_scan(msg):
        try:
            tf = buffer.lookup_transform("base_link", msg.header.frame_id, Time.from_msg(msg.header.stamp)).transform
        except TransformException:
            return
        t, q = tf.translation, tf.rotation
        samples.append(dict(frame=msg.header.frame_id, a=msg.angle_min, inc=msg.angle_increment,
                            stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                            r=[v if msg.range_min <= v < msg.range_max else None for v in msg.ranges],
                            translation=[t.x, t.y, t.z], q=[q.x, q.y, q.z, q.w]))

    try:
        node.create_subscription(TwistWithCovarianceStamped, "/r1pro_nav/chassis_twist", record_speed, qos_profile_sensor_data)
        node.create_subscription(LaserScan, "/r1pro_nav/scan_raw", record_scan, qos_profile_sensor_data)
        end = time.monotonic() + 12
        while time.monotonic() < end and len(samples) < 20:
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if len(samples) < 20:
        raise RuntimeError("Need 20 timestamped scans with base_link transforms")
    measured = np.array([s for s in speeds if samples[0]["stamp"] <= s[0] <= samples[-1]["stamp"]])
    if len(measured) < 2:
        raise RuntimeError("Missing chassis speed during scan capture")
    stamps = np.r_[samples[0]["stamp"], measured[:, 0], samples[-1]["stamp"]]
    if np.max(np.diff(stamps)) > 0.2 or not np.all(np.isfinite(measured)):
        raise RuntimeError("Incomplete chassis speed coverage during scan capture")
    displacement, rotation = np.trapz(measured[:, 1:], measured[:, 0], axis=0)
    if displacement > 0.04 or rotation > np.deg2rad(2):
        raise RuntimeError("Chassis moved during stationary scan capture")
    return samples


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--map", type=Path, required=True, help="Saved trinary occupancy map YAML")
parser.add_argument("--scans", type=Path, help="Replay saved scans instead of sampling the robot")
parser.add_argument("--output", type=Path, required=True, help="Directory for captured scans and fit.json")
parser.add_argument("--plot", action="store_true", help="Write overlay.png (requires matplotlib)")
args = parser.parse_args()
folder = args.output
folder.mkdir(parents=True, exist_ok=True)
map_folder = args.map.parent
metadata = yaml.safe_load(args.map.read_text())
origin = np.array(metadata["origin"][:2])
if metadata["origin"][2] != 0 or metadata["negate"] != 0:
    raise ValueError("Expected an axis-aligned occupancy map with black obstacles")
resolution = metadata["resolution"]
grid = np.flipud(np.asarray(Image.open(map_folder / metadata["image"])))
if not {0, 254}.issubset(np.unique(grid)) or not set(np.unique(grid)).issubset({0, 205, 254}):
    raise ValueError("Expected Nav2 trinary occupancy values (0, 205, 254)")
distance = distance_transform_edt(grid != 0) * resolution
field = np.exp(-0.5 * (distance / 0.12) ** 2)
free = grid == 254
scans = json.loads(args.scans.read_text())["scans"] if args.scans else capture()
if len(scans) < 20:
    raise RuntimeError("Need at least 20 frames for two independent batches")
if not args.scans:
    (folder / "scans.json").write_text(json.dumps({"scans": scans}))


def endpoints(batch):
    first = batch[0]
    for scan in batch:
        assert all(scan[k] == first[k] for k in ["frame", "inc", "a", "translation", "q"])
    values = np.asarray([s["r"] for s in batch], dtype=float)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", category=RuntimeWarning)
        ranges = np.nanmedian(values, axis=0)
    angles = first["a"] + np.arange(values.shape[1]) * first["inc"]
    valid = np.sum(np.isfinite(values), axis=0) >= len(batch) // 2 + 1
    if np.count_nonzero(valid) < 100:
        raise RuntimeError("Insufficient persistent laser returns")
    xyz = np.column_stack((ranges * np.cos(angles), ranges * np.sin(angles), np.zeros_like(ranges)))
    return (Rotation.from_quat(first["q"]).apply(xyz[valid]) + first["translation"])[:, :2]


def rotate(points, angle):
    c, s = np.cos(angle), np.sin(angle)
    return points @ np.array([[c, s], [-s, c]])


def coords(points, pose):
    xy = rotate(points, pose[2]) + pose[:2]
    return ((xy - origin) / resolution - 0.5).T[::-1]


def different(a, b):
    delta = a[2] - b[2]
    return np.linalg.norm(a[:2] - b[:2]) > 0.45 or abs(np.arctan2(np.sin(delta), np.cos(delta))) > np.deg2rad(15)


def search(points):
    rows, cols = np.nonzero(free[::3, ::3])
    rows, cols = rows * 3, cols * 3
    candidates = []
    for deg in range(-180, 180, 4):
        angle = np.deg2rad(deg)
        offsets = np.rint(rotate(points[::5], angle) / resolution).astype(int)
        x = cols[:, None] + offsets[:, 0]
        y = rows[:, None] + offsets[:, 1]
        inside = (x >= 0) & (x < grid.shape[1]) & (y >= 0) & (y < grid.shape[0])
        scores = np.mean(field[np.clip(y, 0, grid.shape[0]-1), np.clip(x, 0, grid.shape[1]-1)] * inside, axis=1)
        for idx in np.argsort(scores)[-32:]:
            xy = origin + (np.array([cols[idx], rows[idx]]) + 0.5) * resolution
            candidates.append((scores[idx], np.array([*xy, angle])))
    seeds = []
    for _, pose in sorted(candidates, key=lambda item: -item[0]):
        if all(different(pose, previous) for previous in seeds):
            seeds.append(pose)
        if len(seeds) == 20:
            break
    def loss(pose):
        base = np.floor((pose[:2] - origin) / resolution).astype(int)
        if not (0 <= base[0] < grid.shape[1] and 0 <= base[1] < grid.shape[0] and free[base[1], base[0]]):
            return 1.0
        return -map_coordinates(field, coords(points, pose), order=1, cval=0).mean()
    results = []
    radius = np.array([0.20, 0.20, np.deg2rad(5)])
    for seed in seeds:
        fit = minimize(loss, seed, method="Powell", bounds=list(zip(seed-radius, seed+radius)))
        results.append((-fit.fun, fit.x))
    distinct = []
    for score, pose in sorted(results, key=lambda item: -item[0]):
        if all(different(pose, previous) for _, previous in distinct):
            distinct.append((score, pose))
    return distinct


def as_record(result, holdout):
    score, pose = result
    residuals = map_coordinates(distance, coords(holdout, pose), order=1, cval=10)
    return dict(x_m=pose[0], y_m=pose[1], yaw_deg=np.rad2deg(pose[2]), score=score,
                holdout_points=len(residuals), within_15cm=np.mean(residuals <= 0.15),
                median_error_m=np.median(residuals))


early, late = endpoints(scans[:10]), endpoints(scans[-10:])
early_matches, late_matches = search(early), search(late)
report = dict(early=[as_record(r, late) for r in early_matches[:3]],
              late=[as_record(r, early) for r in late_matches[:3]])
(folder / "fit.json").write_text(json.dumps(report, indent=2))
print(json.dumps(report))
if not args.plot:
    raise SystemExit(0)
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
pose = early_matches[0][1]
xy = rotate(late, pose[2]) + pose[:2]
fig, ax = plt.subplots(figsize=(7, 7))
upper = origin + resolution * np.array(grid.shape[::-1])
ax.imshow(grid, origin="lower", cmap="gray", vmin=0, vmax=255, interpolation="nearest",
          extent=[origin[0], upper[0], origin[1], upper[1]])
ax.scatter(*xy.T, s=5, c="#007f83", label="Held-out laser returns")
ax.quiver(*pose[:2], np.cos(pose[2]), np.sin(pose[2]), color="#e54920", angles="xy",
          scale_units="xy", scale=1.5, width=0.009, label="Estimated base heading")
ax.set(xlabel="Map x (m)", ylabel="Map y (m)", aspect="equal", title="Stationary relocalization")
ax.legend(loc="upper right")
fig.tight_layout()
fig.savefig(folder / "overlay.png", dpi=150)
