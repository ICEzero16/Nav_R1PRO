"""Select SLAM keyframes by translation OR yaw, without ROS dependencies."""
import math


class ScanGate:
    def __init__(self, distance=.1, heading=.1):
        if not all(math.isfinite(v) and v > 0 for v in (distance, heading)):
            raise ValueError('Keyframe thresholds must be positive and finite')
        self.distance = distance
        self.heading = heading
        self.accepted = None
        self.accepted_stamp = None

    def wants(self, pose):
        if not all(math.isfinite(v) for v in pose):
            return False
        if self.accepted is None:
            return True
        x, y, yaw = pose
        px, py, pyaw = self.accepted
        angle = abs(math.atan2(math.sin(yaw-pyaw), math.cos(yaw-pyaw)))
        return math.hypot(x-px, y-py) >= self.distance or angle >= self.heading

    def observe_accepted(self, pose, stamp):
        # A relayed scan may be rejected downstream. Advance only when native
        # SLAM publishes an accepted pose and its timestamped odometry exists.
        if not all(math.isfinite(v) for v in pose) or not math.isfinite(stamp):
            return False
        if self.accepted_stamp is not None and stamp <= self.accepted_stamp:
            return False
        self.accepted = tuple(pose)
        self.accepted_stamp = stamp
        return True
