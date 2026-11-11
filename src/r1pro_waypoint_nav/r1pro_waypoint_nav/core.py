import math


def integrate(x, y, yaw, vx, vy, wz, dt):
    """Exact constant body-twist integration, including lateral motion."""
    if not all(math.isfinite(v) for v in (x, y, yaw, vx, vy, wz, dt)) or dt <= 0:
        raise ValueError('Non-finite state or invalid interval')
    a = wz * dt
    if abs(wz) < 1e-8:
        dx, dy = vx * dt, vy * dt
    else:
        dx = (vx * math.sin(a) + vy * (math.cos(a) - 1)) / wz
        dy = (vx * (1 - math.cos(a)) + vy * math.sin(a)) / wz
    return (x + math.cos(yaw)*dx - math.sin(yaw)*dy,
            y + math.sin(yaw)*dx + math.cos(yaw)*dy,
            math.atan2(math.sin(yaw+a), math.cos(yaw+a)))


class FeedbackWarmup:
    """Require continuous source and receipt timestamps before defining odom origin."""
    def __init__(self, samples=25, max_gap=.2):
        self.samples = samples
        self.max_gap = max_gap
        self.reset()

    def reset(self):
        self.count = 0
        self.last = None

    def observe(self, stamp, arrival):
        if not all(math.isfinite(v) for v in (stamp, arrival)) or stamp <= 0:
            self.reset()
            return False
        if self.last is not None and not all(0 < a-b <= self.max_gap for a,b in zip((stamp, arrival), self.last)):
            self.reset()
        self.count += 1
        self.last = (stamp, arrival)
        return self.count >= self.samples


class OdomIntegrator:
    def __init__(self, max_gap=.2):
        self.pose = (0., 0., 0.)
        self.last = None
        self.velocity = None
        self.max_gap = max_gap
        self.fault = ''

    def update(self, stamp, velocity):
        if self.fault:
            return False
        if not math.isfinite(stamp) or stamp <= 0 or not all(math.isfinite(v) for v in velocity):
            self.fault = 'invalid_measurement'
            return False
        if self.last is not None:
            dt = stamp-self.last
            if not 0 < dt <= self.max_gap:
                self.fault = 'timestamp_jump_or_gap'
                return False
            average = tuple((a+b)/2 for a,b in zip(self.velocity, velocity))
            self.pose = integrate(*self.pose, *average, dt)
        self.last, self.velocity = stamp, velocity
        return True


def validate_points(data, map_id):
    if not isinstance(data, dict) or data.get('map_id') != map_id:
        raise ValueError('Waypoint map_id does not match selected map')
    if data.get('frame_id') != 'map' or not isinstance(data.get('points'), dict):
        raise ValueError('Expected map frame and points dictionary')
    for name, point in data['points'].items():
        if not isinstance(name, str) or not isinstance(point, dict):
            raise ValueError('Invalid waypoint')
        for k in ('x','y','yaw'):
            v = point.get(k)
            if isinstance(v, bool) or not isinstance(v, (int,float)) or not math.isfinite(v):
                raise ValueError('Waypoint coordinates must be finite numbers')
    return data
