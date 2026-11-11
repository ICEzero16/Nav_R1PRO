"""Pure health policy: transient diagnostic flags, zero-speed holds, hard aborts.

No ROS calls and no actuator access. Missing primary odometry is never filled
or silently rebased. Recovery only permits continuing the remaining target.
"""
from dataclasses import dataclass,asdict
from pathlib import Path
import math
import yaml


@dataclass
class GuardConfig:
    diagnostic_grace_sec: float = .8
    recovery_stable_sec: float = .8
    max_pause_sec: float = 5.
    max_total_pause_sec: float = 10.
    max_pause_count: int = 3
    status_timeout_sec: float = 2.5
    imu_timeout_sec: float = .2
    scan_timeout_sec: float = .6
    fused_timeout_sec: float = .25
    wheel_timeout_sec: float = .15
    controller_timeout_sec: float = .2
    motor_status_timeout_sec: float = 2.
    command_ttl_sec: float = .2
    wheel_max_gap_sec: float = .2
    feedback_max_age_sec: float = .15
    max_command_time_sec: float = 20.
    max_turn_wall_sec: float = 35.
    max_linear_speed: float = .05
    max_yaw_speed: float = .25
    max_translation: float = .06
    angle_margin_deg: float = 3.
    reverse_angle_rad: float = .03
    imu_difference_rad: float = .10
    imu_difference_ratio: float = .15
    no_response_sec: float = 3.
    no_response_deg: float = 1.

    def __post_init__(self):
        if any(not math.isfinite(v) or v<=0 for v in asdict(self).values()):raise ValueError('Guard settings must be finite and positive')
        if not isinstance(self.max_pause_count,int):raise ValueError('max_pause_count must be integer')
        if self.max_pause_sec<self.recovery_stable_sec or self.max_total_pause_sec<self.max_pause_sec:raise ValueError('Invalid recovery budget')
        if self.wheel_timeout_sec>self.wheel_max_gap_sec:raise ValueError('Stop before wheel integration continuity is lost')


def load_config(path=None):
    if path is None:return GuardConfig()
    return GuardConfig(**yaml.safe_load(Path(path).read_text())['rotation_guard'])


def pipeline_issues(now,seen,statuses,cfg):
    """Return (immediate hold reasons, diagnostic-grace reasons, hard reasons)."""
    immediate=[];diagnostic=[];hard=[]
    for key,limit in [('imu',cfg.imu_timeout_sec),('scan',cfg.scan_timeout_sec),('fused',cfg.fused_timeout_sec)]:
        if now-seen.get(key,-math.inf)>limit:immediate.append('stale_'+key)
    for key in ('adapters','filter','temporal','slam'):
        if now-seen.get(key,-math.inf)>cfg.status_timeout_sec:immediate.append('stale_status_'+key)
    adapters=statuses.get('adapters',{})
    if adapters.get('odometry_fault'):hard.append('wheel_odometry_'+adapters['odometry_fault'])
    elif adapters.get('odometry_samples',0)<20:immediate.append('wheel_warmup')
    for key in ('filter','temporal','slam'):
        item=statuses.get(key,{})
        if not item.get('ready'):diagnostic.append(key+':'+str(item.get('reason',item.get('state','not_ready'))))
    if statuses.get('slam',{}).get('state')=='error':hard.append('slam_error:'+statuses['slam'].get('error',''))
    return immediate,diagnostic,hard


class MotionGuard:
    def __init__(self,cfg=None):
        self.cfg=cfg or GuardConfig();self.state='run';self.flag_since=None
        self.pause_since=None;self.good_since=None;self.pauses=0;self.total_pause=0.;self.reason=''

    def update(self,now,immediate=(),diagnostic=(),hard=()):
        if self.state=='abort':return self.state
        if hard:self.state='abort';self.reason='; '.join(hard);return self.state
        # One diagnostic failure may be a discarded frame. Require continuous
        # bad diagnostics; never grant this grace to stale actual measurements.
        if diagnostic:
            if self.flag_since is None:self.flag_since=now
        else:self.flag_since=None
        flag_expired=self.flag_since is not None and now-self.flag_since>=self.cfg.diagnostic_grace_sec-1e-9
        must_hold=bool(immediate) or flag_expired
        reasons=list(immediate)+list(diagnostic)
        if self.state!='pause' and must_hold:
            self.state='pause';self.pause_since=now;self.good_since=None;self.pauses+=1
        if self.state=='pause':
            elapsed=now-self.pause_since
            if elapsed>=self.cfg.max_pause_sec or self.total_pause+elapsed>=self.cfg.max_total_pause_sec or self.pauses>self.cfg.max_pause_count:
                self.state='abort';self.reason='recovery_budget_exhausted: '+'; '.join(reasons or [self.reason]);return self.state
            if immediate or diagnostic:self.good_since=None
            elif self.good_since is None:self.good_since=now
            elif now-self.good_since>=self.cfg.recovery_stable_sec-1e-9:
                self.total_pause+=elapsed;self.pause_since=None;self.good_since=None;self.state='run'
        else:self.state='degraded' if diagnostic else 'run'
        self.reason='; '.join(reasons)
        return self.state

    @property
    def may_move(self):return self.state in ('run','degraded')
