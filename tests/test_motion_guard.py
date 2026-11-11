import math
from pathlib import Path
import pytest
from r1pro_waypoint_nav.motion_guard import GuardConfig,MotionGuard,load_config,pipeline_issues
from r1pro_waypoint_nav.rotation_control import angular_command


def test_one_bad_status_does_not_abort_or_stop_fresh_data():
    g=MotionGuard()
    assert g.update(1.,diagnostic=['filter:incomplete_timestamped_tf'])=='degraded'
    assert g.may_move
    assert g.update(1.5,diagnostic=['filter:incomplete_timestamped_tf'])=='degraded'
    assert g.update(1.6)=='run' and g.pauses==0


def test_persistent_status_holds_then_requires_stable_recovery():
    g=MotionGuard()
    g.update(1.,diagnostic=['filter:tf'])
    assert g.update(1.8,diagnostic=['filter:tf'])=='pause'
    assert not g.may_move
    assert g.update(2.)=='pause'
    assert g.update(2.5)=='pause'
    assert g.update(2.8)=='run'


def test_stale_measurement_holds_without_diagnostic_grace():
    g=MotionGuard()
    assert g.update(1.,immediate=['stale_scan'])=='pause'
    assert not g.may_move


def test_intermittent_recovery_does_not_release_motion():
    g=MotionGuard();g.update(1.,immediate=['stale_scan'])
    g.update(1.2);g.update(1.8,diagnostic=['temporal:not_ready'])
    assert g.update(2.)=='pause'
    assert g.update(2.7)=='pause'
    assert g.update(2.8)=='run'


def test_hard_fault_latches_even_when_next_sample_looks_good():
    g=MotionGuard()
    assert g.update(1.,hard=['motor_error'])=='abort'
    assert g.update(2.)=='abort' and not g.may_move


def test_long_pause_aborts():
    g=MotionGuard();g.update(1.,immediate=['stale_scan'])
    assert g.update(6.,immediate=['stale_scan'])=='abort'


def test_repeated_short_pauses_are_bounded():
    g=MotionGuard()
    for i in range(3):
        t=i*2.;assert g.update(t,immediate=['stale_scan'])=='pause'
        g.update(t+.1);assert g.update(t+.9)=='run'
    assert g.update(6.,immediate=['stale_scan'])=='abort'


def test_cumulative_pause_budget_is_bounded():
    cfg=GuardConfig(max_pause_sec=5.,max_total_pause_sec=6.)
    g=MotionGuard(cfg);g.update(0.,immediate=['stale_scan']);g.update(3.);g.update(3.8)
    assert g.state=='run'
    g.update(4.,immediate=['stale_scan'])
    assert g.update(6.3,immediate=['stale_scan'])=='abort'


def test_status_delay_is_separate_from_actual_data_loss():
    cfg=GuardConfig();now=10.
    seen={k:now for k in ('imu','scan','fused','adapters','filter','temporal','slam')}
    seen['filter']=8.
    statuses={k:{'ready':True} for k in ('filter','temporal','slam')};statuses['adapters']={'odometry_samples':100}
    assert pipeline_issues(now,seen,statuses,cfg)==([],[],[])
    seen['scan']=9.
    assert 'stale_scan' in pipeline_issues(now,seen,statuses,cfg)[0]
    statuses['adapters']['odometry_fault']='feedback_timeout'
    assert pipeline_issues(now,seen,statuses,cfg)[2]==['wheel_odometry_feedback_timeout']


@pytest.mark.parametrize('direction',[1,-1])
def test_simulated_recovery_continues_remaining_angle_not_a_new_full_turn(direction):
    g=MotionGuard();dt=.02;angle=velocity=command=0.;target=direction*math.pi/3
    held=0
    for step in range(1750):
        now=step*dt
        # Brief flag first, then a real scan dropout part-way through the turn.
        diag=['filter:one_dropped_frame'] if 1.<now<1.5 else []
        immediate=['stale_scan'] if 4.<now<4.6 else []
        level=g.update(now,immediate,diag)
        assert level!='abort'
        if not g.may_move:command=0.;held+=1
        else:command=angular_command(angle,target,command,dt)
        velocity+=(command-velocity)*dt/.12;angle+=velocity*dt
        if g.may_move and abs(target)-direction*angle<=math.radians(.4):break
    else:raise AssertionError('No completion')
    for _ in range(100):velocity+=(0-velocity)*dt/.12;angle+=velocity*dt
    assert held>30 and g.pauses==1
    assert abs(math.degrees(angle)-direction*60)<1.


def test_deployed_config_is_valid():
    path=Path(__file__).resolve().parents[1]/'src/r1pro_waypoint_nav/config/rotation_guard.yaml'
    cfg=load_config(path);assert cfg.diagnostic_grace_sec==.8
    with pytest.raises(ValueError):GuardConfig(wheel_timeout_sec=.3)
