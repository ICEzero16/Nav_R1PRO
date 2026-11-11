import math
import pytest
from r1pro_waypoint_nav.translation_control import linear_command,bounded_velocity,translation_fault


@pytest.mark.parametrize('direction',[-1,1])
def test_20cm_with_lag_stops_within_one_cm(direction):
    position=velocity=command=0.;dt=.02;target=direction*.2
    for i in range(1000):
        new=linear_command(position,target,command,dt)
        assert abs(new)<=.04+1e-8
        if new==0 and abs(position)>.15:break
        assert abs(new-command)<=.04*dt+1e-8
        command=new;velocity+=(command-velocity)*dt/.12;position+=velocity*dt
    else:raise AssertionError('Translation did not finish')
    for i in range(150):velocity+=(0-velocity)*dt/.12;position+=velocity*dt
    assert abs(position-target)<.01


def test_distance_direction_and_lateral_limits():
    assert not translation_fault((-.19,.001,.001),(-.04,0,0),-.2)
    assert translation_fault((-.221,0,0),(-.04,0,0),-.2)
    assert translation_fault((.006,0,0),(.03,0,0),-.2)
    assert translation_fault((-.1,.026,0),(-.03,0,0),-.2)
    assert translation_fault((-.1,0,.061),(-.03,0,0),-.2)
    assert translation_fault((-.1,0,0),(-.06,0,0),-.2)


def test_sender_limits_and_no_mixed_commands():
    assert bounded_velocity(0.,-.04)==(0.,-.04)
    assert bounded_velocity(.12)==(.12,0.)
    for w,v in [(0,.05),(.13,0),(.1,.01),(math.nan,0),(0,math.inf)]:
        with pytest.raises(ValueError):bounded_velocity(w,v)
    assert linear_command(-.21,-.2,-.01,.02)==0
    with pytest.raises(ValueError):linear_command(0,.3,0,.02)
