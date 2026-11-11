import math
import pytest
from r1pro_waypoint_nav.rotation_control import angular_command


@pytest.mark.parametrize('direction',[1,-1])
def test_simulated_turn_with_lag_reaches_target_and_stops(direction):
    angle=velocity=command=0.;dt=.02;target=direction*math.pi/3
    for i in range(1000):
        new=angular_command(angle,target,command,dt)
        assert abs(new)<=.12+1e-8
        if new==0. and abs(angle)>.5:break
        assert abs(new-command)<=.1*dt+1e-8
        command=new;velocity+=(command-velocity)*dt/.12;angle+=velocity*dt
    else:raise AssertionError('Turn never completed')
    for _ in range(150):velocity+=(0.-velocity)*dt/.12;angle+=velocity*dt
    assert abs(math.degrees(angle)-direction*60)<1.


def test_no_command_past_target():
    assert angular_command(1.1,math.pi/3,.1,.02)==0
    assert angular_command(-1.1,-math.pi/3,-.1,.02)==0
