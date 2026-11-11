from types import SimpleNamespace as S
import time
from geometry_msgs.msg import TwistStamped
from r1pro_waypoint_nav.adapters import Adapters
from r1pro_waypoint_nav.core import OdomIntegrator,FeedbackWarmup


def dummy(fresh=True):
    odom=OdomIntegrator();odom.update(10.,(0.,0.,0.))
    return S(odom=odom,warmup=FeedbackWarmup(),fresh_stamp=lambda stamp:fresh,
             feedback_rejected=0,feedback_rejection_reason='',last_feedback=time.monotonic())


def test_old_timestamp_is_rejected_without_latching_fault_or_refreshing_watchdog():
    node=dummy();msg=TwistStamped();msg.header.stamp.sec=9
    before=node.last_feedback;Adapters.feedback(node,msg)
    assert not node.odom.fault and node.odom.last==10. and node.last_feedback==before
    assert node.feedback_rejected==1


def test_stale_packet_does_not_poison_fresh_odometry():
    node=dummy(False);msg=TwistStamped();msg.header.stamp.sec=5
    Adapters.feedback(node,msg)
    assert not node.odom.fault and node.feedback_rejected==1


def test_nonfinite_packet_is_dropped_but_excessive_speed_latches():
    node=dummy();msg=TwistStamped();msg.header.stamp.sec=11;msg.twist.angular.z=float('nan')
    Adapters.feedback(node,msg)
    assert not node.odom.fault and node.feedback_rejected==1
    msg.twist.angular.z=4.;Adapters.feedback(node,msg)
    assert node.odom.fault=='invalid_or_excessive_velocity'


def test_true_gap_still_latches_after_rejected_packet():
    node=dummy();msg=TwistStamped();msg.header.stamp.sec=9;Adapters.feedback(node,msg)
    assert not node.odom.update(10.4,(0.,0.,0.))
    assert node.odom.fault=='timestamp_jump_or_gap'
