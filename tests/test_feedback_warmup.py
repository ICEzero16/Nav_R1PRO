from r1pro_waypoint_nav.core import FeedbackWarmup, OdomIntegrator


def test_startup_gap_restarts_warmup():
    gate = FeedbackWarmup(samples=3)
    assert not gate.observe(1., 10.)
    assert not gate.observe(1.02, 10.02)
    assert not gate.observe(2., 11.)
    assert gate.count == 1
    assert not gate.observe(2.02, 11.02)
    assert gate.observe(2.04, 11.04)


def test_receipt_gap_and_timestamp_reversal_restart():
    gate = FeedbackWarmup(samples=3)
    gate.observe(1., 10.)
    assert not gate.observe(1.02, 11.)
    assert gate.count == 1
    gate.observe(1.04, 11.02)
    assert not gate.observe(1.03, 11.04)
    assert gate.count == 1


def test_runtime_gap_still_latches_fault():
    gate = FeedbackWarmup(samples=3)
    odom = OdomIntegrator()
    for stamp in (1., 1.02, 1.04):
        if gate.observe(stamp, stamp):
            assert odom.update(stamp, (0., 0., 0.))
    assert odom.last == 1.04
    assert not odom.update(2., (0., 0., 0.))
    assert not odom.update(2.02, (0., 0., 0.))
    assert odom.fault == 'timestamp_jump_or_gap'
