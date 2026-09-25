"""Fault injection at the ROS/controller boundary, without a running executor."""

import threading

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger

from planar_arm_interfaces.srv import MoveTo, PickPlace
from planar_arm_control.controller_node import ControllerNode
from planar_arm_control.planning import PlanningError


@pytest.fixture
def controller():
    rclpy.init(args=[])
    node = ControllerNode()
    try:
        yield node
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_first_leg_failure_is_reported_not_an_uncaught_service_error(controller, monkeypatch):
    def fail():
        raise PlanningError('Injected first-leg validation failure')

    monkeypatch.setattr(controller, 'next_leg', fail)
    reply = controller.pick_place(PickPlace.Request(
        pick=Point(x=4., y=2.), place=Point(x=-3., y=3.), duration=1.5),
        PickPlace.Response())
    assert not reply.accepted
    assert 'Injected' in reply.message
    assert controller.state == 'FAILED' and not controller.busy
    np.testing.assert_allclose(controller.q_ref, controller.backend.read_state()[0])
    assert not controller.route
    # A failure must not leave command admission reserved forever.
    reply = controller.move(MoveTo.Request(target=Point(x=4., y=2.)), MoveTo.Response())
    assert reply.accepted


def test_feedback_failure_reference_matches_requested_hold(controller, monkeypatch):
    reply = controller.move(MoveTo.Request(target=Point(x=4., y=2.)), MoveTo.Response())
    assert reply.accepted
    controller.q_ref = controller.trajectory.goal.copy()
    monkeypatch.setattr(controller.backend, 'ready', lambda: False)
    controller.tick()
    assert controller.state == 'FAILED' and not controller.busy
    np.testing.assert_allclose(controller.q_ref, controller.backend.read_state()[0])
    np.testing.assert_array_equal(controller.dq_ref, [0., 0., 0.])


def test_cancel_during_planning_cannot_commit_a_late_result(controller, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    original = controller.planner.plan

    def delayed(*args, **kwargs):
        entered.set()
        assert release.wait(3), 'Test did not release the planner'
        return original(*args, **kwargs)

    monkeypatch.setattr(controller.planner, 'plan', delayed)
    replies = []
    worker = threading.Thread(target=lambda: replies.append(controller.move(
        MoveTo.Request(target=Point(x=4., y=2.)), MoveTo.Response())))
    worker.start()
    try:
        assert entered.wait(3)
        assert controller.cancel(Trigger.Request(), Trigger.Response()).success
    finally:
        release.set()
        worker.join(timeout=4)
    assert not worker.is_alive()
    assert len(replies) == 1 and not replies[0].accepted
    assert controller.state == 'CANCELED' and controller.trajectory is None
