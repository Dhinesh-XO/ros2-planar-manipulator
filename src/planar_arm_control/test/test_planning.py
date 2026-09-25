"""Independent geometry and adversarial cases, not screenshots as proof."""

import math

import numpy as np
import pytest

from planar_arm_control.backends import SimBackend
from planar_arm_control.planning import INITIAL_Q, Planner, PlanningError, certify_path


def independent_points(q):
    angle, x, y = 0.0, 0.0, 0.0
    points = [(x, y)]
    for length, joint in zip([3.0, 2.0, 1.5], q):
        angle += joint
        x, y = x + length * math.cos(angle), y + length * math.sin(angle)
        points.append((x, y))
    return np.asarray(points)


@pytest.mark.parametrize('target', [(4, 2), (-3, 3), (7, 3)])
def test_required_targets_and_continuous_motion(target):
    planner = Planner()
    trajectory = planner.plan(INITIAL_Q, target, duration=0.2)
    expected = np.asarray(target) * min(1.0, 6.5 / np.linalg.norm(target))
    np.testing.assert_allclose(independent_points(trajectory.goal)[-1], expected, atol=1e-4)
    assert trajectory.projected == (target == (7, 3))
    for t in np.linspace(0, trajectory.duration, 301):
        q, velocity, acceleration = trajectory.sample(t)
        assert np.min(independent_points(q)[:, 1]) >= -1e-6
        assert np.max(np.abs(velocity)) <= planner.max_velocity + 1e-8
        assert np.max(np.abs(acceleration)) <= planner.max_acceleration + 1e-8
    for t in (0, trajectory.duration):
        _, velocity, acceleration = trajectory.sample(t)
        np.testing.assert_allclose(velocity, 0, atol=1e-10)
        np.testing.assert_allclose(acceleration, 0, atol=1e-10)


@pytest.mark.parametrize('target', [(0, 0), (0, 1), (1, -1), (np.nan, 2), (np.inf, 3)])
def test_invalid_or_unsafe_ik_results_are_rejected(target):
    with pytest.raises(PlanningError):
        Planner().plan(INITIAL_Q, target)


def test_no_motion_when_already_at_target():
    planner = Planner()
    q = np.radians([60, -40, -30])
    trajectory = planner.plan(q, independent_points(q)[-1])
    np.testing.assert_array_equal(trajectory.start, trajectory.goal)


def test_ground_crossing_between_valid_endpoints_is_rejected():
    planner = Planner()
    start = [0.6368691749, -1.2091370465, 2.0024207485]
    goal = [0.5921094857, -0.6528603255, -1.6888776973]
    assert np.min(independent_points(start)[:, 1]) >= 0
    assert np.min(independent_points(goal)[:, 1]) >= 0
    with pytest.raises(PlanningError, match='path'):
        certify_path(planner.arm, start, goal)


def test_pick_place_cannot_silently_project_objects():
    with pytest.raises(PlanningError, match='Pick/place'):
        Planner().plan(INITIAL_Q, (7, 3), allow_projection=False)


@pytest.mark.parametrize('mode', ['position', 'velocity_pid'])
def test_backend_completes_required_sequence_with_safe_feedback(mode):
    planner = Planner()
    backend = SimBackend(planner.arm)
    for target in [(4, 2), (-3, 3), (7, 3)]:
        trajectory = planner.plan(backend.read_state()[0], target)
        worst_error = 0.0
        for t in np.arange(0.02, trajectory.duration + 2.0, 0.02):
            reference, velocity, _ = trajectory.sample(t)
            q, actual_velocity = backend.step(reference, velocity, 0.02, mode)
            assert np.min(independent_points(q)[:, 1]) >= -1e-6
            worst_error = max(worst_error, float(np.max(np.abs(reference-q))))
            if t >= trajectory.duration and np.max(np.abs(q-trajectory.goal)) < 0.001:
                break
        np.testing.assert_allclose(independent_points(q)[-1], trajectory.resolved, atol=0.015)
        if mode == 'velocity_pid':
            assert worst_error > 1e-4  # A distinct plant, not a renamed position mode.
        backend.stop()
        np.testing.assert_array_equal(backend.read_state()[1], np.zeros(3))


def test_interval_certificate_against_independent_random_geometry():
    planner, rng = Planner(), np.random.default_rng(73)
    configurations = []
    for _ in range(300):
        q = rng.uniform(np.radians([0, -120, -120]), np.radians([180, 120, 120]))
        if np.min(independent_points(q)[:, 1]) >= 0:
            configurations.append(q)
    accepted = 0
    for a, b in zip(configurations[::2], configurations[1::2]):
        try:
            certify_path(planner.arm, a, b)
        except PlanningError:
            continue
        accepted += 1
        for scalar in np.linspace(0, 1, 101):
            assert np.min(independent_points(a + scalar * (b-a))[:, 1]) >= -1e-6
    assert accepted > 20
