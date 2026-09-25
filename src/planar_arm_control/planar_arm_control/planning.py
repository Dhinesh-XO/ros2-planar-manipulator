"""Pure planning, separate from ROS, Qt and command execution.

The supplied IK is an untrusted numerical boundary: validate its results without
changing its implementation. Paths are conservative: certify or reject, never
claim to have found every possible collision-free path.
"""

from dataclasses import dataclass
import math

import numpy as np

from .planar_arm import PlanarArm

LINK_LENGTHS = (3.0, 2.0, 1.5)
INITIAL_Q = np.radians([30.0, -20.0, -10.0])
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3']


class PlanningError(ValueError):
    """A command cannot be executed within the supported motion model."""


def check_configuration(arm, q):
    q = np.asarray(q, dtype=float)
    if q.shape != (3,) or not np.all(np.isfinite(q)):
        raise PlanningError('Joint configuration must contain three finite angles.')
    if not arm.within_joint_limits(q):
        raise PlanningError('IK/motion violates joint limits.')
    if not arm.arm_above_base(q):
        raise PlanningError('IK/motion violates the ground constraint.')
    return q


def _minimum_sine(a, b):
    lo, hi = sorted((a, b))
    # sin(theta) attains -1 at -pi/2 + 2*k*pi.
    k = math.ceil((lo + math.pi / 2.0) / (2.0 * math.pi))
    if -math.pi / 2.0 + k * 2.0 * math.pi <= hi:
        return -1.0
    return min(math.sin(lo), math.sin(hi))


def certify_path(arm, start, goal):
    """Bound every link endpoint's height over the continuous joint-space path.

Joint limits are convex and each joint interpolates monotonically. For ground
clearance, interval bounds on sine cover ALL scalar path positions, rather than
only timer samples. Subdivide conservative bounds; fail closed if undecidable.
    """
    start = check_configuration(arm, start)
    goal = check_configuration(arm, goal)
    theta0, theta1 = np.cumsum(start), np.cumsum(goal)

    def safe_interval(lo, hi, depth):
        angles_a = theta0 + lo * (theta1 - theta0)
        angles_b = theta0 + hi * (theta1 - theta0)
        lower_heights = np.cumsum([
            length * _minimum_sine(a, b)
            for length, a, b in zip(arm.link_lengths, angles_a, angles_b)
        ])
        if np.all(lower_heights >= -1e-6):
            return True
        mid = (lo + hi) / 2.0
        if not arm.arm_above_base(start + mid * (goal - start)) or depth == 14:
            return False
        return (safe_interval(lo, mid, depth + 1)
                and safe_interval(mid, hi, depth + 1))

    if not safe_interval(0.0, 1.0, 0):
        raise PlanningError('Direct joint path is unsafe or cannot be certified; '
                            'choose an intermediate target.')


@dataclass(frozen=True)
class Trajectory:
    start: np.ndarray
    goal: np.ndarray
    duration: float
    requested: np.ndarray
    resolved: np.ndarray
    projected: bool

    def sample(self, elapsed):
        u = float(np.clip(elapsed / self.duration, 0.0, 1.0))
        s = 10 * u**3 - 15 * u**4 + 6 * u**5
        ds = (30 * u**2 - 60 * u**3 + 30 * u**4) / self.duration
        dds = (60 * u - 180 * u**2 + 120 * u**3) / self.duration**2
        delta = self.goal - self.start
        return self.start + s * delta, ds * delta, dds * delta


class Planner:
    def __init__(self, max_velocity=1.5, max_acceleration=3.0):
        self.arm = PlanarArm(LINK_LENGTHS)
        self.max_velocity = float(max_velocity)
        self.max_acceleration = float(max_acceleration)
        if not (math.isfinite(self.max_velocity) and self.max_velocity > 0
                and math.isfinite(self.max_acceleration) and self.max_acceleration > 0):
            raise PlanningError('Motion limits must be finite and positive.')

    def plan(self, current, target, duration=4.0, allow_projection=True):
        current = check_configuration(self.arm, current).copy()
        target = np.asarray(target, dtype=float)
        if target.shape != (2,) or not np.all(np.isfinite(target)):
            raise PlanningError('Target must contain two finite coordinates.')
        if target[1] < 0:
            raise PlanningError('Target is below the ground (y < 0).')
        if not math.isfinite(duration) or not 0.2 <= duration <= 60.0:
            raise PlanningError('Requested duration must be between 0.2 and 60 seconds.')
        resolved = np.asarray(self.arm.reachable_target(target))
        projected = bool(np.linalg.norm(resolved - target) > 1e-8)
        if projected and not allow_projection:
            raise PlanningError('Pick/place targets must be reachable; projected '
                                'targets cannot represent a successful grasp/place.')
        if np.linalg.norm(np.asarray(self.arm.end_effector(current)) - resolved) < 1e-5:
            goal = current.copy()
        else:
            goal = check_configuration(
                self.arm, self.arm.inverse_kinematics(resolved, initial_guess=current))
        residual = np.linalg.norm(np.asarray(self.arm.end_effector(goal)) - resolved)
        if residual > 1e-4:
            raise PlanningError(f'IK did not reach target (residual {residual:.4f}).')
        certify_path(self.arm, current, goal)
        distance = float(np.max(np.abs(goal - current)))
        # Exact maxima of quintic scalar velocity/acceleration on [0,1].
        duration = max(duration, 1.875 * distance / self.max_velocity,
                       math.sqrt((10 / math.sqrt(3)) * distance / self.max_acceleration))
        return Trajectory(current, goal.copy(), duration, target.copy(),
                          resolved.copy(), projected)
