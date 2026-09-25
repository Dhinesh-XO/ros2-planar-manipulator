"""Execution boundary and an illustrative simulated velocity servo.

This is not a dynamics/current model. Real hardware must return measured state
and implement stop locally; it must not report its command as measured feedback.
"""

from abc import ABC, abstractmethod
import time

import numpy as np

from .planning import INITIAL_Q, certify_path


class Backend(ABC):
    """Full boundary required by the controller; no GUI/ROS planning inside it."""

    @abstractmethod
    def read_state(self):
        """Return measured (positions, velocities), in radians and rad/s."""

    @abstractmethod
    def step(self, q_ref, dq_ref, dt, mode):
        """Execute a reference for one interval; return actual state."""

    @abstractmethod
    def stop(self):
        """Stop movement and hold the current position."""

    @abstractmethod
    def ready(self):
        """True only after initialization and with fresh execution feedback."""

    @abstractmethod
    def motion_time(self):
        """Monotonic trajectory time for this plant, in seconds."""

    @abstractmethod
    def hold_reference(self, q_ref):
        """Retain a successfully settled reference without a stop transient."""

    @abstractmethod
    def set_gripper(self, opening):
        """Request normalized opening: 0 closed, 1 open."""

    @abstractmethod
    def grasp(self, target):
        """Nonblocking request/poll; True after attachment is confirmed."""

    @abstractmethod
    def release(self):
        """Nonblocking request/poll; True after release is confirmed."""

    @abstractmethod
    def object_state(self):
        """Measured (planar position, rotation), or None for ideal simulation."""

    @abstractmethod
    def is_holding(self):
        """Measured attachment state, or None when controller owns ideal grasp."""


class SimBackend(Backend):
    name = 'simulation'
    position_tolerance = 0.001

    def __init__(self, arm, max_velocity=1.5):
        self.arm = arm
        self.max_velocity = max_velocity
        self.q = INITIAL_Q.copy()
        self.velocity = np.zeros(3)
        self.integral = np.zeros(3)

    def read_state(self):
        return self.q.copy(), self.velocity.copy()

    def ready(self):
        return True

    def motion_time(self):
        return time.monotonic()

    def set_gripper(self, opening):
        pass

    def grasp(self, target):
        return True

    def release(self):
        return True

    def object_state(self):
        return None

    def is_holding(self):
        return None  # Ideal simulation attachment is controller-owned.

    def hold_reference(self, q_ref):
        self.stop()

    def stop(self):
        self.velocity[:] = 0.0
        self.integral[:] = 0.0

    def reset(self):
        self.q = INITIAL_Q.copy()
        self.stop()

    def step(self, q_ref, dq_ref, dt, mode):
        if mode == 'position':
            certify_path(self.arm, self.q, q_ref)
            self.q = q_ref.copy()
            self.velocity = dq_ref.copy()
        elif mode == 'velocity_pid':
            # Substeps keep the first-order velocity actuator numerically stable.
            remaining = dt
            while remaining > 1e-9:
                step = min(remaining, 0.005)
                error = q_ref - self.q
                integral = np.clip(self.integral + error * step, -0.25, 0.25)
                command = np.clip(dq_ref + 5.0 * error + 0.3 * integral
                                  + 0.08 * (dq_ref - self.velocity),
                                  -self.max_velocity, self.max_velocity)
                velocity = self.velocity + (command - self.velocity) * (1 - np.exp(-step / 0.05))
                candidate = self.q + velocity * step
                certify_path(self.arm, self.q, candidate)
                self.q, self.velocity, self.integral = candidate, velocity, integral
                remaining -= step
        else:
            raise ValueError(f'Unsupported control mode: {mode}')
        return self.read_state()
