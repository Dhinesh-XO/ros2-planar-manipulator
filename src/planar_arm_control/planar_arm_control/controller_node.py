"""ROS boundary, execution timers and completion-driven pick/place sequence."""

from collections import deque
import math
import threading
import time
import uuid

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from planar_arm_interfaces.msg import ControllerStatus
from planar_arm_interfaces.srv import MoveTo, PickPlace

from .backends import SimBackend
from .planning import JOINT_NAMES, Planner, PlanningError


def point(xy):
    return Point(x=float(xy[0]), y=float(xy[1]), z=0.0)


class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')
        self.lock = threading.RLock()
        defaults = {'publish_rate_hz': 50.0, 'trajectory_duration': 4.0,
                    'max_velocity': 1.5, 'max_acceleration': 3.0,
                    'grasp_duration': 0.7}
        for name, value in defaults.items():
            self.declare_parameter(name, value, ParameterDescriptor(read_only=True))
        self.declare_parameter('control_mode', 'position')
        self.rate = self.get_parameter('publish_rate_hz').value
        self.duration = self.get_parameter('trajectory_duration').value
        self.dwell = self.get_parameter('grasp_duration').value
        self.mode = self.get_parameter('control_mode').value
        if not (math.isfinite(self.rate) and 10 <= self.rate <= 200):
            raise ValueError('publish_rate_hz must be between 10 and 200.')
        if not (math.isfinite(self.duration) and 0.2 <= self.duration <= 60):
            raise ValueError('trajectory_duration must be between 0.2 and 60.')
        if not (math.isfinite(self.dwell) and 0.1 <= self.dwell <= 10):
            raise ValueError('grasp_duration must be between 0.1 and 10.')
        if self.mode not in ('position', 'velocity_pid'):
            raise ValueError('control_mode must be position or velocity_pid.')
        self.planner = Planner(self.get_parameter('max_velocity').value,
                               self.get_parameter('max_acceleration').value)
        self.backend = SimBackend(self.planner.arm, self.planner.max_velocity)
        self.q_ref, self.dq_ref = self.backend.read_state()
        self.state, self.detail, self.command_id = 'IDLE', 'Ready for a target.', ''
        self.busy = self.holding = self.object_visible = self.projected = False
        self.requested = self.resolved = np.asarray(self.planner.arm.end_effector(self.q_ref))
        self.object_xy = np.zeros(2)
        self.trajectory = self.place_plan = None
        self.stage_started = self.last_tick = time.monotonic()
        self.jitter = deque(maxlen=2000)
        self.command_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()
        self.joints_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.reference_pub = self.create_publisher(JointState, 'joint_commands', 10)
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(ControllerStatus, 'controller_status', status_qos)
        self.create_service(MoveTo, 'move_to_target', self.move, callback_group=self.command_group)
        self.create_service(PickPlace, 'pick_place', self.pick_place, callback_group=self.command_group)
        self.create_service(Trigger, 'cancel_motion', self.cancel)
        self.create_service(Trigger, 'reset_simulation', self.reset)
        self.create_timer(1.0 / self.rate, self.tick, callback_group=self.timer_group)
        self.create_timer(0.1, self.publish_status, callback_group=self.timer_group)
        self.add_on_set_parameters_callback(self.change_parameters)
        self.get_logger().info(f'Ready: {self.mode}, {self.rate:g} Hz. Kinematics unchanged.')

    def change_parameters(self, parameters):
        with self.lock:
            for parameter in parameters:
                if parameter.name == 'control_mode':
                    if self.busy:
                        return SetParametersResult(successful=False, reason='Cancel/finish motion before changing mode.')
                    if parameter.value not in ('position', 'velocity_pid'):
                        return SetParametersResult(successful=False, reason='Supported: position, velocity_pid.')
            for parameter in parameters:
                if parameter.name == 'control_mode':
                    self.mode = parameter.value
                    self.backend.stop()
                    self.q_ref, self.dq_ref = self.backend.read_state()
                    self.detail = f'Control mode changed to {self.mode}.'
        return SetParametersResult(successful=True)

    def reserve(self, response, sequence=False):
        with self.lock:
            if self.busy:
                response.message = 'Controller busy; cancel or wait for completion.'
                return None
            if sequence and self.holding:
                response.message = 'Object is already held; reset the simulation before a new sequence.'
                return None
            self.command_id = uuid.uuid4().hex[:12]
            self.busy, self.state, self.detail = True, 'PLANNING', 'Validating target and complete path.'
            self.projected = False
            return self.command_id, self.backend.read_state()[0]

    @staticmethod
    def coordinates(target):
        if not math.isfinite(target.z) or abs(target.z) > 1e-9:
            raise PlanningError('This arm is planar; target z must be zero.')
        return (target.x, target.y)

    def reject(self, token, response, error):
        response.message = str(error)
        with self.lock:
            if self.command_id == token and self.state == 'PLANNING':
                self.busy, self.state, self.detail = False, 'REJECTED', str(error)
        self.get_logger().warning(f'Rejected: {error}')
        return response

    def begin_trajectory(self, trajectory, state):
        self.trajectory = trajectory
        self.requested, self.resolved = trajectory.requested, trajectory.resolved
        self.projected = trajectory.projected
        self.stage_started = time.monotonic()
        self.state = state
        self.detail = ('Target projected to outer reach boundary. ' if trajectory.projected else '')
        self.detail += f'Executing {trajectory.duration:.2f} s quintic trajectory.'

    def move(self, request, response):
        reservation = self.reserve(response)
        if reservation is None:
            return response
        token, q = reservation
        try:
            trajectory = self.planner.plan(q, self.coordinates(request.target),
                                           request.duration or self.duration)
        except (PlanningError, ValueError, ArithmeticError) as error:
            return self.reject(token, response, error)
        with self.lock:
            if self.command_id != token or self.state != 'PLANNING':
                response.message = 'Planning was canceled.'
                return response
            self.begin_trajectory(trajectory, 'MOVING')
            response.accepted = True
            response.command_id = token
            response.projected = trajectory.projected
            response.resolved_target = point(trajectory.resolved)
            response.message = self.detail
        return response

    def pick_place(self, request, response):
        reservation = self.reserve(response, sequence=True)
        if reservation is None:
            return response
        token, q = reservation
        try:
            duration = request.duration or self.duration
            pick = self.planner.plan(q, self.coordinates(request.pick), duration, allow_projection=False)
            place = self.planner.plan(pick.goal, self.coordinates(request.place), duration, allow_projection=False)
        except (PlanningError, ValueError, ArithmeticError) as error:
            return self.reject(token, response, error)
        with self.lock:
            if self.command_id != token or self.state != 'PLANNING':
                response.message = 'Planning was canceled.'
                return response
            self.place_plan = place
            self.object_xy, self.object_visible = pick.resolved.copy(), True
            self.begin_trajectory(pick, 'MOVING_TO_PICK')
            response.accepted, response.command_id = True, token
            response.message = 'Both paths validated; starting pick/place.'
        return response

    def cancel(self, request, response):
        del request
        with self.lock:
            response.success = self.busy
            response.message = 'Motion canceled; holding present pose.' if self.busy else 'No active motion.'
            if self.busy:
                self.finish('CANCELED', response.message)
                self.q_ref, self.dq_ref = self.backend.read_state()
        return response

    def reset(self, request, response):
        del request
        with self.lock:
            response.success = not self.busy
            response.message = 'Cancel/finish before resetting.' if self.busy else 'Simulation reset to initial pose.'
            if not self.busy:
                self.backend.reset()
                self.q_ref, self.dq_ref = self.backend.read_state()
                self.state, self.detail, self.command_id = 'IDLE', response.message, ''
                self.holding = self.object_visible = self.projected = False
                self.requested = self.resolved = np.asarray(self.planner.arm.end_effector(self.q_ref))
        return response

    def finish(self, state, detail):
        self.backend.stop()
        self.busy, self.state, self.detail = False, state, detail
        self.trajectory = None
        self.dq_ref = np.zeros(3)

    def tick(self):
        now = time.monotonic()
        dt, self.last_tick = now - self.last_tick, now
        self.jitter.append(abs(dt - 1.0 / self.rate) * 1000)
        with self.lock:
            try:
                self.advance(now, dt)
            except (PlanningError, ValueError, ArithmeticError) as error:
                self.finish('FAILED', str(error))
                self.q_ref, self.dq_ref = self.backend.read_state()
                self.get_logger().error(str(error))
            q, velocity = self.backend.read_state()
            if self.holding:
                self.object_xy = np.asarray(self.planner.arm.end_effector(q))
            stamp = self.get_clock().now().to_msg()
            for publisher, position, speeds in ((self.joints_pub, q, velocity),
                                                 (self.reference_pub, self.q_ref, self.dq_ref)):
                message = JointState()
                message.header.stamp, message.header.frame_id = stamp, 'base'
                message.name = JOINT_NAMES
                message.position, message.velocity = position.tolist(), speeds.tolist()
                publisher.publish(message)

    def advance(self, now, dt):
        if not self.busy or self.state == 'PLANNING':
            return
        if dt > 0.25:
            raise PlanningError('Execution timer stalled for more than 250 ms; motion stopped.')
        elapsed = now - self.stage_started
        if self.state in ('MOVING', 'MOVING_TO_PICK', 'MOVING_TO_PLACE'):
            trajectory = self.trajectory
            self.q_ref, self.dq_ref, _ = trajectory.sample(elapsed)
            q, velocity = self.backend.step(self.q_ref, self.dq_ref, dt, self.mode)
            settled = (np.max(np.abs(q - trajectory.goal)) < 0.001
                       and np.max(np.abs(velocity)) < 0.01)
            if elapsed > trajectory.duration + 3.0 and not settled:
                raise PlanningError('Tracking did not settle within three seconds of the trajectory end.')
            if elapsed >= trajectory.duration and settled:
                self.backend.stop()
                if self.state == 'MOVING':
                    self.finish('SUCCEEDED', 'Reached projected target.' if self.projected else 'Target reached.')
                else:
                    self.state = 'PICKING' if self.state == 'MOVING_TO_PICK' else 'PLACING'
                    self.detail = 'Simulated grasp dwell.' if self.state == 'PICKING' else 'Simulated release dwell.'
                    self.stage_started = now
        elif self.state == 'PICKING' and elapsed >= self.dwell:
            self.holding = True
            # Replan from measured state, which can differ slightly after tracking.
            place = self.planner.plan(self.backend.read_state()[0], self.place_plan.requested,
                                      self.place_plan.duration, allow_projection=False)
            self.begin_trajectory(place, 'MOVING_TO_PLACE')
        elif self.state == 'PLACING' and elapsed >= self.dwell:
            self.holding = False
            self.object_xy = np.asarray(self.planner.arm.end_effector(self.backend.read_state()[0]))
            self.finish('SUCCEEDED', 'Pick/place complete; simulated object released.')

    def publish_status(self):
        with self.lock:
            q, _ = self.backend.read_state()
            message = ControllerStatus()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = 'base'
            message.command_id, message.state, message.detail = self.command_id, self.state, self.detail
            message.control_mode, message.busy = self.mode, self.busy
            message.holding_object, message.object_visible = self.holding, self.object_visible
            message.projected = self.projected
            message.requested_target, message.resolved_target = point(self.requested), point(self.resolved)
            message.end_effector = point(self.planner.arm.end_effector(q))
            message.object_position = point(self.object_xy)
            message.tracking_error = (self.q_ref - q).tolist()
            message.timer_jitter_mean_ms = float(np.mean(self.jitter)) if self.jitter else 0.0
            message.timer_jitter_max_ms = float(max(self.jitter)) if self.jitter else 0.0
            self.status_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.backend.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
