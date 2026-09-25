#!/usr/bin/env python3
"""Black-box acceptance test: separate ROS process, independent geometry oracle.

Never imports the controller, planner or supplied kinematics. An attractive GUI
cannot make this test pass: it observes actual messages and service responses.
"""

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import SetParameters
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from planar_arm_interfaces.msg import ControllerStatus
from planar_arm_interfaces.srv import MoveTo, PickPlace

ROOT = Path(__file__).resolve().parents[1]
EXPECTED_LIBRARY_SHA256 = '1e1c5f4bb5718839da924d192458fb2a679cc25cbded89cccf04381ad16f3dde'


def geometry(q):
    angle, x, y = 0.0, 0.0, 0.0
    heights = [0.0]
    for length, joint in zip((3.0, 2.0, 1.5), q):
        angle += joint
        x += length * math.cos(angle)
        y += length * math.sin(angle)
        heights.append(y)
    return np.array([x, y]), min(heights)


class Witness(Node):
    def __init__(self):
        super().__init__('independent_validation_witness')
        self.samples, self.events = [], []
        self.latest = None
        self.create_subscription(JointState, 'joint_states', self.observe, 100)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ControllerStatus, 'controller_status', self.status, qos)
        self.move = self.create_client(MoveTo, 'move_to_target')
        self.sequence = self.create_client(PickPlace, 'pick_place')
        self.cancel = self.create_client(Trigger, 'cancel_motion')
        self.reset = self.create_client(Trigger, 'reset_simulation')
        self.parameters = self.create_client(SetParameters, 'controller_node/set_parameters')

    def observe(self, message):
        assert list(message.name) == ['joint_1', 'joint_2', 'joint_3']
        q, velocity = np.asarray(message.position), np.asarray(message.velocity)
        assert q.shape == velocity.shape == (3,)
        assert np.all(np.isfinite(q)) and np.all(np.isfinite(velocity))
        assert np.all(q >= np.radians([0, -120, -120]) - 1e-8)
        assert np.all(q <= np.radians([180, 120, 120]) + 1e-8)
        xy, min_y = geometry(q)
        assert min_y >= -1e-6, f'Ground violation: {q}, {min_y}'
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        if self.samples:
            assert stamp > self.samples[-1][0], 'Non-monotonic telemetry timestamps'
        self.samples.append([stamp, *q, *velocity, *xy, min_y])

    def status(self, message):
        self.latest = message
        event = (message.command_id, message.state)
        if not self.events or self.events[-1][:2] != event:
            self.events.append((*event, message.detail))

    def until(self, predicate, timeout=15):
        deadline = time.monotonic() + timeout
        while not predicate():
            assert time.monotonic() < deadline, 'Timed out waiting for ROS observation'
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait(self, duration):
        deadline = time.monotonic() + duration
        self.until(lambda: time.monotonic() >= deadline, timeout=duration + 1)

    def call(self, client, request):
        assert client.wait_for_service(timeout_sec=5), 'Service unavailable'
        future = client.call_async(request)
        self.until(future.done)
        return future.result()

    def completed(self, command_id):
        self.until(lambda: self.latest and self.latest.command_id == command_id
                   and not self.latest.busy, timeout=25)
        assert self.latest.state == 'SUCCEEDED', self.latest.detail


def validate_mode(mode, artifacts):
    log_path = artifacts / f'controller_{mode}.log'
    with log_path.open('w') as log:
        process = subprocess.Popen([sys.executable, '-m', 'planar_arm_control.controller_node',
                                    '--ros-args', '-p', f'control_mode:={mode}'], stdout=log, stderr=log)
        witness = Witness()
        try:
            witness.until(lambda: witness.latest is not None)
            assert witness.latest.control_mode == mode
            for x, y, z in [(0.0, 0.0, 0.0), (0.0, 1.0, 0.0), (1.0, -1.0, 0.0),
                            (float('nan'), 1.0, 0.0), (4.0, 2.0, 1.0)]:
                response = witness.call(witness.move, MoveTo.Request(target=Point(x=x, y=y, z=z)))
                assert not response.accepted, f'Invalid target accepted: {(x,y,z)}'
            bad_sequence = witness.call(witness.sequence, PickPlace.Request(
                pick=Point(x=7.0, y=3.0), place=Point(x=-3.0, y=3.0)))
            assert not bad_sequence.accepted

            response = witness.call(witness.sequence, PickPlace.Request(
                pick=Point(x=4.0, y=2.0), place=Point(x=-3.0, y=3.0), duration=1.5))
            assert response.accepted, response.message
            busy_response = witness.call(witness.move, MoveTo.Request(target=Point(x=4.0, y=2.0)))
            assert not busy_response.accepted and 'busy' in busy_response.message
            future = witness.parameters.call_async(SetParameters.Request(
                parameters=[Parameter('control_mode', value='position').to_parameter_msg()]))
            witness.until(future.done)
            assert not future.result().results[0].successful, 'Mode change accepted during motion'
            witness.completed(response.command_id)
            phases = [event[1] for event in witness.events if event[0] == response.command_id]
            required = ['APPROACH_PICK', 'MOVING_TO_PICK', 'PICKING', 'LIFTING',
                        'TRANSFERRING', 'APPROACH_PLACE', 'MOVING_TO_PLACE',
                        'PLACING', 'RETREATING', 'SUCCEEDED']
            assert [phase for phase in phases if phase in required] == required, phases
            assert not witness.latest.holding_object and witness.latest.object_visible
            assert np.linalg.norm(np.array(witness.samples[-1][7:9]) - [-3, 3.7]) < 0.02
            assert np.linalg.norm(np.array([witness.latest.object_position.x,
                                           witness.latest.object_position.y])-[-3,3]) < 0.02
            assert np.all(np.ptp(np.array(witness.samples)[:,1:4], axis=0) > 0.1), 'Not all joints moved'
            sequence_samples = len(witness.samples)

            response = witness.call(witness.move, MoveTo.Request(target=Point(x=7.0, y=3.0), duration=1.5))
            assert response.accepted and response.projected
            expected = np.array([7.0, 3.0]) * 6.5 / math.hypot(7, 3)
            np.testing.assert_allclose([response.resolved_target.x, response.resolved_target.y], expected)
            witness.completed(response.command_id)
            assert np.linalg.norm(np.array(witness.samples[-1][7:9]) - expected) < 0.02

            response = witness.call(witness.move, MoveTo.Request(target=Point(x=4.0, y=2.0)))
            assert response.accepted
            witness.wait(0.25)
            canceled = witness.call(witness.cancel, Trigger.Request())
            assert canceled.success
            witness.until(lambda: witness.latest.state == 'CANCELED')
            witness.wait(0.1)
            held = np.array(witness.samples[-1][1:4])
            witness.wait(0.4)
            np.testing.assert_allclose(witness.samples[-1][1:4], held, atol=1e-10)

            # No GUI exists in this test: all planning/execution is autonomous.
            intervals = np.diff([sample[0] for sample in witness.samples])
            rate = 1.0 / float(np.mean(intervals))
            assert 40 <= rate <= 60, f'Unexpected telemetry rate: {rate}'
            assert float(np.max(intervals)) < 0.25, 'Telemetry stalled'
            assert witness.call(witness.reset, Trigger.Request()).success
            with (artifacts / f'telemetry_{mode}.csv').open('w') as stream:
                writer = csv.writer(stream)
                writer.writerow(['timestamp', 'q1', 'q2', 'q3', 'dq1', 'dq2', 'dq3', 'x', 'y', 'min_y'])
                writer.writerows(witness.samples)
            return {'mode': mode, 'passed': True, 'samples': len(witness.samples),
                    'sequence_samples': sequence_samples, 'mean_rate_hz': rate,
                    'max_message_interval_ms': float(np.max(intervals) * 1000),
                    'sequence_phases': required, 'projection': expected.tolist(),
                    'checks': ['joint limits', 'independent FK/ground', 'invalid inputs',
                               'unsafe IK', 'sequence order', 'endpoint accuracy',
                               'busy rejection', 'mode change rejection', 'projection',
                               'cancellation holds pose', 'fixed-rate telemetry', 'GUI independence']}
        finally:
            witness.destroy_node()
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                process.wait(timeout=3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts')
    artifacts = parser.parse_args().output.resolve()
    artifacts.mkdir(parents=True, exist_ok=True)
    library = ROOT / 'src/planar_arm_control/planar_arm_control/planar_arm.py'
    digest = hashlib.sha256(library.read_bytes()).hexdigest()
    assert digest == EXPECTED_LIBRARY_SHA256, 'Supplied library has changed'
    rclpy.init()
    try:
        report = {'library_sha256': digest, 'modes': []}
        for mode in ('position', 'velocity_pid'):
            result = validate_mode(mode, artifacts)
            report['modes'].append(result)
            print(json.dumps(result, indent=2), flush=True)
        (artifacts / 'validation.json').write_text(json.dumps(report, indent=2) + '\n')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
