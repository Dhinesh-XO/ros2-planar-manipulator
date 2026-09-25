#!/usr/bin/env python3
"""Black-box physical-backend witness. Run on an otherwise unused ROS domain.

Launches/owns a headless workcell. Observes raw Gazebo joints, physical payload
poses and attachment acknowledgements in addition to the public controller API.
Does not import the planner, backend, GUI or provided kinematics.
"""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from planar_arm_interfaces.srv import MoveTo, PickPlace

from validate_ros import ROOT, Witness


class PhysicsWitness(Witness):
    def __init__(self):
        super().__init__()
        self.physical_joints, self.objects, self.attachments, self.references = [], [], [], []
        self.create_subscription(JointState, '/gazebo/joint_states', self.raw_joints, 100)
        self.create_subscription(TFMessage, '/gazebo/object_pose', self.raw_object, 100)
        self.create_subscription(String, '/gazebo/grasp_state',
                                 lambda m: self.attachments.append(m.data), 100)
        self.create_subscription(JointState, '/joint_commands',
                                 lambda m: self.references.append(list(m.position)), 100)

    def raw_joints(self, message):
        indices = [message.name.index(f'joint_{i}') for i in (1,2,3)]
        self.physical_joints.append([message.position[i] for i in indices])

    def raw_object(self, message):
        for tf in message.transforms:
            if tf.child_frame_id == 'payload':
                p = tf.transform.translation
                self.objects.append([p.x/.1, (p.z-.08)/.1, p.y])

    def completed(self, command_id):
        self.until(lambda: self.latest and self.latest.command_id == command_id
                   and not self.latest.busy, timeout=65)
        assert self.latest.state == 'SUCCEEDED', self.latest.detail


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT / 'artifacts')
    artifacts = parser.parse_args().output.resolve()
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '0', '67', '68'), 'Use an isolated test domain, e.g. 71'
    partition = 'kineshia_'+os.environ['ROS_DOMAIN_ID']
    rclpy.init()
    witness = PhysicsWitness()
    artifacts.mkdir(parents=True, exist_ok=True)
    report = {'passed': False}
    with (artifacts / 'gazebo_validation.log').open('w') as log:
        process = subprocess.Popen(['ros2', 'launch', 'planar_arm_control', 'gazebo.launch.py',
                                    'gui:=false', 'record_telemetry:=true',
                                    f'telemetry_directory:={artifacts / "telemetry"}'],
                                   stdout=log, stderr=log, start_new_session=True)
        try:
            witness.until(lambda: witness.latest and witness.latest.state == 'IDLE', timeout=25)
            assert witness.latest.backend == 'gazebo'
            np.testing.assert_allclose(witness.objects[-1][:2], [4,2], atol=.05)
            start_samples = len(witness.physical_joints)
            start_objects = len(witness.objects)
            start_attachments = len(witness.attachments)
            response = witness.call(witness.sequence, PickPlace.Request(
                pick=Point(x=4.,y=2.), place=Point(x=-3.,y=3.), duration=2.5))
            assert response.accepted, response.message
            witness.completed(response.command_id)
            witness.wait(1.5)
            required = ['APPROACH_PICK','MOVING_TO_PICK','PICKING','LIFTING','TRANSFERRING',
                        'APPROACH_PLACE','MOVING_TO_PLACE','PLACING','RETREATING','SUCCEEDED']
            phases = [event[1] for event in witness.events if event[0] == response.command_id]
            assert [phase for phase in phases if phase in required] == required, phases
            attachment_events = witness.attachments[start_attachments:]
            assert 'attached' in attachment_events and 'detached' in attachment_events
            assert attachment_events.index('attached') < attachment_events.index('detached')
            physical = np.array(witness.physical_joints[start_samples:])
            assert np.all(np.ptp(physical, axis=0) > .1)
            objects = np.array(witness.objects[start_objects:])
            assert np.ptp(objects[:,0]) > 6.5 and np.max(objects[:,1]) > 4.0
            final_object = witness.objects[-1][:2]
            assert np.linalg.norm(np.array(final_object)-[-3,3]) < .15, final_object
            assert np.linalg.norm(np.array(witness.samples[-1][7:9])-[-3,3.7]) < .08
            assert not witness.latest.holding_object
            response = witness.call(witness.move, MoveTo.Request(target=Point(x=7.,y=3.), duration=3.0))
            assert response.accepted and response.projected
            witness.completed(response.command_id)
            expected = np.array([7.,3.])*6.5/np.hypot(7,3)
            assert np.linalg.norm(np.array(witness.samples[-1][7:9])-expected) < .08
            # Pause the physical clock mid-command: must fail closed, not finish
            # using wall time or continue reporting invented moving feedback.
            response = witness.call(witness.move, MoveTo.Request(target=Point(x=4.,y=2.), duration=3.0))
            assert response.accepted
            witness.wait(.25)
            env = dict(os.environ, GZ_PARTITION=partition)
            subprocess.run(['gz','service','-s','/world/workcell/control',
                            '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean',
                            '--timeout','2000','--req','pause: true'], env=env,
                           check=True, capture_output=True, timeout=5)
            witness.until(lambda: witness.latest.state == 'FAILED', timeout=4)
            assert 'stale' in witness.latest.detail
            # Resume the clock: a failed command must not silently restart.
            subprocess.run(['gz','service','-s','/world/workcell/control',
                            '--reqtype','gz.msgs.WorldControl','--reptype','gz.msgs.Boolean',
                            '--timeout','2000','--req','pause: false'], env=env,
                           check=True, capture_output=True, timeout=5)
            count = len(witness.samples)
            witness.until(lambda: len(witness.samples) >= count+15, timeout=5)
            assert witness.latest.state == 'FAILED' and not witness.latest.busy
            assert np.max(np.abs(np.array(witness.references[-1])-
                                 np.array(witness.samples[-1][1:4]))) < .04
            report = {'passed': True, 'physical_joint_samples': len(physical),
                      'physical_object_samples': len(objects), 'sequence_phases': required,
                      'attachment_events': attachment_events, 'final_object_xy': final_object,
                      'projection': expected.tolist(), 'clock_stall_detected': True,
                      'clock_resume_does_not_replay_command': True,
                      'limitations': ['constraint-based grasp, not frictional grasp',
                                      'simplified fixture collisions', 'no hardware safety validation']}
            print(json.dumps(report, indent=2), flush=True)
        finally:
            (artifacts / 'gazebo_validation.json').write_text(json.dumps(report, indent=2)+'\n')
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            witness.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
