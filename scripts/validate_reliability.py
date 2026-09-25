#!/usr/bin/env python3
"""Repeat/cancel/crash audit on an explicitly isolated ROS domain.

The observer imports only ROS interfaces and the independent geometry witness.
Optional GUI probes instantiate the real widgets in separate processes.
"""

import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger
from planar_arm_interfaces.srv import MoveTo, PickPlace

from validate_ros import Witness


def stop(process, crash=False):
    if process.poll() is None:
        process.send_signal(signal.SIGKILL if crash else signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=3)


class Audit:
    def __init__(self, output):
        self.output, self.processes, self.streams = output, [], []
        self.w = Witness()
        self.results = []

    def start(self, name, module=None, args=()):
        log = (self.output / f'{name}.log').open('w')
        self.streams.append(log)
        command = [sys.executable, '-m', module] if module else [sys.executable]
        process = subprocess.Popen([*command, *args], stdout=log, stderr=log)
        self.processes.append(process)
        return process

    def controller(self, name, mode):
        self.w.latest = None
        process = self.start(name, 'planar_arm_control.controller_node',
                             ['--ros-args', '-p', f'control_mode:={mode}'])
        self.w.until(lambda: self.w.latest is not None and self.w.latest.state == 'IDLE')
        assert self.w.latest.control_mode == mode
        return process

    def reset(self):
        assert self.w.call(self.w.reset, Trigger.Request()).success
        self.w.until(lambda: self.w.latest.state == 'IDLE' and not self.w.latest.command_id)

    def sequence(self):
        reply = self.w.call(self.w.sequence, PickPlace.Request(
            pick=Point(x=4., y=2.), place=Point(x=-3., y=3.), duration=1.2))
        assert reply.accepted, reply.message
        return reply.command_id

    def record(self, name, **data):
        result = {'check': name, 'passed': True, **data}
        self.results.append(result)
        print(json.dumps(result), flush=True)

    def mode(self, mode):
        process = self.controller(f'controller_{mode}', mode)
        for cycle in range(3):
            self.reset()
            token = self.sequence()
            assert not self.w.call(self.w.reset, Trigger.Request()).success
            self.w.completed(token)
            s = self.w.latest
            assert not s.holding_object
            np.testing.assert_allclose([s.object_position.x, s.object_position.y], [-3., 3.], atol=.02)
            self.record('repeat_sequence', mode=mode, cycle=cycle+1, command_id=token)

        # Exercise both sides of attachment and release. Early PLACING still
        # holds the object; RETREATING has already confirmed release.
        for phase, holding in [('APPROACH_PICK', False), ('PICKING', False),
                               ('LIFTING', True), ('TRANSFERRING', True),
                               ('PLACING', True), ('RETREATING', False)]:
            self.reset()
            token = self.sequence()
            self.w.until(lambda: self.w.latest.command_id == token
                         and self.w.latest.state == phase, timeout=25)
            assert self.w.call(self.w.cancel, Trigger.Request()).success
            self.w.until(lambda: self.w.latest.command_id == token
                         and self.w.latest.state == 'CANCELED')
            assert self.w.latest.holding_object == holding, (phase, self.w.latest)
            self.w.wait(.1)
            q = self.w.samples[-1][1:4]
            self.w.wait(.25)
            np.testing.assert_allclose(self.w.samples[-1][1:4], q, atol=1e-10)
            assert not self.w.call(self.w.cancel, Trigger.Request()).success
            if holding:
                rejected = self.w.call(self.w.sequence, PickPlace.Request(
                    pick=Point(x=4., y=2.), place=Point(x=-3., y=3.)))
                assert not rejected.accepted and 'held' in rejected.message
            self.reset()
            recovery = self.w.call(self.w.move, MoveTo.Request(target=Point(x=4., y=2.), duration=.5))
            assert recovery.accepted
            self.w.completed(recovery.command_id)
            self.record('cancel_hold_reset_recover', mode=mode, phase=phase, holding=holding)

        for duration in [-1., float('nan'), float('inf'), 61.]:
            reply = self.w.call(self.w.move, MoveTo.Request(target=Point(x=4., y=2.), duration=duration))
            assert not reply.accepted
        self.reset()
        self.w.completed(self.sequence())
        self.record('invalid_duration_then_recovery', mode=mode)
        stop(process)
        self.w.wait(.5)

    def gui_crashes(self):
        snapshot = self.output / 'gui_snapshot.json'
        probe = Path(__file__).with_name('gui_audit_probe.py')
        controller = self.controller('controller_before_crash', 'position')
        gui = self.start('gui_before_crash', args=[str(probe), str(snapshot)])

        def view(predicate):
            if not snapshot.exists():
                return False
            data = json.loads(snapshot.read_text())
            return time.time()-data['wall_time'] < 1.0 and predicate(data)

        self.w.until(lambda: view(lambda d: d['move_enabled']), timeout=20)
        token = self.sequence()
        self.w.until(lambda: view(lambda d: d['command_id'] == token and d['busy']))
        stop(gui, crash=True)
        self.w.completed(token)
        self.record('GUI_process_crash_does_not_abort_controller')

        gui = self.start('gui_after_crash', args=[str(probe), str(snapshot)])
        self.w.until(lambda: view(lambda d: d['pid'] == gui.pid and d['command_id'] == token
                                 and d['state'] == 'SUCCEEDED'), timeout=20)
        self.record('GUI_restart_receives_existing_state')

        self.reset()
        token = self.sequence()
        self.w.until(lambda: self.w.latest.command_id == token and self.w.latest.state == 'TRANSFERRING')
        stop(controller, crash=True)
        self.w.until(lambda: view(lambda d: d['stale'] and not any(d['commands_enabled'])), timeout=5)
        self.record('controller_crash_disables_GUI_commands')
        controller = self.controller('controller_after_crash', 'position')
        self.w.until(lambda: view(lambda d: d['state'] == 'IDLE' and d['move_enabled']
                                 and not d['command_id']), timeout=10)
        self.w.wait(1.)
        assert self.w.latest.state == 'IDLE' and not self.w.latest.busy
        self.record('controller_restart_is_idle_no_command_replay')

        # The recorder is installed as a console entry point, not a -m module.
        logger = self.start('logger_before_crash', args=[
            '-c', 'from planar_arm_control.telemetry_recorder import main; main()',
            '--ros-args', '-p', f'output_directory:={self.output / "interrupted_logs"}'])
        token = self.sequence()
        self.w.wait(2.)
        assert logger.poll() is None
        stop(logger, crash=True)
        self.w.completed(token)
        self.record('logger_process_crash_does_not_abort_controller')
        stop(gui)
        stop(controller)

    def close(self):
        for process in reversed(self.processes):
            stop(process)
        for stream in self.streams:
            stream.close()
        self.w.destroy_node()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gui', action='store_true', help='Also exercise real Qt/OpenGL process restarts')
    parser.add_argument('--output', type=Path, default=Path('artifacts/reliability') /
                        datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    args = parser.parse_args()
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    if domain in ('0', '67', '68'):
        parser.error('Set an unused isolated ROS domain, e.g. KINESHIA_ROS_DOMAIN_ID=72 source scripts/env.sh')
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    audit = Audit(args.output.resolve())
    started = time.monotonic()
    report = {'passed': False, 'ros_domain': domain, 'checks': audit.results,
              'python': sys.executable}
    try:
        audit.w.wait(1.)
        assert audit.w.latest is None, 'Another controller is publishing on this domain; choose an unused domain'
        for mode in ('position', 'velocity_pid'):
            audit.mode(mode)
        if args.gui:
            audit.gui_crashes()
        report['passed'] = True
    except BaseException as error:
        report['error'] = f'{type(error).__name__}: {error}'
        raise
    finally:
        report['elapsed_seconds'] = time.monotonic()-started
        report['joint_samples_checked'] = len(audit.w.samples)
        audit.close()
        rclpy.shutdown()
        (args.output / 'report.json').write_text(json.dumps(report, indent=2)+'\n')


if __name__ == '__main__':
    main()
