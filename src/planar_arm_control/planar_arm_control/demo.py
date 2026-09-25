"""Reproducible 75-second operator demonstration, driven through GUI controls."""

from pathlib import Path
import time

from PyQt5 import QtCore, QtWidgets


class DemoDirector(QtCore.QObject):
    def __init__(self, window, app):
        super().__init__(window)
        self.window, self.app = window, app
        self.started = None
        self.stage = 0
        self.events = []
        self.snapshots = Path.cwd() / 'artifacts'
        self.snapshots.mkdir(exist_ok=True)
        self.heading = QtWidgets.QLabel('DEMO · connecting')
        self.heading.setStyleSheet('background:#204354;color:#c6f4ff;padding:8px;')
        window.centralWidget().layout().insertWidget(1, self.heading)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self.wait_started = time.monotonic()

    def announce(self, text):
        self.heading.setText(text)
        print(text, flush=True)

    def tick(self):
        w = self.window
        if w.status is None:
            if time.monotonic()-self.wait_started > 15:
                print('DEMO FAILED: controller unavailable', flush=True)
                self.app.exit(2)
            return
        if self.started is None:
            self.started = time.monotonic()
            self.announce('1 / 5 · Position control · pick (4, 2), place (−3, 3)')
        elapsed = time.monotonic()-self.started
        if self.stage == 0 and elapsed > 2 and not w.status.busy:
            w.sequence_button.click()
            self.stage = 1
        elif self.stage == 1 and elapsed > 15 and not w.status.busy:
            if w.status.state != 'SUCCEEDED':
                self.app.exit(2)
                return
            w.grab().save(str(self.snapshots / 'position.png'))
            self.announce('2 / 5 · Outside reach · requested (7, 3), projected (5.974, 2.560)')
            w.target_x.setValue(7)
            w.target_y.setValue(3)
            w.move_button.click()
            self.stage = 2
        elif self.stage == 2 and elapsed > 27 and not w.status.busy:
            if w.status.state != 'SUCCEEDED' or not w.status.projected:
                self.app.exit(2)
                return
            w.grab().save(str(self.snapshots / 'projection.png'))
            self.announce('3 / 5 · Unsafe IK result · reject target (0, 1) and hold position')
            w.target_x.setValue(0)
            w.target_y.setValue(1)
            w.move_button.click()
            self.stage = 3
        elif self.stage == 3 and elapsed > 35 and not w.status.busy:
            if w.status.state != 'REJECTED':
                self.app.exit(2)
                return
            w.reset_button.click()
            self.stage = 4
        elif self.stage == 4 and elapsed > 37 and w.status.state == 'IDLE':
            self.announce('4 / 5 · Velocity + PID control · watch reference, feedback and tracking error')
            w.change_mode('velocity_pid')
            self.stage = 5
        elif self.stage == 5 and elapsed > 39 and w.status.control_mode == 'velocity_pid':
            w.sequence_button.click()
            self.stage = 6
        elif self.stage == 6 and elapsed > 55 and not w.status.busy:
            if w.status.state != 'SUCCEEDED':
                self.app.exit(2)
                return
            w.grab().save(str(self.snapshots / 'velocity_pid.png'))
            self.announce('5 / 5 · Cancellation · stop the sequence and retain the current pose')
            w.sequence_button.click()
            self.stage = 7
        elif self.stage == 7 and elapsed > 57 and w.status.busy:
            w.cancel_button.click()
            self.stage = 8
        elif self.stage == 8 and elapsed > 62 and w.status.state == 'CANCELED':
            self.announce('DEMO COMPLETE · both modes, projection, rejection and cancellation demonstrated')
            self.stage = 9
        if elapsed > 75:
            print(f'DEMO {"PASSED" if self.stage == 9 else "FAILED"}: stage {self.stage}', flush=True)
            self.app.exit(0 if self.stage == 9 else 2)
