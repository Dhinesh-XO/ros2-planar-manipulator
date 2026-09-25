"""Live operator client: ROS executor in a worker, Qt exclusively on main thread."""

import argparse
from collections import deque
import signal
import sys
import threading
import time

import numpy as np
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
import rclpy
from geometry_msgs.msg import Point
from rcl_interfaces.srv import SetParameters
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from planar_arm_interfaces.msg import ControllerStatus
from planar_arm_interfaces.srv import MoveTo, PickPlace

from .planar_arm import PlanarArm
from .planning import JOINT_NAMES, LINK_LENGTHS
from .recording import WindowRecorder

COLORS = ['#58d9ed', '#ffbd69', '#bf9cff']


class Bridge(QtCore.QObject):
    joints = QtCore.pyqtSignal(object)
    reference = QtCore.pyqtSignal(object)
    status = QtCore.pyqtSignal(object)
    reply = QtCore.pyqtSignal(str, bool, str)


class GuiNode(Node):
    def __init__(self, bridge):
        super().__init__('gui_node')
        self.bridge = bridge
        self.create_subscription(JointState, 'joint_states', bridge.joints.emit, 10)
        self.create_subscription(JointState, 'joint_commands', bridge.reference.emit, 10)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ControllerStatus, 'controller_status', bridge.status.emit, qos)
        self.move = self.create_client(MoveTo, 'move_to_target')
        self.sequence = self.create_client(PickPlace, 'pick_place')
        self.cancel = self.create_client(Trigger, 'cancel_motion')
        self.reset = self.create_client(Trigger, 'reset_simulation')
        self.mode = self.create_client(SetParameters, 'controller_node/set_parameters')

    def request(self, label, client, request):
        if not client.service_is_ready():
            self.bridge.reply.emit(label, False, 'Controller service is unavailable.')
            return
        future = client.call_async(request)

        def complete(done):
            try:
                response = done.result()
                if hasattr(response, 'accepted'):
                    ok, text = response.accepted, response.message
                    if hasattr(response, 'projected') and response.projected:
                        p = response.resolved_target
                        text += f' Resolved target: ({p.x:.3f}, {p.y:.3f}).'
                elif hasattr(response, 'results'):
                    ok = all(item.successful for item in response.results)
                    text = '; '.join(item.reason for item in response.results) or 'Mode updated.'
                else:
                    ok, text = response.success, response.message
                self.bridge.reply.emit(label, ok, text)
            except Exception as error:
                self.bridge.reply.emit(label, False, str(error))

        future.add_done_callback(complete)


class ArmWindow(QtWidgets.QMainWindow):
    def __init__(self, node, bridge):
        super().__init__()
        self.node, self.arm = node, PlanarArm(LINK_LENGTHS)
        self.setWindowTitle('Kineshia | Planar arm control')
        self.resize(1280, 900)
        self.origin = time.monotonic()
        self.last_joint = self.last_status = 0.0
        self.status = None
        self.q = self.reference = None
        self.history = deque(maxlen=1500)
        self.pairs = {}
        self.previous_event = None
        self.gui_ticks = 0
        self.last_paint = time.monotonic()
        self.max_gui_gap = 0.0
        self.setStyleSheet("""
            QMainWindow, QWidget {background:#101923;color:#e3ecf4;font-size:13px;}
            QGroupBox {border:1px solid #304151;border-radius:6px;margin-top:14px;padding-top:12px;}
            QGroupBox::title {subcontrol-origin:margin;left:10px;color:#9baec1;}
            QPushButton {background:#26394b;border:1px solid #41586c;border-radius:5px;padding:9px;}
            QPushButton:hover {background:#35526a;}
            QPushButton:disabled {color:#596a7a;background:#1b2734;}
            QDoubleSpinBox,QComboBox,QPlainTextEdit {background:#172633;border:1px solid #364b5e;padding:5px;}
        """)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        root = QtWidgets.QVBoxLayout(central)
        title = QtWidgets.QLabel('KINESHIA  /  PLANAR ARM')
        title.setStyleSheet('font-size:24px;font-weight:600;')
        root.addWidget(title)
        self.banner = QtWidgets.QLabel('Connecting to the controller…')
        self.banner.setWordWrap(True)
        self.banner.setMinimumHeight(42)
        root.addWidget(self.banner)
        content = QtWidgets.QHBoxLayout()
        root.addLayout(content, 1)
        left = QtWidgets.QVBoxLayout()
        content.addLayout(left, 3)
        pg.setConfigOptions(antialias=True, background='#101923', foreground='#a9bfd0')
        self.arm_plot = pg.PlotWidget(title='Live arm · base frame · length units')
        self.arm_plot.setAspectLocked()
        self.arm_plot.setXRange(-7.8, 7.8)
        self.arm_plot.setYRange(-0.4, 7.5)
        self.arm_plot.showGrid(x=True, y=True, alpha=0.15)
        self.arm_plot.setLabel('bottom', 'x')
        self.arm_plot.setLabel('left', 'y')
        self.arm_plot.addItem(pg.InfiniteLine(pos=0, angle=0, pen=pg.mkPen('#71869a')))
        self.links = self.arm_plot.plot(pen=pg.mkPen('#58d9ed', width=5), symbol='o',
                                        symbolSize=13, symbolBrush='#e3ecf4')
        self.arm_plot.addLegend(offset=(10, 10))
        self.requested_marker = self.arm_plot.plot(pen=None, symbol='x', symbolSize=15,
                                                  symbolPen='#ff7979', name='Requested')
        self.resolved_marker = self.arm_plot.plot(pen=None, symbol='o', symbolSize=17,
                                                 symbolBrush=None, symbolPen='#ffbd69', name='Resolved')
        self.object_marker = self.arm_plot.plot(pen=None, symbol='s', symbolSize=15,
                                               symbolBrush='#8be0a4', name='Simulated object')
        left.addWidget(self.arm_plot, 4)
        self.telemetry = QtWidgets.QLabel('Waiting for joint feedback')
        self.telemetry.setWordWrap(True)
        self.telemetry.setMinimumHeight(52)
        left.addWidget(self.telemetry)
        self.angles = pg.PlotWidget(title='Joint angles · solid: feedback / dashed: command')
        self.angles.setLabel('left', 'Angle', units='deg')
        self.angles.setLabel('bottom', 'Time', units='s')
        self.angles.addLegend()
        self.angle_curves, self.reference_curves = [], []
        for i, color in enumerate(COLORS):
            self.angle_curves.append(self.angles.plot(pen=pg.mkPen(color, width=2), name=f'J{i+1}'))
            self.reference_curves.append(self.angles.plot(pen=pg.mkPen(color, style=QtCore.Qt.DashLine)))
        left.addWidget(self.angles, 3)
        self.error_plot = pg.PlotWidget(title='Tracking error · command minus feedback')
        self.error_plot.setLabel('left', 'Error', units='deg')
        self.error_plot.setLabel('bottom', 'Time', units='s')
        self.error_curves = [self.error_plot.plot(pen=pg.mkPen(color, width=2)) for color in COLORS]
        left.addWidget(self.error_plot, 2)

        panel = QtWidgets.QVBoxLayout()
        content.addLayout(panel, 1)
        mode_group = QtWidgets.QGroupBox('Control')
        mode_layout = QtWidgets.QVBoxLayout(mode_group)
        self.mode = QtWidgets.QComboBox()
        self.mode.addItems(['position', 'velocity_pid'])
        self.mode.activated[str].connect(self.change_mode)
        mode_layout.addWidget(self.mode)
        self.duration = self.spin(4.0, 0.2, 60.0)
        mode_layout.addWidget(QtWidgets.QLabel('Minimum trajectory duration (s)'))
        mode_layout.addWidget(self.duration)
        panel.addWidget(mode_group)

        target_group = QtWidgets.QGroupBox('Move to a target')
        form = QtWidgets.QFormLayout(target_group)
        self.target_x, self.target_y = self.spin(7), self.spin(3)
        form.addRow('x', self.target_x)
        form.addRow('y', self.target_y)
        self.move_button = QtWidgets.QPushButton('Move to target')
        self.move_button.clicked.connect(self.send_move)
        form.addRow(self.move_button)
        panel.addWidget(target_group)

        sequence_group = QtWidgets.QGroupBox('Pick and place')
        form = QtWidgets.QFormLayout(sequence_group)
        self.pick_x, self.pick_y = self.spin(4), self.spin(2)
        self.place_x, self.place_y = self.spin(-3), self.spin(3)
        for label, widget in [('Pick x', self.pick_x), ('Pick y', self.pick_y),
                               ('Place x', self.place_x), ('Place y', self.place_y)]:
            form.addRow(label, widget)
        self.sequence_button = QtWidgets.QPushButton('Run pick and place')
        self.sequence_button.setStyleSheet('background:#1b5963;')
        self.sequence_button.clicked.connect(self.send_sequence)
        form.addRow(self.sequence_button)
        panel.addWidget(sequence_group)
        self.cancel_button = QtWidgets.QPushButton('Cancel motion')
        self.cancel_button.clicked.connect(lambda: self.node.request('Cancel', self.node.cancel, Trigger.Request()))
        panel.addWidget(self.cancel_button)
        self.reset_button = QtWidgets.QPushButton('Reset simulation')
        self.reset_button.clicked.connect(lambda: self.node.request('Reset', self.node.reset, Trigger.Request()))
        panel.addWidget(self.reset_button)
        self.connection = QtWidgets.QLabel('Waiting for telemetry')
        self.connection.setWordWrap(True)
        panel.addWidget(self.connection)
        self.events = QtWidgets.QPlainTextEdit()
        self.events.setReadOnly(True)
        self.events.setMaximumBlockCount(150)
        panel.addWidget(self.events, 1)

        # Explicit queued connections make thread ownership visible in code.
        bridge.joints.connect(self.on_joints, QtCore.Qt.QueuedConnection)
        bridge.reference.connect(self.on_reference, QtCore.Qt.QueuedConnection)
        bridge.status.connect(self.on_status, QtCore.Qt.QueuedConnection)
        bridge.reply.connect(self.on_reply, QtCore.Qt.QueuedConnection)
        self.paint_timer = QtCore.QTimer(self)
        self.paint_timer.timeout.connect(self.refresh)
        self.paint_timer.start(50)
        self.refresh()

    @staticmethod
    def spin(value, low=-20.0, high=20.0):
        widget = QtWidgets.QDoubleSpinBox()
        widget.setRange(low, high)
        widget.setDecimals(2)
        widget.setValue(float(value))
        widget.setSingleStep(0.1)
        return widget

    @staticmethod
    def angles_from(message):
        try:
            result = np.array([message.position[message.name.index(name)] for name in JOINT_NAMES])
            return result if np.all(np.isfinite(result)) else None
        except (ValueError, IndexError):
            return None

    @QtCore.pyqtSlot(object)
    def on_joints(self, message):
        q = self.angles_from(message)
        if q is None:
            return
        self.q, self.last_joint = q, time.monotonic()
        self.join_sample(message, q, 0)

    @QtCore.pyqtSlot(object)
    def on_reference(self, message):
        self.reference = self.angles_from(message)
        if self.reference is not None:
            self.join_sample(message, self.reference, 1)

    def join_sample(self, message, q, index):
        # Match source timestamps, not callback arrival order, for honest error plots.
        key = (message.header.stamp.sec, message.header.stamp.nanosec)
        pair = self.pairs.setdefault(key, [None, None, time.monotonic()-self.origin])
        pair[index] = q.copy()
        if pair[0] is not None and pair[1] is not None:
            self.history.append((pair[2], pair[0], pair[1]))
            del self.pairs[key]
        while len(self.pairs) > 100:
            self.pairs.pop(next(iter(self.pairs)))

    @QtCore.pyqtSlot(object)
    def on_status(self, message):
        self.status, self.last_status = message, time.monotonic()
        event = (message.command_id, message.state)
        if event != self.previous_event:
            self.events.appendPlainText(f'{message.state}: {message.detail}')
            self.previous_event = event
        self.mode.blockSignals(True)
        self.mode.setCurrentText(message.control_mode)
        self.mode.blockSignals(False)

    @QtCore.pyqtSlot(str, bool, str)
    def on_reply(self, label, ok, text):
        self.events.appendPlainText(f'{label} / {"accepted" if ok else "rejected"}: {text}')

    def send_move(self):
        target = Point(x=self.target_x.value(), y=self.target_y.value())
        self.node.request('Move', self.node.move,
                          MoveTo.Request(target=target, duration=self.duration.value()))

    def send_sequence(self):
        self.node.request('Pick/place', self.node.sequence, PickPlace.Request(
            pick=Point(x=self.pick_x.value(), y=self.pick_y.value()),
            place=Point(x=self.place_x.value(), y=self.place_y.value()),
            duration=self.duration.value()))

    def change_mode(self, mode):
        self.node.request('Mode', self.node.mode, SetParameters.Request(
            parameters=[Parameter('control_mode', value=mode).to_parameter_msg()]))

    def refresh(self):
        now = time.monotonic()
        self.max_gui_gap = max(self.max_gui_gap, now-self.last_paint)
        self.last_paint = now
        self.gui_ticks += 1
        connected = now-self.last_joint < 1.0 and now-self.last_status < 1.0
        busy = bool(self.status and self.status.busy)
        for button in [self.move_button, self.sequence_button, self.reset_button, self.mode]:
            button.setEnabled(connected and not busy)
        self.cancel_button.setEnabled(connected and busy)
        if not connected:
            self.banner.setText('DISCONNECTED / STALE TELEMETRY — check the controller')
            self.connection.setText('No fresh state received within 1 second.')
        else:
            s = self.status
            self.banner.setText(f'{s.state}  ·  {s.detail}')
            self.banner.setStyleSheet('color:#ffbd69;' if s.projected else 'color:#e3ecf4;')
            self.connection.setText(f'Connected · {s.control_mode}\n'
                                    f'Timer jitter: mean {s.timer_jitter_mean_ms:.2f} ms'
                                    f' / max {s.timer_jitter_max_ms:.2f} ms')
            self.requested_marker.setData([s.requested_target.x], [s.requested_target.y])
            self.resolved_marker.setData([s.resolved_target.x], [s.resolved_target.y])
            self.object_marker.setData([s.object_position.x] if s.object_visible else [],
                                       [s.object_position.y] if s.object_visible else [])
        if self.q is not None:
            points = np.asarray(self.arm.forward_kinematics(self.q))
            self.links.setData(points[:, 0], points[:, 1])
            degrees = np.degrees(self.q)
            self.telemetry.setText(
                f'End effector  x = {points[-1,0]:.3f}   y = {points[-1,1]:.3f}\n'
                f'J1 {degrees[0]:7.2f}°     J2 {degrees[1]:7.2f}°     J3 {degrees[2]:7.2f}°'
                f'     Grasp: {"holding" if self.status and self.status.holding_object else "open"}')
        if self.history:
            times = np.array([item[0] for item in self.history])
            actual = np.degrees(np.array([item[1] for item in self.history]))
            reference = np.degrees(np.array([item[2] for item in self.history]))
            for i in range(3):
                self.angle_curves[i].setData(times, actual[:, i])
                self.reference_curves[i].setData(times, reference[:, i])
                self.error_curves[i].setData(times, reference[:, i]-actual[:, i])
            for plot in [self.angles, self.error_plot]:
                plot.setXRange(max(0, times[-1]-30), max(30, times[-1]), padding=0)


def main(args=None):
    cli = remove_ros_args(args=sys.argv if args is None else args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--demo', action='store_true')
    parser.add_argument('--record')
    options = parser.parse_args(cli[1:])
    rclpy.init(args=args)
    app = QtWidgets.QApplication(sys.argv[:1])
    bridge = Bridge()
    node = GuiNode(bridge)
    window = ArmWindow(node, bridge)
    recorder = None
    if options.record:
        window.setFixedSize(1280, 900)
        recorder = WindowRecorder(options.record, 1280, 900)
        record_timer = QtCore.QTimer(window)
        record_timer.timeout.connect(lambda: recorder.capture(window))
        record_timer.start(100)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    worker = threading.Thread(target=spin, name='ros-executor')
    worker.start()
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    director = None
    if options.demo:
        from .demo import DemoDirector
        director = DemoDirector(window, app)
    window.show()
    try:
        code = app.exec_()
    finally:
        executor.shutdown()
        worker.join(timeout=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if recorder:
            record_timer.stop()
            recorder.close()
        print(f'GUI ticks: {window.gui_ticks}; maximum paint gap: {window.max_gui_gap:.3f}s', flush=True)
    return code


if __name__ == '__main__':
    sys.exit(main())
