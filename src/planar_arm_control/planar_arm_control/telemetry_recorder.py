"""Independent ROS observer using the attributed upstream logging helpers.

No command publishers or services: logging cannot control the arm. Disk I/O
runs in this separate process, outside both the control timer and Qt thread.
"""

from datetime import datetime, timezone
from pathlib import Path
import time
import uuid

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from planar_arm_interfaces.msg import ControllerStatus

from .vendor.telemetry_csv import CsvLog, JsonlLog


class TelemetryRecorder(Node):
    def __init__(self):
        super().__init__('telemetry_recorder')
        root = self.declare_parameter('output_directory', 'artifacts/telemetry').value
        session = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S_%fZ') + '_' + uuid.uuid4().hex[:8]
        self.directory = Path(root).expanduser() / session
        # CsvLog uses mode 'w'; a newly created session prevents overwriting runs.
        self.directory.mkdir(parents=True, exist_ok=False)
        header = ['receive_monotonic_ns', 'source_sec', 'source_nanosec',
                  'joint', 'position_rad', 'velocity_rad_s', 'effort']
        self.actual = CsvLog(self.directory / 'joint_states.csv', header)
        self.command = CsvLog(self.directory / 'joint_commands.csv', header)
        self.events = JsonlLog(self.directory / 'events.jsonl')
        self.previous_event = None
        self.events.write({'event': 'session_start', 'utc': session, 'schema_version': 1})
        self.create_subscription(JointState, 'joint_states',
                                 lambda msg: self.joints(msg, self.actual), 100)
        self.create_subscription(JointState, 'joint_commands',
                                 lambda msg: self.joints(msg, self.command), 100)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(ControllerStatus, 'controller_status', self.status, qos)
        self.create_timer(1.0, self.flush)
        self.get_logger().info(f'Observing telemetry into {self.directory}')

    @staticmethod
    def joints(message, log):
        received = time.monotonic_ns()
        stamp = message.header.stamp
        for index, name in enumerate(message.name):
            fields = [values[index] if index < len(values) else ''
                      for values in (message.position, message.velocity, message.effort)]
            log.write([received, stamp.sec, stamp.nanosec, name, *fields])

    def status(self, message):
        event = (message.command_id, message.state, message.control_mode, message.detail)
        if event == self.previous_event:
            return
        self.previous_event = event
        self.events.write({
            'event': 'controller_transition', 'receive_monotonic_ns': time.monotonic_ns(),
            'source_sec': message.header.stamp.sec,
            'source_nanosec': message.header.stamp.nanosec,
            'command_id': message.command_id, 'state': message.state,
            'detail': message.detail, 'mode': message.control_mode,
            'backend': message.backend, 'holding_object': message.holding_object,
            'projected': message.projected,
            'object_xy': [message.object_position.x, message.object_position.y],
        })

    def flush(self):
        for log in (self.actual, self.command, self.events):
            log.flush()

    def close(self):
        self.events.write({'event': 'session_end', 'actual_rows': self.actual.count,
                           'command_rows': self.command.count})
        for log in (self.actual, self.command, self.events):
            log.close()


def main(args=None):
    rclpy.init(args=args)
    node = TelemetryRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
