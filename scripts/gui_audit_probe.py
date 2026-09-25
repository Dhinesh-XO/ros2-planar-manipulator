#!/usr/bin/env python3
"""Instrument the real GUI for the audit; never sends a command itself."""

import json
import os
from pathlib import Path
import signal
import sys
import threading
import time

from PyQt5 import QtCore, QtWidgets
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from planar_arm_control.gui_node import ArmWindow, Bridge, GuiNode


def main():
    destination = Path(sys.argv[1])
    rclpy.init(args=[])
    app = QtWidgets.QApplication(sys.argv[:1])
    bridge = Bridge()
    node = GuiNode(bridge)
    window = ArmWindow(node, bridge)
    window.setWindowTitle('Isolated submission audit — not the live workcell')
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin():
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    worker = threading.Thread(target=spin)
    worker.start()

    def snapshot():
        status = window.status
        data = {'pid': os.getpid(), 'wall_time': time.time(),
                'stale': 'STALE' in window.banner.text(),
                'move_enabled': window.move_button.isEnabled(),
                'commands_enabled': [widget.isEnabled() for widget in
                                     (window.move_button, window.sequence_button,
                                      window.cancel_button, window.reset_button, window.mode)],
                'command_id': status.command_id if status else None,
                'state': status.state if status else None,
                'busy': status.busy if status else None,
                'gui_ticks': window.gui_ticks, 'max_gui_gap': window.max_gui_gap}
        temporary = destination.with_suffix('.pending')
        temporary.write_text(json.dumps(data))
        temporary.replace(destination)

    timer = QtCore.QTimer()
    timer.timeout.connect(snapshot)
    timer.start(100)
    # A bounded probe cannot be left running after a failed parent audit.
    QtCore.QTimer.singleShot(180000, app.quit)
    signal.signal(signal.SIGINT, lambda *_: app.quit())
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    window.show()
    try:
        app.exec_()
    finally:
        snapshot()
        executor.shutdown()
        worker.join(timeout=5)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        window.close()


if __name__ == '__main__':
    main()
