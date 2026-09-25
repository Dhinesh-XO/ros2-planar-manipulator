"""Physical Gazebo adapter; planning remains in assignment coordinates.

One assignment length unit = 0.1 m. Math (x,y) maps to world
(0.1*x, 0, 0.08+0.1*y). Joint angles require no scaling.
Grasp is an acknowledged detachable constraint, not friction simulation.
"""

import math
import os
import subprocess
import threading
import time

import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Float64, String
from tf2_msgs.msg import TFMessage

from .backends import Backend
from .planning import INITIAL_Q, JOINT_NAMES, PlanningError, check_configuration


class GazeboBackend(Backend):
    name = 'gazebo'
    position_tolerance = 0.004

    def __init__(self, node, arm):
        self.node, self.arm = node, arm
        self.lock = threading.RLock()
        self.q, self.velocity = INITIAL_Q.copy(), np.zeros(3)
        self.last_joint = self.last_clock = self.last_object = 0.0
        self.clock = 0.0
        self.object_pose = None
        self.grasp_state = ''
        self.init_started = time.monotonic()
        self.init_probed = False
        self.initialized = False
        self.pose_process = None
        self.pose_set_at = None
        self.group = MutuallyExclusiveCallbackGroup()
        self.commands = [node.create_publisher(Float64, f'/arm/{joint}/command', 10)
                         for joint in JOINT_NAMES]
        self.gripper = node.create_publisher(Float64, '/arm/gripper/command', 10)
        self.attach = node.create_publisher(Empty, '/arm/grasp/attach', 10)
        self.detach = node.create_publisher(Empty, '/arm/grasp/detach', 10)
        node.create_subscription(JointState, '/gazebo/joint_states', self.on_joints, 10,
                                 callback_group=self.group)
        node.create_subscription(TFMessage, '/gazebo/object_pose', self.on_object, 10,
                                 callback_group=self.group)
        node.create_subscription(String, '/gazebo/grasp_state', self.on_grasp, 10,
                                 callback_group=self.group)

    def on_joints(self, message):
        try:
            indices = [message.name.index(name) for name in JOINT_NAMES]
            q = np.array([message.position[i] for i in indices])
            dq = np.array([message.velocity[i] for i in indices])
            if not (np.all(np.isfinite(q)) and np.all(np.isfinite(dq))):
                return
        except (ValueError, IndexError):
            return
        with self.lock:
            self.q, self.velocity, self.last_joint = q, dq, time.monotonic()
            # Gazebo stamps this measured message with simulation time. Use
            # its 100 Hz clock sample, not a separate 1 kHz Python callback.
            clock = message.header.stamp.sec + message.header.stamp.nanosec*1e-9
            if clock > self.clock:
                self.last_clock = time.monotonic()
            self.clock = clock

    def on_object(self, message):
        for transform in message.transforms:
            if transform.child_frame_id.split('::')[-1] != 'payload':
                continue
            p, r = transform.transform.translation, transform.transform.rotation
            # Rotation around -world Y is positive in assignment coordinates.
            angle = -math.atan2(2*(r.w*r.y-r.z*r.x), 1-2*(r.y*r.y+r.x*r.x))
            with self.lock:
                self.object_pose = (np.array([p.x/0.1, (p.z-0.08)/0.1]), angle)
                self.last_object = time.monotonic()

    def on_grasp(self, message):
        with self.lock:
            self.grasp_state = message.data

    def ready(self):
        with self.lock:
            now = time.monotonic()
            fresh = all(now-stamp < 0.6 for stamp in
                        (self.last_joint, self.last_clock, self.last_object))
            if not fresh:
                return False
            if self.initialized:
                return True
            # Gazebo's DetachableJoint initially attaches its child. Explicitly
            # detach, then restore the cube before exposing a ready controller.
            self.set_gripper(1.0)
            if self.grasp_state != 'detached':
                if not self.grasp_state and now-self.init_started > 1.0 and not self.init_probed:
                    self.attach.publish(Empty())
                    self.init_probed = True
                    return False
                self.detach.publish(Empty())
                return False
            if np.max(np.abs(self.q-INITIAL_Q)) > 0.025:
                self.publish_positions(INITIAL_Q)
                return False
            if self.pose_process is None:
                if now-self.init_started < 3.0:
                    return False
                # The Humble/Harmonic binary bridge lacks SetEntityPose even
                # though newer upstream source supports it. One nonblocking
                # native Gazebo request initializes the fixture; ALL motion
                # commands and feedback still use the ROS/Gazebo bridge.
                env = dict(os.environ)
                env.setdefault('GZ_PARTITION', 'kineshia_'+env.get('ROS_DOMAIN_ID', '67'))
                self.pose_process = subprocess.Popen([
                    'gz', 'service', '-s', '/world/workcell/set_pose',
                    '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '2000', '--req',
                    'name: "payload", position: {x: 0.4, y: 0, z: 0.28}, orientation: {w: 1}'],
                    env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                return False
            if self.pose_process.poll() is None:
                return False
            if self.pose_process.returncode != 0:
                return False
            if self.pose_set_at is None:
                self.pose_set_at = now
            if now-self.pose_set_at < 0.5:
                return False
            self.initialized = bool(np.linalg.norm(self.object_pose[0]-[4., 2.]) < 0.05)
            return self.initialized

    def read_state(self):
        with self.lock:
            return self.q.copy(), self.velocity.copy()

    def motion_time(self):
        with self.lock:
            return self.clock

    def object_state(self):
        with self.lock:
            if self.object_pose is None:
                return None
            return self.object_pose[0].copy(), self.object_pose[1]

    def publish_positions(self, q):
        for publisher, angle in zip(self.commands, q):
            publisher.publish(Float64(data=float(angle)))

    def step(self, q_ref, dq_ref, dt, mode):
        del dq_ref, dt
        if mode != 'position':
            raise PlanningError('Gazebo backend uses physical position servos.')
        q, velocity = self.read_state()
        check_configuration(self.arm, q)
        self.publish_positions(q_ref)
        return q, velocity

    def stop(self):
        self.publish_positions(self.read_state()[0])

    def hold_reference(self, q_ref):
        # Successful completion keeps the settled target. Replacing it with
        # the instantaneous measurement introduces a derivative kick and
        # destroys the gravity-compensating steady-state servo error.
        self.publish_positions(q_ref)

    def set_gripper(self, opening):
        self.gripper.publish(Float64(data=0.013*float(np.clip(opening, 0, 1))))

    def grasp(self, target):
        with self.lock:
            if self.grasp_state == 'attached':
                return True
            if self.object_pose is None or np.linalg.norm(self.object_pose[0]-target) > 0.15:
                return False
            tip = np.asarray(self.arm.end_effector(self.q))
            if np.linalg.norm(tip-self.object_pose[0]) > 0.15:
                return False
            self.attach.publish(Empty())
            return False

    def release(self):
        with self.lock:
            if self.grasp_state == 'detached':
                return True
            self.detach.publish(Empty())
            return False

    def is_holding(self):
        with self.lock:
            return self.grasp_state == 'attached'
