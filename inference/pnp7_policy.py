#!/usr/bin/env python3
"""Recorded replay, live shadow, and guarded live execution for PNP-7 PI0.5.

The live mode is intentionally hold-to-run: the foot switch configured as
``/dev/pnp7_deadman`` must be released at startup and then held continuously.
The first release ends the run and immediately replaces the policy target with
the measured Franka pose.
"""

from __future__ import annotations

import argparse
import csv
import fcntl
import json
import math
import os
import socket
import struct
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

from protocol import (
    decode_json,
    encode_inference_request,
    encode_json,
    recv_frame,
    send_frame,
)


PROMPT = "pick up the blue cube and place it in the plate"
EXTERNAL_SERIAL = "213622078826"
WRIST_SERIAL = "233622071437"
IMAGE_SIZE = 224

# Per-axis 1st/99th percentiles from the exact 30-episode normalization stats.
ACTION_LOWER = np.array(
    [-0.00472018, -0.00705170, -0.00345769, -0.00528421, -0.01004742, -0.01390426, 0.0],
    dtype=np.float64,
)
ACTION_UPPER = np.array(
    [0.00471258, 0.00727484, 0.00460831, 0.00592704, 0.00949706, 0.01481510, 1.0],
    dtype=np.float64,
)

# Training position q01/q99 expanded by roughly 2 cm.  This is tighter than
# the robot's mechanical reach and prevents the policy from leaving the region
# demonstrated by the operator.
WORKSPACE_LOWER = np.array([0.255, -0.218, 0.000], dtype=np.float64)
WORKSPACE_UPPER = np.array([0.763, 0.306, 0.263], dtype=np.float64)
MAX_SESSION_TRANSLATION = 0.25
MAX_SESSION_ROTATION = math.radians(50.0)
MAX_STATE_AGE_S = 0.100
ACTION_HZ = 30.0


class SafetyError(RuntimeError):
    pass


def multiply3(left, right):
    return np.asarray(left, dtype=np.float64).reshape(3, 3).dot(
        np.asarray(right, dtype=np.float64).reshape(3, 3)
    )


def rpy_to_rotation(rpy):
    roll, pitch, yaw = [float(v) for v in rpy]
    cx, sx = math.cos(roll), math.sin(roll)
    cy, sy = math.cos(pitch), math.sin(pitch)
    cz, sz = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]], dtype=np.float64)
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]], dtype=np.float64)
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]], dtype=np.float64)
    return rz.dot(ry).dot(rx)


def rotation_to_rpy(rotation):
    r = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    sy = math.sqrt(float(r[0, 0] ** 2 + r[1, 0] ** 2))
    if sy > 1e-8:
        return np.array(
            [
                math.atan2(r[2, 1], r[2, 2]),
                math.atan2(-r[2, 0], sy),
                math.atan2(r[1, 0], r[0, 0]),
            ],
            dtype=np.float64,
        )
    return np.array(
        [math.atan2(-r[1, 2], r[1, 1]), math.atan2(-r[2, 0], sy), 0.0],
        dtype=np.float64,
    )


def rotation_angle(rotation):
    value = (float(np.trace(rotation)) - 1.0) * 0.5
    return math.acos(max(-1.0, min(1.0, value)))


def matrix_to_quaternion(rotation):
    r = np.asarray(rotation, dtype=np.float64).reshape(3, 3)
    trace = float(np.trace(r))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        return np.array(
            [(r[2, 1] - r[1, 2]) / scale, (r[0, 2] - r[2, 0]) / scale,
             (r[1, 0] - r[0, 1]) / scale, 0.25 * scale]
        )
    if r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        scale = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        return np.array(
            [0.25 * scale, (r[0, 1] + r[1, 0]) / scale,
             (r[0, 2] + r[2, 0]) / scale, (r[2, 1] - r[1, 2]) / scale]
        )
    if r[1, 1] > r[2, 2]:
        scale = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        return np.array(
            [(r[0, 1] + r[1, 0]) / scale, 0.25 * scale,
             (r[1, 2] + r[2, 1]) / scale, (r[0, 2] - r[2, 0]) / scale]
        )
    scale = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
    return np.array(
        [(r[0, 2] + r[2, 0]) / scale, (r[1, 2] + r[2, 1]) / scale,
         0.25 * scale, (r[1, 0] - r[0, 1]) / scale]
    )


def pose_from_column_major(values):
    matrix = np.asarray(values, dtype=np.float64).reshape(4, 4).T
    return matrix[:3, 3].copy(), matrix[:3, :3].copy()


def make_state(position, rotation, gripper_open):
    rot6d = np.asarray(rotation, dtype=np.float64)[:, :2].T.reshape(-1)
    state = np.concatenate([position, rot6d, [float(gripper_open)]]).astype(np.float32)
    if state.shape != (10,) or not np.isfinite(state).all():
        raise SafetyError("invalid 10D policy state")
    return state


def resize_rgb(bgr):
    resized = cv2.resize(bgr, (IMAGE_SIZE, IMAGE_SIZE), interpolation=cv2.INTER_AREA)
    return cv2.cvtColor(resized, cv2.COLOR_BGR2RGB)


class PolicyConnection:
    def __init__(self, host, port, token, timeout):
        self.sock = socket.create_connection((host, port), timeout=timeout)
        self.sock.settimeout(timeout)
        self.token = token
        self.request_id = 0

    def close(self):
        try:
            self.sock.close()
        except Exception:
            pass

    def health(self):
        send_frame(self.sock, encode_json({"kind": "health"}))
        response = decode_json(recv_frame(self.sock))
        if not response.get("ok"):
            raise RuntimeError("policy server health failed: {}".format(response))
        return response

    def infer(self, state, base_rgb, wrist_rgb, prompt, seed=None):
        """`seed` is honored only by per-request-seed servers; others ignore it."""
        request_id = self.request_id
        self.request_id += 1
        metadata = {
            "token": self.token,
            "request_id": request_id,
            "state": np.asarray(state, dtype=np.float32).tolist(),
            "prompt": prompt,
        }
        if seed is not None:
            metadata["seed"] = int(seed)
        payload = encode_inference_request(
            metadata,
            np.ascontiguousarray(base_rgb, dtype=np.uint8),
            np.ascontiguousarray(wrist_rgb, dtype=np.uint8),
        )
        send_frame(self.sock, payload)
        response = decode_json(recv_frame(self.sock))
        if not response.get("ok"):
            raise RuntimeError("policy inference failed: {}".format(response.get("error")))
        if response.get("request_id") != request_id:
            raise RuntimeError("policy response request ID mismatch")
        actions = np.asarray(response["actions"], dtype=np.float64)
        if actions.shape != (10, 7) or not np.isfinite(actions).all():
            raise RuntimeError("invalid action tensor from server: {}".format(actions.shape))
        return actions, float(response["inference_ms"])


def row_pose(row):
    return pose_from_column_major([float(row["O_T_EE{}".format(i)]) for i in range(16)])


def run_recorded(args):
    episode = Path(args.recorded_episode).expanduser().resolve()
    rows = list(csv.DictReader((episode / "episode.csv").open()))
    if len(rows) < 4:
        raise RuntimeError("recorded episode has too few rows")
    indices = np.linspace(0, len(rows) - 2, args.samples + 2, dtype=int)[1:-1]
    conn = PolicyConnection(args.server, args.port, args.token, args.timeout)
    report = {"mode": "recorded", "episode": str(episode), "health": conn.health(), "samples": []}
    try:
        for index in indices:
            row, next_row = rows[int(index)], rows[int(index) + 1]
            position, rotation = row_pose(row)
            next_position, next_rotation = row_pose(next_row)
            gripper_open = 1.0 if float(row["gripper_command"]) > 0.04 else 0.0
            next_gripper = 1.0 if float(next_row["gripper_command"]) > 0.04 else 0.0
            state = make_state(position, rotation, gripper_open)
            base = cv2.imread(str(episode / row["rgb_external"]))
            wrist = cv2.imread(str(episode / row["rgb_wrist"]))
            if base is None or wrist is None:
                raise FileNotFoundError("recorded camera image is missing at row {}".format(index))
            actions, elapsed_ms = conn.infer(
                state, resize_rgb(base), resize_rgb(wrist), args.prompt
            )
            truth = np.concatenate(
                [next_position - position, rotation_to_rpy(rotation.T.dot(next_rotation)), [next_gripper]]
            )
            clipped = np.clip(actions, ACTION_LOWER, ACTION_UPPER)
            clipped_values = int(np.count_nonzero(np.abs(clipped - actions) > 1e-12))
            sample = {
                "row": int(index),
                "inference_ms": elapsed_ms,
                "state_position": position.tolist(),
                "ground_truth_action": truth.tolist(),
                "predicted_first_action": actions[0].tolist(),
                "predicted_min": actions.min(axis=0).tolist(),
                "predicted_max": actions.max(axis=0).tolist(),
                "clipped_values": clipped_values,
            }
            report["samples"].append(sample)
            print(
                "REPLAY row={} inference_ms={:.1f} first={} clipped_values={}".format(
                    index, elapsed_ms, np.round(actions[0], 6).tolist(), clipped_values
                ),
                flush=True,
            )
    finally:
        conn.close()
    output = Path(args.log).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    print("RECORDED_REPLAY_PASS samples={} log={}".format(len(report["samples"]), output), flush=True)


class CameraStream:
    def __init__(self, serial, role):
        self.serial = serial
        self.role = role
        self.lock = threading.Lock()
        self.frame = None
        self.frame_ns = 0
        self.error = None
        self.stop_event = threading.Event()
        self.thread = None
        self.pipeline = None
        self.config = None
        self.pipeline_started = False

    def start(self):
        import pyrealsense2 as rs

        self.rs = rs
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.serial)
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config = config
        # RealSense pipeline.start() occasionally blocks for tens of seconds
        # while UVC negotiates a stream.  Starting each device on its own
        # thread matches collect/record_cameras.py, prevents one camera from
        # delaying the other, and lets the live preflight's finite freshness
        # timeout reject the run without ever arming robot commands.
        self.thread = threading.Thread(target=self._run, name="camera-{}".format(self.role), daemon=True)
        self.thread.start()

    def _run(self):
        try:
            self.pipeline.start(self.config)
            self.pipeline_started = True
            while not self.stop_event.is_set():
                ok, frames = self.pipeline.try_wait_for_frames(1000)
                if not ok:
                    continue
                frame = frames.get_color_frame()
                if not frame:
                    continue
                image = np.asanyarray(frame.get_data()).copy()
                with self.lock:
                    self.frame = image
                    self.frame_ns = time.monotonic_ns()
        except Exception as exc:
            self.error = exc
        finally:
            if self.pipeline_started:
                try:
                    self.pipeline.stop()
                except Exception:
                    pass
                self.pipeline_started = False

    def latest(self, max_age_s=0.2):
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
            frame_ns = self.frame_ns
        if self.error is not None:
            raise RuntimeError("{} camera failed: {}".format(self.role, self.error))
        if frame is None:
            raise RuntimeError("{} camera has not produced a frame".format(self.role))
        age = (time.monotonic_ns() - frame_ns) / 1e9
        if age > max_age_s:
            raise RuntimeError("{} camera frame is stale ({:.0f} ms)".format(self.role, age * 1000))
        return resize_rgb(frame), age

    def stop(self):
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=5.0)


def message_has_errors(errors):
    return any(bool(getattr(errors, name)) for name in getattr(errors, "__slots__", ()))


class RobotStateCache:
    def __init__(self):
        self.lock = threading.Lock()
        self.message = None
        self.received_ns = 0
        self.gripper_width = None
        self.gripper_received_ns = 0

    def state_callback(self, message):
        with self.lock:
            self.message = message
            self.received_ns = time.monotonic_ns()

    def gripper_callback(self, message):
        if len(message.position) >= 2:
            with self.lock:
                self.gripper_width = float(message.position[0] + message.position[1])
                self.gripper_received_ns = time.monotonic_ns()

    def snapshot(self, require_fresh=True):
        with self.lock:
            message = self.message
            received_ns = self.received_ns
            width = self.gripper_width
            gripper_ns = self.gripper_received_ns
        if message is None:
            raise SafetyError("no Franka state received")
        age = (time.monotonic_ns() - received_ns) / 1e9
        if require_fresh and age > MAX_STATE_AGE_S:
            raise SafetyError("Franka state is stale ({:.1f} ms)".format(age * 1000))
        if message_has_errors(message.current_errors):
            raise SafetyError("Franka reports current_errors")
        if message_has_errors(message.last_motion_errors):
            raise SafetyError("Franka reports last_motion_errors")
        position, rotation = pose_from_column_major(message.O_T_EE)
        grip_age = None if not gripper_ns else (time.monotonic_ns() - gripper_ns) / 1e9
        return message, position, rotation, width, age, grip_age


EVENT = struct.Struct("llHHi")
KEY_MAX = 0x2FF
KEY_BYTES = (KEY_MAX + 8) // 8
KEY_F3 = 61


def _ioc(direction, type_value, number, size):
    return (direction << 30) | (type_value << 8) | number | (size << 16)


EVIOCGKEY = _ioc(2, ord("E"), 0x18, KEY_BYTES)
EVIOCGRAB = _ioc(1, ord("E"), 0x90, struct.calcsize("i"))


class DeadmanMonitor:
    def __init__(self, device, on_release):
        self.device = device
        self.on_release = on_release
        self.fd = -1
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.enabled_value = False
        self.armed = False
        self.ever_pressed = False
        self.release_latched = False
        self.fault = None
        self.thread = None

    def _pressed(self):
        bits = bytearray(KEY_BYTES)
        fcntl.ioctl(self.fd, EVIOCGKEY, bits, True)
        return bool(bits[KEY_F3 // 8] & (1 << (KEY_F3 % 8)))

    def start(self):
        self.fd = os.open(self.device, os.O_RDONLY | os.O_NONBLOCK | os.O_CLOEXEC)
        fcntl.ioctl(self.fd, EVIOCGRAB, 1)
        if self._pressed():
            self.close()
            raise SafetyError("dead-man must be released before startup")
        self.thread = threading.Thread(target=self._run, name="deadman-monitor", daemon=True)
        self.thread.start()

    def arm(self):
        if self._pressed():
            raise SafetyError("dead-man must still be released when controls arm")
        with self.lock:
            self.armed = True

    def _run(self):
        previous = False
        while not self.stop_event.wait(0.005):
            try:
                pressed = self._pressed()
                try:
                    while os.read(self.fd, EVENT.size * 64):
                        pass
                except BlockingIOError:
                    pass
            except Exception as exc:
                with self.lock:
                    self.fault = exc
                    self.enabled_value = False
                    self.release_latched = True
                self.on_release("deadman_fault")
                return
            callback = None
            with self.lock:
                if pressed and not previous and self.armed and not self.release_latched:
                    self.enabled_value = True
                    self.ever_pressed = True
                    print("OPERATOR_EVENT deadman=pressed", flush=True)
                elif not pressed and previous:
                    self.enabled_value = False
                    if self.ever_pressed:
                        self.release_latched = True
                        callback = "deadman_released"
                        print("OPERATOR_EVENT deadman=released", flush=True)
            previous = pressed
            if callback:
                self.on_release(callback)

    def enabled(self):
        with self.lock:
            return self.enabled_value and not self.release_latched and self.fault is None

    def stopped(self):
        with self.lock:
            return self.release_latched or self.fault is not None

    def wait_for_press(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self.enabled():
                return True
            if self.stopped():
                return False
            time.sleep(0.02)
        return False

    def close(self):
        self.stop_event.set()
        if self.thread is not None and threading.current_thread() is not self.thread:
            self.thread.join(timeout=1.0)
        if self.fd >= 0:
            try:
                fcntl.ioctl(self.fd, EVIOCGRAB, 0)
            except Exception:
                pass
            os.close(self.fd)
            self.fd = -1


class GripperController:
    def __init__(self, initial_open):
        import actionlib
        from franka_gripper.msg import GraspAction, MoveAction, StopAction

        self.actionlib = actionlib
        self.move = actionlib.SimpleActionClient("/franka_gripper/move", MoveAction)
        self.grasp = actionlib.SimpleActionClient("/franka_gripper/grasp", GraspAction)
        self.stop_client = actionlib.SimpleActionClient("/franka_gripper/stop", StopAction)
        self.command_open = bool(initial_open)
        self.candidate = None
        self.candidate_count = 0
        self.commands = 0

    def wait_for_servers(self, rospy, timeout_s=8.0):
        timeout = rospy.Duration(timeout_s)
        if not self.move.wait_for_server(timeout):
            raise SafetyError("Franka gripper move action is unavailable")
        if not self.grasp.wait_for_server(timeout):
            raise SafetyError("Franka gripper grasp action is unavailable")
        if not self.stop_client.wait_for_server(timeout):
            raise SafetyError("Franka gripper stop action is unavailable")

    def update(self, value):
        from franka_gripper.msg import GraspEpsilon, GraspGoal, MoveGoal

        value = float(value)
        desired = None
        if value >= 0.75:
            desired = True
        elif value <= 0.25:
            desired = False
        if desired is None or desired == self.command_open:
            self.candidate = None
            self.candidate_count = 0
            return
        if desired != self.candidate:
            self.candidate = desired
            self.candidate_count = 1
            return
        self.candidate_count += 1
        if self.candidate_count < 2:
            return
        if desired:
            self.grasp.cancel_all_goals()
            self.move.send_goal(MoveGoal(width=0.08, speed=0.10))
        else:
            self.move.cancel_all_goals()
            self.grasp.send_goal(
                GraspGoal(
                    width=0.0,
                    epsilon=GraspEpsilon(inner=0.005, outer=0.005),
                    speed=0.10,
                    force=20.0,
                )
            )
        self.command_open = desired
        self.commands += 1
        self.candidate = None
        self.candidate_count = 0
        print("GRIPPER_COMMAND open={} raw={:.3f}".format(int(desired), value), flush=True)

    def emergency_stop(self):
        from franka_gripper.msg import StopGoal

        try:
            self.move.cancel_all_goals()
            self.grasp.cancel_all_goals()
            self.stop_client.send_goal(StopGoal())
        except Exception:
            pass


class LiveController:
    def __init__(self, rospy, cache, publisher):
        self.rospy = rospy
        self.cache = cache
        self.publisher = publisher
        self.command_lock = threading.Lock()
        self.gripper = None
        self.deadman = None
        self.stop_reason = None

    def _pose_message(self, position, rotation):
        from geometry_msgs.msg import PoseStamped

        quaternion = matrix_to_quaternion(rotation)
        msg = PoseStamped()
        msg.header.stamp = self.rospy.Time.now()
        msg.header.frame_id = "panda_link0"
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = quaternion
        return msg

    def publish_policy_target(self, position, rotation, gripper_value):
        with self.command_lock:
            if self.deadman is None or not self.deadman.enabled():
                return False
            self.publisher.publish(self._pose_message(position, rotation))
            if self.gripper is not None:
                self.gripper.update(gripper_value)
            return True

    def emergency_hold(self, reason):
        with self.command_lock:
            self.stop_reason = reason
            try:
                _, position, rotation, _, _, _ = self.cache.snapshot(require_fresh=False)
                for _ in range(3):
                    self.publisher.publish(self._pose_message(position, rotation))
            except Exception:
                pass
            if self.gripper is not None:
                self.gripper.emergency_stop()


def validate_live_state(cache):
    message, position, rotation, width, state_age, grip_age = cache.snapshot()
    if np.any(position < WORKSPACE_LOWER) or np.any(position > WORKSPACE_UPPER):
        raise SafetyError(
            "current EE position {} is outside trained workspace {}..{}".format(
                np.round(position, 4).tolist(), WORKSPACE_LOWER.tolist(), WORKSPACE_UPPER.tolist()
            )
        )
    return message, position, rotation, width, state_age, grip_age


def sanitize_action(raw):
    raw = np.asarray(raw, dtype=np.float64)
    motion_scale = np.maximum(np.abs(ACTION_LOWER[:6]), np.abs(ACTION_UPPER[:6]))
    if np.any(np.abs(raw[:6]) > motion_scale * 4.0):
        raise SafetyError("policy emitted a >4x training-percentile motion outlier: {}".format(raw.tolist()))
    clipped = np.clip(raw, ACTION_LOWER, ACTION_UPPER)
    return clipped, int(np.count_nonzero(np.abs(clipped - raw) > 1e-12))


def run_live(args):
    import rospy
    from franka_msgs.msg import FrankaState
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import JointState

    rospy.init_node("pnp7_pi05_policy", anonymous=False, disable_signals=True)
    cache = RobotStateCache()
    rospy.Subscriber("/franka_state_controller/franka_states", FrankaState, cache.state_callback, queue_size=1)
    rospy.Subscriber("/franka_gripper/joint_states", JointState, cache.gripper_callback, queue_size=1)
    publisher = rospy.Publisher(
        "/cartesian_impedance_controller/equilibrium_pose", PoseStamped, queue_size=1, tcp_nodelay=True
    )
    controller = LiveController(rospy, cache, publisher)
    external = CameraStream(EXTERNAL_SERIAL, "external")
    wrist = CameraStream(WRIST_SERIAL, "wrist")
    conn = None
    deadman = None
    log_rows = []
    initial_position = None
    initial_rotation = None
    first_base = None
    first_wrist = None
    clip_count = 0
    executed = 0
    inference_times = []
    started = None
    result = "not_started"
    try:
        external.start()
        wrist.start()
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            try:
                validate_live_state(cache)
                first_base, _ = external.latest()
                first_wrist, _ = wrist.latest()
                break
            except Exception:
                time.sleep(0.05)
        else:
            raise SafetyError("timed out waiting for fresh robot/camera state")

        if publisher.get_num_connections() < 1:
            raise SafetyError("SERL equilibrium-pose controller is not subscribed")
        _, position, rotation, width, state_age, grip_age = validate_live_state(cache)
        gripper_open = 1.0 if width is not None and width > 0.04 else 0.0
        state = make_state(position, rotation, gripper_open)

        conn = PolicyConnection(args.server, args.port, args.token, args.timeout)
        health = conn.health()
        print("POLICY_SERVER_READY {}".format(json.dumps(health, sort_keys=True)), flush=True)
        actions, inference_ms = conn.infer(state, first_base, first_wrist, args.prompt)
        inference_times.append(inference_ms)
        for action in actions:
            _, clipped = sanitize_action(action)
            clip_count += clipped
        print(
            "LIVE_PREFLIGHT_PASS position={} gripper_open={} inference_ms={:.1f} action_min={} action_max={}".format(
                np.round(position, 4).tolist(), int(gripper_open), inference_ms,
                np.round(actions.min(axis=0), 6).tolist(), np.round(actions.max(axis=0), 6).tolist(),
            ),
            flush=True,
        )

        if args.mode == "shadow":
            result = "shadow_pass"
            shadow_started = time.monotonic()
            request_count = 1
            while time.monotonic() - shadow_started < args.duration:
                _, position, rotation, width, state_age, grip_age = validate_live_state(cache)
                base, base_age = external.latest()
                wrist_rgb, wrist_age = wrist.latest()
                gripper_open = 1.0 if width is not None and width > 0.04 else gripper_open
                state = make_state(position, rotation, gripper_open)
                actions, inference_ms = conn.infer(state, base, wrist_rgb, args.prompt)
                inference_times.append(inference_ms)
                local_clips = 0
                for action in actions:
                    _, count = sanitize_action(action)
                    local_clips += count
                clip_count += local_clips
                request_count += 1
                log_rows.append(
                    {
                        "mode": "shadow",
                        "request": request_count,
                        "position": position.tolist(),
                        "state_age_ms": state_age * 1000.0,
                        "camera_age_ms": [base_age * 1000.0, wrist_age * 1000.0],
                        "inference_ms": inference_ms,
                        "actions": actions.tolist(),
                        "clipped_values": local_clips,
                    }
                )
                print(
                    "SHADOW request={} inference_ms={:.1f} clips={} first={}".format(
                        request_count, inference_ms, local_clips, np.round(actions[0], 6).tolist()
                    ),
                    flush=True,
                )
            print("SHADOW_PASS requests={} commands_sent=0".format(request_count), flush=True)
            return

        # The server intentionally closes an idle client after its request
        # timeout.  An operator can take much longer than that to press the
        # dead-man after preflight, so do not leave a connection parked while
        # the robot is armed.  Reconnect only after F3 is pressed; any failure
        # here occurs before the first command can be published.
        conn.close()
        conn = None
        gripper = GripperController(gripper_open >= 0.5)
        gripper.wait_for_servers(rospy)
        controller.gripper = gripper
        deadman = DeadmanMonitor(args.deadman, controller.emergency_hold)
        controller.deadman = deadman
        deadman.start()
        deadman.arm()
        print(
            "LIVE_READY hold_F3_to_run=true first_release_ends=true "
            "max_duration_s={:.1f} actions_per_chunk={}".format(
                args.duration, args.actions_per_chunk
            ),
            flush=True,
        )
        if not deadman.wait_for_press(args.arm_timeout):
            raise SafetyError("operator did not press F3 before arm timeout")

        conn = PolicyConnection(args.server, args.port, args.token, args.timeout)
        health = conn.health()
        print("POLICY_SERVER_RECONNECTED {}".format(json.dumps(health, sort_keys=True)), flush=True)
        _, initial_position, initial_rotation, width, _, _ = validate_live_state(cache)
        target_position = initial_position.copy()
        target_rotation = initial_rotation.copy()
        started = time.monotonic()
        next_tick = started
        result = "duration_complete"

        while time.monotonic() - started < args.duration:
            if not deadman.enabled():
                result = controller.stop_reason or "deadman_released"
                break
            _, measured_position, measured_rotation, width, state_age, grip_age = validate_live_state(cache)
            base, base_age = external.latest()
            wrist_rgb, wrist_age = wrist.latest()
            gripper_open = 1.0 if gripper.command_open else 0.0
            state = make_state(measured_position, measured_rotation, gripper_open)
            actions, inference_ms = conn.infer(state, base, wrist_rgb, args.prompt)
            inference_times.append(inference_ms)

            # Re-anchor every short receding-horizon execution to measured
            # state.  The model predicts ten actions, but executing only the
            # first few limits open-loop motion before both cameras and robot
            # state are observed again.
            target_position = measured_position.copy()
            target_rotation = measured_rotation.copy()
            for chunk_index, raw_action in enumerate(actions[: args.actions_per_chunk]):
                if not deadman.enabled():
                    result = controller.stop_reason or "deadman_released"
                    break
                validate_live_state(cache)
                action, local_clips = sanitize_action(raw_action)
                clip_count += local_clips
                candidate_position = target_position + action[:3]
                candidate_rotation = target_rotation.dot(rpy_to_rotation(action[3:6]))
                if np.any(candidate_position < WORKSPACE_LOWER) or np.any(candidate_position > WORKSPACE_UPPER):
                    raise SafetyError("policy target would leave trained workspace: {}".format(candidate_position.tolist()))
                if np.linalg.norm(candidate_position - initial_position) > MAX_SESSION_TRANSLATION:
                    raise SafetyError("policy target exceeded session translation limit")
                if rotation_angle(initial_rotation.T.dot(candidate_rotation)) > MAX_SESSION_ROTATION:
                    raise SafetyError("policy target exceeded session rotation limit")
                if not controller.publish_policy_target(candidate_position, candidate_rotation, action[6]):
                    result = controller.stop_reason or "deadman_released"
                    break
                target_position = candidate_position
                target_rotation = candidate_rotation
                executed += 1
                log_rows.append(
                    {
                        "mode": "live",
                        "elapsed_s": time.monotonic() - started,
                        "chunk_index": chunk_index,
                        "raw_action": raw_action.tolist(),
                        "action": action.tolist(),
                        "target_position": target_position.tolist(),
                        "target_rotation": target_rotation.reshape(-1).tolist(),
                        "inference_ms": inference_ms,
                        "clipped_values": local_clips,
                    }
                )
                next_tick += 1.0 / ACTION_HZ
                delay = next_tick - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    # Inference is intentionally not hidden behind motion.  A
                    # missed schedule is logged, but never causes catch-up
                    # commands to be emitted in a burst.
                    next_tick = time.monotonic()
            if result != "duration_complete":
                break

        controller.emergency_hold(result)
        print(
            "LIVE_RESULT result={} active_s={:.2f} actions_executed={} gripper_commands={} clipped_values={}".format(
                result, 0.0 if started is None else time.monotonic() - started,
                executed, gripper.commands, clip_count,
            ),
            flush=True,
        )
    except Exception:
        result = "error"
        controller.emergency_hold("exception")
        raise
    finally:
        if deadman is not None:
            deadman.close()
        if controller.gripper is not None:
            controller.gripper.emergency_stop()
        if conn is not None:
            conn.close()
        external.stop()
        wrist.stop()
        output = Path(args.log).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        summary = {
            "mode": args.mode,
            "result": result,
            "prompt": args.prompt,
            "server": args.server,
            "duration_requested_s": args.duration,
            "actions_per_chunk": args.actions_per_chunk,
            "active_duration_s": None if started is None else time.monotonic() - started,
            "actions_executed": executed,
            "clipped_values": clip_count,
            "inference_ms": inference_times,
            "initial_position": None if initial_position is None else initial_position.tolist(),
            "rows": log_rows,
        }
        output.write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        if first_base is not None:
            cv2.imwrite(str(output.with_suffix(".external.jpg")), cv2.cvtColor(first_base, cv2.COLOR_RGB2BGR))
        if first_wrist is not None:
            cv2.imwrite(str(output.with_suffix(".wrist.jpg")), cv2.cvtColor(first_wrist, cv2.COLOR_RGB2BGR))
        print("RUN_LOG {}".format(output), flush=True)


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("recorded", "shadow", "live"), required=True)
    parser.add_argument("--server", required=True)
    parser.add_argument("--port", type=int, default=5559)
    parser.add_argument("--token", default="pnp7-local-demo")
    parser.add_argument("--timeout", type=float, default=120.0)
    parser.add_argument("--prompt", default=PROMPT)
    parser.add_argument("--recorded-episode")
    parser.add_argument("--samples", type=int, default=3)
    parser.add_argument("--duration", type=float, default=10.0)
    parser.add_argument(
        "--actions-per-chunk",
        type=int,
        default=3,
        help="execute this many leading actions from each 10-action prediction before reobserving",
    )
    parser.add_argument("--arm-timeout", type=float, default=120.0)
    parser.add_argument("--deadman", default="/dev/pnp7_deadman")
    parser.add_argument("--log", required=True)
    args = parser.parse_args()
    if args.mode == "recorded" and not args.recorded_episode:
        parser.error("--recorded-episode is required in recorded mode")
    if args.duration <= 0 or args.duration > 60:
        parser.error("--duration must be in (0, 60]")
    if args.samples < 1 or args.samples > 10:
        parser.error("--samples must be between 1 and 10")
    if args.actions_per_chunk < 1 or args.actions_per_chunk > 10:
        parser.error("--actions-per-chunk must be between 1 and 10")
    return args


def main():
    args = parse_args()
    if args.mode == "recorded":
        run_recorded(args)
    else:
        run_live(args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("INTERRUPTED", file=sys.stderr, flush=True)
        raise SystemExit(130)
    except Exception as exc:
        print("RUN_FATAL {}: {}".format(type(exc).__name__, exc), file=sys.stderr, flush=True)
        raise
