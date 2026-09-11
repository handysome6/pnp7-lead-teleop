#!/usr/bin/env python3
"""FR3/Robotiq profile of the existing ROS PI0.5 client.

Reuses its camera capture, socket protocol, pose math, ROS state cache, target
publisher, and released-at-start/hold-to-run pedal monitor. No new FCI bridge.
"""

import argparse
import json
import threading
import time
from pathlib import Path

import cv2
import numpy as np

import pnp7_policy as old
from robotiq_policy import RobotiqPolicy


MODEL_ID = "pi05_pnp7_40merged_step7500"
EXTERNAL = "317622072022"
WRIST = "233622071437"
GRIPPER_PORT = "/dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DAAQMQW7-if00-port0"
TRACKING_POSITION_LIMIT = .045
TRACKING_ROTATION_LIMIT = .30
STEPS_PER_INFERENCE = 2


def check_tracking(position, rotation, measured, measured_rotation):
    distance = np.linalg.norm(position - measured)
    angle = old.rotation_angle(measured_rotation.T @ rotation)
    if not np.isfinite(distance + angle) or distance > TRACKING_POSITION_LIMIT or angle > TRACKING_ROTATION_LIMIT:
        raise old.SafetyError("target tracking error limit: translation_mm={:.2f}, rotation_rad={:.4f}".format(
            1000 * distance, angle))


class Camera(old.CameraStream):
    def latest(self, max_age_s=.2):
        with self.lock:
            frame = None if self.frame is None else self.frame.copy()
            timestamp = self.frame_ns
        age = (time.monotonic_ns() - timestamp) / 1e9
        if self.error or frame is None or age > max_age_s:
            raise old.SafetyError("{} camera not fresh: {}".format(self.role, self.error or age))
        if not hasattr(self, "first_frame_seen"):
            self.first_frame_seen = time.monotonic()
        if time.monotonic() - self.first_frame_seen < 2:
            raise old.SafetyError("{} camera exposure/white-balance warmup".format(self.role))
        # Match export_lerobot's default cv2.resize (INTER_LINEAR), then BGR->RGB.
        rgb = cv2.cvtColor(cv2.resize(frame, (224, 224)), cv2.COLOR_BGR2RGB)
        return rgb, age


def start_observation_devices(cameras, gripper_port, timeout=15):
    """Finish blocking RealSense setup BEFORE starting timed serial exchanges.

    Camera SDK initialization can delay other Python threads beyond the 80 ms
    serial deadline. Keep that deadline unchanged; avoid overlapping startup.
    The caller owns camera cleanup even when this preflight fails.
    """
    deadline = time.monotonic() + timeout
    for camera in cameras:
        camera.start()
    while True:
        try:
            for camera in cameras:
                camera.latest()
            break
        except old.SafetyError as exc:
            if any(camera.error is not None for camera in cameras) or time.monotonic() >= deadline:
                raise old.SafetyError("camera startup preflight failed: {}".format(exc)) from exc
            time.sleep(.02)
    print("CAMERAS_READY starting_readonly_gripper_status=true", flush=True)
    return RobotiqPolicy(gripper_port)


class Limits:
    def __init__(self, path, profile="smoke"):
        self.profile = profile
        stats = json.loads(Path(path).read_text())["norm_stats"]
        self.lower = np.asarray(stats["actions"]["q01"][:7])
        self.upper = np.asarray(stats["actions"]["q99"][:7])
        self.upper[6] = 1.0
        self.workspace_lower = np.asarray(stats["state"]["q01"][:3]) - .02
        self.workspace_upper = np.asarray(stats["state"]["q99"][:3]) + .02

    def position(self, position):
        if (np.any(position < self.workspace_lower) or
                np.any(position > self.workspace_upper)):
            raise old.SafetyError("EE outside trained workspace: {} not in {}..{}".format(
                position.tolist(), self.workspace_lower.tolist(), self.workspace_upper.tolist()))

    def action(self, raw):
        raw = np.asarray(raw, dtype=float)
        if raw.shape != (7,) or not np.isfinite(raw).all():
            raise old.SafetyError("invalid action")
        scale = np.maximum(np.abs(self.lower[:6]), np.abs(self.upper[:6]))
        if np.any(np.abs(raw[:6]) > 4 * scale):
            raise old.SafetyError("action exceeds 4x training-percentile bound")
        if self.profile == "full":
            # Native learned delta at 30 Hz; no smoke-test amplitude scaling.
            action = raw.copy()
            action[6] = np.clip(action[6], 0, 1)
            return action
        action = np.clip(raw, self.lower, self.upper)
        # Initial validation only: <=1 mm and <=0.003 rad per 30 Hz target.
        for section, maximum in ((slice(0, 3), .001), (slice(3, 6), .003)):
            norm = np.linalg.norm(action[section])
            if norm > maximum:
                action[section] *= maximum / norm
        return action


class Controller(old.LiveController):
    def _pose_message(self, position, rotation):
        message = super()._pose_message(position, rotation)
        message.header.frame_id = "fr3_link0"
        return message


class AsyncPolicy:
    """One inference in flight; main thread alone publishes robot commands.

    The command loop consumes only the first two actions of selected chunks.
    Delta targets remain continuous, with current-state tracking checks on
    every tick. This is not RTC or latency-compensated training.
    """
    def __init__(self, infer):
        self.infer = infer
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.packet = None
        self.error = None
        self.thread = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        sequence = 0
        try:
            while not self.stop_event.is_set():
                observed_at = time.monotonic()
                p, rotation, actions = self.infer()
                sequence += 1
                with self.lock:
                    self.packet = (sequence, observed_at, p, rotation, actions)
        except Exception as exc:
            self.error = exc

    def start(self):
        self.thread.start()

    def latest(self):
        if self.error:
            raise old.SafetyError("async policy failed: {}".format(self.error))
        with self.lock:
            packet = self.packet
        if packet is not None and time.monotonic() - packet[1] > .4:
            raise old.SafetyError("policy observation is older than 400 ms")
        return packet

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2.5)
        if self.thread.is_alive():
            raise old.SafetyError("inference worker did not stop")


class TwoStepChunks:
    """Execute indices 0,1 once, then wait for a strictly newer prediction.

    Finish the selected prefix before selecting the latest available chunk.
    Waiting never replays an action or consumes indices 2..9. This limits delta
    accumulation rate; it does not reset accumulated targets to measured pose.
    """
    def __init__(self):
        self.packet = None
        self.index = 0

    def next_action(self, latest, now):
        if self.packet is None or self.index >= STEPS_PER_INFERENCE:
            if latest is None or (self.packet is not None and latest[0] <= self.packet[0]):
                return None
            actions = np.asarray(latest[4])
            if actions.ndim != 2 or actions.shape[0] < STEPS_PER_INFERENCE or actions.shape[1] != 7:
                raise old.SafetyError("policy chunk must contain at least two 7D actions")
            self.packet, self.index = latest, 0
        if not 0 <= now - self.packet[1] <= .4:
            raise old.SafetyError("selected policy observation is stale or future-dated")
        sequence, observed_at, _, _, actions = self.packet
        index = self.index
        self.index += 1
        return sequence, index, actions[index], observed_at


def run(args):
    import rospy
    from controller_manager_msgs.srv import ListControllers, SwitchController
    from franka_msgs.msg import FrankaState
    from geometry_msgs.msg import PoseStamped

    rospy.init_node("pnp7_robot_s0_policy", disable_signals=True)
    cache = old.RobotStateCache()
    rospy.Subscriber("/franka_state_controller/franka_states", FrankaState,
                     cache.state_callback, queue_size=1, tcp_nodelay=True)
    publisher = rospy.Publisher("/cartesian_impedance_controller/equilibrium_pose",
                                PoseStamped, queue_size=1, tcp_nodelay=True)
    controller = Controller(rospy, cache, publisher)
    list_controllers = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
    switch = rospy.ServiceProxy("/controller_manager/switch_controller", SwitchController)
    rospy.wait_for_service("/controller_manager/list_controllers", timeout=5)
    states = {c.name: c.state for c in list_controllers().controller}
    if states.get("cartesian_impedance_controller") not in ("stopped", "initialized"):
        raise old.SafetyError("impedance controller must be loaded/stopped, got {}".format(states))
    home_report = None
    if args.mode == "live":
        # Mandatory for every live entry point, including direct Python calls.
        # Shadow remains a strictly read-only diagnostic, not a validation run.
        from restore_home import restore_home
        home_report = restore_home(rospy, args.home_config)
    limits = Limits(args.norm_stats, args.profile)
    cameras = [Camera(EXTERNAL, "external"), Camera(WRIST, "wrist")]
    gripper = None
    conn = None
    deadman = None
    own_controller = False
    rows = []
    commands = 0
    result = "error"
    health = None
    first_images = None
    started = None
    worker = None
    targets = []
    waiting_ticks = 0
    error_message = None
    motion_elapsed_s = None
    output = Path(args.log)
    output.parent.mkdir(parents=True, exist_ok=True)

    def observe():
        message, p, rotation, _, state_age, _ = cache.snapshot()
        if message.robot_mode not in (FrankaState.ROBOT_MODE_IDLE, FrankaState.ROBOT_MODE_MOVE):
            raise old.SafetyError("robot mode not idle/move: {}".format(message.robot_mode))
        grip = gripper.snapshot()
        frames = [camera.latest() for camera in cameras]
        if abs(frames[0][1] - frames[1][1]) > .1:
            raise old.SafetyError("camera capture skew exceeds 100 ms")
        state = old.make_state(p, rotation, float(grip["requested"] < 128))
        return p, rotation, state, [frame[0] for frame in frames], dict(
            state_age_ms=state_age * 1000,
            camera_age_ms=[frame[1] * 1000 for frame in frames], gripper=grip,
            robot_mode=message.robot_mode)

    def inference():
        p, rotation, state, images, ages = observe()
        before = time.monotonic()
        actions, inference_ms = conn.infer(state, *images, old.PROMPT)
        elapsed = time.monotonic() - before
        if elapsed > .8:
            raise old.SafetyError("inference round trip exceeds 800 ms")
        for action in actions:
            limits.action(action)
        entry = dict(position=p.tolist(), state=state.tolist(), actions=actions.tolist(),
                     inference_ms=inference_ms, round_trip_ms=elapsed * 1000, **ages)
        rows.append(entry)
        print("INFERENCE sample={} round_trip_ms={:.1f} first={}".format(
            len(rows), elapsed * 1000, np.round(actions[0], 6).tolist()), flush=True)
        return p, rotation, actions

    try:
        gripper = start_observation_devices(cameras, args.gripper_port)
        deadline = time.monotonic() + 15
        while True:
            try:
                p, rotation, state, first_images, _ = observe()
                break
            except Exception as exc:
                if gripper.error is not None:
                    raise old.SafetyError("gripper preflight failed (worker stopped): {}".format(exc)) from exc
                if time.monotonic() > deadline:
                    raise old.SafetyError("observation preflight failed: {}".format(exc)) from exc
                time.sleep(.05)
        conn = old.PolicyConnection(args.server, 5559, "pnp7-local-demo", 2.0)
        health = conn.health()
        if health.get("model") != MODEL_ID or not health.get("strict_checkpoint"):
            raise old.SafetyError("wrong checkpoint: {}".format(health))
        inference()
        print("PREFLIGHT_PASS mode={} model={}".format(args.mode, MODEL_ID), flush=True)
        if args.mode == "shadow":
            started = time.monotonic()
            while time.monotonic() - started < args.duration:
                inference()
            result = "shadow_pass"
            return

        # A controller-side watchdog must be built/loaded before live execution.
        if not rospy.get_param("/cartesian_impedance_controller/policy_watchdog_available", False):
            raise old.SafetyError("controller-side policy watchdog is unavailable")
        limits.position(p)
        controller.gripper = gripper
        deadman = old.DeadmanMonitor(args.deadman, controller.emergency_hold)
        controller.deadman = deadman
        deadman.start()
        deadman.arm()
        conn.close()
        conn = None
        print("LIVE_READY hold_F3=true release_ends_run=true duration={} profile={} async={} actions_per_inference={}".format(
            args.duration, args.profile, args.profile == "full", STEPS_PER_INFERENCE), flush=True)
        if not deadman.wait_for_press(args.arm_timeout):
            raise old.SafetyError("pedal arm timeout")
        if not deadman.enabled():
            raise old.SafetyError("pedal released before start")
        own_controller = True
        response = switch(start_controllers=["cartesian_impedance_controller"],
                          stop_controllers=[], strictness=2, start_asap=True, timeout=1.0)
        if not response.ok:
            raise old.SafetyError("could not start impedance controller")
        conn = old.PolicyConnection(args.server, 5559, "pnp7-local-demo", 2.0)
        if conn.health().get("model") != MODEL_ID:
            raise old.SafetyError("checkpoint changed while arming")
        gripper.enable_motion(deadman.enabled)
        initial_p, initial_r, _, _, _ = observe()
        limits.position(initial_p)
        started = time.monotonic()
        result = "duration_complete"
        if args.profile == "full":
            worker = AsyncPolicy(inference)
            worker.start()
            p, rotation = initial_p.copy(), initial_r.copy()
            chunks = TwoStepChunks()
            next_tick = time.monotonic()
            while time.monotonic() - started < args.duration and deadman.enabled():
                packet = worker.latest()
                if packet is None:
                    if time.monotonic() - started > .8:
                        raise old.SafetyError("no initial asynchronous policy result")
                    time.sleep(.005)
                    next_tick = time.monotonic()
                    continue
                measured, measured_r, _, _, _ = observe()
                limits.position(measured)
                selected = chunks.next_action(packet, time.monotonic())
                if selected is not None:
                    sequence, index, raw, observed_at = selected
                    action = limits.action(raw)
                    p = p + action[:3]
                    rotation = rotation @ old.rpy_to_rotation(action[3:6])
                else:
                    # No repeated target publications or gripper updates while
                    # waiting. Existing robot/gripper watchdogs stay effective.
                    waiting_ticks += 1
                limits.position(p)
                # The absolute demonstrated workspace replaces the 5 cm smoke
                # envelope; tracking-error and orientation guards remain.
                if old.rotation_angle(initial_r.T @ rotation) > 1.5:
                    raise old.SafetyError("full-run rotation limit")
                check_tracking(p, rotation, measured, measured_r)
                if selected is not None and controller.publish_policy_target(p, rotation, action[6]):
                    commands += 1
                    targets.append(dict(elapsed_s=time.monotonic() - started,
                                        sequence=sequence, index=index, action=action.tolist(),
                                        observation_age_ms=1000 * (time.monotonic() - observed_at),
                                        target_position=p.tolist(), measured_position=measured.tolist()))
                next_tick += 1 / 30
                delay = next_tick - time.monotonic()
                if delay < -.05:
                    raise old.SafetyError("30 Hz command loop missed deadline by >50 ms")
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_tick = time.monotonic()  # never send catch-up bursts
        while args.profile == "smoke" and time.monotonic() - started < args.duration and deadman.enabled():
            p, rotation, actions = inference()
            for raw in actions[:STEPS_PER_INFERENCE]:
                if not deadman.enabled() or time.monotonic() - started >= args.duration:
                    break
                measured, measured_r, _, _, _ = observe()
                limits.position(measured)
                action = limits.action(raw)
                p = p + action[:3]
                rotation = rotation @ old.rpy_to_rotation(action[3:6])
                limits.position(p)
                if np.linalg.norm(p - initial_p) > .05 or np.linalg.norm(measured - initial_p) > .055:
                    raise old.SafetyError("session translation limit")
                if old.rotation_angle(initial_r.T @ rotation) > .15:
                    raise old.SafetyError("session rotation limit")
                check_tracking(p, rotation, measured, measured_r)
                if controller.publish_policy_target(p, rotation, action[6]):
                    commands += 1
                time.sleep(1 / 30)
        if deadman.stopped():
            result = "deadman_released"
    except BaseException as exc:
        result = "error"
        error_message = "{}: {}".format(type(exc).__name__, exc)
        raise
    finally:
        if started is not None:
            motion_elapsed_s = time.monotonic() - started
        cleanup_errors = []
        if own_controller:
            controller.emergency_hold("run_end")
            try:
                response = switch(start_controllers=[], stop_controllers=["cartesian_impedance_controller"],
                                  strictness=2, start_asap=True, timeout=1.0)
                if not response.ok:
                    raise old.SafetyError("controller manager rejected stop")
            except Exception as exc:
                cleanup_errors.append("controller stop: {}".format(exc))
                print("STOP_FAILED use physical stop and inspect controller", flush=True)
        if deadman:
            deadman.close()
        if conn:
            if worker:
                try:
                    worker.close()
                except Exception as exc:
                    cleanup_errors.append("inference worker: {}".format(exc))
            conn.close()
        for camera in cameras:
            camera.stop()
        if gripper:
            try:
                gripper.close()
            except Exception as exc:
                cleanup_errors.append("gripper stop: {}".format(exc))
        if cleanup_errors:
            result = "cleanup_failed"
        report = dict(result=result, mode=args.mode, health=health, prompt=old.PROMPT,
                      commands_sent=commands, samples=rows,
                      gripper_writes=0 if gripper is None else gripper.commands,
                      cleanup_errors=cleanup_errors, profile=args.profile,
                      duration_requested=args.duration, targets=targets,
                      error=error_message, active_duration_s=motion_elapsed_s,
                      home=home_report, tracking_position_limit_m=TRACKING_POSITION_LIMIT,
                      tracking_rotation_limit_rad=TRACKING_ROTATION_LIMIT,
                      actions_per_inference=STEPS_PER_INFERENCE, waiting_ticks=waiting_ticks)
        output.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
        if first_images:
            for role, image in zip(("external", "wrist"), first_images):
                cv2.imwrite(str(output.with_suffix("." + role + ".jpg")),
                            cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        print("RESULT {} commands_sent={} log={}".format(result, commands, output), flush=True)
        if cleanup_errors:
            raise old.SafetyError("; ".join(cleanup_errors))


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("shadow", "live"), default="shadow")
    parser.add_argument("--profile", choices=("smoke", "full"), default="smoke")
    parser.add_argument("--server", default="100.71.83.59")
    parser.add_argument("--duration", type=float, default=5)
    parser.add_argument("--arm-timeout", type=float, default=120)
    parser.add_argument("--deadman", default="/dev/foot_brake")
    parser.add_argument("--gripper-port", default=GRIPPER_PORT)
    parser.add_argument("--norm-stats", default=str(Path(__file__).parent / "ros/norm_stats_40merged.json"))
    parser.add_argument("--log", required=True)
    parser.add_argument("--home-config", default=str(Path(__file__).resolve().parents[1] / "conf/full100b.conf"))
    args = parser.parse_args()
    maximum = 40 if args.profile == "full" else 10
    if not 0 < args.duration <= maximum:
        parser.error("duration exceeds selected profile limit: {} seconds".format(maximum))
    return args


if __name__ == "__main__":
    import signal
    def interrupted(signum, frame):
        raise KeyboardInterrupt("signal {}".format(signum))
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGHUP, interrupted)
    from restore_home import validation_lock
    with validation_lock():
        run(parse_args())
