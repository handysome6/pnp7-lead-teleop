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
from tracking_governor import (TrackingGovernor, TrackingMonitor, apply_action, cap_lead, check_tracking,
                               TRACKING_POSITION_LIMIT, TRACKING_ROTATION_LIMIT)


# (40merged checkpoint, per-request noise seed) -> robot-s1 server port. Deployed
# servers reset seed 0 before every inference; the seedexp server takes a seed
# per request. Model id and seed capability are verified via health.
POLICY_SERVERS = {(7500, False): 5559, (2500, False): 5560, (5000, False): 5561, (7500, True): 5562}
EXTERNAL = "317622072022"
WRIST = "233622071437"
GRIPPER_PORT = "/dev/serial/by-id/usb-FTDI_USB_TO_RS-485_DAAQMQW7-if00-port0"
# Default leading actions executed per fresh chunk (the full profile may use 2-4).
STEPS_PER_INFERENCE = 2
# Tail gripper vote: mean of the last predicted steps, the state a chunk plans to end in.
GRIPPER_TAIL_STEPS = 5
# Action k of a chunk is meant for k/30 s after its observation; never run it later than this.
MAX_ACTION_LATENESS_S = .4
# End the run only when no newer prediction has arrived for this long.
MAX_POLICY_SILENCE_S = 1.0


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


def gripper_vote_value(chunk, mode):
    """Clipped gripper value one chunk votes with.

    tail: mean of its last GRIPPER_TAIL_STEPS steps, so an open-first-then-closed
    noise sample votes closed while a planned release votes open. chunk: mean of
    the whole horizon.
    """
    horizon = np.clip(np.asarray(chunk, dtype=float)[:, 6], 0, 1)
    return float((horizon[-GRIPPER_TAIL_STEPS:] if mode == "tail" else horizon).mean())


class Controller(old.LiveController):
    def _pose_message(self, position, rotation):
        message = super()._pose_message(position, rotation)
        message.header.frame_id = "fr3_link0"
        return message

    def publish_policy_target(self, position, rotation, gripper_value, source=None):
        """With `source`, the gripper gets one debounced vote per policy chunk."""
        if source is None:
            return super().publish_policy_target(position, rotation, gripper_value)
        with self.command_lock:
            if self.deadman is None or not self.deadman.enabled():
                return False
            self.publisher.publish(self._pose_message(position, rotation))
            if self.gripper is not None:
                self.gripper.vote(gripper_value, source)
            return True


class AsyncPolicy:
    """One inference in flight; main thread alone publishes robot commands.

    The command loop consumes only a short leading prefix of selected chunks.
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
        # Waiting for a newer chunk is safe (nothing is published); stale actions
        # are dropped by PrefixChunks. Only a silent worker/server ends the run.
        if packet is not None and time.monotonic() - packet[1] > MAX_POLICY_SILENCE_S:
            raise old.SafetyError("newest policy observation is older than {:.0f} ms".format(
                1000 * MAX_POLICY_SILENCE_S))
        return packet

    def close(self):
        self.stop_event.set()
        self.thread.join(timeout=2.5)
        if self.thread.is_alive():
            raise old.SafetyError("inference worker did not stop")


class PrefixChunks:
    """Execute indices 0..steps-1 once, then wait for a strictly newer prediction.

    Finish the selected prefix before selecting the latest available chunk.
    Waiting never replays an action or consumes later indices. This limits delta
    accumulation rate; it does not reset accumulated targets to measured pose.
    An action whose observation is too old is dropped with the rest of its
    prefix (recorded in `skipped`), never executed.
    """
    def __init__(self, steps=STEPS_PER_INFERENCE):
        self.steps = steps
        self.packet = None
        self.index = 0
        self.skipped = []

    def next_action(self, latest, now):
        if self.packet is None or self.index >= self.steps:
            if latest is None or (self.packet is not None and latest[0] <= self.packet[0]):
                return None
            actions = np.asarray(latest[4])
            if actions.ndim != 2 or actions.shape[0] < self.steps or actions.shape[1] != 7:
                raise old.SafetyError("policy chunk must contain at least {} 7D actions".format(self.steps))
            self.packet, self.index = latest, 0
        age = now - self.packet[1]
        if not age >= 0:
            raise old.SafetyError("selected policy observation is future-dated")
        lateness = age - self.index / old.ACTION_HZ
        if lateness > MAX_ACTION_LATENESS_S:
            self.skipped.append(dict(sequence=self.packet[0], index=self.index, age_ms=1000 * age,
                                     lateness_ms=1000 * lateness))
            self.index = self.steps
            return None
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
    chunks = PrefixChunks(args.actions_per_inference)
    per_request_seed = args.noise_seed == "per-request"
    model_id = "pi05_pnp7_40merged_step{}{}".format(args.policy_step, "_seedexp" if per_request_seed else "")
    policy_port = POLICY_SERVERS[(args.policy_step, per_request_seed)]
    # A fresh, logged base per run; inference k uses base + k.
    noise_seed_base = time.time_ns() % 2**31 if per_request_seed else None
    governor = TrackingGovernor()
    governor_samples = []
    guarded = args.client_tracking_stop == "on"
    monitor = TrackingMonitor()
    tracking_samples = []
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
        seed = None if noise_seed_base is None else (noise_seed_base + len(rows)) % 2**31
        before = time.monotonic()
        actions, inference_ms = conn.infer(state, *images, old.PROMPT, seed=seed)
        elapsed = time.monotonic() - before
        if elapsed > .8:
            raise old.SafetyError("inference round trip exceeds 800 ms")
        for action in actions:
            limits.action(action)
        entry = dict(position=p.tolist(), state=state.tolist(), actions=actions.tolist(),
                     inference_ms=inference_ms, round_trip_ms=elapsed * 1000, noise_seed=seed, **ages)
        rows.append(entry)
        print("INFERENCE sample={} round_trip_ms={:.1f} first={}".format(
            len(rows), elapsed * 1000, np.round(actions[0], 6).tolist()), flush=True)
        return p, rotation, actions

    def govern(p, rotation, measured, measured_r, action):
        p, rotation, applied, info = governor.step(
            p, rotation, measured, measured_r, action, time.monotonic())
        governor_samples.append(dict(elapsed_s=time.monotonic() - started, **info))
        if info["state"] == "paused":
            # Do not advance gripper state while the arm cannot advance.
            gripper.emergency_stop()
        previous = governor_samples[-2]["state"] if len(governor_samples) > 1 else "normal"
        if info["state"] != previous:
            print("TRACKING_GOVERNOR state={} scale={:.3f} error_mm={:.2f} error_rad={:.4f}".format(
                info["state"], info["scale"], 1000 * info["position_error_m"], info["rotation_error_rad"]), flush=True)
        return p, rotation, applied

    def unguarded(p, rotation, measured, measured_r, action):
        # Diagnostic mode: unscaled policy deltas, leashed to the measured pose so
        # the target cannot wind up ahead of a slower arm; errors never stop.
        dropped_m = dropped_rad = 0.0
        if action is not None:
            p, rotation = apply_action(p, rotation, action)
            p, rotation, dropped_m, dropped_rad = cap_lead(
                p, rotation, measured, measured_r, args.target_lead_cap_mm / 1000, args.target_lead_cap_rad)
        info, changed = monitor.step(p, rotation, measured, measured_r)
        tracking_samples.append(dict(elapsed_s=time.monotonic() - started, has_action=action is not None,
                                     dropped_translation_m=dropped_m, dropped_rotation_rad=dropped_rad, **info))
        if changed:
            print("TRACKING_UNGUARDED level={} error_mm={:.2f} error_rad={:.4f} controller_ignores_target={}".format(
                info["level"], 1000 * info["position_error_m"], info["rotation_error_rad"],
                str(info["level"] == 2).lower()), flush=True)
        return p, rotation, action

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
        conn = old.PolicyConnection(args.server, policy_port, "pnp7-local-demo", 2.0)
        health = conn.health()
        if (health.get("model") != model_id or not health.get("strict_checkpoint") or
                bool(health.get("per_request_seed")) != per_request_seed):
            raise old.SafetyError("wrong checkpoint or seed mode: {}".format(health))
        inference()
        print("PREFLIGHT_PASS mode={} model={} port={} noise_seed={} noise_seed_base={}".format(
            args.mode, model_id, policy_port, args.noise_seed, noise_seed_base), flush=True)
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
        print("LIVE_READY hold_F3=true release_ends_run=true client_tracking_stop={} tracking_governor={} target_lead_cap_mm={} target_lead_cap_rad={} duration={} profile={} async={} actions_per_inference={} max_action_lateness_ms={:.0f} max_policy_silence_ms={:.0f} gripper_vote={}".format(
            args.client_tracking_stop, "two_stage" if guarded else "bypassed",
            args.target_lead_cap_mm, args.target_lead_cap_rad,
            args.duration, args.profile, args.profile == "full", args.actions_per_inference,
            1000 * MAX_ACTION_LATENESS_S, 1000 * MAX_POLICY_SILENCE_S, args.gripper_vote), flush=True)
        if not guarded:
            print("CLIENT_TRACKING_STOP_OFF unscaled policy deltas leashed to {} mm / {} rad of measured pose, "
                  "excess dropped; SERL watchdog, workspace and pedal remain; physical stop is the tracking "
                  "safeguard".format(args.target_lead_cap_mm, args.target_lead_cap_rad), flush=True)
        if not deadman.wait_for_press(args.arm_timeout):
            raise old.SafetyError("pedal arm timeout")
        if not deadman.enabled():
            raise old.SafetyError("pedal released before start")
        own_controller = True
        response = switch(start_controllers=["cartesian_impedance_controller"],
                          stop_controllers=[], strictness=2, start_asap=True, timeout=1.0)
        if not response.ok:
            raise old.SafetyError("could not start impedance controller")
        conn = old.PolicyConnection(args.server, policy_port, "pnp7-local-demo", 2.0)
        if conn.health().get("model") != model_id:
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
                skipped = len(chunks.skipped)
                selected = chunks.next_action(packet, time.monotonic())
                if len(chunks.skipped) > skipped:
                    print("STALE_ACTION_SKIPPED sequence={sequence} index={index} age_ms={age_ms:.0f} lateness_ms={lateness_ms:.0f}".format(
                        **chunks.skipped[-1]), flush=True)
                if selected is not None:
                    sequence, index, raw, observed_at = selected
                    action = limits.action(raw)
                else:
                    # No repeated target publications or gripper updates while
                    # waiting. Existing robot/gripper watchdogs stay effective.
                    waiting_ticks += 1
                    action = None
                p, rotation, applied = (govern if guarded else unguarded)(
                    p, rotation, measured, measured_r, action)
                limits.position(p)
                # The absolute demonstrated workspace replaces the 5 cm smoke
                # envelope; the orientation guard remains, tracking guard unless off.
                if old.rotation_angle(initial_r.T @ rotation) > 1.5:
                    raise old.SafetyError("full-run rotation limit")
                if guarded:
                    check_tracking(p, rotation, measured, measured_r)
                if applied is not None:
                    if args.gripper_vote == "action":
                        grip, source = float(applied[6]), None
                    else:
                        # One debounced vote per chunk (see gripper_vote_value).
                        grip, source = gripper_vote_value(chunks.packet[4], args.gripper_vote), sequence
                    if controller.publish_policy_target(p, rotation, grip, source):
                        commands += 1
                        targets.append(dict(elapsed_s=time.monotonic() - started,
                                            sequence=sequence, index=index, action=applied.tolist(),
                                            policy_action=action.tolist(), gripper_command=grip,
                                            governor_scale=governor.scale if guarded else None,
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
                p, rotation, applied = govern(p, rotation, measured, measured_r, action)
                limits.position(p)
                if np.linalg.norm(p - initial_p) > .05 or np.linalg.norm(measured - initial_p) > .055:
                    raise old.SafetyError("session translation limit")
                if old.rotation_angle(initial_r.T @ rotation) > .15:
                    raise old.SafetyError("session rotation limit")
                check_tracking(p, rotation, measured, measured_r)
                if applied is not None and controller.publish_policy_target(p, rotation, applied[6]):
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
                      actions_per_inference=args.actions_per_inference, waiting_ticks=waiting_ticks,
                      client_tracking_stop=args.client_tracking_stop, tracking_samples=tracking_samples,
                      target_lead_cap_mm=args.target_lead_cap_mm, target_lead_cap_rad=args.target_lead_cap_rad,
                      noise_seed=args.noise_seed, noise_seed_base=noise_seed_base,
                      max_action_lateness_s=MAX_ACTION_LATENESS_S,
                      max_policy_silence_s=MAX_POLICY_SILENCE_S, skipped_stale_actions=chunks.skipped,
                      gripper_vote=args.gripper_vote,
                      gripper_switches=[] if gripper is None or started is None else
                      [dict(elapsed_s=t - started, target=target) for t, target in gripper.switches],
                      tracking_governor=governor.settings, governor_samples=governor_samples,
                      governor_last=governor.last_info)
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
    parser.add_argument("--policy-step", type=int, choices=sorted({step for step, _ in POLICY_SERVERS}), default=7500,
                        help="40merged checkpoint; selects the robot-s1 server port, verified by model id")
    parser.add_argument("--noise-seed", choices=("fixed", "per-request"), default="fixed",
                        help="fixed: deployed servers (seed 0 every inference); per-request: a new logged "
                             "seed per inference via the seedexp server")
    parser.add_argument("--actions-per-inference", type=int, choices=range(2, 11), default=STEPS_PER_INFERENCE,
                        help="full profile: leading actions executed from each fresh chunk (2-10); action k runs "
                             "at most 400 ms after its intended time, observation + k/30 s")
    parser.add_argument("--gripper-vote", choices=("tail", "chunk", "action"), default="tail",
                        help="full profile: one debounced vote per policy chunk, two consecutive chunks to switch; "
                             "tail = mean of its last {} predicted steps, chunk = whole-horizon mean; "
                             "action = legacy per-action votes".format(GRIPPER_TAIL_STEPS))
    parser.add_argument("--duration", type=float, default=5)
    parser.add_argument("--arm-timeout", type=float, default=120)
    parser.add_argument("--deadman", default="/dev/foot_brake")
    parser.add_argument("--gripper-port", default=GRIPPER_PORT)
    parser.add_argument("--norm-stats", default=str(Path(__file__).parent / "ros/norm_stats_40merged.json"))
    parser.add_argument("--log", required=True)
    parser.add_argument("--home-config", default=str(Path(__file__).resolve().parents[1] / "conf/full100b.conf"))
    parser.add_argument("--client-tracking-stop", choices=("on", "off"), default="on",
                        help="off (full profile, supervised diagnostics only): no governor or client "
                             "tracking-error stop; unscaled deltas on a lead leash; errors logged, crossings printed")
    parser.add_argument("--target-lead-cap-mm", type=float,
                        help="with --client-tracking-stop off: max target lead ahead of the measured position "
                             "(default 20); excess policy motion is dropped")
    parser.add_argument("--target-lead-cap-rad", type=float,
                        help="with --client-tracking-stop off: max target rotation lead (default 0.10)")
    args = parser.parse_args()
    maximum = 90 if args.profile == "full" else 10
    if not 0 < args.duration <= maximum:
        parser.error("duration exceeds selected profile limit: {} seconds".format(maximum))
    if args.actions_per_inference != STEPS_PER_INFERENCE and args.profile != "full":
        parser.error("--actions-per-inference is only available for the full profile")
    if args.client_tracking_stop == "off" and args.profile != "full":
        parser.error("--client-tracking-stop off is only available for the full profile")
    if args.client_tracking_stop == "on":
        if (args.target_lead_cap_mm, args.target_lead_cap_rad) != (None, None):
            parser.error("target lead caps apply only with --client-tracking-stop off")
    else:
        if args.target_lead_cap_mm is None:
            args.target_lead_cap_mm = 20.0
        if args.target_lead_cap_rad is None:
            args.target_lead_cap_rad = .10
        if not (0 < args.target_lead_cap_mm <= 45 and 0 < args.target_lead_cap_rad <= .30):
            parser.error("target lead caps must be within (0, 45] mm and (0, 0.30] rad")
    if (args.policy_step, args.noise_seed == "per-request") not in POLICY_SERVERS:
        parser.error("no server for --policy-step {} with --noise-seed {}".format(args.policy_step, args.noise_seed))
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
