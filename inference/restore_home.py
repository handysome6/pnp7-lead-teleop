"""Mandatory live-validation Home using the existing native joint-home routine.

ROS disconnect/connect releases exclusive FCI ownership without starting any
motion controller. Never recover robot errors or modify collision thresholds.
"""
import contextlib
import fcntl
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]


@contextlib.contextmanager
def validation_lock():
    # Keep the same inode, including after a crash; never unlink this lock.
    with (ROOT / "inference/ros/.validate.lock").open("a") as handle:
        fcntl.flock(handle, fcntl.LOCK_EX | fcntl.LOCK_NB)
        yield


def read_home_config(path):
    entries = {}
    for line in Path(path).read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if "=" in line:
            key, value = line.split("=", 1)
            if key.strip() in entries:
                raise RuntimeError("duplicate configuration key: " + key.strip())
            entries[key.strip()] = value.strip()
    q = [float(x) for x in entries["home_qpos"].split()]
    if len(q) != 7 or not all(math.isfinite(x) for x in q):
        raise RuntimeError("home_qpos must contain seven finite joint angles")
    return entries["robot_ip"], q


def require_idle(message):
    if message.robot_mode != message.ROBOT_MODE_IDLE:
        raise RuntimeError("Home requires idle robot; no automatic error recovery")
    for field in ("current_errors", "last_motion_errors"):
        errors = getattr(message, field)
        if any(getattr(errors, key) for key in errors.__slots__):
            raise RuntimeError("robot reports " + field)


def run_native_home(config):
    env = os.environ.copy()
    # The native teleop binary uses its own libfranka/Poco ABI, NOT ROS's ABI.
    env["LD_LIBRARY_PATH"] = str(ROOT / ".pixi/envs/default/lib") + ":" + str(
        ROOT / "DynamixelSDK/c++/build/linux64")
    command = [str(ROOT / "bin/pnp7_teleop"), "home", str(config)]
    child = subprocess.Popen(command, cwd=ROOT, env=env, start_new_session=True,
                             stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        output, _ = child.communicate(timeout=90)
    except BaseException:
        # Native Home decelerates on SIGINT. Never SIGKILL a robot controller.
        child.send_signal(signal.SIGINT)
        print("HOME_STOPPING: waiting for native smooth stop; physical stop if needed", flush=True)
        try:
            child.wait(timeout=10)
        except subprocess.TimeoutExpired:
            print("HOME_STOP_UNCONFIRMED: use physical stop; ROS remains disconnected", flush=True)
        raise
    print(output, end="", flush=True)
    if child.returncode or not any(line.startswith("HOME_OK") for line in output.splitlines()):
        raise RuntimeError("native Home failed; ROS remains disconnected; inference forbidden")


def restore_home(rospy, config):
    from controller_manager_msgs.srv import ListControllers
    from franka_msgs.msg import FrankaState
    from std_srvs.srv import Trigger

    config = Path(config).resolve(strict=True)
    robot_ip, target = read_home_config(config)
    if robot_ip != rospy.get_param("/franka_control/robot_ip"):
        raise RuntimeError("Home config and ROS refer to different robots")
    prefix = "/cartesian_impedance_controller/"
    if (not rospy.get_param(prefix + "policy_watchdog_available", False) or
            rospy.get_param(prefix + "policy_tracking_position_limit", 0) != .09 or
            rospy.get_param(prefix + "policy_tracking_rotation_limit", 0) != .60):
        raise RuntimeError("restart ROS with rebuilt 3x watchdog before validation")
    states = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)().controller
    if any(c.state == "running" and c.name != "franka_state_controller" for c in states):
        raise RuntimeError("stop all motion controllers before Home")
    topic = "/franka_state_controller/franka_states"
    before = rospy.wait_for_message(topic, FrankaState, timeout=3)
    require_idle(before)
    if not 0 <= (rospy.Time.now() - before.header.stamp).to_sec() < .1:
        raise RuntimeError("stale pre-Home robot state")
    print("HOME_BEGIN config={} target={} (pedal RELEASED; physical stop available)".format(
        config, target), flush=True)
    for name in ("disconnect", "connect"):
        rospy.wait_for_service("/franka_control/" + name, timeout=3)
    result = rospy.ServiceProxy("/franka_control/disconnect", Trigger)()
    if not result.success:
        raise RuntimeError("cannot release FCI for Home: " + result.message)
    # On any Home failure leave ROS disconnected, and never proceed to infer.
    run_native_home(config)
    reconnect_at = rospy.Time.now()
    result = rospy.ServiceProxy("/franka_control/connect", Trigger)()
    if not result.success:
        raise RuntimeError("Home finished but ROS reconnect failed: " + result.message)
    deadline = time.monotonic() + 5
    while True:
        after = rospy.wait_for_message(topic, FrankaState, timeout=3)
        if after.header.stamp > reconnect_at and (rospy.Time.now() - after.header.stamp).to_sec() < .1:
            break
        if time.monotonic() > deadline:
            raise RuntimeError("no fresh post-Home robot state")
    require_idle(after)
    if len(after.q) != 7 or not all(math.isfinite(x) for x in after.q):
        raise RuntimeError("invalid post-Home joint state")
    error = max(abs(a - b) for a, b in zip(after.q, target))
    if error > .01:
        raise RuntimeError("Home verification failed: max joint error {} rad".format(error))
    # Reconnection may create threads inheriting realtime priorities.
    subprocess.run([sys.executable, str(ROOT / "inference/ros/prepare_realtime.py")], check=True)
    report = dict(config=str(config), target_q=target, measured_q=list(after.q),
                  max_joint_error_rad=error)
    print("HOME_VERIFIED " + json.dumps(report), flush=True)
    return report
