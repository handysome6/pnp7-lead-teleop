"""Observe camera/serial startup latency. Never Home, arm, or write FC16.

Run in a fresh ROS Pixi Python process; --staged warms cameras before serial.
"""
import argparse
import json
import threading
import time

from robot_s0_policy import Camera, EXTERNAL, WRIST, GRIPPER_PORT
from robotiq_policy import RobotiqPolicy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--staged", action="store_true")
    args = parser.parse_args()
    events, gaps = [], []
    started = time.monotonic()
    done = threading.Event()
    def event(name, **fields):
        events.append(dict(t=time.monotonic() - started, name=name, **fields))
    def heartbeat():
        last = time.monotonic()
        while not done.wait(.005):
            now = time.monotonic()
            if now - last > .04:
                gaps.append(dict(t=now - started, gap_ms=1000 * (now - last)))
            last = now
    class TracedGripper(RobotiqPolicy):
        def _exchange(self, payload, size):
            assert payload[1] == 4, "read-only diagnostic refuses output writes"
            t = time.monotonic()
            try:
                reply = super()._exchange(payload, size)
            except Exception as exc:
                event("serial_error", elapsed_ms=1000 * (time.monotonic() - t), error=repr(exc))
                raise
            elapsed = time.monotonic() - t
            if elapsed > .04:
                event("slow_serial", elapsed_ms=1000 * elapsed)
            return reply
    cameras = [Camera(EXTERNAL, "external"), Camera(WRIST, "wrist")]
    grip = None
    watcher = threading.Thread(target=heartbeat, daemon=True)
    watcher.start()
    try:
        if not args.staged:
            grip = TracedGripper(GRIPPER_PORT)
            event("serial_started")
        for camera in cameras:
            event("camera_start_begin", camera=camera.role)
            camera.start()
            event("camera_start_return", camera=camera.role)
        deadline = time.monotonic() + 15
        while True:
            try:
                for camera in cameras:
                    camera.latest()
                break
            except Exception:
                if time.monotonic() >= deadline:
                    raise
                time.sleep(.02)
        event("cameras_warm")
        if args.staged:
            grip = TracedGripper(GRIPPER_PORT)
            event("serial_started")
        done.wait(3)
        event("result", status=grip.status, error=repr(grip.error), commands=grip.commands)
    finally:
        if grip:
            grip.close()
        for camera in cameras:
            camera.stop()
        done.set()
        watcher.join(1)
        print(json.dumps(dict(staged=args.staged, events=events, python_thread_gaps=gaps), indent=2), flush=True)


if __name__ == "__main__":
    main()
