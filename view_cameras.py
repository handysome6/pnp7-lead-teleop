"""Show both RealSense colour streams on the robot PC's screen for inspection.

Opens the same devices, resolution and format the recorder uses, so what you
see here is what lands in an episode. Overlays the role, serial and live frame
rate on each pane.

Must run on the machine's own display:

  DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
    .venv/bin/python view_cameras.py

  q or ESC   quit
  s          save a still of both panes next to this script
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

DEFAULT_ROLES = {
    "213622078826": "external",
    "233622071437": "wrist",
}


class Stream:
    def __init__(self, serial, role, width, height, fps):
        self.serial, self.role = serial, role
        self.width, self.height = width, height
        self.pipeline = rs.pipeline()
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        self.pipeline.start(cfg)
        self.frames = 0
        self.t0 = time.monotonic()
        self.last = np.zeros((height, width, 3), np.uint8)

    def read(self):
        ok, frames = self.pipeline.try_wait_for_frames(200)
        if ok:
            c = frames.get_color_frame()
            if c:
                self.last = np.asanyarray(c.get_data())
                self.frames += 1
        return self.last

    @property
    def fps(self):
        dt = time.monotonic() - self.t0
        return self.frames / dt if dt > 0 else 0.0

    def stop(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass


STATE_NAMES = {0: "READY", 1: "TELEOP", 2: "PAUSED"}
STATE_COLOURS = {0: (140, 140, 140), 1: (60, 220, 60), 2: (40, 190, 240)}


def read_status(path):
    """Read the bridge's status file. It is written atomically via rename, so a
    partial read is not possible -- but the file is absent until teleop runs."""
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def status_panel(width, status):
    """Render the teleop state strip drawn beneath the camera panes."""
    h = 150
    panel = np.zeros((h, width, 3), np.uint8)
    panel[:] = (24, 24, 24)

    if status is None:
        cv2.putText(panel, "teleop not running", (16, 46),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (120, 120, 120), 2,
                    cv2.LINE_AA)
        cv2.putText(panel, "start pnp7_teleop to see live state", (16, 86),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (90, 90, 90), 1, cv2.LINE_AA)
        return panel

    state = int(status.get("state", 0))
    engaged = bool(status.get("deadman", 0))
    colour = STATE_COLOURS.get(state, (140, 140, 140))
    name = STATE_NAMES.get(state, "?")

    cv2.rectangle(panel, (0, 0), (width, 6), colour, -1)
    cv2.putText(panel, name, (16, 52), cv2.FONT_HERSHEY_SIMPLEX, 1.3,
                colour, 3, cv2.LINE_AA)
    cv2.putText(panel, "DEAD-MAN HELD" if engaged else "dead-man released",
                (230, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.75,
                (60, 220, 60) if engaged else (90, 90, 90), 2, cv2.LINE_AA)

    age = float(status.get("lead_age_ms", 0.0))
    cv2.putText(panel,
                f"lead {status.get('lead_seq', 0)}  age {age:.1f} ms",
                (width - 300, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (200, 200, 200) if age < 50 else (40, 40, 240), 1, cv2.LINE_AA)

    q = status.get("q", [])
    if q:
        txt = "  ".join(f"J{i+1} {math.degrees(v):+6.1f}"
                        for i, v in enumerate(q))
        cv2.putText(panel, txt, (16, 92), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (210, 210, 210), 1, cv2.LINE_AA)

    gw = float(status.get("gripper_width", -1.0))
    if gw >= 0:
        bar_x, bar_y, bar_w = 16, 112, 260
        cv2.rectangle(panel, (bar_x, bar_y), (bar_x + bar_w, bar_y + 22),
                      (70, 70, 70), 1)
        filled = int(bar_w * min(max(gw / 0.08, 0.0), 1.0))
        cv2.rectangle(panel, (bar_x + 1, bar_y + 1),
                      (bar_x + filled, bar_y + 21), (200, 160, 40), -1)
        cv2.putText(panel, f"gripper {gw*1000:5.1f} mm",
                    (bar_x + bar_w + 14, bar_y + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (210, 210, 210), 1,
                    cv2.LINE_AA)
    return panel


def label(img, lines):
    out = img.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 26 * len(lines) + 8), (0, 0, 0), -1)
    for i, text in enumerate(lines):
        cv2.putText(out, text, (10, 24 + i * 26), cv2.FONT_HERSHEY_SIMPLEX,
                    0.7, (0, 255, 0), 2, cv2.LINE_AA)
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="auto-quit after this long (0 = run until keypress)")
    ap.add_argument("--status", default="",
                    help="teleop status file written by pnp7_teleop; when "
                         "given, live state is drawn under the camera panes")
    args = ap.parse_args()

    devices = list(rs.context().query_devices())
    if not devices:
        print("no RealSense cameras found")
        return 1

    streams = []
    for d in devices:
        serial = d.get_info(rs.camera_info.serial_number)
        role = DEFAULT_ROLES.get(serial, f"cam{serial[-4:]}")
        streams.append(Stream(serial, role, args.width, args.height, args.fps))
        print(f"opened {role:<10} {serial}")

    # external on the left when both are present, so the layout is predictable
    streams.sort(key=lambda s: s.role != "external")

    window = "PNP7 cameras  --  q to quit, s to save"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, args.width * len(streams),
                     args.height + (150 if args.status else 0))

    t0 = time.monotonic()
    saved = 0
    while True:
        panes = []
        for s in streams:
            img = s.read()
            panes.append(label(img, [
                f"{s.role}  {s.serial}",
                f"{img.shape[1]}x{img.shape[0]}  {s.fps:5.1f} fps  "
                f"n={s.frames}",
            ]))
        frame = np.hstack(panes)
        if args.status:
            frame = np.vstack([
                frame, status_panel(frame.shape[1], read_status(args.status))])
        cv2.imshow(window, frame)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            saved += 1
            for s, pane in zip(streams, panes):
                path = Path(__file__).parent / f"view_{s.role}_{saved:02d}.png"
                cv2.imwrite(str(path), pane)
                print(f"saved {path}")
        if args.seconds and time.monotonic() - t0 > args.seconds:
            break

    for s in streams:
        s.stop()
    cv2.destroyAllWindows()
    print("\n" + "\n".join(
        f"{s.role:<10} {s.frames} frames, {s.fps:.1f} fps" for s in streams))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
