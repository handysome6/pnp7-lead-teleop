"""Test double for `collect/record_cameras.py`, for machines with no RealSense.

Reproduces the interface the supervisor depends on: the `CAMERAS_READY` marker
after a settle, `cam_<role>/NNNNNN.jpg` plus `cam_<role>_index.csv` stamped with
`time.monotonic_ns()`, `cameras_meta.json`, and the `--preview-dir` JPEG tap.
`--no-write` holds "the cameras" and records nothing.

Frames are 64x48 and the rate is deliberately higher than the real 30 fps, so a
test reaches `validate_episode.py`'s 150-frame floor in a couple of seconds
rather than five. Nothing downstream cares: `build_episode.py` anchors on
whatever the index says, and the timestamps are real.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import signal
import time
from pathlib import Path

import cv2
import numpy as np

ROLES = ("external", "wrist")
WIDTH, HEIGHT = 64, 48
FPS = 100

_stop = False


def _on_signal(_sig, _frame):
    global _stop
    _stop = True


def frame_for(role: str, seq: int) -> np.ndarray:
    image = np.full((HEIGHT, WIDTH, 3), (seq * 3) % 200, np.uint8)
    cv2.putText(image, f"{role[:3]}{seq}", (2, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (255, 255, 255), 1, cv2.LINE_AA)
    return image


def main() -> int:
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None)
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--preview-dir", default=None)
    ap.add_argument("--no-write", action="store_true")
    # Accepted and ignored, so the stub takes the same command line.
    ap.add_argument("--width", type=int, default=WIDTH)
    ap.add_argument("--height", type=int, default=HEIGHT)
    ap.add_argument("--fps", type=int, default=FPS)
    ap.add_argument("--jpeg-quality", type=int, default=90)
    ap.add_argument("--preview", action="store_true")
    args = ap.parse_args()

    write = not args.no_write
    out = Path(args.out) if args.out else None
    preview_dir = Path(args.preview_dir) if args.preview_dir else None
    if preview_dir:
        preview_dir.mkdir(parents=True, exist_ok=True)

    writers = {}
    if write:
        for role in ROLES:
            (out / f"cam_{role}").mkdir(parents=True, exist_ok=True)
            fh = open(out / f"cam_{role}_index.csv", "w", newline="")
            w = csv.writer(fh)
            w.writerow(["seq", "host_monotonic_ns", "device_ts_ms", "file"])
            writers[role] = (fh, w)

    # The real recorder settles for a second before declaring itself ready.
    time.sleep(0.4)
    print("CAMERAS_READY", flush=True)

    t0 = time.monotonic()
    seq = 0
    period = 1.0 / FPS
    next_preview = 0.0
    while not _stop and (time.monotonic() - t0) < args.duration:
        loop_start = time.monotonic()
        for role in ROLES:
            image = frame_for(role, seq)
            if write:
                name = f"{seq:06d}.jpg"
                cv2.imwrite(str(out / f"cam_{role}" / name), image)
                writers[role][1].writerow(
                    [seq, time.monotonic_ns(), f"{seq * 10.0:.3f}", name])
            if preview_dir and loop_start >= next_preview:
                ok, buf = cv2.imencode(".jpg", image)
                if ok:
                    tmp = preview_dir / f".{role}.jpg.tmp"
                    tmp.write_bytes(buf.tobytes())
                    os.replace(tmp, preview_dir / f"{role}.jpg")
        if preview_dir and loop_start >= next_preview:
            next_preview = loop_start + 0.1
        seq += 1
        slack = period - (time.monotonic() - loop_start)
        if slack > 0:
            time.sleep(slack)

    for fh, _ in writers.values():
        fh.close()

    if write:
        elapsed = time.monotonic() - t0
        meta = {
            "started_monotonic_ns": 0, "ended_monotonic_ns": 0,
            "duration_s": elapsed, "width": WIDTH, "height": HEIGHT,
            "fps": FPS,
            "cameras": [{"role": r, "serial": f"stub-{r}", "frames": seq,
                         "dropped": 0, "mean_fps": round(seq / max(elapsed, 1e-6), 2),
                         "error": None} for r in ROLES],
        }
        (out / "cameras_meta.json").write_text(json.dumps(meta, indent=2))
        print(f"written to {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
