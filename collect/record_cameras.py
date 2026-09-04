"""Record both RealSense colour streams for a teleoperation episode.

Runs alongside pnp7_teleop. Both processes stamp with time.monotonic_ns()
(CLOCK_MONOTONIC), the same clock the C++ bridge uses, so frames and robot
state can be joined afterwards by build_episode.py without any clock fitting.

Each camera gets its own thread, because a blocked frame on one must not stall
the other. Frames are encoded to JPEG on the writer thread and indexed in a CSV.

  python record_cameras.py --duration 60 --out episodes/ep001

--preview-dir publishes the latest frame per camera as a JPEG, so a supervisor
in another process can show the scene without opening the devices itself. Every
RealSense pipeline is exclusive -- view_cameras.py and this script cannot both
have a camera -- so tapping the frames already captured is the only way for
anything else to see them. Files are written to a temporary name and renamed
into place, the same trick StatusPublisher uses in the C++ bridge, so a reader
never gets half a JPEG.

--no-write skips the episode output entirely, for framing the scene between
takes. Combined with --preview-dir it makes this a viewfinder that holds the
cameras open and writes nothing.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import queue
import signal
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

# Set by --preview. The display loop runs on the main thread because cv2
# windows must; workers only publish their latest frame.
PREVIEW = False

# Set by --preview-dir. Publishing happens on the main thread too, for the same
# reason: the camera threads must never block on an encode or a disk write.
PREVIEW_DIR: Path | None = None

# Verified roles from camera_roles.json in ~/workspace/andyls.
DEFAULT_ROLES = {
    "213622078826": "external",
    "233622071437": "wrist",
}

g_stop = threading.Event()


class CameraWorker(threading.Thread):
    def __init__(self, serial, role, out_dir, width, height, fps, jpeg_quality,
                 write=True):
        super().__init__(name=f"cam-{role}", daemon=True)
        self.serial = serial
        self.role = role
        self.write = write
        self.dir = Path(out_dir) / f"cam_{role}" if out_dir else None
        if self.write:
            self.dir.mkdir(parents=True, exist_ok=True)
        self.width, self.height, self.fps = width, height, fps
        self.jpeg_quality = jpeg_quality
        self.index_path = (Path(out_dir) / f"cam_{role}_index.csv"
                           if out_dir else None)
        self.frames = 0
        self.dropped = 0
        self.error: str | None = None
        self.preview = None
        self._queue: queue.Queue = queue.Queue(maxsize=240)

    def _writer(self):
        with open(self.index_path, "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(["seq", "host_monotonic_ns", "device_ts_ms", "file"])
            while True:
                item = self._queue.get()
                if item is None:
                    break
                seq, host_ns, dev_ts, image = item
                name = f"{seq:06d}.jpg"
                cv2.imwrite(
                    str(self.dir / name), image,
                    [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality])
                writer.writerow([seq, host_ns, f"{dev_ts:.3f}", name])

    def run(self):
        writer_thread = None
        if self.write:
            writer_thread = threading.Thread(target=self._writer, daemon=True)
            writer_thread.start()
        pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.serial)
        config.enable_stream(rs.stream.color, self.width, self.height,
                             rs.format.bgr8, self.fps)
        try:
            pipeline.start(config)
            while not g_stop.is_set():
                ok, frames = pipeline.try_wait_for_frames(1000)
                if not ok:
                    continue
                frame = frames.get_color_frame()
                if not frame:
                    continue
                host_ns = time.monotonic_ns()
                image = np.asanyarray(frame.get_data())
                if PREVIEW or PREVIEW_DIR is not None:
                    # Plain reference assignment; the reader only ever needs
                    # the most recent frame, never a consistent series.
                    self.preview = image
                if self.write:
                    try:
                        self._queue.put_nowait(
                            (self.frames, host_ns, frame.get_timestamp(), image))
                    except queue.Full:
                        # Never block the camera thread; record the loss instead.
                        self.dropped += 1
                self.frames += 1
        except Exception as exc:
            self.error = str(exc)
        finally:
            try:
                pipeline.stop()
            except Exception:
                pass
            if writer_thread is not None:
                self._queue.put(None)
                writer_thread.join(timeout=10)


def publish_previews(workers, directory: Path, quality: int) -> None:
    """Write the latest frame per camera as `<role>.jpg`, atomically.

    Encoding here rather than on the camera threads keeps a slow disk from
    ever costing a frame. Renaming into place means a reader polling the
    directory cannot catch a half-written file.
    """
    for worker in workers:
        image = worker.preview
        if image is None:
            continue
        ok, buf = cv2.imencode(".jpg", image,
                               [int(cv2.IMWRITE_JPEG_QUALITY), quality])
        if not ok:
            continue
        final = directory / f"{worker.role}.jpg"
        tmp = directory / f".{worker.role}.jpg.tmp"
        try:
            tmp.write_bytes(buf.tobytes())
            os.replace(tmp, final)
        except OSError:
            # A preview is never worth interrupting a take for.
            pass


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=None,
                    help="episode directory. Required unless --no-write.")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--jpeg-quality", type=int, default=90)
    ap.add_argument("--preview", action="store_true",
                    help="show the frames being recorded, so the operator can "
                         "see if the scene leaves frame mid-episode. Displays "
                         "the frames already captured -- no extra camera load.")
    ap.add_argument("--preview-dir", default=None,
                    help="publish the latest frame per camera as <role>.jpg "
                         "here, ~10 Hz, for a supervisor in another process. "
                         "Also taps frames already captured.")
    ap.add_argument("--no-write", action="store_true",
                    help="hold the cameras open but record nothing. For "
                         "framing the scene between takes.")
    args = ap.parse_args()

    if not args.no_write and not args.out:
        ap.error("--out is required unless --no-write is given")

    global PREVIEW, PREVIEW_DIR
    PREVIEW = args.preview
    if args.preview_dir:
        PREVIEW_DIR = Path(args.preview_dir)
        PREVIEW_DIR.mkdir(parents=True, exist_ok=True)

    out = Path(args.out) if args.out else None
    if out is not None and not args.no_write:
        out.mkdir(parents=True, exist_ok=True)

    devices = list(rs.context().query_devices())
    if not devices:
        print("no RealSense cameras found")
        return 1

    workers = []
    for dev in devices:
        serial = dev.get_info(rs.camera_info.serial_number)
        role = DEFAULT_ROLES.get(serial, f"cam{serial[-4:]}")
        workers.append(CameraWorker(serial, role, out, args.width, args.height,
                                    args.fps, args.jpeg_quality,
                                    write=not args.no_write))

    def on_sigint(_s, _f):
        g_stop.set()

    signal.signal(signal.SIGINT, on_sigint)

    verb = "previewing" if args.no_write else "recording"
    print(f"{verb} {len(workers)} camera(s) for {args.duration:.0f}s "
          f"at {args.width}x{args.height}@{args.fps}")
    for w in workers:
        print(f"  {w.role:<10} {w.serial}")

    t0 = time.monotonic_ns()
    for w in workers:
        w.start()

    # CAMERAS_READY is the cue that streams are live; start the bridge after it.
    time.sleep(1.0)
    print("CAMERAS_READY", flush=True)

    # External first, so a horizontal stack and a preview directory agree on
    # which camera is which.
    order = sorted(workers, key=lambda w: w.role != "external")
    if args.preview:
        window = "recording -- external | wrist"
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window, args.width * len(workers), args.height)

    last_publish = 0.0
    while not g_stop.is_set() and (time.monotonic_ns() - t0) / 1e9 < args.duration:
        if PREVIEW_DIR is not None:
            now = time.monotonic()
            if now - last_publish >= 0.1:
                publish_previews(order, PREVIEW_DIR, args.jpeg_quality)
                last_publish = now
        if args.preview:
            panes = []
            for w in order:
                img = w.preview
                if img is None:
                    img = np.zeros((args.height, args.width, 3), np.uint8)
                pane = img.copy()
                elapsed = (time.monotonic_ns() - t0) / 1e9
                cv2.rectangle(pane, (0, 0), (pane.shape[1], 34), (0, 0, 0), -1)
                cv2.putText(pane, f"REC {w.role}  {w.frames} frames  "
                            f"{elapsed:5.1f}/{args.duration:.0f}s",
                            (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.62,
                            (40, 40, 240), 2, cv2.LINE_AA)
                panes.append(pane)
            cv2.imshow(window, np.hstack(panes))
            if (cv2.waitKey(20) & 0xFF) in (ord("q"), 27):
                break
        else:
            time.sleep(0.05)
    g_stop.set()
    if args.preview:
        cv2.destroyAllWindows()
    for w in workers:
        w.join(timeout=15)

    t1 = time.monotonic_ns()
    meta = {
        "started_monotonic_ns": t0,
        "ended_monotonic_ns": t1,
        "duration_s": (t1 - t0) / 1e9,
        "width": args.width,
        "height": args.height,
        "fps": args.fps,
        "cameras": [],
    }
    ok = True
    for w in workers:
        rate = w.frames / max((t1 - t0) / 1e9, 1e-6)
        meta["cameras"].append({
            "role": w.role, "serial": w.serial, "frames": w.frames,
            "dropped": w.dropped, "mean_fps": round(rate, 2),
            "error": w.error,
        })
        status = "ok" if w.error is None and w.dropped == 0 else "CHECK"
        print(f"  {w.role:<10} {w.frames:5d} frames  {rate:5.1f} fps  "
              f"dropped={w.dropped}  {status}"
              + (f"  error={w.error}" if w.error else ""))
        if w.error or (w.dropped and not args.no_write):
            ok = False

    if args.no_write:
        return 0 if ok else 1

    with open(out / "cameras_meta.json", "w") as fh:
        json.dump(meta, fh, indent=2)
    print(f"\nwritten to {out}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
