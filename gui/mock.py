"""A backend that runs anywhere, so the UI can be built without the rig.

None of the real imports resolve on a laptop -- pyrealsense2, franky, evdev and
ray are all Linux-and-hardware -- and the robot PC is not a place to iterate on
CSS. This stands in for all of it: two synthetic camera streams, a seven-joint
arm that drifts plausibly, and an F3 pedal the browser can toggle.

It is not a simulator. It exists to exercise the state machine, the command
plumbing and the MJPEG path, and to make the F3-clipping arithmetic testable
without a foot pedal.
"""
from __future__ import annotations

import math
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from gui.session import Backend, Mode

#: Roughly the Franka start configuration the rig calibrates against.
HOME_QPOS = [0.0, -0.4, 0.0, -2.0, 0.0, 1.6, 0.785]


class MockBackend(Backend):
    def __init__(self, config_dir: str | Path | None = None,
                 width: int = 640, height: int = 480):
        self.config_dir = Path(config_dir) if config_dir else None
        self.width, self.height = width, height
        self.camera_names = []
        self._open = False
        self._preview = False
        self._t0 = time.monotonic()
        self._qpos = list(HOME_QPOS)
        self._gripper = 0.08
        self._frame_no = 0

        # Toggled from the browser so the pedal-gated paths are reachable
        # without a pedal. The real backend reads this from the env info.
        self.deadman_held = False

        self._episode_open = False
        self._steps = 0
        self._skipped = 0
        self._segments = 0
        self._was_capturing = False
        self._episodes_kept = 0
        self._dataset_dir: str | None = None

    # --- lifecycle --------------------------------------------------------
    def open_preview(self) -> None:
        self._preview = True
        self.camera_names = ["external", "wrist"]

    def close_preview(self) -> None:
        self._preview = False
        if not self._open:
            self.camera_names = []

    def open_session(self, config_name: str, overrides: dict[str, Any],
                     mode: Mode) -> None:
        # The real backend spends several seconds here building the env and
        # running the GELLO alignment check; make that visible in the UI.
        time.sleep(1.0)
        self._open = True
        self.camera_names = ["external", "wrist"]
        self._t0 = time.monotonic()
        self._episodes_kept = 0
        self._dataset_dir = str(
            overrides.get("save_dir")
            or f"/tmp/mock_collect/{config_name}/collected_data")

    def close_session(self) -> None:
        self._open = False
        self._episode_open = False
        self.camera_names = []

    # --- per-step ---------------------------------------------------------
    def step(self, recording: bool) -> dict[str, Any]:
        self._frame_no += 1
        t = time.monotonic() - self._t0

        # Only drift while the pedal is held, so a still frame in the preview
        # means the same thing it means on the rig.
        if self.deadman_held:
            for j in range(7):
                self._qpos[j] = HOME_QPOS[j] + 0.25 * math.sin(0.6 * t + j)
            self._gripper = 0.04 + 0.04 * (0.5 + 0.5 * math.sin(1.3 * t))

        capturing = recording and self.deadman_held
        if self._episode_open:
            if capturing:
                self._steps += 1
                if not self._was_capturing:
                    self._segments += 1
            else:
                self._skipped += 1
        self._was_capturing = capturing

        return {
            "deadman_held": self.deadman_held,
            "stream_enabled": self.deadman_held,
            "block_reason": None if self.deadman_held else "deadman_released",
            "alignment_error": 0.012,
            "joints": list(self._qpos),
            "gripper_width": self._gripper,
            "episode_steps": self._steps,
            "episode_segments": self._segments,
            "recorded_steps": self._steps,
            "skipped_steps": self._skipped,
            "dataset_dir": self._dataset_dir,
        }

    # --- operator actions -------------------------------------------------
    def restore_joints(self, qpos: list[float] | None = None) -> None:
        if self.deadman_held:
            raise RuntimeError("release F3 before restoring the arm")
        time.sleep(1.2)
        self._qpos = list(qpos or HOME_QPOS)

    def begin_episode(self) -> None:
        self._episode_open = True
        self._steps = 0
        self._skipped = 0
        self._segments = 0
        self._was_capturing = False

    def end_episode(self, keep: bool) -> dict[str, Any]:
        self._episode_open = False
        result = {"steps": self._steps, "segments": self._segments,
                  "skipped": self._skipped}
        if keep and self._steps > 0:
            self._episodes_kept += 1
            result["kept"] = True
        else:
            result["kept"] = False
            result["reason"] = "no F3-held frames" if keep else "discarded"
        return result

    def finalize_dataset(self) -> dict[str, Any]:
        return {"dataset_dir": self._dataset_dir, "episodes": self._episodes_kept}

    # --- preview ----------------------------------------------------------
    def latest_frame(self, camera: str) -> np.ndarray | None:
        if camera not in self.camera_names:
            return None
        return self._render(camera)

    def _render(self, camera: str) -> np.ndarray:
        """A synthetic scene that moves, so a frozen stream is obvious."""
        h, w = self.height, self.width
        img = np.zeros((h, w, 3), np.uint8)
        base = (34, 28, 24) if camera == "external" else (24, 30, 36)
        img[:] = base

        t = time.monotonic() - self._t0
        # A drifting checker so dropped frames read as a stutter, not a still.
        step = 64
        for y in range(0, h, step):
            for x in range(0, w, step):
                if ((x // step) + (y // step) + int(t * 2)) % 2:
                    img[y:y + step, x:x + step] = tuple(c + 14 for c in base)

        cx = int(w / 2 + (w / 3) * math.sin(0.8 * t + (0 if camera == "external" else 1.4)))
        cy = int(h / 2 + (h / 4) * math.cos(0.5 * t))
        cv2.circle(img, (cx, cy), 42, (60, 170, 240), -1, cv2.LINE_AA)
        cv2.circle(img, (cx, cy), 42, (255, 255, 255), 2, cv2.LINE_AA)

        label = f"MOCK {camera}  f{self._frame_no}  {t:6.1f}s"
        cv2.rectangle(img, (0, 0), (w, 30), (0, 0, 0), -1)
        cv2.putText(img, label, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (220, 220, 220), 1, cv2.LINE_AA)
        if self.deadman_held:
            cv2.rectangle(img, (2, 2), (w - 3, h - 3), (40, 220, 90), 4)
        return img

    # --- configs ----------------------------------------------------------
    def list_configs(self) -> list[dict[str, Any]]:
        if self.config_dir and self.config_dir.is_dir():
            return [{"name": p.stem, "path": str(p)}
                    for p in sorted(self.config_dir.glob("*.yaml"))]
        return [{"name": "realworld_collect_data_gello_franky", "path": "(mock)"}]
