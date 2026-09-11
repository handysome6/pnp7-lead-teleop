"""The C++ path: supervise the bridge and the camera recorder as subprocesses.

Unlike the RLinf stack, everything here is *already* a separate process, so
supervision is the natural fit rather than a workaround. `bin/pnp7_teleop` is a
binary and `collect/record_cameras.py` is a script; this module runs them in the
order `scripts/collect_episode.sh` established, and then owns the cleanup that
the shell script does not.

Three properties of the bridge shape every decision below, and all three are
easy to get wrong:

* **The log lives in RAM until the process exits.** `runRobot` allocates
  `rows(duration_s * 1100)` up front and calls `writeLog` exactly once, at the
  end. `SIGKILL` therefore destroys the entire take. Stopping is always SIGINT,
  which decelerates the arm to rest, writes the CSV, and exits 0.
* **Exit code 0 means nothing.** A clean SIGINT and a completed run are
  indistinguishable by status, so completion is judged by row count.
* **stdout is useless as a channel.** The bridge never flushes, so over a pipe
  it is 4 KB block-buffered -- `CONTROL_READY` may never arrive. The real
  telemetry is `status_path`: a JSON file rewritten at 10 Hz via
  write-tmp-then-rename, deleted on clean exit. Its appearance is the readiness
  signal and its disappearance is the crash signal.

Clipping is deliberately absent from this file. `deadman` is a column in
`teleop.csv` and `build_episode.py` drops the released rows, so what gets kept
stays a pure function of data already on disk -- re-runnable with different
settings long after the robot has been switched off.
"""
from __future__ import annotations

import json
import re
from contextlib import contextmanager
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

from gui.session import Backend, EpisodeReflex, Mode, Rejected
from gui.home_config import save_current_home
from gui.episode_catalog import progress, reserve

REPO = Path(__file__).resolve().parent.parent
# Recorder and post-processing must use the environment running the GUI.
# This works for both pixi on robot-s0 and the old controller's .venv.
PYTHON = Path(sys.executable)
BRIDGE = REPO / "bin" / "pnp7_teleop"
RECORDER = REPO / "collect" / "record_cameras.py"

#: How stale the bridge's status file may get before we call it dead. It is
#: rewritten every 100 ms, so a second is ten missed publishes.
STATUS_STALE_S = 1.0

#: The camera recorder must outlive the bridge at both ends, or `build_episode`
#: has no contemporaneous frames at the edges. `collect_episode.sh` uses +6.
CAMERA_OVERHANG_S = 6

#: Below this the bridge barely ran; `collect_episode.sh` refuses to build.
MIN_TELEOP_ROWS = 1000


class _Proc:
    """A supervised child process. Never SIGKILLed unless it ignores SIGINT."""

    def __init__(self, name: str, argv: list[str], log_path: Path | None = None,
                 cwd: Path = REPO):
        self.name = name
        self.argv = [str(a) for a in argv]
        self.log_path = log_path
        self._log = open(log_path, "w") if log_path else subprocess.DEVNULL
        self.proc = subprocess.Popen(
            self.argv, cwd=str(cwd),
            stdout=self._log if log_path else subprocess.DEVNULL,
            stderr=subprocess.STDOUT if log_path else subprocess.DEVNULL,
        )

    @property
    def alive(self) -> bool:
        return self.proc.poll() is None

    def stop(self, timeout: float = 30.0) -> int | None:
        """SIGINT, then wait. SIGKILL only as a last resort.

        The bridge writes its whole log during this window, so the timeout is
        generous: a 60 s take is ~50 MB of CSV formatted with setprecision(10).
        """
        if self.proc.poll() is not None:
            return self.proc.returncode
        self.proc.send_signal(signal.SIGINT)
        try:
            self.proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            # Only reachable if the bridge is wedged inside robot.control().
            # The log is lost either way at this point.
            self.proc.kill()
            self.proc.wait(timeout=5)
        return self.proc.returncode

    def close(self) -> None:
        if self.log_path and self._log is not subprocess.DEVNULL:
            try:
                self._log.close()
            except OSError:
                pass


class LegacyBackend(Backend):
    """Drives one collection session over the C++ bridge.

    Args:
        episodes_dir: Where episode directories are created.
        preview_dir: Scratch directory for the camera recorder's JPEG tap.
        status_path: Where the bridge publishes its 10 Hz status JSON.
    """

    def __init__(self, episodes_dir: str | Path = "episodes",
                 preview_dir: str | Path = "/tmp/pnp7_preview",
                 status_path: str | Path = "/tmp/pnp7_gui_status.json",
                 bridge: str | Path = BRIDGE,
                 python: str | Path = PYTHON,
                 recorder: str | Path = RECORDER,
                 conf_dir: str | Path | None = None,
                 scratch_dir: str | Path = "/tmp"):
        self.episodes_dir = Path(episodes_dir)
        self.preview_dir = Path(preview_dir)
        self.status_path = Path(status_path)
        # Overridable so the orchestration can be exercised against stand-in
        # binaries. Every default is the real thing.
        self.bridge = Path(bridge)
        self.python = Path(python)
        self.recorder = Path(recorder)
        self.conf_dir = Path(conf_dir) if conf_dir else REPO / "conf"
        self.scratch_dir = Path(scratch_dir)

        self.camera_names: list[str] = []
        self._viewfinder: _Proc | None = None
        self._recorder: _Proc | None = None
        self._bridge: _Proc | None = None

        self._config_path: Path | None = None
        self._mode = Mode.COLLECT
        self._dry_run = False
        self._duration = 60.0
        self._prefix = "ep"
        self._episode_dir: Path | None = None
        self._episode_started = 0.0

        # Frames captured with F3 held, counted from status edges. The bridge
        # reports no such count; `episode_meta.json` corrects this to the exact
        # number once `build_episode.py` has run.
        self._held_samples = 0
        self._total_samples = 0
        self._segments = 0
        self._was_held = False
        self._last_status: dict[str, Any] = {}
        self._last_report: dict[str, Any] = {}

    # --- lifecycle --------------------------------------------------------
    def open_preview(self) -> None:
        """Hold the cameras open and write nothing -- a viewfinder.

        Available here in a way it is not on the RLinf path, because the
        recorder is a separate process that can be told not to record.
        """
        if self._viewfinder is not None or self._recorder is not None:
            return
        self.preview_dir.mkdir(parents=True, exist_ok=True)
        self._clear_previews()
        self._viewfinder = _Proc(
            "viewfinder",
            [self.python, self.recorder,
             "--no-write", "--preview-dir", self.preview_dir,
             # Long enough to outlast any sitting; the GUI stops it explicitly.
             "--duration", 86400],
            log_path=self.scratch_dir / "pnp7_viewfinder.log",
        )
        self._await_cameras(self.scratch_dir / "pnp7_viewfinder.log",
                            timeout=30)
        self.camera_names = self._discover_cameras()

    def close_preview(self) -> None:
        if self._viewfinder is not None:
            self._viewfinder.stop(timeout=15)
            self._viewfinder.close()
            self._viewfinder = None
        if self._recorder is None:
            self.camera_names = []

    def open_session(self, config_name: str, overrides: dict[str, Any],
                     mode: Mode) -> None:
        config = self.conf_dir / f"{config_name}.conf"
        if not config.is_file():
            raise RuntimeError(f"no such config: {config}")
        if not self.bridge.is_file():
            raise RuntimeError(
                f"{self.bridge} is missing -- run ./build.sh on the robot PC")

        self._config_path = config
        self._mode = mode
        self._dry_run = bool(overrides.get("dry_run", False))
        self._duration = float(overrides.get("duration", 60))
        self._prefix = str(overrides.get("prefix", "ep"))
        if overrides.get("episodes_dir"):
            self.episodes_dir = Path(overrides["episodes_dir"])
        self.episodes_dir.mkdir(parents=True, exist_ok=True)
        self.dataset_progress()  # Validate prefix and persisted sequence before hardware access.

        self.open_preview()

        if mode is Mode.TELEOP:
            # Teleop needs the bridge running the whole time, but the bridge
            # buffers its log in RAM, so the sitting is bounded rather than
            # open-ended: 600 s is ~340 MB. No log path is passed, so nothing
            # is written -- `writeLog` returns immediately on an empty path.
            self._duration = float(overrides.get("teleop_duration", 600))
            self._start_bridge(log_csv=None)

    def close_session(self) -> None:
        self._stop_bridge()
        self._stop_recorder()
        self.close_preview()
        self._config_path = None
        self._episode_dir = None

    def abort_session(self, reason: str) -> None:
        """Keep a failed take's raw files and error log for diagnosis."""
        try:
            if self._episode_dir is not None:
                (self._episode_dir / "failure.json").write_text(json.dumps(
                    {"error": reason, "time": time.time(), "usable_episode": False},
                    ensure_ascii=False, indent=2) + "\n")
        finally:
            self.close_session()

    # --- per-step ---------------------------------------------------------
    def step(self, recording: bool) -> dict[str, Any]:
        """Poll the bridge's status file and the children's health.

        Nothing is commanded here. The bridge owns the 1 kHz control loop on
        its own; this is a 10 Hz observer that happens to match the status
        publisher's rate.
        """
        telemetry: dict[str, Any] = {"cameras": list(self.camera_names)}
        # A fresh status file can survive a crash. Check the process before
        # trusting it, and never classify a nonzero exit as a duration limit.
        bridge_exit = self._bridge.proc.poll() if self._bridge is not None else None
        failure = self._bridge_failure(self._bridge, recording=recording)
        if failure is not None:
            raise failure
        status = self._read_status()
        if bridge_exit is not None:
            status = None

        if status is not None:
            self._last_status = status
            held = bool(status.get("deadman"))
            telemetry.update(
                deadman_held=held,
                stream_enabled=held and status.get("state") == 1,
                block_reason=(None if held and status.get("state") == 1 else
                              "控制已暂停：等待 GELLO 新数据或会话结束" if held else
                              "脚刹已松开，或启动后尚未先松开脚刹"),
                joints=list(status.get("q") or []),
                lead_age_ms=self._nonnegative(status.get("lead_age_ms")),
                gripper_width=self._nonnegative(status.get("gripper_width")),
                gripper_position_raw=self._nonnegative(status.get("gripper_position_raw")),
                gripper_requested_raw=self._nonnegative(status.get("gripper_requested_raw")),
                gripper_fault=self._nonnegative(status.get("gripper_fault")),
            )
            if recording:
                self._total_samples += 1
                if held:
                    self._held_samples += 1
                    if not self._was_held:
                        self._segments += 1
                self._was_held = held
        else:
            telemetry.update(deadman_held=False, stream_enabled=False)
            if self._bridge is not None and not self._bridge.alive:
                # The bridge stops itself once it reaches the duration it was
                # given. That is a take running to its natural end, not a
                # fault, so ask the control loop to close the episode rather
                # than raising -- raising would discard a perfectly good take.
                telemetry["block_reason"] = "控制程序正常结束（到时或收到停止信号）"
                telemetry["message"] = "控制程序已结束；继续操作前请关闭并重新打开会话"
                if recording:
                    telemetry["episode_finished"] = True
            elif self._bridge is not None:
                telemetry["block_reason"] = "控制程序仍在运行，但状态更新缺失或已超过 1 秒"
            else:
                telemetry["block_reason"] = "bridge not running"

        if recording:
            # The status file is sampled at 10 Hz, so these are seconds of
            # held pedal rather than frames. `episode_meta.json` replaces them
            # with the true frame count once the episode is built.
            telemetry.update(
                episode_steps=self._held_samples,
                episode_segments=self._segments,
                recorded_steps=self._held_samples,
                skipped_steps=self._total_samples - self._held_samples,
            )

        self._check_children()
        telemetry["dataset_dir"] = str(self.episodes_dir)
        return telemetry

    # --- operator actions -------------------------------------------------
    @contextmanager
    def _home_operation(self):
        """Yield the FCI to Home, keeping the teleop session/cameras open.

        Resume after success or an operator refusal, never after a fault. The
        new bridge captures a new clutch origin and requires release if F3
        was pressed during the handover.
        """
        if self._config_path is not None and self._dry_run:
            raise Rejected("dry run 会话不能读取或恢复实际机器人 Home")
        resume = self._mode is Mode.TELEOP and self._config_path is not None
        bridge = self._bridge
        if resume:
            if bridge is None or not bridge.alive:
                raise RuntimeError("遥操控制程序已停止，请关闭会话后重新打开")
            status = self._read_status()
            if (status is None or status.get("deadman") != 0
                    or status.get("state") not in (0, 2)):
                raise Rejected("请松开 F3，并等待遥操暂停状态更新后再操作 Home")
            self._stop_bridge()
            if bridge.proc.poll() != 0:
                raise RuntimeError("暂停遥操失败，未执行 Home 操作：\n" +
                                   self._tail(bridge.log_path, lines=12))
        elif bridge is not None and bridge.alive:
            raise Rejected("请先结束录制，再操作 Home")
        try:
            yield
        except Rejected:
            if resume:
                self._start_bridge(log_csv=None)
            raise
        else:
            if resume:
                self._start_bridge(log_csv=None)

    def save_home(self, config_name: str) -> dict[str, Any]:
        if self._config_path is not None and self._dry_run:
            raise Rejected("请先关闭 dry run 会话，再读取实际机器人姿态")
        if not isinstance(config_name, str) or not config_name or Path(config_name).name != config_name:
            raise Rejected("请选择有效的配置文件")
        config = self.conf_dir / f"{config_name}.conf"
        if (config.is_symlink() or config.resolve().parent != self.conf_dir.resolve()
                or not config.is_file()):
            raise Rejected("配置必须是 conf 目录中的普通 .conf 文件")
        if self._config_path is not None and config.resolve() != self._config_path.resolve():
            raise Rejected("只能保存到当前会话使用的配置")

        def read_q() -> list[float]:
            try:
                result = subprocess.run(
                    [str(self.bridge), "read-home", str(config)], cwd=str(REPO),
                    capture_output=True, text=True, timeout=15,
                )
            except subprocess.TimeoutExpired as exc:
                raise Rejected("读取机器人姿态超时，配置未修改") from exc
            if result.returncode != 0:
                detail = (result.stderr or result.stdout or "FCI read failed").strip()
                raise Rejected("读取机器人姿态失败，配置未修改：" + detail.splitlines()[-1])
            records = [line.removeprefix("HOME_QPOS ") for line in result.stdout.splitlines()
                       if line.startswith("HOME_QPOS ")]
            if len(records) != 1:
                raise Rejected("未收到机器人姿态，配置未修改；请重新编译 bridge")
            try:
                return json.loads(records[0])
            except json.JSONDecodeError as exc:
                raise Rejected("机器人姿态数据格式错误，配置未修改") from exc

        with self._home_operation():
            return save_current_home(config, read_q)

    def restore_joints(self, qpos: list[float] | None = None) -> None:
        """Temporarily yield the FCI and restore the selected config's Home."""
        if self._config_path is None:
            raise Rejected("no session is open")
        with self._home_operation():
            self._restore_home()

    def _restore_home(self) -> None:
        result = subprocess.run(
            [str(self.bridge), "home", str(self._config_path)],
            cwd=str(REPO), capture_output=True, text=True, timeout=120,
        )
        if result.returncode != 0:
            detail = "\n".join((result.stderr or result.stdout or "home failed")
                               .strip().splitlines()[-12:])
            # The pedal can change after the GUI's last poll. The bridge's
            # preflight refusal is still an operator guard, not a runtime fault.
            if "home refused:" in detail:
                raise Rejected(detail)
            raise RuntimeError(detail)

    def begin_episode(self) -> None:
        """Cameras first, then the bridge -- the order collect_episode.sh set.

        RealSense auto-exposure settles over the first frames, so the recorder
        is given its head start before the arm can be driven.
        """
        if self._config_path is None:
            raise Rejected("no session is open")

        episode = reserve(self.episodes_dir, self._prefix)
        self._episode_dir = episode
        self._held_samples = 0
        self._total_samples = 0
        self._segments = 0
        self._was_held = False

        # The recorder holds the cameras exclusively, so the viewfinder has to
        # let go first. pyrealsense2 gives no useful error for a double claim.
        self.close_preview()

        shutil.copy(self._config_path, episode / "config.conf")
        calibration = REPO / "calibration.json"
        if calibration.is_file():
            shutil.copy(calibration, episode / "calibration.json")

        cam_log = episode / "cameras.log"
        self._recorder = _Proc(
            "recorder",
            [self.python, self.recorder,
             "--out", episode,
             "--duration", int(self._duration + CAMERA_OVERHANG_S),
             "--preview-dir", self.preview_dir],
            log_path=cam_log,
        )
        try:
            self._await_cameras(cam_log, timeout=30)
            self.camera_names = self._discover_cameras()
            self._start_bridge(log_csv=episode / "teleop.csv",
                               config=episode / "config.conf")
        except Exception:
            # Release hardware, but retain logs and the directory for
            # abort_session() to mark as a failed take.
            self._stop_bridge()
            self._stop_recorder()
            raise
        self._episode_started = time.monotonic()

    def end_episode(self, keep: bool) -> dict[str, Any]:
        episode = self._episode_dir
        bridge = self._bridge
        self._stop_bridge()
        self._stop_recorder()
        if keep:
            failure = self._bridge_failure(bridge, recording=True)
            if failure is not None:
                raise failure
        if episode is None:
            return {"kept": False, "reason": "no episode was open"}

        if not keep:
            shutil.rmtree(episode, ignore_errors=True)
            self._episode_dir = None
            self.open_preview()
            return {"kept": False, "reason": "discarded"}

        rows = self._teleop_rows(episode / "teleop.csv")
        if rows < MIN_TELEOP_ROWS:
            # The same floor collect_episode.sh enforces: below this the bridge
            # barely ran, usually a refused preflight.
            shutil.rmtree(episode, ignore_errors=True)
            self._episode_dir = None
            self.open_preview()
            return {"kept": False,
                    "reason": f"bridge wrote only {rows} rows; take discarded"}

        report = self._build_and_validate(episode)
        self._last_report = report
        if not (episode / 'episode.csv').is_file() or report.get('frames', 0) <= 0:
            raise RuntimeError('本条原始数据已保留，但汇总生成失败，未计入已保存条数：' +
                               str(report.get('build_output', report.get('problems'))))
        self._episode_dir = None
        self.open_preview()
        return {
            "kept": True,
            "steps": report.get("frames", 0),
            "segments": report.get("segments", self._segments),
            "verdict": report.get("verdict", "?"),
            "episode": episode.name,
        }

    def finalize_dataset(self) -> dict[str, Any]:
        """Re-validate every episode, so the sitting ends with a known state."""
        episodes = sorted(d for d in self.episodes_dir.glob(f"{self._prefix}*")
                          if (d / "episode.csv").is_file())
        verdicts: dict[str, str] = {}
        for episode in episodes:
            verdicts[episode.name] = self._validate(episode).get("verdict", "?")
        passed = sum(1 for v in verdicts.values() if v == "PASS")
        return {
            "dataset_dir": str(self.episodes_dir),
            "episodes": len(episodes),
            "passed": passed,
            "verdicts": verdicts,
        }

    # --- preview ----------------------------------------------------------
    def latest_jpeg(self, camera: str) -> bytes | None:
        """Serve the recorder's tap straight through, without a re-encode."""
        path = self.preview_dir / f"{camera}.jpg"
        try:
            return path.read_bytes()
        except OSError:
            return None

    def latest_frame(self, camera: str) -> np.ndarray | None:
        data = self.latest_jpeg(camera)
        if not data:
            return None
        return cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)

    def list_configs(self) -> list[dict[str, Any]]:
        presets = {
            "full100b": ("standard", "标准遥操 · 1:1 · 二值夹爪", 0),
            "full50b": ("standard", "半幅遥操 · 0.5 倍 · 二值夹爪", 1),
            "j6only": ("debug", "J6 单关节检查 · 0.25 倍", 2),
            "j7only": ("debug", "J7 单关节检查 · 0.25 倍", 3),
            "demo": ("script", "演示脚本专用", 4),
        }
        configs = []
        for p in self.conf_dir.glob("*.conf"):
            if not p.is_file() or p.is_symlink():
                continue
            category, label, order = presets.get(p.stem, ("debug", p.stem, 99))
            fields = dict(line.split("=", 1) for line in
                          (line.split("#", 1)[0].strip() for line in p.read_text().splitlines())
                          if "=" in line)
            fields = {k.strip(): v.strip() for k, v in fields.items()}
            if fields.get("gripper_type") == "robotiq":
                label = label.replace("二值夹爪", "Robotiq 夹爪")
            if fields.get("gripper_enabled", "0") == "0":
                label = label.replace("二值夹爪", "夹爪关闭").replace("Robotiq 夹爪", "夹爪关闭")
            configs.append({"name": p.stem, "path": str(p), "label": label,
                            "category": category, "order": order})
        return sorted(configs, key=lambda c: (c["order"], c["name"]))

    # --- internals: the bridge -------------------------------------------
    def _start_bridge(self, log_csv: Path | None,
                      config: Path | None = None) -> None:
        """Launch the bridge and wait for its status file to appear.

        The status file is the readiness signal rather than the `CONTROL_READY`
        line, because the bridge never flushes stdout and would sit in a 4 KB
        pipe buffer.
        """
        source = config or self._config_path
        assert source is not None
        prepared = self._prepare_config(source)

        # A stale file from a previous run would read as instant readiness.
        self.status_path.unlink(missing_ok=True)

        mode = "dry" if self._dry_run else "robot"
        argv = [self.bridge, mode, prepared, f"{self._duration:.0f}"]
        if log_csv is not None:
            argv.append(str(log_csv))
        log_path = ((log_csv.parent / "bridge.log") if log_csv
                    else self.scratch_dir / f"pnp7_bridge_{time.time_ns()}.log")
        self._bridge = _Proc("bridge", argv, log_path=log_path)

        deadline = time.monotonic() + 30.0
        while time.monotonic() < deadline:
            if not self._bridge.alive:
                raise RuntimeError(
                    f"控制程序启动失败（退出码 {self._bridge.proc.returncode}）\n"
                    f"日志：{log_path}\n" + self._tail(log_path, lines=12))
            if self.status_path.is_file():
                return
            time.sleep(0.1)
        self._stop_bridge()
        raise RuntimeError(
            "bridge never published a status file: " + self._tail(log_path))

    def _stop_bridge(self) -> None:
        if self._bridge is None:
            return
        # SIGINT, never SIGKILL: the whole log is still in RAM at this point.
        self._bridge.stop(timeout=60)
        self._bridge.close()
        self._bridge = None
        self.status_path.unlink(missing_ok=True)

    def _read_status(self) -> dict[str, Any] | None:
        try:
            stat = self.status_path.stat()
            if time.time() - stat.st_mtime > STATUS_STALE_S:
                return None
            return json.loads(self.status_path.read_text())
        except (OSError, json.JSONDecodeError):
            # The rename is atomic, so a decode failure means it is being
            # replaced right now; the next poll is 100 ms away.
            return None

    def _prepare_config(self, source: Path) -> Path:
        """Copy the config and append `status_path`.

        Appending is safe because `loadConfig` takes the last occurrence of a
        duplicated key. It is also the only option: unknown keys are silently
        ignored, so there is no way to verify a rewrite took effect.
        """
        if source.parent == self.conf_dir:
            # Never edit a tracked config in place.
            prepared = self.scratch_dir / f"pnp7_gui_{source.name}"
            shutil.copy(source, prepared)
        else:
            # Already the episode's own copy; appending there is what makes it
            # a faithful record of what actually ran.
            prepared = source
        with open(prepared, "a") as fh:
            fh.write(f"\n# appended by the collection GUI\n"
                     f"status_path={self.status_path}\n")
        return prepared

    # --- internals: the cameras ------------------------------------------
    def _stop_recorder(self) -> None:
        if self._recorder is None:
            return
        self._recorder.stop(timeout=30)
        self._recorder.close()
        self._recorder = None

    def _await_cameras(self, log_path: Path, timeout: float) -> None:
        """Wait for CAMERAS_READY, the recorder's one-second settle marker."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            try:
                if "CAMERAS_READY" in log_path.read_text():
                    return
            except OSError:
                pass
            proc = self._recorder or self._viewfinder
            if proc is not None and not proc.alive:
                raise RuntimeError(
                    "camera recorder died: " + self._tail(log_path))
            time.sleep(0.25)
        raise RuntimeError("cameras never became ready: " + self._tail(log_path))

    def _discover_cameras(self) -> list[str]:
        try:
            return sorted(p.stem for p in self.preview_dir.glob("*.jpg"))
        except OSError:
            return []

    def _clear_previews(self) -> None:
        for stale in self.preview_dir.glob("*.jpg"):
            stale.unlink(missing_ok=True)

    def _check_children(self) -> None:
        """Surface a dead child rather than letting the UI look healthy.

        Only while the bridge is still up: the recorder is deliberately given
        `duration + CAMERA_OVERHANG_S`, so it outliving the bridge is the
        design, and its own exit afterwards is expected.
        """
        bridge_up = self._bridge is not None and self._bridge.alive
        if bridge_up and self._recorder is not None and not self._recorder.alive:
            log = self._recorder.log_path
            self._recorder.close()
            self._recorder = None
            raise RuntimeError("camera recorder exited: " + self._tail(log))

    # --- internals: post-processing --------------------------------------
    def _build_and_validate(self, episode: Path) -> dict[str, Any]:
        roles = sorted(p.name[len('cam_'):-len('_index.csv')]
                       for p in episode.glob('cam_*_index.csv'))
        # Preserve actual camera role names. Prefer the named external view,
        # otherwise the non-wrist camera when the USB serial changed.
        anchor = 'external' if 'external' in roles else next(
            (role for role in roles if role != 'wrist'), roles[0] if roles else 'external')
        build = subprocess.run(
            [str(self.python), str(REPO / "collect" / "build_episode.py"),
             "--episode", str(episode), "--no-events", "--anchor", anchor],
            cwd=str(REPO), capture_output=True, text=True, timeout=600,
        )
        # Exit 1 here means a stream exceeded the skew tolerance -- episode.csv
        # is still written. That is a warning about alignment, not a failure.
        report: dict[str, Any] = {
            "build_ok": build.returncode == 0,
            "build_output": ((build.stdout or "") + "\n" + (build.stderr or "")).strip().splitlines()[-10:],
        }
        if not (episode / "episode.csv").is_file():
            report["verdict"] = "FAIL"
            report["problems"] = ["build_episode wrote no episode.csv"]
            return report

        meta_path = episode / "episode_meta.json"
        if meta_path.is_file():
            try:
                meta = json.loads(meta_path.read_text())
                report["frames"] = meta.get("frames", 0)
                report["rate_hz"] = meta.get("rate_hz")
                report["dropped_idle"] = meta.get("dropped_idle_frames")
            except (OSError, json.JSONDecodeError):
                pass
        report.update(self._validate(episode))
        return report

    def _validate(self, episode: Path) -> dict[str, Any]:
        result = subprocess.run(
            [str(self.python), str(REPO / "collect" / "validate_episode.py"),
             str(episode)],
            cwd=str(REPO), capture_output=True, text=True, timeout=300,
        )
        text = (result.stdout or "") + (result.stderr or "")
        verdict = "?"
        problems: list[str] = []
        for line in text.splitlines():
            stripped = line.strip()
            if stripped.startswith("verdict:"):
                verdict = stripped.split(":", 1)[1].strip()
            elif stripped.startswith(("WARN ", "FAIL ")):
                problems.append(stripped)
        if verdict == "?" and result.returncode != 0:
            # validate_episode.py can raise IndexError on a header-only CSV,
            # in which case there is no verdict line to find.
            verdict = "FAIL"
            problems.append(text.strip().splitlines()[-1] if text else "crashed")
        return {"verdict": verdict, "problems": problems}

    # --- small helpers ----------------------------------------------------
    @staticmethod
    def _collision_reflex_only(detail: str) -> bool:
        """Classify the SDK error list, not the generic 'reflex' wording.

        Communication and discontinuity failures also say 'reflex'. A mixed
        list must keep the full fault behavior, even if it includes collision.
        """
        matches = re.findall(r'motion aborted by reflex!\s*(\[[^\]]*\])', detail)
        matches += re.findall(r'Franka fault onset:[^\n]*?errors=(\[[^\]]*\])', detail)
        if 'franka error: libfranka: Move command aborted: motion aborted by reflex!' not in detail:
            return False
        if not matches:
            return False
        for raw in matches:
            try:
                errors = json.loads(raw)
                if not errors or not set(errors) <= {'cartesian_reflex', 'joint_reflex'}:
                    return False
            except (TypeError, ValueError):
                return False
        return True

    def _bridge_failure(self, bridge: _Proc | None, *, recording: bool) -> Exception | None:
        code = bridge.proc.poll() if bridge is not None else None
        if code is None or code == 0:
            return None
        log = bridge.log_path
        detail = self._tail(log, lines=20)
        collision = code == 1 and self._collision_reflex_only(detail)
        if collision:
            reason = "Franka 触发碰撞保护，本条数据无效"
        elif "FCI timing guard:" in detail:
            reason = "FCI 控制回调超时，已取消过期关节指令并停止"
        elif "discontinuity" in detail:
            reason = "Franka 检测到关节指令不连续，已中止运动"
        elif "communication_constraints_violation" in detail:
            reason = "Franka 通信时序不满足要求，已中止运动"
        else:
            reason = "控制程序异常退出"
        exception = (EpisodeReflex if collision and recording and
                     self._mode is Mode.COLLECT and self._episode_dir is not None
                     else RuntimeError)
        return exception(f"{reason}（退出码 {code}）\n日志：{log}\n{detail}")

    def discard_reflex_episode(self, reason: str) -> dict[str, Any]:
        episode = self._episode_dir
        if episode is None or self._mode is not Mode.COLLECT:
            raise RuntimeError("没有可丢弃的采集条目")
        self._stop_bridge()
        self._stop_recorder()
        diagnostic = self.scratch_dir / f"pnp7_discarded_{episode.name}_{time.time_ns()}"
        diagnostic.mkdir(parents=True)
        for name in ('bridge.log', 'cameras.log', 'config.conf'):
            source = episode / name
            if source.is_file():
                shutil.copy2(source, diagnostic / name)
        (diagnostic / 'failure.json').write_text(json.dumps({
            'error': reason, 'episode': episode.name, 'time': time.time(),
            'usable_episode': False, 'discarded': True,
        }, ensure_ascii=False, indent=2) + '\n')
        # Fail visibly if removal fails; never report invalid data discarded
        # while it is still sitting among the dataset's episodes.
        shutil.rmtree(episode)
        self._episode_dir = None
        self._held_samples = self._total_samples = self._segments = 0
        self._was_held = False
        self._last_status = {}
        self.open_preview()
        return {'episode': episode.name, 'diagnostic_dir': str(diagnostic)}

    def dataset_progress(self) -> dict[str, Any]:
        return progress(self.episodes_dir, self._prefix)

    def _next_episode_dir(self) -> Path:
        return self.episodes_dir / self.dataset_progress()['next_episode']

    @staticmethod
    def _teleop_rows(path: Path) -> int:
        try:
            with open(path, "rb") as fh:
                return max(sum(1 for _ in fh) - 1, 0)
        except OSError:
            return 0

    @staticmethod
    def _tail(path: Path | None, lines: int = 4) -> str:
        if path is None:
            return "(no log)"
        try:
            return " / ".join(path.read_text().strip().splitlines()[-lines:])
        except OSError:
            return "(no log)"

    @staticmethod
    def _nonnegative(value: Any) -> float | None:
        # The bridge writes -1.0 for width when the gripper is disabled or the
        # run is dry; that is "no reading", not a measurement.
        try:
            number = float(value)
        except (TypeError, ValueError):
            return None
        return None if number < 0 else number
