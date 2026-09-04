"""State machine and backend contract for the collection GUI.

Every piece of hardware on this rig is claimed exclusively by whoever opens it
first -- ``RealSenseCamera`` calls ``pipeline.start()``, franky holds the FCI
control connection, and the GELLO leader is a serial port. A GUI that pokes the
hardware from its own process would therefore fight the collection run for it.
So the GUI does not own hardware at all: it owns a *session*, and the session
owns the env. One process, one claim on each device.

The state machine below is the whole contract between the browser and that
process. The server translates HTTP into `Command`s and renders `SessionState`
back out as JSON; the backend is whatever actually moves.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Callable

import numpy as np


class Rejected(Exception):
    """A command the state machine declines to run right now.

    Distinct from a failure on purpose. "You cannot restore the arm while an
    episode is open" is the guard doing its job; treating it like a fault --
    which an earlier version did -- aborted the episode the operator was in the
    middle of, turning a misclick into lost data.
    """


class Phase(str, Enum):
    """Where the session is. Transitions are driven by the control loop."""

    IDLE = "idle"          # no env; cameras may be open for a bare preview
    OPENING = "opening"    # building the env -- seconds, and it can fail
    READY = "ready"        # env live, robot holding, nothing being recorded
    RECORDING = "recording"  # an episode is open; F3-held frames are kept
    CLOSING = "closing"
    ERROR = "error"        # the last operation raised; message in `error`


#: Phases in which the env exists and the control loop is stepping it.
LIVE_PHASES = (Phase.READY, Phase.RECORDING)


class Mode(str, Enum):
    """Why the session was opened.

    ``TELEOP`` is item 7: the same env stack, the same safety chain, but the
    episode controls are refused so nothing can be recorded by accident. It is
    a mode rather than a separate program precisely because tearing the env
    down and back up costs an alignment check and several seconds.
    """

    COLLECT = "collect"
    TELEOP = "teleop"


@dataclass
class Command:
    """One operator action, queued for the control loop.

    The browser never touches the env directly. It appends a Command, the
    control loop applies it between steps, and the result shows up in the next
    `SessionState`. That keeps every env call on one thread, which franky and
    pyrealsense2 both require in practice.
    """

    name: str
    args: dict[str, Any] = field(default_factory=dict)
    #: Set when the loop has applied it, so the HTTP handler can report back.
    done: threading.Event = field(default_factory=threading.Event, repr=False)
    error: str | None = None


@dataclass
class SessionState:
    """Everything the browser renders. Serialised as JSON on every poll."""

    phase: Phase = Phase.IDLE
    mode: Mode = Mode.COLLECT
    config_name: str | None = None
    error: str | None = None
    message: str = ""

    # --- teleop / safety -------------------------------------------------
    deadman_held: bool = False
    stream_enabled: bool = False
    block_reason: str | None = None
    alignment_error: float | None = None
    joints: list[float] = field(default_factory=list)
    gripper_width: float | None = None

    # --- episode bookkeeping --------------------------------------------
    episode_index: int = 0        # episodes kept this session
    episode_target: int = 0       # runner.num_data_episodes
    episode_steps: int = 0        # steps appended to the open episode
    episode_segments: int = 0     # F3 intervals within the open episode
    recorded_steps: int = 0       # steps that passed the F3 gate
    skipped_steps: int = 0        # steps dropped because F3 was released
    dataset_dir: str | None = None

    # --- loop health ------------------------------------------------------
    step_hz: float = 0.0
    uptime_s: float = 0.0
    cameras: list[str] = field(default_factory=list)

    def to_json(self) -> dict[str, Any]:
        out = {
            "phase": self.phase.value,
            "mode": self.mode.value,
            "config_name": self.config_name,
            "error": self.error,
            "message": self.message,
            "deadman_held": self.deadman_held,
            "stream_enabled": self.stream_enabled,
            "block_reason": self.block_reason,
            "alignment_error": self.alignment_error,
            "joints": [round(float(q), 5) for q in self.joints],
            "gripper_width": self.gripper_width,
            "episode_index": self.episode_index,
            "episode_target": self.episode_target,
            "episode_steps": self.episode_steps,
            "episode_segments": self.episode_segments,
            "recorded_steps": self.recorded_steps,
            "skipped_steps": self.skipped_steps,
            "dataset_dir": self.dataset_dir,
            "step_hz": round(self.step_hz, 2),
            "uptime_s": round(self.uptime_s, 1),
            "cameras": list(self.cameras),
        }
        return out


class Backend:
    """What the control loop needs from the world.

    Two implementations: `gui.mock.MockBackend`, which runs anywhere and is what
    the UI is developed and tested against, and `gui.rlinf_backend.RlinfBackend`,
    which drives the real single-arm RLinf stack on the robot PC. Keeping the
    seam here is what makes the GUI testable off the rig at all -- none of the
    hardware imports (pyrealsense2, franky, evdev, ray) resolve on a laptop.
    """

    #: Camera names available from `latest_frame`, once opened.
    camera_names: list[str] = []

    # --- lifecycle --------------------------------------------------------
    def open_preview(self) -> None:
        """Open cameras only, for framing the scene before a session starts."""
        raise NotImplementedError

    def close_preview(self) -> None:
        """Release the cameras. Must be called before `open_session`."""
        raise NotImplementedError

    def open_session(self, config_name: str, overrides: dict[str, Any],
                     mode: Mode) -> None:
        raise NotImplementedError

    def close_session(self) -> None:
        raise NotImplementedError

    # --- per-step ---------------------------------------------------------
    def step(self, recording: bool) -> dict[str, Any]:
        """Advance one control period.

        Args:
            recording: Whether an episode is open. Combined with the live F3
                state inside the backend, this decides whether the frame is
                appended -- the GUI never sees a frame it has to filter itself.

        Returns:
            A telemetry dict merged into `SessionState` by the control loop.
        """
        raise NotImplementedError

    # --- operator actions -------------------------------------------------
    def restore_joints(self, qpos: list[float] | None = None) -> None:
        """Item 1: drive the arm back to the configured start configuration."""
        raise NotImplementedError

    def begin_episode(self) -> None:
        raise NotImplementedError

    def end_episode(self, keep: bool) -> dict[str, Any]:
        raise NotImplementedError

    def finalize_dataset(self) -> dict[str, Any]:
        """Flush LeRobot ``info.json``/``stats.json`` so the shard is loadable."""
        raise NotImplementedError

    # --- preview ----------------------------------------------------------
    def latest_frame(self, camera: str) -> np.ndarray | None:
        raise NotImplementedError

    def list_configs(self) -> list[dict[str, Any]]:
        raise NotImplementedError


class ControlLoop:
    """Owns the backend, applies queued commands, and steps at a fixed rate.

    One thread, because that is the only safe arrangement: franky's controller
    handle, the RealSense pipelines and the GELLO serial port are all opened
    here and must be touched from the same thread that opened them.
    """

    def __init__(self, backend: Backend, fps: float = 10.0,
                 on_state: Callable[[SessionState], None] | None = None):
        self.backend = backend
        self.period = 1.0 / float(fps)
        self.state = SessionState()
        self._lock = threading.Lock()
        self._commands: list[Command] = []
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._on_state = on_state
        self._t_open = 0.0
        self._step_times: list[float] = []

    # --- public API -------------------------------------------------------
    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="control-loop",
                                        daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=15)

    def submit(self, name: str, **args) -> Command:
        cmd = Command(name=name, args=args)
        with self._lock:
            self._commands.append(cmd)
        return cmd

    def snapshot(self) -> SessionState:
        with self._lock:
            # Shallow copy is enough: the loop replaces list fields wholesale
            # rather than mutating them in place.
            import copy

            return copy.copy(self.state)

    # --- loop -------------------------------------------------------------
    def _run(self) -> None:
        while not self._stop.is_set():
            t0 = time.perf_counter()
            self._drain_commands()
            if self.state.phase in LIVE_PHASES:
                self._step_once()
            else:
                # Nothing to drive; keep the poll cheap.
                time.sleep(0.02)
            self._pace(t0)
        self._safe_close()

    def _pace(self, t0: float) -> None:
        if self.state.phase not in LIVE_PHASES:
            return
        elapsed = time.perf_counter() - t0
        if elapsed < self.period:
            time.sleep(self.period - elapsed)
        # Rolling step rate over the last ~2 s, so a stall is visible in the UI
        # rather than only in the log.
        self._step_times.append(time.perf_counter())
        cutoff = time.perf_counter() - 2.0
        while self._step_times and self._step_times[0] < cutoff:
            self._step_times.pop(0)
        if len(self._step_times) > 1:
            span = self._step_times[-1] - self._step_times[0]
            self.state.step_hz = (len(self._step_times) - 1) / span if span else 0.0

    def _step_once(self) -> None:
        try:
            telemetry = self.backend.step(
                recording=self.state.phase is Phase.RECORDING)
        except Exception as exc:  # noqa: BLE001 - surface it, never crash the loop
            self._fail(f"step failed: {exc}")
            return
        with self._lock:
            for key, value in telemetry.items():
                if hasattr(self.state, key):
                    setattr(self.state, key, value)
            self.state.uptime_s = time.monotonic() - self._t_open

    def _drain_commands(self) -> None:
        with self._lock:
            pending, self._commands = self._commands, []
        for cmd in pending:
            try:
                self._apply(cmd)
            except Rejected as exc:
                # A guard declined: "not from this phase", "release F3 first".
                # Expected, and emphatically not a reason to end the take --
                # clicking a button that is not allowed right now must never
                # cost the operator the episode they are in the middle of.
                cmd.error = str(exc)
                self.state.error = f"{cmd.name}: {exc}"
            except Exception as exc:  # noqa: BLE001
                # Something actually broke while applying the command.
                cmd.error = str(exc)
                self._fail(f"{cmd.name}: {exc}")
            finally:
                cmd.done.set()

    def _apply(self, cmd: Command) -> None:  # noqa: C901 - a flat dispatch table
        name = cmd.name
        st = self.state
        if name == "open_preview":
            if st.phase is not Phase.IDLE:
                raise Rejected("preview is only available while idle")
            self.backend.open_preview()
            st.cameras = list(self.backend.camera_names)
            st.message = "preview open"

        elif name == "close_preview":
            self.backend.close_preview()
            st.cameras = []
            st.message = "preview closed"

        elif name == "open_session":
            if st.phase is not Phase.IDLE:
                raise Rejected(f"cannot open a session from {st.phase.value}")
            # The cameras must be handed over, not shared: pyrealsense2 refuses
            # a second pipeline on the same serial.
            self.backend.close_preview()
            st.phase = Phase.OPENING
            st.error = None
            mode = Mode(cmd.args.get("mode", "collect"))
            st.message = f"opening {mode.value} session..."
            self.backend.open_session(
                config_name=cmd.args["config_name"],
                overrides=cmd.args.get("overrides", {}),
                mode=mode,
            )
            st.mode = mode
            st.config_name = cmd.args["config_name"]
            st.cameras = list(self.backend.camera_names)
            st.phase = Phase.READY
            st.episode_index = 0
            st.recorded_steps = 0
            st.skipped_steps = 0
            self._t_open = time.monotonic()
            st.message = ("teleop only -- recording is disabled"
                          if mode is Mode.TELEOP else "ready")

        elif name == "close_session":
            st.phase = Phase.CLOSING
            self.backend.close_session()
            st.phase = Phase.IDLE
            st.config_name = None
            st.cameras = []
            st.message = "session closed"

        elif name == "restore_joints":
            # Refused while recording: the move would be written into the
            # episode as if the operator had demonstrated it.
            if st.phase is not Phase.READY:
                raise Rejected(
                    "restore the arm from READY only -- stop the episode first")
            if st.deadman_held:
                raise Rejected("release F3 before restoring the arm")
            st.message = "restoring joints..."
            self.backend.restore_joints(cmd.args.get("qpos"))
            st.message = "joints restored"

        elif name == "begin_episode":
            if st.mode is Mode.TELEOP:
                raise Rejected("this session is teleop-only")
            if st.phase is not Phase.READY:
                raise Rejected(f"cannot start an episode from {st.phase.value}")
            self.backend.begin_episode()
            st.phase = Phase.RECORDING
            st.episode_steps = 0
            st.episode_segments = 0
            st.message = "recording -- hold F3 to capture"

        elif name in ("end_episode", "discard_episode"):
            if st.phase is not Phase.RECORDING:
                raise Rejected("no episode is open")
            keep = name == "end_episode"
            result = self.backend.end_episode(keep=keep)
            st.phase = Phase.READY
            if keep and result.get("kept"):
                st.episode_index += 1
                st.message = (f"episode kept: {result.get('steps', 0)} frames, "
                              f"{result.get('segments', 0)} segment(s)")
            elif keep:
                st.message = f"episode dropped: {result.get('reason', 'empty')}"
            else:
                st.message = "episode discarded"

        elif name == "finalize_dataset":
            result = self.backend.finalize_dataset()
            st.dataset_dir = result.get("dataset_dir", st.dataset_dir)
            st.message = f"dataset finalized: {result.get('episodes', 0)} episode(s)"

        elif name == "clear_error":
            st.error = None
            if st.phase is Phase.ERROR:
                st.phase = Phase.IDLE

        else:
            raise Rejected(f"unknown command {name!r}")

    def _fail(self, message: str) -> None:
        self.state.error = message
        # A failure mid-episode must not silently keep appending frames.
        if self.state.phase is Phase.RECORDING:
            try:
                self.backend.end_episode(keep=False)
            except Exception:  # noqa: BLE001
                pass
            self.state.phase = Phase.READY

    def _safe_close(self) -> None:
        try:
            if self.state.phase in LIVE_PHASES:
                self.backend.close_session()
            self.backend.close_preview()
        except Exception:  # noqa: BLE001
            pass
