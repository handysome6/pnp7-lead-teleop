"""The real backend: the single-arm RLinf GELLO/Franky stack, driven by the GUI.

This runs **inside** an RLinf Ray worker, and it has to. The realworld stack is
not importable-and-callable on its own:

* ``FrankaEnv._setup_hardware`` asserts ``isinstance(self.hardware_info,
  FrankaHWInfo)``, so the cluster hardware registry is mandatory rather than
  optional -- ``worker_info=None`` fails outright.
* ``FrankyController.launch_controller`` spawns the arm controller as a Ray
  actor via ``create_group(...).launch(cluster=Cluster(), ...)`` and every call
  on it returns an RLinf future (``.wait()[0]``).

So the GUI does not talk to RLinf over a socket, and it does not shell out to
``collect_data.sh``. It *is* the collector: `gui.app` launches a worker whose
``run()`` starts the HTTP server and the control loop in-process. That is also
the only arrangement that can work, because every device here is claimed
exclusively -- ``RealSenseCamera`` calls ``pipeline.start()``, franky holds the
FCI connection, and the GELLO leader is a serial port. Two processes cannot
share them, so there is exactly one process and the GUI lives in it.

Nothing in RLinf is modified. The three seams used are all public-by-construction:

1. ``CollectEpisode`` already honours ``pre_record`` / ``record_reset`` /
   ``segment_advance`` from ``info`` -- see `gui.episode_control`.
2. ``FrankaEnv.camera_player`` is a plain attribute assigned after the cameras
   open, so swapping in a capture shim tees the frames the env is already
   reading. No second pipeline, and no stealing frames out of the queue.
3. ``FrankyController.reset_joint`` is what ``go_to_rest`` itself calls.
"""
from __future__ import annotations

import copy
import threading
import time
from pathlib import Path
from typing import Any

import numpy as np
from omegaconf import OmegaConf

from gui.episode_control import GuiEpisodeControl
from gui.session import Backend, Mode


class _FrameTap:
    """Stands in for ``VideoPlayer``, keeping the latest frame per camera.

    Same duck type (``put_frame`` / ``stop``), so ``FrankaEnv`` cannot tell the
    difference, and it optionally forwards to the real player so an operator who
    does want the cv2 window still gets it.
    """

    def __init__(self, forward=None):
        self._frames: dict[str, np.ndarray] = {}
        self._lock = threading.Lock()
        self._forward = forward
        self.is_running = True

    def put_frame(self, frames) -> None:
        if isinstance(frames, dict):
            with self._lock:
                for name, image in frames.items():
                    if isinstance(image, np.ndarray) and image.ndim == 3:
                        # Depth-augmented frames carry four channels; the
                        # preview only wants colour.
                        self._frames[str(name)] = image[:, :, :3]
        if self._forward is not None:
            try:
                self._forward.put_frame(frames)
            except Exception:  # noqa: BLE001 - a dead cv2 window must not stop collection
                self._forward = None

    def names(self) -> list[str]:
        with self._lock:
            return sorted(self._frames)

    def latest(self, name: str) -> np.ndarray | None:
        with self._lock:
            frame = self._frames.get(name)
        return None if frame is None else frame.copy()

    def stop(self) -> None:
        self.is_running = False
        if self._forward is not None:
            try:
                self._forward.stop()
            except Exception:  # noqa: BLE001
                pass


#: Config keys the GUI is allowed to override, mapped to where they live.
#: Deliberately a short allowlist: the calibration-derived values (joint signs,
#: offsets, the J7 bias, the acceleration bound) were settled by live sweeps and
#: are not things to retune from a browser between takes.
OVERRIDE_TARGETS = {
    "fps": ("data_collection", "fps"),
    "num_data_episodes": ("runner", "num_data_episodes"),
    "gello_joint_max_velocity": ("eval", "gello_joint_max_velocity"),
    "max_episode_steps": ("eval", "max_episode_steps"),
    "save_dir": ("data_collection", "save_dir"),
    "task_description": ("override_cfg", "task_description"),
}


class RlinfBackend(Backend):
    """Drives one RLinf single-arm collection session.

    Args:
        cfg: The resolved hydra config, as ``collect_real_data.py`` receives it.
        worker_info: ``self.worker_info`` from the enclosing RLinf ``Worker``.
        config_dir: Directory of selectable ``*.yaml`` configs, for the picker.
    """

    def __init__(self, cfg, worker_info, config_dir: str | Path | None = None):
        self.base_cfg = cfg
        self.worker_info = worker_info
        self.config_dir = Path(config_dir) if config_dir else None

        self.camera_names: list[str] = []
        self._env = None            # CollectEpisode(GuiEpisodeControl(RealWorldEnv))
        self._gui_wrapper: GuiEpisodeControl | None = None
        self._realworld = None
        self._tap: _FrameTap | None = None
        self._action_dim = 0
        self._mode = Mode.COLLECT
        self._dataset_dir: str | None = None
        self._episode_target = 0
        self._last_obs = None

    # --- lifecycle --------------------------------------------------------
    def open_preview(self) -> None:
        # There is no preview before a session: opening the RealSense here would
        # claim the very devices the env is about to need, and releasing them
        # cleanly enough for pyrealsense2 to re-open is not reliable. The UI
        # shows a placeholder instead of pretending otherwise.
        raise RuntimeError(
            "standalone preview is unavailable on the real rig -- the "
            "environment claims the RealSense pipelines exclusively. Open a "
            "teleop-only session to see the cameras without recording.")

    def close_preview(self) -> None:
        return

    def open_session(self, config_name: str, overrides: dict[str, Any],
                     mode: Mode) -> None:
        from rlinf.envs.realworld.realworld_env import RealWorldEnv
        from rlinf.envs.wrappers import CollectEpisode

        cfg = self._compose(config_name, overrides, mode)
        self._mode = mode

        realworld = RealWorldEnv(
            cfg.env.eval,
            num_envs=1,
            seed_offset=0,
            total_num_processes=1,
            worker_info=self.worker_info,
        )
        self._realworld = realworld
        self._install_frame_tap(realworld)

        env = GuiEpisodeControl(
            realworld,
            require_deadman=bool(
                cfg.env.eval.get("gui_require_deadman", True)),
            segment_min_s=float(cfg.env.eval.get("gui_segment_min_s", 0.25)),
        )
        self._gui_wrapper = env

        dc = cfg.env.eval.get("data_collection")
        if mode is Mode.COLLECT and dc and dc.get("enabled", False):
            self._dataset_dir = str(dc.save_dir)
            env = CollectEpisode(
                env,
                save_dir=dc.save_dir,
                export_format=dc.get("export_format", "lerobot"),
                robot_type=dc.get("robot_type", "panda"),
                fps=dc.get("fps", 10),
                # Every episode the GUI keeps is one the operator chose to keep,
                # so the success filter would only ever throw them away: the
                # teleop task publishes no reward signal.
                only_success=False,
                finalize_interval=dc.get("finalize_interval", 100),
                resume=bool(dc.get("resume", True)),
            )
        else:
            # Teleop-only: no writer is constructed at all, so there is nothing
            # that *could* write, rather than a writer we promise not to call.
            self._dataset_dir = None

        self._env = env
        self._action_dim = int(env.action_space.shape[-1])
        self._episode_target = int(cfg.runner.get("num_data_episodes", 0) or 0)

        # `skip_reset_to_home` holds the current pose: absolute GELLO mapping
        # requires the leader and follower to already agree within
        # `startup_max_error`, and a reset that drove the arm to a historical
        # Cartesian pose would guarantee they do not.
        obs, _ = env.reset(options={"skip_reset_to_home": True})
        self._last_obs = obs
        self.camera_names = self._tap.names() if self._tap else []

    def close_session(self) -> None:
        env, self._env = self._env, None
        self._gui_wrapper = None
        self._realworld = None
        self.camera_names = []
        if env is not None:
            try:
                env.close()
            finally:
                self._tap = None

    # --- per-step ---------------------------------------------------------
    def step(self, recording: bool) -> dict[str, Any]:
        if self._env is None:
            return {}
        # The action is ignored: `GelloJointIntervention` overrides it from the
        # leader and, under `teleop_direct_stream`, has already sent it. What
        # `env.step` does here is read state and cameras, and record.
        action = np.zeros((1, self._action_dim), dtype=np.float32)
        obs, reward, terminated, truncated, info = self._env.step(action)
        self._last_obs = obs

        if self.camera_names != (names := (self._tap.names() if self._tap else [])):
            self.camera_names = names

        telemetry: dict[str, Any] = {
            "deadman_held": _flag(info, "gello_deadman_held"),
            "stream_enabled": _flag(info, "gello_stream_enabled"),
            "block_reason": _scalar(info, "gello_stream_block_reason"),
            "alignment_error": _float(info, "gello_alignment_error"),
            "dataset_dir": self._dataset_dir,
            "episode_target": self._episode_target,
        }
        if self._gui_wrapper is not None:
            telemetry.update(
                episode_steps=self._gui_wrapper.steps_recorded,
                episode_segments=self._gui_wrapper.segments,
                recorded_steps=self._gui_wrapper.steps_recorded,
                skipped_steps=self._gui_wrapper.steps_skipped,
            )
        joints, width = self._read_arm()
        if joints is not None:
            telemetry["joints"] = joints
        if width is not None:
            telemetry["gripper_width"] = width
        return telemetry

    # --- operator actions -------------------------------------------------
    def restore_joints(self, qpos: list[float] | None = None) -> None:
        """Item 1: drive the arm back to the configured start configuration.

        Uses ``FrankyController.reset_joint``, which is what ``go_to_rest``
        calls for the same purpose. It is a blocking ``franky.JointMotion``, so
        the control loop pauses here -- which is correct: nothing should be
        streaming targets at the arm while it is being repositioned.
        """
        base = self._base_env()
        if base is None:
            raise RuntimeError("no session is open")
        target = list(qpos) if qpos else list(base.config.joint_reset_qpos)
        if len(target) != 7:
            raise ValueError(f"expected 7 joint targets, got {len(target)}")
        base._controller.reset_joint(target).wait()
        time.sleep(0.5)

    def begin_episode(self) -> None:
        if self._gui_wrapper is None:
            raise RuntimeError("no session is open")
        self._gui_wrapper.begin_episode()

    def end_episode(self, keep: bool) -> dict[str, Any]:
        if self._gui_wrapper is None:
            raise RuntimeError("no session is open")
        result = self._gui_wrapper.end_episode()
        kept = bool(keep and result["steps"] > 0)
        if kept:
            # CollectEpisode flushes on a terminated/truncated step, so the take
            # is closed by stepping once with the flag set rather than by
            # reaching into its buffers.
            self._flush_episode()
        elif not keep:
            self._discard_episode()
        result["kept"] = kept
        if keep and not kept:
            result["reason"] = "no frames were captured with F3 held"
        return result

    def finalize_dataset(self) -> dict[str, Any]:
        env = self._env
        episodes = 0
        if env is not None and hasattr(env, "_finalize_lerobot"):
            env._finalize_lerobot()
            episodes = int(getattr(env, "preexisting_episode_count", 0) or 0)
        return {"dataset_dir": self._dataset_dir, "episodes": episodes}

    # --- preview ----------------------------------------------------------
    def latest_frame(self, camera: str) -> np.ndarray | None:
        return self._tap.latest(camera) if self._tap else None

    def list_configs(self) -> list[dict[str, Any]]:
        if self.config_dir and self.config_dir.is_dir():
            names = sorted(p for p in self.config_dir.glob("*.yaml"))
            return [{"name": p.stem, "path": str(p)} for p in names]
        name = getattr(self.base_cfg, "_config_name_", None) or "current"
        return [{"name": str(name), "path": "(launched config)"}]

    # --- internals --------------------------------------------------------
    def _compose(self, config_name: str, overrides: dict[str, Any], mode: Mode):
        """Apply the GUI's parameter overrides onto the launched config.

        Hydra composes at process start, so a config *switch* would mean
        relaunching the worker. Selecting a different name here is therefore
        only honoured when it matches what was launched; the parameters are what
        the GUI actually varies between takes.
        """
        cfg = copy.deepcopy(self.base_cfg)
        OmegaConf.set_struct(cfg, False)

        for key, value in (overrides or {}).items():
            if key not in OVERRIDE_TARGETS or value in (None, ""):
                continue
            section, field = OVERRIDE_TARGETS[key]
            if section == "runner":
                cfg.runner[field] = value
            elif section == "eval":
                cfg.env.eval[field] = value
            elif section == "data_collection":
                dc = cfg.env.eval.get("data_collection")
                if dc is not None:
                    dc[field] = value
            elif section == "override_cfg":
                cfg.env.eval.override_cfg[field] = value

        # `max_episode_steps: null` is RLinf's own spelling for "an external
        # wrapper owns the episode end", which is exactly what the GUI is.
        cfg.env.eval.max_episode_steps = None
        cfg.env.eval.override_cfg.manual_episode_control_only = True
        if mode is Mode.TELEOP:
            dc = cfg.env.eval.get("data_collection")
            if dc is not None:
                dc.enabled = False
        OmegaConf.set_struct(cfg, True)
        return cfg

    def _install_frame_tap(self, realworld) -> None:
        base = self._base_env(realworld)
        if base is None or not hasattr(base, "camera_player"):
            return
        self._tap = _FrameTap(forward=base.camera_player)
        base.camera_player = self._tap

    def _base_env(self, realworld=None):
        """The innermost ``FrankaEnv`` behind the vector env and its wrappers."""
        realworld = realworld or self._realworld
        try:
            return realworld.env.envs[0].unwrapped
        except (AttributeError, IndexError):
            return None

    def _read_arm(self) -> tuple[list[float] | None, float | None]:
        base = self._base_env()
        if base is None:
            return None, None
        state = getattr(base, "_franka_state", None)
        if state is None:
            return None, None
        joints = getattr(state, "arm_joint_position", None)
        width = getattr(state, "gripper_width", None)
        return (list(np.asarray(joints, dtype=float)) if joints is not None else None,
                float(width) if width is not None else None)

    def _flush_episode(self) -> None:
        """Close the open take by stepping once with ``terminated`` set.

        ``CollectEpisode._maybe_flush`` is what writes the LeRobot episode, and
        it fires on a terminated or truncated step. Rather than reach into its
        buffers, ask the wrapper below to report termination exactly once.
        """
        wrapper = self._gui_wrapper
        if wrapper is None or self._env is None:
            return
        # The frame this step produces is itself outside the take -- the
        # episode has already been closed by `end_episode` -- so it is gated
        # out by `pre_record` and only the termination flag survives.
        wrapper.force_terminate = True
        self._env.step(np.zeros((1, self._action_dim), dtype=np.float32))

    def _discard_episode(self) -> None:
        env = self._env
        if env is not None and hasattr(env, "_reset_env_buffer"):
            env._reset_env_buffer(0)


# --- info helpers ---------------------------------------------------------
def _flag(info: dict, key: str) -> bool:
    value = info.get(key)
    if value is None:
        return False
    return bool(np.asarray(value).any())


def _scalar(info: dict, key: str):
    value = info.get(key)
    if value is None:
        return None
    if isinstance(value, (list, tuple, np.ndarray)) and len(value):
        value = value[0]
    return None if value is None else str(value)


def _float(info: dict, key: str) -> float | None:
    value = info.get(key)
    if value is None:
        return None
    try:
        array = np.asarray(value, dtype=float).reshape(-1)
        return float(array[0]) if array.size else None
    except (TypeError, ValueError):
        return None
