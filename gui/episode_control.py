"""Episode boundaries from the GUI, recording intervals from the F3 pedal.

RLinf already has the machinery for both halves of this, it is just wired to a
three-pedal keyboard the PNP7 rig does not have. ``KeyboardStartEndWrapper``
publishes ``pre_record`` / ``record_reset`` / ``segment_advance`` into ``info``,
and ``CollectEpisode`` consumes them: a step with ``pre_record`` set is dropped
before it reaches the buffer, ``record_reset`` starts a fresh buffer, and
``segment_advance`` bumps a ``segment_id`` that is written per frame into the
LeRobot episode.

So the F3 clipping the operator asked for is not a new mechanism. It is that
mechanism with a different source of truth:

    pre_record = not (episode_open and F3_held)

``GelloJointIntervention`` already publishes ``info["gello_deadman_held"]`` on
every step, so nothing here has to open the pedal a second time -- which
matters, because the deadman is the one device where two readers really would
be a safety question.

Consequences worth being explicit about, because they shape the dataset:

* Frames recorded while F3 is released are **dropped, not flagged**. That is
  what "only the enabled intervals are valid" means, and it matches what
  ``build_episode.py`` already does for the old C++ pipeline.
* A released-then-re-held pedal leaves a *time* discontinuity in an otherwise
  continuous trajectory -- the robot does not move while F3 is up (the stream
  holds at the measured position) and re-engagement is refused unless GELLO and
  Franka agree within ``startup_max_error``, so the arm pose is continuous, but
  the wall-clock gap is not. Each interval therefore gets its own
  ``segment_id``, so a training pipeline can either honour the seams or ignore
  them, rather than having to guess where they were.

This wrapper deliberately lives in the PNP7 repo rather than in RLinf: it is
rig-specific glue, and the single-arm Franky/GELLO branch is headed upstream,
where a GUI-driven wrapper would only be noise.
"""
from __future__ import annotations

import math
import time
from typing import Any

import gymnasium as gym


class GuiEpisodeControl(gym.Wrapper):
    """Drive episode boundaries from the GUI and record gating from F3.

    Args:
        env: The wrapped single-arm env, at or above ``GelloJointIntervention``
            so that ``info["gello_deadman_held"]`` is present.
        require_deadman: Gate recording on the F3 pedal. Set ``False`` to record
            every frame of an open episode, which is occasionally what you want
            for a scripted or replayed take.
        segment_min_s: Ignore an F3 interval shorter than this. A pedal bounce
            or a flinch would otherwise open a one-frame segment and put a seam
            in the middle of a perfectly good demonstration.
    """

    def __init__(self, env: gym.Env, *, require_deadman: bool = True,
                 segment_min_s: float = 0.25, suppress_termination: bool = True):
        super().__init__(env)
        self.require_deadman = bool(require_deadman)
        self.segment_min_s = float(segment_min_s)
        self.suppress_termination = bool(suppress_termination)

        self._episode_open = False
        self._was_capturing = False
        self._pending_reset = False
        self._segment_started = -math.inf
        # One-shot: makes the next step report `terminated`, which is what
        # `CollectEpisode._maybe_flush` waits for before it writes the episode.
        self.force_terminate = False
        # Counters the GUI reads back for the operator; per-episode.
        self.steps_recorded = 0
        self.steps_skipped = 0
        self.segments = 0

    # --- called by the control loop, between steps -----------------------
    def begin_episode(self) -> None:
        """Open an episode. The next step publishes ``record_reset``."""
        self._episode_open = True
        self._was_capturing = False
        self._pending_reset = True
        self._segment_started = -math.inf
        self.steps_recorded = 0
        self.steps_skipped = 0
        self.segments = 0

    def end_episode(self) -> dict[str, Any]:
        """Close the episode and report what was captured."""
        result = {
            "steps": self.steps_recorded,
            "skipped": self.steps_skipped,
            "segments": self.segments,
        }
        self._episode_open = False
        self._was_capturing = False
        return result

    @property
    def capturing(self) -> bool:
        return self._episode_open and self._was_capturing

    # --- gym ---------------------------------------------------------------
    def reset(self, *, seed=None, options=None):
        self._episode_open = False
        self._was_capturing = False
        self._pending_reset = False
        return self.env.reset(seed=seed, options=options)

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)

        # The GUI owns episode boundaries, so the env's own termination must not
        # end a take underneath it. `max_episode_steps` truncation in particular
        # would otherwise cut a demonstration at 300 steps mid-motion. Setting
        # `max_episode_steps: null` and `manual_episode_control_only: true` in the
        # config is the supported way to say that; this is the belt to that
        # braces, and it preserves the container type because RealWorldEnv hands
        # these back as torch tensors that CollectEpisode goes on to slice.
        if self.force_terminate:
            # The operator pressed "stop & keep". Report the end of the episode
            # exactly once, so CollectEpisode flushes the buffer to LeRobot.
            self.force_terminate = False
            terminated = _ones_like_flag(terminated)
            truncated = _zeros_like_flag(truncated)
        elif self.suppress_termination:
            terminated = _zeros_like_flag(terminated)
            truncated = _zeros_like_flag(truncated)

        deadman = self._deadman_from_info(info)
        capturing = self._episode_open and (deadman or not self.require_deadman)

        record_reset = self._pending_reset
        self._pending_reset = False

        segment_advance = False
        if capturing and not self._was_capturing and not record_reset:
            # Re-engaged after a release: a new interval starts here.
            now = time.monotonic()
            if now - self._segment_started >= self.segment_min_s:
                segment_advance = True
                self.segments += 1
                self._segment_started = now
        elif capturing and record_reset:
            self._segment_started = time.monotonic()
            self.segments = 1

        if capturing:
            self.steps_recorded += 1
        elif self._episode_open:
            self.steps_skipped += 1

        self._was_capturing = capturing

        info["pre_record"] = not capturing
        info["record_reset"] = record_reset
        info["keyboard_phase"] = "rec" if capturing else "pre"
        info["keyboard_event"] = "start" if record_reset else None
        info["segment_advance"] = segment_advance
        # Read back by the GUI for the operator's status line.
        info["gui_capturing"] = capturing
        info["gui_steps_recorded"] = self.steps_recorded
        info["gui_steps_skipped"] = self.steps_skipped
        info["gui_segments"] = self.segments
        return obs, reward, terminated, truncated, info

    # --- helpers -----------------------------------------------------------
    @staticmethod
    def _deadman_from_info(info: dict[str, Any]) -> bool:
        """Read the pedal state ``GelloJointIntervention`` already published.

        It arrives as a length-1 array from the vector env, or as a bare bool
        when the wrapper is used unvectorised in a test.
        """
        value = info.get("gello_deadman_held")
        if value is None:
            # No GELLO wrapper below us (dummy env, replay). Treating that as
            # "not held" would silently record nothing at all, which is a far
            # worse failure than recording too much, so fail loudly instead.
            raise RuntimeError(
                "GuiEpisodeControl requires info['gello_deadman_held']; wrap it "
                "above GelloJointIntervention, or pass require_deadman=False."
            )
        if hasattr(value, "__len__") and not isinstance(value, (str, bytes)):
            return bool(value[0]) if len(value) else False
        return bool(value)


def _zeros_like_flag(flag):
    """Return a falsy value of the same shape and container type as *flag*.

    ``RealWorldEnv`` returns torch tensors, the raw gym stack returns numpy
    arrays, and an unvectorised test returns a bare bool. Replacing any of them
    with a plain ``False`` would break the caller's ``.unsqueeze`` / slicing.
    """
    if hasattr(flag, "zero_"):        # torch.Tensor
        return flag.new_zeros(flag.shape, dtype=flag.dtype)
    if hasattr(flag, "shape") and hasattr(flag, "dtype"):   # np.ndarray
        import numpy as np

        return np.zeros_like(flag)
    return False


def _ones_like_flag(flag):
    """The truthy counterpart of `_zeros_like_flag`, same container type."""
    if hasattr(flag, "new_ones"):      # torch.Tensor
        return flag.new_ones(flag.shape, dtype=flag.dtype)
    if hasattr(flag, "shape") and hasattr(flag, "dtype"):   # np.ndarray
        import numpy as np

        return np.ones_like(flag)
    return True
