"""Bound target lead without buffering unexecuted policy deltas.

This is a command governor, not a certified safety controller. Hard tracking
limits are unchanged. The opt-in diagnostic alternative is cap_lead (a leash to
the measured pose) plus TrackingMonitor, which records and reports errors but
never stops. No ROS, serial, socket or robot writes occur here.
"""
import numpy as np
import pnp7_policy as old

TRACKING_POSITION_LIMIT = .045
TRACKING_ROTATION_LIMIT = .30
# policy_watchdog.h: beyond these the controller ignores targets and holds.
CONTROLLER_POSITION_LIMIT = .09
CONTROLLER_ROTATION_LIMIT = .60
SOFT_POSITION = .015
SOFT_ROTATION = .10
HOLD_POSITION = .030
HOLD_ROTATION = .20
RESET_POSITION = .012
RESET_ROTATION = .08
PERSISTENCE_SECONDS = 1.0
RECOVERY_SECONDS = .5


def errors(position, rotation, measured, measured_rotation):
    distance = float(np.linalg.norm(position - measured))
    angle = old.rotation_angle(measured_rotation.T @ rotation)
    if not np.isfinite(distance + angle):
        raise old.SafetyError("nonfinite target tracking error")
    return distance, angle


def check_tracking(position, rotation, measured, measured_rotation):
    distance, angle = errors(position, rotation, measured, measured_rotation)
    if distance > TRACKING_POSITION_LIMIT or angle > TRACKING_ROTATION_LIMIT:
        raise old.SafetyError("target tracking error limit: translation_mm={:.2f}, rotation_rad={:.4f}".format(
            1000 * distance, angle))


class TrackingGovernor:
    settings = dict(soft_position_m=SOFT_POSITION, soft_rotation_rad=SOFT_ROTATION,
                    hold_position_m=HOLD_POSITION, hold_rotation_rad=HOLD_ROTATION,
                    hard_position_m=TRACKING_POSITION_LIMIT, hard_rotation_rad=TRACKING_ROTATION_LIMIT,
                    reset_position_m=RESET_POSITION, reset_rotation_rad=RESET_ROTATION,
                    persistence_s=PERSISTENCE_SECONDS, recovery_s=RECOVERY_SECONDS)

    def __init__(self):
        self.scale = 1.0
        self.last_time = None
        self.exceeded_since = None
        self.last_info = None

    def step(self, position, rotation, measured, measured_rotation, action, now):
        if not np.isfinite(now) or (self.last_time is not None and now < self.last_time):
            raise old.SafetyError("invalid governor monotonic time")
        dt = 0 if self.last_time is None else now - self.last_time
        self.last_time = now
        distance, angle = errors(position, rotation, measured, measured_rotation)
        if distance >= SOFT_POSITION or angle >= SOFT_ROTATION:
            if self.exceeded_since is None:
                self.exceeded_since = now
        elif distance <= RESET_POSITION and angle <= RESET_ROTATION:
            self.exceeded_since = None
        elapsed = 0 if self.exceeded_since is None else now - self.exceeded_since
        info = dict(position_error_m=distance, rotation_error_rad=angle,
                    deviation_duration_s=elapsed, scale=self.scale, state="normal",
                    has_action=action is not None, discarded_translation_m=0.0,
                    discarded_rpy_norm_rad=0.0)
        self.last_info = info
        if distance > TRACKING_POSITION_LIMIT or angle > TRACKING_ROTATION_LIMIT:
            info["state"] = "hard_stop"
            check_tracking(position, rotation, measured, measured_rotation)
        if self.exceeded_since is not None and elapsed >= PERSISTENCE_SECONDS:
            info["state"] = "persistent_stop"
            raise old.SafetyError("persistent tracking deviation for {:.3f}s: {:.2f} mm / {:.4f} rad".format(
                elapsed, 1000 * distance, angle))

        # C1-continuous easing in the soft band. Reductions act immediately;
        # recovery is slew-limited, so catching up cannot unleash a target burst.
        u = np.clip(max((distance - SOFT_POSITION) / (HOLD_POSITION - SOFT_POSITION),
                        (angle - SOFT_ROTATION) / (HOLD_ROTATION - SOFT_ROTATION)), 0, 1)
        desired = float(1 - 3 * u * u + 2 * u * u * u)
        scale = min(desired, self.scale + dt / RECOVERY_SECONDS)
        applied = None
        new_position, new_rotation = position.copy(), rotation.copy()
        if action is not None:
            action = np.asarray(action, dtype=float)
            if action.shape != (7,) or not np.isfinite(action).all():
                raise old.SafetyError("invalid governor action")
            dp = float(np.linalg.norm(action[:3]))
            # sum(abs(RPY)) bounds the composed rotation angle by triangle
            # inequality, including for scaled RPY (angle itself is nonlinear).
            dr_bound = float(np.abs(action[3:6]).sum())
            if dp > 0:
                scale = min(scale, max(0, HOLD_POSITION - distance) / dp)
            if dr_bound > 0:
                scale = min(scale, max(0, HOLD_ROTATION - angle) / dr_bound)
            if scale > 1e-6:
                applied = action.copy()
                applied[:6] *= scale
                new_position += applied[:3]
                new_rotation = rotation @ old.rpy_to_rotation(applied[3:6])
                check_tracking(new_position, new_rotation, measured, measured_rotation)
            else:
                scale = 0.0
            info["discarded_translation_m"] = (1 - scale) * dp
            info["discarded_rpy_norm_rad"] = (1 - scale) * float(np.linalg.norm(action[3:6]))
        self.scale = scale
        info.update(scale=scale, state="paused" if scale <= 1e-6 else "slowing" if scale < .999999 else "normal")
        return new_position, new_rotation, applied, info


def apply_action(position, rotation, action):
    """The unmodified policy delta: base-frame translation, local-frame RPY."""
    return position + action[:3], rotation @ old.rpy_to_rotation(action[3:6])


def cap_lead(position, rotation, measured, measured_rotation, max_position, max_rotation):
    """Leash a target to the measured pose, keeping lead direction and rotation axis.

    Excess lead is dropped, never buffered. With a force-clipped impedance
    controller it adds no pull, only windup: a lagging arm keeps the policy
    commanding motion the arm has not made yet.
    """
    lead = position - measured
    distance = float(np.linalg.norm(lead))
    dropped_position = max(0.0, distance - max_position)
    if dropped_position > 0:
        position = measured + lead * (max_position / distance)
    x, y, z, w = old.matrix_to_quaternion(measured_rotation.T @ rotation)
    axis = np.array([x, y, z]) * np.sign(w or 1)
    angle = 2 * float(np.arctan2(np.linalg.norm(axis), abs(w)))
    dropped_rotation = max(0.0, angle - max_rotation)
    if dropped_rotation > 0:
        k = axis / np.linalg.norm(axis)
        skew = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        rotation = measured_rotation @ (np.eye(3) + np.sin(max_rotation) * skew +
                                        (1 - np.cos(max_rotation)) * skew @ skew)
    return position, rotation, dropped_position, dropped_rotation


class TrackingMonitor:
    """Diagnostic only: record tracking error and report crossings; never stops.

    Level 1 is beyond the client hard limit. Level 2 is beyond the controller-side
    limit, where SERL ignores new targets and holds the measured pose. A level
    drops only below 80% of its threshold, so staircase targets do not flood logs.
    """
    limits = ((TRACKING_POSITION_LIMIT, TRACKING_ROTATION_LIMIT),
              (CONTROLLER_POSITION_LIMIT, CONTROLLER_ROTATION_LIMIT))

    def __init__(self):
        self.level = 0

    def _beyond(self, level, distance, angle, factor):
        position_limit, rotation_limit = self.limits[level - 1]
        return distance > factor * position_limit or angle > factor * rotation_limit

    def step(self, position, rotation, measured, measured_rotation):
        distance, angle = errors(position, rotation, measured, measured_rotation)
        previous = self.level
        while self.level < len(self.limits) and self._beyond(self.level + 1, distance, angle, 1):
            self.level += 1
        while self.level > 0 and not self._beyond(self.level, distance, angle, .8):
            self.level -= 1
        return dict(position_error_m=distance, rotation_error_rad=angle,
                    level=self.level), self.level != previous
