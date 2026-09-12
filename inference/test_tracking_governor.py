"""Pure numerical tests; no robot, cameras, ROS or serial I/O."""
import unittest
import numpy as np
import pnp7_policy as old
from tracking_governor import TrackingGovernor, TrackingMonitor, apply_action, cap_lead, errors


class Tests(unittest.TestCase):
    def step(self, governor, lead=0, now=0, action=None, angle=0):
        return governor.step(np.array([lead, 0., 0.]), old.rpy_to_rotation([0, 0, angle]),
                             np.zeros(3), np.eye(3), action, now)

    def test_normal_action_unchanged(self):
        raw = np.array([.002, -.001, .003, .004, -.003, .002, .9])
        p, r, applied, info = self.step(TrackingGovernor(), action=raw)
        np.testing.assert_allclose(applied, raw)
        np.testing.assert_allclose(p, raw[:3])
        self.assertEqual(info["state"], "normal")

    def test_soft_band_halves_motion_not_gripper(self):
        raw = np.array([.002, 0, 0, 0, 0, .002, .8])
        p, r, applied, info = self.step(TrackingGovernor(), lead=.0225, action=raw)
        self.assertAlmostEqual(info["scale"], .5)
        self.assertAlmostEqual(p[0], .0235)
        self.assertEqual(applied[6], .8)
        self.assertAlmostEqual(info["discarded_translation_m"], .001)

    def test_candidate_cannot_create_large_target_backlog(self):
        p, _, applied, info = self.step(TrackingGovernor(), action=[.10, 0, 0, 0, 0, 0, 1])
        self.assertAlmostEqual(p[0], .030)
        self.assertAlmostEqual(info["scale"], .3)

    def test_pause_discards_action_including_gripper(self):
        governor = TrackingGovernor()
        p, r, applied, info = self.step(governor, lead=.031, action=[.01, 0, 0, 0, 0, 0, 0])
        self.assertIsNone(applied)
        self.assertEqual(info["state"], "paused")
        self.assertAlmostEqual(p[0], .031)
        self.assertAlmostEqual(info["discarded_translation_m"], .01)
        # At a later tick the robot has caught up. Only this NEW delta is used;
        # recovery is ramped, and none of the discarded 10 mm is replayed.
        p, r, applied, info = self.step(governor, now=.1, action=[.002, 0, 0, 0, 0, 0, 1])
        self.assertAlmostEqual(info["scale"], .2)
        self.assertAlmostEqual(p[0], .0004)

    def test_persistent_error_stops_even_without_new_actions(self):
        governor = TrackingGovernor()
        self.step(governor, lead=.020, now=10)
        self.step(governor, lead=.020, now=10.99)
        with self.assertRaisesRegex(old.SafetyError, "persistent"):
            self.step(governor, lead=.020, now=11)
        self.assertEqual(governor.last_info["state"], "persistent_stop")

    def test_hysteresis_prevents_timer_reset_by_threshold_chatter(self):
        governor = TrackingGovernor()
        self.step(governor, lead=.016, now=10)
        self.step(governor, lead=.014, now=10.5)
        with self.assertRaisesRegex(old.SafetyError, "persistent"):
            self.step(governor, lead=.014, now=11)
        governor = TrackingGovernor()
        self.step(governor, lead=.016, now=10)
        self.step(governor, lead=.011, now=10.5)
        self.step(governor, lead=.016, now=11)
        self.assertEqual(governor.last_info["deviation_duration_s"], 0)

    def test_hard_error_still_stops_immediately(self):
        for distance, angle in ((.0451, 0), (0, .3001)):
            governor = TrackingGovernor()
            with self.assertRaisesRegex(old.SafetyError, "target tracking error limit"):
                self.step(governor, lead=distance, angle=angle, action=[-.001, 0, 0, 0, 0, 0, 1])
            self.assertEqual(governor.last_info["state"], "hard_stop")

    def test_rotation_soft_band_and_pause(self):
        _, _, _, info = self.step(TrackingGovernor(), angle=.15, action=[0, 0, 0, 0, 0, .002, 1])
        self.assertAlmostEqual(info["scale"], .5)
        _, _, applied, info = self.step(TrackingGovernor(), angle=.21, action=[0, 0, 0, 0, 0, .002, 1])
        self.assertIsNone(applied)

    def test_bad_time_and_nonfinite_inputs_fail_closed(self):
        governor = TrackingGovernor()
        self.step(governor, now=1)
        with self.assertRaises(old.SafetyError):
            self.step(governor, now=.9)
        for kwargs in (dict(now=float("nan")), dict(lead=float("nan")), dict(action=[float("nan")] * 7)):
            with self.assertRaises(old.SafetyError):
                self.step(TrackingGovernor(), **kwargs)

    def test_random_candidate_pose_stays_inside_hold_envelope(self):
        random = np.random.default_rng(42)
        for _ in range(300):
            position = random.uniform(-.01, .01, 3)
            rotation = old.rpy_to_rotation(random.uniform(-.05, .05, 3))
            raw = random.uniform(-.05, .05, 7)
            p, r, _, _ = TrackingGovernor().step(position, rotation, np.zeros(3), np.eye(3), raw, 0)
            distance, angle = errors(p, r, np.zeros(3), np.eye(3))
            self.assertLessEqual(distance, .030000001)
            self.assertLessEqual(angle, .200000001)

    def test_apply_action_is_unmodified_delta(self):
        action = np.array([.07, -.02, .03, .1, -.2, .3, 1])  # far beyond the governor envelope
        start = old.rpy_to_rotation([0, .1, 0])
        p, r = apply_action(np.array([.5, 0, .4]), start, action)
        np.testing.assert_allclose(p, [.57, -.02, .43])
        np.testing.assert_allclose(r, start @ old.rpy_to_rotation(action[3:6]))

    def test_monitor_reports_levels_without_stopping(self):
        monitor = TrackingMonitor()
        seen = []
        for lead in (.010, .046, .040, .091, .080, .070, .030, .500):
            info, changed = monitor.step(np.array([lead, 0., 0.]), np.eye(3), np.zeros(3), np.eye(3))
            seen.append((info["level"], changed))
        self.assertEqual(seen, [(0, False), (1, True), (1, False), (2, True), (2, False),
                                (1, True), (0, True), (2, True)])
        info, changed = TrackingMonitor().step(np.zeros(3), old.rpy_to_rotation([0, 0, .61]),
                                               np.zeros(3), np.eye(3))
        self.assertEqual((info["level"], changed), (2, True))
        with self.assertRaises(old.SafetyError):
            TrackingMonitor().step(np.array([float("nan"), 0, 0]), np.eye(3), np.zeros(3), np.eye(3))

    def test_lead_cap_keeps_direction_and_axis_and_drops_excess(self):
        def axis(m):
            v = np.array([m[2, 1] - m[1, 2], m[0, 2] - m[2, 0], m[1, 0] - m[0, 1]])
            return v / np.linalg.norm(v)
        relative = old.rpy_to_rotation([.3, -.2, .1])
        measured_r = old.rpy_to_rotation([0, .1, 0])
        p, r, dropped_m, dropped_rad = cap_lead(np.array([.09, 0, -.12]), measured_r @ relative,
                                                np.zeros(3), measured_r, .02, .10)
        np.testing.assert_allclose(p, [.012, 0, -.016])
        self.assertAlmostEqual(dropped_m, .13)
        self.assertAlmostEqual(errors(p, r, np.zeros(3), measured_r)[1], .10)
        self.assertAlmostEqual(dropped_rad, old.rotation_angle(relative) - .10)
        np.testing.assert_allclose(axis(measured_r.T @ r), axis(relative), atol=1e-9)
        p0, r0 = np.array([.01, .005, 0]), old.rpy_to_rotation([0, 0, .05])
        p, r, dropped_m, dropped_rad = cap_lead(p0, r0, np.zeros(3), np.eye(3), .02, .10)
        np.testing.assert_allclose(p, p0)
        np.testing.assert_allclose(r, r0)
        self.assertEqual((dropped_m, dropped_rad), (0.0, 0.0))

    def test_random_capped_targets_stay_on_leash(self):
        random = np.random.default_rng(7)
        for _ in range(300):
            measured_r = old.rpy_to_rotation(random.uniform(-1, 1, 3))
            p, r, _, _ = cap_lead(random.uniform(-.2, .2, 3), old.rpy_to_rotation(random.uniform(-1, 1, 3)),
                                  np.zeros(3), measured_r, .02, .10)
            distance, angle = errors(p, r, np.zeros(3), measured_r)
            self.assertLessEqual(distance, .020000001)
            self.assertLessEqual(angle, .100000001)

    def test_nonfinite_target_passes_cap_then_monitor_fails_closed(self):
        p, r, _, _ = cap_lead(np.array([float("nan"), 0, 0]), np.eye(3), np.zeros(3), np.eye(3), .02, .10)
        with self.assertRaises(old.SafetyError):
            TrackingMonitor().step(p, r, np.zeros(3), np.eye(3))


if __name__ == "__main__":
    unittest.main()
