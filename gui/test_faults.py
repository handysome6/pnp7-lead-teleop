"""Offline regression tests for errors that used to look like a silent stop."""
from __future__ import annotations

import contextlib
import io
import json
import os
import signal
import subprocess
import sys
import tempfile
import unittest
import urllib.request
from pathlib import Path
from unittest.mock import patch

from gui.legacy_backend import LegacyBackend, _Proc
from gui.server import serve
from gui.session import Command, ControlLoop, Mode, Phase, Rejected


ROBOT_ERROR = ('franka error: libfranka: Move command aborted: motion aborted by reflex! '
               '["joint_motion_generator_velocity_discontinuity", '
               '"joint_motion_generator_acceleration_discontinuity"]\n'
               'control_command_success_rate: 1')


class FaultTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.backend = LegacyBackend(status_path=self.root / "status.json",
                                     scratch_dir=self.root)
        self.addCleanup(self.backend.close_session)

    def test_robotiq_raw_status_is_not_reported_as_metres(self):
        self.status(held=True, state=1)
        value = json.loads(self.backend.status_path.read_text())
        value.update(gripper_width=-1, gripper_position_raw=128,
                     gripper_requested_raw=255, gripper_fault=0)
        self.backend.status_path.write_text(json.dumps(value))
        result = self.backend.step(False)
        self.assertIsNone(result["gripper_width"])
        self.assertEqual(result["gripper_position_raw"], 128)
        self.assertEqual(result["gripper_requested_raw"], 255)
        self.assertEqual(result["gripper_fault"], 0)

    def test_robotiq_counts_cannot_be_exported_as_metres(self):
        from collect.export_lerobot import frame_state, frame_action
        with self.assertRaisesRegex(ValueError, "Robotiq"):
            frame_state({"gripper_position_raw": "0"})
        with self.assertRaisesRegex(ValueError, "Robotiq"):
            frame_action({"gripper_requested_raw": "255"})

    def exited_bridge(self, code=1, detail=ROBOT_ERROR, fresh_status=False):
        proc = _Proc("bridge", [sys.executable, "-c",
                     f"import sys; print({detail!r},file=sys.stderr); sys.exit({code})"],
                     log_path=self.root / "bridge.log")
        self.backend._bridge = proc
        proc.proc.wait(timeout=5)
        if fresh_status:
            self.status(held=True, state=1)
        return proc

    def status(self, held, state):
        self.backend.status_path.write_text(json.dumps(
            {"deadman": int(held), "state": state, "lead_age_ms": 1, "q": [0] * 7}))

    def test_franka_fault_includes_actual_error_exit_code_and_log(self):
        self.exited_bridge()
        with self.assertRaises(RuntimeError) as raised:
            self.backend.step(False)
        text = str(raised.exception)
        self.assertIn("退出码 1", text)
        self.assertIn("velocity_discontinuity", text)
        self.assertIn("acceleration_discontinuity", text)
        self.assertIn(str(self.root / "bridge.log"), text)
        self.assertNotIn("duration reached", text)

    def test_fresh_status_cannot_hide_a_dead_process(self):
        self.exited_bridge(fresh_status=True)
        with self.assertRaisesRegex(RuntimeError, "Franka"):
            self.backend.step(False)

    def test_deadline_guard_has_a_specific_reason_and_stays_a_fault(self):
        self.exited_bridge(detail='error: FCI timing guard: host_gap_ms=6.7 robot_period_ms=1')
        loop, _ = self.fault_loop()
        self.assertEqual(loop.state.phase, Phase.ERROR)
        self.assertIn('FCI 控制回调超时', loop.state.error)
        self.assertIn('host_gap_ms=6.7', loop.state.error)

    def test_signal_exit_is_an_error(self):
        proc = _Proc("bridge", [sys.executable, "-c", "import time; time.sleep(30)"],
                     log_path=self.root / "signal.log")
        self.backend._bridge = proc
        os.kill(proc.proc.pid, signal.SIGKILL)
        proc.proc.wait(timeout=5)
        with self.assertRaisesRegex(RuntimeError, "退出码 -9"):
            self.backend.step(False)

    def test_home_pedal_preflight_remains_a_nonfatal_guard(self):
        self.backend._config_path = self.root / "demo.conf"
        result = subprocess.CompletedProcess([], 1, "", "error: home refused: the foot brake is held")
        with patch("gui.legacy_backend.subprocess.run", return_value=result):
            with self.assertRaises(Rejected):
                self.backend.restore_joints()

    def test_home_fault_keeps_error_before_diagnostic_file_line(self):
        self.backend._config_path = self.root / "demo.conf"
        result = subprocess.CompletedProcess([], 1, "", ROBOT_ERROR + "\nFranka diagnostic log: /tmp/fault.csv")
        with patch("gui.legacy_backend.subprocess.run", return_value=result):
            with self.assertRaisesRegex(RuntimeError, "velocity_discontinuity"):
                self.backend.restore_joints()

    def test_normal_exit_still_finishes_a_recording(self):
        self.exited_bridge(code=0, detail="teleop finished", fresh_status=True)
        telemetry = self.backend.step(True)
        self.assertTrue(telemetry["episode_finished"])
        self.assertFalse(telemetry["stream_enabled"])

    def test_paused_while_pedal_held_has_a_reason(self):
        self.status(held=True, state=2)
        telemetry = self.backend.step(False)
        self.assertFalse(telemetry["stream_enabled"])
        self.assertIn("GELLO", telemetry["block_reason"])

    def fault_loop(self, recording=False):
        loop = ControlLoop(self.backend)
        loop.state.phase = Phase.RECORDING if recording else Phase.READY
        loop.state.mode = Mode.COLLECT if recording else Mode.TELEOP
        loop.state.stream_enabled = True
        loop.state.deadman_held = True
        stderr = io.StringIO()
        with contextlib.redirect_stderr(stderr):
            loop._step_once()
        return loop, stderr.getvalue()

    def test_fault_is_latched_and_printed_to_gui_terminal(self):
        self.exited_bridge(fresh_status=True)
        loop, terminal = self.fault_loop()
        self.assertEqual(loop.snapshot().phase, Phase.ERROR)
        self.assertFalse(loop.snapshot().stream_enabled)
        self.assertIn("velocity_discontinuity", loop.snapshot().error)
        self.assertIn("velocity_discontinuity", terminal)
        self.assertIsNone(self.backend._bridge)
        self.assertTrue((self.root / "bridge.log").is_file())
        loop._apply(Command("close_session"))
        self.assertEqual(loop.snapshot().phase, Phase.IDLE)

    def test_failed_recording_is_preserved_and_marked_unusable(self):
        self.exited_bridge()
        episode = self.root / "ep001"
        episode.mkdir()
        original = b"partial camera data"
        (episode / "camera.raw").write_bytes(original)
        self.backend._episode_dir = episode
        loop, _ = self.fault_loop(recording=True)
        self.assertEqual(loop.snapshot().phase, Phase.ERROR)
        self.assertEqual((episode / "camera.raw").read_bytes(), original)
        failure = json.loads((episode / "failure.json").read_text())
        self.assertFalse(failure["usable_episode"])
        self.assertIn("velocity_discontinuity", failure["error"])

    def collision_take(self, errors='["cartesian_reflex"]'):
        episode = self.root / "ep009"
        episode.mkdir()
        (episode / "camera.raw").write_bytes(b"invalid take")
        detail = ('franka error: libfranka: Move command aborted: motion aborted by reflex! '
                  + errors + '\ncontrol_command_success_rate: 1')
        self.exited_bridge(detail=detail, fresh_status=True)
        (episode / "bridge.log").write_text(detail)
        self.backend._episode_dir = episode
        self.backend._config_path = self.root / "demo.conf"
        self.backend._mode = Mode.COLLECT
        loop = ControlLoop(self.backend)
        loop.state.phase, loop.state.mode = Phase.RECORDING, Mode.COLLECT
        loop.state.episode_index = 8
        loop.state.recorded_steps = 71
        loop.state.stream_enabled = loop.state.deadman_held = True
        return loop, episode

    def test_collision_discards_only_current_take_returns_ready_and_keeps_diagnostic(self):
        loop, episode = self.collision_take()
        good = self.root / "ep008"
        good.mkdir()
        (good / "camera.raw").write_bytes(b"good")
        with patch.object(self.backend, "open_preview") as preview, \
             patch.object(self.backend, "_start_bridge") as start, \
             contextlib.redirect_stderr(io.StringIO()):
            loop._step_once()
        self.assertEqual(loop.state.phase, Phase.READY)
        self.assertIsNone(loop.state.error)
        self.assertEqual(loop.state.episode_index, 8)
        self.assertEqual(loop.state.recorded_steps, 0)
        self.assertFalse(loop.state.stream_enabled)
        self.assertFalse(loop.state.deadman_held)
        self.assertFalse(episode.exists())
        self.assertEqual((good / "camera.raw").read_bytes(), b"good")
        self.assertIsNotNone(self.backend._config_path)
        preview.assert_called_once()
        start.assert_not_called()
        diagnostic, = self.root.glob('pnp7_discarded_ep009_*')
        self.assertIn('cartesian_reflex', (diagnostic / 'bridge.log').read_text())
        self.assertTrue(json.loads((diagnostic / 'failure.json').read_text())['discarded'])
        with patch.object(self.backend, "begin_episode") as begin:
            loop._apply(Command('begin_episode'))
            begin.assert_called_once()
        self.assertEqual(loop.state.phase, Phase.RECORDING)

    def test_keep_before_health_poll_cannot_save_collision_take(self):
        loop, episode = self.collision_take()
        command = loop.submit('end_episode')
        with patch.object(self.backend, "open_preview"), \
             patch.object(self.backend, "_build_and_validate") as build, \
             contextlib.redirect_stderr(io.StringIO()):
            loop._drain_commands()
        self.assertIsNone(command.error)
        self.assertEqual(loop.state.phase, Phase.READY)
        self.assertFalse(episode.exists())
        build.assert_not_called()

    def test_keep_cannot_save_other_faults_either(self):
        loop, episode = self.collision_take('["joint_motion_generator_acceleration_discontinuity"]')
        command = loop.submit('end_episode')
        with patch.object(self.backend, "_build_and_validate") as build, \
             contextlib.redirect_stderr(io.StringIO()):
            loop._drain_commands()
        self.assertIsNotNone(command.error)
        self.assertEqual(loop.state.phase, Phase.ERROR)
        self.assertTrue(episode.exists())
        self.assertTrue((episode / 'failure.json').exists())
        build.assert_not_called()

    def test_mixed_reflex_errors_remain_latched_faults(self):
        loop, episode = self.collision_take('["cartesian_reflex", "communication_constraints_violation"]')
        with contextlib.redirect_stderr(io.StringIO()):
            loop._step_once()
        self.assertEqual(loop.state.phase, Phase.ERROR)
        self.assertTrue(episode.exists())
        self.assertTrue((episode / 'failure.json').exists())

    def test_teleop_collision_does_not_resume_automatically(self):
        self.exited_bridge(detail='franka error: libfranka: Move command aborted: '
                          'motion aborted by reflex! ["cartesian_reflex"]')
        self.backend._mode = Mode.TELEOP
        loop, _ = self.fault_loop()
        self.assertEqual(loop.state.phase, Phase.ERROR)
        self.assertIsNone(self.backend._bridge)

    def test_discard_failure_is_not_reported_as_ready(self):
        loop, episode = self.collision_take()
        with patch('gui.legacy_backend.shutil.rmtree', side_effect=OSError('disk failure')), \
             contextlib.redirect_stderr(io.StringIO()):
            loop._step_once()
        self.assertEqual(loop.state.phase, Phase.ERROR)
        self.assertIn('disk failure', loop.state.error)
        self.assertTrue(episode.exists())

    def test_preview_failure_after_discard_stays_visible(self):
        loop, episode = self.collision_take()
        with patch.object(self.backend, 'open_preview', side_effect=RuntimeError('camera failed')), \
             contextlib.redirect_stderr(io.StringIO()):
            loop._step_once()
        self.assertEqual(loop.state.phase, Phase.ERROR)
        self.assertIn('camera failed', loop.state.error)
        self.assertFalse(episode.exists())

    def test_collision_classifier_requires_exact_sdk_error_lists(self):
        prefix = 'franka error: libfranka: Move command aborted: motion aborted by reflex! '
        for errors in ('["cartesian_reflex"]', '["joint_reflex"]',
                       '["cartesian_reflex", "joint_reflex"]'):
            self.assertTrue(self.backend._collision_reflex_only(prefix + errors))
        for detail in ('cartesian_reflex', prefix + '[]', prefix + '[garbage]',
                       prefix + '["cartesian_reflex", "other"]',
                       prefix + '["cartesian_reflex"] / Franka fault onset: errors=["other"]'):
            self.assertFalse(self.backend._collision_reflex_only(detail))

    def test_failed_startup_keeps_logs_instead_of_deleting_take(self):
        config = self.root / "demo.conf"
        config.write_text("lead_port=/dev/unused\n")
        recorder = self.root / "recorder.py"
        recorder.write_text("import sys; sys.exit('injected camera startup failure')\n")
        self.backend._config_path = config
        self.backend.episodes_dir = self.root / "episodes"
        self.backend.recorder = recorder
        self.backend.preview_dir = self.root / "preview"
        loop = ControlLoop(self.backend)
        loop.state.phase = Phase.READY
        cmd = loop.submit("begin_episode")
        with contextlib.redirect_stderr(io.StringIO()):
            loop._drain_commands()
        self.assertIsNotNone(cmd.error)
        self.assertEqual(loop.snapshot().phase, Phase.ERROR)
        episode = self.backend.episodes_dir / "ep001"
        self.assertTrue((episode / "config.conf").exists())
        self.assertIn("injected camera startup failure", (episode / "cameras.log").read_text())
        self.assertTrue((episode / "failure.json").exists())

    def test_http_state_exposes_fault_to_real_page(self):
        self.exited_bridge()
        loop, _ = self.fault_loop()
        server = serve(loop, host="127.0.0.1", port=0)
        self.addCleanup(server.server_close)
        self.addCleanup(server.shutdown)
        base = f"http://127.0.0.1:{server.server_address[1]}"
        with urllib.request.urlopen(base + "/api/state") as response:
            state = json.load(response)
        self.assertEqual(state["phase"], "error")
        self.assertIn("acceleration_discontinuity", state["error"])
        self.assertFalse(state["stream_enabled"])
        with urllib.request.urlopen(base + "/") as response:
            html = response.read().decode()
        self.assertIn('role="alert"', html)
        self.assertIn('s.phase !== "error"', html)


if __name__ == "__main__":
    unittest.main()
