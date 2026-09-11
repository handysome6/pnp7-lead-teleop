"""Offline Save Home regression tests; all config writes use temporary files.

Run with: pixi run python -m unittest gui.test_home
"""
from __future__ import annotations

import json
import os
import stat
import sys
import tempfile
import unittest
import urllib.error
import urllib.request
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from gui.home_config import save_current_home
from gui.legacy_backend import LegacyBackend
from gui.mock import MockBackend
from gui.server import serve
from gui.session import Command, ControlLoop, Mode, Phase, Rejected

Q = [0.1, -0.4, 0.2, -2.2, 0.3, 1.9, 0.8]


class SaveHomeTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.conf = self.root / "conf"
        self.conf.mkdir()
        self.config = self.conf / "demo.conf"
        self.original = b"# pose\r\nrobot_ip=172.16.0.2\r\n  home_qpos = 0 0 0 -2 0 1 0  # keep comment\r\nscale=0.25\r\n"
        self.config.write_bytes(self.original)
        self.config.chmod(0o640)
        self.stub = self.root / "read_pose.py"
        self.stub.write_text(
            f"#!{sys.executable}\nimport sys\n"
            "assert sys.argv[1] == 'read-home', 'a motion command was issued'\n"
            f"print('HOME_QPOS {json.dumps(Q)}')\n"
        )
        self.stub.chmod(0o755)
        self.backend = LegacyBackend(bridge=self.stub, conf_dir=self.conf)

    def test_updates_selected_config_preserves_comments_permissions_and_backup(self):
        other = self.conf / "other.conf"
        other.write_text("home_qpos=unchanged\n")
        result = self.backend.save_home("demo")
        actual = self.config.read_bytes()
        self.assertEqual(result["q"], Q)
        self.assertEqual(result["config_name"], "demo")
        self.assertTrue(actual.startswith(b"# pose\r\nrobot_ip=172.16.0.2\r\n  home_qpos = "))
        self.assertTrue(actual.endswith(b"  # keep comment\r\nscale=0.25\r\n"))
        self.assertEqual(Path(result["backup"]).read_bytes(), self.original)
        self.assertEqual(stat.S_IMODE(self.config.stat().st_mode), 0o640)
        self.assertEqual(other.read_text(), "home_qpos=unchanged\n")

    def test_appends_missing_home_without_merging_previous_line(self):
        self.config.write_text("scale=0.25")
        save_current_home(self.config, lambda: Q)
        self.assertTrue(self.config.read_text().startswith("scale=0.25\nhome_qpos="))

    def test_updates_duplicate_active_keys_leaves_commented_key(self):
        self.config.write_text("# home_qpos=old\nhome_qpos=first\nhome_qpos=last # note\n")
        save_current_home(self.config, lambda: Q)
        lines = self.config.read_text().splitlines()
        self.assertEqual(lines[0], "# home_qpos=old")
        self.assertEqual(lines[1], lines[2].split(" #", 1)[0])

    def test_invalid_or_missing_pose_never_changes_config(self):
        for q in ([], Q[:6], Q + [0], [float('nan')] + Q[1:],
                  [float('inf')] + Q[1:], [True] + Q[1:], ["0"] + Q[1:], {}):
            with self.subTest(q=q), self.assertRaises(Rejected):
                save_current_home(self.config, lambda: q)
            self.assertEqual(self.config.read_bytes(), self.original)
        self.assertFalse(self.config.with_name("demo.conf.home.bak").exists())

    def test_failed_read_keeps_config_unchanged(self):
        self.stub.write_text(f"#!{sys.executable}\nimport sys\nsys.exit('FCI unavailable')\n")
        with self.assertRaisesRegex(Rejected, "FCI unavailable"):
            self.backend.save_home("demo")
        self.assertEqual(self.config.read_bytes(), self.original)

    def test_concurrent_edit_is_not_overwritten(self):
        edited = b"# changed by operator\nscale=0.4\n"
        def read_q():
            self.config.write_bytes(edited)
            return Q
        with self.assertRaises(Rejected):
            save_current_home(self.config, read_q)
        self.assertEqual(self.config.read_bytes(), edited)

    def test_atomic_replace_failure_keeps_original(self):
        replace = os.replace
        def fail_config(source, target):
            if Path(target) == self.config:
                raise OSError("disk failure")
            return replace(source, target)
        with patch("gui.home_config.os.replace", side_effect=fail_config):
            with self.assertRaises(OSError):
                save_current_home(self.config, lambda: Q)
        self.assertEqual(self.config.read_bytes(), self.original)
        self.assertFalse(list(self.conf.glob(".home-*")))

    def test_refuses_path_traversal_symlink_and_wrong_session_config(self):
        for name in ("../demo", "/tmp/demo", "", None):
            with self.subTest(name=name), self.assertRaises(Rejected):
                self.backend.save_home(name)
        (self.conf / "linked.conf").symlink_to(self.config)
        with self.assertRaises(Rejected):
            self.backend.save_home("linked")
        self.backend._config_path = self.conf / "other.conf"
        with self.assertRaises(Rejected):
            self.backend.save_home("demo")

    def test_refuses_active_bridge_and_dry_run(self):
        self.backend._bridge = SimpleNamespace(alive=True)
        with self.assertRaises(Rejected):
            self.backend.save_home("demo")
        self.backend._bridge = None
        self.backend._config_path = self.config
        self.backend._dry_run = True
        with self.assertRaises(Rejected):
            self.backend.save_home("demo")
        self.assertEqual(self.config.read_bytes(), self.original)

    def test_recording_and_opening_guards_do_not_call_backend(self):
        loop = ControlLoop(self.backend)
        for phase, mode in ((Phase.RECORDING, Mode.COLLECT),
                            (Phase.OPENING, Mode.COLLECT)):
            loop.state.phase, loop.state.mode = phase, mode
            with patch.object(self.backend, "save_home") as save:
                with self.assertRaises(Rejected):
                    loop._apply(Command("save_home", {"config_name": "demo"}))
                save.assert_not_called()
            self.assertEqual(loop.state.phase, phase)

    def teleop(self, held=0, state=2):
        self.backend._mode = Mode.TELEOP
        self.backend._config_path = self.config
        bridge = Mock(alive=True)
        bridge.proc.poll.return_value = 0
        self.backend._bridge = bridge
        events = []
        def stop():
            events.append("stop")
            self.backend._bridge = None
        def start(log_csv):
            self.assertIsNone(log_csv)
            events.append("start")
            self.backend._bridge = bridge
        self.stop = patch.object(self.backend, "_stop_bridge", side_effect=stop).start()
        self.start = patch.object(self.backend, "_start_bridge", side_effect=start).start()
        self.status = patch.object(self.backend, "_read_status", return_value={
            "deadman": held, "state": state}).start()
        self.addCleanup(patch.stopall)
        self.backend._viewfinder = object()
        return events, bridge

    def test_teleop_save_reads_measured_pose_after_stop_then_resumes(self):
        events, _ = self.teleop()
        preview = self.backend._viewfinder
        run = __import__("subprocess").run
        def read(*args, **kwargs):
            self.assertIsNone(self.backend._bridge)
            events.append("read")
            return run(*args, **kwargs)
        with patch("gui.legacy_backend.subprocess.run", side_effect=read):
            result = self.backend.save_home("demo")
        self.assertEqual(result["q"], Q)
        self.assertEqual(events, ["stop", "read", "start"])
        self.assertIs(self.backend._viewfinder, preview)
        self.assertIsNotNone(self.backend._bridge)

    def test_teleop_restore_yields_fci_and_resumes(self):
        events, _ = self.teleop()
        def restore():
            self.assertIsNone(self.backend._bridge)
            events.append("home")
        with patch.object(self.backend, "_restore_home", side_effect=restore):
            self.backend.restore_joints()
        self.assertEqual(events, ["stop", "home", "start"])

    def test_fresh_backend_pedal_guard_overrides_stale_gui(self):
        events, _ = self.teleop()
        for status in (None, {}, {"deadman": 1, "state": 2},
                       {"deadman": 0, "state": 1}):
            self.status.return_value = status
            for action in (lambda: self.backend.save_home("demo"), self.backend.restore_joints):
                with self.subTest(status=status), self.assertRaises(Rejected):
                    action()
        self.assertEqual(events, [])
        self.assertEqual(self.config.read_bytes(), self.original)

    def test_stale_status_file_is_rejected_before_handover(self):
        events, _ = self.teleop()
        self.backend.status_path = self.root / "status.json"
        self.backend.status_path.write_text('{"deadman": 0, "state": 2}')
        os.utime(self.backend.status_path, (1, 1))
        self.status.side_effect = lambda: LegacyBackend._read_status(self.backend)
        with self.assertRaises(Rejected):
            self.backend.restore_joints()
        self.assertEqual(events, [])

    def test_failed_bridge_exit_never_starts_home_or_resumes(self):
        events, bridge = self.teleop()
        bridge.proc.poll.return_value = 1
        bridge.log_path = self.root / "fault.log"
        bridge.log_path.write_text("control fault")
        with patch.object(self.backend, "_restore_home") as home:
            with self.assertRaisesRegex(RuntimeError, "暂停遥操失败"):
                self.backend.restore_joints()
            home.assert_not_called()
        self.assertEqual(events, ["stop"])

    def test_home_refusal_resumes_but_fault_does_not(self):
        for error, expected in ((Rejected("home refused: F3 held"), ["stop", "start"]),
                                (RuntimeError("reflex"), ["stop"])):
            events, _ = self.teleop()
            with patch.object(self.backend, "_restore_home", side_effect=error):
                with self.assertRaises(type(error)):
                    self.backend.restore_joints()
            self.assertEqual(events, expected)
            patch.stopall()

    def test_teleop_failed_save_preserves_config_and_resumes(self):
        events, _ = self.teleop()
        self.stub.write_text(f"#!{sys.executable}\nimport sys\nsys.exit('save home refused: F3 held')\n")
        with self.assertRaises(Rejected):
            self.backend.save_home("demo")
        self.assertEqual(events, ["stop", "start"])
        self.assertEqual(self.config.read_bytes(), self.original)

    def test_resume_failure_propagates_as_fault(self):
        self.teleop()
        self.start.side_effect = RuntimeError("bridge restart failed")
        with patch.object(self.backend, "_restore_home"):
            with self.assertRaisesRegex(RuntimeError, "bridge restart failed"):
                self.backend.restore_joints()

    def test_teleop_dry_run_never_commands_real_home(self):
        events, _ = self.teleop()
        self.backend._dry_run = True
        with patch.object(self.backend, "_restore_home") as home:
            with self.assertRaises(Rejected):
                self.backend.restore_joints()
            home.assert_not_called()
        self.assertEqual(events, [])

    def test_session_allows_released_teleop_home_and_reports_busy(self):
        loop = ControlLoop(self.backend)
        loop.state.phase, loop.state.mode = Phase.READY, Mode.TELEOP
        loop.state.config_name = "demo"
        def save(name):
            self.assertTrue(loop.snapshot().to_json()["home_busy"])
            return {"config_name": name, "q": Q}
        with patch.object(self.backend, "save_home", side_effect=save):
            loop._apply(Command("save_home", {"config_name": "demo"}))
        self.assertFalse(loop.state.home_busy)
        self.assertEqual(loop.state.saved_home["q"], Q)
        with patch.object(self.backend, "restore_joints") as restore:
            loop._apply(Command("restore_joints"))
            restore.assert_called_once()
        self.assertEqual(loop.state.phase, Phase.READY)

    def test_session_rejects_held_or_mapping_for_both_actions(self):
        loop = ControlLoop(self.backend)
        loop.state.phase, loop.state.mode = Phase.READY, Mode.TELEOP
        loop.state.config_name = "demo"
        for held, enabled in ((True, False), (False, True)):
            loop.state.deadman_held, loop.state.stream_enabled = held, enabled
            for name in ("save_home", "restore_joints"):
                with patch.object(self.backend, name) as action:
                    with self.assertRaises(Rejected):
                        loop._apply(Command(name, {"config_name": "demo"}))
                    action.assert_not_called()

    def test_mock_cannot_persist_simulated_pose(self):
        loop = ControlLoop(MockBackend(config_dir=self.conf))
        with self.assertRaises(Rejected):
            loop._apply(Command("save_home", {"config_name": "demo"}))
        self.assertEqual(self.config.read_bytes(), self.original)

    def test_http_idle_save_uses_measured_pose_without_opening_cameras(self):
        loop = ControlLoop(self.backend)
        loop.start()
        server = serve(loop, host="127.0.0.1", port=0)
        try:
            body = json.dumps({"name": "save_home", "args": {
                "config_name": "demo", "qpos": [9] * 7}}).encode()
            req = urllib.request.Request(
                f"http://127.0.0.1:{server.server_port}/api/command", body,
                {"Content-Type": "application/json"})
            with urllib.request.urlopen(req, timeout=5) as response:
                result = json.load(response)
            self.assertTrue(result["accepted"])
            self.assertEqual(result["state"]["phase"], "idle")
            self.assertEqual(result["state"]["saved_home"]["q"], Q)
            self.assertIsNone(self.backend._viewfinder)
            self.assertIsNone(self.backend._bridge)
        finally:
            server.shutdown()
            server.server_close()
            loop.stop()


if __name__ == "__main__":
    unittest.main()
