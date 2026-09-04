"""Offline self-test for the collection GUI. No hardware, no RLinf, no robot.

Mirrors ``./bin/pnp7_teleop selftest`` in spirit: exercise everything that can
be checked without the rig, so that the only things left to verify on the robot
PC are the ones that genuinely need it.

What it covers is the part most worth covering -- the F3 clipping arithmetic.
That a released pedal drops frames, that re-engaging opens a new segment, and
that an episode with no held frames is not written, are all decisions about what
lands in the dataset, and none of them need a foot pedal to test.

    python -m gui.selftest
"""
from __future__ import annotations

import json
import os
import shutil
import socket
import subprocess
import sys
import tempfile
import time
import urllib.error
import urllib.request
from pathlib import Path

from gui.legacy_backend import LegacyBackend
from gui.mock import MockBackend
from gui.server import serve
from gui.session import ControlLoop

REPO = Path(__file__).resolve().parent.parent

#: A config with everything `home` and the status publisher need. Written for
#: the stub bridge, but every key here is one the real loadConfig accepts.
STUB_CONFIG = """\
lead_port=/dev/pnp7_lead
lead_baud=1000000
robot_ip=172.16.0.2
deadman_device=/dev/pnp7_deadman
deadman_key=KEY_F3
deadman_grab=1
lead_servo_id=1 2 3 4 5 6 7
sign=1 -1 1 -1 1 1 1
scale=0.5 0.5 0.5 0.5 0.5 0.5 0.5
enabled_joints=1111111
lowpass_hz=6.0
max_joint_velocity=0.3
max_joint_acceleration=1.5
max_session_delta=0.5
watchdog_ms=100
home_qpos=0.0 -0.785 0.0 -2.356 0.0 1.571 0.785
"""


class Client:
    def __init__(self, base: str):
        self.base = base

    def _post(self, path: str, payload: dict) -> dict:
        req = urllib.request.Request(
            self.base + path, data=json.dumps(payload).encode(),
            headers={"Content-Type": "application/json"}, method="POST")
        try:
            with urllib.request.urlopen(req, timeout=70) as response:
                return json.load(response)
        except urllib.error.HTTPError as exc:
            return {"http_error": exc.code, **json.load(exc)}

    def cmd(self, name: str, **args) -> dict:
        return self._post("/api/command", {"name": name, "args": args})

    def pedal(self, held: bool) -> dict:
        return self._post("/api/mock/deadman", {"held": held})

    def state(self) -> dict:
        with urllib.request.urlopen(self.base + "/api/state", timeout=10) as r:
            return json.load(r)


class Report:
    def __init__(self):
        self.failures: list[str] = []
        self.checks = 0

    def check(self, label: str, ok: bool, detail: str = "") -> None:
        self.checks += 1
        mark = "PASS" if ok else "FAIL"
        print(f"  [{mark}] {label}" + (f"  --  {detail}" if detail else ""))
        if not ok:
            self.failures.append(label)

    def note(self, text: str) -> None:
        print(f"         {text}")


def _hold(c: Client, seconds: float) -> None:
    """Press the pedal and let the control loop run for `seconds`."""
    c.pedal(True)
    time.sleep(seconds)


def _release(c: Client, seconds: float) -> None:
    c.pedal(False)
    time.sleep(seconds)


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def run_checks(c: Client, r: Report) -> None:  # noqa: C901 - a linear script
    print("\n-- guards while idle ------------------------------------------")
    r.check("start episode refused while idle",
            c.cmd("begin_episode").get("http_error") == 409)
    r.check("restore joints refused while idle",
            c.cmd("restore_joints").get("http_error") == 409)

    print("\n-- open a collect session (item 3) ----------------------------")
    c.cmd("open_session", config_name="realworld_collect_data_gello_franky",
          overrides={"duration": 60, "prefix": "ep",
                     "episodes_dir": "/tmp/mock_ds"},
          mode="collect")
    s = c.state()
    r.check("phase -> ready", s["phase"] == "ready", s["phase"])
    r.check("both cameras announced (item 2)",
            s["cameras"] == ["external", "wrist"], str(s["cameras"]))
    r.check("episodes_dir override applied",
            s["dataset_dir"] == "/tmp/mock_ds", str(s["dataset_dir"]))
    time.sleep(0.6)
    hz = c.state()["step_hz"]
    r.check("control loop is stepping", hz > 5, f"{hz:.1f} Hz")

    print("\n-- restore the arm (item 1) -----------------------------------")
    _hold(c, 0.4)
    r.check("restore refused while F3 is held",
            c.cmd("restore_joints").get("http_error") == 409)
    _release(c, 0.4)
    r.check("restore accepted once F3 is released",
            c.cmd("restore_joints").get("accepted") is True)

    print("\n-- episode control + F3 clipping (items 4, 6) -----------------")
    c.cmd("begin_episode")
    r.check("phase -> recording", c.state()["phase"] == "recording")

    _hold(c, 1.0)
    a = c.state()
    r.note(f"interval A: kept={a['episode_steps']} dropped={a['skipped_steps']} "
           f"segments={a['episode_segments']}")
    r.check("frames captured while F3 is held", a["episode_steps"] > 0,
            str(a["episode_steps"]))
    r.check("first interval is segment 1", a["episode_segments"] == 1,
            str(a["episode_segments"]))

    _release(c, 1.0)
    b = c.state()
    r.note(f"pedal gap:  kept={b['episode_steps']} dropped={b['skipped_steps']} "
           f"segments={b['episode_segments']}")
    r.check("nothing captured while F3 is released",
            b["episode_steps"] == a["episode_steps"],
            f"{a['episode_steps']} -> {b['episode_steps']}")
    r.check("released frames are counted as dropped",
            b["skipped_steps"] > a["skipped_steps"],
            f"{a['skipped_steps']} -> {b['skipped_steps']}")

    _hold(c, 1.0)
    d = c.state()
    r.note(f"interval B: kept={d['episode_steps']} dropped={d['skipped_steps']} "
           f"segments={d['episode_segments']}")
    r.check("capture resumes when F3 is re-pressed",
            d["episode_steps"] > b["episode_steps"],
            f"{b['episode_steps']} -> {d['episode_steps']}")
    r.check("re-pressing opens a second segment",
            d["episode_segments"] == 2, str(d["episode_segments"]))
    _release(c, 0.3)

    r.check("restore refused mid-episode",
            c.cmd("restore_joints").get("http_error") == 409)

    print("\n-- keep / discard / empty -------------------------------------")
    c.cmd("end_episode")
    s = c.state()
    r.check("stop & keep returns to ready", s["phase"] == "ready", s["phase"])
    r.check("kept episode is counted", s["episode_index"] == 1,
            str(s["episode_index"]))
    r.note(f"message: {s['message']}")

    c.cmd("begin_episode")
    _hold(c, 0.6)
    _release(c, 0.2)
    before = c.state()["episode_index"]
    c.cmd("discard_episode")
    s = c.state()
    r.check("discard returns to ready", s["phase"] == "ready", s["phase"])
    r.check("discarded episode is not counted",
            s["episode_index"] == before, str(s["episode_index"]))

    c.cmd("begin_episode")          # never touch the pedal
    time.sleep(0.6)
    before = c.state()["episode_index"]
    c.cmd("end_episode")
    s = c.state()
    r.check("episode with no F3-held frames is not written",
            s["episode_index"] == before, s["message"])
    r.note(f"message: {s['message']}")

    print("\n-- teleop-only mode (item 7) ----------------------------------")
    c.cmd("close_session")
    r.check("session closed", c.state()["phase"] == "idle")
    c.cmd("open_session", config_name="realworld_collect_data_gello_franky",
          overrides={}, mode="teleop")
    s = c.state()
    r.check("teleop session is ready",
            s["phase"] == "ready" and s["mode"] == "teleop",
            f"{s['phase']}/{s['mode']}")
    r.check("recording refused in teleop mode",
            c.cmd("begin_episode").get("http_error") == 409)
    _hold(c, 0.5)
    r.check("teleop still streams to the arm",
            c.state()["stream_enabled"] is True)
    c.pedal(False)

    print("\n-- preview transport (item 2) ---------------------------------")
    with urllib.request.urlopen(c.base + "/stream/external", timeout=10) as resp:
        ctype = resp.headers.get("Content-Type", "")
        chunk = resp.read(20000)
    r.check("multipart MJPEG header",
            "multipart/x-mixed-replace" in ctype, ctype)
    r.check("JPEG payload present", b"\xff\xd8\xff" in chunk,
            f"{len(chunk)} bytes")
    c.cmd("close_session")

    print("\n-- server hygiene ---------------------------------------------")
    try:
        urllib.request.urlopen(c.base + "/static/../session.py", timeout=5)
        r.check("path traversal refused", False, "it served the file")
    except urllib.error.HTTPError as exc:
        r.check("path traversal refused", exc.code == 404, str(exc.code))
    r.check("unknown command rejected",
            c.cmd("definitely_not_a_command").get("http_error") == 409)


def run_orchestration_checks(r: Report) -> None:
    """Drive the real supervisor against stand-in hardware.

    The point of this phase is that `build_episode.py` and
    `validate_episode.py` are the genuine articles here -- only the bridge and
    the camera recorder are stubs. So the join, the F3 clipping and the
    validation are all exercised for real, on a laptop, which is most of what
    could quietly break.
    """
    workspace = Path(tempfile.mkdtemp(prefix="pnp7_gui_selftest_"))
    pedal = workspace / "pedal"
    os.environ["PNP7_STUB_PEDAL"] = str(pedal)

    conf_dir = workspace / "conf"
    conf_dir.mkdir()
    (conf_dir / "stub.conf").write_text(STUB_CONFIG)

    backend = LegacyBackend(
        episodes_dir=workspace / "episodes",
        preview_dir=workspace / "preview",
        status_path=workspace / "status.json",
        bridge=REPO / "gui" / "stubs" / "bridge.py",
        python=sys.executable,
        recorder=REPO / "gui" / "stubs" / "cameras.py",
        conf_dir=conf_dir,
        scratch_dir=workspace,
    )
    port = _free_port()
    loop = ControlLoop(backend, fps=10.0)
    loop.start()
    server = serve(loop, host="127.0.0.1", port=port)
    time.sleep(0.3)
    c = Client(f"http://127.0.0.1:{port}")

    try:
        print("\n-- supervised session over stub hardware ----------------------")
        r.check("stub config is offered",
                any(cfg["name"] == "stub"
                    for cfg in json.loads(urllib.request.urlopen(
                        c.base + "/api/configs", timeout=10).read())["configs"]))

        result = c.cmd("open_session", config_name="stub",
                       overrides={"duration": 30, "prefix": "ep",
                                  "episodes_dir": str(workspace / "episodes")},
                       mode="collect")
        state = c.state()
        r.check("session opens", state["phase"] == "ready",
                str(result.get("error") or state["phase"]))
        r.check("viewfinder announces both cameras",
                state["cameras"] == ["external", "wrist"], str(state["cameras"]))
        r.check("preview frames are published",
                (workspace / "preview" / "external.jpg").is_file())

        print("\n-- restore the arm (item 1) -----------------------------------")
        r.check("home accepted with the pedal up",
                c.cmd("restore_joints").get("accepted") is True)
        pedal.touch()
        time.sleep(0.2)
        refused = c.cmd("restore_joints")
        r.check("home refused with the pedal held",
                refused.get("accepted") is False, str(refused.get("error"))[:70])
        pedal.unlink(missing_ok=True)

        print("\n-- a take, with the pedal released in the middle ---------------")
        c.cmd("begin_episode")
        r.check("phase -> recording", c.state()["phase"] == "recording")

        pedal.touch()
        time.sleep(1.8)
        mid = c.state()
        r.check("bridge status reaches the GUI", mid["deadman_held"] is True,
                str(mid.get("block_reason")))
        r.check("joints arrive from the status file",
                len(mid["joints"]) == 7, str(len(mid["joints"])))

        pedal.unlink(missing_ok=True)
        time.sleep(0.8)
        gap = c.state()
        r.check("pedal release is visible", gap["deadman_held"] is False)

        pedal.touch()
        time.sleep(1.8)
        pedal.unlink(missing_ok=True)
        time.sleep(0.3)

        print("\n-- stop & keep: real build_episode + validate_episode ----------")
        c.cmd("end_episode", timeout=180)
        state = c.state()
        r.check("returns to ready", state["phase"] == "ready", state["phase"])
        r.check("episode counted", state["episode_index"] == 1,
                str(state["episode_index"]))
        r.note(f"message: {state['message']}")

        episode = workspace / "episodes" / "ep001"
        r.check("teleop.csv written", (episode / "teleop.csv").is_file())
        r.check("episode.csv built by build_episode.py",
                (episode / "episode.csv").is_file())

        meta_path = episode / "episode_meta.json"
        r.check("episode_meta.json written", meta_path.is_file())
        if meta_path.is_file():
            meta = json.loads(meta_path.read_text())
            r.note(f"frames={meta.get('frames')} "
                   f"dropped_idle={meta.get('dropped_idle_frames')} "
                   f"rate={meta.get('rate_hz')} Hz")
            r.check("frames survived the join", meta.get("frames", 0) > 0,
                    str(meta.get("frames")))
            # This is the whole architectural claim: released-pedal frames are
            # dropped downstream, by build_episode.py, from the deadman column.
            r.check("released-pedal frames were dropped downstream",
                    meta.get("dropped_idle_frames", 0) > 0,
                    str(meta.get("dropped_idle_frames")))

        print("\n-- the converter reads what build_episode wrote ----------------")
        # The fixture used to develop export_lerobot.py had hand-written column
        # names. This is the same check against the real thing, so a rename in
        # build_episode.py cannot silently break conversion.
        export = subprocess.run(
            [sys.executable, str(REPO / "collect" / "export_lerobot.py"),
             str(episode), "--out", str(workspace / "ds"),
             "--task", "stub", "--dry-run"],
            capture_output=True, text=True, cwd=str(REPO), timeout=120)
        r.check("export_lerobot accepts a real episode",
                export.returncode == 0,
                (export.stderr or "").strip().splitlines()[-1:] and
                (export.stderr or "").strip().splitlines()[-1] or "")
        r.check("it finds the two F3 segments",
                "2 segment(s)" in export.stdout,
                export.stdout.strip().splitlines()[-1] if export.stdout else "")
        r.check("a dry run writes nothing", not (workspace / "ds").exists())

        print("\n-- config keys the GUI appends are real ones -------------------")
        # loadConfig silently ignores unknown keys, so a typo here would be
        # invisible: the bridge would simply never publish status and the GUI
        # would report a startup timeout with no hint why.
        bridge_src = (REPO / "src" / "pnp7_teleop.cpp").read_text()
        prepared = (workspace / "conf" / "stub.conf")
        appended = [line.split("=", 1)[0].strip()
                    for line in (episode / "config.conf").read_text().splitlines()
                    if "=" in line and not line.strip().startswith("#")]
        unknown = [k for k in appended if f'key == "{k}"' not in bridge_src]
        r.check("every key in the episode's config.conf is one loadConfig parses",
                not unknown, str(unknown))
        r.check("status_path was actually appended", "status_path" in appended)
        del prepared

        print("\n-- discard really deletes -------------------------------------")
        c.cmd("begin_episode")
        pedal.touch()
        time.sleep(0.6)
        pedal.unlink(missing_ok=True)
        before = c.state()["episode_index"]
        c.cmd("discard_episode", timeout=120)
        r.check("discard returns to ready", c.state()["phase"] == "ready")
        r.check("discard does not count the episode",
                c.state()["episode_index"] == before)
        r.check("discarded directory is gone -- not merely reported",
                not (workspace / "episodes" / "ep002").exists())

        print("\n-- teleop-only mode (item 7) ----------------------------------")
        c.cmd("close_session")
        c.cmd("open_session", config_name="stub",
              overrides={"teleop_duration": 30}, mode="teleop")
        state = c.state()
        r.check("teleop session ready",
                state["phase"] == "ready" and state["mode"] == "teleop",
                f"{state['phase']}/{state['mode']}")
        r.check("recording refused in teleop mode",
                c.cmd("begin_episode").get("http_error") == 409)
        pedal.touch()
        time.sleep(0.5)
        r.check("teleop still reports the pedal",
                c.state()["deadman_held"] is True)
        pedal.unlink(missing_ok=True)
        c.cmd("close_session", timeout=120)
        r.check("session closed", c.state()["phase"] == "idle")
        r.check("no episode directory was created by teleop",
                not (workspace / "episodes" / "ep002").exists())
    finally:
        server.shutdown()
        loop.stop()
        os.environ.pop("PNP7_STUB_PEDAL", None)
        shutil.rmtree(workspace, ignore_errors=True)


def main() -> int:
    port = _free_port()
    backend = MockBackend()
    loop = ControlLoop(backend, fps=10.0)
    loop.start()
    server = serve(loop, host="127.0.0.1", port=port, mock_backend=backend)
    time.sleep(0.4)

    print(f"gui selftest -- mock backend on 127.0.0.1:{port}")
    report = Report()
    try:
        run_checks(Client(f"http://127.0.0.1:{port}"), report)
    finally:
        server.shutdown()
        loop.stop()

    # Second phase: the real supervisor, stubbed hardware, real post-processing.
    run_orchestration_checks(report)

    print()
    if report.failures:
        print(f"SELFTEST FAILED -- {len(report.failures)} of {report.checks}:")
        for name in report.failures:
            print(f"  - {name}")
        return 1
    print(f"SELFTEST_OK -- {report.checks} checks")
    return 0


if __name__ == "__main__":
    sys.exit(main())
