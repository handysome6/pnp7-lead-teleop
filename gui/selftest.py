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
import socket
import sys
import time
import urllib.error
import urllib.request

from gui.mock import MockBackend
from gui.server import serve
from gui.session import ControlLoop


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
          overrides={"fps": 10, "num_data_episodes": 5,
                     "save_dir": "/tmp/mock_ds"},
          mode="collect")
    s = c.state()
    r.check("phase -> ready", s["phase"] == "ready", s["phase"])
    r.check("both cameras announced (item 2)",
            s["cameras"] == ["external", "wrist"], str(s["cameras"]))
    r.check("save_dir override applied",
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
