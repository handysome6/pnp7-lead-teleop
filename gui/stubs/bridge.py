#!/usr/bin/env python3
"""Test double for `bin/pnp7_teleop`. Not a simulator -- a stand-in.

It reproduces exactly the interface `gui.legacy_backend` depends on, and
nothing else:

* publishes the same 10 Hz status JSON to the config's `status_path`, via
  write-tmp-then-rename, and deletes it on a clean exit;
* buffers rows and writes the 65-column CSV once, at the end, on SIGINT or when
  the duration expires -- and exits 0 either way;
* stamps `t_ns` with `time.monotonic_ns()`, the same clock the camera recorder
  uses, so `build_episode.py` can join the two for real.

The pedal is a file: it counts as held while `$PNP7_STUB_PEDAL` exists. That
lets a test drive the F3 gating without a foot switch, and it is why the rows
this writes carry a realistic `deadman` column rather than a constant.

    PNP7_STUB_PEDAL=/tmp/pedal python -m gui.stubs.bridge robot conf.conf 10 out.csv
"""
from __future__ import annotations

import json
import os
import signal
import sys
import time
from pathlib import Path

NJ = 7
#: The real bridge runs its FCI callback at 1 kHz; rows are emitted at that
#: density against wall-clock so row counts look like the real thing.
ROW_HZ = 1000
STATUS_HZ = 10

_stop = False


def _on_signal(_sig, _frame):
    global _stop
    _stop = True


def read_status_path(config: Path) -> str:
    """Last occurrence wins, matching loadConfig's if/else chain."""
    found = ""
    for line in config.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if line.startswith("status_path="):
            found = line.split("=", 1)[1].strip()
    return found


def header() -> list[str]:
    cols = ["t_ns", "dt_s", "lead_seq", "state", "deadman"]
    for name in ("q_robot", "q_target", "lead_delta", "dq_robot", "tau_robot"):
        cols += [f"{name}{j}" for j in range(NJ)]
    cols += [f"O_T_EE{j}" for j in range(16)]
    cols += [f"O_F_ext{j}" for j in range(6)]
    cols += ["gripper_ticks", "gripper_width", "gripper_target"]
    return cols


def main() -> int:
    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    if len(sys.argv) >= 3 and sys.argv[1] == "home":
        config = Path(sys.argv[2])
        if "home_qpos" not in config.read_text():
            print("home refused: this config has no home_qpos", file=sys.stderr)
            return 1
        if os.environ.get("PNP7_STUB_PEDAL") and Path(
                os.environ["PNP7_STUB_PEDAL"]).exists():
            print("home refused: the dead-man is held", file=sys.stderr)
            return 1
        time.sleep(0.3)
        print("HOME_OK")
        return 0

    if len(sys.argv) < 4:
        print("usage: bridge <mode> <config> <seconds> [log.csv]",
              file=sys.stderr)
        return 2

    config = Path(sys.argv[2])
    duration = float(sys.argv[3])
    log_path = Path(sys.argv[4]) if len(sys.argv) > 4 else None
    status_path = read_status_path(config)
    pedal = os.environ.get("PNP7_STUB_PEDAL")

    print("CONTROL_READY")

    rows: list[list] = []
    t0 = time.monotonic()
    next_status = 0.0
    seq = 0
    q = [0.0, -0.4, 0.0, -2.2, 0.0, 1.9, 0.8]

    while not _stop and (time.monotonic() - t0) < duration:
        now = time.monotonic()
        held = bool(pedal and Path(pedal).exists())

        if now >= next_status:
            next_status = now + 1.0 / STATUS_HZ
            seq += 1
            if status_path:
                payload = {
                    "state": 1 if held else 0,
                    "deadman": 1 if held else 0,
                    "lead_seq": seq,
                    "lead_age_ms": 1.0,
                    "gripper_width": 0.0800,
                    "gripper_target": 0.0800,
                    "q": [round(v, 4) for v in q],
                    "q_target": [round(v, 4) for v in q],
                }
                tmp = status_path + ".tmp"
                Path(tmp).write_text(json.dumps(payload))
                os.replace(tmp, status_path)

        # Emit a block of rows at the nominal rate for the slice just elapsed.
        target_rows = int((now - t0) * ROW_HZ)
        while len(rows) < target_rows:
            k = len(rows)
            t_ns = time.monotonic_ns()
            if held:
                # Move only while the pedal is down, so q_command varies across
                # exactly the intervals validate_episode.py should see.
                for j in range(NJ):
                    q[j] += 0.00002 * (j + 1)
            row = [t_ns, 0.001, seq, 1 if held else 0, 1 if held else 0]
            row += [round(v, 6) for v in q]                       # q_robot
            row += [round(v + 0.001, 6) for v in q]               # q_target
            row += [round(0.01 * (k % 100), 6)] * NJ              # lead_delta
            row += [0.0] * NJ + [0.0] * NJ                        # dq, tau
            row += [0.0] * 16 + [0.0] * 6                         # O_T_EE, F_ext
            row += [1000 + (k % 300), 0.08 if k % 200 < 100 else 0.02, 0.08]
            rows.append(row)

        time.sleep(0.005)

    if log_path is not None:
        # One write, at the end -- the property that makes SIGKILL fatal.
        with open(log_path, "w") as fh:
            fh.write(",".join(header()) + "\n")
            for row in rows:
                fh.write(",".join(str(v) for v in row) + "\n")
        print(f"log written: {log_path} ({len(rows)} rows)")

    if status_path:
        Path(status_path).unlink(missing_ok=True)
    print("teleop finished. lead read_failures=0 rejected_jumps=0")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
