"""Live read-out of the PNP-7 lead arm, for joint identification and calibration.

This is roadmap Step A: move one lead-arm joint at a time and record which
servo ID moves, in which direction, and over what range. Nothing is written to
the servo bus and the Franka is never contacted.

  python monitor.py                 # live display
  python monitor.py --record cal.csv  # also log every sample to CSV
"""
from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time

from pnp7.lead import ARM_IDS, GRIPPER_ID, PNP7Lead

BAR_WIDTH = 21


def bar(value: float, lo: float, hi: float) -> str:
    """Render a centred bar so direction of motion is obvious at a glance."""
    if hi <= lo:
        return "-" * BAR_WIDTH
    frac = min(max((value - lo) / (hi - lo), 0.0), 1.0)
    pos = int(frac * (BAR_WIDTH - 1))
    cells = ["-"] * BAR_WIDTH
    cells[pos] = "#"
    return "".join(cells)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/pnp7_lead")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--record", help="write every sample to this CSV")
    ap.add_argument("--hz", type=float, default=20.0, help="display refresh rate")
    args = ap.parse_args()

    lead = PNP7Lead(args.port, args.baud)
    lead.open()
    torque = lead.assert_torque_disabled()
    print(f"torque check OK (all servos passive): {torque}")

    writer = None
    handle = None
    if args.record:
        handle = open(args.record, "w", newline="")
        writer = csv.writer(handle)
        writer.writerow(
            ["t_monotonic_ns", "seq"]
            + [f"ticks_j{i}" for i in ARM_IDS]
            + ["ticks_grip"]
            + [f"q{i}_rad" for i in ARM_IDS]
            + ["grip_rad"]
        )

    lo = [math.inf] * 8
    hi = [-math.inf] * 8

    stop = False

    def on_sigint(_sig, _frm):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_sigint)

    lead.start()
    period = 1.0 / args.hz
    print("\nMove ONE lead-arm joint at a time. Ctrl-C to stop.\n")
    printed_lines = 0

    try:
        while not stop:
            s = lead.latest()
            if s is None:
                time.sleep(period)
                continue

            for i, t in enumerate(s.ticks):
                lo[i] = min(lo[i], t)
                hi[i] = max(hi[i], t)

            if writer:
                writer.writerow(
                    [s.t_monotonic_ns, s.seq] + s.ticks
                    + [f"{v:.6f}" for v in s.q_rad] + [f"{s.gripper_rad:.6f}"]
                )

            lines = [
                f"seq={s.seq:<8} fails={lead.read_failures:<5} "
                f"t={s.t_monotonic_ns/1e9:.3f}",
                f"{'joint':<8}{'id':<4}{'ticks':>7}{'deg':>9}{'rad':>9}"
                f"{'dq':>8}  {'range seen':<21} {'min..max'}",
            ]
            for idx, sid in enumerate(ARM_IDS):
                deg = s.ticks[idx] * 360.0 / 4096.0
                lines.append(
                    f"{'J'+str(sid):<8}{sid:<4}{s.ticks[idx]:>7}{deg:>9.2f}"
                    f"{s.q_rad[idx]:>9.3f}{s.dq_rad_s[idx]:>8.2f}  "
                    f"{bar(s.ticks[idx], lo[idx], hi[idx]):<21} "
                    f"{lo[idx]}..{hi[idx]}"
                )
            gdeg = s.gripper_ticks * 360.0 / 4096.0
            lines.append(
                f"{'GRIP':<8}{GRIPPER_ID:<4}{s.gripper_ticks:>7}{gdeg:>9.2f}"
                f"{s.gripper_rad:>9.3f}{'':>8}  "
                f"{bar(s.gripper_ticks, lo[7], hi[7]):<21} {lo[7]}..{hi[7]}"
            )

            if printed_lines:
                sys.stdout.write(f"\033[{printed_lines}A")
            for line in lines:
                sys.stdout.write("\033[2K" + line + "\n")
            sys.stdout.flush()
            printed_lines = len(lines)

            time.sleep(period)
    finally:
        lead.close()
        if handle:
            handle.close()
            print(f"\nrecorded to {args.record}")

    print("\n--- observed travel (ticks) ---")
    for idx, sid in enumerate(ARM_IDS):
        span = hi[idx] - lo[idx] if hi[idx] > lo[idx] else 0
        print(f"  J{sid}: {lo[idx]}..{hi[idx]}  span={span} "
              f"({span * 360.0 / 4096.0:.1f} deg)")
    span = hi[7] - lo[7] if hi[7] > lo[7] else 0
    print(f"  GRIP: {lo[7]}..{hi[7]}  span={span} "
          f"({span * 360.0 / 4096.0:.1f} deg)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
