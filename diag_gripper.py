"""Measure the Franka Hand's real reaction latency from a teleop log.

Cross-correlation gives a best global alignment, which conflates transport
delay with the hand's travel time. What matters for teleoperation is reaction
latency: how long after the commanded target moves does the measured width
start moving at all.
"""
from __future__ import annotations

import argparse
import csv


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--window", type=float, default=0.0,
                    help="if set, dump a raw time window starting here")
    ap.add_argument("--span", type=float, default=4.0)
    args = ap.parse_args()

    rows = list(csv.DictReader(open(args.csv)))
    t = [int(r["t_ns"]) / 1e9 for r in rows]
    t0 = t[0]
    gt = [float(r["gripper_target"]) for r in rows]
    gw = [float(r["gripper_width"]) for r in rows]
    gk = [int(r["gripper_ticks"]) for r in rows]

    if args.window:
        print(f"{'t':>7} {'trig':>7} {'target_mm':>10} {'width_mm':>9}")
        i0 = next(k for k in range(len(t)) if t[k] - t0 >= args.window)
        i1 = next((k for k in range(len(t)) if t[k] - t0 >= args.window + args.span),
                  len(t) - 1)
        step = max((i1 - i0) // 40, 1)
        for k in range(i0, i1, step):
            print(f"{t[k]-t0:7.2f} {gk[k]:7d} {gt[k]*1000:10.2f} {gw[k]*1000:9.2f}")
        return 0

    # Find target moves of at least 5 mm that begin after a quiet period, then
    # time how long until the width responds by 1 mm.
    events = []
    k = 1
    while k < len(rows) - 1:
        if abs(gt[k] - gt[k - 1]) > 0.0005:
            start = k
            base_w = gw[k]
            target_at_start = gt[k]
            # let the command settle
            j = k
            while j < len(rows) - 1 and t[j] - t[start] < 1.5:
                j += 1
            final_t = gt[j]
            if abs(final_t - target_at_start) < 0.0 or abs(final_t - base_w) < 0.005:
                k += 1
                continue
            resp = None
            m = start
            while m < len(rows) and t[m] - t[start] < 3.0:
                if abs(gw[m] - base_w) > 0.001:
                    resp = t[m] - t[start]
                    break
                m += 1
            if resp is not None:
                events.append((t[start] - t0, (final_t - base_w) * 1000, resp * 1000))
            k = j
        else:
            k += 1

    if not events:
        print("no clean command->response events found")
        return 1

    print(f"{'t_s':>8} {'move_mm':>9} {'reaction_ms':>12}")
    for ts, mm, ms in events[:25]:
        print(f"{ts:8.2f} {mm:9.1f} {ms:12.0f}")
    lat = sorted(e[2] for e in events)
    print(f"\nevents={len(events)}  reaction latency: "
          f"min={lat[0]:.0f} med={lat[len(lat)//2]:.0f} max={lat[-1]:.0f} ms")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
