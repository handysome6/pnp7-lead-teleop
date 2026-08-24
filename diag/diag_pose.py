"""Compare lead-arm and Franka joint configurations across a run.

Relative joint teleoperation maps lead J_n to Franka J_n. If the two arms sit
in DIFFERENT configurations on an upstream joint, downstream joints appear
mirrored to the operator even though the mapping is correct -- rotating the
lead forearm 180 degrees flips which way its wrist pitch looks like it moves.

Clutching is what lets the two drift apart: releasing the dead-man, moving the
lead arm, and re-engaging changes the lead pose without moving the robot.
"""
from __future__ import annotations

import argparse
import csv
import math

NJ = 7


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    args = ap.parse_args()

    rows = list(csv.DictReader(open(args.csv)))
    t = [int(r["t_ns"]) / 1e9 for r in rows]
    t0 = t[0]

    print(f"{'joint':<7}{'lead rel span':>16}{'franka span':>14}"
          f"{'ratio':>9}")
    for j in range(NJ):
        ld = [float(r[f"lead_delta{j}"]) for r in rows]
        qr = [float(r[f"q_robot{j}"]) for r in rows]
        ls = max(ld) - min(ld)
        rs = max(qr) - min(qr)
        ratio = rs / ls if ls > 1e-6 else 0.0
        print(f"{'J'+str(j+1):<7}{math.degrees(ls):>13.1f} deg"
              f"{math.degrees(rs):>11.1f} deg{ratio:>9.2f}")

    st = [r["state"] for r in rows]
    trans = [(round(t[k] - t0, 2), st[k - 1], st[k])
             for k in range(1, len(rows)) if st[k] != st[k - 1]]
    print(f"\nstate transitions: {len(trans)}")
    for tt, a, b in trans[:20]:
        print(f"  t={tt:7.2f}  {a} -> {b}")

    # During each engaged segment, how far did lead and robot each travel?
    # A large lead excursion with the robot not following means the operator
    # moved the lead arm while disengaged.
    print("\nper-engagement travel (J5, J6):")
    seg_start = None
    for k in range(1, len(rows)):
        if st[k] == "1" and st[k - 1] != "1":
            seg_start = k
        elif seg_start is not None and st[k] != "1" and st[k - 1] == "1":
            for j, name in ((4, "J5"), (5, "J6")):
                ld = [float(rows[i][f"lead_delta{j}"])
                      for i in range(seg_start, k)]
                qr = [float(rows[i][f"q_robot{j}"]) for i in range(seg_start, k)]
                print(f"  [{t[seg_start]-t0:6.2f}..{t[k]-t0:6.2f}] {name}: "
                      f"lead {math.degrees(max(ld)-min(ld)):7.1f} deg, "
                      f"robot {math.degrees(max(qr)-min(qr)):7.1f} deg")
            seg_start = None
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
