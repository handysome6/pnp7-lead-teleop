"""Report how far the lead arm has drifted out of correspondence with the Franka.

Relative joint teleoperation never establishes absolute correspondence -- that
is deliberate (roadmap section 1). The consequence is that the two arms can end
up in different configurations, because moving the lead arm while the dead-man
is released changes the lead pose without moving the robot.

When that happens on an upstream joint, downstream joints LOOK mirrored even
though their signs are correct. Rotating the lead forearm (J5) 180 degrees
flips the wrist, so J6 appears to move the wrong way. Flipping J6's sign to
"fix" that would be wrong: it would only hold while J5 is inverted.

This measures drift against the pose recorded at calibration time.

  python check_correspondence.py
"""
from __future__ import annotations

import argparse
import json
import math
import os
import subprocess
import sys
import tempfile
import csv as csvmod

from pnp7_lead import ALL_IDS, PNP7Lead, TICKS_TO_RAD, wrap_delta

NJ = 7


def read_franka(ip):
    tmp = os.path.join(tempfile.gettempdir(), "pnp7_corr.csv")
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = (
        "/opt/openrobots/lib:"
        + os.path.expanduser("~/catkin_franka/libfranka/build")
        + ":" + env.get("LD_LIBRARY_PATH", ""))
    binary = os.path.expanduser("~/workspace/andyls/bin/read_franka_state")
    subprocess.run([binary, ip, tmp, "1", "5"], check=True, timeout=25,
                   capture_output=True, env=env)
    rows = list(csvmod.DictReader(open(tmp)))
    return [float(x) for x in rows[-1]["q"].split(";")]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibration", default="calibration.json")
    ap.add_argument("--port", default="/dev/pnp7_lead")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--robot-ip", default="172.16.0.2")
    ap.add_argument("--reference",
                    help="posture file to measure drift against, instead of "
                         "the calibration pose. Use this when you deliberately "
                         "work in a different configuration -- otherwise the "
                         "check reports the intended offset as an error.")
    ap.add_argument("--save-reference", metavar="FILE",
                    help="record the CURRENT poses as a working posture")
    args = ap.parse_args()

    cal = json.load(open(args.calibration))
    by_label = {m["label"]: m for m in cal["joint_map"]}

    source = "calibration pose"
    if args.reference:
        ref = json.load(open(args.reference))
        rest = ref["franka_q"]
        neutral = ref["lead_ticks"]
        source = f"posture '{ref.get('label', args.reference)}'"
    else:
        rest = (cal.get("franka_rest_pose") or {}).get("q")
        neutral = cal["neutral_ticks"]
        if not rest:
            print("calibration has no franka_rest_pose to compare against")
            return 1

    lead = PNP7Lead(args.port, args.baud)
    lead.open()
    lead.assert_torque_disabled()
    sample = None
    for _ in range(80):
        sample = lead.read() or sample
    lead.close()
    if sample is None:
        print("could not read the lead arm")
        return 1

    q_now = read_franka(args.robot_ip)

    if args.save_reference:
        from datetime import datetime, timezone
        ref = {
            "label": args.save_reference,
            "captured": datetime.now(timezone.utc).isoformat(),
            "lead_ticks": list(sample.ticks_cont),
            "franka_q": q_now,
            "note": "Working posture. Drift is measured against this rather "
                    "than the calibration pose. If the config carries a "
                    "POSTURE OVERRIDE, it belongs with this file.",
        }
        with open(args.save_reference, "w") as fh:
            json.dump(ref, fh, indent=2)
        print(f"saved working posture to {args.save_reference}\n")

    print(f"measuring drift against {source}\n")
    print(f"{'joint':<7}{'lead moved':>13}{'franka moved':>15}"
          f"{'mismatch':>12}")
    worst = []
    for j in range(NJ):
        label = f"J{j+1}"
        m = by_label[label]
        idx = ALL_IDS.index(m["lead_servo_id"])
        # Continuous tick accumulation only lives for one process, so a
        # neutral captured after the arm wound past a revolution is 4096 ticks
        # from what a fresh process reads at the same physical position.
        # Compare modulo one turn: a genuine 180 deg offset still shows, a
        # bookkeeping full turn does not.
        lead_rad = wrap_delta(sample.ticks_cont[idx],
                              neutral[idx]) * TICKS_TO_RAD
        lead_signed = m["observed_sign"] * lead_rad
        franka_moved = q_now[j] - rest[j]
        mismatch = lead_signed - franka_moved
        worst.append((abs(mismatch), label, mismatch))
        flag = ""
        if abs(abs(mismatch) - math.pi) < 0.6:
            flag = "  <-- about 180 deg out"
        elif abs(mismatch) > 1.0:
            flag = "  <-- large"
        print(f"{label:<7}{math.degrees(lead_signed):>10.1f} deg"
              f"{math.degrees(franka_moved):>11.1f} deg"
              f"{math.degrees(mismatch):>9.1f} deg{flag}")

    print("\nMismatch is how far the lead arm has drifted from the robot since")
    print("calibration. It does NOT affect the mapping -- only deltas are used")
    print("while engaged -- but a large value on an upstream joint makes the")
    print("joints below it look mirrored to you.")
    worst.sort(reverse=True)
    top = worst[0]
    if top[0] > 1.0:
        print(f"\n{top[1]} is {math.degrees(top[2]):.0f} deg out of "
              f"correspondence. Bring the lead arm back to a pose resembling")
        print("the robot's before concluding a downstream sign is wrong.")
    else:
        print("\nAll joints are reasonably in correspondence.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
