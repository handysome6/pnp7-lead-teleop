"""Move a verified calibration onto a new working posture.

Why not just re-run calibrate.py: the servo-to-joint mapping is physical
wiring. It does not depend on how the arm is posed, and it was established
once by moving each joint in isolation and confirming it live. Re-deriving it
from a fresh sweep can only lose information -- and in a posture where the
passive wrist sags under gravity it actively misreads, because the flopping
joint out-travels the one being moved.

So this keeps the mapping and the verified signs, and changes only what the
posture actually changes:

  - neutral_ticks and the Franka reference pose are recaptured here;
  - signs named with --flip are inverted and marked unverified;
  - signs named with --unverify keep their value but must be re-confirmed.

Only joints DOWNSTREAM of a rotated joint can change direction. Rotating the
forearm cannot alter how the base or shoulder move, so those signs carry over
untouched.

  python rebase_calibration.py --flip J6 --unverify J7 --label "J5 +180"
"""
from __future__ import annotations

import argparse
import csv as csvmod
import json
import math
import os
import subprocess
import sys
import tempfile
from datetime import datetime, timezone

from pnp7.lead import ALL_IDS, PNP7Lead, wrap_delta

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]


def read_franka(ip):
    tmp = os.path.join(tempfile.gettempdir(), "pnp7_rebase.csv")
    env = dict(os.environ)
    env["LD_LIBRARY_PATH"] = (
        "/opt/openrobots/lib:"
        + os.path.expanduser("~/catkin_franka/libfranka/build")
        + ":" + env.get("LD_LIBRARY_PATH", ""))
    binary = os.path.expanduser("~/workspace/andyls/bin/read_franka_state")
    subprocess.run([binary, ip, tmp, "1", "5"], check=True, timeout=25,
                   capture_output=True, env=env)
    rows = list(csvmod.DictReader(open(tmp)))
    last = rows[-1]
    return {
        "q": [float(x) for x in last["q"].split(";")],
        "robot_mode": int(last["robot_mode"]),
        "gripper_width": float(last["gripper_width"]),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibration", default="calibration.json")
    ap.add_argument("--flip", nargs="*", default=[], metavar="Jn",
                    help="invert these signs and mark them unverified. This is "
                         "a TOGGLE: running it twice returns the original "
                         "sign. Prefer --set-sign, which is idempotent.")
    ap.add_argument("--set-sign", nargs="*", default=[], metavar="Jn=+1",
                    help="set these signs absolutely, e.g. J6=+1. Idempotent, "
                         "so re-running is harmless.")
    ap.add_argument("--unverify", nargs="*", default=[], metavar="Jn",
                    help="keep the sign but require live re-confirmation")
    ap.add_argument("--label", default="", help="name for this posture")
    ap.add_argument("--port", default="/dev/pnp7_lead")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--robot-ip", default="172.16.0.2")
    args = ap.parse_args()

    with open(args.calibration) as fh:
        cal = json.load(fh)
    by_label = {m["label"]: m for m in cal["joint_map"]}

    flips = {j.upper() for j in args.flip}
    unver = {j.upper() for j in args.unverify}

    wanted: dict[str, int] = {}
    for spec in args.set_sign:
        if "=" not in spec:
            print(f"--set-sign needs Jn=+1 form, got {spec}", file=sys.stderr)
            return 1
        name, val = spec.split("=", 1)
        try:
            sign = int(val)
        except ValueError:
            sign = 0
        if sign not in (1, -1):
            print(f"sign must be +1 or -1, got {val}", file=sys.stderr)
            return 1
        wanted[name.upper()] = sign

    unknown = (flips | unver | set(wanted)) - set(JOINTS)
    if unknown:
        print(f"unknown joints: {sorted(unknown)}", file=sys.stderr)
        return 1

    ids = [by_label[j]["lead_servo_id"] for j in JOINTS]
    if len(set(ids)) != len(ids):
        print(f"refusing: the calibration being rebased has a duplicate "
              f"mapping {ids}. Restore a good one first.", file=sys.stderr)
        return 1

    prior = cal.get("posture_history", [])
    if flips and prior:
        last = prior[-1]
        if sorted(flips) == last.get("flipped", []) and \
                last.get("label") == (args.label or "unnamed"):
            print("REFUSING: the previous rebase already flipped "
                  f"{sorted(flips)} under the same label.")
            print("--flip is a toggle, so repeating it would undo the flip.")
            print("Use --set-sign J6=+1 to state the sign you want instead.")
            return 1

    print("Hold the lead arm in the posture you want as canonical.")
    input("Press Enter to capture it... ")

    lead = PNP7Lead(args.port, args.baud)
    lead.open()
    lead.assert_torque_disabled()
    sample = None
    for _ in range(120):
        sample = lead.read() or sample
    lead.close()
    if sample is None:
        print("could not read the lead arm", file=sys.stderr)
        return 1

    franka = read_franka(args.robot_ip)

    old_neutral = cal["neutral_ticks"]
    # Store within one turn so the value means the same thing in any later
    # process, whatever the continuous accumulator happened to reach here.
    new_neutral = [t % 4096 for t in sample.ticks_cont]
    stamp = datetime.now(timezone.utc).isoformat()

    print(f"\n{'joint':<7}{'servo':<7}{'neutral shift':>16}{'sign':>7}"
          f"{'verified':>11}")
    for j_i, label in enumerate(JOINTS):
        m = by_label[label]
        idx = ALL_IDS.index(m["lead_servo_id"])
        shift = wrap_delta(new_neutral[idx], old_neutral[idx]) * 360.0 / 4096.0
        if label in wanted:
            if m["observed_sign"] != wanted[label]:
                m["observed_sign"] = wanted[label]
                m.pop("sign_verified", None)
                state = "SET"
            else:
                state = "kept" if m.get("sign_verified") else "no"
        elif label in flips:
            m["observed_sign"] = -m["observed_sign"]
            m.pop("sign_verified", None)
            state = "FLIPPED"
        elif label in unver:
            m.pop("sign_verified", None)
            state = "recheck"
        else:
            state = "kept" if m.get("sign_verified") else "no"
        m["neutral_ticks"] = new_neutral[idx]
        print(f"{label:<7}{m['lead_servo_id']:<7}{shift:>13.1f} deg"
              f"{m['observed_sign']:>+7d}{state:>11}")

    cal["neutral_ticks"] = new_neutral
    cal["gripper"]["neutral_ticks"] = new_neutral[ALL_IDS.index(8)]
    cal["franka_rest_pose"] = franka
    cal["signs_verified"] = all(
        by_label[j].get("sign_verified") for j in JOINTS)
    cal.setdefault("posture_history", []).append({
        "rebased": stamp,
        "label": args.label or "unnamed",
        "flipped": sorted(flips),
        "set_sign": {k: v for k, v in sorted(wanted.items())},
        "unverified": sorted(unver),
        "neutral_ticks": new_neutral,
    })

    with open(args.calibration, "w") as fh:
        json.dump(cal, fh, indent=2)

    print(f"\nrebased onto posture '{args.label or 'unnamed'}'")
    print(f"Franka reference q: {[round(v, 4) for v in franka['q']]}")
    remaining = [j for j in JOINTS if not by_label[j].get("sign_verified")]
    if remaining:
        print(f"\nneeds live confirmation: {', '.join(remaining)}")
        print("Verify each one alone, at low scale, before raising gain.")
    else:
        print("\nall signs still verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
