"""Record that a joint's direction has been confirmed under supervision.

Signs come out of calibrate.py provisional. Each one is only trustworthy once
the joint has been driven at low scale and the operator confirmed the Franka
moved the intended way. This records that, so make_teleop_config.py stops
warning and higher scales can be justified.

  python mark_verified.py J7                 # confirm as calibrated
  python mark_verified.py J5 --flip          # confirm, but the sign was wrong
  python mark_verified.py --show
"""
from __future__ import annotations

import argparse
import json
import sys
from datetime import datetime, timezone

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("joints", nargs="*", help="e.g. J5 J6 J7")
    ap.add_argument("--calibration", default="calibration.json")
    ap.add_argument("--flip", action="store_true",
                    help="the joint moved the wrong way: invert its sign")
    ap.add_argument("--show", action="store_true")
    args = ap.parse_args()

    with open(args.calibration) as fh:
        cal = json.load(fh)
    by_label = {m["label"]: m for m in cal["joint_map"]}

    if args.show or not args.joints:
        print(f"{'joint':<7}{'servo':<7}{'sign':<7}{'verified'}")
        for j in JOINTS:
            m = by_label.get(j, {})
            v = m.get("sign_verified")
            print(f"{j:<7}{m.get('lead_servo_id','?'):<7}"
                  f"{m.get('observed_sign','?'):+}      "
                  f"{v if v else 'no'}")
        print(f"\nsigns_verified (all): {cal.get('signs_verified')}")
        return 0

    stamp = datetime.now(timezone.utc).isoformat()
    for label in (j.upper() for j in args.joints):
        if label not in by_label:
            print(f"unknown joint {label}", file=sys.stderr)
            return 1
        m = by_label[label]
        if args.flip:
            m["observed_sign"] = -m["observed_sign"]
            print(f"{label}: sign flipped to {m['observed_sign']:+d}")
        m["sign_verified"] = stamp
        print(f"{label}: verified (servo {m['lead_servo_id']}, "
              f"sign {m['observed_sign']:+d})")

    cal["signs_verified"] = all(
        by_label[j].get("sign_verified") for j in JOINTS)
    with open(args.calibration, "w") as fh:
        json.dump(cal, fh, indent=2)

    remaining = [j for j in JOINTS if not by_label[j].get("sign_verified")]
    if remaining:
        print(f"\nstill provisional: {', '.join(remaining)}")
    else:
        print("\nall 7 joint directions verified")
    return 0


if __name__ == "__main__":
    sys.exit(main())
