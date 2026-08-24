"""Guided calibration of the PNP-7 lead arm -> Franka joint mapping.

Roadmap Step A. For each Franka joint in turn you move ONE lead-arm joint;
the wizard watches all 8 servos and identifies which one moved, by how much,
and in which direction. It writes calibration.json for the teleop bridge.

Nothing is written to the servo bus and the Franka is never commanded. The
Franka is only read, to capture the pose it is resting in.

Joint SIGNS are recorded as observed here but are deliberately NOT trusted:
they are confirmed during the supervised single-joint test (Step C) where a
wrong sign is obvious and harmless at low scale.

  python calibrate.py --out calibration.json
"""
from __future__ import annotations

import argparse
import json
import statistics
import sys
import time
from datetime import datetime, timezone

from pnp7_lead import ALL_IDS, ARM_IDS, GRIPPER_ID, PNP7Lead

# What the operator is told to do for each Franka joint. Kept deliberately
# physical rather than referring to DH conventions.
JOINT_PROMPTS = [
    ("J1", "base yaw -- rotate the whole lead arm left/right about its base"),
    ("J2", "shoulder pitch -- swing the upper arm forward/back"),
    ("J3", "upper-arm roll -- twist the upper arm about its own axis"),
    ("J4", "elbow -- bend the elbow"),
    ("J5", "forearm roll -- twist the forearm about its own axis"),
    ("J6", "wrist pitch -- bend the wrist up/down"),
    ("J7", "wrist roll -- twist the final wrist axis"),
]

SETTLE_S = 0.4
MIN_TRAVEL_TICKS = 40          # below this we assume nothing really moved
CROSSTALK_RATIO = 0.35         # 2nd-largest mover must be under this fraction


def collect(lead: PNP7Lead, duration: float) -> list[list[int]]:
    """Gather raw tick vectors for `duration` seconds."""
    frames = []
    t_end = time.time() + duration
    while time.time() < t_end:
        s = lead.latest()
        if s is not None:
            frames.append(list(s.ticks))
        time.sleep(0.005)
    return frames


def snapshot(lead: PNP7Lead) -> list[int]:
    frames = collect(lead, SETTLE_S)
    if not frames:
        raise RuntimeError("no samples from lead arm")
    return [round(statistics.median(f[i] for f in frames))
            for i in range(len(ALL_IDS))]


def read_franka(ip: str) -> dict | None:
    """Best-effort read of the Franka resting pose (optional context)."""
    try:
        import subprocess
        import csv
        import os
        tmp = "/tmp/pnp7_cal_franka.csv"
        env = dict(os.environ)
        env["LD_LIBRARY_PATH"] = (
            "/opt/openrobots/lib:"
            + os.path.expanduser("~/catkin_franka/libfranka/build")
            + ":" + env.get("LD_LIBRARY_PATH", "")
        )
        binary = os.path.expanduser("~/workspace/andyls/bin/read_franka_state")
        subprocess.run([binary, ip, tmp, "1", "5"], check=True, timeout=25,
                       capture_output=True, env=env)
        rows = list(csv.DictReader(open(tmp)))
        last = rows[-1]
        return {
            "q": [float(x) for x in last["q"].split(";")],
            "robot_mode": int(last["robot_mode"]),
            "gripper_width": float(last["gripper_width"]),
        }
    except Exception as exc:
        print(f"  (could not read Franka pose: {exc})")
        return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/pnp7_lead")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--robot-ip", default="172.16.0.2")
    ap.add_argument("--out", default="calibration.json")
    ap.add_argument("--move-seconds", type=float, default=6.0,
                    help="window to move each joint in")
    ap.add_argument("--only", nargs="+", metavar="Jn",
                    help="redo just these joints, merging into an existing "
                         "calibration file (e.g. --only J6)")
    args = ap.parse_args()

    lead = PNP7Lead(args.port, args.baud)
    lead.open()
    lead.assert_torque_disabled()
    lead.start()
    print("lead arm connected, all servos passive\n")

    print("Put the lead arm in a comfortable neutral pose and leave it still.")
    input("Press Enter to capture the neutral pose... ")
    neutral = snapshot(lead)
    print(f"  neutral ticks: {neutral}\n")

    franka_pose = read_franka(args.robot_ip)
    if franka_pose:
        print(f"  Franka resting q: "
              f"{[round(v, 4) for v in franka_pose['q']]}")
        print(f"  Franka mode={franka_pose['robot_mode']} "
              f"gripper={franka_pose['gripper_width']:.4f} m\n")

    existing: dict = {}
    if args.only:
        try:
            existing = json.load(open(args.out))
            print(f"merging into existing {args.out}\n")
        except OSError:
            print(f"--only given but {args.out} not found; doing a full pass\n")
            args.only = None

    wanted = {j.upper() for j in args.only} if args.only else None
    mapping = []
    used_ids: set[int] = set()

    for idx, (label, description) in enumerate(JOINT_PROMPTS):
        if wanted and label not in wanted:
            prior = next((m for m in existing.get("joint_map", [])
                          if m["label"] == label), None)
            if prior:
                mapping.append(prior)
                used_ids.add(prior["lead_servo_id"])
            continue
        print(f"--- Franka {label}: {description} ---")
        print("    Move ONLY that lead-arm joint, through a good sweep, "
              "then return near neutral.")
        input(f"    Press Enter to start the {args.move_seconds:.0f}s window... ")

        print("    recording... ", end="", flush=True)
        frames = collect(lead, args.move_seconds)
        print(f"{len(frames)} samples")

        if not frames:
            print("    ERROR: no data, skipping\n")
            continue

        travel = []
        for i in range(len(ALL_IDS)):
            col = [f[i] for f in frames]
            travel.append(max(col) - min(col))

        order = sorted(range(len(travel)), key=lambda i: travel[i], reverse=True)
        best, second = order[0], order[1]
        best_id = ALL_IDS[best]

        # Signed excursion: which way did it go relative to neutral?
        col = [f[best] for f in frames]
        far = max(col, key=lambda v: abs(v - neutral[best]))
        sign = 1 if far >= neutral[best] else -1

        status = "ok"
        if travel[best] < MIN_TRAVEL_TICKS:
            status = "no-motion"
        elif travel[second] > travel[best] * CROSSTALK_RATIO:
            status = "ambiguous"
        elif best_id in used_ids:
            status = "duplicate"

        deg = travel[best] * 360.0 / 4096.0
        print(f"    -> servo id {best_id}: travel {travel[best]} ticks "
              f"({deg:.1f} deg), observed sign {sign:+d}  [{status}]")
        if status == "ambiguous":
            print(f"       (id {ALL_IDS[second]} also moved "
                  f"{travel[second]} ticks -- joints may be coupled)")
        if status == "duplicate":
            print(f"       (id {best_id} was already claimed by an earlier joint)")
        print()

        used_ids.add(best_id)
        mapping.append({
            "franka_joint": idx,
            "label": label,
            "lead_servo_id": best_id,
            "observed_sign": sign,
            "travel_ticks": travel[best],
            "travel_deg": round(deg, 2),
            "neutral_ticks": neutral[best],
            "status": status,
            "all_travel": {str(ALL_IDS[i]): travel[i] for i in range(len(ALL_IDS))},
        })

    # Gripper trigger
    if wanted and "GRIP" not in wanted:
        grip = existing.get("gripper", {})
        print("--- Gripper trigger: keeping existing ---\n")
        frames = []
        gcol = []
    else:
        print("--- Gripper trigger ---")
        input("    Squeeze and release the lead-arm trigger fully, then Enter... ")
        frames = collect(lead, args.move_seconds)
        gcol = [f[ALL_IDS.index(GRIPPER_ID)] for f in frames] if frames else []
        grip = {
            "lead_servo_id": GRIPPER_ID,
            "min_ticks": min(gcol) if gcol else None,
            "max_ticks": max(gcol) if gcol else None,
            "neutral_ticks": neutral[ALL_IDS.index(GRIPPER_ID)],
        }
    if gcol:
        print(f"    -> range {grip['min_ticks']}..{grip['max_ticks']} "
              f"({(grip['max_ticks'] - grip['min_ticks']) * 360.0 / 4096.0:.1f} deg)\n")

    cal = {
        "created": datetime.now(timezone.utc).isoformat(),
        "port": args.port,
        "baud": args.baud,
        "neutral_ticks": neutral,
        "servo_ids": list(ALL_IDS),
        "arm_ids": list(ARM_IDS),
        "joint_map": mapping,
        "gripper": grip,
        "franka_rest_pose": franka_pose,
        "signs_verified": False,
        "note": "observed_sign is provisional; confirm in the supervised "
                "single-joint test before raising scale above 0.25",
    }
    with open(args.out, "w") as fh:
        json.dump(cal, fh, indent=2)

    lead.close()

    print("=" * 62)
    print(f"written to {args.out}\n")
    print(f"{'Franka':<8}{'lead id':<9}{'sign':<7}{'travel':<12}{'status'}")
    for m in mapping:
        print(f"{m['label']:<8}{m['lead_servo_id']:<9}{m['observed_sign']:+d}      "
              f"{m['travel_deg']:>6.1f} deg  {m['status']}")

    bad = [m for m in mapping if m["status"] != "ok"]
    ids = [m["lead_servo_id"] for m in mapping]
    if bad:
        print(f"\n{len(bad)} joint(s) need another pass: "
              f"{', '.join(m['label'] for m in bad)}")
    if len(set(ids)) != len(ids):
        print("\nWARNING: the same servo was picked for more than one joint.")
    if not bad and len(set(ids)) == len(ids):
        print("\nAll 7 joints resolved to distinct servos.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
