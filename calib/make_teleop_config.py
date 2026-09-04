"""Turn calibration.json into the flat key=value config the C++ bridge reads.

The C++ side deliberately parses a simple key=value file rather than JSON, to
match the existing spacemouse_teleop controller and to keep the realtime binary
free of a JSON dependency.

Defaults follow roadmap Step C: J7 only, scale 0.25, conservative limits.

  python make_teleop_config.py --enable J7 --scale 0.25 -o pnp7_teleop.conf
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import re
import sys

JOINTS = ["J1", "J2", "J3", "J4", "J5", "J6", "J7"]


def find_deadman() -> str | None:
    """Locate the dead-man device.

    Prefers the udev symlink, which survives a replug. Event numbers are
    assigned in probe order, so a bare /dev/input/eventN in a config breaks the
    moment the device moves to another port -- which is exactly what happened
    when both devices were moved onto a hub.

    The fallback can no longer search by device name. The SpaceMouse announced
    itself; the button that replaced it publishes no manufacturer, product or
    serial string at all and shows up as the bare "HID 0483:5750", so the only
    thing left to recognise it by is VID:PID and the interface number.
    """
    stable = "/dev/pnp7_deadman"
    if os.path.exists(stable):
        return stable
    # if01 is the keyboard interface carrying the button; if02 is a mouse
    # interface the same device exports and nothing here uses.
    for by_id in ("/dev/input/by-id/usb-0483_5750-if01-event-kbd",):
        if os.path.exists(by_id):
            return by_id
    for path in sorted(glob.glob("/dev/input/event*")):
        name_file = (f"/sys/class/input/{os.path.basename(path)}/device/name")
        try:
            with open(name_file) as fh:
                if "spacemouse" in fh.read().strip().lower():
                    return path
        except OSError:
            continue
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--calibration", default="calibration.json")
    ap.add_argument("-o", "--out", default="conf/pnp7_teleop.conf")
    ap.add_argument("--enable", nargs="+", default=["J7"],
                    help="joints to drive; the rest are held (default: J7)")
    ap.add_argument("--scale", type=float, default=0.25,
                    help="lead-arm to Franka joint gain")
    ap.add_argument("--robot-ip", default="172.16.0.2")
    ap.add_argument("--max-velocity", nargs="+", type=float, default=[0.30],
                    help="rad/s: one value for the whole arm, or seven")
    ap.add_argument("--max-acceleration", nargs="+", type=float, default=[1.50],
                    help="rad/s^2: one value for the whole arm, or seven")
    ap.add_argument("--max-session-delta", type=float, default=0.50,
                    help="max |q_target - q_origin| per joint, rad")
    ap.add_argument("--lowpass-hz", type=float, default=6.0)
    ap.add_argument("--watchdog-ms", type=int, default=100)
    ap.add_argument("--deadman-device", default=None)
    ap.add_argument("--deadman-key", default="KEY_F3",
                    help="key the dead-man button emits. The SpaceMouse sent "
                         "BTN_0; the button that replaced it is a plain HID "
                         "keyboard sending KEY_F3. Find it with find_button.py.")
    ap.add_argument("--no-deadman-grab", action="store_true",
                    help="do not take the button exclusively (EVIOCGRAB). Only "
                         "for debugging -- without the grab the desktop also "
                         "sees every press and acts on its own binding.")
    ap.add_argument("--flip-sign", nargs="+", default=[], metavar="Jn",
                    help="invert these joints for the working POSTURE, without "
                         "touching calibration.json. Use when the lead arm is "
                         "deliberately held in a configuration that inverts a "
                         "joint -- e.g. J5 rotated 180 deg flips J6's apparent "
                         "direction. Calibration keeps the physically verified "
                         "signs; this records a posture-specific override.")
    ap.add_argument("--lead-deadband", type=float, default=2.0,
                    help="encoder counts of hysteresis on the lead arm. A "
                         "servo resting on a count boundary flips between two "
                         "values indefinitely, which at scale 1.0 becomes a "
                         "~1.5 mrad command dither the robot chases audibly. "
                         "0 disables.")
    ap.add_argument("--status-path", default="",
                    help="write live teleop state here for view_cameras.py")
    ap.add_argument("--gripper", action="store_true",
                    help="enable Franka Hand control from the lead trigger")
    ap.add_argument("--gripper-flip", action="store_true",
                    help="invert the trigger mapping (squeeze opens instead)")
    ap.add_argument("--gripper-speed", type=float, default=0.10)
    ap.add_argument("--gripper-binary", action="store_true",
                    help="open/close on a trigger threshold instead of "
                         "tracking width continuously; far more responsive, "
                         "and closing uses grasp() so it holds an object")
    ap.add_argument("--gripper-binary-threshold", type=float, default=0.5,
                    help="trigger fraction above which the hand opens")
    ap.add_argument("--gripper-open-width", type=float, default=0.0,
                    help="how far the hand opens, in m (0 = full range). "
                         "Travel time is the dominant latency when reversing, "
                         "so opening only as far as the task needs cuts it "
                         "proportionally")
    ap.add_argument("--gripper-force", type=float, default=20.0,
                    help="grasp force in N, binary mode only")
    ap.add_argument("--gripper-preempt", type=float, default=0.004,
                    help="abort an in-flight hand move once the trigger has "
                         "diverged by this much (m); smaller tracks the "
                         "operator more closely at the cost of more aborts")
    args = ap.parse_args()

    with open(args.calibration) as fh:
        cal = json.load(fh)

    by_label = {m["label"]: m for m in cal["joint_map"]}
    missing = [j for j in JOINTS if j not in by_label]
    if missing:
        print(f"calibration is missing {missing}", file=sys.stderr)
        return 1

    bad = [j for j in JOINTS if by_label[j]["status"] != "ok"]
    if bad:
        print(f"refusing: joints {bad} are not marked ok in {args.calibration}",
              file=sys.stderr)
        return 1

    ids = [by_label[j]["lead_servo_id"] for j in JOINTS]
    if len(set(ids)) != len(ids):
        print(f"refusing: duplicate servo ids in mapping: {ids}", file=sys.stderr)
        return 1

    enable = {j.upper() for j in args.enable}
    if enable == {"ALL"}:
        enable = set(JOINTS)
    unknown = enable - set(JOINTS)
    if unknown:
        print(f"unknown joints: {sorted(unknown)}", file=sys.stderr)
        return 1

    for name, vals in (("--max-velocity", args.max_velocity),
                       ("--max-acceleration", args.max_acceleration)):
        if len(vals) not in (1, 7):
            print(f"{name} takes 1 or 7 values, got {len(vals)}",
                  file=sys.stderr)
            return 1

    deadman = args.deadman_device or find_deadman()
    if not deadman:
        print("no dead-man device found; check that the button is plugged in "
              "and that 99-pnp7-lead.rules is installed, or pass "
              "--deadman-device explicitly", file=sys.stderr)
        return 1

    flips = {j.upper() for j in args.flip_sign}
    unknown_flip = flips - set(JOINTS)
    if unknown_flip:
        print(f"unknown joints in --flip-sign: {sorted(unknown_flip)}",
              file=sys.stderr)
        return 1

    mask = "".join("1" if j in enable else "0" for j in JOINTS)
    sign_vals = [by_label[j]["observed_sign"] * (-1 if j in flips else 1)
                 for j in JOINTS]
    signs = " ".join(str(v) for v in sign_vals)
    servo_ids = " ".join(str(i) for i in ids)
    scales = " ".join(f"{args.scale:.4f}" for _ in JOINTS)

    lines = [
        "# Generated by make_teleop_config.py -- do not hand-edit ids/signs;",
        "# regenerate from calibration.json instead.",
        f"# calibration created: {cal.get('created')}",
        f"# signs_verified: {cal.get('signs_verified')}",
    ] + ([
        f"# POSTURE OVERRIDE: {', '.join(sorted(flips))} inverted relative to "
        f"calibration.",
        "# Valid only while the lead arm is held in the posture this was made "
        "for.",
        "# Check with check_correspondence.py before recording.",
    ] if flips else []) + [
        "",
        f"lead_port={cal.get('port', '/dev/pnp7_lead')}",
        f"lead_baud={cal.get('baud', 1000000)}",
        f"robot_ip={args.robot_ip}",
        f"deadman_device={deadman}",
        f"deadman_key={args.deadman_key}",
        f"deadman_grab={0 if args.no_deadman_grab else 1}",
        "",
        "# J1..J7 in order",
        f"lead_servo_id={servo_ids}",
        f"sign={signs}",
        f"scale={scales}",
        f"enabled_joints={mask}",
        "",
        f"lowpass_hz={args.lowpass_hz}",
        f"max_joint_velocity={' '.join(str(v) for v in args.max_velocity)}",
        f"max_joint_acceleration="
        f"{' '.join(str(v) for v in args.max_acceleration)}",
        f"max_session_delta={args.max_session_delta}",
        f"watchdog_ms={args.watchdog_ms}",
        f"status_path={args.status_path}",
        f"lead_deadband={args.lead_deadband}",
    ]

    # Where `pnp7_teleop home` drives the arm. Taken from the pose the
    # calibration was captured at, because that is the configuration the lead
    # arm's neutral corresponds to -- homing anywhere else would leave the two
    # arms out of correspondence from the first take. Omitted rather than
    # guessed when the calibration has no rest pose: `home` refuses to run
    # without the key, which is better than moving to a default nobody chose.
    rest = (cal.get("franka_rest_pose") or {}).get("q")
    if rest and len(rest) == len(JOINTS):
        lines += [
            "",
            "# Joint pose `home` mode drives to, from the calibration posture.",
            f"home_qpos={' '.join(f'{v:.6f}' for v in rest)}",
        ]

    grip = cal.get("gripper", {})
    lo, hi = grip.get("min_ticks"), grip.get("max_ticks")
    if args.gripper:
        if lo is None or hi is None:
            print("calibration has no gripper trigger range; re-run "
                  "calibrate.py --only GRIP", file=sys.stderr)
            return 1
        # Trigger at rest reads high and squeezing drives it low, so the high
        # end maps to an open hand unless --gripper-flip says otherwise.
        closed, opened = (hi, lo) if args.gripper_flip else (lo, hi)
        lines += [
            "",
            "gripper_enabled=1",
            f"gripper_ticks_closed={closed}",
            f"gripper_ticks_open={opened}",
            f"gripper_speed={args.gripper_speed}",
            "gripper_min_change=0.002",
            f"gripper_preempt={args.gripper_preempt}",
            f"gripper_binary={1 if args.gripper_binary else 0}",
            f"gripper_binary_threshold={args.gripper_binary_threshold}",
            f"gripper_force={args.gripper_force}",
            f"gripper_open_width={args.gripper_open_width}",
        ]
    else:
        lines += ["", "gripper_enabled=0"]

    with open(args.out, "w") as fh:
        fh.write("\n".join(lines) + "\n")

    print(f"wrote {args.out}")
    print(f"  enabled joints : {sorted(enable)}  (mask {mask})")
    print(f"  scale          : {args.scale}")
    print(f"  signs          : {signs}"
          + (f"   (posture flip: {', '.join(sorted(flips))})" if flips else ""))
    print(f"  servo ids      : {servo_ids}")
    print(f"  deadman        : {deadman}  key {args.deadman_key}"
          + ("  (NOT grabbed -- desktop still sees it)"
             if args.no_deadman_grab else "  (exclusive)"))
    if rest and len(rest) == len(JOINTS):
        print("  home pose      : "
              + " ".join(f"{v:.3f}" for v in rest))
    else:
        print("  home pose      : ABSENT -- `pnp7_teleop home` will refuse; "
              "re-run calibrate.py to capture franka_rest_pose")
    print(f"  lead deadband  : {args.lead_deadband} counts "
          f"({args.lead_deadband * 360.0 / 4096.0:.3f} deg of lead motion)")
    if args.gripper:
        closed, opened = (hi, lo) if args.gripper_flip else (lo, hi)
        mode = "binary" if args.gripper_binary else "continuous"
        print(f"  gripper        : enabled ({mode}), ticks {closed} (closed) "
              f"-> {opened} (open)")
        if args.gripper_binary:
            print(f"                   threshold {args.gripper_binary_threshold}"
                  f", grasp force {args.gripper_force} N")
        ow = args.gripper_open_width
        print("                   opens to "
              + (f"{ow*1000:.0f} mm" if ow > 0 else "full range")
              + f", speed {args.gripper_speed} m/s")
    else:
        print("  gripper        : disabled")
    if not cal.get("signs_verified"):
        print("\n  NOTE: signs are still provisional -- confirm each joint at "
              "low scale before enabling more.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
