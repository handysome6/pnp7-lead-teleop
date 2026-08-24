"""Capture the current verified configuration as a restorable known-good state.

Records what the system was actually verified with -- calibration, configs,
servo EEPROM settings, camera identities, and the measured performance -- so a
later session can tell whether anything has drifted, and restore if it has.

  python snapshot_state.py --label "ready-for-collection"
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
from datetime import datetime, timezone
from pathlib import Path

from dynamixel_sdk import PortHandler, PacketHandler

from pnp7.lead import ALL_IDS

ADDR_BAUD, ADDR_RETURN_DELAY, ADDR_TORQUE = 8, 9, 64
CONFIGS = ["conf/full100b.conf", "conf/full50b.conf",
           "conf/j6only.conf", "conf/j7only.conf"]


def servo_state(port_name, baud):
    port, packet = PortHandler(port_name), PacketHandler(2.0)
    if not port.openPort() or not port.setBaudRate(baud):
        return {"error": f"cannot open {port_name} at {baud}"}
    out = {}
    for sid in ALL_IDS:
        vals = {}
        for name, addr in (("baud_code", ADDR_BAUD),
                           ("return_delay", ADDR_RETURN_DELAY),
                           ("torque_enable", ADDR_TORQUE)):
            v, comm, err = packet.read1ByteTxRx(port, sid, addr)
            vals[name] = v if comm == 0 and err == 0 else None
        out[str(sid)] = vals
    port.closePort()
    return out


def cameras():
    try:
        import pyrealsense2 as rs
        return [{
            "serial": d.get_info(rs.camera_info.serial_number),
            "name": d.get_info(rs.camera_info.name),
            "firmware": d.get_info(rs.camera_info.firmware_version),
        } for d in rs.context().query_devices()]
    except Exception as exc:
        return [{"error": str(exc)}]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--label", default="known-good")
    ap.add_argument("--out", default="known_good")
    ap.add_argument("--port", default="/dev/pnp7_lead")
    ap.add_argument("--baud", type=int, default=1000000)
    args = ap.parse_args()

    # The repo root, not this script's directory -- calibration.json, conf/
    # and episodes/ all live one level up now.
    here = Path(__file__).resolve().parents[1]
    out = here / args.out
    out.mkdir(parents=True, exist_ok=True)

    cal = json.load(open(here / "calibration.json"))
    by_label = {m["label"]: m for m in cal["joint_map"]}

    for name in ["calibration.json"] + CONFIGS:
        src = here / name
        if src.exists():
            # Flatten: a snapshot is a flat restorable set, so conf/x.conf
            # lands as x.conf beside calibration.json.
            shutil.copy2(src, out / Path(name).name)

    episodes = []
    ep_dir = here / "episodes"
    if ep_dir.exists():
        for d in sorted(ep_dir.iterdir()):
            meta = d / "episode_meta.json"
            if meta.exists():
                m = json.load(open(meta))
                episodes.append({
                    "name": d.name,
                    "frames": m.get("frames"),
                    "rate_hz": m.get("rate_hz"),
                    "robot_skew_ms": m.get("skew_ms", {}).get("robot"),
                })

    try:
        rev = subprocess.run(["git", "rev-parse", "--short", "HEAD"],
                             cwd=here, capture_output=True, text=True,
                             timeout=5).stdout.strip() or None
    except Exception:
        rev = None

    state = {
        "label": args.label,
        "captured": datetime.now(timezone.utc).isoformat(),
        "git_rev": rev,
        "lead_arm": {
            "port": args.port,
            "baud": args.baud,
            "servos": servo_state(args.port, args.baud),
            "note": "baud code 3 = 1 Mbps, return_delay 0, torque_enable 0 on "
                    "all eight. Factory values were code 1 and 250.",
        },
        "calibration": {
            "signs_verified": cal.get("signs_verified"),
            "joints": {j: {"servo": by_label[j]["lead_servo_id"],
                           "sign": by_label[j]["observed_sign"],
                           "verified": by_label[j].get("sign_verified")}
                       for j in sorted(by_label)},
            "posture_history": cal.get("posture_history", []),
        },
        "cameras": cameras(),
        "configs": [c for c in CONFIGS if (here / c).exists()],
        "accepted_episodes": episodes,
    }

    with open(out / "state.json", "w") as fh:
        json.dump(state, fh, indent=2)

    print(f"snapshot '{args.label}' -> {out}")
    print(f"  configs   : {', '.join(state['configs'])}")
    print(f"  signs     : verified={state['calibration']['signs_verified']}")
    for j, v in state["calibration"]["joints"].items():
        print(f"    {j} servo {v['servo']} sign {v['sign']:+d} "
              f"{'verified' if v['verified'] else 'UNVERIFIED'}")
    bad = [s for s, v in state["lead_arm"]["servos"].items()
           if isinstance(v, dict) and (v.get("baud_code") != 3
                                       or v.get("return_delay") != 0
                                       or v.get("torque_enable") != 0)]
    print(f"  servos    : {len(state['lead_arm']['servos'])} read"
          + (f", UNEXPECTED settings on {bad}" if bad else ", all as expected"))
    print(f"  cameras   : {len(state['cameras'])}")
    print(f"  episodes  : {len(episodes)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
