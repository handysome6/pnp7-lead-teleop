"""Join a teleop log and camera streams into one VLA training episode.

Both producers stamp with CLOCK_MONOTONIC, so rows are matched by nearest
timestamp with no clock fitting -- the same method as run_sync_audit.py in
~/workspace/andyls.

Frames are the scarce resource (30 Hz vs the bridge's 1 kHz), so the episode is
anchored on one camera and every other stream is matched to it. Rows where the
dead-man was not held are dropped by default: they are not demonstration.

Actions follow roadmap section 14 -- absolute q_command is kept, and the delta
form a_t = q_cmd(t+1) - q_robot(t) is derived alongside it.

  python build_episode.py --episode episodes/ep001
"""
from __future__ import annotations

import argparse
import bisect
import csv
import json
import math
from pathlib import Path

NJ = 7


def read_csv(path):
    with open(path) as fh:
        return list(csv.DictReader(fh))


def nearest(sorted_ts, index, t):
    """Index of the entry in `index` whose timestamp is closest to t."""
    i = bisect.bisect_left(sorted_ts, t)
    best = None
    for j in (i - 1, i, i + 1):
        if 0 <= j < len(sorted_ts):
            d = abs(sorted_ts[j] - t)
            if best is None or d < best[0]:
                best = (d, j)
    return best[1], best[0]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episode", required=True,
                    help="directory holding the camera output")
    ap.add_argument("--teleop-csv", default=None,
                    help="defaults to <episode>/teleop.csv")
    ap.add_argument("--anchor", default="external",
                    help="camera role to anchor timing on")
    ap.add_argument("--no-events", action="store_true",
                    help="即使存在 events.hdf5 也不索引事件流")
    ap.add_argument("--keep-idle", action="store_true",
                    help="keep rows where the dead-man was released")
    ap.add_argument("--max-robot-skew-ms", type=float, default=20.0,
                    help="DROP frames whose nearest robot sample is further "
                         "off than this; cameras usually run longer than the "
                         "bridge, and a frame outside that overlap would "
                         "otherwise be paired with seconds-stale state")
    ap.add_argument("--max-cam-skew-ms", type=float, default=20.0,
                    help="warn if a secondary camera is further off than this")
    args = ap.parse_args()

    ep = Path(args.episode)
    teleop_path = Path(args.teleop_csv) if args.teleop_csv else ep / "teleop.csv"
    if not teleop_path.exists():
        print(f"missing teleop log: {teleop_path}")
        return 1

    robot = read_csv(teleop_path)
    if not robot:
        print("teleop log is empty")
        return 1
    robot_ts = [int(r["t_ns"]) for r in robot]

    cams = {}
    for index_file in sorted(ep.glob("cam_*_index.csv")):
        role = index_file.name[len("cam_"):-len("_index.csv")]
        rows = read_csv(index_file)
        cams[role] = {
            "rows": rows,
            "ts": [int(r["host_monotonic_ns"]) for r in rows],
            "dir": f"cam_{role}",
        }
    if args.anchor not in cams:
        print(f"anchor camera '{args.anchor}' not found; have {list(cams)}")
        return 1

    anchor = cams[args.anchor]
    others = {k: v for k, v in cams.items() if k != args.anchor}

    # 事件流：读取时钟映射，用于把宿主 monotonic 时间反算成事件时钟
    ev_meta = None
    ev_path = ep / "events.hdf5"
    if not args.no_events and ev_path.exists():
        try:
            ev_meta = json.load(open(ep / "events_meta.json"))
        except OSError:
            ev_meta = None

    def to_event_us(mono_ns):
        """宿主 CLOCK_MONOTONIC 纳秒 -> 事件时钟微秒。"""
        cm = ev_meta["clock_map"]
        if cm["slope"] is None:
            return None
        return ((mono_ns - ev_meta["t0_monotonic_ns"]) / 1e3
                - cm["intercept_us"]) / cm["slope"] + ev_meta["event_t_first_us"]

    out_rows = []
    skew = {k: [] for k in cams if k != args.anchor}
    skew["robot"] = []
    dropped_idle = 0
    dropped_skew = 0

    for a_i, a_row in enumerate(anchor["rows"]):
        t = anchor["ts"][a_i]
        r_i, r_d = nearest(robot_ts, robot, t)
        rec = robot[r_i]

        # Outside the bridge's window there is no contemporaneous robot state,
        # so the frame is not a valid observation at all.
        if r_d / 1e6 > args.max_robot_skew_ms:
            dropped_skew += 1
            continue
        if not args.keep_idle and rec["deadman"] != "1":
            dropped_idle += 1
            continue
        skew["robot"].append(r_d / 1e6)

        row = {
            "t_ns": t,
            "robot_skew_ms": round(r_d / 1e6, 3),
            f"rgb_{args.anchor}": f"{anchor['dir']}/{a_row['file']}",
        }
        for role, cam in others.items():
            o_i, o_d = nearest(cam["ts"], cam["rows"], t)
            skew[role].append(o_d / 1e6)
            row[f"rgb_{role}"] = f"{cam['dir']}/{cam['rows'][o_i]['file']}"
            row[f"{role}_skew_ms"] = round(o_d / 1e6, 3)

        if ev_meta is not None:
            # 本帧覆盖的事件窗口 = 上一锚点帧到本帧
            prev_t = anchor["ts"][a_i - 1] if a_i > 0 else t
            e0 = to_event_us(prev_t)
            e1 = to_event_us(t)
            row["event_t_start_us"] = round(e0, 1) if e0 is not None else ""
            row["event_t_end_us"] = round(e1, 1) if e1 is not None else ""

        for j in range(NJ):
            row[f"q_robot{j}"] = rec[f"q_robot{j}"]
            row[f"dq_robot{j}"] = rec[f"dq_robot{j}"]
            row[f"tau_robot{j}"] = rec[f"tau_robot{j}"]
            row[f"q_command{j}"] = rec[f"q_target{j}"]
            row[f"q_master{j}"] = rec[f"lead_delta{j}"]
        for j in range(16):
            row[f"O_T_EE{j}"] = rec[f"O_T_EE{j}"]
        for j in range(6):
            row[f"O_F_ext{j}"] = rec.get(f"O_F_ext{j}", "")
        row["gripper_width"] = rec["gripper_width"]
        row["gripper_command"] = rec["gripper_target"]
        row["gripper_master_ticks"] = rec["gripper_ticks"]
        row["deadman"] = rec["deadman"]
        row["state"] = rec["state"]
        out_rows.append(row)

    # Delta actions: a_t = q_command(t+1) - q_robot(t), per roadmap section 14.
    for k, row in enumerate(out_rows):
        nxt = out_rows[k + 1] if k + 1 < len(out_rows) else row
        for j in range(NJ):
            row[f"dq_action{j}"] = round(
                float(nxt[f"q_command{j}"]) - float(row[f"q_robot{j}"]), 9)
        row["dgripper_action"] = round(
            float(nxt["gripper_command"]) - float(row["gripper_width"]), 9
        ) if float(row["gripper_width"]) >= 0 else ""

    if not out_rows:
        print(f"no rows survived (dropped {dropped_skew} for skew, "
              f"{dropped_idle} idle); was the dead-man ever held?")
        return 1

    out_path = ep / "episode.csv"
    with open(out_path, "w", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(out_rows[0].keys()))
        writer.writeheader()
        writer.writerows(out_rows)

    dur = (out_rows[-1]["t_ns"] - out_rows[0]["t_ns"]) / 1e9
    meta = {
        "events": {
            "file": "events.hdf5",
            "written": ev_meta["events_written"],
            "rate_mev_s": ev_meta["event_rate_mev_s"],
            "hot_fraction": ev_meta["hot_fraction"],
            "drift_ppm": ev_meta["clock_map"]["drift_ppm"],
            "max_residual_ms": ev_meta["clock_map"]["max_residual_ms"],
            "note": "event_t_start_us / event_t_end_us 为每帧覆盖的事件窗口，"
                    "使用事件时钟；读取 hdf5 需要 libH5Zecf.so 插件",
        } if ev_meta is not None else None,
        "frames": len(out_rows),
        "duration_s": round(dur, 3),
        "rate_hz": round(len(out_rows) / dur, 2) if dur > 0 else 0,
        "anchor": args.anchor,
        "dropped_idle_frames": dropped_idle,
        "dropped_skew_frames": dropped_skew,
        "max_robot_skew_ms": args.max_robot_skew_ms,
        "skew_ms": {},
    }
    if ev_meta is not None:
        print(f"events: {ev_meta['events_written']:,} 个 "
              f"({ev_meta['event_rate_mev_s']:.2f} Mev/s), "
              f"热像素滤除 {ev_meta['hot_fraction']*100:.0f}%, "
              f"漂移 {ev_meta['clock_map']['drift_ppm']:+.1f} ppm")
    elif not args.no_events:
        print("events: 无（本 episode 只有 RGB）")

    print(f"episode: {len(out_rows)} frames over {dur:.2f}s "
          f"({meta['rate_hz']} Hz)")
    print(f"dropped {dropped_skew} frames outside the robot log window "
          f"(>{args.max_robot_skew_ms} ms)")
    print(f"dropped {dropped_idle} frames where the dead-man was released")
    print("\nstream alignment to the anchor camera:")
    ok = True
    for name, vals in skew.items():
        if not vals:
            continue
        vals_sorted = sorted(vals)
        stats = {
            "mean_ms": round(sum(vals) / len(vals), 3),
            "p95_ms": round(vals_sorted[int(len(vals) * 0.95)], 3),
            "max_ms": round(max(vals), 3),
        }
        meta["skew_ms"][name] = stats
        flag = ""
        limit = (args.max_robot_skew_ms if name == "robot"
                 else args.max_cam_skew_ms)
        if stats["max_ms"] > limit:
            flag = f"  EXCEEDS {limit} ms"
            ok = False
        print(f"  {name:<10} mean={stats['mean_ms']:6.3f} ms  "
              f"p95={stats['p95_ms']:6.3f} ms  max={stats['max_ms']:6.3f} ms{flag}")

    with open(ep / "episode_meta.json", "w") as fh:
        json.dump(meta, fh, indent=2)
    print(f"\nwritten to {out_path}")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
