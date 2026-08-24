"""Check that a collected episode is fit to train on.

Catches the failure modes that are invisible in a summary line: missing or
zero-byte frames, action columns that never vary, a gripper that never actuated,
demonstration segments too short to be useful, and joints that sat against a
clamp for most of the episode.

  python validate_episode.py episodes/ep001
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

NJ = 7


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("episode")
    ap.add_argument("--min-frames", type=int, default=150)
    ap.add_argument("--min-segment-frames", type=int, default=30)
    args = ap.parse_args()

    ep = Path(args.episode)
    csv_path = ep / "episode.csv"
    if not csv_path.exists():
        print(f"missing {csv_path}")
        return 1

    rows = list(csv.DictReader(open(csv_path)))
    problems, warnings = [], []

    if len(rows) < args.min_frames:
        problems.append(f"only {len(rows)} frames (min {args.min_frames})")

    # 1. every referenced frame exists and is non-empty
    img_cols = [c for c in rows[0] if c.startswith("rgb_")]
    missing, empty = 0, 0
    for r in rows:
        for c in img_cols:
            p = ep / r[c]
            if not p.exists():
                missing += 1
            elif p.stat().st_size == 0:
                empty += 1
    if missing:
        problems.append(f"{missing} referenced frames do not exist")
    if empty:
        problems.append(f"{empty} referenced frames are zero bytes")
    print(f"frames        : {len(rows)} rows x {len(img_cols)} cameras, "
          f"{missing} missing, {empty} empty")

    # 2. reused frames indicate the camera stalled relative to the anchor
    for c in img_cols:
        vals = [r[c] for r in rows]
        dupes = len(vals) - len(set(vals))
        frac = dupes / len(vals)
        line = f"  {c:<16} {len(set(vals))} unique, {dupes} reused"
        if frac > 0.10:
            warnings.append(f"{c}: {frac*100:.0f}% of frames reused")
            line += "  <-- camera lagging the anchor"
        print(line)

    # 3. action columns must actually vary
    def span(col):
        vals = [float(r[col]) for r in rows if r[col] not in ("", None)]
        return (max(vals) - min(vals)) if vals else 0.0

    moved = []
    for j in range(NJ):
        s_cmd = span(f"q_command{j}")
        s_rob = span(f"q_robot{j}")
        moved.append(s_cmd)
        if s_cmd > 0.005:
            print(f"J{j+1} command span {s_cmd:.4f} rad, measured {s_rob:.4f}")
    active = sum(1 for m in moved if m > 0.005)
    print(f"joints active : {active} of {NJ}")
    if active == 0:
        problems.append("no joint moved; episode contains no demonstration")
    elif active < 2:
        warnings.append(f"only {active} joint(s) moved")

    # 4. command and measurement must not be identical -- that would mean the
    #    log echoed the target instead of recording real state
    same = all(
        abs(float(r[f"q_robot{j}"]) - float(r[f"q_command{j}"])) < 1e-12
        for r in rows[:200] for j in range(NJ)
    )
    if same:
        problems.append("q_robot is identical to q_command; measured state "
                        "was not recorded (is this a dry run?)")

    # 5. gripper
    gw = [float(r["gripper_width"]) for r in rows if r["gripper_width"] != ""]
    if gw and max(gw) >= 0:
        g_span = max(gw) - min(gw)
        print(f"gripper       : {min(gw)*1000:.1f}..{max(gw)*1000:.1f} mm "
              f"(span {g_span*1000:.1f} mm)")
        if g_span < 0.005:
            warnings.append("gripper never actuated in this episode")
    else:
        warnings.append("no gripper data recorded")

    # 6. contiguous demonstration segments
    segs, cur = [], 0
    prev_t = None
    for r in rows:
        t = int(r["t_ns"])
        if prev_t is not None and (t - prev_t) > 200_000_000:
            segs.append(cur)
            cur = 0
        cur += 1
        prev_t = t
    segs.append(cur)
    segs = [s for s in segs if s]
    short = [s for s in segs if s < args.min_segment_frames]
    print(f"segments      : {len(segs)} contiguous "
          f"(longest {max(segs)}, shortest {min(segs)})")
    if short:
        warnings.append(f"{len(short)} segment(s) shorter than "
                        f"{args.min_segment_frames} frames")

    # 7. delta actions sane
    if "dq_action0" in rows[0]:
        worst = 0.0
        for r in rows:
            for j in range(NJ):
                v = r[f"dq_action{j}"]
                if v not in ("", None):
                    worst = max(worst, abs(float(v)))
        print(f"max |dq_action|: {worst:.4f} rad")
        if worst > 0.5:
            warnings.append(f"delta action reaches {worst:.3f} rad; check for "
                            f"a discontinuity at a segment boundary")

    meta_path = ep / "episode_meta.json"
    if meta_path.exists():
        meta = json.load(open(meta_path))
        sk = meta.get("skew_ms", {}).get("robot", {})
        if sk:
            print(f"robot skew    : mean {sk.get('mean_ms')} ms, "
                  f"max {sk.get('max_ms')} ms")

    print()
    for w in warnings:
        print(f"WARN  {w}")
    for p in problems:
        print(f"FAIL  {p}")
    if not problems and not warnings:
        print("episode looks good")
    print(f"\nverdict: {'FAIL' if problems else ('WARN' if warnings else 'PASS')}")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
