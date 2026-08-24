"""Reclaim space from a validated episode.

The recorder saves every frame it captures; the episode keeps only the frames
where the dead-man was held. The remainder is typically ~60% of the bytes.

Pruning is irreversible and forfeits the ability to rebuild the episode with a
different filter (--keep-idle, a different anchor, a wider skew tolerance), so
this reports by default and only deletes when told to. Run validate_episode.py
first: pruning an episode you have not validated can leave you with neither the
frames nor a usable dataset.

  python prune_episode.py episodes/ep001            # report only
  python prune_episode.py episodes/ep001 --yes      # delete + gzip
"""
from __future__ import annotations

import argparse
import csv
import gzip
import shutil
import sys
from pathlib import Path


def human(n):
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024:
            return f"{n:.1f} {unit}"
        n /= 1024
    return f"{n:.1f} TB"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("episode")
    ap.add_argument("--yes", action="store_true", help="actually delete")
    ap.add_argument("--keep-teleop-csv", action="store_true",
                    help="do not gzip the 1 kHz log")
    args = ap.parse_args()

    ep = Path(args.episode)
    csv_path = ep / "episode.csv"
    if not csv_path.exists():
        print(f"missing {csv_path}; refusing to prune an unbuilt episode")
        return 1

    rows = list(csv.DictReader(open(csv_path)))
    if not rows:
        print("episode.csv is empty; refusing to prune")
        return 1

    img_cols = [c for c in rows[0] if c.startswith("rgb_")]
    referenced = {r[c] for r in rows for c in img_cols}

    unused, freed = [], 0
    for cam_dir in sorted(ep.glob("cam_*")):
        if not cam_dir.is_dir():
            continue
        for f in cam_dir.iterdir():
            rel = f"{cam_dir.name}/{f.name}"
            if rel not in referenced:
                unused.append(f)
                freed += f.stat().st_size

    teleop = ep / "teleop.csv"
    gz_saving = 0
    if teleop.exists() and not args.keep_teleop_csv:
        raw = teleop.stat().st_size
        gz_saving = raw - len(gzip.compress(teleop.read_bytes(), 6))

    print(f"referenced frames : {len(referenced)}")
    print(f"unused frames     : {len(unused)}  ({human(freed)})")
    if gz_saving:
        print(f"teleop.csv gzip   : saves {human(gz_saving)}")
    print(f"total reclaimable : {human(freed + gz_saving)}")

    if not args.yes:
        print("\nreport only. re-run with --yes to delete.")
        print("run validate_episode.py first if you have not already.")
        return 0

    for f in unused:
        f.unlink()
    if gz_saving:
        with open(teleop, "rb") as src, gzip.open(f"{teleop}.gz", "wb", 6) as dst:
            shutil.copyfileobj(src, dst)
        teleop.unlink()
    print(f"\ndeleted {len(unused)} frames, reclaimed "
          f"{human(freed + gz_saving)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
