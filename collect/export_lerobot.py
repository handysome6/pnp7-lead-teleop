"""Convert built episodes into a LeRobot dataset for RLinf training.

This is deliberately an offline batch step, not part of collection. The clipping
decision -- which frames count as demonstration -- already happened in
`build_episode.py`, from the `deadman` column of `teleop.csv`, and it can be
re-derived at any time by re-running that with different settings. Conversion
sits downstream of that, so changing your mind about the dataset never means
going back to the robot.

It also runs in a different Python environment from everything else here. The
repo's `.venv` is pinned to five packages and has neither `lerobot` nor `torch`;
this needs both, so it runs under the RLinf environment:

    export PYTHONPATH=$HOME/workspace/andyls/RLinf:$PYTHONPATH
    $RLINF_PYTHON collect/export_lerobot.py episodes/ \\
        --out ~/datasets/pnp7_lerobot \\
        --task "pick the block and place it in the bin"

The writer itself is RLinf's `LeRobotDatasetWriter`, reused rather than
reimplemented: it depends only on a compat shim and a logger, with no Ray and no
environment, so it lifts out of the framework cleanly.

## Segments

`build_episode.py` drops the rows where the dead-man was released, so a take in
which the pedal was released and re-pressed comes out as one continuous table
with a *time* jump at each seam. The robot pose is continuous across such a jump
-- the bridge freezes the target on release -- but wall-clock time is not, so the
action delta spanning a seam is not a real one-step transition.

By default each seam is recorded in a per-frame `segment_id`, leaving the
training pipeline to honour or ignore it. `--split-segments` instead emits every
segment as its own LeRobot episode, which makes the seams structurally
impossible to cross at the cost of shorter episodes.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

NJ = 7

#: Same rule `validate_episode.py` uses to find contiguous stretches: rows are
#: at the camera rate, so a gap far larger than a frame interval is a seam where
#: released-pedal rows were dropped.
SEGMENT_GAP_NS = 200_000_000


def read_episode(path: Path) -> list[dict[str, str]]:
    with open(path / "episode.csv") as fh:
        return list(csv.DictReader(fh))


def split_segments(rows: list[dict[str, str]]) -> list[list[dict[str, str]]]:
    """Break rows wherever `t_ns` jumps, i.e. wherever F3 was released."""
    if not rows:
        return []
    segments: list[list[dict[str, str]]] = [[rows[0]]]
    for previous, row in zip(rows, rows[1:]):
        if int(row["t_ns"]) - int(previous["t_ns"]) > SEGMENT_GAP_NS:
            segments.append([])
        segments[-1].append(row)
    return segments


def load_image(episode: Path, relative: str):
    import cv2

    image = cv2.imread(str(episode / relative))
    if image is None:
        raise RuntimeError(f"unreadable frame: {episode / relative}")
    # LeRobot stores RGB; the recorder wrote BGR because that is what the
    # RealSense stream and cv2.imwrite both use.
    return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)


def frame_state(row: dict[str, str]) -> np.ndarray:
    """Measured configuration: seven joints plus the measured gripper width."""
    width = float(row.get("gripper_width") or -1.0)
    return np.asarray(
        [float(row[f"q_robot{j}"]) for j in range(NJ)] + [max(width, 0.0)],
        dtype=np.float32)


def frame_action(row: dict[str, str]) -> np.ndarray:
    """The commanded configuration.

    `q_command` is what the safety chain actually sent to the arm, which the
    roadmap is emphatic about: the training action is never the raw lead-arm
    encoder, because the master was not what the robot did.
    """
    target = float(row.get("gripper_command") or -1.0)
    return np.asarray(
        [float(row[f"q_command{j}"]) for j in range(NJ)] + [max(target, 0.0)],
        dtype=np.float32)


def build_frames(episode: Path, rows: list[dict[str, str]], task: str,
                 segment_ids: list[int], wrist_key: str | None) -> list[dict]:
    frames = []
    last = len(rows) - 1
    for index, row in enumerate(rows):
        frame = {
            "image": load_image(episode, row["rgb_external"]),
            "state": frame_state(row),
            "actions": frame_action(row),
            "task": task,
            "done": np.asarray([index == last], dtype=bool),
            # Every episode here was kept by the operator; that decision is the
            # success signal, since the teleop task publishes no reward.
            "is_success": np.asarray([True], dtype=bool),
            # Wholly human-demonstrated, so the flag is uniformly true -- the
            # same thing collect_real_data.py does for its teleop trajectories.
            "intervene_flag": np.asarray([True], dtype=bool),
            "segment_id": np.asarray([segment_ids[index]], dtype=np.uint8),
        }
        if wrist_key and row.get("rgb_wrist"):
            frame[wrist_key] = load_image(episode, row["rgb_wrist"])
        frames.append(frame)
    return frames


def find_episodes(targets: list[str]) -> list[Path]:
    """Accept episode directories, or parents containing them."""
    found: list[Path] = []
    for target in targets:
        path = Path(target)
        if (path / "episode.csv").is_file():
            found.append(path)
            continue
        found.extend(sorted(child for child in path.iterdir()
                            if (child / "episode.csv").is_file()))
    return found


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("episodes", nargs="+",
                    help="episode directories, or a directory holding them")
    ap.add_argument("--out", required=True, help="dataset root to write")
    ap.add_argument("--task", required=True,
                    help="task instruction stored with every frame")
    ap.add_argument("--fps", type=int, default=30,
                    help="metadata only; episodes are anchored on the external "
                         "camera, which runs at 30")
    ap.add_argument("--robot-type", default="panda")
    ap.add_argument("--split-segments", action="store_true",
                    help="emit each F3 interval as its own LeRobot episode "
                         "instead of one episode carrying segment_id")
    ap.add_argument("--dry-run", action="store_true",
                    help="report what would be written and exit")
    args = ap.parse_args()

    episodes = find_episodes(args.episodes)
    if not episodes:
        print("no built episodes found (looked for episode.csv)",
              file=sys.stderr)
        return 1

    plan = []
    total_frames = 0
    for episode in episodes:
        rows = read_episode(episode)
        segments = split_segments(rows)
        plan.append((episode, rows, segments))
        total_frames += len(rows)
        print(f"{episode.name:<12} {len(rows):5d} frames  "
              f"{len(segments)} segment(s)")
    print(f"\n{len(episodes)} episode(s), {total_frames} frames total")
    if args.split_segments:
        print(f"--split-segments: {sum(len(s) for _, _, s in plan)} "
              "LeRobot episodes")

    if args.dry_run:
        return 0

    from rlinf.data.storage.lerobot import LeRobotDatasetWriter

    first_rows = plan[0][1]
    if not first_rows:
        print("the first episode has no rows", file=sys.stderr)
        return 1
    sample = load_image(plan[0][0], first_rows[0]["rgb_external"])
    wrist_key = "wrist_image" if first_rows[0].get("rgb_wrist") else None
    wrist_keys = None
    if wrist_key:
        wrist_sample = load_image(plan[0][0], first_rows[0]["rgb_wrist"])
        wrist_keys = {wrist_key: wrist_sample.shape}

    writer = LeRobotDatasetWriter()
    # `repo_id` is used as a filesystem path, and the rank_/id_ layout is what
    # CollectEpisode produces, so RLinf's readers find this dataset unchanged.
    out_root = Path(args.out) / "rank_0" / "id_0"
    writer.create(
        repo_id=str(out_root),
        robot_type=args.robot_type,
        fps=args.fps,
        image_shape=sample.shape,
        state_dim=8,
        action_dim=8,
        has_image=True,
        wrist_image_keys=wrist_keys,
        has_intervene_flag=True,
        has_segment_id=True,
    )

    written = 0
    for episode, rows, segments in plan:
        if args.split_segments:
            groups = segments
        else:
            groups = [rows]
        for group in groups:
            if not group:
                continue
            if args.split_segments:
                ids = [0] * len(group)
            else:
                ids = []
                for index, segment in enumerate(segments):
                    # uint8 in the schema; wrap rather than overflow on a take
                    # with an implausible number of pedal presses.
                    ids.extend([index % 256] * len(segment))
            writer.add_episode(
                build_frames(episode, group, args.task, ids, wrist_key))
            written += 1

    writer.finalize()
    manifest = {
        "source_episodes": [str(e) for e, _, _ in plan],
        "lerobot_episodes": written,
        "frames": total_frames,
        "task": args.task,
        "fps": args.fps,
        "split_segments": args.split_segments,
    }
    (Path(args.out) / "export_manifest.json").write_text(
        json.dumps(manifest, indent=2))
    print(f"\nwrote {written} LeRobot episode(s) to {out_root}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
