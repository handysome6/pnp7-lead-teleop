"""Offline audit probes. Uses synthetic episodes; never accesses robot hardware."""
from __future__ import annotations

import copy
import csv
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[1]


def module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    result = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(result)
    return result


def write_csv(path, rows):
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def run(script, *args):
    result = subprocess.run([sys.executable, str(REPO / script), *map(str, args)],
                            text=True, capture_output=True)
    return {"returncode": result.returncode, "stdout": result.stdout,
            "stderr": result.stderr}


def main():
    exporter = module("joint_export", REPO / "collect/export_lerobot.py")
    training = module("ee_export", REPO.parent / "training/convert_to_lerobot.py")
    report = {"data_source": "synthetic only", "checks": {}}
    checks = report["checks"]
    with tempfile.TemporaryDirectory(prefix="pnp7-semantics-") as temp:
        ep = Path(temp)
        times = [1_000_000_000, 1_033_333_333, 2_033_333_333, 2_066_666_666]
        rows = []
        for k, t in enumerate(times):
            row = {"t_ns": t, "dt_s": .001, "lead_seq": k + 1,
                   "state": 2 if k == 1 else 1, "deadman": 1}
            for j in range(7):
                row.update({f"q_robot{j}": .01 * k,
                            f"q_target{j}": .01 * k + .001,
                            f"lead_delta{j}": .02 * k,
                            f"dq_robot{j}": .1, f"tau_robot{j}": .2})
            transform = np.eye(4)
            transform[0, 3] = [.0, .001, .051, .052][k]
            row.update({f"O_T_EE{j}": v for j, v in enumerate(transform.T.ravel())})
            row.update({f"O_F_ext{j}": 0 for j in range(6)})
            row.update(gripper_width=.06, gripper_target=0 if k > 1 else .08,
                       gripper_ticks=1000)
            rows.append(row)
        write_csv(ep / "teleop.csv", rows)
        for role in ("external", "wrist"):
            (ep / f"cam_{role}").mkdir()
            index = []
            for k, t in enumerate(times):
                name = f"{k:06d}.jpg"
                cv2.imwrite(str(ep / f"cam_{role}" / name), np.zeros((8, 8, 3), np.uint8))
                index.append(dict(seq=k, host_monotonic_ns=t,
                                  device_ts_ms=k * 33.333, file=name))
            write_csv(ep / f"cam_{role}_index.csv", index)
        build = run("collect/build_episode.py", "--episode", ep, "--no-events")
        assert build["returncode"] == 0, build
        built = list(csv.DictReader((ep / "episode.csv").open()))
        checks["paused_but_pedal_held"] = {
            "retained_rows": sum(r["state"] == "2" for r in built),
            "builder_exit": build["returncode"]}
        checks["delta_across_one_second_gap"] = {
            "gap_ms": (int(built[2]["t_ns"]) - int(built[1]["t_ns"])) / 1e6,
            "dq_action0": float(built[1]["dq_action0"]),
            "last_row_delta": float(built[-1]["dq_action0"])}
        original = training.load_episode(ep, .08)
        checks["ee_transition_across_gap"] = {
            "dx_m": float(original[1]["action"][0]),
            "source_interval_s": 1.0,
            "state_gripper_at_closed_command_with_open_measurement": float(original[2]["state"][-1])}
        stationary_seam = copy.deepcopy(built)
        for j in range(16):
            stationary_seam[2][f"O_T_EE{j}"] = stationary_seam[1][f"O_T_EE{j}"]
        write_csv(ep / "episode.csv", stationary_seam)
        seam_output = training.load_episode(ep, .08)
        checks["stationary_pose_across_gap"] = {
            "source_interval_s": 1.0,
            "action_pose_delta": seam_output[1]["action"][:6].tolist(),
            "transition_still_emitted": True}
        # Two command switches in 584 one-step labels: a synthetic persistence baseline.
        sparse_switches = []
        for k in range(585):
            sparse_switches.append(dict(built[0],
                                       t_ns=str(1_000_000_000 + round(k * 1e9 / 30)),
                                       gripper_command="0" if 100 <= k < 400 else "0.08"))
        write_csv(ep / "episode.csv", sparse_switches)
        sparse_output = training.load_episode(ep, .08)
        truth = np.array([r["action"][-1] for r in sparse_output])
        copied = np.array([r["state"][-1] for r in sparse_output])
        switches = copied != truth
        checks["gripper_copy_baseline_synthetic"] = {
            "frames": len(truth), "switches": int(switches.sum()),
            "overall_accuracy": float(np.mean(~switches)),
            "overall_l1": float(np.mean(np.abs(truth - copied))),
            "switch_frame_accuracy": float(np.mean(copied[switches] == truth[switches])),
            "scope": "one-step synthetic baseline, not actual checkpoint inference"}
        changed = copy.deepcopy(built)
        for row in changed:
            for j in range(7):
                row[f"q_command{j}"] = str(float(row[f"q_command{j}"]) + 1.0)
            row["gripper_width"] = "0.001"
        write_csv(ep / "episode.csv", changed)
        modified = training.load_episode(ep, .08)
        checks["ee_export_ignores_joint_commands_and_gripper_feedback"] = {
            "states_unchanged": all(np.array_equal(a["state"], b["state"]) for a, b in zip(original, modified)),
            "actions_unchanged": all(np.array_equal(a["action"], b["action"]) for a, b in zip(original, modified))}
        missing = dict(built[0], gripper_width="-1", gripper_command="-1")
        checks["joint_export_missing_gripper"] = {
            "input": -1, "output_state": float(exporter.frame_state(missing)[-1]),
            "output_action": float(exporter.frame_action(missing)[-1])}
        checks["short_pause_segment_detection"] = {
            "gap_ms": 150,
            "segments": len(exporter.split_segments([{"t_ns": "0"}, {"t_ns": "150000000"}]))}

        # A moving 180-frame baseline, valid image files and sufficient gripper span.
        baseline = []
        for k in range(180):
            row = dict(built[0], t_ns=str(1_000_000_000 + round(k * 1e9 / 30)),
                       gripper_width=str(.02 + k * .0001), gripper_command="0.08")
            for j in range(7):
                row[f"q_command{j}"] = str(k * .0001 + .001)
                row[f"q_robot{j}"] = str(k * .0001)
            for role in ("external", "wrist"):
                path = ep / f"cam_{role}" / f"{k:06d}.jpg"
                cv2.imwrite(str(path), np.zeros((8, 8, 3), np.uint8))
                row[f"rgb_{role}"] = f"cam_{role}/{k:06d}.jpg"
            baseline.append(row)
        write_csv(ep / "episode.csv", baseline)
        baseline_result = run("collect/validate_episode.py", ep)
        assert "verdict: PASS" in baseline_result["stdout"], baseline_result
        invalid = copy.deepcopy(baseline)
        invalid[50]["q_command0"] = "nan"
        write_csv(ep / "episode.csv", invalid)
        result = run("collect/validate_episode.py", ep)
        checks["validator_nan"] = result
        write_csv(ep / "episode.csv", baseline)
        for role in ("external", "wrist"):
            for path in (ep / f"cam_{role}").glob("*.jpg"):
                path.write_bytes(b"not an image")
        checks["validator_undecodable_images"] = run("collect/validate_episode.py", ep)
        for role in ("external", "wrist"):
            for path in (ep / f"cam_{role}").glob("*.jpg"):
                cv2.imwrite(str(path), np.zeros((8, 8, 3), np.uint8))
        for row in baseline:
            row["wrist_skew_ms"] = "500"
        write_csv(ep / "episode.csv", baseline)
        (ep / "episode_meta.json").write_text(json.dumps({"skew_ms": {
            "robot": {"mean_ms": 0, "max_ms": 0},
            "wrist": {"mean_ms": 500, "max_ms": 500}}}))
        checks["validator_bad_camera_skew"] = run("collect/validate_episode.py", ep)
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
