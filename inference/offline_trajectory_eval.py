"""Read-only dataset replay against the deployed inference server; NO robot APIs.

Each prediction starts at a recorded observation. Future recorded states are
used only as references, never as model inputs for that prediction. Predictions
are not concatenated into an unsupported open-loop episode rollout.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path
import socket
import time

import cv2
import numpy as np
import pyarrow.parquet as pq
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from protocol import send_frame, recv_frame, encode_json, decode_json, encode_inference_request

MODEL = "pi05_pnp7_40merged_step7500"


def rotation(rpy):
    x, y, z = rpy
    cx, sx, cy, sy, cz, sz = math.cos(x), math.sin(x), math.cos(y), math.sin(y), math.cos(z), math.sin(z)
    return np.array([[cz*cy, cz*sy*sx-sz*cx, cz*sy*cx+sz*sx],
                     [sz*cy, sz*sy*sx+cz*cx, sz*sy*cx-cz*sx], [-sy, cy*sx, cy*cx]])


def state_rotation(state):
    a, b = state[3:6].copy(), state[6:9].copy()
    a /= np.linalg.norm(a)
    b -= a * np.dot(a, b)
    b /= np.linalg.norm(b)
    return np.column_stack((a, b, np.cross(a, b)))


def angle(a, b):
    return np.degrees(np.arccos(np.clip((np.trace(a.T @ b) - 1) / 2, -1, 1)))


def integrate(state, actions):
    p, r = state[:3].copy(), state_rotation(state)
    positions, rotations = [], []
    for action in actions:
        p = p + action[:3]  # base-frame translation, not rotated by tool pose
        r = r @ rotation(action[3:6])  # body-frame relative rotation
        positions.append(p.copy())
        rotations.append(r.copy())
    return np.array(positions), np.array(rotations)


def video(path, n):
    capture = cv2.VideoCapture(str(path))
    frames = []
    try:
        while len(frames) < n:
            ok, image = capture.read()
            if not ok:
                raise RuntimeError("video ended before dataset rows: " + str(path))
            if image.shape != (224, 224, 3):
                raise RuntimeError("unexpected training image shape")
            frames.append(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
    finally:
        capture.release()
    return frames


def stats(values):
    a = np.asarray(values, dtype=float)
    if not len(a):
        return {"n": 0}
    return dict(n=len(a), mean=float(a.mean()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), max=float(a.max()))


def summarize(rows):
    result = {}
    for horizon in (1, 5, 10):
        selected = [r for r in rows if r["horizon"] == horizon]
        result[str(horizon)] = {key: stats([r[key] for r in selected]) for key in
            ("position_mm", "rotation_deg", "hold_position_mm", "constant_velocity_mm")}
        result[str(horizon)]["gripper_accuracy"] = float(np.mean([r["grip_correct"] for r in selected]))
        result[str(horizon)]["hold_gripper_accuracy"] = float(np.mean([r["hold_grip_correct"] for r in selected]))
        moving = [r for r in selected if r["hold_position_mm"] >= 10]
        result[str(horizon)]["moving_gt_at_least_10mm"] = stats([r["position_mm"] for r in moving])
        directional = [r["direction_cosine"] for r in selected if r["direction_cosine"] is not None]
        result[str(horizon)]["direction_cosine_gt_at_least_5mm"] = stats(directional)
        transitions = [r for r in selected if not r["hold_grip_correct"]]
        result[str(horizon)]["grip_change_windows"] = dict(n=len(transitions),
            correct=sum(r["grip_correct"] for r in transitions))
    return result


def plot_episode(out, name, state, samples, frames, wrist, fps):
    plt.rcParams.update({"font.size": 10, "axes.spines.top": False, "axes.spines.right": False})
    fig = plt.figure(figsize=(15, 10), layout="constrained")
    ax = fig.add_subplot(2, 2, 1, projection="3d")
    ax.plot(*state[:, :3].T, color="#1d3557", lw=2, label="Recorded demonstration")
    for j, sample in enumerate(samples[::3]):
        p = np.vstack([state[sample["frame"], :3], sample["predicted_positions"]])
        ax.plot(*p.T, color="#e76f51", alpha=.7, lw=1.1,
                label="Independent 10-step prediction" if j == 0 else None)
    ax.set(xlabel="Base X (m)", ylabel="Base Y (m)", zlabel="Base Z (m)", title="Short forecasts anchored to recorded states")
    ax.legend(fontsize=8)
    ax = fig.add_subplot(2, 2, 2)
    t = np.array([s["frame"] for s in samples]) / fps
    for key, label, color in (("position_mm", "Model endpoint error", "#e76f51"),
            ("hold_position_mm", "No-motion baseline", "#777777"),
            ("constant_velocity_mm", "Last-velocity baseline", "#2a9d8f")):
        ax.plot(t, [s["metrics"][-1][key] for s in samples], label=label, color=color)
    ax.set(xlabel="Observation time (s; pauses compacted)", ylabel="Error at +0.333 s (mm)")
    ax.legend(fontsize=8)
    ax = fig.add_subplot(2, 2, 3)
    for k, color in enumerate(("#457b9d", "#2a9d8f", "#e76f51")):
        future = [state[s["frame"]+10, k] for s in samples]
        predicted = [s["predicted_positions"][-1][k] for s in samples]
        ax.plot(t, future, color=color, label="GT " + "XYZ"[k])
        ax.plot(t, predicted, "--", color=color, label="Pred " + "XYZ"[k])
    ax.set(xlabel="Observation time (s)", ylabel="Endpoint at +0.333 s (m)")
    ax.legend(ncol=3, fontsize=8)
    ax = fig.add_subplot(2, 2, 4)
    ax.step(t, [state[s["frame"]+10, 9] for s in samples], where="mid", label="GT future command", color="#1d3557")
    ax.plot(t, [s["prediction"][-1][6] for s in samples], label="Model future command (raw)", color="#e76f51")
    ax.axhline(.5, color="gray", ls=":", lw=1)
    ax.set(xlabel="Observation time (s)", ylabel="Gripper: 1=open, 0=closed", ylim=(-.15, 1.15))
    ax.legend(fontsize=8)
    fig.suptitle(name + " — training-set replay, NOT a closed-loop success test", fontsize=16)
    fig.savefig(out / (name + "_trajectory.png"), dpi=150)
    plt.close(fig)
    switches = np.flatnonzero(np.diff(state[:, 9]) != 0) + 1
    selected = sorted(set([0, len(state)//2, len(state)-1] + switches.tolist()))
    fig, axes = plt.subplots(2, len(selected), figsize=(3*len(selected), 6), squeeze=False, layout="constrained")
    for col, frame in enumerate(selected):
        for row, images in enumerate((frames, wrist)):
            axes[row, col].imshow(images[frame])
            axes[row, col].axis("off")
        axes[0, col].set_title("t={:.2f}s | {}".format(frame/fps, "OPEN" if state[frame, 9] else "CLOSED"))
    fig.suptitle(name + " | external camera (top), wrist camera (bottom)")
    fig.savefig(out / (name + "_observations.png"), dpi=120)
    plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", type=Path, required=True)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--episodes", default="0,19,39")
    parser.add_argument("--stride", type=int, default=5)
    parser.add_argument("--host", default="100.71.83.59")
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    info = json.loads((args.dataset / "meta/info.json").read_text())
    conversion = json.loads((args.dataset / "conversion_summary.json").read_text())
    episodes = [json.loads(s) for s in (args.dataset / "meta/episodes.jsonl").read_text().splitlines()]
    fps = info["fps"]
    assert fps == 30 and args.stride > 0
    all_rows, results = [], []
    with socket.create_connection((args.host, 5559), timeout=10) as sock:
        send_frame(sock, encode_json({"kind": "health"}))
        health = decode_json(recv_frame(sock))
        assert health["ok"] and health["model"] == MODEL and health["strict_checkpoint"]
        for ep in map(int, args.episodes.split(",")):
            source = conversion["episodes"][ep]
            name = source["source_episode"]
            path = args.dataset / info["data_path"].format(episode_chunk=ep//1000, episode_index=ep)
            data = pq.read_table(path).to_pydict()
            state, labels = np.array(data["state"], float), np.array(data["actions"], float)
            n = len(state)
            assert n == episodes[ep]["length"] and state.shape == (n, 10) and labels.shape == (n, 7)
            assert np.isfinite(state).all() and np.isfinite(labels).all()
            # Independently validate action decoding against subsequent states.
            checks_p, checks_r = [], []
            for i in range(n-1):
                p, r = integrate(state[i], labels[i:i+1])
                checks_p.append(float(np.linalg.norm(p[0] - state[i+1, :3])))
                checks_r.append(angle(r[0], state_rotation(state[i+1])))
            assert max(checks_p) < 2e-6 and max(checks_r) < .002
            assert np.array_equal(labels[:-1, 6], state[1:, 9])
            images = [video(args.dataset / info["video_path"].format(episode_chunk=ep//1000,
                episode_index=ep, video_key=key), n) for key in ("image", "wrist_image")]
            samples, rows = [], []
            prompt = episodes[ep]["tasks"][0]
            for i in range(0, n-10, args.stride):
                identity = "{}:{}".format(ep, i)
                send_frame(sock, encode_inference_request(dict(token="pnp7-local-demo", request_id=identity,
                    state=state[i].tolist(), prompt=prompt), images[0][i], images[1][i]))
                response = decode_json(recv_frame(sock))
                if not response.get("ok") or response.get("request_id") != identity:
                    raise RuntimeError(str(response))
                prediction = np.array(response["actions"], dtype=float)
                assert prediction.shape == (10, 7) and np.isfinite(prediction).all()
                positions, rotations = integrate(state[i], prediction)
                # Baseline uses ONLY a previous recorded velocity, no future input.
                previous_delta = state[i, :3] - state[i-1, :3] if i else np.zeros(3)
                metrics = []
                for h in (1, 5, 10):
                    truth = state[i+h]
                    gt_delta, pred_delta = truth[:3] - state[i, :3], positions[h-1] - state[i, :3]
                    ng, npred = np.linalg.norm(gt_delta), np.linalg.norm(pred_delta)
                    cosine = float(np.dot(gt_delta, pred_delta) / (ng*npred)) if ng >= .005 and npred > 1e-9 else None
                    row = dict(episode=ep, frame=i, horizon=h,
                        position_mm=float(1000*np.linalg.norm(positions[h-1] - truth[:3])),
                        rotation_deg=float(angle(rotations[h-1], state_rotation(truth))),
                        hold_position_mm=float(1000*ng),
                        constant_velocity_mm=float(1000*np.linalg.norm(state[i, :3] + h*previous_delta - truth[:3])),
                        grip_correct=bool((prediction[h-1, 6] >= .5) == (truth[9] >= .5)),
                        hold_grip_correct=bool(state[i, 9] == truth[9]), direction_cosine=cosine)
                    metrics.append(row)
                rows.extend(metrics)
                samples.append(dict(frame=i, prediction=prediction.tolist(), predicted_positions=positions.tolist(),
                    metrics=metrics, inference_ms=response["inference_ms"]))
                if len(samples) % 25 == 0:
                    print("PROGRESS {} {} windows".format(name, len(samples)), flush=True)
            result = dict(source=name, dataset_episode=ep, frames=n, windows=len(samples), prompt=prompt,
                source_metadata=source, parquet_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                decoder_max_position_error_m=max(checks_p), decoder_max_rotation_error_deg=max(checks_r),
                summary=summarize(rows))
            (args.out / (name + "_predictions.json")).write_text(json.dumps(dict(**result, samples=samples), allow_nan=False))
            plot_episode(args.out, name, state, samples, *images, fps)
            results.append(result)
            all_rows.extend(rows)
            print("EPISODE_DONE " + json.dumps(result), flush=True)
    report = dict(model=health, dataset=str(args.dataset), training_split=info["splits"],
        evaluation_type="teacher-forced training-set replay; no robot motion; not held-out or closed-loop evaluation",
        stride=args.stride, fps=fps, horizons=[1, 5, 10], position_unit="mm", rotation_unit="degree",
        gripper_metric="raw output >= 0.5; NOT deployed 0.25/0.75 threshold plus debounce",
        summary=summarize(all_rows), episodes=results, total_windows=sum(r["windows"] for r in results))
    (args.out / "summary.json").write_text(json.dumps(report, indent=2, allow_nan=False))
    body = """<!doctype html><meta charset="utf-8"><title>PI0.5 offline trajectory check</title>
    <style>body{font:16px system-ui;max-width:1300px;margin:35px auto;padding:20px;color:#203047}
    img{width:100%;border:1px solid #ddd;margin-bottom:24px}table{border-collapse:collapse}td,th{padding:10px;border:1px solid #ddd}
    .note{background:#fff4d6;padding:18px;border-radius:8px}</style>
    <h1>PI0.5：离线短时轨迹对照</h1>
    <p class="note">训练集回放，不是独立验证或真机成功率测试。每次使用真实当前观测预测未来 10 步（0.333 秒），
    不拼接成长程闭环轨迹。暂停保持在同一 episode，时间轴去掉暂停时间。没有连接机械臂或发送夹爪指令。</p>
    <p>位置误差是三维欧氏距离；旋转误差是相对旋转角。虚线/橙线为模型，实线为示教。
    夹爪统计采用 0.5 阈值，不等同于真机去抖后的触发时间。重叠窗口不是独立样本。</p>
    <table><tr><th>预测步数</th><th>位置均值 / P95 (mm)</th><th>不动基线 (mm)</th><th>旋转均值 (°)</th><th>夹爪一致率</th></tr>"""
    for h, m in report["summary"].items():
        body += "<tr><td>{}</td><td>{:.2f} / {:.2f}</td><td>{:.2f}</td><td>{:.2f}</td><td>{:.1%}</td></tr>".format(
            h, m["position_mm"]["mean"], m["position_mm"]["p95"], m["hold_position_mm"]["mean"],
            m["rotation_deg"]["mean"], m["gripper_accuracy"])
    body += "</table>"
    for result in results:
        name = result["source"]
        body += '<h2>{} · {} 个窗口</h2><img src="{}_trajectory.png"><img src="{}_observations.png">'.format(
            name, result["windows"], name, name)
    body += '<p><a href="summary.json">完整指标与模型信息</a></p>'
    (args.out / "report.html").write_text(body)
    print("EVALUATION_COMPLETE windows={}".format(report["total_windows"]), flush=True)


if __name__ == "__main__":
    main()
