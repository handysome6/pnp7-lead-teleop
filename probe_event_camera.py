"""EVK4 事件相机基线测试：码率、时钟漂移、连接稳定性。

必须在正式采集前跑，因为这三件事决定了：
  1. 一个 60 秒 episode 会产生多少数据（决定存储策略）
  2. 事件时间戳能否与机器人 / RealSense 的 CLOCK_MONOTONIC 对齐
  3. 加上 EVK4 后 USB 总线是否还稳

时钟对齐是重点。事件时间戳来自相机硬件，起点为流开始时刻，单位微秒。要和
其余数据流对齐，就必须确认它相对 CLOCK_MONOTONIC 的走时比例为 1。60 秒里
千分之一的偏差就是 60 ms，约等于两个 RGB 帧，足以毁掉对齐。

  python probe_event_camera.py --seconds 30
"""
from __future__ import annotations

import argparse
import time

import numpy as np
from metavision_core.event_io import EventsIterator


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--delta-t", type=int, default=20000,
                    help="每批事件的时间窗口，微秒")
    args = ap.parse_args()

    print(f"打开相机，采集 {args.seconds:.0f} 秒 …")
    print("（请在相机视野内制造一些运动，静止场景事件率会接近 0）\n")

    it = EventsIterator(input_path="", delta_t=args.delta_t)
    h, w = it.get_size()
    print(f"分辨率: {w} x {h}\n")

    t_wall_start = time.monotonic_ns()
    ev_t_first = None
    ev_t_last = 0
    total = 0
    batches = 0
    per_batch = []
    samples = []          # (wall_ns, event_us) 用于漂移拟合

    for evs in it:
        now = time.monotonic_ns()
        n = len(evs)
        batches += 1
        if n:
            if ev_t_first is None:
                ev_t_first = int(evs["t"][0])
            ev_t_last = int(evs["t"][-1])
            total += n
            per_batch.append(n)
            samples.append((now - t_wall_start, ev_t_last - ev_t_first))
        if (now - t_wall_start) / 1e9 >= args.seconds:
            break

    wall_s = (time.monotonic_ns() - t_wall_start) / 1e9

    print(f"{'总事件数':<16} {total:,}")
    print(f"{'实际时长':<16} {wall_s:.2f} s")
    print(f"{'平均事件率':<16} {total / wall_s / 1e6:.3f} Mev/s")
    if per_batch:
        pb = np.array(per_batch)
        print(f"{'每批事件数':<16} 中位 {int(np.median(pb)):,}   "
              f"p95 {int(np.percentile(pb, 95)):,}   最大 {int(pb.max()):,}")
        peak_rate = pb.max() / (args.delta_t / 1e6) / 1e6
        print(f"{'峰值事件率':<16} {peak_rate:.3f} Mev/s")

    # 存储估算：EVT3 编码约每事件 2-3 字节
    for name, bpe in (("EVT3 (约2.5B/事件)", 2.5), ("原始 8B/事件", 8.0)):
        mb60 = total / wall_s * 60 * bpe / 1e6
        print(f"{'60秒 episode 估算':<16} {name}: {mb60:,.0f} MB"
              f"   100个: {mb60 * 100 / 1000:,.1f} GB")

    # 时钟漂移
    print()
    if len(samples) > 10:
        wall = np.array([s[0] for s in samples], dtype=np.float64) / 1e3  # us
        evt = np.array([s[1] for s in samples], dtype=np.float64)         # us
        slope, intercept = np.polyfit(wall, evt, 1)
        drift_ppm = (slope - 1.0) * 1e6
        drift_ms_60s = (slope - 1.0) * 60_000
        resid = evt - (slope * wall + intercept)
        print(f"{'时钟走时比':<16} {slope:.9f}  (理想 1.0)")
        print(f"{'漂移':<16} {drift_ppm:+.1f} ppm  →  60 秒累计 {drift_ms_60s:+.2f} ms")
        print(f"{'拟合残差':<16} 标准差 {resid.std() / 1000:.3f} ms  "
              f"最大 {np.abs(resid).max() / 1000:.3f} ms")
        print()
        if abs(drift_ms_60s) < 5:
            print("时钟：良好。60 秒内偏差远小于一个 30 fps 帧间隔(33 ms)，")
            print("      线性映射即可对齐，无需逐帧校正。")
        elif abs(drift_ms_60s) < 33:
            print("时钟：可用，但建议按本次拟合的斜率做线性校正。")
        else:
            print("时钟：偏差超过一帧，必须做漂移校正，不能只记录起始偏移。")
    else:
        print("样本太少，无法拟合时钟漂移 —— 场景可能过于静止。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
