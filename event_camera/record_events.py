"""录制 EVK4 事件流，作为一个 episode 的第三路观测。

在 Metavision 容器内运行（OpenEB 5.3 需要 Ubuntu 22.04，而采集机是 20.04）。
容器与宿主共享内核，CLOCK_MONOTONIC 是同一时钟源，因此这里记录的时间戳与
宿主上 RealSense、机器人状态的时间戳直接可比。

输出:
  events.hdf5       事件流，ECF 压缩。读取需要 h5py 加 48 KB 的 libH5Zecf.so,
                    不需要整个 Metavision SDK。
  events_meta.json  时钟映射、热像素、统计量

时钟映射
--------
事件时间戳来自相机硬件，起点是流开始时刻，单位微秒。实测走时比与 1 的偏差
在 -3.5 到 +35.7 ppm 之间浮动 —— 不是固定的晶振误差，所以不能把某次测得的
斜率写死。这里每个 episode 独立拟合，把 (slope, intercept) 写进元数据:

    monotonic_ns ≈ (event_t_us * slope + intercept) * 1000 + t0_ns

即便按 ±50 ppm 的保守上限，60 秒也只偏 3 ms，远小于一个 30 fps 帧间隔。

热像素
------
EVK4 上实测有 2 个缺陷像素持续自激，贡献了 74%-91% 的事件量，却只占画面
0.0002%。不滤掉的话存储会浪费一个数量级。位置随温度变化，所以由调用方
传入当次检测结果，而不是写死在代码里。
"""
from __future__ import annotations

import argparse
import json
import os
import signal
import sys
import time

import numpy as np

g_stop = False


def _on_sig(_s, _f):
    global g_stop
    g_stop = True


def load_hot_pixels(path: str) -> np.ndarray:
    """读取热像素坐标，返回 (N,2) 数组。"""
    if not path or not os.path.exists(path):
        return np.zeros((0, 2), dtype=np.int64)
    rows = []
    for line in open(path):
        parts = line.split()
        if len(parts) >= 2:
            rows.append((int(parts[0]), int(parts[1])))
    return np.array(rows, dtype=np.int64) if rows else np.zeros((0, 2), np.int64)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, help="episode 目录")
    ap.add_argument("--duration", type=float, default=60.0)
    ap.add_argument("--hot-pixels", default="", help="热像素坐标文件")
    ap.add_argument("--delta-t", type=int, default=20000, help="批窗口(微秒)")
    args = ap.parse_args()

    from metavision_core.event_io import EventsIterator
    from metavision_sdk_stream import HDF5EventFileWriter

    signal.signal(signal.SIGINT, _on_sig)
    signal.signal(signal.SIGTERM, _on_sig)

    os.makedirs(args.out, exist_ok=True)
    h5_path = os.path.join(args.out, "events.hdf5")
    meta_path = os.path.join(args.out, "events_meta.json")
    if os.path.exists(h5_path):
        os.remove(h5_path)

    hot = load_hot_pixels(args.hot_pixels)
    it = EventsIterator(input_path="", delta_t=args.delta_t)
    h, w = it.get_size()

    # 把 (x,y) 编码成单个整数，这样热像素数量增加时过滤仍是一次 isin
    hot_codes = (hot[:, 0].astype(np.int64) * h + hot[:, 1]) if len(hot) else None

    writer = HDF5EventFileWriter(h5_path, {
        "source": "Prophesee EVK4",
        "width": str(w),
        "height": str(h),
        "hot_pixels_masked": str(len(hot)),
    })

    t0_ns = time.monotonic_ns()
    kept = dropped = batches = 0
    ev_t_first = None
    samples: list[tuple[int, int]] = []

    print("EVENTS_READY", flush=True)

    for evs in it:
        now = time.monotonic_ns()
        batches += 1
        if len(evs):
            if ev_t_first is None:
                ev_t_first = int(evs["t"][0])
            # 采样一对时钟点用于事后拟合
            samples.append((now - t0_ns, int(evs["t"][-1]) - ev_t_first))

            if hot_codes is not None and len(hot_codes):
                codes = evs["x"].astype(np.int64) * h + evs["y"]
                mask = ~np.isin(codes, hot_codes)
                dropped += int((~mask).sum())
                evs = evs[mask]
            if len(evs):
                writer.add_cd_events(evs)
                kept += len(evs)

        if g_stop or (now - t0_ns) / 1e9 >= args.duration:
            break

    writer.flush()
    writer.close()
    t_end_ns = time.monotonic_ns()
    dur = (t_end_ns - t0_ns) / 1e9

    # 事件时钟 -> 宿主 CLOCK_MONOTONIC 的线性映射
    slope = intercept = None
    resid_ms = None
    if len(samples) > 10:
        wall_us = np.array([s[0] for s in samples], dtype=np.float64) / 1e3
        evt_us = np.array([s[1] for s in samples], dtype=np.float64)
        slope, intercept = (float(v) for v in np.polyfit(evt_us, wall_us, 1))
        resid = wall_us - (slope * evt_us + intercept)
        resid_ms = float(np.abs(resid).max() / 1000.0)

    total = kept + dropped
    meta = {
        "t0_monotonic_ns": t0_ns,
        "t_end_monotonic_ns": t_end_ns,
        "duration_s": round(dur, 4),
        "width": w,
        "height": h,
        "events_written": kept,
        "events_dropped_hot": dropped,
        "hot_fraction": round(dropped / total, 4) if total else 0.0,
        "event_rate_mev_s": round(kept / dur / 1e6, 4) if dur > 0 else 0.0,
        "hot_pixels": hot.tolist(),
        "event_t_first_us": ev_t_first,
        "clock_map": {
            "note": "monotonic_ns ≈ (event_t_us - event_t_first_us) * slope "
                    "* 1000 + intercept * 1000 + t0_monotonic_ns",
            "slope": slope,
            "intercept_us": intercept,
            "drift_ppm": round((slope - 1.0) * 1e6, 2) if slope else None,
            "max_residual_ms": round(resid_ms, 4) if resid_ms is not None else None,
        },
    }
    with open(meta_path, "w") as fh:
        json.dump(meta, fh, indent=2)

    size_mb = os.path.getsize(h5_path) / 1e6 if os.path.exists(h5_path) else 0
    print(f"  事件写入 {kept:,}  滤除热像素 {dropped:,} "
          f"({meta['hot_fraction']*100:.0f}%)")
    print(f"  {meta['event_rate_mev_s']:.2f} Mev/s   {size_mb:.1f} MB   "
          f"{dur:.1f}s")
    if slope:
        print(f"  时钟漂移 {meta['clock_map']['drift_ppm']:+.1f} ppm   "
              f"残差最大 {resid_ms:.3f} ms")
    return 0


if __name__ == "__main__":
    sys.exit(main())
