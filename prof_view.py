"""剖析事件预览各阶段耗时，定位卡顿来源。

嫌疑排序：
  1. np.add.at 是 unbuffered 散射累加，比等价的 bincount 慢一个数量级
  2. 热像素过滤——它们占 93% 事件，过滤发生在数组仍然很大的时候
  3. X11 图像传输：1280x720x3 每帧 2.7 MB
"""
from __future__ import annotations

import os
import time

import cv2
import numpy as np
from metavision_core.event_io import EventsIterator

HOT = []
if os.path.exists("/work/hot_pixels.txt"):
    for line in open("/work/hot_pixels.txt"):
        p = line.split()
        if len(p) >= 2:
            HOT.append((int(p[0]), int(p[1])))

it = EventsIterator(input_path="", delta_t=20000)
h, w = it.get_size()
surface = np.zeros((h, w), np.float32)
hot_codes = np.array([x * h + y for x, y in HOT], dtype=np.int64)

T = {k: 0.0 for k in ("read", "filter_loop", "filter_isin", "add_at",
                      "bincount", "decay", "colormap", "resize")}
N = 0
n_ev_total = 0
t_end = time.monotonic() + 8

for evs in it:
    t0 = time.perf_counter()
    N += 1
    n_ev_total += len(evs)
    T["read"] += time.perf_counter() - t0

    if not len(evs):
        continue
    x, y, p = evs["x"], evs["y"], evs["p"]

    # 方案 A：逐个热像素做全数组比较（当前实现）
    t0 = time.perf_counter()
    m = np.ones(len(evs), bool)
    for hx, hy in HOT:
        m &= ~((x == hx) & (y == hy))
    T["filter_loop"] += time.perf_counter() - t0

    # 方案 B：编码成整数后一次 isin
    t0 = time.perf_counter()
    codes = x.astype(np.int64) * h + y
    m2 = ~np.isin(codes, hot_codes)
    T["filter_isin"] += time.perf_counter() - t0

    xf, yf, pf = x[m], y[m], p[m]

    # 方案 A：np.add.at（当前实现）
    t0 = time.perf_counter()
    fp = np.zeros((h, w), np.float32)
    fn = np.zeros((h, w), np.float32)
    pos = pf == 1
    np.add.at(fp, (yf[pos], xf[pos]), 1.0)
    np.add.at(fn, (yf[~pos], xf[~pos]), 1.0)
    T["add_at"] += time.perf_counter() - t0

    # 方案 B：bincount
    t0 = time.perf_counter()
    idx = yf.astype(np.int64) * w + xf
    cnt = np.bincount(idx, minlength=h * w).astype(np.float32).reshape(h, w)
    T["bincount"] += time.perf_counter() - t0

    t0 = time.perf_counter()
    surface *= 0.9
    surface += fp + fn
    T["decay"] += time.perf_counter() - t0

    t0 = time.perf_counter()
    vis = np.clip(surface * 40, 0, 255).astype(np.uint8)
    img = cv2.applyColorMap(vis, cv2.COLORMAP_BONE)
    T["colormap"] += time.perf_counter() - t0

    t0 = time.perf_counter()
    small = cv2.resize(img, (w // 2, h // 2))
    T["resize"] += time.perf_counter() - t0

    if time.monotonic() > t_end:
        break

print(f"分辨率 {w}x{h}   批数 {N}   平均每批 {n_ev_total/max(N,1):,.0f} 个事件")
print(f"事件率 {n_ev_total/8/1e6:.2f} Mev/s   热像素 {len(HOT)} 个\n")
print(f"{'阶段':<16}{'总计(ms)':>12}{'每批(ms)':>12}")
for k, v in T.items():
    print(f"{k:<16}{v*1000:>12.1f}{v*1000/max(N,1):>12.2f}")

per_frame_now = (T["read"] + T["filter_loop"] + T["add_at"] + T["decay"]
                 + T["colormap"]) / max(N, 1) * 1000
per_frame_opt = (T["read"] + T["filter_isin"] + T["bincount"] + T["decay"]
                 + T["colormap"]) / max(N, 1) * 1000
print(f"\n当前实现每帧   {per_frame_now:6.2f} ms  →  上限 {1000/max(per_frame_now,1e-9):5.1f} fps")
print(f"优化后每帧     {per_frame_opt:6.2f} ms  →  上限 {1000/max(per_frame_opt,1e-9):5.1f} fps")
print(f"\nX11 每帧传输量 {w*h*3/1e6:.2f} MB（全分辨率）")
