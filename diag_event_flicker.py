"""诊断事件流里的持续高事件率是不是光源闪烁。

判据有两个：
  时域 —— 市电 50 Hz 供电的灯，亮度以 100 Hz 波动，事件计数序列会在 100 Hz
          (以及 200/300 Hz 谐波) 出现明显谱峰。真实的手部运动是宽带的，没有
          这种尖锐的线谱。
  空域 —— 闪烁照亮整个视野，事件在画面上近似均匀铺开；运动只在物体边缘附近
          产生事件，空间上高度集中。
"""
from __future__ import annotations

import argparse
import time

import numpy as np
from metavision_core.event_io import EventsIterator


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--bin-us", type=int, default=500, help="计数分箱宽度(微秒)")
    args = ap.parse_args()

    it = EventsIterator(input_path="", delta_t=args.bin_us)
    h, w = it.get_size()

    counts = []
    xs, ys = [], []
    t0 = time.monotonic_ns()
    for evs in it:
        counts.append(len(evs))
        if len(evs) and len(xs) < 60:
            xs.append(evs["x"][::37].copy())
            ys.append(evs["y"][::37].copy())
        if (time.monotonic_ns() - t0) / 1e9 >= args.seconds:
            break

    c = np.array(counts, dtype=np.float64)
    c = c - c.mean()
    fs = 1e6 / args.bin_us
    spec = np.abs(np.fft.rfft(c * np.hanning(len(c))))
    freqs = np.fft.rfftfreq(len(c), 1.0 / fs)

    band = (freqs > 5) & (freqs < 500)
    peak_f = freqs[band][np.argmax(spec[band])]
    peak_p = spec[band].max()
    median_p = np.median(spec[band])
    ratio = peak_p / median_p

    print(f"分箱 {args.bin_us} us，采样率 {fs:.0f} Hz，共 {len(c)} 个分箱\n")
    print(f"{'主峰频率':<14} {peak_f:.1f} Hz")
    print(f"{'主峰/中位':<14} {ratio:.1f}x")
    print("\n5-500 Hz 频段前 6 个谱峰:")
    idx = np.argsort(spec[band])[::-1][:6]
    for i in sorted(idx):
        print(f"   {freqs[band][i]:7.1f} Hz   强度 {spec[band][i]/median_p:6.1f}x 中位")

    print()
    if len(xs):
        X = np.concatenate(xs)
        Y = np.concatenate(ys)
        hist, _, _ = np.histogram2d(X, Y, bins=[16, 16],
                                    range=[[0, w], [0, h]])
        frac = hist / hist.sum()
        occupancy = float((frac > 0.001).mean())
        gini_top = float(np.sort(frac.ravel())[::-1][:26].sum())
        print(f"{'空间占用率':<14} {occupancy*100:.0f}% 的网格有事件")
        print(f"{'最热 10% 网格':<14} 占全部事件的 {gini_top*100:.0f}%")

    print()
    flicker = ratio > 8 and (
        abs(peak_f - 100) < 12 or abs(peak_f - 120) < 12
        or abs(peak_f - 200) < 15 or abs(peak_f - 50) < 8)
    if flicker:
        print(f"判定：光源闪烁。{peak_f:.0f} Hz 处有强线谱，与市电供电的灯具吻合。")
        print("      这些事件是照明噪声，不是任务信号。")
    else:
        print("判定：未见明显闪烁线谱，高事件率可能来自真实运动或传感器噪声。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
