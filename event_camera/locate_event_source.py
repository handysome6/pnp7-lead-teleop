"""定位事件热点在画面中的位置，并导出一张累积图便于肉眼确认。

用于找出持续产生事件的周期性光源。做法是把事件按 214 Hz 主峰的相位无关地
累积，热点即为光源所在。
"""
from __future__ import annotations

import argparse
import time

import numpy as np
from metavision_core.event_io import EventsIterator


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--out", default="/work/event_accum.png")
    args = ap.parse_args()

    it = EventsIterator(input_path="", delta_t=20000)
    h, w = it.get_size()
    accum = np.zeros((h, w), dtype=np.float64)

    t0 = time.monotonic_ns()
    total = 0
    for evs in it:
        if len(evs):
            np.add.at(accum, (evs["y"], evs["x"]), 1)
            total += len(evs)
        if (time.monotonic_ns() - t0) / 1e9 >= args.seconds:
            break

    print(f"累积 {total:,} 个事件，画面 {w} x {h}\n")

    # 16x9 粗网格，报告热点
    gh, gw = 9, 16
    grid = accum.reshape(gh, h // gh, gw, w // gw).sum(axis=(1, 3))
    frac = grid / grid.sum()

    print("事件密度分布（每格占比 %，行=上到下，列=左到右）:")
    for r in range(gh):
        print("   " + " ".join(f"{frac[r, c]*100:4.1f}" for c in range(gw)))

    flat = np.argsort(frac.ravel())[::-1][:5]
    print("\n最热的 5 个区域:")
    for i in flat:
        r, c = divmod(int(i), gw)
        x0, x1 = c * (w // gw), (c + 1) * (w // gw)
        y0, y1 = r * (h // gh), (r + 1) * (h // gh)
        pos_v = ["上", "上", "上", "中上", "中", "中下", "下", "下", "下"][r]
        pos_h = "左" if c < 5 else ("中" if c < 11 else "右")
        print(f"   {pos_v}{pos_h}  x[{x0}:{x1}] y[{y0}:{y1}]  "
              f"占 {frac[r, c]*100:.1f}%")

    try:
        import cv2
        img = np.log1p(accum)
        img = (255 * img / max(img.max(), 1e-9)).astype(np.uint8)
        img = cv2.applyColorMap(img, cv2.COLORMAP_INFERNO)
        cv2.imwrite(args.out, img)
        print(f"\n累积图已保存: {args.out}")
    except Exception as exc:
        print(f"\n(未能保存图片: {exc})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
