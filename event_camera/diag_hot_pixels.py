"""检查事件是否集中在少数“热像素”上。

热像素是事件相机的常见传感器缺陷：个别像素持续自激输出，速率远高于场景
信号。它们会主导事件率统计，但在累积图上只有几个点，肉眼几乎看不见 ——
这正是本例中网格统计与累积图不一致的原因。
"""
from __future__ import annotations

import argparse
import time

import numpy as np
from metavision_core.event_io import EventsIterator


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--out", default="/work/hot_pixels.txt")
    args = ap.parse_args()

    it = EventsIterator(input_path="", delta_t=20000)
    h, w = it.get_size()
    accum = np.zeros((h, w), dtype=np.int64)

    t0 = time.monotonic_ns()
    total = 0
    for evs in it:
        if len(evs):
            np.add.at(accum, (evs["y"], evs["x"]), 1)
            total += len(evs)
        if (time.monotonic_ns() - t0) / 1e9 >= args.seconds:
            break
    dur = (time.monotonic_ns() - t0) / 1e9

    flat = accum.ravel()
    order = np.argsort(flat)[::-1]
    npix = w * h

    print(f"总事件 {total:,}，时长 {dur:.1f}s，像素 {npix:,}\n")
    print(f"{'最热像素数':<12}{'事件占比':>10}{'累计占比':>10}")
    cum = 0
    for k in (1, 2, 5, 10, 50, 100, 1000):
        cum_k = flat[order[:k]].sum()
        print(f"{k:<12}{flat[order[k-1]]/total*100:>9.2f}%{cum_k/total*100:>9.1f}%")

    print(f"\n{'像素中位事件数':<16}{np.median(flat):.0f}")
    print(f"{'像素均值':<16}{flat.mean():.1f}")

    print("\n最热的 10 个像素:")
    for i in order[:10]:
        y, x = divmod(int(i), w)
        rate = flat[i] / dur / 1e3
        print(f"   ({x:4d},{y:4d})  {flat[i]:>10,} 事件   "
              f"{rate:8.1f} kev/s   占总量 {flat[i]/total*100:5.2f}%")

    # 定义热像素：速率超过中位像素 1000 倍
    med = max(np.median(flat), 1)
    hot = np.where(flat > med * 1000)[0]
    hot_share = flat[hot].sum() / total if len(hot) else 0.0
    print(f"\n速率超过中位 1000 倍的像素: {len(hot)} 个 "
          f"({len(hot)/npix*100:.4f}% 的像素)")
    print(f"它们贡献了 {hot_share*100:.1f}% 的事件")

    if len(hot):
        with open(args.out, "w") as fh:
            for i in hot:
                y, x = divmod(int(i), w)
                fh.write(f"{x} {y} {flat[i]}\n")
        print(f"坐标已写入 {args.out}")

    print()
    if hot_share > 0.3:
        print("判定：热像素主导。事件率统计被少数缺陷像素拉高，")
        print("      真实场景信号远低于测得的总事件率。")
        print(f"      屏蔽这 {len(hot)} 个像素后，事件率约降至 "
              f"{total*(1-hot_share)/dur/1e6:.2f} Mev/s。")
    else:
        print("判定：未见热像素主导，高事件率来自场景本身。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
