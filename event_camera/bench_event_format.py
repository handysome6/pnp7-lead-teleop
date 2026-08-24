"""对比事件存储格式的体积，并量化热像素过滤的收益。

需要实测而非估算：EVT2 是固定 8 字节/事件，HDF5 可能带压缩，两者相差可能
数倍，直接决定 100 个 episode 是 30 GB 还是 120 GB。
"""
from __future__ import annotations

import argparse
import os
import time

import numpy as np
from metavision_core.event_io import EventsIterator
from metavision_sdk_stream import HDF5EventFileWriter, RAWEvt2EventFileWriter


def load_hot(path):
    if not path or not os.path.exists(path):
        return set()
    out = set()
    for line in open(path):
        p = line.split()
        if len(p) >= 2:
            out.add((int(p[0]), int(p[1])))
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=8.0)
    ap.add_argument("--hot", default="/work/hot_pixels.txt")
    ap.add_argument("--outdir", default="/work/fmt_test")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    hot = load_hot(args.hot)
    print(f"热像素屏蔽列表: {sorted(hot) if hot else '无'}\n")

    it = EventsIterator(input_path="", delta_t=20000)
    h, w = it.get_size()

    raw_p = os.path.join(args.outdir, "test.raw")
    h5_p = os.path.join(args.outdir, "test.hdf5")
    for p in (raw_p, h5_p):
        if os.path.exists(p):
            os.remove(p)

    meta = {"source": "EVK4", "note": "format benchmark"}
    wr_raw = RAWEvt2EventFileWriter(w, h, raw_p, False, meta)
    wr_h5 = HDF5EventFileWriter(h5_p, meta)

    kept = dropped = 0
    t0 = time.monotonic_ns()
    for evs in it:
        if len(evs):
            if hot:
                mask = np.ones(len(evs), dtype=bool)
                for (hx, hy) in hot:
                    mask &= ~((evs["x"] == hx) & (evs["y"] == hy))
                dropped += int((~mask).sum())
                evs = evs[mask]
            if len(evs):
                wr_raw.add_cd_events(evs)
                wr_h5.add_cd_events(evs)
                kept += len(evs)
        if (time.monotonic_ns() - t0) / 1e9 >= args.seconds:
            break
    dur = (time.monotonic_ns() - t0) / 1e9

    wr_raw.flush(); wr_raw.close()
    wr_h5.flush(); wr_h5.close()

    total = kept + dropped
    print(f"时长 {dur:.1f}s   原始事件 {total:,}")
    print(f"热像素滤除 {dropped:,} ({dropped/max(total,1)*100:.1f}%)")
    print(f"写入事件 {kept:,}  →  {kept/dur/1e6:.2f} Mev/s\n")

    print(f"{'格式':<10}{'体积':>12}{'字节/事件':>12}{'60秒估算':>14}{'100个episode':>16}")
    for name, p in (("RAW EVT2", raw_p), ("HDF5", h5_p)):
        sz = os.path.getsize(p)
        bpe = sz / max(kept, 1)
        mb60 = kept / dur * 60 * bpe / 1e6
        print(f"{name:<10}{sz/1e6:>10.1f} MB{bpe:>11.2f}"
              f"{mb60:>12.0f} MB{mb60*100/1000:>14.1f} GB")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
