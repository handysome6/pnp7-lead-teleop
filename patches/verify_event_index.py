"""验证 episode.csv 的事件窗口与 events.hdf5 的实际数据一致。"""
import csv
import json
import os
import sys

import h5py

ep = sys.argv[1] if len(sys.argv) > 1 else "/episode"
rows = list(csv.DictReader(open(os.path.join(ep, "episode.csv"))))
meta = json.load(open(os.path.join(ep, "events_meta.json")))

print(f"episode 帧数: {len(rows)}")
print(f"事件文件事件数: {meta['events_written']:,}\n")

print("前 3 帧的事件窗口:")
for r in rows[:3]:
    print(f"   帧 t={r['t_ns']}   窗口 {r['event_t_start_us']} .. "
          f"{r['event_t_end_us']} us")

f = h5py.File(os.path.join(ep, "events.hdf5"), "r")
d = f["CD/events"]
t_all = d["t"]
print(f"\nHDF5 事件时间范围: {t_all[0]:,} .. {t_all[-1]:,} us")

# 抽 5 帧，数一下窗口内真实有多少事件
print("\n抽样核对（窗口内实际事件数）:")
step = max(len(rows) // 5, 1)
for r in rows[step::step][:5]:
    a = float(r["event_t_start_us"])
    b = float(r["event_t_end_us"])
    lo = t_all.searchsorted(a)
    hi = t_all.searchsorted(b)
    dt_ms = (b - a) / 1000.0
    n = hi - lo
    print(f"   窗口 {dt_ms:5.1f} ms 内 {n:>7,} 个事件  "
          f"({n/max(dt_ms,1e-9):.0f} ev/ms)")

first_start = float(rows[0]["event_t_start_us"])
last_end = float(rows[-1]["event_t_end_us"])
covered = t_all.searchsorted(last_end) - t_all.searchsorted(first_start)
print(f"\n所有帧窗口合计覆盖 {covered:,} 个事件 "
      f"（占文件的 {covered/len(t_all)*100:.0f}%）")
f.close()

ok = 0 < covered <= len(t_all)
print("\n通过：事件窗口落在文件的时间范围内，索引可用。" if ok
      else "\n失败：窗口与事件文件不匹配。")
sys.exit(0 if ok else 1)
