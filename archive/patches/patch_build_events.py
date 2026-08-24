"""让 build_episode.py 把事件流索引进 episode.csv。

事件是连续流而非帧，所以不能像相机那样"匹配最近的一帧"。正确做法是给每个
锚点帧标出它所覆盖的事件时间窗口，训练时按需切片。窗口用事件时钟表示，由
events_meta.json 里的 clock_map 反变换得到。
"""
p = "/home/franka/workspace/pnp7_teleop/build_episode.py"
s = open(p).read()

s = s.replace(
    '''    ap.add_argument("--keep-idle", action="store_true",''',
    '''    ap.add_argument("--no-events", action="store_true",
                    help="即使存在 events.hdf5 也不索引事件流")
    ap.add_argument("--keep-idle", action="store_true",''',
)

old = '''    out_rows = []
    skew = {k: [] for k in cams if k != args.anchor}'''
new = '''    # 事件流：读取时钟映射，用于把宿主 monotonic 时间反算成事件时钟
    ev_meta = None
    ev_path = ep / "events.hdf5"
    if not args.no_events and ev_path.exists():
        try:
            ev_meta = json.load(open(ep / "events_meta.json"))
        except OSError:
            ev_meta = None

    def to_event_us(mono_ns):
        """宿主 CLOCK_MONOTONIC 纳秒 -> 事件时钟微秒。"""
        cm = ev_meta["clock_map"]
        if cm["slope"] is None:
            return None
        return ((mono_ns - ev_meta["t0_monotonic_ns"]) / 1e3
                - cm["intercept_us"]) / cm["slope"] + ev_meta["event_t_first_us"]

    out_rows = []
    skew = {k: [] for k in cams if k != args.anchor}'''
assert old in s
s = s.replace(old, new, 1)

old2 = '''        for j in range(NJ):
            row[f"q_robot{j}"] = rec[f"q_robot{j}"]'''
new2 = '''        if ev_meta is not None:
            # 本帧覆盖的事件窗口 = 上一锚点帧到本帧
            prev_t = anchor["ts"][a_i - 1] if a_i > 0 else t
            e0 = to_event_us(prev_t)
            e1 = to_event_us(t)
            row["event_t_start_us"] = round(e0, 1) if e0 is not None else ""
            row["event_t_end_us"] = round(e1, 1) if e1 is not None else ""

        for j in range(NJ):
            row[f"q_robot{j}"] = rec[f"q_robot{j}"]'''
assert old2 in s
s = s.replace(old2, new2, 1)

old3 = '''    meta = {
        "frames": len(out_rows),'''
new3 = '''    meta = {
        "events": {
            "file": "events.hdf5",
            "written": ev_meta["events_written"],
            "rate_mev_s": ev_meta["event_rate_mev_s"],
            "hot_fraction": ev_meta["hot_fraction"],
            "drift_ppm": ev_meta["clock_map"]["drift_ppm"],
            "max_residual_ms": ev_meta["clock_map"]["max_residual_ms"],
            "note": "event_t_start_us / event_t_end_us 为每帧覆盖的事件窗口，"
                    "使用事件时钟；读取 hdf5 需要 libH5Zecf.so 插件",
        } if ev_meta is not None else None,
        "frames": len(out_rows),'''
assert old3 in s
s = s.replace(old3, new3, 1)

old4 = '''    print(f"episode: {len(out_rows)} frames over {dur:.2f}s '''
new4 = '''    if ev_meta is not None:
        print(f"events: {ev_meta['events_written']:,} 个 "
              f"({ev_meta['event_rate_mev_s']:.2f} Mev/s), "
              f"热像素滤除 {ev_meta['hot_fraction']*100:.0f}%, "
              f"漂移 {ev_meta['clock_map']['drift_ppm']:+.1f} ppm")
    elif not args.no_events:
        print("events: 无（本 episode 只有 RGB）")

    print(f"episode: {len(out_rows)} frames over {dur:.2f}s '''
assert old4 in s
s = s.replace(old4, new4, 1)

open(p, "w").write(s)
print("build_episode.py 已接入事件流索引")
