"""把事件相机检查加进 check_ready.py。

包含热像素重新检测：坏点位置会随温度和时间变化，把某次测得的坐标写死会
在几周后失效，届时事件率会悄悄涨一个数量级而没人发现。
"""
p = "/home/franka/workspace/pnp7_teleop/check_ready.py"
s = open(p).read()

s = s.replace(
    '''def check_franka(rep: Report, ip: str) -> None:''',
    '''def check_event_camera(rep: Report, image: str, hot_path: str,
                       hot_seconds: float) -> None:
    """检查 EVK4 是否可见，并可选地重测热像素。

    EVK4 只能在容器内访问（OpenEB 5.3 需要 Ubuntu 22.04，本机是 20.04）。
    """
    import subprocess

    probe = (
        "python3 -c \\"from metavision_hal import DeviceDiscovery as D; "
        "l=list(D.list()); print('SERIAL', l[0] if l else 'NONE')\\""
    )
    try:
        r = subprocess.run(
            ["docker", "run", "--rm", "--privileged",
             "-v", "/dev/bus/usb:/dev/bus/usb", image, "bash", "-lc", probe],
            capture_output=True, text=True, timeout=90,
        )
    except Exception as exc:
        rep.add(WARN, "事件相机", f"无法运行容器: {exc}")
        return

    line = next((l for l in r.stdout.splitlines() if l.startswith("SERIAL")), "")
    if "NONE" in line or not line:
        rep.add(FAIL, "事件相机", "未发现 EVK4 —— 检查 USB 连接")
        return
    serial = line.split(None, 1)[1]
    rep.add(OK, "事件相机", f"EVK4 {serial}")

    if hot_seconds <= 0:
        if os.path.exists(hot_path):
            n = sum(1 for _ in open(hot_path))
            rep.add(OK, "热像素表", f"{n} 个坐标（未重测，用 --hot-seconds 触发）")
        else:
            rep.add(WARN, "热像素表", f"{hot_path} 不存在，事件率会偏高一个数量级")
        return

    hot_dir = os.path.dirname(hot_path) or "."
    cmd = (f"python3 diag_hot_pixels.py --seconds {hot_seconds} "
           f"--out /work/{os.path.basename(hot_path)}")
    try:
        r2 = subprocess.run(
            ["docker", "run", "--rm", "--privileged",
             "-v", "/dev/bus/usb:/dev/bus/usb",
             "-v", f"{hot_dir}:/work", "-w", "/work",
             image, "bash", "-lc", cmd],
            capture_output=True, text=True, timeout=180,
        )
    except Exception as exc:
        rep.add(WARN, "热像素检测", str(exc))
        return

    share = next((l for l in r2.stdout.splitlines()
                  if "它们贡献了" in l), "")
    count = next((l for l in r2.stdout.splitlines()
                  if "速率超过中位" in l), "")
    n_hot = 0
    if os.path.exists(hot_path):
        n_hot = sum(1 for _ in open(hot_path))
    detail = f"{n_hot} 个坏点"
    if share:
        detail += f"，{share.strip()}"
    rep.add(OK if n_hot < 50 else WARN, "热像素检测", detail)


def check_franka(rep: Report, ip: str) -> None:''',
)

s = s.replace(
    '''    ap.add_argument("--skip-cameras", action="store_true")''',
    '''    ap.add_argument("--skip-cameras", action="store_true")
    ap.add_argument("--skip-events", action="store_true")
    ap.add_argument("--event-image", default="metavision:5.3.0")
    ap.add_argument("--hot-pixels",
                    default=os.path.expanduser("~/metavision/hot_pixels.txt"))
    ap.add_argument("--hot-seconds", type=float, default=0.0,
                    help="重新检测热像素的时长；0 表示沿用现有表")''',
)

s = s.replace(
    '''    check_deadman(rep, args.deadman)''',
    '''    check_deadman(rep, args.deadman)
    if not args.skip_events:
        print("\\n=== 事件相机 ===")
        check_event_camera(rep, args.event_image, args.hot_pixels,
                           args.hot_seconds)''',
)

if "\nimport os\n" not in s:
    s = s.replace("import json\n", "import json\nimport os\n", 1)

open(p, "w").write(s)
print("check_ready.py 已加入事件相机检查")
