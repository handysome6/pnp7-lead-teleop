"""独占接管 dead-man 按钮，让桌面看不到它的按键。

背景：换上来的按钮是一个无名 HID 键盘（0483:5750），按下时会发出普通键码。
桌面（GNOME/X11）对那个键有自己的绑定，于是每按一次就弹一个置顶窗口。
teleop 运行时靠 EVIOCGRAB 独占设备解决；但不跑 teleop 的时候按钮仍然归桌面。

这个脚本做两件事：
  1. EVIOCGRAB 独占设备，按键不再流向桌面；
  2. 打印按下/抬起，用来确认接管确实生效、以及键码是否配对。

内核在 fd 关闭时自动解除独占（进程崩溃也一样），所以不存在把按钮永久
锁死的情况。

    python3 grab_button.py                       # 用 /dev/pnp7_deadman
    python3 grab_button.py --device /dev/input/event6 --key KEY_V
    python3 grab_button.py --no-grab             # 只看事件，不接管
"""
from __future__ import annotations

import argparse
import fcntl
import os
import re
import select
import signal
import struct
import sys
import time

EV_FMT = "llHHi"
EV_SIZE = struct.calcsize(EV_FMT)
EV_KEY = 0x01

# _IOW('E', 0x90, int)
EVIOCGRAB = (1 << 30) | (4 << 16) | (ord("E") << 8) | 0x90

HEADER = "/usr/include/linux/input-event-codes.h"


def key_table() -> dict[str, int]:
    """名字 -> 键码，直接来自内核头文件。"""
    out: dict[str, int] = {}
    try:
        text = open(HEADER).read()
    except OSError:
        return out
    raw = dict(re.findall(r"^#define\s+((?:KEY|BTN)_\w+)\s+(\S+)", text, re.M))
    for name, val in raw.items():
        seen = set()
        while val in raw and val not in seen:
            seen.add(val)
            val = raw[val]
        try:
            out[name] = int(val, 0)
        except ValueError:
            pass
    return out


def _ioc_read(nr: int, size: int) -> int:
    return (2 << 30) | (size << 16) | (ord("E") << 8) | nr


def dev_name(fd: int) -> str:
    buf = bytearray(256)
    try:
        fcntl.ioctl(fd, _ioc_read(0x06, len(buf)), buf)
    except OSError:
        return "?"
    return buf.split(b"\x00", 1)[0].decode(errors="replace")


def main() -> int:
    names = key_table()
    codes = {v: k for k, v in names.items()}

    ap = argparse.ArgumentParser()
    ap.add_argument("--device", default="/dev/pnp7_deadman")
    ap.add_argument("--key", default="",
                    help="只关心这个键（名字或数字）。留空则显示所有键。")
    ap.add_argument("--no-grab", action="store_true",
                    help="不独占，仅旁观。用来对比接管前后的行为。")
    ap.add_argument("--seconds", type=float, default=0.0,
                    help="0 表示一直运行到 Ctrl-C")
    args = ap.parse_args()

    want = None
    if args.key:
        want = names.get(args.key)
        if want is None:
            try:
                want = int(args.key, 0)
            except ValueError:
                print(f"未知键名 {args.key}", file=sys.stderr)
                return 2

    try:
        fd = os.open(args.device, os.O_RDONLY)
    except OSError as exc:
        print(f"打不开 {args.device}: {exc}", file=sys.stderr)
        return 1

    print(f"设备 {args.device} -> {os.path.realpath(args.device)}")
    print(f"名称 {dev_name(fd)}")

    if not args.no_grab:
        try:
            fcntl.ioctl(fd, EVIOCGRAB, 1)
        except OSError as exc:
            print(f"独占失败: {exc}（多半是已经有别的进程占着）", file=sys.stderr)
            os.close(fd)
            return 1
        print("已独占（EVIOCGRAB）——现在按下按钮，桌面应该毫无反应")
    else:
        print("未独占，桌面仍会收到按键")
    print("Ctrl-C 退出\n")

    stop = False

    def on_sig(_sig, _frm):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    t_end = time.monotonic() + args.seconds if args.seconds > 0 else None
    presses = 0
    down_at = None
    while not stop:
        if t_end is not None and time.monotonic() >= t_end:
            break
        ready, _, _ = select.select([fd], [], [], 0.5)
        if not ready:
            continue
        try:
            data = os.read(fd, EV_SIZE * 64)
        except OSError as exc:
            # ENODEV once the device is unplugged. The whole point of this tool
            # is to survive replugs, so say so and stop rather than dumping a
            # traceback -- the fd is dead and select() would spin on it forever.
            print(f"\n设备消失了({exc.strerror}) —— 大概是拔掉了。"
                  f"重新插上后再运行一次。", file=sys.stderr)
            break
        for off in range(0, len(data) - EV_SIZE + 1, EV_SIZE):
            _, _, etype, code, value = struct.unpack_from(EV_FMT, data, off)
            if etype != EV_KEY:
                continue
            if want is not None and code != want:
                continue
            label = codes.get(code, f"code={code}")
            if value == 1:
                presses += 1
                down_at = time.monotonic()
                print(f"  [{presses:3d}] {label} 按下")
            elif value == 0:
                held = (time.monotonic() - down_at) * 1000 if down_at else 0.0
                down_at = None
                print(f"        {label} 抬起  (按住 {held:.0f} ms)")
            # value == 2 是自动重复，键盘按住会一直发，不必刷屏

    if not args.no_grab:
        try:
            fcntl.ioctl(fd, EVIOCGRAB, 0)
        except OSError:
            pass
    os.close(fd)
    print(f"\n共 {presses} 次按下，已释放设备")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
