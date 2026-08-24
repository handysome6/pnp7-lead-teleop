"""找出某个按钮对应的 /dev/input/eventN，以及它按下时发出的键码。

这台机器上的按钮是无名 HID 设备（0483:5750，USB 字符串描述符全空），
光看设备名认不出来，只能让人按一下、看谁在说话。

用法：
    python3 find_button.py            # 监听所有 event 设备 20 秒
    python3 find_button.py --seconds 30
    python3 find_button.py --only 6,9 # 只看指定的 eventN

输出会区分「点动」和「保持」——这决定了它能不能当 hold-to-enable 的
dead-man 用：点动式按钮按下即抬起，握不住，只能改成 toggle 语义。
"""
from __future__ import annotations

import argparse
import fcntl
import glob
import os
import re
import select
import signal
import struct
import sys
import time

# struct input_event: { struct timeval time; __u16 type; __u16 code; __s32 value; }
# 64 位内核上 timeval 是两个 64 位量，所以是 "llHHi" 再加 4 字节尾部对齐。
EV_FMT = "llHHi"
EV_SIZE = struct.calcsize(EV_FMT)

EV_SYN, EV_KEY, EV_REL, EV_ABS, EV_MSC = 0x00, 0x01, 0x02, 0x03, 0x04
TYPE_NAME = {EV_SYN: "EV_SYN", EV_KEY: "EV_KEY", EV_REL: "EV_REL",
             EV_ABS: "EV_ABS", EV_MSC: "EV_MSC"}

HEADER = "/usr/include/linux/input-event-codes.h"


def load_key_names() -> dict[int, str]:
    """从内核头文件里解析 KEY_*/BTN_* 名字。

    自己维护一份表迟早会和内核对不上，而且这些设备偏偏爱用冷门键码。
    """
    names: dict[int, str] = {}
    try:
        with open(HEADER) as fh:
            text = fh.read()
    except OSError:
        return names
    # 两遍：先收所有 #define，再解析，因为有 KEY_X 定义成 KEY_Y 的别名
    raw: dict[str, str] = {}
    for m in re.finditer(r"^#define\s+((?:KEY|BTN)_\w+)\s+(\S+)", text, re.M):
        raw[m.group(1)] = m.group(2)
    for name, val in raw.items():
        seen = set()
        v = val
        while v in raw and v not in seen:      # 跟随别名链
            seen.add(v)
            v = raw[v]
        try:
            code = int(v, 0)
        except ValueError:
            continue
        # 同一码值可能有多个名字（BTN_LEFT/BTN_MOUSE）。留最短的，读起来干净。
        if code not in names or len(name) < len(names[code]):
            names[code] = name
    return names


def stamp() -> str:
    """HH:MM:SS.mmm off one clock.

    Both halves have to come from the same clock. Taking the seconds from
    time.strftime() and the milliseconds from time.monotonic() prints
    timestamps whose milliseconds run backwards inside a second, which makes
    a hold impossible to measure off the log.
    """
    t = time.time()
    return f"{time.strftime('%H:%M:%S', time.localtime(t))}.{int(t * 1000) % 1000:03d}"


def dev_name(fd: int) -> str:
    buf = bytearray(256)
    try:
        # EVIOCGNAME(len) = _IOR('E', 0x06, len)
        fcntl.ioctl(fd, (2 << 30) | (len(buf) << 16) | (ord("E") << 8) | 0x06, buf)
    except OSError:
        return "?"
    return buf.split(b"\x00", 1)[0].decode(errors="replace")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--only", default="",
                    help="逗号分隔的 eventN 编号，只监听这几个")
    ap.add_argument("--all-types", action="store_true",
                    help="连 EV_REL/EV_MSC 一起打印（默认只看按键）")
    args = ap.parse_args()

    keys = load_key_names()
    if not keys:
        print(f"注意：读不到 {HEADER}，键码只能显示数字", file=sys.stderr)

    wanted = {int(x) for x in args.only.split(",") if x.strip()}

    fds: dict[int, tuple[str, str]] = {}

    def scan(quiet: bool = False) -> list[str]:
        """打开所有还没打开的 event 节点，返回新增的。

        必须能重扫：拔插之后内核会重建节点，只在启动时枚举一次的话，重新插上
        的设备就再也看不见了 —— 而"拔了再插"恰恰是最需要观察的时刻。
        """
        found = sorted(glob.glob("/dev/input/event*"),
                       key=lambda q: int(q.rsplit("event", 1)[1]))
        if wanted:
            found = [q for q in found
                     if int(q.rsplit("event", 1)[1]) in wanted]
        have = {v[0] for v in fds.values()}
        added = []
        for path in found:
            if path in have:
                continue
            try:
                fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
            except OSError as exc:
                if not quiet:
                    print(f"  跳过 {path}: {exc}", file=sys.stderr)
                continue
            fds[fd] = (path, dev_name(fd))
            added.append(path)
        return added

    scan()

    if not fds:
        print("没有能打开的 event 设备（需要在 input 组里）", file=sys.stderr)
        return 1

    print(f"监听 {len(fds)} 个设备 {args.seconds:.0f} 秒 —— 现在请按几下按钮"
          f"（长按、短按各来一次）\n")
    for fd, (path, name) in sorted(fds.items(), key=lambda kv: kv[1][0]):
        print(f"   {path:22s} {name}")
    print()

    # (path, code) -> [按下次数, 累计按住时长, 最近一次按下的时刻]
    stats: dict[tuple[str, int], list] = {}
    stopping = False

    def on_sig(_sig, _frm):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    t_end = time.monotonic() + args.seconds
    last_scan = time.monotonic()
    while not stopping:
        remain = t_end - time.monotonic()
        if remain <= 0:
            break
        if fds:
            ready, _, _ = select.select(list(fds), [], [], min(remain, 0.5))
        else:
            time.sleep(min(remain, 0.5))
            ready = []
        dead: list[int] = []
        for fd in ready:
            path, name = fds[fd]
            try:
                data = os.read(fd, EV_SIZE * 64)
            except OSError as exc:
                # ENODEV means the device was unplugged. Continuing would spin:
                # select() keeps reporting the dead fd readable forever.
                print(f"  {path} 消失了({exc.strerror})，不再监听它",
                      file=sys.stderr)
                dead.append(fd)
                continue
            for off in range(0, len(data) - EV_SIZE + 1, EV_SIZE):
                _, _, etype, code, value = struct.unpack_from(EV_FMT, data, off)
                if etype == EV_SYN:
                    continue
                if etype != EV_KEY and not args.all_types:
                    continue
                now = time.monotonic()
                label = keys.get(code, f"0x{code:x}") if etype == EV_KEY \
                    else f"code={code}"
                if etype == EV_KEY:
                    st = stats.setdefault((path, code), [0, 0.0, None])
                    if value == 1:
                        st[0] += 1
                        st[2] = now
                    elif value == 0 and st[2] is not None:
                        st[1] += now - st[2]
                        st[2] = None
                    action = {0: "抬起", 1: "按下", 2: "重复"}.get(value, str(value))
                    print(f"  {stamp()} "
                          f"{path:16s} {TYPE_NAME.get(etype, etype):7s} "
                          f"{label:20s} {action}")
                else:
                    print(f"  {stamp()} "
                          f"{path:16s} {TYPE_NAME.get(etype, etype):7s} "
                          f"{label:20s} value={value}")
        for fd in dead:
            os.close(fd)
            fds.pop(fd, None)

        # 定期重扫，接住拔插之后重建出来的节点
        if time.monotonic() - last_scan >= 2.0:
            last_scan = time.monotonic()
            for path in scan(quiet=True):
                fd = next(f for f, v in fds.items() if v[0] == path)
                print(f"  + 新设备 {path}  {fds[fd][1]}", flush=True)
        if not fds:
            print("  当前没有可读设备，等待插入…", file=sys.stderr, flush=True)

    print("\n=== 汇总 ===")
    if not stats:
        print("  没收到任何按键事件。按钮可能只走 hidraw（不产生 evdev 事件），"
              "或者根本没按到。")
        for fd in fds:
            os.close(fd)
        return 2

    for (path, code), (count, held, _) in sorted(stats.items()):
        name = fds[next(fd for fd, v in fds.items() if v[0] == path)][1]
        label = keys.get(code, f"0x{code:x}")
        avg = held / count if count else 0.0
        kind = "保持式（能当 hold-to-enable）" if avg >= 0.15 else \
               "点动式（按下即抬起，握不住）"
        print(f"  {path}  ({name})")
        print(f"    {label} (code={code})  按了 {count} 次，"
              f"平均按住 {avg*1000:.0f} ms  -> {kind}")

    for fd in fds:
        os.close(fd)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
