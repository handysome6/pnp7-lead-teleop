"""Pre-flight readiness check for PNP-7 -> Franka data collection.

Reports every gate that must be green before teleoperation can run. Purely
diagnostic: it reads the lead arm bus, queries the Franka controller's status
endpoint and TCP ports, and enumerates cameras. It never commands anything.
"""
from __future__ import annotations

import argparse
import json
import socket
import ssl
import sys
import time
import urllib.request

ROBOT_IP = "172.16.0.2"
KEY_MAX = 0x2ff
PORT_FCI = 1337
PORT_GRIPPER = 1338

OK, WARN, FAIL = "PASS", "WARN", "FAIL"
SYMBOL = {OK: "[ ok ]", WARN: "[warn]", FAIL: "[FAIL]"}


class Report:
    def __init__(self) -> None:
        self.rows: list[tuple[str, str, str]] = []

    def add(self, status: str, name: str, detail: str = "") -> None:
        self.rows.append((status, name, detail))
        print(f"{SYMBOL[status]} {name}" + (f"  --  {detail}" if detail else ""))

    def worst(self) -> str:
        if any(r[0] == FAIL for r in self.rows):
            return FAIL
        if any(r[0] == WARN for r in self.rows):
            return WARN
        return OK


def tcp_open(host: str, port: int, timeout: float = 2.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout):
            return True
    except OSError:
        return False


def check_lead(rep: Report, port: str, baud: int) -> None:
    try:
        from pnp7.lead import ALL_IDS, PNP7Lead
    except ImportError as exc:
        rep.add(FAIL, "lead arm driver import", str(exc))
        return

    lead = PNP7Lead(port, baud)
    try:
        lead.open()
    except Exception as exc:
        rep.add(FAIL, f"lead arm bus {port} @ {baud}", str(exc))
        return

    try:
        torque = lead.assert_torque_disabled()
        rep.add(OK, "lead arm torque disabled",
                f"all {len(torque)} servos passive")
    except Exception as exc:
        rep.add(FAIL, "lead arm torque state", str(exc))
        lead.close()
        return

    n, t0 = 0, time.perf_counter()
    while time.perf_counter() - t0 < 1.0:
        if lead.read() is not None:
            n += 1
    rate = n / (time.perf_counter() - t0)
    status = OK if rate >= 100 else (WARN if rate >= 50 else FAIL)
    rep.add(status, "lead arm sample rate",
            f"{rate:.0f} Hz over {len(ALL_IDS)} servos, "
            f"{lead.read_failures} failed frames")
    lead.close()


def load_key_names() -> dict[str, int]:
    """Kernel key names -> codes, parsed from the header.

    The dead-man key is no longer fixed: the SpaceMouse reported BTN_0, the
    button that replaced it emits an ordinary keyboard code. Reading the header
    keeps this in step with the kernel instead of with a stale local table.
    """
    import re
    try:
        text = open("/usr/include/linux/input-event-codes.h").read()
    except OSError:
        return {}
    raw = dict(re.findall(r"^#define\s+((?:KEY|BTN)_\w+)\s+(\S+)", text, re.M))
    out: dict[str, int] = {}
    for name, val in raw.items():
        seen = set()
        while val in raw and val not in seen:      # follow alias chains
            seen.add(val)
            val = raw[val]
        try:
            out[name] = int(val, 0)
        except ValueError:
            pass
    return out


def resolve_key(spec: str) -> tuple[int, str]:
    names = load_key_names()
    if spec in names:
        return names[spec], spec
    code = int(spec, 0)
    for name, value in names.items():
        if value == code:
            return code, name
    return code, f"code {code}"


def check_desktop_ignores(rep: Report, path: str) -> None:
    """The desktop must not also be listening to the dead-man button.

    The replacement button enumerates as a plain HID keyboard emitting KEY_F3,
    which some application holds a global binding for, so every press raised a
    window over the session. udev has to mark the device
    LIBINPUT_IGNORE_DEVICE; teleop's own EVIOCGRAB only covers the window in
    which teleop is actually running.
    """
    import os
    import subprocess

    target = os.path.realpath(path)
    try:
        info = subprocess.run(["udevadm", "info", "--query=env", f"--name={target}"],
                              capture_output=True, text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError) as exc:
        rep.add(WARN, "desktop ignores dead-man", f"cannot query udev: {exc}")
        return
    if "LIBINPUT_IGNORE_DEVICE=1" in info:
        rep.add(OK, "desktop ignores dead-man",
                "LIBINPUT_IGNORE_DEVICE=1 -- presses do not reach GNOME")
    else:
        rep.add(WARN, "desktop ignores dead-man",
                "not set -- the desktop still acts on the button outside "
                "teleop; reload 99-pnp7-lead.rules and replug")


def check_deadman(rep: Report, path: str, key_spec: str) -> None:
    """The dead-man is a safety device: verify it before a session, not during.

    This gate exists because a replug once renumbered the SpaceMouse from
    event11 to event6 while every other check still read PASS.
    """
    import fcntl
    import os
    import struct

    try:
        key_code, key_name = resolve_key(key_spec)
    except ValueError:
        rep.add(FAIL, "dead-man switch", f"unknown key {key_spec!r}")
        return

    if not os.path.exists(path):
        rep.add(FAIL, f"dead-man device {path}",
                "missing -- check the udev rule and that it is plugged in")
        return
    try:
        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    except OSError as exc:
        rep.add(FAIL, f"dead-man device {path}", f"cannot open: {exc}")
        return
    try:
        def _ioc_read(nr, size):
            # _IOC(_IOC_READ=2, 'E', nr, size)
            return (2 << 30) | (size << 16) | (ord("E") << 8) | nr

        # EVIOCGNAME(len) is nr 0x06
        name = bytearray(256)
        fcntl.ioctl(fd, _ioc_read(0x06, len(name)), name)
        dev_name = name.split(b"\x00", 1)[0].decode(errors="replace")

        # EVIOCGBIT(ev, len) is nr 0x20 + ev, so key bits are 0x20 + EV_KEY(1).
        # Passing 0x20 queries the event-TYPE bits instead and finds no buttons.
        EV_KEY = 1
        nbytes = (KEY_MAX + 7) // 8      # whole key bitmap, not just up to BTN_0
        keys = bytearray(nbytes)
        fcntl.ioctl(fd, _ioc_read(0x20 + EV_KEY, nbytes), keys)
        has_key = bool(keys[key_code // 8] & (1 << (key_code % 8)))
    except OSError as exc:
        os.close(fd)
        rep.add(WARN, f"dead-man device {path}", f"opened but query failed: {exc}")
        return
    os.close(fd)

    target = os.path.realpath(path)
    if has_key:
        rep.add(OK, "dead-man switch",
                f"{dev_name} at {target}, {key_name} present")
    else:
        rep.add(FAIL, "dead-man switch",
                f"{dev_name} at {target} does not report {key_name}")


def check_franka(rep: Report, ip: str) -> None:
    try:
        socket.setdefaulttimeout(2.0)
        t0 = time.perf_counter()
        sock = socket.create_connection((ip, PORT_FCI), 2.0)
        sock.close()
        rep.add(OK, "Franka FCI port 1337",
                f"open ({(time.perf_counter() - t0) * 1000:.1f} ms)")
    except OSError as exc:
        rep.add(FAIL, "Franka FCI port 1337", str(exc))
        return

    if tcp_open(ip, PORT_GRIPPER):
        rep.add(OK, "Franka gripper port 1338", "open, Franka Hand reachable")
    else:
        rep.add(FAIL, "Franka gripper port 1338",
                "closed -- enable Franka Hand in Desk > Settings > End Effector")

    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    try:
        with urllib.request.urlopen(
            f"https://{ip}/admin/api/system-status", context=ctx, timeout=4
        ) as resp:
            status = json.loads(resp.read())
    except Exception as exc:
        rep.add(WARN, "Franka Desk status endpoint", str(exc))
        return

    safety = status.get("safety", {})
    brakes = safety.get("brakeState", [])
    # This controller reports "Unlocked"; some firmware reports "Open".
    released = {"Open", "Unlocked"}
    locked = [i + 1 for i, b in enumerate(brakes) if b not in released]
    if brakes and not locked:
        rep.add(OK, "Franka brakes", f"all {len(brakes)} released")
    else:
        rep.add(FAIL, "Franka brakes",
                f"joints {locked} still locked -- unlock in Desk")

    sto = safety.get("stoState")
    if sto == "SafeTorqueOff":
        rep.add(FAIL, "Franka STO",
                "SafeTorqueOff active -- release the enabling device / e-stop")
    else:
        rep.add(OK, "Franka STO", str(sto))

    ctrl = safety.get("safetyControllerStatus")
    rep.add(OK if ctrl == "Work" else WARN, "Franka safety controller", str(ctrl))

    warnings = safety.get("activeWarnings", {})
    active = [k for k, v in warnings.items() if v]
    rep.add(WARN if active else OK, "Franka active warnings",
            ", ".join(active) if active else "none")


def check_cameras(rep: Report) -> None:
    try:
        import pyrealsense2 as rs
    except ImportError as exc:
        rep.add(WARN, "RealSense SDK", str(exc))
        return
    devices = list(rs.context().query_devices())
    if not devices:
        rep.add(FAIL, "RealSense cameras", "none enumerated")
        return
    names = ", ".join(
        f"{d.get_info(rs.camera_info.name)}:{d.get_info(rs.camera_info.serial_number)}"
        for d in devices
    )
    rep.add(OK if len(devices) >= 2 else WARN,
            f"RealSense cameras ({len(devices)})", names)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/pnp7_lead")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--robot-ip", default=ROBOT_IP)
    ap.add_argument("--deadman", default=None)
    ap.add_argument("--deadman-key", default=None,
                    help="key name or code the button emits (e.g. BTN_0, KEY_V)")
    ap.add_argument("--config", default="conf/pnp7_teleop.conf",
                    help="teleop config to take deadman_device/deadman_key "
                         "from, so this check and the bridge cannot disagree")
    ap.add_argument("--skip-cameras", action="store_true")
    args = ap.parse_args()

    # The config is the single source of truth for which device and which key.
    # Duplicating the default here is how the two drift apart, and a dead-man
    # that the pre-flight validated but the bridge cannot see is the worst
    # possible failure of this script.
    conf: dict[str, str] = {}
    try:
        with open(args.config) as fh:
            for line in fh:
                line = line.split("#", 1)[0].strip()
                if "=" in line:
                    k, v = line.split("=", 1)
                    conf[k.strip()] = v.strip()
    except OSError:
        pass

    deadman = args.deadman or conf.get("deadman_device", "/dev/pnp7_deadman")
    deadman_key = args.deadman_key or conf.get("deadman_key", "BTN_0")

    rep = Report()
    print("=== lead arm ===")
    check_lead(rep, args.port, args.baud)
    check_deadman(rep, deadman, deadman_key)
    check_desktop_ignores(rep, deadman)
    print("\n=== franka ===")
    check_franka(rep, args.robot_ip)
    if not args.skip_cameras:
        print("\n=== cameras ===")
        check_cameras(rep)

    worst = rep.worst()
    print(f"\noverall: {worst}")
    if worst == FAIL:
        print("teleoperation is blocked until the FAIL rows are cleared.")
    return 0 if worst != FAIL else 1


if __name__ == "__main__":
    sys.exit(main())
