"""Benchmark sustained SyncRead rate from the PNP-7 lead arm.

Read-only. Registers 128..135 are contiguous (present_velocity 128:4,
present_position 132:4), so one SyncRead fetches both per servo.
"""
import argparse
import statistics
import time

from dynamixel_sdk import PortHandler, PacketHandler, GroupSyncRead

PORT = "/dev/ttyUSB0"
IDS = list(range(1, 9))
ADDR_PRESENT_VELOCITY = 128


def bench(baud, length, seconds=3.0):
    port = PortHandler(PORT)
    packet = PacketHandler(2.0)
    port.openPort()
    port.setBaudRate(baud)

    addr = ADDR_PRESENT_VELOCITY if length == 8 else 132
    group = GroupSyncRead(port, packet, addr, length)
    for sid in IDS:
        group.addParam(sid)

    dts, ok, fail = [], 0, 0
    t_end = time.time() + seconds
    prev = time.perf_counter()
    while time.time() < t_end:
        comm = group.txRxPacket()
        good = comm == 0 and all(
            group.isAvailable(sid, addr, length) for sid in IDS
        )
        ok, fail = (ok + 1, fail) if good else (ok, fail + 1)
        now = time.perf_counter()
        dts.append((now - prev) * 1000.0)
        prev = now

    port.closePort()
    dts = dts[1:]
    return {
        "baud": baud, "len": length, "ok": ok, "fail": fail,
        "hz": 1000.0 / statistics.mean(dts),
        "mean_ms": statistics.mean(dts),
        "p95_ms": sorted(dts)[int(len(dts) * 0.95)],
        "max_ms": max(dts),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baud", type=int, default=57600)
    args = ap.parse_args()

    for length in (4, 8):
        r = bench(args.baud, length)
        print(f"baud={r['baud']:<8} bytes/servo={r['len']}  "
              f"rate={r['hz']:6.1f} Hz  mean={r['mean_ms']:5.2f} ms  "
              f"p95={r['p95_ms']:5.2f} ms  max={r['max_ms']:6.2f} ms  "
              f"ok={r['ok']} fail={r['fail']}")


if __name__ == "__main__":
    main()
