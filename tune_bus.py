"""Tune the PNP-7 lead arm bus timing.

Writes EEPROM address 9 (Return Delay Time) on IDs 1..8. Each servo waits this
long before answering a request; the factory default of 250 (=500 us) costs
~4 ms per SyncRead once eight servos are chained, which dominates the cycle at
1 Mbps. Setting it to 0 removes that wait.

Like set_baud.py this touches nothing else -- no torque enable, no goal
register, no limits. Reversible by running again with --raw 250.

  python tune_bus.py --show
  python tune_bus.py --raw 0 --yes
"""
from __future__ import annotations

import argparse
import sys

from dynamixel_sdk import PortHandler, PacketHandler

from pnp7_lead import ALL_IDS

ADDR_RETURN_DELAY = 9
ADDR_TORQUE_ENABLE = 64


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/pnp7_lead")
    ap.add_argument("--baud", type=int, default=1000000)
    ap.add_argument("--raw", type=int, choices=range(0, 254), metavar="0-253",
                    help="return delay in units of 2 us (0 = no delay)")
    ap.add_argument("--show", action="store_true")
    ap.add_argument("--yes", action="store_true")
    args = ap.parse_args()

    port = PortHandler(args.port)
    packet = PacketHandler(2.0)
    if not port.openPort() or not port.setBaudRate(args.baud):
        raise SystemExit(f"could not open {args.port} at {args.baud}")

    current = {}
    for sid in ALL_IDS:
        val, comm, err = packet.read1ByteTxRx(port, sid, ADDR_RETURN_DELAY)
        torque, tcomm, terr = packet.read1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE)
        if comm == 0 and err == 0:
            current[sid] = (val, torque if tcomm == 0 and terr == 0 else None)

    print(f"return delay at {args.baud} baud:")
    for sid, (val, torque) in sorted(current.items()):
        print(f"  id {sid}: raw={val} ({val * 2} us)  torque_enable={torque}")

    if args.show or args.raw is None:
        port.closePort()
        return 0

    missing = [s for s in ALL_IDS if s not in current]
    if missing:
        port.closePort()
        raise SystemExit(f"refusing: ids {missing} did not answer")

    live = [sid for sid, (_, t) in current.items() if t]
    if live:
        port.closePort()
        raise SystemExit(f"refusing: torque is enabled on {live}")

    if not args.yes:
        port.closePort()
        raise SystemExit("refusing to write without --yes")

    print(f"\nwriting return delay raw={args.raw} ({args.raw * 2} us) ...")
    ok = True
    for sid in ALL_IDS:
        comm, err = packet.write1ByteTxRx(port, sid, ADDR_RETURN_DELAY, args.raw)
        readback, rcomm, rerr = packet.read1ByteTxRx(port, sid, ADDR_RETURN_DELAY)
        good = comm == 0 and err == 0 and rcomm == 0 and readback == args.raw
        ok &= good
        print(f"  id {sid}: {'OK' if good else 'FAILED'} (readback={readback})")

    port.closePort()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
