"""Change the bus baud rate of the PNP-7 lead arm servos.

This is the ONLY tool here that writes to the servos. It writes exactly one
EEPROM register (address 8, Baud Rate) on IDs 1..8. It never touches torque
enable, goal position, operating mode, limits or homing offset.

The change is persistent across power cycles and fully reversible by running
this tool again with the old rate.

Why it is needed: at the factory default of 57600 baud a SyncRead of 8 servos
takes ~29 ms (~34 Hz), because ~142 bytes must cross the wire per cycle. That
is the hard floor for lead-arm sampling and it is too slow for teleoperation.
At 1 Mbps the same traffic takes ~1.4 ms.

  python set_baud.py --show
  python set_baud.py --to 1000000 --yes
  python set_baud.py --from-baud 1000000 --to 57600 --yes   # revert
"""
from __future__ import annotations

import argparse
import sys
import time

from dynamixel_sdk import PortHandler, PacketHandler

from pnp7_lead import ALL_IDS

ADDR_BAUD_RATE = 8
ADDR_TORQUE_ENABLE = 64

BAUD_CODES = {9600: 0, 57600: 1, 115200: 2, 1000000: 3,
              2000000: 4, 3000000: 5, 4000000: 6}
CODE_TO_BAUD = {v: k for k, v in BAUD_CODES.items()}


def connect(port_name: str, baud: int):
    port = PortHandler(port_name)
    packet = PacketHandler(2.0)
    if not port.openPort():
        raise SystemExit(f"could not open {port_name}")
    if not port.setBaudRate(baud):
        raise SystemExit(f"could not set host baud {baud}")
    return port, packet


def survey(port, packet):
    found = {}
    for sid in ALL_IDS:
        code, comm, err = packet.read1ByteTxRx(port, sid, ADDR_BAUD_RATE)
        if comm == 0 and err == 0:
            torque, _, _ = packet.read1ByteTxRx(port, sid, ADDR_TORQUE_ENABLE)
            found[sid] = (code, torque)
    return found


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/pnp7_lead")
    ap.add_argument("--from-baud", type=int, default=1000000,
                    help="baud the servos are currently using")
    ap.add_argument("--to", type=int, choices=sorted(BAUD_CODES),
                    help="new baud rate to write")
    ap.add_argument("--show", action="store_true", help="report only, no writes")
    ap.add_argument("--yes", action="store_true", help="confirm the write")
    args = ap.parse_args()

    port, packet = connect(args.port, args.from_baud)
    found = survey(port, packet)

    if not found:
        port.closePort()
        raise SystemExit(f"no servos answered at {args.from_baud} baud")

    print(f"servos responding at {args.from_baud}:")
    for sid, (code, torque) in sorted(found.items()):
        print(f"  id {sid}: baud={CODE_TO_BAUD.get(code, '?')} (code {code}) "
              f"torque_enable={torque}")

    if args.show or args.to is None:
        port.closePort()
        return 0

    missing = [s for s in ALL_IDS if s not in found]
    if missing:
        port.closePort()
        raise SystemExit(f"refusing to change baud: ids {missing} did not answer")

    live = [sid for sid, (_, torque) in found.items() if torque]
    if live:
        port.closePort()
        raise SystemExit(f"refusing: torque is enabled on {live}")

    if not args.yes:
        port.closePort()
        raise SystemExit("refusing to write without --yes")

    code = BAUD_CODES[args.to]
    print(f"\nwriting baud code {code} ({args.to}) to ids {list(ALL_IDS)} ...")
    for sid in ALL_IDS:
        comm, err = packet.write1ByteTxRx(port, sid, ADDR_BAUD_RATE, code)
        # The servo switches rate immediately, so its status packet is sent at
        # the NEW rate and cannot be read back here. A comm timeout is expected
        # and is not an error; verification happens below at the new rate.
        print(f"  id {sid}: sent (comm={comm}, err={err})")
        time.sleep(0.05)

    port.closePort()
    time.sleep(0.3)

    print(f"\nverifying at {args.to} ...")
    port, packet = connect(args.port, args.to)
    after = survey(port, packet)
    port.closePort()

    for sid in ALL_IDS:
        if sid in after:
            print(f"  id {sid}: OK, baud={CODE_TO_BAUD.get(after[sid][0], '?')}")
        else:
            print(f"  id {sid}: NO RESPONSE")

    if len(after) != len(ALL_IDS):
        print("\nWARNING: not every servo answered at the new rate.")
        print(f"Re-run with --from-baud {args.from_baud} to find stragglers.")
        return 1

    print(f"\nall {len(ALL_IDS)} servos now at {args.to} baud")
    return 0


if __name__ == "__main__":
    sys.exit(main())
