"""Read Robotiq 2F status over RS-485; never activate, reset or command motion.

Close other serial controllers before use. Both 2F-85 and 2F-140 use this status
layout; it does NOT identify the model or convert positions into millimetres.
Reference: Robotiq 2F-85/2F-140 instruction manual, Modbus RTU FC04, 0x07D0.
"""
import argparse
import json

import serial


def crc16(data):
    value = 0xFFFF
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ 0xA001 if value & 1 else value >> 1
    return value


def decode(response, slave):
    if len(response) != 11 or response[:3] != bytes((slave, 4, 6)):
        raise ValueError(f"invalid/incomplete FC04 response: {response.hex(' ')}")
    if crc16(response[:-2]) != int.from_bytes(response[-2:], "little"):
        raise ValueError("Robotiq response CRC mismatch")
    status, _, fault, requested, position, current = response[3:9]
    return dict(activated=bool(status & 1), go_to=bool(status & 8),
                activation_status=(status >> 4) & 3, object_status=status >> 6,
                fault=f"0x{fault:02x}", requested_position=requested,
                actual_position=position, current_mA=current * 10,
                ready=bool(status & 1) and ((status >> 4) & 3) == 3 and fault == 0,
                model="2F-85/2F-140 not distinguished by status registers")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", required=True, help="stable /dev/serial/by-id path")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--slave", type=int, default=9)
    args = parser.parse_args()
    if not 1 <= args.slave <= 247:
        parser.error("slave ID must be between 1 and 247")
    request = bytes((args.slave, 4, 7, 0xD0, 0, 3))
    request += crc16(request).to_bytes(2, "little")
    with serial.Serial(args.port, args.baud, timeout=0.5, write_timeout=0.5,
                       exclusive=True) as port:
        # FC04 reads three input registers; there are no output-register writes.
        port.write(request)
        result = decode(port.read(11), args.slave)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
