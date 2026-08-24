"""Identify the PNP-7 lead arm servo bus protocol.

Sends only PING instructions (0x01). A ping carries no payload and cannot
enable torque or command motion; it only asks a servo to identify itself.
Covers Feetech SCS/STS/SMS + Dynamixel Protocol 1.0 (identical framing) and
Dynamixel Protocol 2.0.
"""
import time

import serial

PORT = "/dev/ttyUSB0"
BAUDS = [1000000, 115200, 500000, 57600, 921600, 250000, 2000000, 9600, 38400]
IDS = list(range(0, 21))


def crc16_ibm(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x8005) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def ping_p1(sid):
    """Feetech SCS/STS and Dynamixel 1.0 ping frame."""
    body = [sid, 0x02, 0x01]
    chk = (~sum(body)) & 0xFF
    return bytes([0xFF, 0xFF] + body + [chk])


def ping_p2(sid):
    """Dynamixel Protocol 2.0 ping frame."""
    pkt = [0xFF, 0xFF, 0xFD, 0x00, sid, 0x03, 0x00, 0x01]
    crc = crc16_ibm(pkt)
    return bytes(pkt + [crc & 0xFF, (crc >> 8) & 0xFF])


def scan(baud, builder, label):
    hits = []
    try:
        s = serial.Serial(PORT, baud, timeout=0.02)
    except Exception as exc:
        print(f"  {label} @ {baud}: OPEN_FAIL {exc}")
        return hits
    for sid in IDS:
        s.reset_input_buffer()
        s.write(builder(sid))
        s.flush()
        time.sleep(0.008)
        resp = s.read(64)
        if resp and len(resp) >= 4:
            hits.append((sid, resp))
    s.close()
    return hits


def main():
    for baud in BAUDS:
        for builder, label in ((ping_p1, "P1/Feetech"), (ping_p2, "P2/Dxl2")):
            hits = scan(baud, builder, label)
            status = "".join(
                f"\n      id={sid:<3} <- {resp[:16].hex(' ')}" for sid, resp in hits
            )
            print(f"{baud:>8} {label:<12} hits={len(hits)}{status}")


if __name__ == "__main__":
    main()
