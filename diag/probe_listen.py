"""Passive serial listener for the PNP-7 lead arm.

Opens /dev/ttyUSB0 at a range of baud rates and reports any unsolicited
traffic. This script never writes to the device.
"""
import time

import serial

PORT = "/dev/ttyUSB0"
BAUDS = [9600, 19200, 38400, 57600, 115200, 230400, 250000,
         460800, 500000, 921600, 1000000, 1500000, 2000000, 3000000]


def main():
    for baud in BAUDS:
        try:
            s = serial.Serial(PORT, baud, timeout=0.4)
        except Exception as exc:
            print(f"{baud:>8}  OPEN_FAIL {exc}")
            continue
        time.sleep(0.05)
        s.reset_input_buffer()
        t0 = time.time()
        buf = b""
        while time.time() - t0 < 1.0:
            n = s.in_waiting
            if n:
                buf += s.read(n)
            else:
                time.sleep(0.01)
        s.close()
        if buf:
            print(f"{baud:>8}  {len(buf):5d} bytes  {buf[:48].hex(' ')}")
        else:
            print(f"{baud:>8}      0 bytes")


if __name__ == "__main__":
    main()
