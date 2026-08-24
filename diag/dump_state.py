"""Read-only inventory of the PNP-7 lead arm servo bus.

Reads EEPROM/RAM registers for IDs 1-8. Performs no writes, so torque state
and every stored setting are left exactly as found.
"""
from dynamixel_sdk import PortHandler, PacketHandler

PORT = "/dev/ttyUSB0"
BAUD = 57600
IDS = range(1, 9)

# XL330 / X-series control table: name -> (address, num_bytes)
REGS = [
    ("model",         0,   2),
    ("firmware",      6,   1),
    ("id",            7,   1),
    ("baud_code",     8,   1),
    ("drive_mode",   10,   1),
    ("operating_mode", 11, 1),
    ("homing_offset", 20,  4),
    ("min_pos_limit", 52,  4),
    ("max_pos_limit", 48,  4),
    ("torque_enable", 64,  1),
    ("present_pwm",  124,  2),
    ("present_current", 126, 2),
    ("present_velocity", 128, 4),
    ("present_position", 132, 4),
    ("present_temp", 146,  1),
    ("present_volt", 144,  2),
]

BAUD_CODES = {0: 9600, 1: 57600, 2: 115200, 3: 1000000, 4: 2000000,
              5: 3000000, 6: 4000000}
MODEL_NAMES = {1190: "XL330-M077-T", 1200: "XL330-M288-T"}
OP_MODES = {0: "current", 1: "velocity", 3: "position",
            4: "extended-position", 5: "current-based-position", 16: "pwm"}


def signed(val, nbytes):
    bits = nbytes * 8
    return val - (1 << bits) if val >= (1 << (bits - 1)) else val


def main():
    port = PortHandler(PORT)
    packet = PacketHandler(2.0)
    if not port.openPort():
        raise SystemExit("failed to open port")
    if not port.setBaudRate(BAUD):
        raise SystemExit("failed to set baud rate")

    for sid in IDS:
        vals = {}
        for name, addr, nbytes in REGS:
            reader = {1: packet.read1ByteTxRx, 2: packet.read2ByteTxRx,
                      4: packet.read4ByteTxRx}[nbytes]
            data, comm, err = reader(port, sid, addr)
            vals[name] = data if comm == 0 and err == 0 else None

        model = vals["model"]
        print(f"--- ID {sid} ---")
        print(f"  model            {model} ({MODEL_NAMES.get(model, '?')})")
        print(f"  firmware         {vals['firmware']}")
        print(f"  baud             {BAUD_CODES.get(vals['baud_code'], '?')} "
              f"(code {vals['baud_code']})")
        print(f"  operating_mode   {vals['operating_mode']} "
              f"({OP_MODES.get(vals['operating_mode'], '?')})")
        print(f"  TORQUE_ENABLE    {vals['torque_enable']}")
        print(f"  homing_offset    {signed(vals['homing_offset'], 4)}")
        print(f"  pos_limits       {vals['min_pos_limit']} .. {vals['max_pos_limit']}")
        print(f"  present_position {vals['present_position']} "
              f"({vals['present_position'] * 360.0 / 4096.0:.2f} deg)")
        print(f"  present_current  {signed(vals['present_current'], 2)}")
        print(f"  temp / volt      {vals['present_temp']} C / "
              f"{vals['present_volt'] / 10.0:.1f} V")

    port.closePort()


if __name__ == "__main__":
    main()
