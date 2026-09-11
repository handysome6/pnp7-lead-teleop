"""Exercise the real C++ driver against a fragmented Modbus pseudo-terminal."""
import errno
import os
import pty
import select
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from read_robotiq import crc16


def scenario(binary, mode):
    master, slave = pty.openpty()
    path = os.ttyname(slave)
    stop = threading.Event()
    actions = []
    errors = []
    state = dict(position=128, requested=128, action=1, reads=0, active=mode != 'init')

    def serve():
        buf = bytearray()
        try:
            while not stop.is_set():
                if not select.select([master], [], [], .1)[0]:
                    continue
                data = os.read(master, 4096)
                buf.extend(data)
                while len(buf) >= 2:
                    count = 8 if buf[1] == 4 else 15
                    if len(buf) < count:
                        break
                    req = bytes(buf[:count]); del buf[:count]
                    assert req[0] == 9 and crc16(req[:-2]) == int.from_bytes(req[-2:], 'little')
                    if req[1] == 16:
                        assert req[2:7] == bytes.fromhex('03 e8 00 03 06')
                        action, _, _, position, speed, force = req[7:13]
                        actions.append((action, position, speed, force))
                        assert action in (0, 1, 9), 'unexpected reset/release/action bits'
                        if mode != 'init':
                            assert action != 0, 'normal controller reset the gripper'
                        state.update(action=action, requested=position, active=bool(action & 1))
                        response = req[:6]
                    elif req[1] == 4:
                        assert req[2:6] == bytes.fromhex('07 d0 00 03')
                        state['reads'] += 1
                        if state['action'] & 8:
                            d = state['requested'] - state['position']
                            state['position'] += min(10, abs(d)) * (1 if d > 0 else -1)
                        fault = 14 if mode == 'fault' and state['requested'] == 0 else 0
                        if mode == 'timeout' and state['requested'] == 0 and state['position'] < 140:
                            continue
                        status = (0x31 if state['active'] else 0) | (state['action'] & 8)
                        response = bytes([9, 4, 6, status, 0, fault,
                                          state['requested'], state['position'], 0])
                    else:
                        raise AssertionError('unexpected function')
                    response += crc16(response).to_bytes(2, 'little')
                    if mode == 'crc' and state['requested'] == 0 and state['position'] < 140:
                        response = response[:-1] + bytes([response[-1] ^ 1])
                    # Exercise partial reads, including CRC arriving separately.
                    os.write(master, response[:3]); time.sleep(.002); os.write(master, response[3:])
        except OSError as exc:
            if exc.errno != errno.EIO:
                errors.append(exc)
        except BaseException as exc:
            errors.append(exc)
    thread = threading.Thread(target=serve)
    thread.start()
    try:
        result = subprocess.run([binary, path, mode], capture_output=True, text=True, timeout=8)
        print(result.stdout, end='')
        assert result.returncode == 0, result.stderr
        assert not errors, errors
        assert actions and actions[0][0] == (0 if mode == 'init' else 1)
        if mode != 'init':
            assert any(a[0] == 9 and a[1] == 255 for a in actions)
            assert any(a[0] == 9 and a[1] == 0 for a in actions)
            assert actions[-1][0] == 1, 'shutdown did not request stop'
    finally:
        stop.set(); thread.join(1); os.close(slave); os.close(master)


if __name__ == '__main__':
    for mode in ('normal', 'fault', 'timeout', 'crc', 'init'):
        scenario(sys.argv[1], mode)
