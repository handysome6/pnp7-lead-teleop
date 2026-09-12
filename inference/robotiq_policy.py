"""Bounded Modbus adapter for the existing ROS policy client, not an arm bridge.

Shadow mode sends FC04 status requests only. Motion mode never resets or
calibrates the gripper. Every GoTo requires an enabled dead-man and a fresh
policy update; release/staleness clears GoTo without opening the fingers.
"""

import fcntl
import threading
import time
import termios

import serial


def crc16(data):
    result = 0xFFFF
    for byte in data:
        result ^= byte
        for _ in range(8):
            result = (result >> 1) ^ 0xA001 if result & 1 else result >> 1
    return result


def decode_status(reply):
    if len(reply) != 11 or reply[:3] != bytes((9, 4, 6)):
        raise RuntimeError("invalid Robotiq status response")
    if crc16(reply[:-2]) != int.from_bytes(reply[-2:], "little"):
        raise RuntimeError("Robotiq CRC mismatch")
    status, _, fault, requested, position, current = reply[3:9]
    return dict(ready=bool(status & 1) and (status >> 4) & 3 == 3 and fault == 0,
                requested=requested, position=position, fault=fault, current=current)


class RobotiqPolicy:
    def __init__(self, path):
        self.port = serial.Serial(path, 115200, timeout=.08, write_timeout=.08, exclusive=True)
        try:
            fcntl.ioctl(self.port.fileno(), termios.TIOCEXCL)
        except Exception:
            self.port.close()
            raise
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.status = None
        self.status_time = 0.0
        self.error = None
        self.motion_mode = False
        self.enabled = lambda: False
        self.target = None
        self.target_time = 0.0
        self.candidate = None
        self.candidate_count = 0
        self.vote_source = None
        self.switches = []
        self.commands = 0
        self.thread = threading.Thread(target=self._worker, daemon=True)
        self.thread.start()

    def _exchange(self, payload, size):
        request = payload + crc16(payload).to_bytes(2, "little")
        started = time.monotonic()
        phase = "write_request"
        try:
            count = self.port.write(request)
            if count != len(request):
                raise RuntimeError("short serial write: {}/{}".format(count, len(request)))
            phase = "read_reply"
            reply = self.port.read(size)
        except Exception as exc:
            raise RuntimeError("Robotiq FC{:02X} {} failed after {:.1f} ms: {}".format(
                payload[1], phase, 1000 * (time.monotonic() - started), exc)) from exc
        if len(reply) != size or reply[:2] != payload[:2]:
            raise RuntimeError("Robotiq response timeout or invalid function")
        if crc16(reply[:-2]) != int.from_bytes(reply[-2:], "little"):
            raise RuntimeError("Robotiq CRC mismatch")
        return reply

    def _write(self, action, target):
        # Same commissioned settings as collection: speed32, force0, slave9.
        payload = bytes((9, 16, 3, 0xE8, 0, 3, 6, action, 0, 0, target, 32, 0))
        reply = self._exchange(payload, 8)
        if reply[2:6] != bytes((3, 0xE8, 0, 3)):
            raise RuntimeError("Robotiq write acknowledgement mismatch")
        self.commands += 1

    def _worker(self):
        last_command = None
        last_write = 0.0
        try:
            while not self.stop_event.is_set():
                state = decode_status(self._exchange(bytes((9, 4, 7, 0xD0, 0, 3)), 11))
                with self.lock:
                    self.status, self.status_time = state, time.monotonic()
                    motion = self.motion_mode
                    target, target_time = self.target, self.target_time
                if motion:
                    if not state["ready"]:
                        raise RuntimeError("Robotiq not ready: {}".format(state))
                    allowed = self.enabled() and time.monotonic() - target_time < .25
                    command = (9, target) if allowed and target is not None else (1, state["position"])
                    if command != last_command or time.monotonic() - last_write >= .1:
                        time.sleep(.005)
                        self._write(*command)
                        last_command, last_write = command, time.monotonic()
                self.stop_event.wait(.02)
        except Exception as exc:
            self.error = exc
        finally:
            if self.motion_mode:
                try:
                    self._write(1, 0 if self.status is None else self.status["position"])
                except Exception as exc:
                    self.error = self.error or exc

    def snapshot(self):
        if self.error is not None:
            raise RuntimeError("Robotiq worker failed: {}".format(self.error))
        with self.lock:
            state, timestamp = self.status, self.status_time
        if state is None or time.monotonic() - timestamp > .2:
            raise RuntimeError("Robotiq status is not fresh")
        if not state["ready"]:
            raise RuntimeError("Robotiq not ready: {}".format(state))
        return dict(state)

    def enable_motion(self, enabled):
        state = self.snapshot()
        with self.lock:
            self.target = 0 if state["requested"] < 128 else 255
            self.enabled = enabled
            self.motion_mode = True

    def update(self, value):
        with self.lock:
            self.target_time = time.monotonic()
            self._consider(value)

    def vote(self, value, source):
        """One vote per policy chunk (`source`); repeats only keep GoTo fresh.

        Two consecutive agreeing chunks are required, so a single flow-noise
        sample cannot open a held grasp however many of its actions execute.
        """
        with self.lock:
            self.target_time = time.monotonic()
            if source != self.vote_source:
                self.vote_source = source
                self._consider(value)

    def _consider(self, value):
        desired = 0 if value >= .75 else 255 if value <= .25 else None
        if desired is None or desired == self.target:
            self.candidate, self.candidate_count = None, 0
        elif desired != self.candidate:
            self.candidate, self.candidate_count = desired, 1
        else:
            self.candidate_count += 1
            if self.candidate_count >= 2:
                self.target = desired
                self.candidate, self.candidate_count = None, 0
                self.switches.append((time.monotonic(), desired))

    def emergency_stop(self):
        with self.lock:
            self.target_time = 0.0

    def close(self):
        self.emergency_stop()
        self.stop_event.set()
        self.thread.join(timeout=1.0)
        if self.thread.is_alive():
            raise RuntimeError("Robotiq thread did not stop")
        try:
            fcntl.ioctl(self.port.fileno(), termios.TIOCNXCL)
        finally:
            self.port.close()
