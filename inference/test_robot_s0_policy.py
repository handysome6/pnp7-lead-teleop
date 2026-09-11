import threading
import io
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

import pnp7_policy as old
from robot_s0_policy import AsyncPolicy, Limits, TwoStepChunks, check_tracking, start_observation_devices
from robotiq_policy import RobotiqPolicy, crc16, decode_status


class FakePort:
    def __init__(self, *args, **kwargs):
        self.frames = []
        self.requested = 0
        self.position = 3
        self.reply = b""

    def fileno(self):
        return -1

    def write(self, message):
        self.frames.append(bytes(message))
        if message[1] == 4:
            body = bytes((9, 4, 6, 0x31, 0, 0, self.requested, self.position, 0))
        else:
            self.requested = message[10]
            body = bytes((9, 16, 3, 0xE8, 0, 3))
        self.reply = body + crc16(body).to_bytes(2, "little")
        return len(message)

    def read(self, size):
        return self.reply[:size]

    def close(self):
        pass


class Tests(unittest.TestCase):
    @staticmethod
    def packet(sequence, observed_at=0):
        return sequence, observed_at, np.zeros(3), np.eye(3), np.arange(70).reshape(10, 7)

    def test_two_steps_then_wait_without_replay(self):
        chunks = TwoStepChunks()
        packet = self.packet(1)
        self.assertIsNone(chunks.next_action(None, 0))
        for index in (0, 1):
            result = chunks.next_action(packet, .2 + index / 30)
            self.assertEqual(result[:2], (1, index))
            np.testing.assert_array_equal(result[2], packet[4][index])
        for _ in range(5):
            self.assertIsNone(chunks.next_action(packet, .3))
        self.assertEqual(chunks.next_action(self.packet(3, .2), .35)[:2], (3, 0))

    def test_new_result_does_not_interrupt_two_step_prefix(self):
        chunks = TwoStepChunks()
        self.assertEqual(chunks.next_action(self.packet(1), .1)[:2], (1, 0))
        self.assertEqual(chunks.next_action(self.packet(2, .05), .14)[:2], (1, 1))
        self.assertEqual(chunks.next_action(self.packet(2, .05), .18)[:2], (2, 0))

    def test_selected_chunk_staleness_and_shape(self):
        chunks = TwoStepChunks()
        chunks.next_action(self.packet(1), .39)
        with self.assertRaises(old.SafetyError):
            chunks.next_action(self.packet(2, .3), .401)
        for packet in (self.packet(1, 1), (1, 0, None, None, np.zeros((1, 7)))):
            with self.assertRaises(old.SafetyError):
                TwoStepChunks().next_action(packet, .1)

    def test_six_hz_inference_uses_only_twelve_actions_per_second(self):
        chunks = TwoStepChunks()
        executed = []
        # 30 Hz control ticks, new inference every five ticks (~167 ms).
        for tick in range(30):
            now = tick / 30
            packet = self.packet(1 + tick // 5, (tick // 5) / 6 - .16)
            result = chunks.next_action(packet, now)
            if result is not None:
                executed.append(result[:2])
        self.assertEqual(executed, [(seq, i) for seq in range(1, 7) for i in (0, 1)])

    def test_camera_warmup_precedes_any_serial_exchange(self):
        events = []
        cameras = [Mock(error=None), Mock(error=None)]
        for index, camera in enumerate(cameras):
            camera.start.side_effect = lambda i=index: events.append("start" + str(i))
            camera.latest.side_effect = lambda i=index: events.append("warm" + str(i))
        with patch("robot_s0_policy.RobotiqPolicy", side_effect=lambda port: events.append("serial")):
            start_observation_devices(cameras, "fake")
        self.assertEqual(events, ["start0", "start1", "warm0", "warm1", "serial"])

    def test_camera_failure_never_opens_serial(self):
        camera = Mock(error=None)
        camera.latest.side_effect = old.SafetyError("warming up")
        with patch("robot_s0_policy.RobotiqPolicy") as serial, self.assertRaises(old.SafetyError):
            start_observation_devices([camera], "fake", timeout=0)
        serial.assert_not_called()

    def test_serial_timeout_retains_failure_phase(self):
        grip = RobotiqPolicy.__new__(RobotiqPolicy)
        grip.port = Mock()
        grip.port.write.side_effect = TimeoutError("Write timeout")
        with self.assertRaisesRegex(RuntimeError, "FC04 write_request failed after .*Write timeout"):
            grip._exchange(bytes((9, 4, 7, 0xD0, 0, 3)), 11)
        grip.port.read.assert_not_called()

    def test_tracking_threefold_boundaries(self):
        p, r = np.zeros(3), np.eye(3)
        check_tracking(np.array([.0449, 0, 0]), old.rpy_to_rotation([0, 0, .2999]), p, r)
        for distance, angle in ((.0451, 0), (0, .3001), (float("nan"), 0)):
            with self.assertRaises(old.SafetyError):
                check_tracking(np.array([distance, 0, 0]), old.rpy_to_rotation([0, 0, angle]), p, r)

    def test_full_profile_and_async_errors(self):
        limits = Limits(Path(__file__).parent / "ros/norm_stats_40merged.json", "full")
        raw = np.array([.002, -.001, .003, .004, -.003, .002, .9])
        np.testing.assert_allclose(limits.action(raw), raw)
        with self.assertRaises(old.SafetyError):
            limits.action(np.ones(7))
        def infer():
            time.sleep(.02)
            return np.zeros(3), np.eye(3), np.tile(raw, (10, 1))
        worker = AsyncPolicy(infer)
        worker.start()
        time.sleep(.06)
        packet = worker.latest()
        self.assertIsNotNone(packet)
        self.assertEqual(packet[4].shape, (10, 7))
        worker.close()
        worker.packet = (packet[0], time.monotonic() - .401, *packet[2:])
        with self.assertRaises(old.SafetyError):
            worker.latest()
        worker.error = RuntimeError("test lost network")
        with self.assertRaises(old.SafetyError):
            worker.latest()

    def test_ros_switch_request_serializes(self):
        from controller_manager_msgs.srv import SwitchControllerRequest
        for start, stop in ((["cartesian_impedance_controller"], []),
                            ([], ["cartesian_impedance_controller"])):
            request = SwitchControllerRequest(start_controllers=start, stop_controllers=stop,
                                              strictness=2, start_asap=True, timeout=1.0)
            buffer = io.BytesIO()
            request.serialize(buffer)
            self.assertEqual(SwitchControllerRequest().deserialize(buffer.getvalue()).timeout, 1.0)

    def test_action_limits_and_frames(self):
        limits = Limits(Path(__file__).parent / "ros/norm_stats_40merged.json")
        for raw in ([0] * 7, [.002] * 6 + [1]):
            action = limits.action(raw)
            self.assertLessEqual(np.linalg.norm(action[:3]), .001000001)
            self.assertLessEqual(np.linalg.norm(action[3:6]), .003000001)
        for bad in ([np.nan] * 7, [1] * 7, [0] * 6):
            with self.assertRaises(old.SafetyError):
                limits.action(bad)
        with self.assertRaises(old.SafetyError):
            limits.position(np.zeros(3))
        rotation = old.rpy_to_rotation([.1, -.2, .3])
        np.testing.assert_allclose(old.make_state([.5, .2, .5], rotation, 1)[3:9],
                                   rotation[:, :2].T.reshape(-1), atol=1e-7)
        delta = np.array([.001, -.002, .003])
        new_rotation = rotation @ old.rpy_to_rotation(delta)
        np.testing.assert_allclose(old.rotation_to_rpy(rotation.T @ new_rotation), delta, atol=1e-10)

    def test_gripper_shadow_deadman_and_watchdog(self):
        with patch("robotiq_policy.serial.Serial", FakePort), patch("robotiq_policy.fcntl.ioctl"):
            grip = RobotiqPolicy("fake")
            try:
                time.sleep(.06)
                self.assertEqual(grip.snapshot()["requested"], 0)
                self.assertTrue(all(frame[1] == 4 for frame in grip.port.frames))
                enabled = threading.Event()
                grip.enable_motion(enabled.is_set)
                grip.update(0)
                self.assertEqual(grip.target, 0)  # debounce has not closed
                grip.update(0)
                self.assertEqual(grip.target, 255)
                time.sleep(.04)
                self.assertFalse(any(f[1] == 16 and f[7] == 9 for f in grip.port.frames))
                enabled.set()
                grip.update(0)
                time.sleep(.05)
                self.assertTrue(any(f[1] == 16 and f[7] == 9 and f[10] == 255 for f in grip.port.frames))
                enabled.clear()
                time.sleep(.05)
                writes = [f for f in grip.port.frames if f[1] == 16]
                self.assertEqual(writes[-1][7], 1)  # release clears GoTo
                enabled.set()
                grip.update(0)
                time.sleep(.32)
                writes = [f for f in grip.port.frames if f[1] == 16]
                self.assertEqual(writes[-1][7], 1)  # stale policy clears GoTo
            finally:
                grip.close()

    def test_bad_crc(self):
        with self.assertRaises(RuntimeError):
            decode_status(bytes((9, 4, 6, 0x31, 0, 0, 0, 3, 0, 0, 0)))


if __name__ == "__main__":
    unittest.main()
