import threading
import io
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import numpy as np

import pnp7_policy as old
from robot_s0_policy import (AsyncPolicy, Controller, Limits, PrefixChunks, check_tracking, gripper_vote_value,
                             parse_args,
                             start_observation_devices)
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
        chunks = PrefixChunks()
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
        chunks = PrefixChunks()
        self.assertEqual(chunks.next_action(self.packet(1), .1)[:2], (1, 0))
        self.assertEqual(chunks.next_action(self.packet(2, .05), .14)[:2], (1, 1))
        self.assertEqual(chunks.next_action(self.packet(2, .05), .18)[:2], (2, 0))

    def test_late_actions_are_skipped_never_executed(self):
        chunks = PrefixChunks()
        self.assertEqual(chunks.next_action(self.packet(1), .39)[:2], (1, 0))
        self.assertIsNone(chunks.next_action(self.packet(2, .3), .45))   # index 1 of seq 1 runs 417 ms late
        self.assertIsNone(chunks.next_action(self.packet(3, .01), .45))  # seq 3 is 440 ms late on arrival
        self.assertIsNone(chunks.next_action(self.packet(3, .01), .48))  # never replayed while waiting
        self.assertEqual(chunks.next_action(self.packet(4, .3), .5)[:2], (4, 0))
        self.assertEqual([(s["sequence"], s["index"]) for s in chunks.skipped], [(1, 1), (3, 0)])
        for packet in (self.packet(1, 1), (1, 0, None, None, np.zeros((1, 7)))):
            with self.assertRaises(old.SafetyError):
                PrefixChunks().next_action(packet, .1)

    def test_six_hz_inference_uses_only_twelve_actions_per_second(self):
        chunks = PrefixChunks()
        executed = []
        # 30 Hz control ticks, new inference every five ticks (~167 ms).
        for tick in range(30):
            now = tick / 30
            packet = self.packet(1 + tick // 5, (tick // 5) / 6 - .16)
            result = chunks.next_action(packet, now)
            if result is not None:
                executed.append(result[:2])
        self.assertEqual(executed, [(seq, i) for seq in range(1, 7) for i in (0, 1)])

    def test_ten_step_prefix_bounds_lateness_per_action(self):
        chunks = PrefixChunks(10)
        # Selected 250 ms after its observation: every action then runs 250 ms after its intended time.
        self.assertEqual([chunks.next_action(self.packet(1), .25 + i / 30)[:2] for i in range(10)],
                         [(1, i) for i in range(10)])
        self.assertIsNone(chunks.next_action(self.packet(1), .6))
        chunks = PrefixChunks(10)
        for now in (.30, .34, .38, .42):
            self.assertIsNotNone(chunks.next_action(self.packet(2), now))
        self.assertIsNone(chunks.next_action(self.packet(2), .55))  # a stalled loop: index 4 runs 417 ms late
        self.assertEqual((chunks.skipped[-1]["sequence"], chunks.skipped[-1]["index"]), (2, 4))
        with self.assertRaises(old.SafetyError):
            PrefixChunks(10).next_action((1, 0, None, None, np.zeros((3, 7))), .1)

    def test_full_profile_duration_and_prefix_options(self):
        base = ["robot_s0_policy.py", "--mode", "live", "--log", "x.json"]
        with patch("sys.argv", base + ["--profile", "full", "--duration", "90", "--actions-per-inference", "10"]):
            args = parse_args()
            self.assertEqual((args.duration, args.actions_per_inference), (90, 10))
        with patch("sys.argv", base + ["--profile", "full"]):
            args = parse_args()
            self.assertEqual((args.actions_per_inference, args.gripper_vote), (2, "tail"))
        for extra in (["--profile", "full", "--duration", "91"], ["--actions-per-inference", "4"],
                      ["--profile", "full", "--actions-per-inference", "11"]):
            with patch("sys.argv", base + extra), patch("sys.stderr", io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args()

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

    def test_client_tracking_stop_off_is_explicit_and_full_only(self):
        base = ["robot_s0_policy.py", "--mode", "live", "--log", "x.json"]
        with patch("sys.argv", base + ["--profile", "full"]):
            args = parse_args()
            self.assertEqual((args.client_tracking_stop, args.target_lead_cap_mm), ("on", None))
        off = base + ["--profile", "full", "--client-tracking-stop", "off"]
        with patch("sys.argv", off):
            args = parse_args()
            self.assertEqual((args.client_tracking_stop, args.target_lead_cap_mm, args.target_lead_cap_rad),
                             ("off", 20.0, .10))
        with patch("sys.argv", off + ["--target-lead-cap-mm", "15"]):
            self.assertEqual(parse_args().target_lead_cap_mm, 15.0)
        for argv in (base + ["--client-tracking-stop", "off"],            # smoke profile
                     base + ["--profile", "full", "--target-lead-cap-mm", "15"],  # cap without off
                     off + ["--target-lead-cap-mm", "60"], off + ["--target-lead-cap-rad", "0"],
                     off + ["--target-lead-cap-mm", "nan"]):
            with patch("sys.argv", argv), patch("sys.stderr", io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args()

    def test_policy_step_and_noise_seed_select_known_server(self):
        base = ["robot_s0_policy.py", "--mode", "live", "--log", "x.json"]
        with patch("sys.argv", base):
            args = parse_args()
            self.assertEqual((args.policy_step, args.noise_seed), (7500, "fixed"))
        with patch("sys.argv", base + ["--policy-step", "2500"]):
            self.assertEqual(parse_args().policy_step, 2500)
        with patch("sys.argv", base + ["--noise-seed", "per-request"]):
            self.assertEqual(parse_args().noise_seed, "per-request")
        for extra in (["--policy-step", "1234"], ["--policy-step", "2500", "--noise-seed", "per-request"]):
            with patch("sys.argv", base + extra), patch("sys.stderr", io.StringIO()):
                with self.assertRaises(SystemExit):
                    parse_args()

    def test_seed_is_sent_only_when_requested(self):
        from protocol import decode_inference_request
        connection = old.PolicyConnection.__new__(old.PolicyConnection)
        connection.sock, connection.token, connection.request_id = Mock(), "t", 0
        sent = []
        def reply(request_id):
            return {"ok": True, "request_id": request_id, "actions": np.zeros((10, 7)).tolist(), "inference_ms": 1.0}
        image = np.zeros((4, 4, 3), dtype=np.uint8)
        with patch("pnp7_policy.send_frame", lambda sock, payload: sent.append(payload)), \
                patch("pnp7_policy.recv_frame", return_value=b""), \
                patch("pnp7_policy.decode_json", side_effect=[reply(0), reply(1)]):
            connection.infer(np.zeros(10), image, image, "p")
            connection.infer(np.zeros(10), image, image, "p", seed=42)
        metadata = [decode_inference_request(payload)[0] for payload in sent]
        self.assertNotIn("seed", metadata[0])
        self.assertEqual(metadata[1]["seed"], 42)

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
        worker.packet = (packet[0], time.monotonic() - .5, *packet[2:])
        self.assertIs(worker.latest(), worker.packet)  # waiting for a newer chunk is not an error
        worker.packet = (packet[0], time.monotonic() - 1.001, *packet[2:])
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

    def test_gripper_vote_value_uses_planned_end_state(self):
        outlier, release = np.zeros((10, 7)), np.zeros((10, 7))
        outlier[:, 6] = [.98, .94, .89, .98, .96, .10, .04, .04, .03, .04]  # dropped a cube mid-transport
        release[:, 6] = [.02, .02, .03, .02, .02, .9, 1, 1, 1.03, 1]
        self.assertLessEqual(gripper_vote_value(outlier, "tail"), .25)  # open-first noise votes closed
        self.assertGreaterEqual(gripper_vote_value(release, "tail"), .75)  # planned release votes open
        self.assertTrue(.25 < gripper_vote_value(outlier, "chunk") < .75)
        self.assertTrue(.25 < gripper_vote_value(release, "chunk") < .75)

    def test_gripper_chunk_votes_need_two_chunks(self):
        with patch("robotiq_policy.serial.Serial", FakePort), patch("robotiq_policy.fcntl.ioctl"):
            grip = RobotiqPolicy("fake")
            try:
                grip.target = 255  # holding a grasp
                for value, source in ((.98, 7), (.98, 7), (.98, 7), (.02, 8)):
                    grip.vote(value, source)
                self.assertEqual(grip.target, 255)  # one outlier chunk never opens, however many actions run
                grip.vote(.98, 9)
                self.assertEqual(grip.target, 255)
                grip.vote(.99, 10)
                self.assertEqual((grip.target, grip.switches[-1][1]), (0, 0))  # two consecutive chunks agree
                for value, source in ((.02, 11), (.5, 12), (.02, 13)):
                    grip.vote(value, source)
                self.assertEqual(grip.target, 0)  # an undecided chunk resets the count
                before = grip.target_time
                time.sleep(.01)
                grip.vote(.02, 13)
                self.assertGreater(grip.target_time, before)  # repeated actions still keep GoTo fresh
                self.assertEqual(grip.target, 0)
            finally:
                grip.close()

    def test_controller_routes_chunk_votes(self):
        controller = Controller(Mock(), Mock(), Mock())
        controller.deadman = Mock(enabled=Mock(return_value=True))
        controller.gripper = Mock()
        with patch.object(Controller, "_pose_message", return_value="pose"):
            self.assertTrue(controller.publish_policy_target(np.zeros(3), np.eye(3), .02, source=5))
            controller.gripper.vote.assert_called_once_with(.02, 5)
            controller.gripper.update.assert_not_called()
            self.assertTrue(controller.publish_policy_target(np.zeros(3), np.eye(3), .9))
            controller.gripper.update.assert_called_once_with(.9)
            controller.deadman.enabled.return_value = False
            self.assertFalse(controller.publish_policy_target(np.zeros(3), np.eye(3), .02, source=6))
        self.assertEqual(controller.publisher.publish.call_count, 2)

    def test_bad_crc(self):
        with self.assertRaises(RuntimeError):
            decode_status(bytes((9, 4, 6, 0x31, 0, 0, 0, 3, 0, 0, 0)))


if __name__ == "__main__":
    unittest.main()
