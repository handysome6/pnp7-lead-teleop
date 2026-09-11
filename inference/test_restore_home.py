"""No hardware: mock every ROS/FCI boundary of the mandatory Home sequence."""
import unittest
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch

import restore_home as home


class Tests(unittest.TestCase):
    def test_config(self):
        with patch.object(home.Path, "read_text", return_value="robot_ip=172.16.0.2\nhome_qpos=0 1 2 3 4 5 6\n"):
            self.assertEqual(home.read_home_config("unused"), ("172.16.0.2", list(range(7))))
        for text in ("home_qpos=0 1", "home_qpos=nan 1 2 3 4 5 6",
                     "robot_ip=a\nrobot_ip=b"):
            with patch.object(home.Path, "read_text", return_value=text), self.assertRaises((ValueError, RuntimeError)):
                home.read_home_config("unused")

    def exercise(self, fail_home=False, bad_q=False, active=False):
        from franka_msgs.msg import FrankaState
        import rospy as real_rospy
        before = FrankaState()
        before.robot_mode = before.ROBOT_MODE_IDLE
        before.header.stamp = real_rospy.Time.from_sec(10)
        after = FrankaState()
        after.robot_mode = after.ROBOT_MODE_IDLE
        after.header.stamp = real_rospy.Time.from_sec(11)
        after.q = [1 if bad_q else 0] * 7
        ros = Mock()
        ros.Time.now.side_effect = [real_rospy.Time.from_sec(10.01),
                                    real_rospy.Time.from_sec(10.5), real_rospy.Time.from_sec(11.01)]
        params = {"/franka_control/robot_ip": "172.16.0.2",
                  "/cartesian_impedance_controller/policy_watchdog_available": True,
                  "/cartesian_impedance_controller/policy_tracking_position_limit": .09,
                  "/cartesian_impedance_controller/policy_tracking_rotation_limit": .6}
        ros.get_param.side_effect = lambda name, *default: params.get(name, default[0] if default else None)
        ros.wait_for_message.side_effect = [before, after]
        events = []
        def service(name, kind):
            def call():
                events.append(name.rsplit("/", 1)[-1])
                if name.endswith("list_controllers"):
                    return NS(controller=[NS(name="motion", state="running" if active else "stopped")])
                return NS(success=True, message="")
            return call
        ros.ServiceProxy.side_effect = service
        def native(config):
            events.append("home")
            if fail_home:
                raise RuntimeError("HOME_INTERRUPTED")
        with patch.object(home.Path, "resolve", return_value=home.ROOT / "conf/full100b.conf"), \
                patch.object(home, "read_home_config", return_value=("172.16.0.2", [0] * 7)), \
                patch.object(home, "run_native_home", side_effect=native), \
                patch.object(home.subprocess, "run") as scheduler:
            if fail_home or bad_q or active:
                with self.assertRaises(RuntimeError):
                    home.restore_home(ros, "unused")
                scheduler.assert_not_called()
            else:
                self.assertEqual(home.restore_home(ros, "unused")["max_joint_error_rad"], 0)
                scheduler.assert_called_once()
        return events

    def test_order_and_pose_verification(self):
        self.assertEqual(self.exercise(), ["list_controllers", "disconnect", "home", "connect"])

    def test_failed_home_never_reconnects(self):
        self.assertEqual(self.exercise(fail_home=True), ["list_controllers", "disconnect", "home"])

    def test_wrong_pose_blocks_execution(self):
        self.exercise(bad_q=True)

    def test_active_controller_blocks_home(self):
        self.assertEqual(self.exercise(active=True), ["list_controllers"])


if __name__ == "__main__":
    unittest.main()
