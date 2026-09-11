"""Keep only franka_control's main FCI thread at FIFO99 before a live test.

libfranka's constructor elevates its caller; later background threads can inherit
that priority. Run while the impedance controller is inactive, never during motion.
Only threads of this user's exact ROS franka_control process are touched.
"""
import json
import os
from pathlib import Path
from xmlrpc.client import ServerProxy


def prepare():
    import rospy
    from controller_manager_msgs.srv import ListControllers
    rospy.init_node("pnp7_prepare_realtime", anonymous=True)
    states = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)().controller
    if any(c.name != "franka_state_controller" and c.state == "running" for c in states):
        raise RuntimeError("stop motion controllers before scheduler setup")
    code, _, uri = rospy.get_master().lookupNode("/franka_control")
    if code != 1:
        raise RuntimeError("franka_control is unavailable")
    code, _, pid = ServerProxy(uri).getPid(rospy.get_name())
    root = Path("/proc") / str(pid)
    if code != 1 or root.stat().st_uid != os.getuid():
        raise RuntimeError("unexpected control process owner")
    executable = root.joinpath("cmdline").read_bytes().split(b"\0")[0].decode()
    if Path(executable).name != "franka_control_node":
        raise RuntimeError("unexpected ROS control executable")
    if os.sched_getscheduler(pid) != os.SCHED_FIFO or os.sched_getparam(pid).sched_priority < 1:
        raise RuntimeError("FCI thread lacks realtime scheduling")
    changed = []
    for item in root.joinpath("task").iterdir():
        tid = int(item.name)
        if tid != pid and os.sched_getscheduler(tid) != os.SCHED_OTHER:
            os.sched_setscheduler(tid, os.SCHED_OTHER, os.sched_param(0))
            changed.append(tid)
    print(json.dumps(dict(result="REALTIME_SETUP_PASS", pid=pid,
                          fci_priority=os.sched_getparam(pid).sched_priority,
                          background_threads_demoted=changed)))


if __name__ == "__main__":
    prepare()
