"""Read a run log and current ROS state; never issue control commands."""
import argparse
import json
import statistics
from pathlib import Path

import numpy as np
import rospy
from controller_manager_msgs.srv import ListControllers
from franka_msgs.msg import FrankaState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log")
    args = parser.parse_args()
    report = json.loads(Path(args.log).read_text())
    rospy.init_node("pnp7_verify_run", anonymous=True)
    state = rospy.wait_for_message("/franka_state_controller/franka_states", FrankaState, timeout=5)
    controllers = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)()
    positions = np.array([row["position"] for row in report["samples"]])
    end = np.asarray(state.O_T_EE)[12:15]
    errors = [name for name in state.current_errors.__slots__ if getattr(state.current_errors, name)]
    last_errors = [name for name in state.last_motion_errors.__slots__ if getattr(state.last_motion_errors, name)]
    result = dict(
        result=report["result"], commands_sent=report["commands_sent"], samples=len(positions),
        controller_states={c.name: c.state for c in controllers.controller},
        robot_mode=state.robot_mode, current_errors=errors, last_motion_errors=last_errors,
        initial_position_m=positions[0].tolist(), final_position_m=end.tolist(),
        net_displacement_mm=float(np.linalg.norm(end - positions[0]) * 1000),
        max_sample_displacement_mm=float(np.linalg.norm(positions - positions[0], axis=1).max() * 1000),
        inference_ms_mean=statistics.mean(row["inference_ms"] for row in report["samples"]),
        round_trip_ms_mean=statistics.mean(row["round_trip_ms"] for row in report["samples"]),
        gripper_requested_values=sorted({row["gripper"]["requested"] for row in report["samples"]}),
        gripper_writes=report["gripper_writes"],
        gripper_faults=sorted({row["gripper"]["fault"] for row in report["samples"]}),
        cleanup_errors=report["cleanup_errors"],
    )
    print(json.dumps(result, indent=2))
    if errors or state.robot_mode != FrankaState.ROBOT_MODE_IDLE or result["controller_states"].get(
            "cartesian_impedance_controller") != "stopped":
        raise RuntimeError("post-run idle check failed")


if __name__ == "__main__":
    main()
