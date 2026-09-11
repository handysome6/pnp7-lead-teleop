#!/usr/bin/env python3
"""Verify ROS imports and shared-library linkage without connecting to the robot."""

import ctypes
import io
import json
from pathlib import Path

import cv2
import pyrealsense2 as rs
import rospy
import serial
from franka_msgs.msg import FrankaState
from geometry_msgs.msg import PoseStamped


def main():
    root = Path(__file__).resolve().parent
    libraries = [
        root / "vendor/install/lib/libfranka.so.0.21.3",
        root / "devel/lib/libfranka_hw.so",
        root / "devel/lib/libfranka_control_services.so",
        root / "devel/lib/libserl_franka_controllers.so",
    ]
    handles = [ctypes.CDLL(str(path)) for path in libraries]
    for message in (FrankaState(), PoseStamped()):
        buffer = io.BytesIO()
        message.serialize(buffer)
        type(message)().deserialize(buffer.getvalue())
    context = rs.context()
    cameras = [device.get_info(rs.camera_info.serial_number)
               for device in context.query_devices()]
    print(json.dumps({
        "result": "ROS_BUILD_CHECK_PASS",
        "libraries_loaded": len(handles),
        "rospy": rospy.__file__,
        "opencv": cv2.__version__,
        "pyserial": serial.__version__,
        "camera_serials": cameras,
        "robot_connected": False,
        "motion_commands_sent": 0,
    }, indent=2))


if __name__ == "__main__":
    main()
