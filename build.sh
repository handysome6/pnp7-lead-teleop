#!/usr/bin/env bash
# Build the PNP-7 teleop bridge.
#
# Two dependency layouts are supported, because the two machines that build
# this are not the same shape:
#
#   libfranka source tree   headers in $LIBFRANKA/include, libfranka.so in
#                           $LIBFRANKA/build. The original robot PC -- see
#                           "libfranka version" in README.md.
#   conda/pixi prefix       headers in $PREFIX/include, libs in $PREFIX/lib.
#                           robot-s0, found via $CONDA_PREFIX with no argument
#                           at all when this runs under `pixi run`.
#
# An explicit LIBFRANKA always wins, so the robot PC's documented invocation
# behaves exactly as before.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DXL="${DXL:-$HERE/DynamixelSDK/c++}"
CXX="${CXX:-g++}"

if [ -n "${LIBFRANKA:-}" ]; then
  FRANKA_INC="$LIBFRANKA/include"
  FRANKA_LIB="$LIBFRANKA/build"
elif [ -n "${CONDA_PREFIX:-}" ] && [ -e "$CONDA_PREFIX/include/franka/robot.h" ]; then
  FRANKA_INC="$CONDA_PREFIX/include"
  FRANKA_LIB="$CONDA_PREFIX/lib"
else
  # The historical default. This tree holds the 0.15.0 build that stopped
  # connecting once the arm passed FR3 System Version 5.9.0, so reaching it
  # is nearly always a mistake -- the check below says so out loud.
  FRANKA_INC="$HOME/catkin_franka/libfranka/include"
  FRANKA_LIB="$HOME/catkin_franka/libfranka/build"
fi

if [ ! -e "$FRANKA_INC/franka/robot.h" ]; then
  echo "no libfranka headers under $FRANKA_INC" >&2
  echo "set LIBFRANKA to a built source tree, or run this under 'pixi run'" >&2
  exit 1
fi

# The bridge itself never names an Eigen type -- it only touches Robot,
# RobotState, JointPositions, Duration and Gripper, which are plain std::array
# and POD. libfranka's headers pull Eigen in transitively, though, so the
# include path still has to be there.
if [ -z "${EIGEN_INC:-}" ]; then
  for cand in "${CONDA_PREFIX:-/nonexistent}/include/eigen3" /usr/include/eigen3; do
    if [ -d "$cand" ]; then EIGEN_INC="$cand"; break; fi
  done
fi

INCS=(-I"$FRANKA_INC" -I"$DXL/include/dynamixel_sdk" -I"$DXL/include")
if [ -n "${EIGEN_INC:-}" ]; then INCS+=(-I"$EIGEN_INC"); fi

mkdir -p "$HERE/bin"

"$CXX" -O2 -std=c++17 -Wall -Wextra -pthread \
  "${INCS[@]}" \
  "$HERE/src/pnp7_teleop.cpp" \
  -o "$HERE/bin/pnp7_teleop" \
  -L"$FRANKA_LIB" -lfranka \
  -L"$DXL/build/linux64" -ldxl_x64_cpp \
  -Wl,-rpath,"$FRANKA_LIB" \
  -Wl,-rpath,"$DXL/build/linux64"

echo "built $HERE/bin/pnp7_teleop"
echo "  libfranka  $FRANKA_LIB"
echo "  eigen      ${EIGEN_INC:-<none found>}"
echo "  dynamixel  $DXL/build/linux64"
echo "  compiler   $("$CXX" -dumpversion) at $(command -v "$CXX")"
