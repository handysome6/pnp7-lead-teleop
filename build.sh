#!/usr/bin/env bash
# Build the PNP-7 teleop bridge against the libfranka and DynamixelSDK trees
# already present on this machine.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIBFRANKA="${LIBFRANKA:-$HOME/catkin_franka/libfranka}"
DXL="${DXL:-$HERE/DynamixelSDK/c++}"

mkdir -p "$HERE/bin"

g++ -O2 -std=c++17 -Wall -Wextra -pthread \
  -I"$LIBFRANKA/include" \
  -I/usr/include/eigen3 \
  -I"$DXL/include/dynamixel_sdk" \
  -I"$DXL/include" \
  "$HERE/src/pnp7_teleop.cpp" \
  -o "$HERE/bin/pnp7_teleop" \
  -L"$LIBFRANKA/build" -lfranka \
  -L"$DXL/build/linux64" -ldxl_x64_cpp \
  -Wl,-rpath,"$LIBFRANKA/build" \
  -Wl,-rpath,"$DXL/build/linux64"

echo "built $HERE/bin/pnp7_teleop"
