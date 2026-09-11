#!/usr/bin/env bash
# Run with: pixi run bash diag/test_fci_continuity.sh (Linux, no hardware).
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${CONDA_PREFIX:?Run this offline test under pixi run}"
TEST_DIR="$(mktemp -d /tmp/pnp7-fci-test-XXXXXX)"
trap 'rm -rf "$TEST_DIR"' EXIT
"${CXX:-$CONDA_PREFIX/bin/g++}" -O2 -std=c++17 -Wall -Wextra -pthread \
  -I"$CONDA_PREFIX/include" -I"$CONDA_PREFIX/include/eigen3" \
  -I"$HERE/DynamixelSDK/c++/include/dynamixel_sdk" \
  -I"$HERE/DynamixelSDK/c++/include" \
  "$HERE/diag/test_fci_continuity.cpp" -o "$TEST_DIR/test" \
  -L"$CONDA_PREFIX/lib" -lfranka \
  -L"$HERE/DynamixelSDK/c++/build/linux64" -ldxl_x64_cpp \
  -Wl,-rpath,"$CONDA_PREFIX/lib" \
  -Wl,-rpath,"$HERE/DynamixelSDK/c++/build/linux64"
"$TEST_DIR/test"
