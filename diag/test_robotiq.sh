#!/usr/bin/env bash
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
: "${CONDA_PREFIX:?Run under pixi run}"
TEST_DIR="$(mktemp -d /tmp/pnp7-robotiq-test-XXXXXX)"
trap 'rm -rf "$TEST_DIR"' EXIT
"${CXX:-$CONDA_PREFIX/bin/g++}" -O2 -std=c++17 -Wall -Wextra -pthread \
  "$HERE/diag/test_robotiq.cpp" -o "$TEST_DIR/test"
python "$HERE/diag/test_robotiq.py" "$TEST_DIR/test"
