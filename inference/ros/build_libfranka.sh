#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
: "${CONDA_PREFIX:?Run through pixi}"
cmake -S vendor/libfranka-0.21.3 -B vendor/build-libfranka \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF \
  -DCMAKE_PREFIX_PATH="$CONDA_PREFIX" \
  -DCMAKE_INSTALL_PREFIX="$PWD/vendor/install" \
  -DCMAKE_INSTALL_RPATH="$CONDA_PREFIX/lib;$PWD/vendor/install/lib"
cmake --build vendor/build-libfranka -j4
cmake --install vendor/build-libfranka
