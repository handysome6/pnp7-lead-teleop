#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
: "${CONDA_PREFIX:?Run through the inference/ros Pixi environment}"
# Catkin's generated setup probes unset shell variables (for example ZSH_VERSION).
set +u
source devel/setup.bash
set -u
export ROS_MASTER_URI=http://127.0.0.1:11311
export ROS_IP=127.0.0.1
mode="${1:-shadow}"
shift || true
profile=smoke
duration=5
if [[ "$mode" == validate ]]; then
  mode=live
  profile=full
  duration=40
fi
if [[ "$mode" != shadow && "$mode" != live ]]; then
  echo 'usage: bash run_client.sh [shadow|live|validate]' >&2
  exit 2
fi
# Live's mandatory Home preflight prepares realtime threads after reconnection.
run_stamp="$(date +%Y%m%d_%H%M%S)"
exec python ../robot_s0_policy.py --mode "$mode" --profile "$profile" --duration "$duration" \
  --log "validation/${mode}_${profile}_${run_stamp}_$$.json" "$@"
