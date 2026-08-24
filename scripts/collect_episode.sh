#!/usr/bin/env bash
# Record one teleoperation episode: cameras + bridge + joined dataset.
#
# The cameras are started first and given time to reach steady state, because
# RealSense auto-exposure settles over the first frames and those frames are not
# worth training on. The bridge starts only after CAMERAS_READY.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
CONF="${CONF:-$REPO/conf/full50.conf}"
DURATION="${DURATION:-60}"
# MODE=dry exercises the whole pipeline without commanding the robot.
MODE="${MODE:-robot}"
# PREVIEW=1 shows the frames as they are recorded, so the operator can see the
# scene leaving frame during a take rather than discovering it afterwards.
PREVIEW="${PREVIEW:-1}"
# EVENTS=0 可关闭事件相机。EVK4 必须在容器内采集（OpenEB 5.3 要求
# Ubuntu 22.04，本机是 20.04），容器与宿主共享内核，所以 CLOCK_MONOTONIC
# 时间戳与 RealSense、机器人状态完全可比。
EVENTS="${EVENTS:-1}"
EVENT_IMAGE="${EVENT_IMAGE:-metavision:5.3.0}"
HOT_PIXELS="${HOT_PIXELS:-$HOME/metavision/hot_pixels.txt}"
OUT="${1:?usage: collect_episode.sh <episode-dir> }"

mkdir -p "$OUT"
cp "$CONF" "$OUT/config.conf"
cp "$REPO/calibration.json" "$OUT/calibration.json" 2>/dev/null || true

# Correspondence pre-flight. Relative mapping only transports deltas, so the
# lead arm can drift into a different configuration from the robot while
# disengaged. That does not break the mapping, but it makes downstream joints
# look mirrored, and demonstrations recorded that way are awkward to reproduce.
# Warn, do not block: a deliberately odd lead posture is sometimes wanted.
echo "=== correspondence check ==="
"$REPO/.venv/bin/python" "$REPO/calib/check_correspondence.py" \
  --calibration "$REPO/calibration.json" 2>&1 | tail -12 || true
echo

echo "=== starting cameras ==="
PREVIEW_FLAG=""
if [ "$PREVIEW" = "1" ]; then
  PREVIEW_FLAG="--preview"
  export DISPLAY="${DISPLAY:-:0}"
  export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"
fi
"$REPO/.venv/bin/python" "$REPO/collect/record_cameras.py" \
  --out "$OUT" --duration "$((DURATION + 6))" $PREVIEW_FLAG \
  > "$OUT/cameras.log" 2>&1 &
CAM_PID=$!

for _ in $(seq 1 60); do
  grep -q CAMERAS_READY "$OUT/cameras.log" 2>/dev/null && break
  kill -0 "$CAM_PID" 2>/dev/null || { echo "camera recorder died"; cat "$OUT/cameras.log"; exit 1; }
  sleep 0.5
done
grep -q CAMERAS_READY "$OUT/cameras.log" || { echo "cameras never became ready"; cat "$OUT/cameras.log"; exit 1; }
echo "cameras ready"

if [ "$EVENTS" = "1" ]; then
  echo "=== starting event camera ==="
  rm -f "$OUT/events.hdf5" "$OUT/events_meta.json"
  docker rm -f pnp7_events >/dev/null 2>&1 || true
  docker run -d --name pnp7_events --privileged \
    -v /dev/bus/usb:/dev/bus/usb \
    -v "$HOME/metavision:/work" \
    -v "$(cd "$OUT" && pwd):/episode" \
    -w /work "$EVENT_IMAGE" \
    python3 record_events.py --out /episode \
      --duration "$((DURATION + 4))" \
      --hot-pixels "/work/$(basename "$HOT_PIXELS")" \
    > /dev/null

  for _ in $(seq 1 60); do
    docker logs pnp7_events 2>&1 | grep -q EVENTS_READY && break
    docker ps --filter name=pnp7_events --format "{{.Names}}" | grep -q pnp7_events \
      || { echo "事件采集启动失败:"; docker logs pnp7_events 2>&1 | tail -10; exit 1; }
    sleep 0.5
  done
  docker logs pnp7_events 2>&1 | grep -q EVENTS_READY \
    || { echo "事件相机未就绪"; docker logs pnp7_events 2>&1 | tail -10; exit 1; }
  echo "event camera ready"
fi

echo "=== starting teleop bridge ($MODE) ==="
# Not fatal on its own: a Ctrl-C is a legitimate way to end a take, and the
# cameras still need to be reaped. The real test is whether a usable log landed.
"$REPO/bin/pnp7_teleop" "$MODE" "$CONF" "$DURATION" "$OUT/teleop.csv"
BRIDGE_RC=$?

echo "=== waiting for cameras to finish ==="
wait "$CAM_PID" || true
tail -4 "$OUT/cameras.log"

if [ "$EVENTS" = "1" ]; then
  echo "=== waiting for event camera ==="
  docker wait pnp7_events >/dev/null 2>&1 || true
  docker logs pnp7_events 2>&1 | grep -vE "^EVENTS_READY$" | tail -4
  docker rm -f pnp7_events >/dev/null 2>&1 || true
  if [ ! -s "$OUT/events.hdf5" ]; then
    echo "警告: 未写出事件文件，本 episode 只有 RGB。"
  fi
fi

# Without robot state the frames are unusable, and leaving them behind wastes
# hundreds of MB and looks like a real episode later. Fail loudly instead.
if [ ! -s "$OUT/teleop.csv" ]; then
  echo
  echo "ERROR: the bridge wrote no teleop log (exit $BRIDGE_RC)."
  echo "The camera frames in $OUT have no robot state to pair with and are"
  echo "unusable. Check the bridge output above -- a refused preflight"
  echo "(robot not kIdle, brakes locked, STO engaged) is the usual cause."
  echo
  echo "Remove it with:  rm -rf $OUT"
  exit 1
fi

ROWS=$(( $(wc -l < "$OUT/teleop.csv") - 1 ))
if [ "$ROWS" -lt 1000 ]; then
  echo
  echo "ERROR: teleop log has only $ROWS rows; the session ended almost"
  echo "immediately. Not building an episode from it."
  echo "Remove it with:  rm -rf $OUT"
  exit 1
fi

echo "=== building episode ==="
"$REPO/.venv/bin/python" "$REPO/collect/build_episode.py" --episode "$OUT"
