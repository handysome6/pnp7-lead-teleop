"""把事件相机接进 collect_episode.sh。"""
p = "/home/franka/workspace/pnp7_teleop/collect_episode.sh"
s = open(p).read()

s = s.replace(
    'PREVIEW="${PREVIEW:-1}"',
    'PREVIEW="${PREVIEW:-1}"\n'
    '# EVENTS=0 可关闭事件相机。EVK4 必须在容器内采集（OpenEB 5.3 要求\n'
    '# Ubuntu 22.04，本机是 20.04），容器与宿主共享内核，所以 CLOCK_MONOTONIC\n'
    '# 时间戳与 RealSense、机器人状态完全可比。\n'
    'EVENTS="${EVENTS:-1}"\n'
    'EVENT_IMAGE="${EVENT_IMAGE:-metavision:5.3.0}"\n'
    'HOT_PIXELS="${HOT_PIXELS:-$HOME/metavision/hot_pixels.txt}"',
)

old_bridge = '''echo "=== starting teleop bridge ($MODE) ==="'''
new_bridge = '''if [ "$EVENTS" = "1" ]; then
  echo "=== starting event camera ==="
  rm -f "$OUT/events.hdf5" "$OUT/events_meta.json"
  docker rm -f pnp7_events >/dev/null 2>&1 || true
  docker run -d --name pnp7_events --privileged \\
    -v /dev/bus/usb:/dev/bus/usb \\
    -v "$HOME/metavision:/work" \\
    -v "$(cd "$OUT" && pwd):/episode" \\
    -w /work "$EVENT_IMAGE" \\
    python3 record_events.py --out /episode \\
      --duration "$((DURATION + 4))" \\
      --hot-pixels "/work/$(basename "$HOT_PIXELS")" \\
    > /dev/null

  for _ in $(seq 1 60); do
    docker logs pnp7_events 2>&1 | grep -q EVENTS_READY && break
    docker ps --filter name=pnp7_events --format "{{.Names}}" | grep -q pnp7_events \\
      || { echo "事件采集启动失败:"; docker logs pnp7_events 2>&1 | tail -10; exit 1; }
    sleep 0.5
  done
  docker logs pnp7_events 2>&1 | grep -q EVENTS_READY \\
    || { echo "事件相机未就绪"; docker logs pnp7_events 2>&1 | tail -10; exit 1; }
  echo "event camera ready"
fi

echo "=== starting teleop bridge ($MODE) ==="'''
assert old_bridge in s
s = s.replace(old_bridge, new_bridge, 1)

old_wait = '''echo "=== waiting for cameras to finish ==="
wait "$CAM_PID" || true
tail -4 "$OUT/cameras.log"'''
new_wait = '''echo "=== waiting for cameras to finish ==="
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
fi'''
assert old_wait in s
s = s.replace(old_wait, new_wait, 1)

open(p, "w").write(s)
print("collect_episode.sh 已接入事件相机")
