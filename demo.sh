#!/usr/bin/env bash
# Live demonstration: PNP-7 lead arm teleoperating the Franka, with both
# RealSense RGB streams and live teleop state on the robot PC's screen.
#
# Shows, in one window: what the robot sees from the external and wrist
# cameras, the teleop state machine, whether the dead-man is held, all seven
# measured joint angles, and the gripper opening.
#
#   ./demo.sh              # 5 minutes
#   DURATION=600 ./demo.sh # 10 minutes
#
# This COMMANDS THE ROBOT. Release the dead-man to freeze it; the E-stop
# remains the hardware backstop.
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONF="${CONF:-$HERE/conf/demo.conf}"
DURATION="${DURATION:-300}"
STATUS="${STATUS:-/tmp/pnp7_status.json}"
export DISPLAY="${DISPLAY:-:0}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"

cleanup() {
  # Bracketed pattern so this never matches its own command line.
  pkill -f "view_camer[a]s.py" 2>/dev/null
  rm -f "$STATUS"
}
trap cleanup EXIT INT TERM

echo "=============================================="
echo " PNP-7  ->  Franka   teleoperation demo"
echo "=============================================="
echo "config   : $CONF"
echo "duration : ${DURATION}s"
echo "display  : $DISPLAY"
echo

echo "--- pre-flight ---"
if ! "$HERE/.venv/bin/python" "$HERE/check_ready.py" --config "$CONF"; then
  echo
  echo "Pre-flight failed. Fix the FAIL rows above before demonstrating."
  exit 1
fi

echo
echo "--- lead arm / robot correspondence ---"
"$HERE/.venv/bin/python" "$HERE/calib/check_correspondence.py" 2>&1 | tail -12 || true

echo
echo "--- opening camera view on the robot screen ---"
rm -f "$STATUS"
setsid nohup "$HERE/.venv/bin/python" "$HERE/collect/view_cameras.py" \
  --status "$STATUS" >/tmp/demo_viewer.log 2>&1 </dev/null &

for _ in $(seq 1 40); do
  pgrep -f "view_camer[a]s.py" >/dev/null && break
  sleep 0.25
done
if ! pgrep -f "view_camer[a]s.py" >/dev/null; then
  echo "the viewer did not start:"
  tail -20 /tmp/demo_viewer.log
  exit 1
fi
sleep 2
echo "camera view is up"

cat <<'BANNER'

==============================================
 HOW TO DRIVE IT
==============================================
  hold the foot pedal down          enable
  lift your foot                    freeze in place
  move the lead arm                 the Franka follows 1:1
  squeeze the lead trigger          close the hand
  Ctrl-C                            end the demo

  The on-screen strip shows the state machine,
  the dead-man, all seven joint angles and the
  gripper opening, live.
==============================================

BANNER

"$HERE/bin/pnp7_teleop" robot "$CONF" "$DURATION" /tmp/demo_run.csv
RC=$?

echo
echo "--- demo finished (exit $RC) ---"
if [ -s /tmp/demo_run.csv ]; then
  "$HERE/.venv/bin/python" "$HERE/diag/analyze_run.py" /tmp/demo_run.csv \
    --conf "$CONF" 2>&1 | tail -22
fi
