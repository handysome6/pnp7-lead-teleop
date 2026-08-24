#!/usr/bin/env bash
# Collect several episodes in one sitting, validating each as it lands.
#
# Pauses between episodes so the scene can be reset. A failed validation stops
# the batch rather than quietly accumulating unusable data.
#
#   COUNT=5 CONF=full100b.conf DURATION=60 ./collect_batch.sh episodes ep
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="${1:?usage: collect_batch.sh <episodes-dir> [prefix]}"
PREFIX="${2:-ep}"
# COUNT  = how many MORE to record.
# TARGET = how many should exist in total when finished; takes precedence.
# TARGET is the safer form across sessions: restarting a COUNT=5 batch when 5
# already exist would collect five more, whereas TARGET=5 stops immediately.
COUNT="${COUNT:-3}"
TARGET="${TARGET:-0}"
CONF="${CONF:-$HERE/full100b.conf}"
DURATION="${DURATION:-60}"

mkdir -p "$ROOT"

# Continue numbering from whatever is already there.
next=1
while [ -d "$ROOT/$(printf '%s%03d' "$PREFIX" "$next")" ]; do
  next=$((next + 1))
done
existing=$((next - 1))

if [ "$TARGET" -gt 0 ]; then
  COUNT=$((TARGET - existing))
  if [ "$COUNT" -le 0 ]; then
    echo "$existing episode(s) already in $ROOT, target is $TARGET."
    echo "Nothing to do."
    exit 0
  fi
fi

echo "config   : $CONF"
echo "duration : ${DURATION}s"
echo "existing : $existing episode(s)"
echo "to record: $COUNT, starting at $(printf '%s%03d' "$PREFIX" "$next")"
[ "$TARGET" -gt 0 ] && echo "target   : $TARGET total"
echo

ok=0; bad=0
for i in $(seq 1 "$COUNT"); do
  name=$(printf '%s%03d' "$PREFIX" "$next")
  echo "=================================================="
  echo " episode $i of $COUNT -> $name"
  echo "=================================================="
  read -r -p "reset the scene, then press Enter to start (q to stop): " reply
  [ "$reply" = "q" ] && { echo "stopped by operator"; break; }

  CONF="$CONF" DURATION="$DURATION" "$HERE/collect_episode.sh" "$ROOT/$name"

  echo
  echo "--- validating $name ---"
  if "$HERE/.venv/bin/python" "$HERE/validate_episode.py" "$ROOT/$name"; then
    ok=$((ok + 1))
  else
    bad=$((bad + 1))
    echo
    echo "$name did not pass validation. Fix the cause before continuing;"
    echo "collecting more episodes with the same fault only multiplies it."
    break
  fi
  next=$((next + 1))
  echo
done

echo "=================================================="
total=$(ls -1d "$ROOT"/${PREFIX}* 2>/dev/null | wc -l)
echo "batch done: $ok passed, $bad failed, $total episode(s) in $ROOT"
ls -1d "$ROOT"/${PREFIX}* 2>/dev/null | tail -10
