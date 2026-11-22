#!/bin/sh
# Wrapper for build/seeker_supervisor: changes to drafts/pi-lane (so the children find
# config/ and logs/), checks the binaries, refuses to start a second copy, then execs the
# supervisor so Ctrl+C and `slay seeker_supervisor` reach it directly.
# POSIX sh: QNX 8.0 sh (pdksh) and Linux sh. Extra arguments go to seeker_supervisor.
#
#   scripts/seeker_supervisor.sh                          # fusion 30r, display 50f, logs/
#   HUD_ARGS="--background wearer --view-node R1" scripts/seeker_supervisor.sh
#   scripts/seeker_supervisor.sh --backoff-min-ms 1500 --hang-ms 1000
#   POSE="R1=-1.5,2.8,150 R2=2,3,215" scripts/seeker_supervisor.sh
#
# Env: SEEKER_BUILD (build), SEEKER_CONFIG (config/seeker.json), SEEKER_LOGS (logs),
#      POSES  pinned phone pose file for hud_fusion (config/poses; "none" = no file),
#      POSE   extra pins "ID=x_m,y_m,yaw_deg ..." (no spaces inside one pin; beat POSES),
#      BACKGROUND (radar|wearer), VIEW_NODE (R1), VIDEO_PORT (47106): hud_display
#             --background, --view-node, --video-port when set (unset = config / built-in),
#      HUD_ARGS / FUSION_ARGS (extra words for hud_display / hud_fusion; split on spaces).
set -u
case "$0" in
  */*) cd "${0%/*}/.." || exit 2 ;;
  *) cd .. || exit 2 ;;
esac
B=${SEEKER_BUILD:-build}
CFG=${SEEKER_CONFIG:-config/seeker.json}
LOGS=${SEEKER_LOGS:-logs}

for x in seeker_supervisor hud_fusion hud_display; do
  [ -x "$B/$x" ] || { echo "missing $B/$x: build first (scripts/build_pi.sh, or cmake in build/)" >&2; exit 2; }
done
[ -f "$CFG" ] || { echo "missing $CFG" >&2; exit 2; }
mkdir -p "$LOGS"
if [ -f "$LOGS/seeker_supervisor.pid" ]; then
  OLD=$(cat "$LOGS/seeker_supervisor.pid" 2>/dev/null)
  if [ -n "$OLD" ] && kill -0 "$OLD" 2>/dev/null; then
    echo "seeker_supervisor already running (pid $OLD); stop it first: scripts/run_demo.sh stop" >&2
    exit 1
  fi
fi

POSES=${POSES:-config/poses}
if [ "$POSES" != none ]; then
  [ -f "$POSES" ] || { echo "missing pose file $POSES (copy config/poses.example, or POSES=none)" >&2; exit 2; }
  set -- "$@" --poses "$POSES"
fi
for a in ${POSE:-}; do set -- "$@" --pose "$a"; done
[ -n "${BACKGROUND:-}" ] && set -- "$@" --background "$BACKGROUND"
[ -n "${VIEW_NODE:-}" ] && set -- "$@" --view-node "$VIEW_NODE"
[ -n "${VIDEO_PORT:-}" ] && set -- "$@" --video-port "$VIDEO_PORT"
for a in ${HUD_ARGS:-}; do set -- "$@" --display-arg "$a"; done
for a in ${FUSION_ARGS:-}; do set -- "$@" --fusion-arg "$a"; done
exec "$B/seeker_supervisor" --bin-dir "$B" --config "$CFG" --log-dir "$LOGS" "$@"
