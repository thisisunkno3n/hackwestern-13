#!/bin/sh
# Stops what scripts/run-live.sh started (bridge, hud_fusion, hud_display), by pid file first,
# then strays started by hand: hud_fusion and hud_display with `slay -f` on QNX (-f: no prompt
# when several match), and any python ws_ingest.py / seeker_bridge.py holding the phone port.
# POSIX sh: QNX 8.0 sh (pdksh) and Linux sh.
#
#   sh scripts/stop-live.sh            # LOG_DIR (~/seeker-logs) holds the pid files
set -u
LOG_DIR=${LOG_DIR:-$HOME/seeker-logs}

have() { command -v "$1" >/dev/null 2>&1; }

stop_pid() { # stop_pid <pid> <label>: SIGTERM, up to 5 s, then SIGKILL
  kill -0 "$1" 2>/dev/null || return 0
  kill -TERM "$1" 2>/dev/null
  N=0
  while kill -0 "$1" 2>/dev/null && [ "$N" -lt 5 ]; do
    sleep 1
    N=$((N + 1))
  done
  kill -0 "$1" 2>/dev/null && kill -KILL "$1" 2>/dev/null
  echo "stop-live: stopped $2 (pid $1)"
}

for NAME in bridge hud_fusion hud_display; do
  F=$LOG_DIR/$NAME.pid
  [ -f "$F" ] || continue
  P=$(cat "$F" 2>/dev/null)
  [ -n "$P" ] && stop_pid "$P" "$NAME"
  rm -f "$F"
done

# Strays: copies started by hand in other windows.
if have slay; then
  for X in hud_fusion hud_display; do
    slay -f "$X" >/dev/null 2>&1 && echo "stop-live: slay -f $X"
  done
else
  for X in hud_fusion hud_display; do
    have pkill && pkill -TERM -x "$X" 2>/dev/null && echo "stop-live: pkill $X"
  done
fi
if have pidin; then
  PIDS=$(pidin arg 2>/dev/null | grep -e ws_ingest.py -e seeker_bridge.py | grep -v grep | awk '{print $1}')
else
  PIDS=$(ps -eo pid=,args= 2>/dev/null | grep -e ws_ingest.py -e seeker_bridge.py | grep -v grep | awk '{print $1}')
fi
for P in $PIDS; do
  [ "$P" = "$$" ] || stop_pid "$P" "phone bridge"
done
exit 0
