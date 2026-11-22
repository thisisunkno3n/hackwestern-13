#!/bin/sh
# Seeker demo driver for checkpoint 6 (slay fusion, FUSION LOST, supervisor restarts it) and
# checkpoint 7 (full demo). Checklist: CHECKPOINTS-6-7.md. Works from any directory; runs in
# drafts/pi-lane. POSIX sh: QNX 8.0 sh (pdksh) and Linux sh.
#
#   scripts/run_demo.sh preflight              # checkpoint 7 pre-check, changes nothing
#   scripts/run_demo.sh start [--bg] [--sim] [--video-sim] [--no-ingest]
#                                              # ws_ingest.py + seeker_supervisor (+ sims)
#   scripts/run_demo.sh chaos                  # checkpoint 6: slay hud_fusion, verify recovery
#   scripts/run_demo.sh video stop|start|pause|resume
#                                              # video stand-in: NO VIDEO / VIDEO LOST checks
#   scripts/run_demo.sh status                 # pids and pidin priorities
#   scripts/run_demo.sh stop                   # stops everything `start` launched
#
# `start` runs the supervisor in the foreground (Ctrl+C stops it all) unless --bg is given.
# --sim feeds R1 and R2 from Daniel's tools/sim/remote.py (rehearsal without phones).
# --video-sim streams scripts/video_send.py (test card, or VIDEO_JPEG files) to hud_display on
# TCP $VIDEO_PORT as node $VIEW_NODE, standing in until Daniel's relay forwards phone video.
# hud_display runs the IF-7 wearer view by default: video full screen with a radar inset, and the
# full radar under NO VIDEO / VIDEO LOST whenever no fresh frame arrives.
#
# Env: HW13       hackwestern-13 checkout (default ~/hackwestern-13, else ../../../hackwestern-13)
#      CERT, KEY  TLS files for iPhone Safari (default $HW13/cert.pem, $HW13/key.pem)
#      WS_PORT    relay port (8080)
#      PHONE_HOST host in the printed phone links (qnxpi-felix.local; raw IPs do not load on the
#                 iPhone hotspot)
#      BRIDGE     seeker (default: ws_ingest.py through scripts/seeker_bridge.py, which turns the
#                 view node's video on and links to hud_display at once) or daniel (as is)
#      POSES      pinned phone pose file for hud_fusion (config/poses; "none" = no file)
#      POSE       extra pins "R1=x_m,y_m,yaw_deg R2=..." (beat POSES; no spaces inside one pin)
#      BACKGROUND wearer (default) or radar: hud_display --background
#      VIEW_NODE  phone shown full screen (R1): hud_display --view-node, video_send --node
#      VIDEO_PORT TCP port for IF-5.6 video (47106): hud_display, ws_ingest and video_send
#      VIDEO_SIZE, VIDEO_FPS  stand-in test card size and rate (1280x720, 15)
#      VIDEO_JPEG stand-in JPEG files to loop instead of the test card ("a.jpg b.jpg")
#      HUD_ARGS   extra hud_display words, e.g. "--mirror h"
#      FUSION_ARGS extra hud_fusion words
#      RESTART_MS first restart delay (1500 so FUSION LOST is visible; IF-6 default is 500)
#      PYTHON (python3), SEEKER_BUILD (build), SEEKER_CONFIG (config/seeker.json), SEEKER_LOGS (logs)
set -u
case "$0" in
  */*) cd "${0%/*}/.." || exit 2 ;;
  *) cd .. || exit 2 ;;
esac
B=${SEEKER_BUILD:-build}
CFG=${SEEKER_CONFIG:-config/seeker.json}
LOGS=${SEEKER_LOGS:-logs}
PY=${PYTHON:-python3}
WS_PORT=${WS_PORT:-8080}
PHONE_HOST=${PHONE_HOST:-qnxpi-felix.local}
BRIDGE=${BRIDGE:-seeker}
RESTART_MS=${RESTART_MS:-1500}
if [ -z "${HW13:-}" ]; then
  if [ -d "$HOME/hackwestern-13" ]; then HW13=$HOME/hackwestern-13; else HW13=../../../hackwestern-13; fi
fi
INGEST=$HW13/remote/ws_ingest.py
SIM=$HW13/tools/sim/remote.py
CERT=${CERT:-$HW13/cert.pem}
KEY=${KEY:-$HW13/key.pem}
POSES=${POSES:-config/poses}
POSE=${POSE:-}
BACKGROUND=${BACKGROUND:-wearer}
VIEW_NODE=${VIEW_NODE:-R1}
VIDEO_PORT=${VIDEO_PORT:-47106}
VIDEO_SIZE=${VIDEO_SIZE:-1280x720}
VIDEO_FPS=${VIDEO_FPS:-15}
VIDEO_JPEG=${VIDEO_JPEG:-}
VSEND=scripts/video_send.py
export SEEKER_BUILD="$B" SEEKER_CONFIG="$CFG" SEEKER_LOGS="$LOGS" POSES POSE
export BACKGROUND VIEW_NODE VIDEO_PORT

FAILS=0
pass() { echo "PASS $*"; }
fail() { echo "FAIL $*"; FAILS=$((FAILS + 1)); }
note() { echo "     $*"; }
have() { command -v "$1" >/dev/null 2>&1; }
is_qnx() { [ "$(uname -s)" = "QNX" ]; }

pid_of() { # pid_of <name>: pid from $LOGS/<name>.pid if that process is alive
  [ -f "$LOGS/$1.pid" ] || return 1
  P=$(cat "$LOGS/$1.pid" 2>/dev/null)
  [ -n "$P" ] && kill -0 "$P" 2>/dev/null && { echo "$P"; return 0; }
  return 1
}

stop_pidfile() { # stop_pidfile <name> [seconds]: SIGTERM, wait, then SIGKILL
  P=$(pid_of "$1") || { rm -f "$LOGS/$1.pid"; return 0; }
  kill -CONT "$P" 2>/dev/null # in case `video pause` stopped it
  kill -TERM "$P" 2>/dev/null
  N=0
  while kill -0 "$P" 2>/dev/null && [ "$N" -lt "${2:-5}" ]; do sleep 1; N=$((N + 1)); done
  if kill -0 "$P" 2>/dev/null; then
    echo "$1 pid $P ignored SIGTERM; sending SIGKILL"
    kill -KILL "$P" 2>/dev/null
  fi
  rm -f "$LOGS/$1.pid"
  echo "stopped $1 (pid $P)"
}

show_prio() { # show_prio <name>: QNX pidin priority column (e.g. 50f), else ps
  if is_qnx && have pidin; then
    pidin -p "$1" 2>/dev/null
  elif P=$(pid_of "$1"); then
    ps -o pid,cls,rtprio,ni,comm -p "$P" 2>/dev/null || ps -p "$P"
  fi
}

sv_check() { # seeker_supervisor --check with the same arguments the wrapper passes
  set -- --bin-dir "$B" --config "$CFG" --log-dir - --check
  [ "$POSES" != none ] && set -- "$@" --poses "$POSES"
  for a in $POSE; do set -- "$@" --pose "$a"; done
  set -- "$@" --background "$BACKGROUND" --view-node "$VIEW_NODE" --video-port "$VIDEO_PORT"
  "$B/seeker_supervisor" "$@"
}

ingest_has_video() {
  [ -f "$INGEST" ] && "$PY" "$INGEST" --help 2>&1 | grep -q -- --video-port
}

start_video_sim() {
  set -- "$PY" "$VSEND" --host 127.0.0.1 --port "$VIDEO_PORT" --node "$VIEW_NODE" --fps "$VIDEO_FPS"
  if [ -n "$VIDEO_JPEG" ]; then
    set -- "$@" --jpeg
    for f in $VIDEO_JPEG; do set -- "$@" "$f"; done
  else
    set -- "$@" --size "$VIDEO_SIZE"
  fi
  start_bg video_send "$@"
}

pose_for() { # pose_for <ID>: x_m,y_m,yaw_deg from POSE, else POSES, else empty
  for a in $POSE; do
    case "$a" in "$1="*) echo "${a#*=}"; return 0 ;; esac
  done
  [ "$POSES" != none ] && [ -f "$POSES" ] \
    && sed -e 's/#.*//' "$POSES" | tr -d ' \t' | sed -n "s/^$1=//p" | tail -1
}

phone_links() {
  echo "  phones: https://$PHONE_HOST:$WS_PORT/?node=$VIEW_NODE&pose=0,0,0&heading=1&video=1"
  echo "          https://$PHONE_HOST:$WS_PORT/?node=R2&pose=<x>,<y>,<yaw>"
}

cmd_preflight() {
  echo "== checkpoint 7 preflight ($(uname -s), in $(pwd))"
  for x in seeker_supervisor hud_fusion hud_display; do
    if [ -x "$B/$x" ]; then pass "$B/$x built"; else fail "$B/$x missing (scripts/build_pi.sh)"; fi
  done
  if [ -f "$CFG" ]; then pass "$CFG present"; else fail "$CFG missing"; fi
  if [ -x "$B/seeker_supervisor" ]; then
    if OUT=$(sv_check 2>&1); then
      pass "supervisor arguments valid (POSES=$POSES${POSE:+, POSE=$POSE}, $BACKGROUND view $VIEW_NODE, video tcp/$VIDEO_PORT)"
      echo "$OUT" | sed 's/^/     /'
      echo "$OUT" | grep -q '^pose ' \
        || note "no pinned poses: main's relay sends pose 0,0,0, which puts both phones on the wearer's mark"
    else
      fail "supervisor argument check: $OUT"
    fi
  fi
  if have "$PY"; then pass "$PY: $("$PY" --version 2>&1)"; else fail "$PY not found"; fi
  if [ -f "$INGEST" ]; then
    pass "relay $INGEST"
    if ingest_has_video; then
      pass "ws_ingest.py has --video-port: phone video goes to tcp/$VIDEO_PORT"
    else
      note "ws_ingest.py has no --video-port yet (Daniel's lane): use start --video-sim, or the"
      note "wearer view shows the full radar under NO VIDEO"
    fi
  else
    fail "no $INGEST (set HW13 to the hackwestern-13 checkout)"
  fi
  if [ -f "$VSEND" ]; then pass "video stand-in $VSEND"; else fail "missing $VSEND"; fi
  if [ -f "$CERT" ] && [ -f "$KEY" ]; then
    pass "TLS cert and key ($CERT, $KEY)"
  else
    fail "no TLS cert/key at $CERT / $KEY: iPhone Safari blocks the camera on plain http"
  fi
  if P=$(pid_of seeker_supervisor); then
    note "seeker_supervisor already running (pid $P)"
  elif P=$(pid_of hud_supervisor); then
    note "hud_supervisor is running (pid $P); run only one supervisor"
  fi
  if is_qnx; then
    for x in pidin slay; do
      if have "$x"; then pass "$x available"; else fail "$x missing"; fi
    done
  fi
  echo "   phone URLs once started (accept the self-signed cert once per phone):"
  phone_links
  echo "== $FAILS problem(s)"
  [ "$FAILS" -eq 0 ]
}

start_bg() { # start_bg <name> <cmd...>: background, stdin /dev/null, output to $LOGS/<name>.log
  NAME=$1; shift
  if have nohup; then
    nohup "$@" </dev/null >>"$LOGS/$NAME.log" 2>&1 &
  else
    "$@" </dev/null >>"$LOGS/$NAME.log" 2>&1 &
  fi
  echo $! >"$LOGS/$NAME.pid"
  echo "started $NAME pid $! (log $LOGS/$NAME.log)"
}

stop_extras() {
  stop_pidfile video_send 2
  stop_pidfile sim_R1 2
  stop_pidfile sim_R2 2
  stop_pidfile ws_ingest 3
}

cmd_start() {
  BG=0 USE_SIM=0 USE_INGEST=1 USE_VSIM=0
  for a in "$@"; do
    case "$a" in
      --bg) BG=1 ;;
      --sim) USE_SIM=1 ;;
      --video-sim) USE_VSIM=1 ;;
      --no-ingest) USE_INGEST=0 ;;
      *) echo "unknown start option $a" >&2; exit 2 ;;
    esac
  done
  mkdir -p "$LOGS"
  if P=$(pid_of seeker_supervisor); then
    echo "already running (seeker_supervisor pid $P); scripts/run_demo.sh stop first" >&2
    exit 1
  fi
  if [ "$USE_INGEST" -eq 1 ]; then
    if [ -f "$INGEST" ] && have "$PY"; then
      set -- --fusion 127.0.0.1 --fusion-port 47103 --port "$WS_PORT"
      if [ -f "$CERT" ] && [ -f "$KEY" ]; then
        set -- "$@" --tls-cert "$CERT" --tls-key "$KEY"
      else
        echo "warning: no $CERT / $KEY; relay on plain http (iPhone camera will be blocked)"
      fi
      if ingest_has_video; then
        set -- "$@" --video-host 127.0.0.1 --video-port "$VIDEO_PORT" --view-node "$VIEW_NODE"
        [ "$USE_VSIM" -eq 1 ] \
          && echo "warning: relay and --video-sim both send video; hud_display keeps the newest connection"
      fi
      if [ "$BRIDGE" = seeker ] && [ -f scripts/seeker_bridge.py ]; then
        start_bg ws_ingest "$PY" -u scripts/seeker_bridge.py --ws-ingest "$INGEST" "$@"
      else
        start_bg ws_ingest "$PY" "$INGEST" "$@"
      fi
      phone_links
    else
      echo "warning: no $INGEST or $PY; starting without the phone relay"
    fi
  fi
  if [ "$USE_SIM" -eq 1 ]; then
    if [ -f "$SIM" ] && have "$PY"; then
      P1=$(pose_for R1)
      P2=$(pose_for R2)
      start_bg sim_R1 "$PY" "$SIM" --pi 127.0.0.1 --node R1 --pose="${P1:--2,3,145}"
      start_bg sim_R2 "$PY" "$SIM" --pi 127.0.0.1 --node R2 --pose="${P2:-2,3,215}"
    else
      echo "warning: --sim needs $SIM and $PY; no sims started"
    fi
  fi
  if [ "$USE_VSIM" -eq 1 ]; then
    if [ -f "$VSEND" ] && have "$PY"; then start_video_sim; else echo "warning: --video-sim needs $VSEND and $PY"; fi
  fi
  echo "hud_display: $BACKGROUND view, node $VIEW_NODE, video tcp/$VIDEO_PORT"

  if [ "$BG" -eq 1 ]; then
    # --ignore-hup: the stack survives the SSH session that started it.
    if have nohup; then
      nohup sh scripts/seeker_supervisor.sh --ignore-hup --backoff-min-ms "$RESTART_MS" \
        </dev/null >>"$LOGS/seeker_supervisor.console" 2>&1 &
    else
      sh scripts/seeker_supervisor.sh --ignore-hup --backoff-min-ms "$RESTART_MS" \
        </dev/null >>"$LOGS/seeker_supervisor.console" 2>&1 &
    fi
    sleep 2
    if pid_of seeker_supervisor >/dev/null; then
      echo "seeker_supervisor running in the background; log: tail -f $LOGS/seeker_supervisor.log"
      echo "stop with: sh scripts/run_demo.sh stop"
    else
      echo "seeker_supervisor did not start; see $LOGS/seeker_supervisor.console" >&2
      stop_extras
      exit 1
    fi
    return 0
  fi

  trap 'stop_extras; exit 0' INT TERM HUP
  echo "seeker_supervisor in the foreground; Ctrl+C stops everything"
  sh scripts/seeker_supervisor.sh --backoff-min-ms "$RESTART_MS"
  RC=$?
  stop_extras
  return $RC
}

cmd_chaos() {
  echo "== checkpoint 6: slay hud_fusion while the HUD keeps drawing"
  pid_of seeker_supervisor >/dev/null || { echo "seeker_supervisor is not running (scripts/run_demo.sh start)"; exit 2; }
  F0=$(pid_of hud_fusion) || { echo "hud_fusion is not running right now; try again in a second"; exit 2; }
  D0=$(pid_of hud_display) || { echo "hud_display is not running"; exit 2; }
  SLOG=$LOGS/seeker_supervisor.log
  N0=$(grep -c "started hud_fusion" "$SLOG")
  if have slay; then
    echo "   slay -f hud_fusion   (pid $F0, SIGTERM; -f: no prompt if several match)"
    slay -f hud_fusion >/dev/null 2>&1
  else
    echo "   kill -TERM $F0   (no slay on this system)"
    kill -TERM "$F0"
  fi
  echo "   watch HDMI now: FUSION LOST banner, video and radar keep drawing"
  N=0
  while [ "$N" -lt 15 ]; do
    sleep 1
    N=$((N + 1))
    [ "$(grep -c "started hud_fusion" "$SLOG")" -gt "$N0" ] && break
  done
  if grep -q "hud_fusion pid $F0 .*restart #" "$SLOG"; then
    pass "supervisor saw the slay: $(grep "hud_fusion pid $F0 " "$SLOG" | tail -1 | sed 's/^\[[^]]*\] //')"
  else
    fail "no exit line for hud_fusion pid $F0 in $SLOG"
  fi
  if [ "$(grep -c "started hud_fusion" "$SLOG")" -gt "$N0" ]; then
    F1=$(pid_of hud_fusion)
    pass "hud_fusion restarted as pid ${F1:-?}: $(grep "started hud_fusion" "$SLOG" | tail -1 | sed 's/.*(\(.*\))/\1/')"
  else
    fail "hud_fusion not restarted within 15 s"
  fi
  if D1=$(pid_of hud_display) && [ "$D1" = "$D0" ]; then
    pass "hud_display untouched (pid $D0 still running)"
  else
    fail "hud_display pid changed or died (was $D0, now ${D1:-none})"
  fi
  echo "   priorities (want hud_display 50f, hud_fusion 30r, seeker_supervisor 45f):"
  show_prio hud_display
  show_prio hud_fusion
  show_prio seeker_supervisor
  echo "== $FAILS failure(s). Visual checks are yours: banner appeared, then cleared; no freeze."
  [ "$FAILS" -eq 0 ]
}

cmd_video() {
  case "${1:-}" in
    start)
      if pid_of video_send >/dev/null; then echo "video_send already running"; return 0; fi
      mkdir -p "$LOGS"
      start_video_sim
      echo "expect the $VIEW_NODE test card full screen within about 1 s" ;;
    stop)
      stop_pidfile video_send 2
      echo "expect NO VIDEO and the full radar within about 0.5 s" ;;
    pause)
      P=$(pid_of video_send) || { echo "video_send is not running"; return 1; }
      kill -STOP "$P"
      echo "video_send pid $P paused (still connected): expect VIDEO STALE (dimmed) at 0.5 s, VIDEO LOST at about 2 s" ;;
    resume)
      P=$(pid_of video_send) || { echo "video_send is not running"; return 1; }
      kill -CONT "$P"
      echo "video_send pid $P resumed: expect video back" ;;
    *) echo "usage: $0 video stop|start|pause|resume" >&2; return 2 ;;
  esac
}

cmd_status() {
  for x in seeker_supervisor hud_fusion hud_display ws_ingest sim_R1 sim_R2 video_send; do
    if P=$(pid_of "$x"); then echo "$x: pid $P"; else echo "$x: not running"; fi
  done
  if is_qnx && have pidin; then
    for x in seeker_supervisor hud_fusion hud_display; do pidin -p "$x" 2>/dev/null; done
  fi
  [ -f "$LOGS/seeker_supervisor.log" ] && grep -e "restart #" -e started -e refused -e "status:" "$LOGS/seeker_supervisor.log" | tail -5
  [ -f "$LOGS/hud_display.log" ] && grep -e "video:" "$LOGS/hud_display.log" | tail -3
  return 0
}

cmd_stop() {
  stop_pidfile seeker_supervisor 5
  stop_extras
}

CMD=${1:-start}
[ $# -gt 0 ] && shift
case "$CMD" in
  preflight) cmd_preflight ;;
  start) cmd_start "$@" ;;
  chaos) cmd_chaos ;;
  video) cmd_video "$@" ;;
  status) cmd_status ;;
  stop) cmd_stop ;;
  *) echo "usage: $0 preflight | start [--bg] [--sim] [--video-sim] [--no-ingest] | chaos |" \
       "video stop|start|pause|resume | status | stop" >&2; exit 2 ;;
esac
