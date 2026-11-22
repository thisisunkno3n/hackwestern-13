#!/bin/sh
# Integration test for seeker_supervisor: backoff on a crash loop, backoff reset after stable
# runs, real hud_fusion + hud_display with slay/kill/freeze of fusion, clean stop on SIGINT and
# SIGTERM (including a child that ignores SIGTERM), pinned poses passed to hud_fusion, argument
# and pose errors, --check, and (with python3) phone video from scripts/video_send.py surviving a
# hud_display crash and restart.
# POSIX sh: QNX 8.0 sh (pdksh) and Linux sh. Run from drafts/pi-lane:
#
#   sh tests/test_supervisor.sh [build-dir]       # default build/
#
# Binds UDP 47103 and 47102 and recreates /seeker_health: stop any running demo first
# (sh scripts/stop-live.sh). Section 3 checks the two ports first and says so if they are taken.
# On a host this is only a pre-filter; it counts once it passes on the QNX Pi.
# shellcheck disable=SC2015,SC2016  # pass/fail always return 0; wait_for evals its condition
set -u
B=${1:-build}
SV=$B/seeker_supervisor
T=logs/test_supervisor
FAILS=0
SH=$(command -v sh)

pass() { echo "PASS $*"; }
fail() { echo "FAIL $*"; FAILS=$((FAILS + 1)); }
check() { # check <file> <pattern> <description>
  if grep -q -e "$2" "$1"; then pass "$3"; else fail "$3 (no '$2' in $1)"; fi
}
count() { C=$(grep -c -e "$2" "$1" 2>/dev/null); echo "${C:-0}"; }
alive() { kill -0 "$1" 2>/dev/null; }

# Sub-second polling where sleep accepts fractions; whole seconds otherwise.
if sleep 0.1 2>/dev/null; then STEP=0.2 PER_S=5; else STEP=1 PER_S=1; fi
wait_for() { # wait_for <seconds> <shell condition>
  N=0
  while [ "$N" -lt $(($1 * PER_S)) ]; do
    eval "$2" && return 0
    sleep "$STEP"
    N=$((N + 1))
  done
  eval "$2"
}
signal_fusion() { # signal_fusion <TERM|KILL> <pid>: slay on QNX for TERM, as in checkpoint 6
  if [ "$1" = TERM ] && command -v slay >/dev/null 2>&1; then
    slay -f hud_fusion >/dev/null 2>&1
  else
    kill "-$1" "$2"
  fi
}
PY=${PYTHON:-python3}
# udp_busy <port>: 0 if another process already holds UDP <port>. A plain bind (no SO_REUSEADDR)
# fails then on QNX and Linux alike. On QNX (BSD sockets) our own SO_REUSEADDR bind fails too, so
# a demo left running makes the test's hud_fusion exit at once and restart in a loop; on Linux it
# would quietly share the port. Without python3 nothing is probed (returns 1).
udp_busy() {
  command -v "$PY" >/dev/null 2>&1 || return 1
  "$PY" -c 'import socket, sys
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    s.bind(("0.0.0.0", int(sys.argv[1])))
except OSError:
    sys.exit(0)
sys.exit(1)' "$1"
}
fusion_pid_not() { # fusion_pid_not <pid>: hud_fusion.pid holds a pid, and not <pid>
  P=$(cat "$T/real/hud_fusion.pid" 2>/dev/null)
  [ -n "$P" ] && [ "$P" != "$1" ]
}

for x in seeker_supervisor hud_fusion hud_display; do
  [ -x "$B/$x" ] || { echo "missing $B/$x (build first)"; exit 2; }
done
rm -rf "$T"
mkdir -p "$T"

echo "== 1. crash loop: exit 3 at once, backoff 100 -> 200 -> 400 -> 400 ms"
"$SV" --only fusion --fusion "$SH" --fusion-arg -c --fusion-arg 'exit 3' --no-health \
  --self-prio 0 --fusion-prio 0 --backoff-min-ms 100 --backoff-max-ms 400 --stable-ms 10000 \
  --log-dir "$T/loop" --status-s 0 --run-for 2.5 2>"$T/loop.err"
RC=$?
L=$T/loop/seeker_supervisor.log
[ "$RC" -eq 0 ] && pass "exit 0 after --run-for" || fail "exit $RC after --run-for"
check "$L" "hud_fusion pid [0-9]* exited with status 3 after" "logs the exit status"
check "$L" "restart #1 in 100 ms" "first restart after 100 ms"
check "$L" "restart #2 in 200 ms" "second restart after 200 ms"
check "$L" "restart #3 in 400 ms" "third restart after 400 ms"
check "$L" "restart #4 in 400 ms" "capped at 400 ms"
check "$L" "down [0-9]* ms" "start line reports downtime"
check "$L" "stopped cleanly" "clean stop after --run-for"
N=$(count "$L" "started hud_fusion")
if [ "$N" -ge 5 ] && [ "$N" -le 8 ]; then pass "$N starts in 2.5 s"; else fail "$N starts in 2.5 s (want 5..8)"; fi
[ ! -f "$T/loop/hud_fusion.pid" ] && [ ! -f "$T/loop/seeker_supervisor.pid" ] \
  && pass "pid files removed" || fail "pid files left behind"

echo "== 2. stable runs reset the backoff (child runs 1 s, stable after 800 ms)"
"$SV" --only fusion --fusion "$SH" --fusion-arg -c --fusion-arg 'sleep 1; exit 0' --no-health \
  --self-prio 0 --fusion-prio 0 --backoff-min-ms 100 --backoff-max-ms 2000 --stable-ms 800 \
  --log-dir "$T/stable" --status-s 0 --run-for 4.5 2>"$T/stable.err"
L=$T/stable/seeker_supervisor.log
N1=$(count "$L" "restart #[0-9]* in 100 ms")
NX=$(count "$L" "restart #[0-9]* in [0-9]* ms")
if [ "$NX" -ge 2 ] && [ "$N1" -eq "$NX" ]; then pass "all $NX restarts at 100 ms"; else fail "$N1 of $NX restarts at 100 ms"; fi

echo "== 3. real hud_fusion + hud_display: slay, SIGKILL and freeze fusion; HUD must not restart"
BUSY=""
for p in 47103 47102; do udp_busy "$p" && BUSY="$BUSY udp/$p"; done
if [ -n "$BUSY" ]; then
  fail "section 3 not run:$BUSY already held by another process (a live run or a hud_fusion /" \
    "hud_display started by hand). Stop it (sh scripts/stop-live.sh, or slay hud_fusion" \
    "hud_display on QNX) and run ctest again"
else
"$SV" --bin-dir "$B" --display-arg --sink --display-arg none --backoff-min-ms 300 \
  --poses config/poses.example --pose R2=2.5,3,210 \
  --backoff-max-ms 1200 --stable-ms 3000 --hang-ms 800 --status-s 1 --log-dir "$T/real" \
  2>"$T/real.err" &
SVPID=$!
L=$T/real/seeker_supervisor.log
FL=$T/real/hud_fusion.log
wait_for 8 '[ -s "$T/real/hud_fusion.pid" ] && [ -s "$T/real/hud_display.pid" ] &&
  grep -q "listening on udp/47103" "$FL" 2>/dev/null'
F0=$(cat "$T/real/hud_fusion.pid" 2>/dev/null)
D0=$(cat "$T/real/hud_display.pid" 2>/dev/null)
[ -n "$F0" ] && [ -n "$D0" ] && pass "both children started (fusion $F0, display $D0)" \
  || fail "children did not start"
if grep -q "cannot bind" "$FL" "$T/real/hud_display.log" 2>/dev/null; then
  fail "a child could not bind its port: $(grep -h "cannot bind" "$FL" "$T/real/hud_display.log" |
    sort -u | tr '\n' ' ')(another process holds it: sh scripts/stop-live.sh)"
fi
sleep 1
check "$L" "hud_fusion pose R1 -2.00,3.00,145.0 pinned from config/poses.example" "supervisor logs file pose"
check "$L" "hud_fusion pose R2 2.50,3.00,210.0 pinned from --pose" "supervisor logs --pose (beats the file)"
check "$T/real/hud_fusion.log" "R1 pose -2.00,3.00,145.0 (file, pinned)" "hud_fusion got --poses"
check "$T/real/hud_fusion.log" "R2 pose 2.50,3.00,210.0 (cli, pinned)" "hud_fusion got --pose"

# Each step waits for its own log line and a new pid, not for a count of starts: a start count
# is also met by unrelated restarts, and the checks then ran before the event they look for.
signal_fusion TERM "$F0"
wait_for 6 'grep -q "hud_fusion pid $F0 exited" "$L" && fusion_pid_not "$F0"'
check "$L" "hud_fusion pid $F0 exited with status 0 after" "SIGTERM/slay seen as a clean exit"
F1=$(cat "$T/real/hud_fusion.pid" 2>/dev/null)
[ -n "$F1" ] && [ "$F1" != "$F0" ] && pass "fusion back as pid $F1" || fail "fusion not restarted"
sleep 1

kill -KILL "$F1" 2>/dev/null
wait_for 6 'grep -q "hud_fusion pid $F1 killed" "$L" && fusion_pid_not "$F1"'
check "$L" "hud_fusion pid $F1 killed by SIGKILL (9)" "crash (SIGKILL) reported"
check "$L" "restart #2 in 600 ms" "backoff doubled for the quick second failure"
F2=$(cat "$T/real/hud_fusion.pid" 2>/dev/null)
sleep 1

kill -STOP "$F2" 2>/dev/null
wait_for 8 'grep -q "hud_fusion pid $F2 killed" "$L" && fusion_pid_not "$F2"'
check "$L" "hud_fusion pid $F2: no heartbeat for" "frozen fusion caught by --hang-ms"
check "$L" "hud_fusion pid $F2 killed by SIGKILL" "frozen fusion killed and reaped"
F3=$(cat "$T/real/hud_fusion.pid" 2>/dev/null)
fusion_pid_not "$F2" && pass "fusion restarted after the freeze (pid $F3)" \
  || fail "fusion not restarted after the freeze"
D1=$(cat "$T/real/hud_display.pid" 2>/dev/null)
if [ "$D1" = "$D0" ] && alive "$D0" && [ "$(count "$L" "started hud_display")" -eq 1 ]; then
  pass "hud_display pid $D0 never restarted"
else
  fail "hud_display restarted or died (was $D0, now ${D1:-none})"
fi
check "$L" "status: hud_fusion pid" "periodic status line"
check "$T/real/hud_fusion.log" "==== seeker_supervisor: start #4 of hud_fusion" "child log has start banners"

kill -INT "$SVPID"
wait "$SVPID"
RC=$?
[ "$RC" -eq 0 ] && pass "SIGINT (Ctrl+C): exit 0" || fail "SIGINT: exit $RC"
check "$L" "signal received; stopping children" "stop logged"
check "$L" "stopped cleanly" "clean stop"
LEFT=""
for p in "$F3" "$D0"; do alive "$p" && LEFT="$LEFT $p"; done
[ -z "$LEFT" ] && pass "no children left running" || fail "children still running:$LEFT"
[ ! -f "$T/real/hud_fusion.pid" ] && [ ! -f "$T/real/hud_display.pid" ] \
  && pass "child pid files removed" || fail "child pid files left behind"
if grep -q "hud_display pid $D0 exited with status 0" "$L"; then
  pass "hud_display stopped with its own SIGTERM handler (status 0)"
else
  fail "hud_display did not exit 0 on stop"
fi
fi

echo "== 4. SIGTERM stop with a child that ignores SIGTERM (SIGKILL after --stop-timeout-ms)"
"$SV" --only display --display "$SH" --display-arg -c \
  --display-arg 'trap "" TERM; while :; do sleep 1; done' --no-health --self-prio 0 \
  --display-prio 0 --stop-timeout-ms 500 --status-s 0 --log-dir "$T/stubborn" 2>"$T/stubborn.err" &
SVPID=$!
wait_for 5 '[ -f "$T/stubborn/hud_display.pid" ]'
DP=$(cat "$T/stubborn/hud_display.pid" 2>/dev/null)
sleep 1
kill -TERM "$SVPID"
wait "$SVPID"
RC=$?
L=$T/stubborn/seeker_supervisor.log
[ "$RC" -eq 0 ] && pass "SIGTERM: exit 0" || fail "SIGTERM: exit $RC"
check "$L" "hud_display pid $DP still running; sending SIGKILL" "escalates to SIGKILL"
check "$L" "hud_display pid $DP killed by SIGKILL" "stubborn child reaped"
alive "$DP" && fail "stubborn child $DP still alive" || pass "stubborn child gone"

echo "== 5. argument errors exit 2 before starting anything"
"$SV" --bin-dir "$T/nonexistent" --no-health --log-dir - 2>"$T/args.err"
[ $? -eq 2 ] && pass "missing binary: exit 2" || fail "missing binary: wrong exit status"
check "$T/args.err" "is not executable" "missing binary explained"
"$SV" --bin-dir "$B" --fusion-prio 50x --no-health --log-dir - 2>>"$T/args.err"
[ $? -eq 2 ] && pass "bad priority: exit 2" || fail "bad priority: wrong exit status"
"$SV" --bogus 2>>"$T/args.err"
[ $? -eq 2 ] && pass "unknown option: exit 2" || fail "unknown option: wrong exit status"
"$SV" --bin-dir "$B" --pose R1=1,2 --check >/dev/null 2>"$T/pose.err"
[ $? -eq 2 ] && pass "bad --pose: exit 2" || fail "bad --pose: wrong exit status"
check "$T/pose.err" "want three numbers" "bad --pose explained"
"$SV" --bin-dir "$B" --poses "$T/no-such-poses" --check >/dev/null 2>>"$T/pose.err"
[ $? -eq 2 ] && pass "missing --poses file: exit 2" || fail "missing --poses file: wrong exit status"
"$SV" --bin-dir "$B" --background video --check >/dev/null 2>>"$T/pose.err"
[ $? -eq 2 ] && pass "bad --background: exit 2" || fail "bad --background: wrong exit status"
"$SV" --bin-dir "$B" --video-port 70000 --check >/dev/null 2>>"$T/pose.err"
[ $? -eq 2 ] && pass "bad --video-port: exit 2" || fail "bad --video-port: wrong exit status"
"$SV" --bin-dir "$B" --view-node "R1;x" --check >/dev/null 2>>"$T/pose.err"
[ $? -eq 2 ] && pass "bad --view-node: exit 2" || fail "bad --view-node: wrong exit status"

echo "== 6. --check prints the command lines and poses without starting anything"
if "$SV" --bin-dir "$B" --config config/seeker.json --poses config/poses --pose "R2=1,1,180" \
  --display-arg --sink --display-arg none --background wearer --view-node R2 --video-port 47116 \
  --check >"$T/check.out" 2>&1; then
  pass "--check: exit 0"
else
  fail "--check: wrong exit status"
fi
check "$T/check.out" "^hud_fusion: $B/hud_fusion --poses config/poses --pose R2=1,1,180 --config config/seeker.json" \
  "fusion command line"
check "$T/check.out" \
  "^hud_display: $B/hud_display --sink none --background wearer --view-node R2 --video-port 47116 --config config/seeker.json" \
  "display command line with the video flags"
check "$T/check.out" "^pose R1 -2.00,3.00,145.0 (config/poses)" "R1 from config/poses"
check "$T/check.out" "^pose R2 1.00,1.00,180.0 (--pose)" "R2 from --pose"

echo "== 7. phone video (scripts/video_send.py) survives a hud_display crash and restart"
VPORT=${VIDEO_TEST_PORT:-47126}
if ! command -v "$PY" >/dev/null 2>&1 || [ ! -f scripts/video_send.py ]; then
  echo "SKIP no $PY or scripts/video_send.py"
else
  "$SV" --bin-dir "$B" --only display --background wearer --view-node R1 --video-port "$VPORT" \
    --display-arg --sink --display-arg none --no-health --self-prio 0 --display-prio 0 \
    --backoff-min-ms 300 --status-s 0 --log-dir "$T/video" 2>"$T/video.err" &
  SVPID=$!
  "$PY" scripts/video_send.py --port "$VPORT" --node R1 --size 320x240 --fps 15 --seconds 20 \
    >"$T/video_send.log" 2>&1 &
  VSPID=$!
  DL=$T/video/hud_display.log
  wait_for 8 '[ "$(count "$DL" "video: first frame from")" -ge 1 ]'
  check "$T/video/seeker_supervisor.log" "hud_display background wearer, view node R1, video tcp/$VPORT" \
    "supervisor logs the video flags"
  check "$DL" "video: listening on tcp/$VPORT" "hud_display listens on --video-port"
  check "$DL" "video: first frame from .* R1 320x240" "hud_display decodes the stand-in video"
  DP=$(cat "$T/video/hud_display.pid" 2>/dev/null)
  kill -KILL "$DP" 2>/dev/null
  wait_for 10 '[ "$(count "$DL" "video: first frame from")" -ge 2 ]'
  check "$T/video/seeker_supervisor.log" "hud_display pid $DP killed by SIGKILL" "display crash seen"
  [ "$(count "$DL" "video: first frame from")" -ge 2 ] \
    && pass "video back after the display restart (sender reconnected)" \
    || fail "no video after the display restart"
  kill "$VSPID" 2>/dev/null
  kill -TERM "$SVPID"
  if wait "$SVPID"; then pass "clean stop with video"; else fail "stop with video: bad exit status"; fi
  wait "$VSPID" 2>/dev/null
fi

echo "== $FAILS failure(s); logs in $T/"
[ "$FAILS" -eq 0 ]
