#!/bin/sh
# Loopback test: Daniel's tools/sim/remote.py -> udp_recv, hud_fusion, hud_supervisor + hud_display.
# Runs from drafts/pi-lane with binaries in build/. POSIX sh (works in QNX's ksh and Linux sh).
#
#   HW13=/path/to/hackwestern-13 scripts/loopback_test.sh
#
# Needs python3 on this machine for the sim. On the Pi without python3, run the sim on a laptop
# with --pi qnxpi-felix.local instead (MORNING.md).

set -u
HW13=${HW13:-../../../hackwestern-13}
SIM="$HW13/tools/sim/remote.py"
B=${BUILD:-build}
CFG=config/seeker.loopback.json
PY=${PYTHON:-python3}
LOG=logs/loopback
FAILS=0

[ -f "$SIM" ] || { echo "missing $SIM (set HW13 to a hackwestern-13 checkout)"; exit 2; }
for x in udp_recv hud_fusion hud_display hud_supervisor; do
  [ -x "$B/$x" ] || { echo "missing $B/$x (build first)"; exit 2; }
done
mkdir -p "$LOG"
rm -f "$LOG"/* logs/hud.ppm logs/events.jsonl logs/hud_fusion.log logs/hud_display.log

pass() { echo "PASS $*"; }
fail() { echo "FAIL $*"; FAILS=$((FAILS + 1)); }
check() { # check <file> <pattern> <description>
  if grep -q "$2" "$1"; then pass "$3"; else fail "$3 (no '$2' in $1)"; fi
}
sim() { # sim <node> <pose> [people]; --pose= so argparse accepts a negative x
  "$PY" "$SIM" --pi 127.0.0.1 --node "$1" --pose="$2" --people "${3:-1}" 2>>"$LOG/sim_$1.log" &
}
stop_name() { # slay on QNX, pkill elsewhere
  if command -v slay >/dev/null 2>&1; then slay "$1" >/dev/null 2>&1; else pkill -x "$1"; fi
}

echo "== 1. sim R1 -> udp_recv 47103 (checkpoint 1 fallback)"
"$B/udp_recv" 47103 --run-for 4 >"$LOG/udp_recv.log" 2>&1 &
RECV=$!
sleep 1
sim R1 0,0,0; S1=$!
sleep 2
kill "$S1"
wait "$RECV"
check "$LOG/udp_recv.log" "remote_contacts R1  1 contact  bearing" "udp_recv prints R1 packets"

echo "== 2. sim R1 + R2 -> hud_fusion --debug -> tracks on 47102 -> udp_recv"
"$B/udp_recv" 47102 --run-for 9 >"$LOG/tracks.log" 2>&1 &
TR=$!
"$B/hud_fusion" --config "$CFG" --debug --no-health --run-for 9 2>"$LOG/fusion.log" &
FU=$!
sleep 1
sim R1 -2,3,145; S1=$!
sim R2 2,3,215; S2=$!
sleep 2
kill "$S1"
echo "   R1 sim stopped; waiting 3 s for NODE R1 LOST"
sleep 3
sim R1 -2,3,145; S1=$!
sleep 2
kill "$S1" "$S2"
wait "$FU" "$TR"
check "$LOG/fusion.log" "rx R1 from 127.0.0.1:[0-9]*  1 contact" "hud_fusion debug line per R1 datagram"
check "$LOG/fusion.log" "rx R2 from" "hud_fusion receives R2"
check "$LOG/fusion.log" "NODE R1 LOST" "NODE R1 LOST after the sim stops"
check "$LOG/fusion.log" "NODE R1 resumed" "NODE R1 resumed after the sim restarts"
check "$LOG/tracks.log" "R1:lost" "tracks carry R1 lost state"
check "$LOG/tracks.log" "R1:ok  R2:ok" "tracks carry both nodes ok"
SILENT=$(sed -n 's/.*NODE R1 LOST (\([0-9]*\) ms silent).*/\1/p' "$LOG/fusion.log" | head -1)
if [ -n "$SILENT" ] && [ "$SILENT" -ge 1500 ] && [ "$SILENT" -le 1600 ]; then
  pass "LOST fired at $SILENT ms of silence (rule: after 1500)"
else
  fail "LOST timing ${SILENT:-none} ms"
fi

echo "== 3. supervisor + fusion + display (ppm sink); slay hud_fusion, then freeze it"
"$B/hud_supervisor" --config "$CFG" --run-for 14 2>"$LOG/supervisor.log" &
SU=$!
sim R1 -2,3,145 2; S1=$!
sim R2 2,3,215; S2=$!
sleep 4
cp logs/hud.ppm "$LOG/hud_radar.ppm" 2>/dev/null
echo "   slay hud_fusion"
stop_name hud_fusion
sleep 3
FPID=$(sed -n 's/.*started hud_fusion pid \([0-9]*\).*/\1/p' "$LOG/supervisor.log" | tail -1)
echo "   SIGSTOP hud_fusion pid $FPID (heartbeat watchdog)"
kill -STOP "$FPID" 2>/dev/null
sleep 1
cp logs/hud.ppm "$LOG/hud_fusion_lost.ppm" 2>/dev/null
sleep 3
kill "$S1" "$S2"
sleep 2
cp logs/hud.ppm "$LOG/hud_nodes_lost.ppm" 2>/dev/null
wait "$SU"
check "$LOG/supervisor.log" "hud_fusion pid [0-9]* dead: exited with 0" "supervisor sees the slay"
check "$LOG/supervisor.log" "no heartbeat for" "supervisor kills a frozen hud_fusion"
N=$(grep -c "started hud_fusion" "$LOG/supervisor.log")
if [ "$N" -ge 3 ]; then pass "hud_fusion restarted twice ($N starts)"; else fail "hud_fusion starts: $N"; fi
check "$LOG/supervisor.log" "started hud_display" "supervisor starts hud_display"
if grep -q "hud_display pid [0-9]* dead" "$LOG/supervisor.log"; then
  fail "hud_display died while fusion was restarted"
else
  pass "hud_display never died while fusion restarted"
fi
[ -s "$LOG/hud_radar.ppm" ] && pass "HUD snapshot written" || fail "no HUD snapshot"

echo "== 4. EagleEye wearer view (scripts/eagleeye_test.sh, logs in logs/eagleeye/)"
if sh scripts/eagleeye_test.sh >"$LOG/eagleeye.log" 2>&1; then
  pass "eagleeye_test.sh"
else
  grep FAIL "$LOG/eagleeye.log"
  fail "eagleeye_test.sh (see $LOG/eagleeye.log)"
fi

echo "== $FAILS failure(s); logs in $LOG/"
[ "$FAILS" -eq 0 ]
