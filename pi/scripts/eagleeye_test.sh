#!/bin/sh
# EagleEye loopback: scripts/eagleeye_sim.py (helmet phone W + R1 + R2, a wall, two people)
# -> hud_fusion -> hud_display --view wearer (ppm sink). Needs only python3 and build/.
# Runs from drafts/pi-lane. POSIX sh (QNX ksh and Linux sh). Host runs are a pre-filter only.
#
#   scripts/eagleeye_test.sh
#
# Snapshots land in logs/eagleeye/*.ppm (and .png when ffmpeg is installed).

set -u
B=${BUILD:-build}
CFG=config/seeker.eagleeye.json
PY=${PYTHON:-python3}
LOG=logs/eagleeye
FAILS=0

for x in udp_recv hud_fusion hud_display; do
  [ -x "$B/$x" ] || { echo "missing $B/$x (build first)"; exit 2; }
done
command -v "$PY" >/dev/null 2>&1 || { echo "missing $PY"; exit 2; }
mkdir -p "$LOG"
rm -f "$LOG"/*

pass() { echo "PASS $*"; }
fail() { echo "FAIL $*"; FAILS=$((FAILS + 1)); }
check() { # check <file> <pattern> <description>
  if grep -q -- "$2" "$1"; then pass "$3"; else fail "$3 (no '$2' in $1)"; fi
}
sim() { # sim <seconds> <args...>: one scene phase, in the foreground
  s=$1; shift
  "$PY" scripts/eagleeye_sim.py --run-for "$s" "$@" 2>>"$LOG/sim.log"
}
snap() { # snap <name>: keep the newest HUD frame
  cp "$LOG/hud.ppm" "$LOG/$1.ppm" 2>/dev/null
  if command -v ffmpeg >/dev/null 2>&1; then
    ffmpeg -loglevel error -y -i "$LOG/$1.ppm" "$LOG/$1.png" 2>/dev/null
  fi
}

echo "== 1. scene -> hud_fusion -> tracks on 47102: helmet phone and R2 fuse person B"
"$B/udp_recv" 47102 --raw --run-for 3 >"$LOG/tracks.log" 2>&1 &
TR=$!
"$B/hud_fusion" --config "$CFG" --no-health --run-for 3.5 2>"$LOG/fusion1.log" &
FU=$!
sleep 0.3
sim 2.5 --at 0
wait "$TR" "$FU"
check "$LOG/tracks.log" '"nodes":"W+R2"' "B seen by W and R2 is one track, W primary"
check "$LOG/tracks.log" '"nodes":"R1"' "A behind the wall is seen by R1 only"
check "$LOG/fusion1.log" "NODE W first datagram" "hud_fusion hears the helmet phone"

echo "== 2. hud_display --view wearer through four scene phases"
"$B/hud_fusion" --config "$CFG" --no-health --run-for 23 2>"$LOG/fusion.log" &
FU=$!
"$B/hud_display" --config "$CFG" --view wearer --sink ppm --ppm "$LOG/hud.ppm" --ppm-every 5 \
  --no-health --run-for 23 2>"$LOG/display.log" &
DI=$!
sleep 0.5
echo "   a. t=0: B in front of the helmet, A behind the wall"
sim 3 --at 0; snap a_seen_and_xray
echo "   b. t=8: B behind the wearer"
sim 3 --at 8; snap b_behind
echo "   c. helmet turns to 100 deg (live pose from the phone)"
sim 3 --at 8 --wearer-yaw 100; snap c_turned
echo "   d. helmet phone off: W LOST (last heading kept), B turns from seen to x-ray"
sim 3.5 --at 8 --wearer-yaw 100 --nodes R1,R2; snap d_helmet_lost
echo "   e. IF-5.6 video from the helmet phone W under the symbology"
"$PY" scripts/video_send.py --node W --size 1280x720 --seconds 3.5 --once 2>>"$LOG/video.log" &
sim 3.5 --at 0; snap e_over_video
echo "   f. video from R1 instead: skipped (the wearer view is pinned to W's video)"
"$PY" scripts/video_send.py --node R1 --seconds 3 --once 2>>"$LOG/video.log" &
sim 3 --at 0; snap f_video_other_node
wait "$DI" "$FU"
check "$LOG/display.log" "eagleeye: wearer W pose 0.00,0.00,0.0 (auto)" "wearer W from config"
check "$LOG/display.log" "W ok pose live 0.00,0.00,0.0  seen 1  xray 1  off 0" "a: B seen, A x-ray"
check "$LOG/display.log" "W ok pose live 0.00,0.00,0.0  seen 0  xray 1  off 1" "b: B off screen"
check "$LOG/display.log" "W ok pose live .*  seen 1  xray 0  off 1" "c: live yaw brings B into view"
check "$LOG/display.log" "W lost pose live 0.00,0.00,100.0  seen 0  xray 1  off 1" "d: helmet lost, B x-ray"
check "$LOG/fusion.log" "NODE W LOST" "hud_fusion marks the helmet phone LOST"
check "$LOG/display.log" "seen 1  xray 1  off 0  dup 0  unplaced 0  video on" "e: drawn over W's video"
check "$LOG/display.log" "video: frames from R1 skipped: this HUD shows only W's video" \
  "f: R1's video is not used for W's projection"
check "$LOG/display.log" "video: wearer view pinned to W's video" "wearer view pinned to W"
if grep -q "video other-node" "$LOG/display.log"; then
  fail "f: R1's frames reached the wearer view"
else
  pass "f: R1's frames never reached the wearer view"
fi
if grep -q "hud_display.*dead\|Segmentation\|Abort" "$LOG/display.log"; then
  fail "hud_display crashed"
else
  pass "hud_display ran all phases"
fi
for s in a_seen_and_xray b_behind c_turned d_helmet_lost e_over_video f_video_other_node; do
  [ -s "$LOG/$s.ppm" ] && pass "snapshot $s" || fail "no snapshot $s"
done

echo "== 3. --video-fallback radar (IF-7 behaviour for the monitor) and --visor settings parse"
"$B/hud_display" --config "$CFG" --view wearer --video-fallback radar --sink none --no-health \
  --run-for 1 2>"$LOG/display_fallback.log"
check "$LOG/display_fallback.log" "no video: radar" "--video-fallback radar"
"$B/hud_display" --config "$CFG" --view wearer --video-fallback radar --visor --mirror --sink none \
  --no-health --run-for 1 2>"$LOG/display_visor.log"
check "$LOG/display_visor.log" "no video: black, visor" "--visor keeps black even with radar fallback"
check "$LOG/display_visor.log" "mirror h" "bare --mirror is horizontal"

echo "== $FAILS failure(s); logs and snapshots in $LOG/"
[ "$FAILS" -eq 0 ]
