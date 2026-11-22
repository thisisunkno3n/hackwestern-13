#!/bin/sh
# IF-5.6 video loopback: scripts/video_send.py -> TCP 47106 -> hud_display (ppm sink, wearer view).
# Runs from drafts/pi-lane with binaries in build/. POSIX sh (QNX ksh and Linux sh). Needs python3.
#
#   scripts/video_loopback_test.sh             # BUILD=/other/build VIDEO_PORT=47116 to override
#   SENDER_EXIT_DELAY=0.5 FIRST_FRAME_DELAY=1 scripts/video_loopback_test.sh   # slow-Pi timing
#
# Checks: NO VIDEO + radar fallback with no relay; a junk prefix forces one resync; frames decode
# and fill the letterboxed picture (red, green and blue test card corners sampled from the PPM);
# a silent but connected relay gives VIDEO LOST (after VIDEO_HOLD_MS, 2 s); a closed relay gives
# NO VIDEO again; the IPv6 listener takes a frame over ::1 when the stack has it; a bursty relay
# (gaps of 1.2 to 1.5 s, like phone video over Wi-Fi) keeps the picture up, dimmed as VIDEO STALE.
#
# No sleeps decide what a snapshot shows. hud_display writes each PPM with a header comment such
# as "# seeker hud frame=42 video=ok drawn=1 node=R1 seq=17" and renames it into place, so each
# snapshot polls (every 0.1 s, up to SNAP_TIMEOUT s) for a whole PPM whose note matches, and the
# sender keeps streaming until the video snapshot is taken. Sampling after the sender exits raced
# the receiver noticing the close and redrawing the radar fallback (black at every sampled pixel).
#
# Env: BUILD, PYTHON, VIDEO_PORT, VIDEO_LOG (default logs/video), SNAP_TIMEOUT (whole seconds,
# default 15), PPM_EVERY (default 3), FIRST_FRAME_DELAY and SENDER_EXIT_DELAY (seconds, default 0:
# a slow first frame, and a sender that lingers after closing its socket).

set -u
B=${BUILD:-build}
PY=${PYTHON:-python3}
PORT=${VIDEO_PORT:-47106}
LOG=${VIDEO_LOG:-logs/video}
TIMEOUT=${SNAP_TIMEOUT:-15}
PPM_EVERY=${PPM_EVERY:-3}
FIRST_DELAY=${FIRST_FRAME_DELAY:-0}
EXIT_DELAY=${SENDER_EXIT_DELAY:-0}
FAILS=0

[ -x "$B/hud_display" ] || { echo "missing $B/hud_display (build first)"; exit 2; }
command -v "$PY" >/dev/null 2>&1 || { echo "missing $PY"; exit 2; }
mkdir -p "$LOG"
rm -f "$LOG"/*

pass() { echo "PASS $*"; }
fail() { echo "FAIL $*"; FAILS=$((FAILS + 1)); }
check() { # check <file> <pattern> <description>
  if grep -q "$2" "$1"; then pass "$3"; else fail "$3 (no '$2' in $1)"; fi
}
note() { # note <ppm>: its "# seeker hud ..." header comment, without the "# "
  head -n 2 "$1" 2>/dev/null | sed -n '2s/^# //p'
}
# snap_when <name> <pattern>: copy the newest PPM to <name>.ppm once its note matches <pattern>.
snap_when() {
  i=0
  while [ "$i" -lt $((TIMEOUT * 10)) ]; do
    if cp "$LOG/hud.ppm" "$LOG/$1.ppm" 2>/dev/null && note "$LOG/$1.ppm" | grep -q "$2"; then
      echo "   $1: $(note "$LOG/$1.ppm")"
      return 0
    fi
    sleep 0.1
    i=$((i + 1))
  done
  echo "   $1: no PPM with '$2' within $TIMEOUT s (last: $(note "$LOG/$1.ppm"))"
  return 1
}
# pixel <ppm> <x> <y> <expect: red|green|blue|notred> <description>
pixel() {
  if "$PY" - "$@" <<'EOF'
import sys
path, x, y, want = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), sys.argv[4]
data = open(path, "rb").read()
pos, fields = 0, []
while len(fields) < 4:  # P6 w h maxval, with "#" comment lines allowed between them
    while data[pos:pos + 1].isspace():
        pos += 1
    if data[pos:pos + 1] == b"#":
        pos = data.index(b"\n", pos) + 1
        continue
    end = pos
    while not data[end:end + 1].isspace():
        end += 1
    fields.append(data[pos:end])
    pos = end
pos += 1
w = int(fields[1])
px = data[pos + (y * w + x) * 3:pos + (y * w + x) * 3 + 3]
r, g, b = px[0], px[1], px[2]
ok = {"red": r > 200 and g < 60 and b < 60, "green": g > 200 and r < 60 and b < 60,
      "blue": b > 200 and r < 60 and g < 60, "notred": not (r > 200 and g < 60 and b < 60)}[want]
print("   %s (%d,%d) = %d,%d,%d" % (path, x, y, r, g, b))
sys.exit(0 if ok else 1)
EOF
  then pass "$5"; else fail "$5"; fi
}

# Canvas is display.size from config/seeker.json (1280x720 unless changed). The 640x480 card is
# pillarboxed to x 160..1120; its 64 px corner squares land at about 96 px on screen. This tests the
# video path, so it runs the classic theme: the plan theme's title block covers the bottom-left
# probe (its layout is tested in test_core).
echo "== hud_display --background wearer --sink ppm on tcp/$PORT"
"$B/hud_display" --sink ppm --ppm "$LOG/hud.ppm" --ppm-every "$PPM_EVERY" --background wearer \
  --theme classic --no-health --video-port "$PORT" --run-for 300 2>"$LOG/hud_display.log" &
HUD=$!
snap_when 1_no_relay "video=none drawn=0" || fail "no relay: no NO VIDEO snapshot"

echo "== video_send.py 640x480 at 15 fps, junk bytes first, until R1's picture is in a snapshot"
"$PY" scripts/video_send.py --port "$PORT" --size 640x480 --fps 15 --seconds 60 --junk --once \
  --first-delay "$FIRST_DELAY" --exit-delay "$EXIT_DELAY" 2>"$LOG/send.log" &
SND=$!
snap_when 2_video "video=ok drawn=1 node=R1 " || fail "video: no snapshot with R1's picture drawn"
kill -TERM "$SND" 2>/dev/null
wait "$SND"
snap_when 3_after_close "video=none drawn=0" || fail "relay closed: no NO VIDEO snapshot"

echo "== one frame over IPv6 ::1, then silent but connected until VIDEO LOST"
"$PY" - "$PORT" "$LOG" "$TIMEOUT" <<'EOF' 2>"$LOG/ipv6.log"
import socket, sys, time
sys.path.insert(0, "scripts")
import video_send as vs
port, log, timeout = int(sys.argv[1]), sys.argv[2], float(sys.argv[3])


def wait_ppm(want):
    """Bytes of the first whole hud.ppm whose note has every string in want, else None."""
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        try:
            data = open(log + "/hud.ppm", "rb").read()
        except OSError:
            data = b""
        lines = data.split(b"\n", 2)
        if len(lines) > 1 and all(w.encode() in lines[1] for w in want):
            print("   %s" % lines[1][2:].decode("ascii", "replace"), file=sys.stderr)
            return data
        time.sleep(0.05)
    print("no PPM with %s within %.0f s" % (" ".join(want), timeout), file=sys.stderr)
    return None


try:
    s = socket.create_connection(("::1", port), timeout=2)
except OSError as e:
    print("no IPv6 connect: %s" % e, file=sys.stderr)
    sys.exit(0)
s.sendall(vs.frame_bytes("R2", 1, vs.test_card(640, 480, 0, 30), (640, 480)))
rc = 1
if wait_ppm(["video=ok drawn=1 node=R2 "]):
    data = wait_ppm(["video=lost drawn=0 node=R2 "])
    if data:
        open(log + "/4_silent.ppm", "wb").write(data)
        rc = 0
s.close()
sys.exit(rc)
EOF
IPV6_RC=$?
cat "$LOG/ipv6.log"

echo "== bursty relay (phone over Wi-Fi): gaps of 1.2 to 1.5 s, the picture stays up (VIDEO STALE)"
"$PY" - "$PORT" "$LOG" <<'EOF' 2>"$LOG/bursty.log"
import socket, sys, time
sys.path.insert(0, "scripts")
import video_send as vs
port, log = int(sys.argv[1]), sys.argv[2]


def lost_lines():
    try:
        return open(log + "/hud_display.log", "rb").read().count(b"picture no longer drawn")
    except OSError:
        return 0


states = set()


def watch(seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            note = open(log + "/hud.ppm", "rb").read(200).split(b"\n", 2)[1]
            if b" video=" in note:
                states.add(note.split(b" video=")[1].split(b" ")[0].decode("ascii", "replace"))
        except (OSError, IndexError):
            pass
        time.sleep(0.05)


s = socket.create_connection(("127.0.0.1", port), timeout=2)
card = vs.test_card(640, 480, 0, 30)
s.sendall(vs.frame_bytes("R1", 1, card, (640, 480)))
# Until this relay's first frame is drawn, hud.ppm may still carry the previous phase's
# "video=lost": start counting only once R1's picture is up.
end = time.monotonic() + 5
while time.monotonic() < end:
    try:
        if b"video=ok drawn=1 node=R1 " in open(log + "/hud.ppm", "rb").read(200):
            break
    except OSError:
        pass
    time.sleep(0.02)
states.clear()
before = lost_lines()
seq = 2
# Gaps well inside the 2 s hold (VIDEO_HOLD_MS), with 0.5 s of margin for a slow QNX scheduler.
# The gaps actually achieved are measured: a gap the sender stretched past 2 s is a real outage,
# and the HUD dropping the picture then is right, not a failure.
HOLD_S = 2.0
achieved = []
last_send = time.monotonic()
for gap in (1.2, 0.1, 1.5, 0.3, 1.4, 0.1):
    watch(gap)
    s.sendall(vs.frame_bytes("R1", seq, card, (640, 480)))
    now = time.monotonic()
    achieved.append(now - last_send)
    last_send = now
    seq += 1
watch(0.3)
flaps = lost_lines() - before
s.close()
real_outages = sum(1 for g in achieved if g > HOLD_S)
print("   gaps sent: %s s; states seen: %s; picture dropped %d time(s), real outages (gap over "
      "%.0f s) %d" % (" ".join("%.2f" % g for g in achieved), " ".join(sorted(states)), flaps,
                      HOLD_S, real_outages), file=sys.stderr)
ok = flaps <= real_outages and "stale" in states and ("lost" not in states or real_outages)
sys.exit(0 if ok else 1)
EOF
BURSTY_RC=$?
cat "$LOG/bursty.log"
[ "$BURSTY_RC" -eq 0 ] && pass "bursty relay: picture kept through 1.2 to 1.5 s gaps (stale, never lost)" ||
  fail "bursty relay: the picture dropped during short gaps (see $LOG/bursty.log)"
kill -TERM "$HUD" 2>/dev/null
wait "$HUD"

check "$LOG/hud_display.log" "video: listening on tcp/$PORT" "receiver listening on tcp/$PORT"
check "$LOG/hud_display.log" "video: relay connected from" "relay connection accepted"
check "$LOG/hud_display.log" "video: lost sync" "junk prefix forced a resync"
check "$LOG/hud_display.log" "first frame from .* R1 640x480" "first R1 640x480 frame decoded"
check "$LOG/hud_display.log" "frame drawn on the HUD: R1 " "R1's picture drawn on the HUD"
check "$LOG/hud_display.log" "disconnected (closed)" "relay close noticed"
check "$LOG/send.log" "done: sent" "sender finished"
if grep -q "IPv4 + IPv6" "$LOG/hud_display.log" && ! grep -q "no IPv6 connect" "$LOG/ipv6.log"; then
  check "$LOG/hud_display.log" "first frame from \[::1\]" "IPv6 relay frame decoded"
  [ "$IPV6_RC" -eq 0 ] && pass "silent relay: VIDEO LOST snapshot" ||
    fail "silent relay: no VIDEO LOST snapshot after R2's frame (see $LOG/ipv6.log)"
else
  echo "SKIP IPv6 (no IPv6 listener or no ::1 on this machine)"
fi
if ! grep -q "JPEG decode failed" "$LOG/hud_display.log"; then pass "no decode errors"; else fail "JPEG decode failed (see $LOG/hud_display.log)"; fi
pixel "$LOG/1_no_relay.ppm" 200 50 notred "no relay: radar fallback, no picture"
pixel "$LOG/2_video.ppm" 200 50 red "video: red test card corner top left"
pixel "$LOG/2_video.ppm" 1080 50 green "video: green corner top right (else try display.swap_rb)"
pixel "$LOG/2_video.ppm" 200 650 blue "video: blue corner bottom left"
pixel "$LOG/3_after_close.ppm" 200 50 notred "relay closed: radar fallback"
[ -f "$LOG/4_silent.ppm" ] && pixel "$LOG/4_silent.ppm" 200 50 notred "silent relay: VIDEO LOST fallback"

echo "logs and snapshots in $LOG/"
if [ "$FAILS" -eq 0 ]; then echo "ALL PASS"; else echo "$FAILS FAILED"; fi
exit "$FAILS"
