#!/bin/sh
# One command for the live run on the QNX Pi: hud_display (EagleEye wearer view of the helmet
# phone, radar when there is no video), hud_fusion, and the phone bridge on HTTPS/WSS 8080
# (Daniel's ws_ingest.py, run through scripts/seeker_bridge.py). Each process logs through tee
# to $LOG_DIR/<name>.log. POSIX sh: QNX 8.0 sh (pdksh) and Linux sh. docs/LIVE-RUN.md has the
# phone links, what every banner and log line means, and what to do about it.
#
#   sh scripts/run-live.sh             # foreground: Ctrl+C stops all three
#   DETACH=1 sh scripts/run-live.sh    # survives logout, logs to files only; stop-live.sh stops it
#   sh scripts/stop-live.sh            # stops a run (and stray copies started by hand)
#
# Env (defaults are the Sat Nov 21 live setup):
#   SEEKER_DIR  pi-lane checkout (~/seeker/drafts/pi-lane)    BUILD  binaries (build)
#   WS_INGEST   Daniel's bridge (~/seeker/remote/ws_ingest.py)
#   CERT, KEY   TLS files for iPhone Safari (~/cert.pem, ~/key.pem)
#   LOG_DIR     ~/seeker-logs            CONFIG  config/seeker.live.json (R1 + R2, 640x480, no walls)
#   WEARER      helmet phone and video source (R1)
#   WEARER_POSE starting wearer pose in room metres and deg (0,-2,0: 2 m in front of the tag,
#               facing it), not pinned: the first full x,y,yaw pose in the wearer's messages
#               (ArUco tag, ?tag=1) replaces it
#   WEARER_MODE tag (default): the wearer moves and finds itself from tag 0, as above.
#               fixed: the wearer stands still (DANIEL-DECISION-fixed-wearer-nov21): hud_fusion
#               --pose $WEARER=$WEARER_POSE pins it (message poses ignored), and the helmet
#               link is ?node=R1&video=1 with no tag detection on the phone (more video fps).
#               Measure WEARER_POSE once, in the tag 0 room frame.
#   R2_MODE     pin (default): R2 is pinned at R2_POSE and its link has &pose=.
#               R2's links never ask for video (&video=1): only the wearer's video reaches the
#               HUD, and a second video stream only fills the hotspot's uplink.
#               tag: R2 finds itself from tag 0 (preferred) or tag R2_TAG_ID (7): link
#               ?tagid=0,7, hud_fusion --tag-pose 7=$TAG7_POSE moves a pose the phone names as
#               tag 7's (pose.tag_id) into the room. Daniel's current phone.html reads
#               tagid=0,7 as 0 and names no tag: tag 0 works today, the tag 7 fallback needs
#               progress/FELIX-asks-daniel-r2-multitag.md.
#               tag7: tag 7 only, works with today's phone.html: link ?tagid=7, hud_fusion
#               --tag-frame R2=$TAG7_POSE.
#               In both tag modes R2 is not pinned (PINS must not name R2) and holds its last
#               pose on pose null like R1.
#               off: R1 only (solo test, no R2 phone): no R2 pin, no R2 link. The HUD status
#               reads "R2 --" in grey (never connected): no NODE R2 LOST banner, no radar flip;
#               fusion places people from R1 alone (single-phone range).
#   R2_POSE     pinned pose for R2 (0,2,180), measured from the tag (room frame below); pin mode.
#               x,y,yaw: x negative = LEFT of tag 0 as you face it, positive = right; y negative
#               = in front of the tag's wall (your side), positive = behind it; yaw 0 faces the
#               tag, 90 faces right, -90 faces left, 180 faces away. The default 0,2,180 is 2 m
#               BEHIND the tag's wall: set it. R2 1.5 m left of R1 (at 0,-2), facing the tag:
#               R2_POSE=-1.5,-2,0. Or measure it: sh scripts/calibrate.sh pose.
#   R2_START    R2's starting pose in tag mode, until its first tag fix (-2.463,0.06,90: 2 m in
#               front of tag 7 with the default TAG7_POSE, facing it); tag7 mode derives it
#   R2_TAG_ID   R2's second tag (7)
#   TAG7_POSE   room pose of that tag: centre x,y in metres and the room yaw of the way you face
#               looking straight at it (-0.463,0.06,90: Felix's living room, the side face of
#               the wall corner to the left of tag 0; docs/LIVE-RUN.md "R2 on tag 7" derives it)
#   PINS        hud_fusion pose pins, space separated (pin mode "R2=$R2_POSE", tag modes none;
#               setting PINS replaces R2_POSE). Do not pin the wearer.
#   WALLS       room wall segments "x1,y1,x2,y2 ..." (or "none" to clear config walls)
#   PORT        phone port (8080)        PHONE_HOST  host in the phone links (qnxpi-felix.local)
#   TAG_MM      printed tag's black square side in mm (186)   TAG_ID  tag id at the origin (0)
#   TAG_W       phone tag detection width in px (640: the full 640x480 frame; phone.html's own
#               default 320 only decodes a 186 mm tag out to about 1.3 m, 640 to about 2.6 m)
#   HFOV        helmet phone horizontal FOV in deg for the link (unset: phone.html default 66)
#   R1_HFOV, R2_HFOV  a phone's real horizontal FOV, on the Pi side (hud_fusion --hfov): degrees
#               or a preset (iphone17pro, iphone16, iphone-ultrawide; landscape or portrait from
#               the frame size). The phone's bearings and box-height ranges are rescaled, and the
#               wearer view projects with R1's (its vertical FOV follows the streamed frame's
#               aspect: square pixels). Tonight: R1_HFOV=iphone16, R2_HFOV=iphone17pro. "" = off.
#   RANGE_HFOV  1: box-height ranges rescaled with R?_HFOV too; 0 if a phone's range was
#               calibrated on its page (the "cal" button)
#   PERSON_H    people's height in m (1.75): box-height range (phone.html assumes 1.7) and the
#               x-ray box top
#   R1_RANGE_K, R2_RANGE_K  extra factor on that phone's box-height ranges ("": none)
#   R1_CAM_H    wearer camera height above the floor in m (1.30: handheld at chest height)
#   R1_PITCH    wearer camera tilt in deg, up positive (0; the phone's pitch_deg wins if it sends
#               one)
#   XRAY_UP     nudge for the x-ray people in R1's view, deg: positive draws them higher (0)
#   PITCH_AUTO  1: learn R1's tilt from people R1 itself sees whole (their box vs the projection)
#   R2_CAM_H, R2_PITCH  R2 camera height in m (1.27) and tilt (0): hud_fusion logs a ground-plane
#               range ("range check") next to the box-height one
#   GROUND_RANGE  1: average R2's ground-plane range into its ranges (0: logged only)
#   DRAIN       1: when R1's uplink falls behind (ws rtt over 400 ms three times), the bridge
#               pauses R1's video until the phone's queue drains, then resumes (0: off)
#   CALIB       calibration file from scripts/calibrate.sh ($LOG_DIR/calib.env): sets
#               WEARER_POSE, R2_POSE (and any of the above) unless given on the command line
#   Tag 7 is optional: the defaults (WEARER_MODE=tag or fixed, R2_MODE=pin) use tag 0 only.
#   TAG_MAP     moving wearer: more tags in the room frame, "1=x,y,yaw 2=x,y,yaw" (measure with
#               scripts/calibrate.sh tag N): hud_fusion --tag-pose each, helmet ?tagid=0,1,2.
#               Without a tag the wearer keeps its position, turns with the phone's compass when
#               the page sends heading_deg, and after fusion.pose_home_ms (10 s) eases to
#               WEARER_POSE; a tag back blends in (no jump). HUD: POSE HELD / POSE HOME.
#   VFPS        helmet video fps (15): the link's &vfps= and the bridge's video_ctrl fps (the
#               bridge's video_ctrl overrides the link, so both are set)
#   VQ          helmet JPEG quality 0..1 for the link's &vq= (unset: phone.html default 0.6)
#   LAT         1 = latency readout on the HUD ("LAT 120MS P95 180MS FPS 28") and a latency
#               line every second in hud_display.log (default: every 10 s, no readout)
#   LAT_BASE_MS the fixed camera->screen part from the stopwatch photo, added to the readout
#               (docs/LIVE-RUN.md "Latency"; 0)
#   SCREEN_BUFFER  WxH, e.g. 1920x1080: hud_display draws at that size and QNX Screen scales it
#               to the monitor (hud_display --screen-buffer). Try it when hud_display.log says
#               "sink screen 3840x2160" and "draw+present" is over 10 ms. Unset: the display size.
#   VSYNC       0 = posts do not wait for vsync (hud_display --vsync off; may tear; the latency
#               line's "post" stage shows the wait it saves). Default 1.
#   THEME       HUD look: plan (default, Daniel's drafting theme) or classic (hud_display --theme)
#   SINK        screen (ppm or none to try it without the monitor)
#   BRIDGE      seeker (video_ctrl, video link at once, status lines) or daniel (ws_ingest.py as is)
#   FUSION_DEBUG 1 = hud_fusion --debug   QUIET 1 = logs to files only
#   HUD_ARGS, FUSION_ARGS, BRIDGE_ARGS   extra words for each command
#
# Room frame with the tag on a wall (phone.html default tagmap=xz): origin at the tag, +y is the
# way you face when you look straight at the tag (into its wall), +x is your right, yaw is
# clockwise from +y (0 faces the tag, 90 faces right). Everything in front of the tag has y < 0.
set -u
SEEKER_DIR=${SEEKER_DIR:-$HOME/seeker/drafts/pi-lane}
BUILD=${BUILD:-build}
WS_INGEST=${WS_INGEST:-$HOME/seeker/remote/ws_ingest.py}
CERT=${CERT:-$HOME/cert.pem}
KEY=${KEY:-$HOME/key.pem}
LOG_DIR=${LOG_DIR:-$HOME/seeker-logs}
CONFIG=${CONFIG:-config/seeker.live.json}
CALIB=${CALIB:-$LOG_DIR/calib.env}
# calib.env lines are NAME=${NAME:-value}: a variable given on the command line still wins.
if [ -f "$CALIB" ]; then
  . "$CALIB"
  echo "run-live: calibration from $CALIB: WEARER_POSE=${WEARER_POSE:-} R2_POSE=${R2_POSE:-}"
fi
WEARER=${WEARER:-R1}
R1_HFOV=${R1_HFOV-iphone16}
R2_HFOV=${R2_HFOV-iphone17pro}
RANGE_HFOV=${RANGE_HFOV:-1}
PERSON_H=${PERSON_H:-1.75}
R1_RANGE_K=${R1_RANGE_K:-}
R2_RANGE_K=${R2_RANGE_K:-}
R1_CAM_H=${R1_CAM_H:-1.30}
R1_PITCH=${R1_PITCH:-0}
XRAY_UP=${XRAY_UP:-0}
PITCH_AUTO=${PITCH_AUTO:-1}
R2_CAM_H=${R2_CAM_H:-1.27}
R2_PITCH=${R2_PITCH:-0}
GROUND_RANGE=${GROUND_RANGE:-0}
DRAIN=${DRAIN:-1}
TAG_MAP=${TAG_MAP:-}
WEARER_POSE=${WEARER_POSE:-0,-2,0}
WEARER_MODE=${WEARER_MODE:-tag}
R2_MODE=${R2_MODE:-pin}
R2_POSE=${R2_POSE:-0,2,180}
R2_START=${R2_START:-"-2.463,0.06,90"}
R2_TAG_ID=${R2_TAG_ID:-7}
TAG7_POSE=${TAG7_POSE:-"-0.463,0.06,90"}
WALLS=${WALLS:-}
PORT=${PORT:-8080}
PHONE_HOST=${PHONE_HOST:-qnxpi-felix.local}
TAG_MM=${TAG_MM:-186}
TAG_ID=${TAG_ID:-0}
TAG_W=${TAG_W:-640}
HFOV=${HFOV:-}
VFPS=${VFPS:-15}
VQ=${VQ:-}
LAT=${LAT:-0}
LAT_BASE_MS=${LAT_BASE_MS:-0}
SCREEN_BUFFER=${SCREEN_BUFFER:-}
THEME=${THEME:-}
VSYNC=${VSYNC:-1}
SINK=${SINK:-screen}
BRIDGE=${BRIDGE:-seeker}
FUSION_DEBUG=${FUSION_DEBUG:-1}
DETACH=${DETACH:-0}
QUIET=${QUIET:-0}
HUD_ARGS=${HUD_ARGS:-}
FUSION_ARGS=${FUSION_ARGS:-}
BRIDGE_ARGS=${BRIDGE_ARGS:-}
PY=${PYTHON:-python3}
export LOG_DIR
[ "$DETACH" = 1 ] && QUIET=1

die() { echo "run-live: $*" >&2; exit 2; }
have() { command -v "$1" >/dev/null 2>&1; }

cd "$SEEKER_DIR" 2>/dev/null || die "no $SEEKER_DIR (set SEEKER_DIR to the pi-lane folder)"
for x in hud_display hud_fusion; do
  [ -x "$BUILD/$x" ] || die "$SEEKER_DIR/$BUILD/$x missing: rm -rf build && cmake -S . -B build && cmake --build build -j4"
done
[ -f "$CONFIG" ] || die "no $CONFIG in $SEEKER_DIR"
have "$PY" || die "$PY not found"
[ -f "$WS_INGEST" ] || die "no bridge at $WS_INGEST (set WS_INGEST)"
[ -f "$CERT" ] && [ -f "$KEY" ] || die "no TLS cert/key at $CERT and $KEY: iPhone Safari blocks the camera without HTTPS"
case "$BRIDGE" in seeker|daniel) ;; *) die "BRIDGE must be seeker or daniel, not $BRIDGE" ;; esac
num() { case "$2" in '' | . | *[!0-9.]* | *.*.*) die "$1 must be a plain number, not '$2'" ;; esac; }
snum() { case "${2#-}" in '' | . | *[!0-9.]* | *.*.*) die "$1 must be a number, not '$2'" ;; esac; }
num TAG_MM "$TAG_MM"
num PERSON_H "$PERSON_H"
num R1_CAM_H "$R1_CAM_H"
snum R1_PITCH "$R1_PITCH"
snum XRAY_UP "$XRAY_UP"
[ -z "$R2_CAM_H" ] || num R2_CAM_H "$R2_CAM_H"
snum R2_PITCH "$R2_PITCH"
[ -z "$R1_RANGE_K" ] || num R1_RANGE_K "$R1_RANGE_K"
[ -z "$R2_RANGE_K" ] || num R2_RANGE_K "$R2_RANGE_K"
case "$PITCH_AUTO$RANGE_HFOV$GROUND_RANGE$DRAIN" in *[!01]*) die "PITCH_AUTO, RANGE_HFOV, GROUND_RANGE, DRAIN are 0 or 1" ;; esac
num TAG_ID "$TAG_ID"
num TAG_W "$TAG_W"
num R2_TAG_ID "$R2_TAG_ID"
case "$WEARER_MODE" in tag|fixed) ;; *) die "WEARER_MODE must be tag or fixed, not $WEARER_MODE" ;; esac
case "$R2_MODE" in
  pin) PINS=${PINS:-R2=$R2_POSE} ;;
  tag | tag7)
    PINS=${PINS:-}
    for p in $PINS; do
      case "$p" in R2=*) die "R2_MODE=$R2_MODE moves R2 with its tags; drop R2 from PINS ($p)" ;; esac
    done
    [ "$R2_TAG_ID" != "$TAG_ID" ] || die "R2_TAG_ID must not be the origin tag $TAG_ID"
    for v in TAG7_POSE R2_START; do
      eval "x=\$$v"
      case "$x" in
        *[!0-9.,+-]* | *,*,*,* | ,* | *,,* | *,) die "$v must be x,y,yaw (metres, metres, deg), not '$x'" ;;
        *,*,*) ;;
        *) die "$v must be x,y,yaw (metres, metres, deg), not '$x'" ;;
      esac
    done
    ;;
  off)
    PINS=${PINS:-}
    for p in $PINS; do
      case "$p" in R2=*) die "R2_MODE=off runs without R2; drop R2 from PINS ($p)" ;; esac
    done
    ;;
  *) die "R2_MODE must be pin, tag, tag7 or off, not $R2_MODE" ;;
esac
[ -z "$HFOV" ] || num HFOV "$HFOV"
num VFPS "$VFPS"
[ -z "$VQ" ] || num VQ "$VQ"
num LAT_BASE_MS "$LAT_BASE_MS"
SEEKER_LAT=$LAT
SEEKER_LAT_BASE_MS=$LAT_BASE_MS
export SEEKER_LAT SEEKER_LAT_BASE_MS
VLINK="&video=1&vfps=$VFPS${VQ:+&vq=$VQ}"
mkdir -p "$LOG_DIR" || die "cannot create $LOG_DIR"

echo "run-live: stopping anything left from an earlier run"
sh scripts/stop-live.sh

# launch <name> <cmd...>: in the background, pid in $LOG_DIR/<name>.pid (the command's own pid,
# via exec), stdout and stderr through tee into $LOG_DIR/<name>.log.
launch() {
  NAME=$1
  shift
  LOG=$LOG_DIR/$NAME.log
  echo "=== $(date) run-live: $*" >>"$LOG"
  if [ "$DETACH" = 1 ] && have nohup; then
    nohup sh -c 'echo $$ >"$0"; exec "$@"' "$LOG_DIR/$NAME.pid" "$@" </dev/null 2>&1 \
      | nohup tee -a "$LOG" >/dev/null &
  elif [ "$QUIET" = 1 ]; then
    sh -c 'echo $$ >"$0"; exec "$@"' "$LOG_DIR/$NAME.pid" "$@" </dev/null 2>&1 | tee -a "$LOG" >/dev/null &
  else
    sh -c 'echo $$ >"$0"; exec "$@"' "$LOG_DIR/$NAME.pid" "$@" </dev/null 2>&1 | tee -a "$LOG" &
  fi
}

alive() { # alive <name>: the process in <name>.pid is running
  P=$(cat "$LOG_DIR/$1.pid" 2>/dev/null) && [ -n "$P" ] && kill -0 "$P" 2>/dev/null
}

wait_log() { # wait_log <name> <text> <seconds>: until the log has text, or the process died
  N=0
  while [ "$N" -lt "$3" ]; do
    grep -q "$2" "$LOG_DIR/$1.log" 2>/dev/null && return 0
    sleep 1
    N=$((N + 1))
    [ "$N" -ge 2 ] && ! alive "$1" && return 1
  done
  return 1
}

# 1. hud_display first, so tcp/47106 is listening before the bridge connects.
set -- "$BUILD/hud_display" --config "$CONFIG" --view wearer --wearer-node "$WEARER" \
  --wearer-pose "$WEARER=$WEARER_POSE" --wearer-pose-src auto \
  --sink "$SINK" --video-fallback radar
[ -n "$SCREEN_BUFFER" ] && set -- "$@" --screen-buffer "$SCREEN_BUFFER"
case "$VSYNC" in
  1) ;;
  0) set -- "$@" --vsync off ;;
  *) die "VSYNC must be 0 or 1, not $VSYNC" ;;
esac
case "$THEME" in
  '') ;;
  plan | classic) set -- "$@" --theme "$THEME" ;;
  *) die "THEME must be plan or classic, not $THEME" ;;
esac
if [ "$WALLS" = none ]; then
  set -- "$@" --no-walls
elif [ -n "$WALLS" ]; then
  for w in $WALLS; do set -- "$@" --wall "$w"; done
fi
set -- "$@" --wearer-cam-h "$R1_CAM_H" --wearer-pitch "$R1_PITCH" --xray-up "$XRAY_UP" \
  --person-h "$PERSON_H"
[ "$PITCH_AUTO" = 1 ] && set -- "$@" --pitch-auto on || set -- "$@" --pitch-auto off
for a in $HUD_ARGS; do set -- "$@" "$a"; done
launch hud_display "$@"
if ! wait_log hud_display "video: listening on tcp/" 8; then
  echo "run-live: hud_display is not listening for video; last log lines:" >&2
  tail -n 5 "$LOG_DIR/hud_display.log" >&2
  sh scripts/stop-live.sh
  exit 1
fi

# 2. hud_fusion with the pinned phone poses and unpinned starting wearer pose.
set -- "$BUILD/hud_fusion" --config "$CONFIG" --wearer-node "$WEARER"
for e in $TAG_MAP; do set -- "$@" --tag-pose "$e"; done
[ -n "$R1_HFOV" ] && set -- "$@" --hfov "R1=$R1_HFOV"
[ -n "$R2_HFOV" ] && set -- "$@" --hfov "R2=$R2_HFOV"
[ "$RANGE_HFOV" = 0 ] && set -- "$@" --no-range-hfov
set -- "$@" --person-h "$PERSON_H"
[ -n "$R1_RANGE_K" ] && set -- "$@" --range-scale "R1=$R1_RANGE_K"
[ -n "$R2_RANGE_K" ] && set -- "$@" --range-scale "R2=$R2_RANGE_K"
[ -n "$R2_CAM_H" ] && set -- "$@" --cam-h "R2=$R2_CAM_H" --cam-pitch "R2=$R2_PITCH"
[ "$GROUND_RANGE" = 1 ] && set -- "$@" --ground-range
[ "$FUSION_DEBUG" = 1 ] && set -- "$@" --debug
WEARER_PINNED=0
for p in $PINS; do
  case "$p" in "$WEARER"=*) WEARER_PINNED=1 ;; esac
  set -- "$@" --pose "$p"
done
if [ "$WEARER_MODE" = fixed ]; then
  [ "$WEARER_PINNED" = 1 ] || set -- "$@" --pose "$WEARER=$WEARER_POSE"
elif [ "$WEARER_PINNED" = 0 ]; then
  set -- "$@" --init-pose "$WEARER=$WEARER_POSE"
else
  echo "run-live: WARNING PINS pins the wearer $WEARER; its live (AprilTag) message pose will be ignored (WEARER_MODE=fixed does this on purpose)" >&2
fi
case "$R2_MODE" in
  tag) set -- "$@" --tag-pose "$R2_TAG_ID=$TAG7_POSE" --init-pose "R2=$R2_START" ;;
  tag7) set -- "$@" --tag-pose "$R2_TAG_ID=$TAG7_POSE" --tag-frame "R2=$TAG7_POSE" ;;
esac
for a in $FUSION_ARGS; do set -- "$@" "$a"; done
launch hud_fusion "$@"

# 3. The phone bridge (HTTPS + WSS for iPhone Safari).
if [ "$BRIDGE" = seeker ]; then
  set -- "$PY" -u scripts/seeker_bridge.py --ws-ingest "$WS_INGEST"
else
  set -- "$PY" -u "$WS_INGEST"
fi
set -- "$@" --port "$PORT" --tls-cert "$CERT" --tls-key "$KEY" --view-node "$WEARER"
[ "$BRIDGE" = seeker ] && set -- "$@" --video-fps "${VFPS%.*}"
[ "$BRIDGE" = seeker ] && [ "$DRAIN" = 0 ] && set -- "$@" --no-drain
for a in $BRIDGE_ARGS; do set -- "$@" "$a"; done
launch bridge "$@"

sleep 3
FAIL=0
for x in hud_display hud_fusion bridge; do
  if alive "$x"; then
    echo "run-live: $x running (pid $(cat "$LOG_DIR/$x.pid"), log $LOG_DIR/$x.log)"
  else
    FAIL=1
    echo "run-live: $x DIED; last log lines:" >&2
    tail -n 5 "$LOG_DIR/$x.log" >&2
    if grep -q -i "address already in use" "$LOG_DIR/$x.log" 2>/dev/null; then
      echo "run-live: a port is still taken (an old window still running?). sh scripts/stop-live.sh, then retry" >&2
    fi
  fi
done
if [ "$FAIL" = 1 ]; then
  sh scripts/stop-live.sh
  exit 1
fi

R2_POSE=
for p in $PINS; do
  case "$p" in R2=*) R2_POSE=${p#R2=} ;; esac
done
if [ "$WEARER_MODE" = fixed ]; then
  HELMET_LINK="https://$PHONE_HOST:$PORT/?node=$WEARER$VLINK${HFOV:+&hfov=$HFOV}"
  echo "Wearer $WEARER is FIXED at $WEARER_POSE (pinned in hud_fusion; the phone does no tag detection)."
else
  TAG_IDS=$TAG_ID
  for e in $TAG_MAP; do TAG_IDS="$TAG_IDS,${e%%=*}"; done
  HELMET_LINK="https://$PHONE_HOST:$PORT/?node=$WEARER&tag=1&tagid=$TAG_IDS&tagmm=$TAG_MM&tagw=$TAG_W$VLINK${HFOV:+&hfov=$HFOV}"
  [ -n "$TAG_MAP" ] && echo "Wearer tag map: tag $TAG_ID (origin) + $TAG_MAP. Daniel's current phone.html reads only the first id; the others need its tag_id change (progress/FELIX-asks-daniel-r2-multitag.md)."
fi
echo ""
echo "Helmet $WEARER link (copy this whole line; Safari autocomplete drops the part after ?node=..):"
echo "$HELMET_LINK"
echo ""
if [ "$R2_MODE" = tag ] || [ "$R2_MODE" = tag7 ]; then
  [ "$TAG7_POSE" = "-0.463,0.06,90" ] && echo "WARNING TAG7_POSE is the living-room value (-0.463,0.06,90). At the venue measure tag $R2_TAG_ID: sh scripts/calibrate.sh tag $R2_TAG_ID, or set TAG7_POSE=x,y,yaw. Tag 0 alone needs neither (R2_MODE=pin)."
fi
if [ "$R2_MODE" = tag ]; then
  echo "R2 link, tag mode (tag $TAG_ID first, else tag $R2_TAG_ID at room $TAG7_POSE; copy the whole line):"
  echo "https://$PHONE_HOST:$PORT/?node=R2&tag=1&tagid=$TAG_ID,$R2_TAG_ID&tagmm=$TAG_MM&tagw=$TAG_W"
  echo "  Today's phone.html reads tagid=$TAG_ID,$R2_TAG_ID as $TAG_ID only: R2 must see tag $TAG_ID until"
  echo "  Daniel's multi-tag change (progress/FELIX-asks-daniel-r2-multitag.md); then it falls back"
  echo "  to tag $R2_TAG_ID by itself. Status line: R2 ..HZ START until the first fix, HOLD when no tag."
elif [ "$R2_MODE" = tag7 ]; then
  echo "R2 link, tag7 mode (tag $R2_TAG_ID only, at room $TAG7_POSE; copy the whole line):"
  echo "https://$PHONE_HOST:$PORT/?node=R2&tag=1&tagid=$R2_TAG_ID&tagmm=$TAG_MM&tagw=$TAG_W"
  echo "  R2 must see tag $R2_TAG_ID: its HUD status reads R2 ..HZ START until the first fix, HOLD"
  echo "  when the tag drops. 2 m in front of tag $R2_TAG_ID, facing it, its page reads about"
  echo "  TAG $R2_TAG_ID 0.00,-2.00 0; hud_fusion.log then says 'NODE R2 pose from message: <room pose>'."
elif [ "$R2_MODE" = off ]; then
  echo "R1 only (R2_MODE=off): no R2 link. The HUD reads R2 -- (grey) and that is fine."
else
  echo "Still R2 link: https://$PHONE_HOST:$PORT/?node=R2${R2_POSE:+&pose=$R2_POSE}"
  echo "  R2 pinned at $R2_POSE (x < 0 is LEFT of tag 0, y < 0 in front of it, yaw 0 faces it);"
  echo "  hud_fusion.log says where that is from the wearer: grep 'node R2 is' $LOG_DIR/hud_fusion.log"
  [ "$R2_POSE" = "0,2,180" ] && echo "  WARNING R2_POSE is the default 0,2,180: 2 m BEHIND tag 0's wall, straight ahead of R1. Set R2_POSE=x,y,yaw (e.g. -1.5,-2,0 for 1.5 m left of R1) or run sh scripts/calibrate.sh pose"
fi
echo "  (raw IP links do not load on the iPhone hotspot; use $PHONE_HOST)"
echo "Room frame: origin at tag $TAG_ID ($TAG_MM mm), +y = the way you face looking straight at the"
echo "  tag, +x = your right, yaw 0 faces the tag, 90 faces right; in front of the tag y < 0."
echo "  Check: 2 m in front of the tag, facing it, the helmet page reads about TAG $TAG_ID 0.00,-2.00 0."
if [ -z "$WALLS" ] || [ "$WALLS" = none ]; then
  echo "  No walls: XRAY still works. For the wall check give WALLS=\"x1,y1,x2,y2\" in this frame"
  echo "  (the tag's own wall is y = 0, e.g. WALLS=\"-2,0,2,0\")."
fi
echo "Logs: tail -f $LOG_DIR/hud_display.log $LOG_DIR/hud_fusion.log $LOG_DIR/bridge.log"
echo "Video check: grep video: $LOG_DIR/hud_display.log $LOG_DIR/bridge.log"
echo "Latency: grep latency: $LOG_DIR/hud_display.log; grep 'ws rtt' $LOG_DIR/bridge.log"

if [ "$DETACH" = 1 ]; then
  echo "Running in the background. Stop: sh $SEEKER_DIR/scripts/stop-live.sh"
  exit 0
fi
echo "Ctrl+C stops all three."
trap 'echo ""; sh scripts/stop-live.sh; exit 0' INT TERM HUP
wait
sh scripts/stop-live.sh
