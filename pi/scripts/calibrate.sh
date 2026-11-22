#!/bin/sh
# Calibrate the two fixed phones for WEARER_MODE=fixed + R2_MODE=pin, with tag 0 only (tag 7 is
# not needed). Runs the phone bridge alone (no hud_fusion: scripts/calib_listen.py takes UDP
# 47103), prints the phone links for the step, and writes $LOG_DIR/calib.env, which
# scripts/run-live.sh reads (a variable given on the command line still wins).
# POSIX sh: QNX 8.0 sh (pdksh) and Linux sh. docs/LIVE-RUN.md "Morrissette calibration".
#
#   sh scripts/calibrate.sh pose        # both phones at their demo spots, tag 0 in view: x,y,yaw
#   sh scripts/calibrate.sh aim X,Y     # then aimed at the scene, one person on the taped mark
#                                       # X,Y (m from tag 0): each phone's yaw from their bearing
#   sh scripts/calibrate.sh yaw [R2]    # a pinned phone's yaw only, from where tag 0 sits in its
#                                       # picture; its x,y stay R2_POSE's (measure those)
#   sh scripts/calibrate.sh tag N       # moving wearer: measure tag N (1..9) against tag 0 with
#                                       # R1 held still where it sees both (TAG_MAP)
#   sh scripts/calibrate.sh show        # print calib.env
#
# Env: as run-live.sh (SEEKER_DIR, WS_INGEST, CERT, KEY, LOG_DIR, PORT, PHONE_HOST, TAG_MM,
# TAG_W, PYTHON), SECONDS_CAL (15), R1_HFOV / R2_HFOV (degrees or iphone17pro, iphone16,
# iphone-ultrawide: used to solve the yaw in the aim step).
set -u
SEEKER_DIR=${SEEKER_DIR:-$HOME/seeker/drafts/pi-lane}
WS_INGEST=${WS_INGEST:-$HOME/seeker/remote/ws_ingest.py}
CERT=${CERT:-$HOME/cert.pem}
KEY=${KEY:-$HOME/key.pem}
LOG_DIR=${LOG_DIR:-$HOME/seeker-logs}
PORT=${PORT:-8080}
PHONE_HOST=${PHONE_HOST:-qnxpi-felix.local}
TAG_MM=${TAG_MM:-186}
TAG_W=${TAG_W:-640}
SECONDS_CAL=${SECONDS_CAL:-15}
PY=${PYTHON:-python3}
CALIB=${CALIB:-$LOG_DIR/calib.env}
export LOG_DIR

die() { echo "calibrate: $*" >&2; exit 2; }
MODE=${1:-}
case "$MODE" in
  pose) ;;
  aim) [ -n "${2:-}" ] || die "aim needs the person's mark: sh scripts/calibrate.sh aim X,Y" ;;
  tag) case "${2:-}" in [1-9]) ;; *) die "tag needs an id 1..9: sh scripts/calibrate.sh tag 1" ;; esac ;;
  yaw) case "${2:-R2}" in R1|R2) ;; *) die "yaw takes R1 or R2" ;; esac ;;
  show) cat "$CALIB" 2>/dev/null || echo "calibrate: no $CALIB yet"; exit 0 ;;
  *) die "usage: sh scripts/calibrate.sh pose | aim X,Y | yaw [R2] | tag N | show" ;;
esac
cd "$SEEKER_DIR" 2>/dev/null || die "no $SEEKER_DIR"
[ -f "$WS_INGEST" ] || die "no bridge at $WS_INGEST (set WS_INGEST)"
[ -f "$CERT" ] && [ -f "$KEY" ] || die "no TLS cert/key at $CERT and $KEY"
mkdir -p "$LOG_DIR" || die "cannot create $LOG_DIR"

sh scripts/stop-live.sh >/dev/null 2>&1
"$PY" -u scripts/seeker_bridge.py --ws-ingest "$WS_INGEST" --port "$PORT" --tls-cert "$CERT" \
  --tls-key "$KEY" --view-node R1 --no-video-ctrl >>"$LOG_DIR/calibrate-bridge.log" 2>&1 &
BR=$!
trap 'kill "$BR" 2>/dev/null; wait "$BR" 2>/dev/null' EXIT INT TERM

L="https://$PHONE_HOST:$PORT"
echo ""
if [ "$MODE" = tag ]; then
  echo "Tag $2 against tag 0: hold R1 still (on a stand) where tag 0 and tag $2 are both in view."
  echo "  First:  $L/?node=R1&tag=1&tagid=0&tagmm=$TAG_MM&tagw=$TAG_W"
  echo "  Then (when asked, phone not moved): $L/?node=R1&tag=1&tagid=$2&tagmm=$TAG_MM&tagw=$TAG_W"
  echo "  Press Enter when the pill reads TAG 0."
  read -r _ || true
  "$PY" -u scripts/calib_listen.py tag --id "$2" --nodes R1 --seconds "$SECONDS_CAL" --out "$CALIB"
  RC=$?
  cat "$CALIB" 2>/dev/null
  exit "$RC"
fi
if [ "$MODE" = yaw ]; then
  NODE=${2:-R2}
  echo "$NODE's yaw: keep it on its spot, aimed as for the demo, with tag 0 fully in its picture."
  echo "  $NODE: $L/?node=$NODE&tag=1&tagid=0&tagmm=$TAG_MM&tagw=$TAG_W"
  echo "  Its x,y stay as measured (R2_POSE / WEARER_POSE). Press Enter when the pill reads TAG 0."
  read -r _ || true
  "$PY" -u scripts/calib_listen.py yaw --nodes "$NODE" --seconds "$SECONDS_CAL" --out "$CALIB"
  RC=$?
  cat "$CALIB" 2>/dev/null
  exit "$RC"
fi
if [ "$MODE" = pose ]; then
  echo "Step 1 of 2: each phone on its fixed demo spot, tag 0 fully in its picture, held still."
  echo "  R1: $L/?node=R1&tag=1&tagid=0&tagmm=$TAG_MM&tagw=$TAG_W"
  echo "  R2: $L/?node=R2&tag=1&tagid=0&tagmm=$TAG_MM&tagw=$TAG_W"
  echo "  The page's pill must read TAG 0 ... before the count starts. Press Enter when both do."
else
  echo "Step 2: phones aimed at the scene (tag 0 may be out of view), ONE person on the mark $2."
  echo "  R1: $L/?node=R1     R2: $L/?node=R2"
  echo "  Both pages must show the person. Press Enter when they do."
fi
read -r _ || true
if [ "$MODE" = pose ]; then
  "$PY" -u scripts/calib_listen.py pose --seconds "$SECONDS_CAL" --out "$CALIB"
else
  "$PY" -u scripts/calib_listen.py aim --mark "$2" --seconds "$SECONDS_CAL" --out "$CALIB"
fi
RC=$?
echo ""
cat "$CALIB" 2>/dev/null
[ "$RC" -eq 0 ] && echo "calibrate: done. Start: WEARER_MODE=fixed sh scripts/run-live.sh" ||
  echo "calibrate: incomplete (see above); fix and run the step again" >&2
exit "$RC"
