#!/bin/sh
# Build without CMake (for the Pi if cmake is not installed). Run from drafts/pi-lane.
#   scripts/build_pi.sh            # on the Pi: uname says QNX, links Screen + libsocket
#   CC=gcc scripts/build_pi.sh     # on a Linux or macOS host: no Screen sink (use --sink ppm|none)
# Output goes to build/. Compiler: $CC, else qcc, cc, gcc (first found).
#
# On QNX, BSD sockets are in libsocket (QNX SDP 8.0 io-sock; link libsocket.so,
# there is no io-sock libsocket.a):
#   https://www.qnx.com/developers/docs/8.0/com.qnx.doc.qnxsdp.migration/topic/migrate_app.html
# _QNX_SOURCE is required so QNX libc extensions stay visible under -std=c11:
#   https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.prog/topic/devel_ConformingToStandards.html
# Native QNX gcc should predefine __QNX__ (800 on SDP 8.0):
#   https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.lib_ref/topic/manifests.html
# If this gcc does not, we add -D__QNX__=800. A host build is not a QNX test
# pass; nothing counts as tested until it runs on the QNX Pi.
set -eu
OUT=${OUT:-build}
mkdir -p "$OUT"
if [ -z "${CC:-}" ]; then
  for c in qcc cc gcc; do
    if command -v "$c" >/dev/null 2>&1; then CC=$c; break; fi
  done
fi
[ -n "${CC:-}" ] || { echo "no C compiler found (tried qcc, cc, gcc)"; exit 1; }

FLAGS="-std=c11 -O2 -Wall -Wextra -D_POSIX_C_SOURCE=200809L -Icommon -Ifusion -Idisplay"
LIBS="-lm"
HUD_SINK=display/sink_noscreen.c
if [ "$(uname -s)" = "QNX" ]; then
  FLAGS="$FLAGS -D_QNX_SOURCE"
  LIBS="$LIBS -lsocket"
  HUD_SINK="display/sink_screen.c"
  HUD_LIBS="-lscreen"
  # Prefer the official SDP header; use the documented fallback only if absent.
  if [ -n "${QNX_TARGET:-}" ] && [ -f "$QNX_TARGET/usr/include/screen/screen.h" ]; then
    echo "using official screen.h at $QNX_TARGET/usr/include/screen/screen.h"
  elif [ -f /usr/include/screen/screen.h ]; then
    echo "using official screen.h at /usr/include/screen/screen.h"
  elif [ -f /system/include/screen/screen.h ]; then
    FLAGS="$FLAGS -I/system/include"
    echo "using official screen.h at /system/include/screen/screen.h"
  else
    FLAGS="$FLAGS -Ithird_party/qnx-screen"
    echo "note: no official screen.h on this sysroot; using third_party/qnx-screen fallback"
  fi
  # Confirm the compiler predefines __QNX__. printf and a stdin compile are
  # used because they are portable; this image already has gcc and a POSIX shell.
  if ! printf '%s\n' '#ifdef __QNX__' 'yes' '#endif' | $CC $FLAGS -E -P - 2>/dev/null | grep -q yes; then
    echo "warning: $CC did not predefine __QNX__; adding -D__QNX__=800 (QNX 8.0 manifest)"
    FLAGS="$FLAGS -D__QNX__=800"
  fi
else
  LIBS="$LIBS -lpthread"
  [ "$(uname -s)" = "Linux" ] && LIBS="$LIBS -lrt"
  HUD_LIBS=""
fi
COMMON="common/seeker_time.c common/seeker_log.c common/seeker_json.c common/seeker_msgs.c \
common/seeker_config.c common/seeker_udp.c common/seeker_health.c common/seeker_colour.c \
common/seeker_posture.c"
FUSION="fusion/seeker_fusion.c fusion/seeker_pose.c fusion/seeker_triangulate.c \
fusion/seeker_assign.c fusion/seeker_skel.c"
VIDEO="display/video_proto.c display/video_decode.c display/video_rx.c display/latency.c"
# Plan theme: drafting primitives, Atkinson Hyperlegible compiled in (third_party/atkinson, made by
# tools/gen_hud_fonts.py) and drawn with stb_truetype.
DRAW="display/draw.c display/draw_font.c display/plan.c third_party/atkinson/hud_fonts.c"

echo "CC=$CC  target=$(uname -s)  out=$OUT/"
set -x
$CC $FLAGS -o "$OUT/udp_recv" tools/udp_recv.c $COMMON $LIBS
$CC $FLAGS -o "$OUT/hud_fusion" fusion/hud_fusion.c $FUSION $COMMON $LIBS
$CC $FLAGS -o "$OUT/hud_supervisor" supervisor/hud_supervisor.c $COMMON $LIBS
$CC $FLAGS -Isupervisor -o "$OUT/seeker_supervisor" supervisor/seeker_supervisor.c \
  supervisor/seeker_sv.c fusion/seeker_pose.c $COMMON $LIBS
$CC $FLAGS -Isupervisor -o "$OUT/test_supervisor" tests/test_supervisor.c supervisor/seeker_sv.c
$CC $FLAGS -o "$OUT/hud_display" display/hud_display.c display/hud_render.c $DRAW \
  display/sink.c display/projection.c display/wearer_view.c fusion/seeker_pose.c $HUD_SINK \
  $VIDEO $COMMON $LIBS $HUD_LIBS
$CC $FLAGS "-DSEEKER_SOURCE_DIR=\"$(pwd)\"" -o "$OUT/test_core" tests/test_core.c $FUSION \
  display/projection.c display/wearer_view.c $DRAW display/sink.c display/sink_noscreen.c \
  display/hud_render.c $VIDEO $COMMON $LIBS
$CC $FLAGS "-DSEEKER_SOURCE_DIR=\"$(pwd)\"" -o "$OUT/sim_smooth" tests/sim_smooth.c $FUSION \
  display/projection.c display/wearer_view.c $DRAW display/sink.c display/sink_noscreen.c \
  display/hud_render.c $VIDEO $COMMON $LIBS
