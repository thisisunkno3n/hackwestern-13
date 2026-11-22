#!/bin/sh
# Plan map animation clips (display/anim_map.h) from tools/hud_anim_map_shot, host only.
#   tools/anim_map_media.sh build/hud_anim_map_shot room.ppm out/
# Writes out/frames/<seq>/f*.png and, with ffmpeg, out/<seq>.gif: appear, down and scan cropped
# to the corner plan (shown at 2x), drawin and boot as the whole frame at 960 wide.
set -eu
[ $# -eq 3 ] || { echo "usage: $0 hud_anim_map_shot room.ppm outdir"; exit 2; }
TOOL=$1 BG=$2 OUT=$3
mkdir -p "$OUT/frames"
"$TOOL" --bg "$BG" --out "$OUT/frames" --seq all
if ! command -v ffmpeg >/dev/null 2>&1; then
  echo "no ffmpeg: frames only, in $OUT/frames"
  exit 0
fi
gif() { # seq filter
  ffmpeg -v error -y -framerate 30 -i "$OUT/frames/$1/f%04d.png" \
    -filter_complex "$2,split[a][b];[a]palettegen=stats_mode=full[p];[b][p]paletteuse=dither=none" \
    "$OUT/$1.gif"
  echo "wrote $OUT/$1.gif"
}
INSET="crop=272:206:iw-285:ih-222,scale=iw*2:ih*2:flags=neighbor"
for s in appear down scan; do gif "$s" "$INSET"; done
for s in drawin boot; do gif "$s" "scale=960:-1:flags=lanczos"; done
