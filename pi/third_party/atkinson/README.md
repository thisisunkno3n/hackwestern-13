# Atkinson Hyperlegible (vendored)

Atkinson Hyperlegible Next (Regular 400, Medium 500, SemiBold 600) for words and Atkinson
Hyperlegible Mono (Regular 400, Medium 500) for numbers, the plan theme's text. Both are on
Google Fonts (TTFs from the fonts.googleapis.com CSS, fonts.gstatic.com) and come from
https://github.com/googlefonts/atkinson-hyperlegible-next and
https://github.com/googlefonts/atkinson-hyperlegible-next-mono. Licence: SIL Open Font License
1.1, both texts in `LICENSE.txt` next to this file.

`hud_fonts.c` holds the five faces as C arrays (about 75 KB in all) so `hud_display` needs no font
file on the Pi. It is generated, subset to what the HUD prints (U+0020 to U+007E plus the
degree, plus-minus, middle dot and multiplication signs, kern kept, hinting dropped):

```sh
python3 tools/gen_hud_fonts.py --family "..." --sans-regular Next-400.ttf --sans-medium Next-500.ttf \
  --sans-semibold Next-600.ttf --mono-regular Mono-400.ttf --mono Mono-500.ttf
```

Used by `display/draw_font.c` through `third_party/stb/stb_truetype.h`. To try another family,
run the same script with `--out` to a new folder and point `CMakeLists.txt` and
`scripts/build_pi.sh` at it; the symbol names (`hud_font_*`) stay the same.
