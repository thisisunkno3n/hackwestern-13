# stb (vendored)

Single-header libraries by Sean Barrett, copied unmodified from
https://github.com/nothings/stb at commit `6b5799bae8a1fe3792691ef4b72ae26434c3e799`
(2026-11-19). Each file ends with its license: public domain (Unlicense) or MIT, your choice.

| File | Version | Used by |
| --- | --- | --- |
| `stb_image.h` | v2.34 | `display/video_decode.c` (hud_display JPEG decode). Built with `STBI_ONLY_JPEG`, `STBI_NO_STDIO`, `STBI_NO_THREAD_LOCALS`, `STBI_MAX_DIMENSIONS 4096`. NEON is used automatically on aarch64 (Pi 5). |
| `stb_truetype.h` | v1.28 | `display/draw_font.c` (plan theme text, Atkinson Hyperlegible in `third_party/atkinson`). One em dash in a comment (line 1903) is replaced with `-` for the repo's no-dash rule; otherwise unmodified. |
| `stb_image_write.h` | v1.19 | `tests/test_core.c` only, to make test JPEGs in memory. Never linked into hud_display. |

No system libraries are needed (no libjpeg or libturbojpeg). If decode is too slow on the Pi,
`libturbojpeg.so.0` is on the image (Daniel found it, progress/ws6-daniel.md) and could replace
`video_jpeg_decode` behind the same function.
