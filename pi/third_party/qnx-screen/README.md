# Fallback QNX Screen header

This directory is a compile-time stand-in for `<screen/screen.h>` on Felix's
QNX 8.0 Raspberry Pi 5 image. That image already has `libscreen.so` in
`/usr/lib` and `/system/lib`, but `find / -name screen.h` returned nothing,
and there is no QNX SDP 8.0 install on the Mac
(`~/qnx800/target/qnx/usr/include/screen/` is missing).

## This is not the official header

The official `screen.h` is part of the QNX Software Development Platform and
is not published as source. These files are original declarations for this
project. They cover only the types, constants, and functions that
`display/sink_screen.c` actually calls. Constant values and prototypes are
cited from the public QNX SDP 8.0 documentation:

- https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_8h_1Screen_Property_Types.html
- https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_8h_1Screen_Pixel_Format_Types.html
- https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/screen_8h_1Screen_Usage_Flag_Types.html
- https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/group__screen__contexts_1Screen_Context_Types.html
- https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/group__screen__events_1Screen_Event_Types.html
- Function and type pages under https://www.qnx.com/developers/docs/8.0/com.qnx.doc.screen/topic/

Public official sources checked that do **not** ship the header:

- https://gitlab.com/qnx (ports and samples `#include <screen/screen.h>` but
  do not contain the header; optional packages in QNX Software Center are
  the documented way to get compiled open-source headers)
- https://github.com/qnx-ports and https://github.com/qnx (same: consume
  Screen, do not publish `screen.h`)
- QNX Everywhere / SDP docs: the header comes with the SDP Screen packages

If the official header later appears on the sysroot or in `$QNX_TARGET`,
the build will use it and ignore this directory.

## How the build picks a header

`CMakeLists.txt` (when `SEEKER_TARGET=qnx`) and `scripts/build_pi.sh` (when
`uname` is QNX) look for, in order:

1. `$QNX_TARGET/usr/include/screen/screen.h`
2. `/usr/include/screen/screen.h`
3. `/system/include/screen/screen.h`

Only if all three are missing do they add `-I third_party/qnx-screen`.

`host-stub/` is a separate laptop fake (different constant values, malloc
buffers). It must not be on the include path of a QNX build.

`display/sink_screen.c` builds with `-DSCREEN_NO_SOURCE_SIZE` against a header
that lacks `SCREEN_PROPERTY_SOURCE_SIZE`; `--screen-buffer` then logs that it
is unsupported and draws at the display size.

## Unverified items

None of the constant values or function prototypes declared in
`screen/screen.h` are unverified. Each is taken from the official QNX 8.0
doc pages listed above (or linked from comments in the header).

Still unverified until run on the Pi (and therefore not a test pass):

- That QNX's `/usr/bin/gcc` on this image predefines `__QNX__` (the
  official manifests page says the compiler does, with value 800 on SDP
  8.0). `build_pi.sh` checks and, only if missing, adds `-D__QNX__=800`.
- That linking `-lscreen` against `/usr/lib/libscreen.so` with these
  declarations produces a running HUD (ABI match is expected because
  values and prototypes come from the same 8.0 docs the library ships
  with, but that is not a substitute for a Pi run).
- HDMI index, RGBX byte order, and z-order, already noted in
  `display/sink_screen.c`.

## Testing rule

Nothing counts as tested until it runs on the QNX Pi. A Linux or macOS
host compile against `host-stub/` is only a compile check.
