# Video on hud_display (IF-5.6, TCP 47106)

Phone video in the HUD: `hud_display --background wearer` draws the relay's JPEG frames full
screen (letterboxed), with track markers on top and a radar inset bottom right. With no fresh
frame it falls back to the full radar under a `NO VIDEO` or `VIDEO LOST` banner.

**Host-tested only (x86 Linux, clang 18 and gcc 13).** Nothing here counts as passed until
Felix runs it on `qnxpi-felix.local` (QNX 8.0, Pi 5).

## Wire format

Exactly IF-5.6 in [INTERFACES.md](../INTERFACES.md#if-56-video_frame-new-relay---display-local-tcp):
a stream of frames, each a 36-byte big-endian header followed by `payload_len` bytes of JPEG.
No other bytes go between frames.

| Off | Size | Field | Value |
| --- | --- | --- | --- |
| 0 | 4 | `magic` | ASCII `SKV2` |
| 4 | 1 | `v` | `2` |
| 5 | 1 | `flags` | `0` (baseline JPEG) |
| 6 | 2 | `reserved` | `0` |
| 8 | 8 | `node_id` | ASCII, NUL padded (`R1`) |
| 16 | 8 | `ts_ms` | sender's monotonic ms (logged, never compared, IF-1 clock rule) |
| 24 | 2 | `width` | px |
| 26 | 2 | `height` | px |
| 28 | 4 | `seq` | per node, counts up |
| 32 | 4 | `payload_len` | 1 to 2097152 |
| 36 | n | JPEG | `FF D8 ... FF D9`, baseline or progressive, any chroma subsampling |

Python sender side (what `ws_ingest.py` needs; `scripts/video_send.py` does exactly this):

```python
import socket, struct, time
HDR = struct.Struct(">4sBBH8sQHHII")
s = socket.create_connection(("127.0.0.1", 47106))
s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
s.sendall(HDR.pack(b"SKV2", 2, 0, 0, b"R1".ljust(8, b"\0"), int(time.monotonic() * 1000),
                   640, 480, seq, len(jpeg)) + jpeg)
```

## What the receiver does

- Listens on TCP 47106 on `0.0.0.0` and `[::]` (two sockets, IPv6 with `IPV6_V6ONLY=1`, like
  the UDP listeners; IPv4 only if the stack refuses IPv6). `127.0.0.1` from `ws_ingest.py` on
  the Pi is the main path; a Mac backup relay can send over the LAN or IPv6 link-local.
- One relay at a time. A new connection replaces the old one (log: `replaced by`).
- A receiver thread drains everything readable, keeps only the newest complete frame, then
  decodes that one with vendored `stb_image` (no system libraries) and scales it (nearest
  neighbour, aspect kept) into a canvas-sized buffer. Older complete frames are counted as
  `dropped`, never queued.
- The draw loop calls `video_rx_latest`, which only try-locks and swaps pointers in a triple
  buffer, so a slow decode or a stuck socket can never delay a HUD frame.
- Bad magic, `v` other than 2, or `payload_len` outside 1 to 2 MB: logged
  (`WARNING unknown SKV2 v`, `lost sync`) and the parser scans forward for the next `SKV2`.
  A frame that fails to decode is logged and skipped.
- Receiver thread priority: `SCHED_RR` at the HUD's priority minus 10 (40 under the
  supervisor's 50 FIFO: below the draw loop, above `hud_fusion` 30 and `hud_stress` 10).
  `--video-prio N` overrides; `0` inherits.

| State | When | Wearer view shows |
| --- | --- | --- |
| `VIDEO OK` | connected and a frame from this connection decoded in the last 500 ms | video, markers, radar inset, `VID 15FPS` |
| `VIDEO STALE` | the newest frame is 0.5 to 2 s old (phone video over Wi-Fi comes in bursts) | the last picture, dimmed, markers on top, small amber `VIDEO STALE 1.2 S` tag, `VID STALE` |
| `NO VIDEO` | no relay connected | full radar, `NO VIDEO` banner, `VID --` |
| `VIDEO LOST` | connected but nothing decodable for 2 s (or nothing yet from a new relay) | full radar, `VIDEO LOST` banner |

The 2 s hold (`VIDEO_HOLD_MS`, `display/video_rx.h`) is on purpose: with a 500 ms cut-off the
helmet phone's bursty video (ArUco and pose detection on the phone, Wi-Fi) flipped the monitor
between the picture and the radar several times a second. In the wearer view (`--view wearer`)
the receiver also takes only the wearer node's frames (`video: wearer view pinned to R1's
video`); another phone's frames are skipped before decoding and counted in the 5 s `video:`
stats line, so they can never replace or refresh R1's picture.
| `VIDEO OFF` | started with `--no-video` | full radar, `VIDEO OFF` banner |

The radar background (`--background radar`, still the config default) ignores video apart
from the `VID` field in the status line.

**Knowing a frame was drawn:** `hud_display` logs `video: frame drawn on the HUD: R1 seq N
640x480` when the picture first appears in a rendered frame, and `video: picture no longer
drawn (video none)` when it goes. Every PPM snapshot carries the same facts as a header
comment, `# seeker hud frame=42 video=ok drawn=1 node=R1 seq=17` (a valid PPM comment; image
viewers and ffmpeg skip it). `scripts/video_loopback_test.sh` polls for the note it needs, with
a timeout, instead of sleeping: the receiver notices a closed relay within one poll, so a
snapshot copied after the sender exits shows the radar fallback, not the video.

**Marker alignment:** brackets come from `node_box`, which is assumed to be in the same pixel
space as the JPEG on screen (true when `phone.html` streams the frames it detects on). If the
relay ever scales video differently from detections, `hud_display` will need IF-5.2 `node_w` /
`node_h` (in the contract, not yet in `seeker_track`) to scale boxes. Horizon diamonds use the
canonical `x = cx + f * tan(bearing)` inside the picture rectangle.

## Files

| File | What |
| --- | --- |
| `display/video_proto.[ch]` | SKV2 header codec and streaming parser (pure, unit tested) |
| `display/video_decode.[ch]` | JPEG decode (stb_image) and letterbox scale |
| `display/video_rx.[ch]` | TCP listener thread, triple buffer, `VIDEO_*` state |
| `display/hud_render.c`, `display/hud_display.c` | draw video plus symbology; start and stop the receiver |
| `third_party/stb/` | vendored `stb_image.h` (and `stb_image_write.h` for tests only) |
| `scripts/video_send.py` | test sender: built-in test card or your JPEG files, no dependencies |
| `scripts/video_loopback_test.sh` | sender to hud_display end to end, PPM pixel checks |

## Run it

```bash
build/hud_display --sink screen --background wearer          # on the Pi
python3 scripts/video_send.py                                  # same machine: test card 640x480 @ 15 fps
python3 scripts/video_send.py --host qnxpi-felix.local --size 1280x720   # from the Mac
python3 scripts/video_send.py --host 'fe80::...%en0'           # Mac over IPv6 link-local
python3 scripts/video_send.py --jpeg shot1.jpg shot2.jpg --fps 30         # real JPEGs
scripts/video_loopback_test.sh                                 # automated, ppm sink, prints ALL PASS
SENDER_EXIT_DELAY=0.5 FIRST_FRAME_DELAY=1 scripts/video_loopback_test.sh   # slow-Pi timing
```

The test card has red top left, green top right, blue bottom left, white bottom right. If red
and blue are swapped on HDMI, set `display.swap_rb: true`.

hud_display flags added: `--video-port N` (default 47106), `--video-prio N`, `--no-video`.

## Host numbers (x86, not the Pi)

1920x1080 canvas, `--sink none`: HUD 30.3 fps with video. Decode plus scale per frame: about
6 ms for 1280x720 and 6 ms for 640x480 test cards (real camera JPEGs have more detail and will
be slower). Flooding 200 fps of 1280x720 kept the HUD at 30.3 fps; the receiver decoded about
166 fps and counted the rest as dropped.

## Not done yet

- Daniel's `ws_ingest.py` forwards video since `fb232e1` (on `main` from `a74c626`), and
  `phone.html` captures it with `&video=1` since `24686f3`. It only connects to TCP 47106 after
  the phone's first frame, and never turns the phone's video on. For live runs use
  `scripts/run-live.sh`, which wraps it with `scripts/seeker_bridge.py`; the HUD banner and logs
  say why there is no video. See [docs/LIVE-RUN.md](docs/LIVE-RUN.md) and the proposed fix in
  [docs/DANIEL-VIDEO-PATCH.md](docs/DANIEL-VIDEO-PATCH.md).
- No `display.video_port` config key is read yet (`--video-port` instead), to keep this change
  out of `common/`.
- Untested on QNX: `stdatomic.h`, `pthread_attr_setschedpolicy(SCHED_RR)` for the receiver,
  `TCP_NODELAY` from `<netinet/tcp.h>`, and NEON decode speed on the Pi 5.
