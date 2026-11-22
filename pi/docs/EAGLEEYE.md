# EagleEye: see people through walls from the wearer's point of view

`hud_display --view wearer` draws everyone `hud_fusion` knows about **in the wearer's own view**.
A phone taped to the helmet (node `W`) looks where the wearer looks; fixed phones (`R1`, `R2`)
watch the rooms the wearer cannot see. Fusion puts every person in the room frame (IF-1), and the
HUD projects each one into the helmet phone's image with a pinhole model.

| What you see | Meaning |
| --- | --- |
| **Solid box** + brackets, `SEEN`, range (`3.2M`) | The helmet phone sees this person too (`W` is in the track's `nodes`). Red under 2 m or closing, amber under 5 m, green beyond. |
| **Dashed cyan box**, `XRAY`, range, `VIA R1` | Only other phones see this person: behind a wall, or just not detected by the helmet phone. |
| **Chevron on the left or right edge**, `4.6M 120 BEHIND` | Outside the helmet phone's FOV. Turn that way by that many degrees; double chevron when behind (more than 90 deg). Solid = seen by the helmet, dashed cyan = other phones only. |
| Heading tape (top), `045 LIVE` / `000 FIX` | Wearer heading in the room frame; `LIVE` when it comes from the helmet phone, `FIX` when static. |
| Radar inset (bottom right) | Heading up, wearer in the middle, FOV wedge, phones, walls (`--wall`), filled square = seen, cyan diamond = x-ray. |
| `SEEN n  XRAY n  OFF n` (top right) | Counts this frame. |
| `W LOST: XRAY ONLY` (amber) | Helmet phone silent for 1.5 s; its last heading is kept and everyone shows as x-ray. |

Radar mode (`--view radar`) and the IF-7 phone-video view (`--background wearer` with no
`display.wearer` block) are unchanged and remain the fallbacks.

Files: `display/projection.{h,c}` (pure math), `display/wearer_view.{h,c}` (classification and
drawing), hooks in `display/hud_render.c` and `display/hud_display.c`, scene sim
`scripts/eagleeye_sim.py`, loopback `scripts/eagleeye_test.sh`, demo config
`config/seeker.eagleeye.json`. Poses use `fusion/seeker_pose.h` (same format as `hud_fusion`).

## Geometry

- Room frame (IF-1): origin on the wearer's floor mark, `+y` forward, `+x` right, `yaw_deg`
  clockwise from `+y`. Same as `hud_fusion --pose`.
- Camera: level phone, boresight along the wearer yaw, `f = (image_w / 2) / tan(hfov / 2)`,
  `u = cx + f * x_cam / z_cam`. That is the same `atan((x - cx) / f)` bearing phone.html reports, so
  a person the helmet phone detects lands exactly on its own detection.
- Box rows: a person 1.75 m tall standing on the floor, camera at `cam_h_m` (1.7 m). Close people
  overflow the frame top and bottom, as they do in the real video.
- Behind the wearer (`z < 0.2 m`) is never projected (no mirrored ghosts), only an edge arrow.
- Same person twice: fusion merges nodes when it can (range or triangulation). If a helmet
  sighting and an other-phone track still arrive separately within 6 deg (and 1.5 m when both have
  a range), the HUD draws one solid box and borrows the other track's range.

## Flags (`hud_display`)

| Flag | Default | Meaning |
| --- | --- | --- |
| `--view radar\|wearer` | `display.background` | `wearer` turns EagleEye on. Plain `--background wearer` (main's `config/seeker.json` and `run_demo.sh`) keeps the IF-7 phone-video view unless the config has a `display.wearer` block or a wearer flag below is given. |
| `--wearer-node ID` | `--view-node`, else `display.wearer.node`, else `display.view_node`, else `W` | The helmet phone's node id. Also becomes `view_node` (the video source). |
| `--wearer-pose [ID=]x,y,yaw` | from config | Static wearer pose; pins it (message poses ignored). `ID=` also sets the node. |
| `--pose ID=x,y,yaw` (repeatable), `--poses FILE` | none | `hud_fusion`'s format and file; the wearer node's entry pins its pose. `--pose` beats `--poses`. |
| `--wearer-pose-src auto\|static\|fusion` | `auto` (`static` once a pose is pinned) | `auto`: `hud_fusion`'s pose for the node once it has been heard (live phone pose), else static. |
| `--wearer-hfov deg` | `remote.nodes[W].hfov_deg`, else 66 | Helmet phone horizontal FOV (calibrate as in IF-1). |
| `--wearer-image WxH` | `remote.nodes[W].image_w/h`, else 1280x720 | Helmet phone frame size (aspect, and to scale its own detection boxes). Video frames override it while video is shown. |
| `--wearer-cam-h m` | 1.7 | Camera height above the floor. |
| `--wall x1,y1,x2,y2` (repeatable) | `display.wearer.walls` | Wall segments for the radar inset (drawing only). |
| `--visor` | off | Draw on black even if video frames arrive (Pepper's ghost: black is transparent). |
| `--mirror [none\|h\|v]` | `display.mirror` | Bare `--mirror` is `h`: flips the whole frame for a reflective visor so the reflection reads correctly. Applied by the Screen sink (PPM snapshots stay unflipped). |
| `--no-inset` | inset on | Hide the radar inset. |
| `--video-fallback black\|radar` | `display.wearer.video_fallback`, else `black` | Without fresh video: keep the wearer view on black (default, visor) or switch to the full radar like IF-7's video mode. `--visor` always stays on black. |

Config (`display.wearer`, Felix's `display.*` keys; all optional, see `config/seeker.eagleeye.json`):

```json
"wearer": { "node": "W", "pose": [0.0, 0.0, 0.0], "pose_source": "auto", "hfov_deg": 66.0,
            "image_w": 1280, "image_h": 720, "cam_h_m": 1.7, "assoc_deg": 6.0,
            "assoc_range_m": 1.5, "yaw_alpha": 0.35, "visor": false, "inset": true,
            "video_fallback": "black",
            "walls": [[-4.0, 2.5, 4.0, 2.5]] }
```

`yaw_alpha` smooths a live heading (weight of each new sample per frame, 1 = raw).

## Live heading from the helmet phone

`ws_ingest.py` already copies a `pose` from any phone message into every `remote_contacts`, and
`hud_fusion` uses the message pose unless the node is pinned. So a helmet phone that sends
`pose: {x_m, y_m, yaw_deg}` with a compass-derived `yaw_deg` turns the HUD live. That needs a small
phone.html change (Daniel's file): [progress/eagleeye-helmet-phone-note.md](../../../progress/eagleeye-helmet-phone-note.md).
Until then the heading is the static pose (`FIX`): the wearer faces room `+y`.

For a live heading, **do not pin `W` in `hud_fusion`** (no `--pose W=...`); give its position in
`remote.nodes[]` or `display.wearer.pose` instead.

## Video

With IF-5.6 video on TCP 47106 (`display/video_rx.c`, [VIDEO.md](VIDEO.md)), the wearer view draws
over the newest fresh frame: the projection uses the frame's decoded size (`src_w x src_h`) and
its letterbox rectangle, so a 4:3 phone frame on the 16:9 monitor still lines up. Rules:

- Only frames whose `node_id` is the wearer node are used (`--view wearer` sets `view_node` to
  it, so the relay forwards the helmet phone). The receiver is pinned to that node: a frame from
  `R1` would put the boxes in the wrong place, so it is skipped before decoding and never
  replaces or refreshes the wearer's picture. `hud_display` logs `video: frames from R1 skipped:
  this HUD shows only W's video` once, and the 5 s `video:` stats line counts them.
- The picture stays up through short gaps: after 500 ms without a frame it is dimmed with a
  small `VIDEO STALE n.n S` tag, and only after 2 s is the video `LOST` (radar fallback or
  black). Phone video over Wi-Fi arrives in bursts; a 500 ms cut-off made the monitor flap
  between the picture and the radar.
- No video, or `--visor`: black background, no `NO VIDEO` banner (that is the visor mode).
  `VIDEO LOST` (a stream that stopped) still shows unless `--visor`. This differs from IF-7's
  "radar when video is absent" on purpose: x-ray symbology on black is the visor HUD. For the
  monitor demo, `--video-fallback radar` restores the IF-7 behaviour (full radar, `NO VIDEO` /
  `VIDEO LOST` banners) whenever there is no fresh frame.
- `hud_display` logs `video on|off|other-node` on its `eagleeye:` line, then `skel N`: XRAY
  people drawn as a ghost skeleton from the phone's `pts` (smoothed in `hud_fusion`,
  `fusion/seeker_skel.h`; see docs/LIVE-RUN.md "Ghost skeletons").

Test card instead of a phone: `python3 scripts/video_send.py --node W --size 1280x720`.

## Phone mounting (helmet phone `W`)

1. Landscape, back camera facing straight ahead, level (no pitch or roll). Tape it to the helmet
   front or side; the closer to the eyes, the less parallax.
2. Open phone.html with `?node=W&hfov=<calibrated>&pose=0,0,0` (wearer starts on the floor mark
   facing room `+y`). Calibrate `hfov` like R1 and R2 (IF-1: a person at a measured 30 deg).
3. Stand on the floor mark, face room `+y` (the direction used for R1 and R2 poses).
4. With live heading (after the phone change): tap "zero heading" while facing `+y`.
5. Fixed phones: measure `x_m, y_m, yaw_deg` from the floor mark and use the same numbers in
   `?pose=` and `remote.nodes[]` (or `hud_fusion --pose`).

## Demo script (60 s inside the main 2-minute demo)

Setup: wearer `W` on the floor mark facing the partition (room `+y`), `R1` in the space behind the
partition looking across it, `R2` behind the wearer. Monitor shows `hud_display --view wearer`
(or the visor with `--visor --mirror`). Daniel is the hidden person.

| Time | Do | Say |
| --- | --- | --- |
| 0:00 | Daniel stands in front of the wearer | "The helmet camera sees him: solid box, range, red because he's close." |
| 0:10 | Daniel walks behind the partition | (dashed cyan `XRAY`, `VIA R1`) "Now he's behind the wall. The phone back there sees him, the Pi fuses it, and the helmet draws him where he really is, through the wall." |
| 0:25 | Daniel walks along behind the wall | "The x-ray box tracks him from my point of view, at the right distance." |
| 0:35 | Volunteer walks behind the wearer | (edge chevron, `BEHIND`) "Someone behind me: an arrow tells me which way to turn and how far." |
| 0:45 | Wearer turns toward the arrow (live heading) or the operator flips `--wearer-pose` | "Turn, and he's in view." |
| 0:55 | `slay hud_fusion` (optional) | "Kill fusion: the HUD keeps drawing, greys out, and comes back in a second." |

Fallback: `--view radar` (full-screen map), or video plus radar inset as before.

## Run it

Host or Pi, from `drafts/pi-lane` (Pi: `cc` is clang, no `qcc`; `scripts/build_pi.sh` picks `cc`):

```bash
scripts/build_pi.sh                     # or cmake (PI-BUILD.md)
build/test_core                          # "352 passed, 0 failed"
scripts/eagleeye_test.sh                 # EagleEye loopback, "0 failure(s)"; snapshots in logs/eagleeye/
```

Sim demo (no phones):

```bash
build/hud_fusion --config config/seeker.eagleeye.json &
build/hud_display --config config/seeker.eagleeye.json --view wearer --sink screen &
python3 scripts/eagleeye_sim.py --wearer-turn 40      # helmet sweeps +-40 deg
python3 scripts/video_send.py --node W --size 1280x720 # optional: test-card video under it
# frozen scenes: --at 0 (seen + x-ray), --at 8 (behind), --at 8 --wearer-yaw 100 (turned)
# helmet phone off: --nodes R1,R2
```

Real phones (Daniel's relay on the Pi, `remote/ws_ingest.py` on :8080), then:

```bash
sudo build/hud_supervisor --config config/seeker.eagleeye.json   # fusion + wearer view
# visor: build/hud_display --config config/seeker.eagleeye.json --view wearer --visor --mirror
# fallback: build/hud_display --config config/seeker.eagleeye.json --view radar
```

Phones: helmet `https://qnxpi-felix.local:8080/?node=W&hfov=..&pose=0,0,0`, `R1`
`https://qnxpi-felix.local:8080/?node=R1&pose=-3,5.5,104`, `R2`
`https://qnxpi-felix.local:8080/?node=R2&pose=2.5,-2.5,315` (or your measured poses; keep
`remote.nodes[]` in sync). Use the `.local` name: raw IPs did not load on the iPhone hotspot.
For the two-phone live run (helmet `R1` with video), use `scripts/run-live.sh` and the links in
[LIVE-RUN.md](LIVE-RUN.md).

## Limits (host-tested only)

- Nothing here has run on QNX yet; Felix verifies on `qnxpi-felix.local`.
- Pitch and roll are assumed zero; a tilted helmet shifts boxes vertically (horizontal placement,
  which matters most, only needs yaw).
- The wearer's position is static unless the phone sends one; only the heading is expected live.
- `--wall` is drawn on the inset only; occlusion is decided by what the helmet phone detects, not
  by geometry.
