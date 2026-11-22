# Live run on the Pi: one command, phone links, and what every video message means

Everything runs on `qnxpi-felix.local` (QNX 8.0), from `~/seeker/drafts/pi-lane`.

## Start and stop

```sh
cd ~/seeker/drafts/pi-lane && sh scripts/run-live.sh           # Ctrl+C stops all three
DETACH=1 sh scripts/run-live.sh                                # or: keep running after logout
sh scripts/stop-live.sh                                        # stops a run, plus strays started by hand
```

`run-live.sh` first runs `stop-live.sh` (old copies of `hud_display`, `hud_fusion` and any
`ws_ingest.py` holding port 8080 are stopped, `slay -f` on QNX). It then starts, in this order:

| Name | Command | Log |
| --- | --- | --- |
| `hud_display` | `build/hud_display --config config/seeker.live.json --view wearer --wearer-node R1 --wearer-pose R1=0,-2,0 --wearer-pose-src auto --sink screen --video-fallback radar` | `~/seeker-logs/hud_display.log` |
| `hud_fusion` | `build/hud_fusion --config config/seeker.live.json --debug --pose R2=0,2,180 --init-pose R1=0,-2,0` | `~/seeker-logs/hud_fusion.log` |
| `bridge` | `python3 -u scripts/seeker_bridge.py --ws-ingest ~/seeker/remote/ws_ingest.py --port 8080 --tls-cert ~/cert.pem --tls-key ~/key.pem --view-node R1` | `~/seeker-logs/bridge.log` |

The terminal shows all three logs (each line starts with `[hud_display]`, `[hud_fusion]`,
`[ws_ingest]` or `[seeker_bridge]`). Each process also logs to its file through `tee`.
Everything is configurable by environment variable (see the header of `scripts/run-live.sh`),
for example `R2_POSE="2.5,1.5,-90"`, `R2_MODE=tag` with `TAG7_POSE="-0.463,0.06,90"` (R2 on
tag 0, else tag 7, [below](#r2-on-tag-0-and-tag-7-r2_modetag)), `WEARER_POSE="0,-2,0"`, `WALLS="-2,0,2,0"`, `TAG_MM=186`,
`TAG_ID=0`, `TAG_W=640`, `HFOV=66`, `WEARER=R1`, `SINK=ppm`, or `BRIDGE=daniel` to run
`ws_ingest.py` without our wrapper. `config/seeker.live.json` has only `R1` and `R2` (640x480, no
walls); the simulator's `config/seeker.eagleeye.json` (with its `W` node) is for
`eagleeye_test.sh` only.

`seeker_bridge.py` runs Daniel's `ws_ingest.py` unchanged, plus four things:

1. It sends `video_ctrl` to the helmet phone, so video streams even if the URL lost `&video=1`.
2. It connects to `hud_display` at once, so the HUD can tell "no bridge" from "phone not
   sending".
3. It prints a status line per phone every 10 s, with the pose the phone sends.
4. It forwards the phone's `pose: null` to `hud_fusion`. Plain `ws_ingest.py` re-sends the last
   pose instead, so the HUD would say `POSE LIVE` while the tag is out of view.
   `--keep-ingest-pose` (in `BRIDGE_ARGS`) turns this off.

The same fixes are proposed to Daniel in [DANIEL-VIDEO-PATCH.md](DANIEL-VIDEO-PATCH.md).

## Phone links

Use the `.local` name: raw IP links (`172.20.10.x`) did not load on the iPhone hotspot.
**Type the whole link.** Safari autocomplete offers older URLs and drops everything after
`?node=..`; that is what lost the video on Sat Nov 21.

`run-live.sh` prints both links after start; copy the helmet line as printed. With the defaults:

| Phone | Link |
| --- | --- |
| Helmet (Felix, `R1`, video to the HUD) | `https://qnxpi-felix.local:8080/?node=R1&tag=1&tagid=0&tagmm=186&tagw=640&video=1` |
| Still (Daniel, `R2`, pinned on the Pi) | `https://qnxpi-felix.local:8080/?node=R2&pose=0,2,180` |
| `R2` on tag 0, else tag 7 (`R2_MODE=tag`, not pinned) | `https://qnxpi-felix.local:8080/?node=R2&tag=1&tagid=0,7&tagmm=186&tagw=640` |
| `R2` on tag 7 only (`R2_MODE=tag7`, today's `phone.html`) | `https://qnxpi-felix.local:8080/?node=R2&tag=1&tagid=7&tagmm=186&tagw=640` |

Accept the self-signed certificate once per phone. The helmet phone gets its pose from the
ArUco tag at the room origin (`?tag=1`, Daniel's PR #5);
the compass (`&heading=1`) is no longer used for `R1`. Link flags:

| Flag | Value | Why |
| --- | --- | --- |
| `tagid` | `TAG_ID` (0) | Only the origin tag. Without it the first tag in view wins, and IDs 1 to 9 are printed too |
| `tagmm` | `TAG_MM` (186) | Black square side in mm, measured on the print. Distances scale with it |
| `tagw` | `TAG_W` (640) | Detection width. `phone.html` defaults to 320, which decodes a 186 mm tag only out to about 1.3 m; 640 (the full 640x480 frame) reaches about 2.6 m (`tools/tag_frame_check.mjs`) |
| `hfov` | `HFOV` (unset) | Helmet phone FOV in deg; the tag distance and every bearing use it (page default 66) |

Open issues on the phone side (Daniel's lane): the tag review, and for R2's tag 7 fallback,
`phone.html` sending `pose.tag_id`.

## Tag setup

- **Tags:** ArUco `ARUCO_MIP_36h12` IDs 0 to 9, one per letter sheet. The black square is
  **186 mm** (18.6 cm) per side, which is `TAG_MM=186` (the default) and `tagmm=186` in the
  helmet link. `phone.html` itself defaults to 150, so the link must carry `tagmm`, or every
  distance reads 150/186 = 0.81 of the truth.
- **Print at 100 %** (actual size, no "fit to page"), then measure the black square with a ruler.
  If it is not 186 mm, run with `TAG_MM=<measured>`.
- **Keep the white border** around the black square. The detector needs white on all four sides;
  do not trim the paper to the black edge or tape over the margin.
- **Tag ID 0 is the room origin.** Tape it flat and upright on a wall at about head height (no
  curl, no glare), and keep IDs 1 to 9 out of the helmet's view: the link has `tagid=0`, so they
  are ignored, but a page without it takes the first tag it sees.
- **Measure `R2_POSE` from tag 0** in the frame below (metres right of the tag, metres in front
  of its wall as negative `y`, yaw clockwise from facing the tag), and give `WALLS` in the same
  frame.
- Range: the helmet decodes the tag out to about 2.6 m with `tagw=640` (about 1.3 m with the
  phone's default 320). Further away the HUD shows `POSE HELD` until the tag is closer again.

## Tag pose for R1: room frame and a 30 second check

With the tag upright on a wall (`phone.html` default `tagmap=xz`):

- Origin = the tag. `+y` = the way you face when you look straight at the tag (into its wall).
  `+x` = your right when you face it. `yaw` is clockwise from `+y`: `0` faces the tag, `90` faces
  right, `-90` faces left, `180` faces away.
- Everything in front of the tag has **`y < 0`**. The tag's own wall is the line `y = 0`
  (`WALLS="-2,0,2,0"` for a 4 m wall centred on the tag). Measure `R2_POSE` and any `WALLS` in
  this frame.
- `tools/tag_frame_check.mjs` checks this end to end on the host (real js-aruco2 on a rendered
  tag, `phone.html`'s `cameraPoseFromMarker`, then `hud_fusion`'s placement math): a person to
  the right of the helmet lands where they stand.

Before the demo, with the helmet page open:

1. Stand about 2 m in front of the tag, facing it. The page's tag pill reads about
   `TAG 0 0.00,-2.00 0°`; the HUD corner reads `POSE LIVE X0.0 Y-2.0`.
2. Step half a metre right: `x` goes up by about 0.5. Turn your head right: the yaw goes up.
3. If both go down instead, the print is upside down on the wall (the frame is then mirrored
   and every XRAY lands on the wrong side): turn the paper 180 deg. If `x` barely moves when you
   step sideways (it follows your height instead), the paper is a quarter turn off: rotate it 90
   deg and repeat step 2. Facing the tag straight on reads about right in every orientation, so
   step 2 is the real test.
4. Look away from the tag: after 1 s the pill says `TAG LOST`, the HUD corner turns amber
   `POSE HELD X.. Y.. (NO TAG)`, the tape reads `HOLD` and the status line `R1 15HZ HOLD`. The
   wearer stays where the tag last put them (never 0,0,0). Look back: `POSE LIVE` again.

| HUD corner | Meaning |
| --- | --- |
| `POSE START X0.0 Y-2.0 (NO TAG YET)` (amber) | No tag fix yet since `hud_fusion` started; the wearer is on `WEARER_POSE` |
| `POSE LIVE X.. Y..` | Live tag pose from the helmet phone |
| `POSE HELD X.. Y.. (NO TAG)` (amber) | The phone lost the tag (sends `pose: null`); the last live pose is held |
| `POSE FIXED X.. Y..` | Static pose (`--wearer-pose-src static`, or a config pose) |

`hud_fusion` also ignores a single-frame tag pose jump of more than 0.75 m or 30 deg (planar
pose estimation can flip to its mirror solution for one frame); three consistent ones in a row
are a real move and are taken (about 200 ms). `--debug` logs each ignored jump;
`FUSION_ARGS=--no-pose-gate` turns the gate off.

## What a good start looks like

```text
[hud_display] video: listening on tcp/47106 (IPv4 + IPv6), ... waiting for the bridge
[seeker_bridge] video: connected to hud_display at 127.0.0.1:47106
[hud_display] video: relay connected from 127.0.0.1:<port>
[hud_display] video: no-frames, HUD says "NO VIDEO: PHONE NOT SENDING"     (no phone yet)
[seeker_bridge] phone R1 connected (view node: its video goes to the HUD)
[seeker_bridge] video: asked R1 to stream video (video_ctrl, 15 fps)
[seeker_bridge] video: first frame from R1
[hud_display] video: first frame from 127.0.0.1:<port>: R1 640x480 seq 0, ...
[hud_display] video: ok, R1 picture on the HUD
[seeker_bridge] status: R1 up 12 s, 150 msgs, video 160 frames 14.8 fps, HUD link up
```

Quick check from another SSH window:
`grep video: ~/seeker-logs/hud_display.log ~/seeker-logs/bridge.log | tail -20`.

## HUD banner: why there is no video

`hud_display` shows the reason as a banner (wearer view, radar fallback). It also shows a short
code in the status line (`VID NO-LINK`, ...) and logs one line when the reason changes
(`video: <code>, HUD says "<banner>"`).

| Banner | Code | Meaning | Fix |
| --- | --- | --- | --- |
| `NO VIDEO: BRIDGE NOT CONNECTED` | `no-link` | Nothing connected to tcp/47106 | Bridge not running, or Daniel's plain `ws_ingest.py` (connects only after the phone's first frame). Use `run-live.sh`, check `bridge.log` |
| `NO VIDEO: PHONE NOT SENDING` | `no-frames` | Bridge connected, no frame yet | Helmet page has no video: reload it with the full link above. `bridge.log` says if `video_ctrl` was sent and whether R1 sends frames |
| `VIDEO ERROR: NOT SKV2 FRAMES` | `bad-data` | Bytes arrive but are not IF-5.6 frames | Something other than the bridge is on 47106 |
| `VIDEO ERROR: JPEG DECODE FAILED` | `decode` | Frames arrive, the JPEG does not decode | Phone or bridge sends a broken image; `hud_display.log` counts errors |
| `VIDEO LOST` | `stalled` | Frames came, then stopped for 2 s | Phone locked, Safari in the background, or Wi-Fi dropped. `hud_display.log` says `no frame from ... for N ms`, then `frames ... resumed` |
| `VIDEO STALE 1.2 S` (small amber tag, not a banner) | `ok` | The last frame is 0.5 to 2 s old; the picture stays up, dimmed | Normal now and then: phone video over Wi-Fi comes in bursts. Often: the helmet phone is slow (pose plus tag detection); the 5 s `video:` stats line counts the gaps |
| `NO VIDEO: ONLY ANOTHER PHONE'S VIDEO` | `other-node` | Frames arrive, but not from the wearer (R1) | The bridge forwards the wrong phone: check `--view-node` / `WEARER` |

`hud_display` also logs, unprompted:

- `video: nothing connected to tcp/47106 yet ...` 10 s after start with no bridge.
- `video: bridge ... connected, no frames yet ...` 5 s after a bridge connects with no frame.

Both repeat every 30 s.

### Screen flips between the video and the radar

Before the fix the wearer view fell back to the full radar 500 ms after the newest frame, and
helmet video over Wi-Fi often has longer gaps (pose plus ArUco detection on the phone), so the
monitor flipped back and forth. Now a frame 0.5 to 2 s old stays up, dimmed, with a small
`VIDEO STALE` tag; only after 2 s with no frame does the radar take over. To see how bursty the
phone is:

```sh
grep "video: .* fps" ~/seeker-logs/hud_display.log | tail -5
# video: 4.1 fps, ... ; gaps over 500 ms: 6 (stale), over 2000 ms: 0 (lost), longest 1450 ms
grep -c "picture no longer drawn" ~/seeker-logs/hud_display.log   # each one is a flip to radar
```

Many `over 2000 ms` gaps (real radar flips) mean the phone itself stalls: close other Safari
tabs and keep the phone cool and plugged in. `video: frames from R2 skipped`
means the wrong phone's video reached the HUD (only R1's is shown; check `WEARER`).

## Bridge log (`bridge.log`)

| Line | Meaning |
| --- | --- |
| `video: hud_display not listening on 127.0.0.1:47106 yet (...)` | Start `hud_display` (run-live does this first); retried every second |
| `video: link to hud_display lost (...), reconnecting` | `hud_display` restarted; video comes back by itself |
| `phone R1 connected (view node ...)` / `video: asked R1 to stream video` | Helmet page is up; `video_ctrl` sent |
| `WARNING R1 is connected but sends no video` | Reload R1 with `&video=1`. If that does not help, its `phone.html` is older than `24686f3` (no video capture) |
| `R2 sends video but only R1 (--view-node) goes to the HUD` | Video from the still phone is dropped by design; only the wearer's video is drawn |
| `[ws_ingest] node R1 connected pose=None` | Normal with `?tag=1` until the phone sees the tag (also `R2` in `R2_MODE=tag` or `tag7`, until it sees its tag). For a pinned `R2` it means the URL lost the `&pose=...` part (the pin on the Pi still holds) |
| `pose R1: live 0.60,-2.00,-15.0` | The helmet phone sees the tag again (logged once per change) |
| `pose R1: null (tag out of view?); hud_fusion holds ...` | The helmet phone lost the tag; the HUD says `POSE HELD` |
| `status: R1 ..., pose null 12 s (held at ...)` | How long the tag has been out of view |
| `status: R1 ..., pose null (no tag fix yet)` | The helmet page has not seen the tag since it connected |

## Range reads too far (R2 said about 5 m for a person about 2 m away)

`phone.html` on `main` computes `range_m = f * 1.7 / box_h`, with `f` from `?hfov=` (default
66 deg). Two things make that read long:

- **No `&hfov=` in the link.** The iPhone 17 Pro may be on another lens or orientation, so `f`
  is wrong.
- **`box_h` is only the visible landmarks**, not head to floor.

Daniel's `daniel/phone-smoothing` branch (`b806419`) fixes this with landmark height and a
**Calibrate 2 m** button that stores `f` per phone. Until then, triangulation (both phones see
the person) gives the position and mostly ignores the mono range.

## Poses and room walls (Morrissette setup)

Room coordinates are metres from the origin (the ArUco tag, frame above): `+x` is your right
when you face the tag, `+y` is the way you face it, and `yaw` is clockwise degrees from `+y` (`0`
faces the tag, `90` faces `+x`, `180` faces away). In front of the tag `y < 0`.

- **R2 is pinned.** `run-live.sh` runs `hud_fusion --pose R2=$R2_POSE` (default `0,2,180`); the
  pin beats whatever `R2` sends in its messages. Override it with the tape-measured pose, for
  example R2 on the far side of the tag's wall, 2.5 m right and 1.5 m behind it, facing left:
  ```sh
  cd ~/seeker/drafts/pi-lane && R2_POSE="2.5,1.5,-90" sh scripts/run-live.sh
  ```
  `PINS="R2=x,y,yaw ..."` still works and replaces `R2_POSE` when set. To let R2 find itself
  from the tags instead, use `R2_MODE=tag` or `tag7` (next section).
- **R1 (the wearer) is never pinned.** It moves, so `hud_fusion` uses the live pose in every
  `R1` message (`pose_source` `message`). Until the first full `x_m,y_m,yaw_deg` pose arrives,
  the wearer sits at `WEARER_POSE` (default `0,-2,0`, 2 m in front of the tag facing it), passed
  as `hud_fusion --init-pose R1=...`
  (a starting pose, not a pin) and `hud_display --wearer-pose R1=...`. This replaces the old
  `R1` entry in `config/seeker.eagleeye.json` (`-3.0,5.5,104`), which used to show up as
  `POSE LIVE X-3.0 Y5.5`.
  ```sh
  cd ~/seeker/drafts/pi-lane && WEARER_POSE="0,-2,0" R2_POSE="2.5,1.5,-90" sh scripts/run-live.sh
  ```
  The HUD corner reads `POSE START` while the wearer is on the starting pose, `POSE LIVE` after a
  full message pose, and `POSE HELD` once the tag drops (table above). A partial pose (`x_m,y_m`
  with `yaw_deg` null, e.g. the old `&heading=1` page before **Zero heading**) moves the
  wearer's `x,y` but is not live; `hud_fusion` logs it as `pose from message (partial)`. Putting
  the wearer in `PINS` defeats the live pose; `run-live.sh` warns if you do.
- **Room walls** for the radar inset and the wall-crossing check. `config/seeker.live.json` has
  none; pass space-separated `x1,y1,x2,y2` segments in `WALLS` (`WALLS=none` clears them):
  ```sh
  cd ~/seeker/drafts/pi-lane && WALLS="-2,0,2,0" R2_POSE="2.5,1.5,-90" sh scripts/run-live.sh
  ```
  Running `hud_display` by hand, `--wall x1,y1,x2,y2` replaces the config walls (repeat it for
  more segments), and `--no-walls` (or `--wall none`) clears them.

## R2 on tag 0 and tag 7 (`R2_MODE=tag`)

R2 finds its own pose from the ArUco tags instead of being pinned: from **tag 0** whenever it
sees it (also at an angle, up to about 50 to 60 deg off-axis), else from **tag 7**; tag 0 wins
when both are in one frame. R2's page runs the same tag code as the helmet. A tag 0 pose is
already in the room frame. A tag 7 pose is in **tag 7's** frame (origin at tag 7, `+y` the way
you face looking at tag 7); `hud_fusion --tag-pose 7=x,y,yaw` knows tag 7's room pose and moves it
into the room frame before fusion uses it. The phone names the tag it used in `pose.tag_id`.

```sh
cd ~/seeker/drafts/pi-lane && R2_MODE=tag sh scripts/run-live.sh
```

This runs `hud_fusion ... --init-pose R1=0,-2,0 --tag-pose 7=-0.463,0.06,90 --init-pose
R2=-2.463,0.06,90` (no `--pose R2`) and prints the R2 link:
`https://qnxpi-felix.local:8080/?node=R2&tag=1&tagid=0,7&tagmm=186&tagw=640`.

**Phone side, today:** Daniel's `phone.html` (`main` `6c4f70e`) takes one tag id
(`parseInt("0,7")` is 0) and does not send `pose.tag_id`. With this link R2 therefore uses
**tag 0 only**, which works end to end now (a pose without `tag_id` is the room frame). The
tag 7 fallback starts working, with no Pi change, once `phone.html` has the three-line change that sends `pose.tag_id`.
Until then, if R2 cannot see tag 0 from where it stands, use **`R2_MODE=tag7`**: link
`?tagid=7` and `hud_fusion --tag-frame R2=$TAG7_POSE` (all of R2's untagged poses are tag 7's).
That works with today's page.

| Mode | R2 link `tagid` | `hud_fusion` | Works with today's `phone.html` |
| --- | --- | --- | --- |
| `pin` (default) | none, `&pose=$R2_POSE` | `--pose R2=$R2_POSE` | yes |
| `tag` | `0,7` | `--tag-pose 7=$TAG7_POSE --init-pose R2=$R2_START` | tag 0 only; tag 7 fallback after Daniel's change |
| `tag7` | `7` | `--tag-pose 7=$TAG7_POSE --tag-frame R2=$TAG7_POSE` | yes (tag 7 only) |

- R2 is **not pinned** in either tag mode. Until its first fix it sits at `R2_START` (default
  `-2.463,0.06,90`, 2 m in front of tag 7 facing it; `tag7` mode uses the same spot), and the
  HUD status line reads `R2 15HZ START` (amber).
- No tag in view: R2's page sends `pose: null`, `hud_fusion` holds R2's last room pose (never
  0,0,0), logs `NODE R2 pose null (tag out of view?): holding its last pose ...`, and the status
  line reads `R2 15HZ HOLD` (amber). A tag back: live again at once.
- **Tag switches** go through the same jump gate as R1's tag flips. If tag 0 and tag 7 put R2
  within 0.75 m and 30 deg of each other (a good `TAG7_POSE`), a switch is instant. If they
  disagree by more, the new tag is taken after 3 consistent frames (about 200 ms), and
  `hud_fusion.log` says `NODE R2 pose jump ... (tag switch) ignored (1 of 3)` once per attempt.
  A one-frame flicker between the tags never moves R2; a real switch is never stuck. A gap of
  0.3 m or more between the tags on every switch means `TAG7_POSE` is off: re-measure.
- `hud_fusion.log` names the tag on each pose change: `NODE R2 pose from message: x,y,yaw (tag 0)`
  or `... (tag 7 frame lx,ly,lyaw)`. A `tag_id` with no `--tag-pose` (say tag 3 in view with an
  open `tagid`) is taken as pose null and logged once.
- R1 is unchanged: tag 0 (`?tagid=0`), never pinned.
- Env: `TAG7_POSE="x,y,yaw"` (tag 7's room pose, default `-0.463,0.06,90`), `R2_TAG_ID` (7),
  `R2_START`. `PINS` must not name R2 in a tag mode (run-live stops with a message;
  `hud_fusion` also refuses `--pose R2` together with `--tag-frame R2`).

### Tag 7's room pose: derivation

Room frame (tag 0, as above): `+y` is the way you face looking straight at tag 0, `+x` is your
right, so **"left of tag 0" as seen by someone facing it is `-x`**. Tag 0's wall face is the line
`y = 0`; the room in front of it is `y < 0`.

Felix's measurements (living room): tag 7 is on the narrow side face of a wall corner to the
left of tag 0, both at camera height. 37 cm from the corner edge to tag 0's nearest black border;
tag 7's centre is 6 cm from that same corner edge, on the side face; the side face is
perpendicular to tag 0's wall and faces the left part of the room.

1. **Corner edge.** Tag 0's black square is 186 mm, so its centre is 9.3 cm in from its border:
   37 + 9.3 = 46.3 cm from tag 0's centre, to the left. Edge at `x = -0.463`, `y = 0`.
2. **Side face.** It faces the left part of the room, so its outward normal points `-x`, and it
   lies in the plane `x = -0.463`. Facing `-x` and meeting tag 0's wall at an outside corner, it
   runs from the edge back, away from the room in front of tag 0, into `y > 0`. Tag 7's centre
   is 6 cm along it: `y = +0.06`.
3. **Yaw.** A tag frame's yaw is the room heading of someone looking straight at that tag. Tag 7
   faces `-x`, so you look at it facing `+x`: yaw `+90` (clockwise from `+y`, 90 = facing right).

**`TAG7_POSE = -0.463,0.06,90`.** Top view, `+y` up the page:

```text
          ######################
   R2 --> [7] tag 7 at (-0.463, +0.06), faces -x
          |#####################        (# = wall)
          +------------[0]----------    tag 0's wall face, y = 0
   corner edge     tag 0 at (0, 0)
   (-0.463, 0)  <- 0.37 m + 0.093 m ->
                          ^
                          you, facing tag 0 (yaw 0); your right = +x; room in front: y < 0
```

What `hud_fusion` does with it: tag 7's own `+x` (your right as you face tag 7, i.e. facing room
`+x`) is room `-y`, toward the front of the room. A tag 7 frame pose `(lx, ly, lyaw)` becomes room
`(-0.463 + ly, 0.06 - lx, lyaw + 90)`. Examples: R2's page reads `TAG 7 0.00,-2.00 0` (2 m in
front of tag 7, facing it) = room `-2.463,0.06,90`; `TAG 7 0.50,-1.50 -20` = room
`-1.963,-0.44,70`. `fusion/seeker_pose.c` (`seeker_pose_from_tag`) is the general form;
`tests/test_core.c` checks these numbers and the round trip.

**If the room is not like this**, measure and override, e.g.
`R2_MODE=tag TAG7_POSE="-0.5,0.1,90" sh scripts/run-live.sh`:

- the side face looks right instead (an inside corner: a side wall comes out toward you from
  tag 0's wall, and you stand to the right of it to see tag 7): `-0.463,-0.06,-90`;
- tag 7 on a wall facing tag 0 (across the room, `d` m out from tag 0's wall): `x,-d,180`.

**Mounting.** Tag 7 is wider than the side face: keep it flat (tape it to stiff card if it
overhangs the edge), never folded round the corner, upright like tag 0, white border showing on
all four sides, and its centre 6 cm from the edge. A folded or curled tag reads wrong or not at
all.

### Tag 7: 30 second check

R2 has to be on tag 7 for steps 1 to 4: with today's `phone.html` run `R2_MODE=tag7`; after
Daniel's change `R2_MODE=tag` with tag 0 covered (or out of R2's view) does the same. Both pages
open.

The log always shows R2 where `TAG7_POSE` puts it, so check it against where R2 **physically**
is, in tag 0's frame.

1. Hold R2 about 2 m in front of tag 7, facing it. R2's page reads about `TAG 7 0.00,-2.00 0°`;
   `hud_fusion.log` says `NODE R2 pose from message: -2.46,0.06,90.0 (tag frame 0.00,-2.00,0.0)`
   (`(tag 7 frame ...)` once the page names its tag);
   the status line reads `R2 15HZ` (no `START`). Physically R2 should now be about 2.5 m to the
   **left** of tag 0's centre (as you face tag 0), about level with tag 0's wall, facing **right**
   toward tag 0's side of the room. If you had to stand to the right of the corner to see tag 7,
   the face looks right: use the inside-corner pose above.
2. Walk R2 half a metre **toward tag 7** (toward tag 0's side of the room): room `x` goes up by
   about 0.5 (to about `-1.96`), `y` stays. Then step R2 half a metre to **its right** (out toward
   the front of the room, away from tag 0's wall line): room `y` goes **down** by about 0.5. If
   `y` goes up instead, tag 7 is upside down (or a quarter turn off) on its face: turn the paper
   and repeat, the same as step 3 of the R1 check.
3. **Same person, same spot.** One person stands where both phones see them, for example about
   1 m left of tag 0 and 1 m out from its wall (room about `-1,-1`). `hud_fusion.log` shows
   `fused track N R1+R2 room -1.0,-1.0 ...` near their real spot, and the HUD shows one `SEEN`
   marker. Two separate tracks for one person, or a fused point far from where they stand, means
   `TAG7_POSE` is off: re-measure.
4. Turn R2 away from tag 7: after 1 s the status line reads `R2 15HZ HOLD` (amber) and the log
   says `NODE R2 pose null ... holding its last pose`. R2 stays where it was (never 0,0,0). Turn
   back: `NODE R2 pose from message: ... (live again after hold)`.
5. **Tag 0 against tag 7 (the best `TAG7_POSE` test).** Put R2 on a stand where it sees both
   tags and do not move it. After Daniel's change (`R2_MODE=tag`): the log reads `(tag 0)`;
   cover tag 0 and it switches to `(tag 7 frame ...)`; the two room poses should agree within
   about 0.2 m and 10 deg. With today's page: note R2's room pose under `R2_MODE=tag` (tag 0),
   then restart with `R2_MODE=tag7` and reload R2 with the tag7 link, phone still in place, and
   compare. A steady offset is a `TAG7_POSE` error: subtract (tag 7 room pose minus tag 0 room
   pose) from `TAG7_POSE`, x from x, y from y, yaw from yaw.

## SEEN, XRAY and OFF

- **SEEN**: the wearer's own camera (`R1`) sees the person.
- **XRAY**: another phone (`R2`) sees the person, the person's bearing from the wearer is inside
  the wearer's FOV, and `R1` does not see them. Walls are not required for XRAY. When walls are
  set, a person on the far side of a wall is never merged into a SEEN person on the near side,
  even on the same bearing.
- **OFF**: the bearing is outside the wearer's FOV (edge chevron with turn angle).

A person seen by only one phone with `range_m: null` still gets a track. On the HUD it is a
dashed bearing ray from that phone across the wearer's view with an XRAY marker (or an OFF
chevron when the ray never enters the FOV), and a dashed ray on the radar inset.

If an XRAY person shows as OFF, check the `POSE` line first: a wrong wearer pose turns the
bearing, which is what happened with the stale `-3.0,5.5,104` pose. With the tag, a long
`POSE HELD` while the wearer walks has the same effect: look at the tag to refresh it.

### HUD theme: plan (default) or classic

`hud_display` draws the **plan** theme by default: a building drawing laid over the camera, in
Atkinson Hyperlegible type (compiled in, `third_party/atkinson`). The "Box colours" table below
describes the **classic** look (5x7 capitals, brackets, red / amber / green / cyan), still one flag
away.

Plan colours are cool only, because warm colours vanish against flames and orange glow:
**cyan** `#22E4FF` for a person R1 sees, **green** `#3BFF9E` for someone behind a wall, **magenta**
`#FF3DD0` for every alert (close, coming closer, person down, video late, warning plates, the map's
alert squares). Every person stroke (frames, ticks, leaders, skeletons, hulls, chevrons, map marks)
sits on a dark halo (black at 65%, 3 px wider, dashes included) so it reads on bright fire and dark
smoke alike. Plates keep their dark fill and near-white text. The tokens live in one place,
`display/plan.h` (`P_SEEN`, `P_HIDDEN`, `P_ALERT`, `P_HALO*`).

```sh
build/hud_display ... --theme classic     # or "theme": "classic" in the config's display object
cd ~/seeker/drafts/pi-lane && THEME=classic sh scripts/run-live.sh   # or HUD_ARGS="--theme classic"
```

| Plan theme shows | Means |
| --- | --- |
| Cyan solid rectangle with short ticks on its left and right sides, leader to a cyan-edged plate "Person" / "3.4 m" | R1's own camera sees the person; the distance is from the wearer. Posture (#28) in plain words: "3.4 m, crouching"; standing and unknown say nothing |
| The same in **magenta**, plate hatched magenta, "1.4 m, coming closer" (or "1.4 m, close") | Close: the classic RED rule (under `colours.near_m`, or approaching faster than `colours.closing_mps`) |
| Magenta frame, magenta-hatched plate "Person down" / "6.6 m" | Posture DOWN, at any distance: the most important state, drawn like close |
| Green dashed outline, light hatch, dashed skeleton with square joints and a dashed head, plate "Behind wall" / "4.7 m, seen by R2" | R1 does not see the person, R2 does, and a configured wall is between R1 and them ("Out of sight" when no wall is in the way). A dashed cross instead of the skeleton: no landmarks (`pts`) for that person |
| The same ghost lying down (lifted to the bottom of the view when the floor is out of sight), magenta-hatched plate "Behind wall, person down" / "2.1 m, seen by R2" | Posture DOWN behind a wall: the plate goes magenta, the ghost stays green |
| Chevron at the screen edge with a plate "4.9 m, 41° right" | Outside R1's view (solid for seen, dashed for behind a wall), ", behind you" past 90 deg |
| Title block bottom left: Camera / Eyes / Position / Time (+ Delay) | "R1 live", "Video 1.2 s late" (cell hatched magenta, also through a frozen VIDEO LOST), "Camera paused" (map fallback); the other phones and their rates; Fixed / Live / Held / Compass (no tag, compass yaw) / Easing home / Starting; the Pi's clock; with `SEEKER_LAT=1` a Delay cell, the camera to HUD median ("140 ms") |
| Plan bottom right (full screen when the camera is paused) | Heading up, the wearer as a triangle at its real pose (full screen: fusion's `wearer_node`, as the classic radar), R1's view cone dashed, walls hatched, R2 as a green circle with its cone, seen people as cyan squares, people behind walls as green dashed squares, magenta when close or down. 0 1 2 m scale bar |
| Magenta-edged plate at the top | What is wrong: "Camera paused, showing plan" or "Video lost, last picture shown" with the reason, "Fusion lost", "R2 lost", "R1 position held", "R1 on compass", "R1 easing home" |

State changes are animated (`display/anim_alert.c`), decoration only: a state that turns true is
drawn in full on that frame, and nobody's drawn position or size is eased beyond fusion and the
box smoothing. Entering an alert, the frame, plate edge and words turn magenta at once and the
plate hatch sweeps in from the bottom-left corner (from 20%, full by 200 ms). Person down (in view,
or behind a wall on the ghost's hull) adds one ring ping out from the frame (500 ms), then a calm
breathing pulse of the frame (1.2 s, alpha 70% to 100%). Leaving an alert, the colours blend back
over 200 ms. A title block cell whose words change crossfades over 150 ms (the new value starts at
60%); a change of digits only (the clock, "1.2 s late", fps) swaps at once. Warning plates slide 8
px in from above while fading in (150 ms) and leave the same way. Everything runs on one timestamp
per frame (`hud_state.now_ms`), so `hud_shot --seq DIR` renders any moment; `now_ms` 0 or
`anim_alert_set_enabled(0)` gives the still plan look exactly.

Stale people are drawn faded rather than grey. The theme only changes drawing: tracking, the
colour hysteresis and the fallback rules are the same as classic.
`tools/hud_shot` renders both themes of a made-up scene on a laptop (no phones, no Pi):
`python3 tools/hud_shot_room.py room.png` then convert it to PPM and run
`build/hud_shot --bg room.ppm --out shots/` (`--seq anim/` for the animation sequences).

### Box colours (wearer view)

| Looks like | Means |
| --- | --- |
| Solid box with corner brackets, **RED**, tag `SEEN` | R1's own camera sees the person, and they are **closer than 2 m** to R1 (`colours.near_m`) **or coming closer faster than 0.5 m/s** (`colours.closing_mps`) |
| Solid box with corner brackets, **ORANGE** (amber, `255,180,0`), tag `SEEN` | R1 sees the person **2 to 5 m** away (`colours.mid_m`), not closing fast |
| Solid box, GREEN | R1 sees the person more than 5 m away |
| Dashed **CYAN** box with a diamond (or a dashed ghost skeleton), tags `XRAY` and `VIA R2` | R1 does **not** see the person; another phone does, and they are inside R1's view (through the wall) |
| Any of the above in **GREY** | Stale: no tracks from `hud_fusion` for 500 ms (`FUSION LOST`), or that person's data is over 500 ms old |
| Chevron at the screen edge | OFF: outside R1's view; same colours (solid = seen by R1, dashed = x-ray), with the turn angle |

So red and orange are both people R1 sees directly; only the distance (and approach speed)
differs. Behind-the-wall people are always cyan, never red or orange.

The colours do not flicker on a noisy range (`common/seeker_colour.h`): the approach speed is
the slope of the range over the last 0.7 s (not a frame-to-frame difference); red starts under
2.0 m and ends above 2.3 m; "approaching fast" needs more than 0.6 m/s for 0.3 s and ends under
0.3 m/s (1.2 and 0.6 times `colours.closing_mps`); amber ends above 5.3 m; and every colour stays
at least 0.4 s. Someone walking in at 1 m/s still turns red within about 0.8 s. In the full-screen
radar fallback the dots use fusion's own `colour` field, with the same rules, measured from the
room origin (tag 0); cyan for a person with no range.

## Ghost skeletons (phone pts)

Daniel's `phone.html` (his PR #7) sends each person's COCO-17 landmarks as `pts`. Copy his
current `remote/phone.html` and `remote/ws_ingest.py` into `~/seeker/remote/` (the page the
bridge serves) to get them; `?skel=0` on a phone link turns them off.

- `hud_fusion` turns the primary phone's landmarks into a person-sized skeleton (person heights
  from the mid-hip, scaled by the smoothed distance from that phone) and **smooths it per track**:
  a One Euro filter per joint (steady when still, little lag when an arm moves), joints the phone
  is unsure of (visibility under 0.3) held 300 ms then hidden, the whole skeleton held 500 ms if a
  few datagrams come without it, and a joint that jumps more than half a person height restarts
  where it is instead of sliding there. Host test: jitter down to 0.46 of raw, 0.05 person
  heights of lag on a wrist moving 1.2 person heights per second.
- Up to 6 skeletons per tracks datagram (people seen by one phone first); `pts` are dropped
  before any track would be.
- The wearer view draws an `XRAY` person with a skeleton as a dashed cyan ghost in a thin hull
  (`SEEN` people never get one). It needs a range from R2 and the person inside R1's view.
- Logs: `hud_fusion.log` starts with `skeletons (pts): One Euro per joint ...`; each
  `eagleeye:` line in `hud_display.log` ends with `skel N` (ghosts on screen).
- `FUSION_ARGS=--no-skel-smooth` sends the raw joints, to compare.

## X-ray to seen: one person when they step out or put a hand out

A person behind the divider is an x-ray ghost from R2. When R1 starts to see them, it stays
**one person with one track id**: the ghost dissolves into the seen box (250 ms), and the
distance label, colour and plan mark carry on.

- **Hand or shoulder round the edge.** R1's phone gets the range from the box of the visible
  landmarks, so a head-and-arm box reads 2 to 3 times too far. Before, that was a second person
  (`R1` box "Person 4.6 m" next to the ghost). Now a contact with no eyes plus ankles, no full
  torso and a box not shaped like a whole person is **partial**. It places the person by its
  bearing only. The person stays XRAY (the ghost lines up with the hand), and the plate still says
  "seen by R2". A partial R1 sighting with nobody near its bearing starts no track.
- **Stepping out.** Track identity allows for the x-ray position's range error (a mono range is
  about 20 to 30% long or short along R2's ray), with a wider gate while a phone joins. Before,
  an R2 about 6 m away reading 30% long gave the person a new id and a 2 m jump. Now the
  position glides onto the two-phone fix over about 350 ms and keeps its walking speed.
- **R1 missing frames.** A person R1 saw stays seen for 400 ms, so the look no longer flips at
  the phone's frame rate. Fusion keeps what the two-phone fix taught about each phone's range, so
  the ghost does not jump back by R2's error when R1 loses them (it fades back to the phone's own
  range over a few seconds).

Host replay (`build/test_handoff`, ctest `handoff`; before = old code on the same seed):

| Scene | Before | After |
| --- | --- | --- |
| Step out, R2 still sees | 5 look changes, 0.32 m jump | 1 look change, 8 crossfade frames, 0.21 m step |
| Step out, R2 loses them | 0.50 m error (R2's range reused on R1's ray) | 0.06 m |
| Step out, far R2 (30% long) | 2 ids, 1.98 m jump, 7 look changes | 1 id, glide (0.62 m first tick), 1 look change |
| Hand out | 2 people for 37 ticks, drawn SEEN 54 frames, ghost 0.40 m off | 1 x-ray person, 0.23 m off |
| R1 sees only a hand, nobody else | a person 2.3 m off | nothing |

Logs: `hud_fusion.log` starts with `handoff: partial contacts by bearing ...`; with `--debug`,
`track N handoff to R1+R2: X m from where it was, gliding over 350 ms`. `hud_display.log`:
`eagleeye: handoff: a person R1 saw stays seen 400 ms ...`. To compare on stage:
`FUSION_ARGS=--no-handoff HUD_ARGS="--handoff off" sh scripts/run-live.sh` is the old behaviour.

Timing: the x-ray ghost is drawn from R2's newest report moved on by up to 100 ms
(`fusion.predict_ms`), over R1's video, which is 120 to 170 ms old (the `LAT` line). With
someone walking at 0.8 m/s it leads the picture by about 0.15 m (about 30 px at 2.5 m). That is
small next to R2's range error, which the two-phone fix removes once R1 sees them.
`FUSION_ARGS="--predict-ms 0"` takes the 100 ms out if the lead shows.

## Morrissette calibration (tag 0 only, both phones fixed)

Tag 7 is not needed: `R2_MODE` defaults to `pin` and `WEARER_MODE=fixed` pins R1, both from
poses measured in the tag 0 room frame. `scripts/calibrate.sh` measures them from what the
phones send and writes `~/seeker-logs/calib.env`; `run-live.sh` reads it (a variable given on
the command line still wins).

1. Tape tag 0 (186 mm, ID 0) flat and upright where both phones can see it from their demo
   spots, at about camera height (on the divider's face toward R1, or a wall both face).
2. Put R1 (wearer, iPhone 17 Pro) and R2 (iPhone 16) on their fixed spots.
3. `sh scripts/calibrate.sh pose`: open the two printed links (`?tag=1&tagid=0`), wait until
   both pills read `TAG 0 ...`, press Enter, hold still 15 s. It prints and saves
   `WEARER_POSE` and `R2_POSE` (median of the tag poses; a spread warning means redo).
4. Only if a phone has to turn away from tag 0 to aim at the scene: aim both, have ONE person
   stand on a taped mark (measure x,y from tag 0, metres: +x right as you face the tag, in
   front is negative y), then `R1_HFOV=iphone17pro R2_HFOV=iphone16 sh scripts/calibrate.sh aim
   X,Y`. It keeps each phone's x,y and solves its yaw from the person's bearing.
5. Start: `WEARER_MODE=fixed R1_HFOV=iphone17pro R2_HFOV=iphone16 sh scripts/run-live.sh`.
   Links: helmet `?node=R1&video=1`, R2 `?node=R2&pose=...` (printed).
6. Check (30 s): one person walks across behind the divider; on the HUD the XRAY box tracks
   them without sliding off to one side. If it leads or lags sideways the FOV is off: try the
   portrait value, or a number (`R2_HFOV=60`).

FOV: `R1_HFOV` / `R2_HFOV` (or `hud_fusion --hfov ID=...`) take degrees or a preset. Presets
are computed from Apple's 35 mm-equivalent focal lengths (iPhone 17 Pro Fusion Main 24 mm,
support.apple.com/en-us/125090; iPhone 16 Fusion Main 26 mm, apple.com/iphone-16/specs;
Ultra Wide 13 mm on both) for a 4:3 640x480 stream over the full sensor width: iphone17pro
71.6 deg landscape / 56.8 portrait, iphone16 67.3 / 53.1, iphone-ultrawide 106.2 / 89.9. They
are estimates (a cropped video mode is narrower); landscape or portrait is picked from the
frame size each phone sends. The phone keeps its link; the Pi rescales its bearings
(tan b' = tan b x tan(real/2) / tan(sent/2)), and the wearer view projects with the same FOV.
Range still comes from the phone: press Calibrate 2 m on each phone once.

`cam_h_m` (1.7 m) only moves boxes up or down; leave it unless the whole overlay sits too high or
low after the steps above. With `fusion.wearer_node` (`R1` in `seeker.live.json`) the distances,
colours and the full-screen radar are measured from R1, not from tag 0.

## Moving wearer (R1 walks, R2 fixed)

`WEARER_MODE=tag` (the default): R1 finds itself from tags; R2 stays pinned (`R2_MODE=pin`).

- More tags: tape 2 or 3 more (IDs 1 to 3, 186 mm) on walls or the divider face, upright, at
  camera height. Measure each against tag 0 once: put R1 on a stand where it sees tag 0 and
  tag N, then `sh scripts/calibrate.sh tag N` (first link `?tagid=0`, then, phone not moved,
  `?tagid=N`). It adds `N=x,y,yaw` to `TAG_MAP` in `calib.env`; run-live passes each as
  `--tag-pose` and lists them in the helmet link (`?tagid=0,1,2,3`). Until Daniel's phone.html
  sends `pose.tag_id`, only tag 0 is used.
- No tag in view: position held; yaw from the compass when the page sends `heading_deg`;
  HUD corner `POSE HELD ... (NO TAG, COMPASS)` or `(NO TAG)`, status `R1 ..HZ HOLD`.
- After 10 s without a tag (`fusion.pose_home_ms`): eases to `WEARER_POSE` (home), HUD `POSE
  HOME`. A tag back eases in over a few hundred ms instead of jumping.
- `hud_fusion.log`: `NODE R1 pose null ... holding`, `no tag for 10 s: easing to its home pose`,
  `pose from message ... (live again after hold, blending in)`.
- Sim (ctest `smoothness`, second part): R1 walking and looking around, tag gone 1, 5 and 20 s.
  The compass keeps the yaw (p95 2.2 deg over 5 s, 4.0 over 20 s, against 6.8 and 38.7 held);
  the box jump when the tag returns drops from 16 / 32 / 26 px to 8 / 18 / 7 px. Walking
  while no tag shows still leaves a position error of about the distance walked (parallax up
  to 20 deg at 3 m for 1 m); only more tags in view fix that.

## Smoothness

- Tracks glide: each 30 Hz fusion tick moves a person on by their recent velocity for up to
  100 ms past the newest phone report (`fusion.predict_ms`, `--predict-ms 0` off); people
  standing still (under 0.2 m/s) are not moved.
- Boxes ease: the wearer view moves each drawn box half way to its new place per frame and
  snaps on a real jump, so detection-box noise does not shake it.
- Video: dim after 0.5 s without a frame, `VIDEO LOST` after 2 s with the last picture kept
  (darker) until 6 s; only then the full radar (`VIDEO_FREEZE_MS`). A relay that is gone goes
  to radar at once.
- `sim_smooth` (ctest `smoothness`) replays two synthetic phones with jitter, bursts and video
  stalls through fusion and the wearer view and prints before/after metrics.

## Posture: STAND, CROUCH, DOWN

`hud_fusion` labels each person with a skeleton from the smoothed `pts`
(`common/seeker_posture.h`), and the HUD shows it in the tag: `SEEN STAND`, `XRAY CROUCH`,
`XRAY DOWN`, also on the edge chevrons and on the full radar (`R2-P1 2.5M DOWN`). No word means
UNKNOWN.

Only joint geometry is used, per leg: the hip-knee-ankle angle and how far the knee sits below
the hip over the torso length (the thigh's share of the torso; short when the thigh is level or
points at the phone). Never the box, the person's size, or where the feet are in the picture, so
a person on the stairs (feet above the floor) is judged like anyone else. A leg only counts when
its hip, knee and ankle are all visible (knee and ankle at visibility 0.5 or more), inside the
phone's frame, with a real shin (MediaPipe invents hidden legs and often folds them onto the knee).

- DOWN: torso more than 65 deg from vertical, or the body much wider than tall, unless a straight
  leg carries the body (bending over is not down). Works on an upper body alone. A ghost skeleton
  that is down is drawn near the floor, and lifted to the bottom of the view if the floor is out
  of the camera's view (a person lying 3 m away usually is).
- STAND: one straight, vertical leg (knee over 150 deg) and the torso upright. One is enough: on a
  staircase the other foot is a step or two up with its knee bent.
- CROUCH: BOTH legs folded (knee under 120 deg, or the knee less than 0.45 torso lengths below
  the hip): squatting, kneeling, sitting. One folded leg with the other hidden is no evidence (the
  upper leg on a stair looks the same): the label stays as it was.
- UNKNOWN: no torso, or no full leg (ankles hidden by a wall, desk or stair, or below the frame).
  No guess.
- Stable: a new label needs 7 of the last 10 skeletons (about 0.5 s at 15 Hz), UNKNOWN 12 of the
  last 15 (about 1 s), and each label stays at least 0.5 s. Host tests: stand to crouch in about
  0.5 s, 0 changes in 10 s of a noisy standing person, 12 s of stair climbing (every other step
  with the lower leg hidden) stays STAND with 0 crouch labels, a 0.2 s crouch glitch never flips.
- Logs: `hud_fusion.log` prints the settings at start (`posture: legs need ...`) and each change,
  e.g. `track 3 R2 posture stand -> crouch (2 deg torso, legs: L knee 90 deg thigh drop 0.09 len
  0.35 R knee 85 deg thigh drop 0.12 len 0.44)`; `L hidden` is a leg that did not count.
  `hud_display.log` `eagleeye:` lines end with `stand N crouch N down N` (people in view).
- Tuning live (environment of `run-live.sh`, read by `hud_fusion` at start; bad values are logged
  and ignored):

  | Variable | Default | Change it when |
  | --- | --- | --- |
  | `SEEKER_POSTURE_LEG_VIS` | `0.5` | hidden legs still count (raise to `0.6`), or real legs drop out and everyone reads UNKNOWN (lower to `0.4`) |
  | `SEEKER_POSTURE_CROUCH_KNEE` | `120` | real crouches are missed (raise to `130`) |
  | `SEEKER_POSTURE_CROUCH_THIGH` | `0.45` | sitting facing the phone is missed (raise to `0.55`) |
  | `SEEKER_POSTURE_STAND_KNEE` | `150` | standing people stay UNKNOWN (lower to `140`) |
  | `SEEKER_POSTURE_STAND_THIGH` | `0.6` | the same, with the thigh short in the picture (phone far above or below) |
  | `SEEKER_POSTURE_ONE_LEG` | `0` | `1`: a single deeply folded leg (knee under 100 deg, thigh level) is enough for CROUCH, for side views where the far leg is hidden. Risk: a big stair step |
  | `SEEKER_POSTURE_VOTES` | `7/10` | `5/10` for a faster label, `9/12` for a steadier one |

  ```sh
  SEEKER_POSTURE_LEG_VIS=0.6 SEEKER_POSTURE_VOTES=8/12 sh scripts/run-live.sh
  ```
- Phone side: `phone.html` drops detections whose box is less than 1.5 times taller than wide
  (`isUpright`), and a deep squat or a side-on sitter is often wider than that, so the person
  blinks out instead of reading CROUCH. Add `&minaspect=1.0` to that phone's link (typed by hand)
  if crouching people vanish.
- Limits: 2D only. Someone lying along the camera's line of sight (head toward the phone) looks
  short and upright and may read STAND or UNKNOWN.

## Fixed wearer (`WEARER_MODE=fixed`)

Per Daniel's fixed-wearer decision (Nov 21),
the wearer can stand still and be pinned like R2: no tag, no `POSE HELD`, more video fps.

```sh
cd ~/seeker/drafts/pi-lane && WEARER_MODE=fixed WEARER_POSE="x,y,yaw" R2_POSE="x,y,yaw" sh scripts/run-live.sh
```

This runs `hud_fusion --pose R1=$WEARER_POSE` (message poses ignored) and prints the helmet link
`?node=R1&video=1` (no `tag=1`). Measure both poses once in the tag 0 room frame. The HUD corner
reads `POSE FIXED`.

## Solo test: R1 only (`R2_MODE=off`)

One phone (R1) and the Pi HUD, no R2 phone. On the Pi:

```sh
cd ~/seeker/drafts/pi-lane && R2_MODE=off WEARER_MODE=fixed WEARER_POSE=0,-2,0 LAT=1 sh scripts/run-live.sh
```

- No R2 pin and no R2 link. R2 stays in `config/seeker.live.json`, so the status line reads
  `R2 --` in grey and the radar inset still draws R2's configured spot. A phone that never
  connected never goes LOST: no `NODE R2 LOST` banner, and the picture/radar choice depends on
  R1's video only.
- People are placed from R1 alone (single-phone range from box height): `XRAY 0` is expected.
- Good start: `hud_display.log` has `video: frame drawn on the HUD: R1 ...` and, with `LAT=1`, a
  `latency:` line every second; `bridge.log` has `status: R1 up ..., video N frames F fps, ws rtt
  p50 X min Y max Z ms`.

## Latency (camera to HUD)

### What is measured

Every wearer frame drawn is timed, and `hud_display.log` gets one line every 10 s (every second
with `LAT=1`, which also puts `LAT 120MS P95 180MS FPS 28` at the right end of the HUD's status
line):

```
latency: 24.0 fps drawn, 0 skipped | total p50 46 p95 71 ms = phone n/a + net p50 18 p95 40
  (above the best frame in 30 s) + pi p50 28 p95 31 (decode 9, wait+draw 19) + base 0
```

- `pi`: last byte received on tcp/47106 to the HUD frame holding it posted to the screen
  (`decode`: JPEG decode and scale; `wait+draw`: until the draw loop picked it up, drew and
  posted it). Exact: one clock, the Pi's.
- `net`: phone send to Pi receive, above the fastest frame of the last 30 s. phone.html stamps
  each SKV2 frame with the phone's `Date.now()` just before `ws.send` and the bridge passes it
  through. The phone's clock and the Pi's are never compared directly: the smallest (receive -
  stamp) of the last 30 s is the offset. So `net` is the queueing on top of the best case (the
  phone's WebSocket buffer, Wi-Fi, the relay), and a rising `net` means a backlog.
- `phone`: camera frame grabbed to `ws.send` (canvas draw plus JPEG encode), only once Daniel's
  page sends the optional field; `n/a` today.
- `base`: what none of the above sees (camera exposure, JPEG encode until the phone sends the
  field, the base Wi-Fi hop, the monitor's own processing). Measure it once with the stopwatch
  photo below and pass it as `LAT_BASE_MS`; the readout then shows the real camera-to-glass time.
- `skipped`: frames that reached the Pi but were never drawn because a newer one was already
  there (the receiver decodes only the newest frame; nothing is queued).
- `stages p50 ms: decode D scale S copy C overlay O blit B post P | received R fps` (end of the
  line): where the Pi's time goes. `decode` is the JPEG decode and `scale` the CPU nearest
  neighbour scale of the 640x480 picture into the canvas-sized buffer (receiver thread, inside
  `pi`'s `decode`); `copy` puts the picture into the canvas, `overlay` draws everything else
  (the theme), `blit` copies the canvas into Screen's buffer, `post` is waiting for a free
  buffer plus `screen_post_window` (with vsync on, up to one refresh). `received` is complete
  frames reaching tcp/47106 per second (the phone's real rate at the Pi); compare it with
  `fps drawn` and with `bridge.log`'s `status: R1 ... video N frames F fps`. If `received` is
  about the same as `fps drawn`, the phone (or Wi-Fi) is the cap, not the Pi.
- Startup: one line `screen: window WxH buffer WxH scaled yes|no ..., swap interval N` says the
  monitor's mode, the size drawn, and whether `SCREEN_BUFFER` took effect (`scaled no
  (SCREEN_BUFFER is the display size)` on a 1080p monitor with `SCREEN_BUFFER=1920x1080`).
- `bridge.log` `ws rtt p50 X min Y`: a WebSocket ping to the phone every second. Safari answers
  in its network stack, but the pong travels behind any video the phone has queued: `p50 - min`
  is the uplink backlog, `min / 2` is about the base Wi-Fi hop.

### Ground truth: the stopwatch photo

1. On the Mac, open `drafts/pi-lane/tools/stopwatch.html` (double-click; no server), full
   screen, right next to the HUD monitor. It shows seconds.milliseconds in big digits.
2. Point R1 at the stopwatch so the HUD shows it.
3. Photo:
   - With a second camera (any phone or camera): take one photo that shows both the Mac's
     stopwatch and the HUD monitor. Latency = the Mac's reading minus the reading on the HUD.
     Example: Mac `12.480`, HUD `12.310`: 170 ms.
   - Solo, R1 only: aim R1 so that both the Mac stopwatch and the HUD monitor are in its view.
     Take a screenshot on R1 (side button + volume up). The page's camera preview shows the
     stopwatch twice: directly, and inside the HUD's picture (smaller). Latency = the direct
     reading minus the one inside the HUD. Both readings come from the same camera frame, so
     the preview's own delay cancels out. Read only the first copy inside the HUD, not the
     smaller copies inside it.
4. Take 5 photos and use the median. A 60 Hz Mac screen redraws every 17 ms, so each reading is
   good to about 17 ms. If a digit looks doubled (two readings blended), use the newer one.
5. Note the `LAT` p50 on the HUD (with `LAT_BASE_MS` unset) at the same time:
   `LAT_BASE_MS` = photo latency - `LAT` p50. Example: photo 170 ms, `LAT 95MS`: `LAT_BASE_MS=75`.
   Restart with it: `R2_MODE=off WEARER_MODE=fixed LAT=1 LAT_BASE_MS=75 sh scripts/run-live.sh`.

### Making it lower

On the Pi side (this build): the receiver decodes only the newest complete frame (stale frames
are skipped, never queued), and a new frame is drawn the moment it is decoded instead of
waiting for the next 1/30 s tick (`--frame-sync on`, the default; host: receive to screen p50 22
to 5 ms, p95 37 to 10 ms). While video flows the frames pace the loop, so a 24 fps phone means 24
draws a second, not 54. `TCP_NODELAY` is set at both ends of tcp/47106, and the relay keeps one
frame at a time. The canvas is the size of the monitor's mode (`screen: window` line): every
stage after `decode` scales with its pixel count. `SCREEN_BUFFER` smaller than the window (for
example `1280x720` on a 1080p monitor, `1920x1080` on 4K) draws fewer pixels and lets Screen
scale them up. `VSYNC=0` stops posts from waiting for the next refresh (watch for tearing); the
`post` stage shows what the wait costs.

Phone link settings to try (`run-live.sh` sets both the link and the bridge's `video_ctrl`;
reload the page with the new link after each restart):

| Try | Command | Look for |
| --- | --- | --- |
| 1 (today) | `VFPS=15` | baseline |
| 2 | `VFPS=20 VQ=0.5` | first choice |
| 3 | `VFPS=24 VQ=0.5` | smoother, more Wi-Fi |
| 4 | `VFPS=30 VQ=0.4` | only if 3 holds up |

Keep the highest one where `fps drawn` stays within 10% of `VFPS`, `skipped` stays near 0,
`net` p95 stays under about 40 ms and `ws rtt` p50 - min stays under about 20 ms. When `net`
or `ws rtt` keeps climbing, the uplink cannot carry it and frames back up in the phone's
WebSocket (phone.html sends with no backpressure today): step back down. `WEARER_MODE=fixed`
(no tag detection on the phone) leaves more of the phone for video. The capture size is fixed
at 640x480 by phone.html (no link option yet).

## Where R2 is (`R2_POSE`): which number means left

`R2_POSE=x,y,yaw` is R2's spot in the tag 0 room frame. Stand in front of tag 0, facing it:

- `x`: metres to your right. Negative is LEFT. R2 1.5 m to the left of the tag line: `x = -1.5`.
- `y`: negative is in front of the tag's wall (your side of it); positive is behind the wall.
  R1's usual spot `0,-2` is 2 m in front of the tag.
- `yaw`: where R2's camera points. 0 faces the tag, 90 faces right, -90 faces left, 180 faces
  away from the tag.

The default `0,2,180` is the old setup (R2 behind the wall, looking back through it): on the HUD
it sits straight ahead of R1, never to the side. For R2 1.5 m to the left of R1 (R1 at `0,-2`),
both facing the tag:

```sh
R2_POSE=-1.5,-2,0 sh scripts/run-live.sh
grep "node R2 is" ~/seeker-logs/hud_fusion.log   # node R2 is 1.50 m to the LEFT and 0.00 m ahead of R1's starting pose (pinned)
```

Or measure it: `sh scripts/calibrate.sh pose` with R2 seeing tag 0 writes `R2_POSE` into
`calib.env`, which `run-live.sh` reads. A pinned R2 never moves in the room. On the HUD's
corner plan (wearer at the bottom, heading up) it moves only when R1 moves or turns. R1's tag
pose is now smoothed (One Euro, `hud_fusion` logs `live poses (tags): One Euro ...`; host test:
R2's shake on the plan 0.063 to 0.028 m sideways, 0.086 to 0.065 m forward with 6 cm / 4 deg
tag noise), so a still wearer sees a still R2. `--no-pose-smooth` (in `FUSION_ARGS`) turns it off.

### R1 jumping between two places (tag pose flips)

A flat tag seen from off its axis has two camera poses that fit the picture almost equally
well. phone.html's POSIT keeps the "best" one, and which one wins flips with image noise: the
wearer jumps between two spots about 1.4 m and 40 deg apart (on the floor, one is the other
mirrored across the tag's normal, same distance from the tag). hud_fusion now recognises the
mirror: such a pose is taken as its mirror (the same place, solved the other way), and the
wearer switches to the other solution only when it wins 8 of 10 frames. `hud_fusion.log`:

```
NODE R1 tag pose flipped to its mirror 0.69,-1.82,-33.6 (planar tag ambiguity): kept -0.69,-1.82,7.9 (1 flips so far)
NODE R1 tag pose: the other solution won 8 of the last 10 frames, switching to it: ...
```

Fewer flips: face the tag more squarely (the two solutions merge straight on), use a bigger tag
or come closer. `--no-pose-flip` in `FUSION_ARGS` turns the catch off (the jump gate alone).

## Calibration knobs for the live run (all `run-live.sh` env, defaults for Sun Nov 22)

| Knob | Default | What it is | Tune when |
| --- | --- | --- | --- |
| `R1_HFOV` | `iphone16` | R1's real horizontal FOV (preset or degrees): bearings, box-height ranges and the x-ray projection. The vertical FOV follows the streamed frame (square pixels), landscape or portrait | an object at the picture's edge is drawn off its real place |
| `R2_HFOV` | `iphone17pro` | the same for R2 | R2's people sit off sideways |
| `RANGE_HFOV` | `1` | box-height ranges rescaled with the real FOV. `0` for a phone whose range was calibrated on its page (`cal` button) | the distance labels are a fixed percentage off |
| `PERSON_H` | `1.75` | people's height, m: box-height range (phone.html assumes 1.7) and the x-ray box top | labels read short for tall people, long for short ones |
| `R1_RANGE_K`, `R2_RANGE_K` | unset | an extra factor on that phone's box-height ranges | one phone's labels read a steady percentage off |
| `R1_CAM_H` | `1.30` | R1's camera height above the floor, m (handheld at chest height) | the x-ray people sit too low (lower it) or too high |
| `R1_PITCH` | `0` | R1's tilt, deg, up positive (the phone's `pitch_deg` wins when it sends one) | it is held tilted |
| `XRAY_UP` | `0` | nudge, deg: positive draws the x-ray people higher | a last fix on stage |
| `PITCH_AUTO` | `1` | learns R1's tilt from people R1 sees whole (its own box vs the projection) | turn off if a half-hidden person keeps dragging it |
| `R2_CAM_H`, `R2_PITCH` | `1.27`, `0` | R2's camera height and tilt: `hud_fusion.log` logs `range check: box height X m, ground plane Y m` | the two disagree: trust the ground plane when R2 is level |
| `GROUND_RANGE` | `0` | `1` averages R2's ground-plane range into its ranges | the range check shows the box height is off and R2 is level |
| `R2_POSE` | measure | R2's x,y,yaw (see "Where R2 is"). `sh scripts/calibrate.sh yaw R2` fills in the yaw from tag 0, keeping the measured x,y | R2's people are rotated around R2 |
| `TAG7_POSE` | living room | tag 7's room pose, only for `R2_MODE=tag` or `tag7`: `sh scripts/calibrate.sh tag 7` | at the venue, if tag 7 is used at all |

Logs to check them: `hud_display.log` `eagleeye: x-ray height: camera ... m, tilt ... deg` at start,
then `eagleeye: x-ray height learned from R1's own sightings: tilt +X deg`; `hud_fusion.log`
`node R1 real horizontal FOV`, `people's height`, `node R2 camera`, `range check`.

### Two-minute tuning on stage

1. Start with the defaults (one command below). Within a few seconds of R1 seeing someone whole,
   `hud_display.log` prints the learned tilt; the x-ray people follow it.
2. One person walks where only R2 sees them (behind the divider), then steps out where R1 sees
   them: the x-ray skeleton should line up with where they appear. Still low: Ctrl+C and rerun
   with `R1_CAM_H=1.20` (or `XRAY_UP=3`); high: `R1_CAM_H=1.45` (or `XRAY_UP=-3`).
3. Distance: stand on a taped 2 m mark in R2's view. `grep "range check" ~/seeker-logs/hud_fusion.log`
   and the HUD label. Off by a steady percentage: `R2_RANGE_K` (for example `0.9` when it reads
   2.2 m). If the ground plane reads 2.0 m and R2 is level: `GROUND_RANGE=1`.
4. Rotated R2 people: `sh scripts/stop-live.sh; R2_POSE=-1.59,-1.4,49 sh scripts/calibrate.sh yaw R2`
   (R2 on its spot, tag 0 in its picture). It prints the new `R2_POSE` and writes it to
   `calib.env`; start again WITHOUT `R2_POSE=` on the command line (a command-line value wins over
   `calib.env`), or paste the printed value.

```sh
R2_MODE=pin R2_POSE=-1.59,-1.4,49 LAT=1 VFPS=24 VQ=0.5 sh scripts/run-live.sh
```
(R1_HFOV=iphone16, R2_HFOV=iphone17pro, R1_CAM_H=1.30, R2_CAM_H=1.27, PERSON_H=1.75 and PITCH_AUTO=1
are the defaults.)

### R1 loses the tag, and the HUD stays choppy after it is back

Two separate things happen when R1 looks away from tag 0:

- **The pose** goes null: hud_fusion holds the last pose (`POSE HELD`). When the tag comes back
  after a second or more, the filters and the flip history start over and the pose is taken once
  two in a row agree (directly or as mirror solutions), so a bad first frame (the tag at the
  picture's edge) is never taken: `hud_fusion.log` says `NODE R1 tag back after 3.0 s ...` then
  `NODE R1 pose back after 3.2 s ...`. Host replay (tag lost 3 s, the wearer turned 30 deg, the
  first pose back an outlier, 20% flipped frames): back 217 ms after the tag, as steady after as
  before.
- **The uplink**: phone.html sends its video with no backpressure and its detections ride the
  same WebSocket. Turning away (Wi-Fi dip, a busier picture, bigger JPEGs) can leave a queue on
  the phone that never drains: everything then arrives late and in bursts, the HUD looks choppy,
  and it stays that way. The bridge now watches its ping round trip to R1 and, when it stays
  over 400 ms, pauses R1's video until the queue has drained (at most 2.5 s, at most once per
  15 s): `bridge.log` `video: R1 uplink backlog ... pausing its video` and `video: R1 video
  resumed after a 1.1 s drain`. The picture dims (VIDEO STALE) for that second. `DRAIN=0` turns
  it off. If drains repeat, the stream is too heavy for the hotspot: `VFPS=15 VQ=0.5`.
