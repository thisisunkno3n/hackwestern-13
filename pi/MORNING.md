# Pi lane draft: read this first (Felix, Sat Nov 21 morning)

Drafted overnight by an agent between 2:41am and about 3:30am ET Sat Nov 21 (inside the hacking
window). It lives here in the context repo under `drafts/pi-lane/` on branch
`felix/pi-lane-draft`. Nothing was pushed to `thisisunkno3n/hackwestern-13`. Copy what you want
into the code repo yourself (your paths: `CMakeLists.txt`, `common/`, `fusion/`, `display/`,
`supervisor/`, `config/`, `scripts/`, `tools/udp_recv.c`).

**Nothing here has run on QNX.** Everything below "ran" ran on an x86 Linux host.

## What's done

| Piece | Files | Compiled | Ran (host only) |
| --- | --- | --- | --- |
| CMake skeleton, `SEEKER_TARGET=laptop\|linux\|qnx` | `CMakeLists.txt`, `cmake/qnx-aarch64le.cmake` | Yes, laptop target | Toolchain file not run (no SDP here) |
| No-CMake build | `scripts/build_pi.sh` | Yes, host gcc and clang | QNX branch (`uname` = QNX) not run |
| `common/`: time, logging, hand-rolled JSON reader (no packages), IF-5 structs and codecs, config, UDP, IF-6 health shm | `common/*` | Yes | Unit tests: 54 pass |
| `udp_recv`: one line per datagram, recv.py's format | `tools/udp_recv.c` | Yes | Yes, against Daniel's sim |
| `hud_fusion`: UDP 47103, tracks R1 and R2, LOST after 1500 ms, merges one person seen by both phones, tracks out on 47102 at 30 Hz | `fusion/*` | Yes | Yes: LOST fired at 1528 ms of silence, resumed on restart |
| `hud_display`: radar map (default) or wearer view (video stubbed), heading tape, contacts with range, `range_m` null drawn as a bearing ray, optional box drawn as brackets, `NODE Rn LOST`, `FUSION LOST`, status line with process lights | `display/*` | Yes, with `sink_screen.c` against the host stub | Yes: ppm sink, and the Screen sink against the stub |
| QNX Screen sink | `display/sink_screen.c` | Only against `host-stub/` (NOT QNX) | Never on QNX |
| `hud_supervisor`: posix_spawn with IF-3 priorities, restarts after exit or 500 ms without a heartbeat | `supervisor/hud_supervisor.c` | Yes | Yes: restarted fusion after a kill and after a SIGSTOP freeze; HUD never died |
| Loopback test | `scripts/loopback_test.sh` (uses Daniel's `tools/sim/remote.py`) | n/a | 15 of 15 checks pass on the host |
| Phone stand-in for PR #1 | `scripts/ws_phone_sim.py` | n/a | Drove Daniel's real `remote/ws_ingest.py` into `hud_fusion` and the HUD |

Host screenshots are in the PR description (radar, fusion killed, both phones lost, range-less ray,
wearer view).

## What's untested or unverified

- **Anything on QNX.** No QNX compiler here. Real `<screen/screen.h>`, `-lscreen`, `-lsocket`,
  `qcc`, `_QNX_SOURCE` under `-std=c11` are all unchecked.
- Screen calls match the QNX 8.0 docs (create context, window, buffers; USAGE needs
  `SCREEN_USAGE_WRITE` before `screen_create_window_buffers`; post the first `RENDER_BUFFERS`
  entry). **Unverified:** that display index 0 is HDMI0; that `SCREEN_FORMAT_RGBX8888` stores a
  uint32 as `0x00RRGGBB` (set `display.swap_rb: true` if red and blue are swapped); whether
  `ZORDER 100` sits above the image's desktop.
- Priorities: on the host the supervisor got EPERM and fell back to inherited priority (logged).
  On QNX, priorities above the user limit need `PROCMGR_AID_PRIORITY`, so expect to need `sudo`.
- `slay` was not run (host used `pkill`). `slay` sends SIGTERM, which our processes handle, so the
  supervisor should see "exited with 0" and restart.
- Logs go to stderr and `logs/<name>.log`, not slog2, so `sloginfo` will not show them (slog2
  would be QNX-only code outside the IF-1 backends). Use `sloginfo` for Screen or driver errors.
- Wearer-view video: no longer a stub as of Sat evening; hud_display decodes IF-5.6 frames from
  TCP 47106 (see VIDEO.md, host-tested only). Two phones in the same room were only simulated.

## On the Pi (`ssh qnxuser@qnxpi-felix.local`, password `qnxuser`)

**Build on the Pi or cross-compile?** qnx-primer.md says the QSTI image includes the QNX
Developer Desktop and can build on the Pi, but Daniel found `camera_api.h` missing from it, so dev
headers may be missing too. Check first:

```bash
ssh qnxuser@qnxpi-felix.local 'uname -a; for c in qcc cc gcc cmake make python3 tar; do printf "%s: " $c; command -v $c || echo missing; done; ls /usr/include/screen/screen.h /usr/lib/libscreen.so* 2>&1'
```

Copy the draft from the laptop (from this repo, branch `felix/pi-lane-draft`):

```bash
cd drafts/pi-lane
tar --no-mac-metadata --no-xattrs -cf - --exclude ./build --exclude ./build-stub --exclude ./logs . | ssh qnxuser@qnxpi-felix.local 'mkdir -p ~/pi-lane && cd ~/pi-lane && tar xf -'
# no tar on the Pi: scp -r CMakeLists.txt cmake common config display fusion scripts supervisor tests tools qnxuser@qnxpi-felix.local:~/pi-lane/
```

**A. A compiler and `screen.h` are on the Pi:**

```bash
ssh qnxuser@qnxpi-felix.local
cd ~/pi-lane
scripts/build_pi.sh                    # or: rm -rf build && cmake -S . -B build && cmake --build build
build/test_core                        # expect "160 passed, 0 failed" (also works from inside build/)
```

No extra compiler flags are needed on QNX (the `-include sys/socket.h` workaround is gone), and
CMake picks `SEEKER_TARGET=qnx` by itself when it runs on QNX. Details in PI-BUILD.md.

**B. Not on the Pi: cross-compile on the laptop** with QNX SDP 8.0:

```bash
source ~/qnx800/qnxsdp-env.sh && qcc -V 2>&1 | grep aarch64le
cmake -S . -B build-qnx -DSEEKER_TARGET=qnx -DCMAKE_TOOLCHAIN_FILE=cmake/qnx-aarch64le.cmake
cmake --build build-qnx
ssh qnxuser@qnxpi-felix.local 'mkdir -p ~/pi-lane/build'
scp build-qnx/udp_recv build-qnx/hud_fusion build-qnx/hud_display build-qnx/hud_supervisor build-qnx/test_core qnxuser@qnxpi-felix.local:~/pi-lane/build/
```

Run (all from `~/pi-lane` on the Pi, each in its own SSH session or with `&`):

```bash
build/udp_recv 47103                                  # checkpoint 1; Ctrl+C to stop
build/hud_fusion --debug                              # one line per datagram, NODE R1 LOST / resumed
build/hud_fusion --debug --no-smooth                  # raw track positions (smoothing flags: PI-BUILD.md section 4)
build/hud_display --sink screen                       # HUD on HDMI0, radar background
build/hud_display --sink screen --background wearer --view-node R1
mkdir -p logs; sudo build/hud_supervisor --config config/seeker.json 2>logs/hud_supervisor.log &
pidin -p hud_display                                  # priority should read 50f; hud_fusion 30r
slay hud_fusion                                       # HUD keeps drawing, FUSION LOST, back in about 1 s
tail logs/hud_supervisor.log logs/hud_fusion.log      # one status line per second, restarts, children's stderr
sloginfo | tail -20                                   # Screen or system errors only
slay hud_supervisor                                   # stops it and its children
```

Feed it from the laptop (Daniel's checkout of `thisisunkno3n/hackwestern-13`):

```bash
python3 tools/sim/remote.py --pi qnxpi-felix.local --node R1 --pose=-2,3,145
python3 tools/sim/remote.py --pi qnxpi-felix.local --node R2 --pose=2,3,215
```

Use `--pose=` with the equals sign: argparse reads `--pose -2,3,145` as a missing value. If
`.local` does not resolve or the hotspot blocks Mac to Pi traffic, use the Pi's address from
`ifconfig` or the hotspot client list (qnx-setup.md). On the iPhone hotspot the Mac may reach the
Pi only over IPv6 link-local: `hud_fusion` and `udp_recv` listen on IPv4 and IPv6, but
`tools/sim/remote.py` only opens IPv4 sockets, so use the wrapper in PI-BUILD.md section 3
(`--pi 'fe80::...%en0'`). Reboot is plain `sudo shutdown`, never `-r`.

The full host loopback test also runs on the Pi if `python3` is there:
`HW13=~/hackwestern-13 scripts/loopback_test.sh`.

## Open decision: HUD background

Both are built; `display.background` in `config/seeker.json` picks one, `--background` overrides.

- `radar` (**default**, PIVOT option A): full-screen top-down room map, phones and view cones,
  fused contacts. Works with detections only.
- `wearer` (option B): one phone (`display.view_node`) is the wearer's view. Symbology is drawn in
  that phone's frame (brackets from its boxes, horizon diamonds and edge chevrons for the other
  phone, radar inset), but **the video is a stub**: nothing streams video to the Pi yet, and
  decoding it on the Pi is the heavy part PIVOT warns about.

Recommendation stays (A) for the MVP. Decide before checkpoint 2.

## Checkpoints after the pivot (proposed; update lanes-and-checkpoints.md once you agree)

The C270 checkpoints 2 and 3 are replaced by phone-based ones; old 5 merges into the new 3, and a
two-phone check takes its slot.

| # | Checkpoint | Owner | Target | Pass |
| --- | --- | --- | --- | --- |
| 1 | Packets from a phone or the sim show up in a terminal on the Pi | Daniel | Sat 10:00am | `build/udp_recv 47103` (or `recv.py`) prints one `remote_contacts R1` line per packet |
| 2 | **New:** our C HUD on HDMI0, fed by phone R1 | Felix | Sat 11:00am | `hud_display --sink screen` shows the radar map, `R1 15HZ` in the status line, 25 fps or more judged on HDMI |
| 3 | **New:** a person in front of phone R1 gets a marker with range | Daniel | Sat 1:00pm | `phone.html` to `ws_ingest.py` to `hud_fusion`: a cyan `R1-1 3.2M` diamond that moves as they walk; frames without range show a dashed ray |
| 4 | The sim drives markers; `NODE R1 LOST` | Felix | Sat 2:00pm | Ctrl+C the sim: `NODE R1 LOST` within 1.5 s; restart: markers back within 3 s |
| 5 | **New:** two phones, one person, one fused marker | Felix | Sat 5:00pm | Person in both cones shows one `R1-1+R2-1` marker; step out of one cone and the marker stays |
| 6 | Slay fusion: the HUD keeps drawing and recovers | Felix | Sat 6:00pm | `slay hud_fusion`: `FUSION LOST`, no freeze, markers back within about 1 s; `pidin` shows HUD at 50 |
| 7 | Full demo (visor, or monitor fallback) | Felix | Sat 9:00pm | demo-script.md end to end in 2 minutes |

## Interface questions for Daniel (also in the PR)

1. **Bearing formula mismatch.** `phone.html` uses `(cx - 0.5) * HFOV` (linear); `sim/remote.py`
   and `hud_remote.py` use `atan((x - cx) / f)`. They differ by a few degrees toward the edges.
   Suggest `atan` everywhere.
2. **No image size in `remote_contacts`.** `box` is in node pixels but the size is not sent, so
   the HUD cannot scale it. The draft reads `remote.nodes[].image_w/h` from config (640x480 for
   `phone.html`). Suggest adding `w`, `h` like IF-5.1. Portrait phones also swap size and HFOV.
3. **Empty box.** `ws_ingest.py` fills a missing box with `[0,0,0,0]`; the draft treats all-zero
   as "no box". `null` would be clearer.
4. **Contact ids are per-message indexes** (`R1-1`, `R1-2`), not stable tracks, so two people can
   swap labels. IF-7 shows `R1-P1`. Fine for one person per phone.
5. **Pose precedence.** IF-5.4 says pose comes from the node. If the phone page has no `pose=`
   it sends `0,0,0`, which puts the phone on the wearer's mark. The draft uses the message pose
   unless `remote.pose_source` is `"config"`.
6. **Where does `ws_ingest.py` run?** Its `--fusion 127.0.0.1` default implies on the Pi, which
   needs Python 3 on the Pi (still unconfirmed). Default port is 8080; the PR text uses 8099.
7. **Range from pose landmarks.** `phone.html` uses the landmark box height (no head top) with
   1.7 m, so range will read long. OK for colour bands.
8. **INTERFACES.md is stale after the pivot** (C270, `hud_capture`, `hud_infer`, Mac relay, "one
   `node_id` per Mac", `visible` and `src: local` in IF-5.2). PIVOT step 3 says that PR is yours.
9. **PROPOSED additions in this draft** that need that INTERFACES.md PR before they go into code:
   a `nodes` list in `tracks` (state, age, pose, hfov, rate per node, so the HUD draws `NODE Rn
   LOST` without its own timers); per-track `node_id`, `nodes`, `node_bearing_deg`, `room_x_m`,
   `room_y_m`, `node_box`; config keys `remote.assoc_m`, `remote.pose_source`,
   `remote.nodes[].hfov_deg/image_w/image_h`, `colours.remote_by_range`, `display.background`,
   `display.view_node`, `display.radar_m`, `display.swap_rb`, `display.display_index`, and the
   `fusion` and `supervisor` sections. All tell-Daniel-first items.
10. **Colours.** With no wearer camera every contact is "remote-only", so IF-7 makes everything
   cyan. `colours.remote_by_range: true` switches to red/amber/green by range from the wearer.
