# Build the Pi lane on the QNX Pi

Nothing in this file counts as a test pass until the binaries are built
with `gcc` on the Pi and actually run there under QNX.

The context repo is private, and `git` may not be installed on the Pi.
Do not rely on `git clone` or a GitHub tarball from the Pi. Copy a tree
from the Mac with `tar` and `scp`. The Pi already has `tar`, `gcc`,
`cc`, `cmake`, `make`, and `python3`. It does not have `qcc` or
`<screen/screen.h>`. Whether it has `curl`, `wget`, `git`, or `scp` is
unverified.

Replace `HOST` with however you normally reach the Pi from the Mac
(address or `qnxpi-felix.local`). `qnxpi-felix.local` did not resolve
when used from the Pi itself.

## 1. On the Mac (not inside the ssh session)

```bash
cd /path/to/hackwestern-13-context
git pull
tar --no-mac-metadata --no-xattrs --exclude 'drafts/pi-lane/build*' \
  -czf /tmp/pi-lane.tar.gz drafts/pi-lane
scp /tmp/pi-lane.tar.gz qnxuser@HOST:~/
```

`--no-mac-metadata --no-xattrs` (macOS `bsdtar`) keeps Finder metadata
out of the tarball, so QNX `tar` stops printing `LIBARCHIVE.xattr`
warnings.

If `scp` cannot find `HOST`, copy the tarball however you usually move
files onto the Pi.

## 2. On the Pi (already logged in as `qnxuser`, home `~`)

```bash
mkdir -p ~/seeker
cd ~/seeker
tar xzf ~/pi-lane.tar.gz
cd drafts/pi-lane
uname -a
for c in qcc cc gcc cmake make python3 tar curl wget git; do
  printf "%s: " "$c"
  command -v "$c" || echo missing
done
```

Preferred build (cmake is on the Pi):

```bash
cd ~/seeker/drafts/pi-lane
rm -rf build
mkdir -p build
cd build
cmake ..
make -j2
./test_core            # from build/
cd .. && build/test_core   # from drafts/pi-lane; both print "352 passed, 0 failed"
```

No extra flags: `SEEKER_TARGET` defaults to `qnx` when CMake runs on QNX,
and the sources include `sys/socket.h` themselves, so the old
`-DCMAKE_C_FLAGS="-include sys/socket.h"` workaround is gone. `rm -rf
build` drops an old cache that may still hold `SEEKER_TARGET=laptop`.
`test_core` finds `config/seeker.json` from either folder.

CMake should warn that the official `screen.h` is missing and that it is
using `third_party/qnx-screen`. Binaries land in `build/`: `hud_display`,
`hud_fusion`, `hud_supervisor`, `seeker_supervisor`, `udp_recv`, `test_core`,
`test_supervisor`.

`seeker_supervisor`, `scripts/run_demo.sh` and the checkpoint 6 and 7 steps
(slay fusion, full demo) are in [CHECKPOINTS-6-7.md](CHECKPOINTS-6-7.md).
`(cd build && ctest --output-on-failure)` runs `test_core`, `test_supervisor`,
`tests/test_supervisor.sh` and `scripts/video_loopback_test.sh` twice
(`video_loopback` on TCP 47116, and `video_loopback_slow` on 47126 with a late
first frame and a sender that lingers after closing). Stop any running demo
first: they bind the same UDP ports. The video tests need `python3`.

```sh
cd ~/seeker/drafts/pi-lane && sh scripts/stop-live.sh && (cd build && ctest --output-on-failure)
```

On QNX a second UDP bind of the same port fails even with `SO_REUSEADDR` (Linux lets it share),
so a `hud_fusion` left from a live run makes the test's `hud_fusion` exit at start and restart in
a loop. `supervisor_restart` now checks UDP 47103 and 47102 first and fails once with
`section 3 not run: udp/47103 already held by another process` instead of a cascade of empty-pid
FAIL lines.

Alternative, no extra cmake flags:

```bash
cd ~/seeker/drafts/pi-lane
scripts/build_pi.sh
```

That script also uses the fallback header when the official one is
absent, defines `_QNX_SOURCE`, links `-lsocket` and `-lscreen`, and
checks whether `gcc` predefines `__QNX__`.

A successful link is still not a test pass. Run the programs on the Pi
(display on the 27" 4K monitor) before treating anything as verified.

## 3. Send contacts from the Mac over IPv6

On the iPhone hotspot the Mac may reach the Pi only over IPv6 link-local
(IPv4 `172.20.10.x` was unreachable from the Mac on Sat morning).
`hud_fusion` and `udp_recv` listen on UDP 47103 for both IPv4 and IPv6
(one socket each; the startup line says `IPv4 + IPv6`, or `IPv4 only` if
the IPv6 socket could not be created). The local `127.0.0.1` paths are
unchanged.

On the Pi, start fusion and note the `inet6 fe80::...` address of the
interface on the hotspot:

```bash
cd ~/seeker/drafts/pi-lane
ifconfig | grep inet6
build/hud_fusion --debug --no-health
```

On the Mac, `PI6` is the Pi's link-local address plus `%` and the Mac's
interface on the hotspot (usually `en0`; `ifconfig en0 | grep inet6`):

```bash
PI6='fe80::1234:56ff:fe78:9abc%en0'
# one datagram:
printf '%s' '{"type":"remote_contacts","v":1,"src":"R1","t_mono_ms":1,"node_id":"R1","pose":{"x_m":0,"y_m":0,"yaw_deg":0},"contacts":[]}' \
  | nc -6 -u -w1 "$PI6" 47103
# Daniel's sim (tools/sim/remote.py only opens AF_INET sockets, so wrap it to use IPv6):
cd ~/hackwestern-13
python3 -c 'import runpy, socket, sys; S = socket.socket; socket.socket = lambda *a, **k: S(socket.AF_INET6, socket.SOCK_DGRAM); sys.argv[0] = "tools/sim/remote.py"; runpy.run_path(sys.argv[0], run_name="__main__")' \
  --pi "$PI6" --node R1 --pose=-2,3,145
```

Fusion should print `rx R1 from [fe80::...%<iface>]:<port>`. IPv4
senders print as `rx R1 from 172.20.10.2:<port>`.

## 4. Track smoothing in `hud_fusion`

`hud_fusion` smooths each fused track's room position before it sends
`tracks` on UDP 47102, so `room_x_m`, `room_y_m`, `range_m` and
`bearing_deg` are all filtered. A track with no room position (one
phone's bearing-only contact that no other phone's ray crosses) is not
filtered, because `phone.html` already smooths bearings (`?smooth=0.35`,
`?confmin=0.45`). Two crossing bearing-only contacts do get a room
position (section 5), so they are filtered.

The filter is a constant-velocity alpha-beta filter per track. It
updates once per new datagram, not once per 30 Hz tick. A person walking
at a steady pace has no steady-state lag, and the noise variance of a
standing person drops to about 40% with the defaults. dt comes from the
message clock (`t_mono_ms`, else `ts_ms`), or from Pi receive time if
that clock is missing or goes backwards. dt is clamped to
`smooth_max_dt_ms`. The filter restarts from the next fix (no sliding)
when the track is new, after it was absent for `smooth_reset_ms` (this
includes a node coming back from LOST), or when a fix lands more than
`smooth_gate_m` from the prediction.

| Config key (`fusion.*`) | Flag | Default | Meaning |
| --- | --- | --- | --- |
| `smooth` | `--no-smooth` | `true` | Turn smoothing off (raw positions, as before) |
| `smooth_alpha` | `--smooth-alpha a` | `0.5` | Position gain, 0 to 1; higher follows fixes faster |
| `smooth_beta` | `--smooth-beta b` | `0.15` | Velocity gain, 0 to 1; 0 means no velocity term |
| `smooth_gate_m` | `--smooth-gate m` | `1.5` | A jump farther than this snaps to the new fix |
| `smooth_max_dt_ms` | none | `250` | Largest dt the filter uses between fixes |
| `smooth_reset_ms` | none | `500` | Track absent this long restarts its filter |

A **single-phone** track (one phone's mono size range, no second ray) does
not use the room filter. Its range alone goes through a 1D alpha-beta
filter along the ray (`range_smooth*`), and the bearing passes through
unfiltered (it is already accurate). On the host at 15 Hz with the defaults, range jitter
drops 2.5x (std ratio 0.40 at 10% sigma noise), and a 1 m step reaches
90% in 462 ms, overshoots by 0.15 m, and is back within 0.1 m by about
1.7 s. A person walking steadily has no steady-state lag. When a fused
track loses its second phone, the range filter starts from the fused
position instead of jumping to the mono range.

| Config key (`fusion.*`) | Flag | Default | Meaning |
| --- | --- | --- | --- |
| `range_smooth` | `--no-range-smooth` | `true` | Single-phone range filter (off: room filter instead) |
| `range_smooth_alpha` | none | `0.2` | Range gain per datagram |
| `range_smooth_beta` | none | `0.015` | Range-rate gain per datagram |
| `range_smooth_gate_frac` | none | `0.65` | Snap when the range jumps more than max(`smooth_gate_m`, frac x range) |

Flags override `config/seeker.json`. At startup fusion logs
`smoothing tracks: alpha 0.50 beta 0.15 gate 1.5 m` (or `smoothing off`),
`single-phone range filter: ...`, `triangulation: ...` and
`track identity: ...`. With `--debug` it logs one line per track per new
datagram, for example
`track 1 R1+R2 intersect cross 67 deg sigma 0.32 m fix -0.25,-0.03 -> -0.25,-0.03 range 0.25 m bearing -96.1`
or `track 2 R1 single size range 4.12 -> 3.98 m (sigma 1.32 m) bearing 122.3 range 0.56 m`,
plus `track N snapped X m (gate 1.5 m)` or `track N range snapped ...`
when a track jumps past its gate.

```bash
build/hud_fusion --debug                                  # defaults
build/hud_fusion --debug --smooth-alpha 0.7 --smooth-beta 0.2   # snappier
build/hud_fusion --debug --no-range-smooth                # single-phone tracks on the room filter
build/hud_fusion --debug --no-smooth                      # raw, for comparison
```

### Triangulation weighting and close-together people

When two phones see the same person, the fix is the inverse-variance
blend of the bearing intersection and the size ranges. The size range
sigma is sqrt((`tri_range_sigma_frac` r)^2 + (`tri_range_sigma_k` r^2)^2),
growing with range squared (box-height pixel jitter), so well-crossed
bearings dominate. Rays closer than `tri_min_cross_deg` to parallel use
the size ranges instead (bearing-only rays still intersect down to
`tri_min_cross_bearing_only_deg`). Host Monte Carlo (phones 4 m apart,
67 deg crossing, bearings 2 deg and ranges 20% sigma): triangulated RMS
0.20 m against 0.66 m for one phone's size range.

Close-together people:

- Two contacts from one phone are never fused into one track.
- Each phone's rays are matched to the other phone's by gated
  minimum-cost assignment (cost = crossing residual + range disagreement
  + distance to predicted tracks), not greedy nearest. In the 2x2 case,
  a cheap ghost crossing therefore cannot force the wrong pairing.
- Track ids follow predicted positions. A track with no fix coasts
  (kept, not sent) for `remote.drop_ms`. Two people crossing keep their
  ids even when the phones' contact labels swap.

| Config key (`fusion.*`) | Default | Meaning |
| --- | --- | --- |
| `tri_bearing_sigma_deg` | `3.0` | Bearing error per ray, 1 sigma |
| `tri_range_sigma_frac` | `0.20` | Size range sigma, linear part (person height guess) |
| `tri_range_sigma_k` | `0.05` | Size range sigma, quadratic part, 1/m (pixel jitter) |
| `tri_min_cross_deg` | `15` | Below this crossing angle, use the size ranges |
| `tri_min_cross_bearing_only_deg` | `5` | Same, when no ray has a range |
| `tri_track_prior_sigma_m` | `0.5` | Pairing prefers fixes near predicted tracks |
| `tri_track_prior_cap` | `9` | Cap of that prior (chi-square), so new people still pair |
| `track_sigma_m` | `0.3` | Track position uncertainty for id matching |
| `track_gate_sigmas` | `3.0` | Id gate in sigmas of track and fix uncertainty |
| `track_gate_max_m` | `3.0` | Id gate cap (floor is `smooth_gate_m`) |

## 5. Phone poses and triangulation (checkpoint 5)

`hud_fusion` turns every contact into a ray from its phone's pose in the
IF-1 room frame (origin on the wearer's floor mark, `+y` forward, `+x`
right, `yaw_deg` clockwise from `+y`). When R1 and R2 see the same person,
the two rays are triangulated into one track (`nodes: "R1+R2"`, one
marker on the HUD), even when both phones send `range_m: null`.

Pose flags (same `x_m,y_m,yaw_deg` numbers as phone.html's `?pose=`):

| Flag | Meaning |
| --- | --- |
| `--pose ID=x_m,y_m,yaw_deg` | Pin one phone's pose. Repeat per phone. Spaces are allowed: `--pose "R2 = 2, 3, 215"`. |
| `--poses FILE` | Pin poses from a file: one `ID=x_m,y_m,yaw_deg` per line, `#` comments. Example: `config/poses.example`. |
| `--pose-source message\|config` | For phones without a pinned pose: `message` (default, from `remote.pose_source`) lets a `pose` in `remote_contacts` replace the `remote.nodes` pose; `config` ignores message poses. |

Precedence for one phone: `--pose` beats `--poses`, a pinned pose beats
the message `pose`, and the message `pose` beats `remote.nodes` unless
`--pose-source config`. A `pose: null` (or missing or partial) message
pose never changes anything. A bad `--pose` or pose file line exits with
code 2 and says which line. The startup log prints every node's pose and
where it came from, e.g. `node R1 pose -2.00,3.00,145.0 (cli, pinned)`.

Measure each phone with a tape: `x_m`, `y_m` from the wearer's floor mark,
and `yaw_deg` as the direction the camera points, clockwise from the
wearer's forward (0 = the way the wearer faces, 90 = the wearer's right,
180 = back toward the wearer). On the Pi:

```bash
cd ~/seeker/drafts/pi-lane
build/hud_fusion --debug --no-health --pose R1=-2,3,145 --pose R2=2,3,215
# or: build/hud_fusion --debug --no-health --poses config/poses.example
```

With `--debug`, each fused track prints at most once a second:
`fused track 1 R1+R2 room -0.40,-0.60 range 0.72 m bearing -146.3 conf 0.79`.
To pin poses under the supervisor, add the flags to the `hud_fusion`
`argv` in `supervisor.children` of the config you pass to `hud_supervisor`
(`config/seeker.json` already passes `--poses config/poses`). For
`seeker_supervisor`, pass `--poses FILE` and `--pose ID=...`; its wrapper
and `scripts/run_demo.sh` use `POSES=config/poses` by default and `POSE`
for one-off pins (see [CHECKPOINTS-6-7.md](CHECKPOINTS-6-7.md)). Edit
`config/poses` with the measured positions before the demo.

`main`'s `remote/ws_ingest.py` (and phone.html without `?pose=`) still
sends `pose {0,0,0}` for every phone, which would move both phones onto
the wearer's mark. `hud_fusion` logs `NODE R1 sends pose 0,0,0 (old relay
default?)` once when that happens. Pin the poses with `--pose` (or use
`--pose-source config`) until Daniel's `daniel/v2-conform` (pose `null`
when not given) is on `main`.

How the fix is computed (`fusion/seeker_triangulate.c`): a weighted
least-squares point over all rays in a cluster. Each ray adds a cross-ray
term (bearing error 3 deg, so the allowed sideways error grows with
distance) and, if `range_m` is known, an along-ray term (25% of the
range). Both are divided by the contact's `conf`. With two crossing
bearings this is the bearing intersection, nudged along each ray by its
range. Fallbacks:

- Bearings within 5 deg of parallel, or crossing behind a phone (for
  example phones facing each other across the person): the range points
  are averaged, weighted by range and confidence.
- One phone only (the other LOST or not seeing the person): that phone's
  own range point; a lone bearing-only contact keeps just its bearing ray.

Two contacts are only fused if the fix is in front of both phones, within
20 m, within each ray's sideways gate, and within `max(remote.assoc_m,
50% of range_m)` of each reported range. Pairs are matched lowest cost
first, and a third phone joins the cluster it fits best. With two people
and bearing-only contacts from two phones the pairing can be ambiguous
(ghost crossings); a range from either phone resolves it.

## 6. EagleEye wearer view (helmet phone `W`)

Full guide: [docs/EAGLEEYE.md](docs/EAGLEEYE.md). After building:

```bash
cd ~/seeker/drafts/pi-lane
scripts/eagleeye_test.sh                 # loopback with the scene sim, "0 failure(s)"
build/hud_fusion --config config/seeker.eagleeye.json &
build/hud_display --config config/seeker.eagleeye.json --view wearer --sink screen &
python3 scripts/eagleeye_sim.py --wearer-turn 40
```

Visor: add `--visor --mirror`. Fallback: `--view radar`.

## Optional: tarball from GitHub, only if you have network plus auth

This repo is private, so an unauthenticated `curl`/`wget` to
`codeload.github.com` will fail. If you later make a snapshot public, or
have a token, try in this order (`curl` and `wget` may be missing):

```bash
URL="https://codeload.github.com/thisisunkno3n/hackwestern-13-context/tar.gz/refs/heads/main"
# curl, if present:
curl -L -o /tmp/ctx.tar.gz "$URL"
# else wget, if present:
wget -O /tmp/ctx.tar.gz "$URL"
# else python3 (present on this image):
python3 -c "import urllib.request; urllib.request.urlretrieve('$URL', '/tmp/ctx.tar.gz')"
mkdir -p ~/seeker && cd ~/seeker && tar xzf /tmp/ctx.tar.gz
# the tree unpacks as hackwestern-13-context-main/drafts/pi-lane
```

Prefer the Mac `tar` + `scp` path.

## 4. Video on hud_display (IF-5.6, TCP 47106)

Details in [VIDEO.md](VIDEO.md). Rebuild on the Pi after copying the tree over (section 1 and 2;
the tarball now includes `third_party/stb/`, which hud_display needs):

```bash
cd ~/seeker/drafts/pi-lane
rm -rf build && mkdir -p build && cd build && cmake .. && make -j4 && cd ..
build/test_core                       # "0 failed"; includes SKV2 framing, JPEG decode, TCP 47106 over IPv4 and ::1
scripts/video_loopback_test.sh        # needs python3; ppm sink, prints ALL PASS
SENDER_EXIT_DELAY=0.5 FIRST_FRAME_DELAY=1 PPM_EVERY=10 scripts/video_loopback_test.sh   # slow timing
```

Or `scripts/build_pi.sh` (no cmake). Either works with `cc` as clang or gcc.

Run on HDMI with the test card (two SSH sessions, or `&`):

```bash
build/hud_display --sink screen --background wearer --view-node R1
python3 scripts/video_send.py                     # on the Pi; or from the Mac: --host qnxpi-felix.local
```

Expect the test card pillarboxed (640x480 on a 16:9 screen), red top left, `CAM R1`, `VID 15FPS`
in the status line, and `video: first frame from ...` in the log. Ctrl+C the sender: `NO VIDEO`
banner over the full radar within one frame. Red and blue swapped: `display.swap_rb: true`.
Under the supervisor (50 FIFO) the receiver thread should show as priority 40 in `pidin`;
the log line `video: listening on tcp/47106 (IPv4 + IPv6) ... receiver rr 40` confirms it.

