# Checkpoints 6 and 7 on the QNX Pi: supervisor, chaos test, full demo

Numbering follows [MORNING.md](MORNING.md): **6** = slay `hud_fusion`, HUD shows `FUSION LOST`,
the supervisor restarts it; **7** = full demo. In
[lanes-and-checkpoints.md](../../lanes-and-checkpoints.md) the same two are numbered 7 and 8.

Nothing here counts as passed until it runs on `qnxpi-felix.local` under QNX 8.0. The host runs
listed at the end are only a pre-filter. Felix verifies on the Pi.

## What is new

| File | What it does |
| --- | --- |
| `supervisor/seeker_supervisor.c` | Small C supervisor: starts `hud_fusion` and `hud_display`, restarts either one on any exit or crash with exponential backoff, logs every restart, stops both cleanly on Ctrl+C, `slay seeker_supervisor` (SIGTERM) or SIGHUP |
| `supervisor/seeker_sv.c`, `.h` | Pure helpers (backoff, `50f` style priority specs, exit status text), unit tested |
| `scripts/seeker_supervisor.sh` | Wrapper: `cd` to `drafts/pi-lane`, checks binaries, refuses a second copy, `exec`s the binary |
| `scripts/run_demo.sh` | `preflight`, `start [--bg] [--sim] [--video-sim] [--no-ingest]`, `chaos`, `video stop\|start\|pause\|resume`, `status`, `stop` |
| `tests/test_supervisor.c` | 63 unit checks (ctest `supervisor_unit`) |
| `tests/test_supervisor.sh` | 57 integration checks with real children, 51 without `python3` (ctest `supervisor_restart`) |
| `config/poses` | Demo room phone poses (`R1=x_m,y_m,yaw_deg`), pinned in `hud_fusion`; measure and edit |

`config/seeker.json` now also passes `--poses config/poses` to `hud_fusion` and
`--background wearer --view-node R1 --video-port 47106` to `hud_display` in
`supervisor.children`, so `hud_supervisor` runs the same demo setup.

`hud_supervisor` (config driven, fixed 500 ms backoff) is unchanged. Run **one** supervisor at a
time: both claim `/seeker_health` and would start two copies of each child.

## seeker_supervisor in short

```
seeker_supervisor [--bin-dir build] [--config f] [--log-dir logs|-] [--only fusion|display]
                  [--fusion path] [--display path] [--fusion-arg a]... [--display-arg a]...
                  [--pose ID=x_m,y_m,yaw_deg]... [--poses FILE]
                  [--background radar|wearer] [--view-node ID] [--video-port N]
                  [--fusion-prio 30r] [--display-prio 50f] [--self-prio 45f]
                  [--backoff-min-ms 500] [--backoff-max-ms 8000] [--stable-ms 5000]
                  [--hang-ms 0] [--stop-timeout-ms 2000] [--status-s 10]
                  [--ignore-hup] [--no-health] [--run-for s] [--check]
```

- **Phone poses (PR #12):** `--poses FILE` and `--pose ID=x_m,y_m,yaw_deg` (repeatable) are
  passed to `hud_fusion` on every start, including restarts. The supervisor parses them first
  with `fusion/seeker_pose.c`, so a typo or a missing file exits 2 at startup instead of
  crash-looping fusion, and the log lists them:
  `hud_fusion pose R1 -2.00,3.00,145.0 pinned from config/poses`. `--pose` beats the file for one
  phone, as in `hud_fusion`. The wrapper and `run_demo.sh` pass `--poses config/poses` by default
  (env `POSES`, `POSES=none` to skip) and add `POSE="R1=x,y,yaw R2=..."` pins. Pin them for now:
  `main`'s relay still sends `pose 0,0,0`, which would put both phones on the wearer's mark.
- **Video (PR #13):** `--background radar|wearer`, `--view-node ID` and `--video-port N` are
  validated and passed to `hud_display` on every start. In the wearer view `hud_display` draws
  the `view_node` phone's JPEG frames from TCP `--video-port` (IF-5.6) full screen with a radar
  inset, and falls back to the full radar under `NO VIDEO` (no sender connected) or `VIDEO LOST`
  (connected, nothing for 2 s; from 0.5 s the last picture stays up dimmed, `VIDEO STALE`). The
  log line reads
  `hud_display background wearer, view node R1, video tcp/47106`. The wrapper takes them from
  `BACKGROUND`, `VIEW_NODE` and `VIDEO_PORT`; `run_demo.sh` defaults to `wearer`, `R1`, `47106`.
  A sender reconnects after a `hud_display` restart, so video comes back with the display.
- **`--check`:** validates everything, prints both command lines and the pinned poses, exits 0
  without starting anything (`run_demo.sh preflight` uses it).

- **Restart policy:** any exit (status 0 from `slay`, a non-zero status, or a signal such as
  SIGSEGV) schedules a restart. Delays double from `--backoff-min-ms` up to `--backoff-max-ms`;
  a child that ran at least `--stable-ms` starts again from the minimum. A failed spawn is retried
  the same way. `--hang-ms N` also SIGKILLs a child whose `/seeker_health` beat is older than N ms
  (off by default).
- **Stop:** SIGTERM to both children, wait `--stop-timeout-ms`, then SIGKILL; exit status 0. A
  second Ctrl+C skips the wait. Children stay in the terminal's process group, so a Ctrl+C reaches
  them too, and they exit through their own handlers.
- **Priorities (IF-3):** `hud_display` 50 FIFO, `hud_fusion` 30 RR, the supervisor 45 FIFO, set at
  spawn with `POSIX_SPAWN_SETSCHEDULER`. The `<prio>[f|r|o]` form is the one QNX `on -p` and
  `slay -P` use, and `pidin` prints it back the same way.
- **Logs:** `logs/seeker_supervisor.log` (also on stderr), `logs/hud_fusion.log` and
  `logs/hud_display.log` (children's stdout and stderr, one banner per start), `logs/<name>.pid`,
  and one `restart` line per restart in `logs/events.jsonl` (IF-9).
- **HUD lights:** the supervisor recreates `/seeker_health`, claims the `hud_fusion` and
  `hud_display` slots and beats the `hud_supervisor` slot, so the HUD's `SUP` light is green.

Restart line examples (from the host run):

```
[seeker_supervisor 17:52:35.445] hud_fusion pid 3498 exited with status 0 after 8.3 s; restart #1 in 1500 ms
[seeker_supervisor 17:52:36.952] started hud_fusion pid 3580 prio inherit (start #2, restarts 1, down 1506 ms)
[seeker_supervisor 17:48:25.490] hud_fusion pid 2421 killed by SIGSEGV (11) after 0.8 s; restart #2 in 600 ms
[seeker_supervisor 17:53:44.087] hud_fusion pid 3952 killed by SIGKILL (9) after 2.1 s; restart #3 in 1200 ms
```

On the Pi the `prio` field should read `30r` / `50f`; `inherit` means the priority was refused
(the log says why).

### Why the demo uses a 1500 ms first restart

`hud_display` shows `FUSION LOST` once tracks are more than 500 ms old (`FUSION_STALE_MS` in
`display/hud_render.c`). With the IF-6 backoff of 500 ms, fusion is back about 50 ms after the
banner appears, so nobody would see it. `run_demo.sh` therefore passes `--backoff-min-ms 1500`
(env `RESTART_MS`): the banner shows for about 1 s and markers return about 1.5 s after the slay.
For the fastest recovery, run with `RESTART_MS=500`.

## QNX 8.0 facts this relies on (checked against the QNX 8.0 docs)

- [`posix_spawn()`](https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.lib_ref/topic/p/posix_spawn.html)
  returns an error number (not -1 and `errno`). `POSIX_SPAWN_SETSCHEDULER` with
  `posix_spawnattr_setschedpolicy()` / `setschedparam()` sets the child's policy and priority;
  `SETSIGMASK` and `SETSIGDEF` are POSIX flags accepted by
  [`posix_spawnattr_setflags()`](https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.lib_ref/topic/p/posix_spawnattr_setflags.html)
  (a `short`). A failure that happens after the spawn returns shows up as the **child exiting
  with status 127**; the supervisor treats a quick 127 after a prioritized spawn as "priority
  refused" and retries at the inherited priority. Ignored signals are inherited, which is why
  `SETSIGDEF` resets SIGINT, SIGTERM, SIGQUIT, SIGPIPE (and SIGHUP unless `--ignore-hup`).
- [`waitpid()`](https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.lib_ref/topic/w/waitpid.html)
  with `-1` and `WNOHANG` returns 0 while children run. Do not set SIGCHLD to `SIG_IGN` (children
  would not become zombies and could not be reaped); the supervisor leaves it at the default.
- [`on -p`](https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.utilities/topic/o/on.html):
  priorities 1 to 63 are usable without privilege; 64 and up need `PROCMGR_AID_PRIORITY`
  (procnto `-P` can move that line). So 50, 45 and 30 should work as `qnxuser` without `sudo`.
- [`slay`](https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.utilities/topic/s/slay.html)
  sends SIGTERM by default, matches the name without the path, and **prompts** when more than
  one process has the name. Scripts use `slay -f` (no prompt). Its exit status is the number of
  processes it signalled, not success or failure, so scripts never test it.
- [`pidin`](https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.utilities/topic/p/pidin.html)
  `-p name` shows one process; the priority column reads `50f` (FIFO) or `30r` (round robin).
- [`sh`](https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.utilities/topic/s/sh.html)
  is the public-domain Korn shell ([`ksh`](https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.utilities/topic/k/ksh.html)).
  The scripts stay POSIX: no arrays, no `[[ ]]`, no `local`, `wait` with one pid (pdksh `wait`
  takes a single job), traps by name (`INT TERM HUP`). In a non-interactive shell, background
  jobs start with SIGINT ignored and a trap cannot re-enable it, so `run_demo.sh` stops background
  helpers with SIGTERM.
- [`nohup`](https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.utilities/topic/n/nohup.html)
  is a toybox utility and may be missing; `run_demo.sh start --bg` uses it only if present and
  also passes `--ignore-hup`.
- Not found in the QNX 8.0 docs: whether `sleep` accepts fractions. Scripts use whole seconds,
  except `tests/test_supervisor.sh`, which probes `sleep 0.1` first.

## One-time setup on the Pi

On the Mac, from this repo (same tarball route as [PI-BUILD.md](PI-BUILD.md)):

```bash
cd /path/to/hackwestern-13-context && git pull
tar --no-mac-metadata --no-xattrs --exclude 'drafts/pi-lane/build*' --exclude 'drafts/pi-lane/logs' \
  -czf /tmp/pi-lane.tar.gz drafts/pi-lane
scp /tmp/pi-lane.tar.gz qnxuser@HOST:~/
```

On the Pi (`ssh qnxuser@qnxpi-felix.local`, password `qnxuser`). `cc` is Clang there; no `qcc`:

```bash
mkdir -p ~/seeker && cd ~/seeker && tar xzf ~/pi-lane.tar.gz && cd drafts/pi-lane
rm -rf build && cmake -S . -B build && cmake --build build -j4
build/test_supervisor                      # expect "63 passed, 0 failed"
build/test_core                            # expect "229 passed, 0 failed" (or more)
sh tests/test_supervisor.sh build          # expect "== 0 failure(s)" (about 20 s; demo must be stopped)
sh scripts/video_loopback_test.sh          # PR #13's video checks (needs python3)
# or all three: (cd build && ctest --output-on-failure)
```

No cmake: `sh scripts/build_pi.sh` builds the same binaries, including `seeker_supervisor` and
`test_supervisor`.

TLS for iPhone Safari (camera needs HTTPS). If `openssl` is on the Pi, run it there; otherwise run
it on the Mac and `scp` both files to `~/hackwestern-13/` on the Pi:

```bash
openssl req -x509 -newkey rsa:2048 -nodes -keyout key.pem -out cert.pem -days 7 -subj /CN=seeker
```

`run_demo.sh` looks for Daniel's relay in `$HW13/remote/ws_ingest.py` (default
`~/hackwestern-13`, else `../../../hackwestern-13`) and for `$HW13/cert.pem` and
`$HW13/key.pem` (override with `CERT=` and `KEY=`).

Phone poses: tape out where R1 and R2 stand (from the wearer's floor mark, `+y` forward, `+x`
right, `yaw_deg` = camera direction clockwise from forward; details in
[PI-BUILD.md](PI-BUILD.md) section 5) and edit the two lines in `config/poses`. Check them with:

```bash
sh scripts/run_demo.sh preflight           # prints "pose R1 ... (config/poses)" for each phone
# one-off override without editing the file:
POSE="R1=-1.8,2.9,150" sh scripts/run_demo.sh preflight
```

## Checkpoint 6: slay fusion, HUD shows FUSION LOST, supervisor restarts it

Two SSH sessions on the Pi, both in `~/seeker/drafts/pi-lane`.

**Session A** (phones optional: `--sim` feeds R1 and R2 from Daniel's sim instead, and
`--video-sim` streams the `scripts/video_send.py` test card as R1's video):

```bash
cd ~/seeker/drafts/pi-lane
export HW13=~/hackwestern-13
sh scripts/run_demo.sh preflight
sh scripts/run_demo.sh start --sim --video-sim   # foreground; Ctrl+C stops everything
```

**Session B:**

```bash
cd ~/seeker/drafts/pi-lane
sh scripts/run_demo.sh chaos               # slay -f hud_fusion, then checks and pidin
pidin -p hud_display; pidin -p hud_fusion; pidin -p seeker_supervisor
tail -5 logs/seeker_supervisor.log
tail -3 logs/events.jsonl
```

Pass when all of these hold:

- [ ] `chaos` prints `PASS supervisor saw the slay: hud_fusion pid N exited with status 0 ...`
- [ ] `chaos` prints `PASS hud_fusion restarted as pid M: start #2, restarts 1, down ~1500 ms`
- [ ] `chaos` prints `PASS hud_display untouched (pid ... still running)`
- [ ] On HDMI: `FUSION LOST` appears within about 0.5 s over the still-moving test card, contacts
      turn grey, `FUS` light red, `SUP` light green; the HUD never freezes (the card's frame
      counter keeps counting, `VID 15FPS` and the FPS in the status line stay up)
- [ ] On HDMI: banner clears and live contacts return about 1.5 s after the slay
- [ ] `pidin` priority column: `hud_display` `50f`, `hud_fusion` `30r`, `seeker_supervisor` `45f`
- [ ] `logs/seeker_supervisor.log` start lines read `prio 30r` and `prio 50f` (not `inherit`)
- [ ] `logs/hud_fusion.log` after the restart again shows
      `node R1 pose ... (file, pinned)` and `node R2 pose ... (file, pinned)`
- [ ] Ctrl+C in session A: `stopped cleanly`, prompt returns, and
      `pidin -p hud_fusion; pidin -p hud_display` show no processes

Also try by hand (each should restart fusion with the next backoff step, HUD untouched):
`slay hud_fusion`, `slay -9 hud_fusion` (SIGKILL), and for the watchdog start with
`sh scripts/seeker_supervisor.sh --hang-ms 1000` and freeze fusion with `slay -h hud_fusion`
(expect `no heartbeat for ... ms; sending SIGKILL`, then a restart).

## Checkpoint 7: full demo (two phones, fused marker, video, restart)

`hud_display` has the TCP 47106 video receiver (PR #13). Daniel's `ws_ingest.py` does not
forward phone video yet (no `--video-port`; `preflight` says so). Until it does,
`scripts/video_send.py` stands in for it: `start --video-sim` streams its test card (or your
JPEGs with `VIDEO_JPEG="a.jpg b.jpg"`) as R1's video to `127.0.0.1:47106`, exactly the IF-5.6
frames the relay will send. Once `preflight` reports `ws_ingest.py has --video-port`, drop
`--video-sim`: `run_demo.sh` then passes `--video-host 127.0.0.1 --video-port $VIDEO_PORT` to the
relay itself.

**Session A:**

```bash
cd ~/seeker/drafts/pi-lane
export HW13=~/hackwestern-13
sh scripts/run_demo.sh preflight           # all PASS; prints the phone URLs and both command lines
sh scripts/run_demo.sh start --video-sim   # phones for detections, video_send.py for video
# once the relay forwards video: sh scripts/run_demo.sh start
# radar only: BACKGROUND=radar sh scripts/run_demo.sh start
# real photos instead of the test card: VIDEO_JPEG="shot1.jpg shot2.jpg" sh scripts/run_demo.sh start --video-sim
# meant to survive an SSH drop (unverified on QNX): add --bg, then `status` / `stop`
```

**Session B** drives the video steps:

```bash
sh scripts/run_demo.sh video pause         # sender stays connected but silent -> VIDEO LOST
sh scripts/run_demo.sh video resume        # video back
sh scripts/run_demo.sh video stop          # sender gone -> NO VIDEO, full radar
sh scripts/run_demo.sh video start         # video back within about 1 s
sh scripts/run_demo.sh chaos               # restart beat over live video
```

Then, in order:

- [ ] `preflight` shows `== 0 problem(s)` (binaries, config, poses, python3, relay, video stand-in,
      TLS, `pidin`, `slay`), lists `pose R1 ...` and `pose R2 ...` with the measured numbers, and
      prints `hud_display: build/hud_display --background wearer --view-node R1 --video-port 47106 ...`
- [ ] `run_demo.sh start` printed `started ws_ingest pid ...` and the
      `https://qnxpi-felix.local:8080/?node=R1...` and `?node=R2` URLs (raw IPs do not load on the
      iPhone hotspot)
- [ ] R1 (Daniel's iPhone 17 Pro) and R2 (Felix's iPhone 16) open their URLs, accept the cert,
      allow the camera; HUD status line shows `R1 15HZ` and `R2 15HZ` (no `NODE Rn LOST`)
- [ ] One person in R1's view only: one cyan marker `R1-1` with a range (or a dashed ray if range
      is null)
- [ ] Same person in the overlap: **one** fused marker labelled `R1-1+R2-1` at the spot where
      they stand on the radar (triangulated from the pinned poses); step out of one cone and the
      marker stays
- [ ] Video: the R1 test card (red top left, green top right, blue bottom left, white bottom
      right, sweeping orange bar, frame counter) full screen on HDMI with the radar inset bottom
      right and `VID 15FPS` in the status line; if red and blue are swapped set
      `display.swap_rb: true`. `logs/hud_display.log` shows `video: first frame from ... R1 1280x720`
- [ ] Markers over video: the fused `R1-1+R2-1` brackets and label drawn on the picture, and the
      same contact on the radar inset
- [ ] `video pause`: dimmed picture with `VIDEO STALE` after 0.5 s, then `VIDEO LOST` and the full
      radar after about 2 s; `video resume`: video back
- [ ] `video stop`: `NO VIDEO` and the full radar; `video start`: video back within about 1 s
- [ ] Real phone video (only once Daniel's relay forwards it): R1's camera full screen instead of
      the test card, same fallbacks when R1 stops streaming
- [ ] Restart beat: session B runs `sh scripts/run_demo.sh chaos` (or the demo-ready
      `slay hud_fusion`): `FUSION LOST` for about 1 s over video that keeps playing, markers return
- [ ] Display restart (optional): `slay -9 hud_display`; the supervisor restarts it, the HUD comes
      back within about 2 s and the video sender reconnects on its own
- [ ] Phone silence: lock R2; `NODE R2 LOST` within 1.5 s; unlock, it recovers within 3 s
- [ ] `pidin` shows `hud_display` at `50f` during the whole run
- [ ] Ctrl+C in session A stops the supervisor, both children and the relay (`stopped cleanly`)
- [ ] 2-minute rehearsal per [demo-script.md](../../demo-script.md) (its kill beat now reads
      `slay hud_fusion` and `FUSION LOST`)

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| `prio inherit` and `spawn at priority ... refused` in the log | The system refused the priority. Check `pidin -p seeker_supervisor`; above 63 needs `PROCMGR_AID_PRIORITY` (e.g. `sudo`). |
| `exit status 127 right after a spawn at priority` | QNX reported a late scheduler failure; the supervisor retried at the inherited priority. Same fix as above. |
| `seeker_supervisor already running` | `sh scripts/run_demo.sh stop`, or remove a stale `logs/seeker_supervisor.pid`. |
| `cannot bind udp/47103` in `logs/hud_fusion.log`, fusion restarting with growing delays | Something else holds the port: another supervisor or a manual `hud_fusion`. `pidin -p hud_fusion`, `slay -f hud_fusion`, and run only one supervisor. |
| `slay` asks "slay hud_fusion (y/n)?" | More than one `hud_fusion` is running: two supervisors. Stop one. |
| Banner never visible | `RESTART_MS` too low (see above). |
| `--pose ...: want three numbers` or `--poses: ...` and exit 2 | Fix `config/poses` or `POSE`; `sh scripts/run_demo.sh preflight` shows the parsed poses. |
| Fused marker in the wrong place, or no `R1-1+R2-1` | Poses do not match the phones: re-measure `config/poses`. `logs/hud_fusion.log` must say `(file, pinned)`; a `sends pose 0,0,0` line means a phone is unpinned. |
| `NO VIDEO` although `--video-sim` is on | `logs/video_send.log`: `connect failed` means nothing listens on `VIDEO_PORT` (check `logs/hud_display.log` for `video: listening on tcp/...`). Both relay and `video_send.py` sending: the newest connection wins (`replaced by`). |
| Phones cannot load the page | TLS files missing (`preflight`), or the phone is not on the `QNX` hotspot. On the hotspot the Pi may only be reachable over IPv6 link-local; see [PI-BUILD.md](PI-BUILD.md) section 3. |

## Host pre-filter results (Linux x86, not QNX)

Ran on the agent's host before the PR, with clang 18 (as `cc` on the Pi) and gcc 13:

- Builds with no warnings through CMake and through `scripts/build_pi.sh` (`-std=c11 -Wall -Wextra`).
- `ctest`: `core` (229 checks), `supervisor_unit` (63), `supervisor_restart` (57) all pass, under
  dash and mksh (a pdksh descendant, the closest to QNX `sh` available on the host). PR #13's
  `scripts/video_loopback_test.sh` and the existing `scripts/loopback_test.sh` pass too.
- `run_demo.sh start --bg --sim` with Daniel's real `ws_ingest.py` (TLS) and `tools/sim/remote.py`,
  then `chaos`: the HUD (ppm sink) showed `FUSION LOST` with a red `FUS` light for about 1 s, then
  live `R1-1` / `R2-1` contacts again; `stop` left no processes or pid files. Foreground
  `start --sim` plus a real Ctrl+C in a terminal stopped everything with exit 0.
- After rebasing on PR #12: the same run with `config/poses` plus `POSE="R2=2.2,3,210"` logged
  both pins in the supervisor and in `hud_fusion` (`file, pinned` / `cli, pinned`) on the first
  start and again after the chaos restart, and the HUD showed one fused `R1-1+R2-1` marker.
- After rebasing on PR #13: `start --bg --sim --video-sim`, then `video pause`, `resume`, `stop`,
  `start`, then `chaos`: wearer view with the test card at `VID 15FPS`, `VIDEO LOST` on pause,
  `NO VIDEO` on stop, video back on start, `FUSION LOST` over still-playing video during the
  chaos restart, then fused `R1-1+R2-1` brackets on the video. In the integration test, video
  came back by itself after a `SIGKILL` of `hud_display` (supervisor restart, sender reconnect).
- On Linux the priorities fall back to `inherit` (no root). The 50f / 30r / 45f path, `slay`,
  `pidin` and QNX `sh` have **not** run yet; that is Felix's check on the Pi.
