# Firewatch (Hack Western 13)

Firewatch is a heads-up display for firefighters and search and rescue that shows the people you
cannot see. Phones act as the cameras: each one runs person detection in the browser and streams
what it sees to a QNX 8.0 Raspberry Pi 5. The Pi fuses every phone into one map of the room and
draws it over the wearer's own camera:

- a solid cyan frame for people the wearer's phone can see;
- a green ghost skeleton for people behind a wall that another phone can see ("Behind wall, 4.7 m,
  seen by R2");
- magenta, pulsing alerts for someone close, coming closer, or down (standing, crouching and down
  are read from the skeleton);
- a floor plan inset with the wearer, the other camera, walls and everyone found.

"Through walls" means through another camera's view, placed correctly in yours. A QNX supervisor
(`seeker_supervisor`, started by `pi/scripts/run_demo.sh`) can restart a crashed stage while the HUD
keeps drawing; the live script `pi/scripts/run-live.sh` starts the stages directly.

The code still uses the earlier project names (`seeker_*`, `EagleEye`); they are the same system.

## How it works

```
 phone R1 (wearer)            phone R2 (other side of the wall)
 Safari + MediaPipe Pose      Safari + MediaPipe Pose
 bearing, range, skeleton     bearing, range, skeleton
 + JPEG video                         |
        \                             /
         \  WSS :8080 (HTTPS page)   /
          v                         v
      Raspberry Pi 5, QNX 8.0
      seeker_bridge.py / ws_ingest.py  -- UDP 47103 -->  hud_fusion  -- tracks -->  hud_display --> HDMI
                         \-- TCP 47106 (video from R1) -----------------------------^
```

- **Phone** (`remote/phone.html`): MediaPipe PoseLandmarker in the browser. Pinhole bearing from the
  calibrated field of view, range from body height (one-tap 2 m calibration), a COCO-17 skeleton
  packed as integers, optional ArUco tag pose, JPEG video over a binary WebSocket.
- **Bridge** (`remote/ws_ingest.py`, wrapped by `pi/scripts/seeker_bridge.py`): WebSocket to UDP
  relay; detections go to fusion as `remote_contacts`, video goes to the display.
- **Fusion** (`pi/fusion`, C): places detections in a shared room frame from each phone's pose,
  tracks people, smooths every joint (One Euro), classifies posture, hands people over between
  phones.
- **Display** (`pi/display`, C, QNX Screen): projects tracks into the wearer's camera, decides seen,
  behind a wall or out of view from the wall geometry, and draws the HUD. Two looks: `plan` (the
  default drafting style) and `classic`.

Message fields, ports and process names are defined in [INTERFACES.md](INTERFACES.md).

## Layout

| Path | What it is |
| --- | --- |
| `remote/` | Phone side: `phone.html` (the phone sensor page) and `ws_ingest.py` (the WebSocket to UDP ingest, also the fallback bridge). See `remote/README.md`. |
| `pi/` | QNX 8.0 Raspberry Pi 5 side in C: fusion (`hud_fusion`), HUD (`hud_display`), supervisor (`seeker_supervisor`), the phone bridge (`scripts/seeker_bridge.py`), configs, scripts and tests. |
| `INTERFACES.md` | Contracts between the pieces: message fields, ports, process names, priorities. |
| `docs/` | Architecture, QNX primer and setup notes, demo script, the all-phone pivot. |
| `reference/` | Parts list. |
| `tools/`, `test_*.py`, `test_*.mjs` | Host tools and tests for the `remote/` side. |

## Demo setup

The wearer phone (R1) and the second phone (R2) stand still at measured spots, with a wall between
R1 and the area R2 watches. Everything is in one room frame: metres, `+y` the way R1 faces the
wall, yaw clockwise from `+y`. For R1 two metres in front of the wall and R2 two metres behind it,
facing back:

```sh
cd ~/seeker/drafts/pi-lane && \
  WEARER_MODE=fixed WEARER_POSE="0,-2,0" R2_POSE="0,2,180" WALLS="-2,0,2,0" sh scripts/run-live.sh
```

Then open the phone links it prints (`?node=R1&video=1` and `?node=R2`) in Safari on each phone,
on the Pi's hotspot, and accept the certificate once. `THEME=classic` switches the HUD back to the
original look. Every option, banner and log line is in [pi/docs/LIVE-RUN.md](pi/docs/LIVE-RUN.md).

## How the demo runs

On the Pi, `pi/scripts/run-live.sh` starts `hud_display`, `hud_fusion` and the phone bridge on
HTTPS/WSS port 8080. The demo bridge is `pi/scripts/seeker_bridge.py`: it loads
`remote/ws_ingest.py` unchanged and adds video control, an immediate video link to the HUD, and
per-phone status lines. `BRIDGE=daniel sh scripts/run-live.sh` runs `remote/ws_ingest.py` by itself
as the fallback.

`run-live.sh` expects the Pi tree at `~/seeker/drafts/pi-lane` and the ingest at
`~/seeker/remote/ws_ingest.py` (override with `SEEKER_DIR` and `WS_INGEST`). The Pi serves its own
copy of the phone page from `~/seeker/remote/phone.html`, so copy `remote/phone.html` there after
changing it.

## Build and run

- Building on the Pi (and on a Linux or macOS host for tests): [pi/PI-BUILD.md](pi/PI-BUILD.md).
- The live run, phone links, banners, logs and tuning knobs: [pi/docs/LIVE-RUN.md](pi/docs/LIVE-RUN.md).
- The wearer view: [pi/docs/EAGLEEYE.md](pi/docs/EAGLEEYE.md).

The `pi/` docs come from the private context repo and still describe the tree as `drafts/pi-lane`;
in this repo that tree is `pi/`. To package it from this repo on a Mac:

```bash
COPYFILE_DISABLE=1 tar --exclude build --exclude logs -s '|^pi|pi-lane|' -czf /tmp/pi-lane.tgz pi
scp /tmp/pi-lane.tgz qnxuser@qnxpi-felix.local:/tmp/
```

The archive unpacks as `pi-lane/`; on the Pi, unpack it under `~/seeker/drafts/` and follow
`pi/PI-BUILD.md` from there.

Host checks (a pre-filter only; the Pi run is what counts):

```bash
cd pi && cmake -S . -B build && cmake --build build -j4 && ctest --test-dir build --output-on-failure
node test_skeleton.mjs && node test_tagpose.mjs
python3 test_skeleton_pts.py && python3 test_contacts_cap.py && python3 test_video_forward.py
```

## Team

- **Daniel:** phone sensor page, WebSocket ingest, skeleton pipeline, HUD and brand design.
- **Felix:** QNX side: fusion, the HUD renderer, supervisor and the phone bridge.

Built in a weekend with a written interface contract between the two lanes, a shared notes repo,
and AI coding agents (Claude Code, Cursor) working in parallel branches. Nothing counted as done
until it ran end to end on the Pi with real phones.
