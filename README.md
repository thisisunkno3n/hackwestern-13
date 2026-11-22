# Seeker (Hack Western 13)

Seeker is a see-through-walls helmet HUD. Two phones act as the vision sensors: each runs person
detection in the browser and streams what it sees to a QNX 8.0 Raspberry Pi 5 on the helmet. The
Pi fuses both phones into one room map and draws it on the helmet display: a solid box for people
the helmet phone can see, a dashed `XRAY` box for people the other phone sees behind the wall, and
edge arrows for people behind you. A QNX supervisor keeps the pipeline running, restarting a
crashed stage while the HUD keeps drawing.

## Layout

| Path | What it is |
| --- | --- |
| `remote/` | Phone side: `phone.html` (the phone sensor page) and `ws_ingest.py` (the WebSocket to UDP ingest, also the fallback bridge). See `remote/README.md`. |
| `pi/` | QNX 8.0 Raspberry Pi 5 lane in C: fusion (`hud_fusion`), HUD (`hud_display`), supervisor (`seeker_supervisor`), the phone bridge (`scripts/seeker_bridge.py`), configs, scripts, and tests. |
| `INTERFACES.md` | Contracts between the pieces: message fields, ports, process names, priorities. |
| `docs/` | Architecture, QNX primer and setup notes, demo script, the all-phone pivot, Devpost outline. |
| `reference/` | Parts list. |
| `tools/`, `test_*.py`, `test_*.mjs` | Host tools and tests for the `remote/` side. |

## How the demo runs

On the Pi, `pi/scripts/run-live.sh` starts `hud_display`, `hud_fusion`, and the phone bridge on
HTTPS/WSS port 8080. The demo bridge is `pi/scripts/seeker_bridge.py`: it loads
`remote/ws_ingest.py` unchanged and adds video control, an immediate video link to the HUD, and
per-phone status lines. `BRIDGE=daniel sh scripts/run-live.sh` runs `remote/ws_ingest.py` by itself
as the fallback.

`run-live.sh` expects the Pi tree at `~/seeker/drafts/pi-lane` and the ingest at
`~/seeker/remote/ws_ingest.py` (override with `SEEKER_DIR` and `WS_INGEST`).

## Build and run

- Building on the Pi (and on a Linux or macOS host for tests): [pi/PI-BUILD.md](pi/PI-BUILD.md).
- The live run, phone links, banners, logs, and tuning knobs: [pi/docs/LIVE-RUN.md](pi/docs/LIVE-RUN.md).
- The EagleEye wearer view: [pi/docs/EAGLEEYE.md](pi/docs/EAGLEEYE.md).

The `pi/` docs come from the private context repo and still describe the tree as `drafts/pi-lane`;
in this repo that tree is `pi/`. To package it from this repo on a Mac:

```bash
COPYFILE_DISABLE=1 tar --exclude build --exclude logs -s '|^pi|pi-lane|' -czf /tmp/pi-lane.tgz pi
scp /tmp/pi-lane.tgz qnxuser@qnxpi-felix.local:/tmp/
```

The archive unpacks as `pi-lane/`; on the Pi, unpack it under `~/seeker/drafts/` and follow
`pi/PI-BUILD.md` from there.

Host check (a pre-filter only; the Pi run is what counts):

```bash
cd pi && cmake -S . -B build && cmake --build build -j4 && ctest --test-dir build --output-on-failure
```
