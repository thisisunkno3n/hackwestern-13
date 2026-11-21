# remote/ — WS6 remote phone-camera nodes (the twist)

Phones are **remote cameras**; a Mac is a **processing relay** (not a camera). Each relay reads one
phone's IP-camera stream with OpenCV, detects people, and sends **only detections** as
`remote_contacts` (IF-5.4) over **UDP :47103** to the helmet Pi's `hud_fusion`
(`qnxpi-felix.local`). The Pi marks people the wearer can't see (behind a wall / in smoke) in cyan.

One `node_id` per phone stream (`R1`, `R2`, …). N nodes supported; `R1` is the minimum demo.

## Files
- `hud_remote.py` — the relay. Backends (`--backend`): `stub` (no deps), `hog` (opencv-python),
  `yolo` (ultralytics + opencv), `mediapipe` (mediapipe + opencv).
- `../tools/sim/remote.py` — no-camera simulator; replays scripted contacts to fusion.
- `../tools/recv.py` — UDP printer for the quick check.

## Run
```bash
# no hardware needed — synthetic walking person:
python3 remote/hud_remote.py --source stub --pi qnxpi-felix.local --node R1

# real phone IP-cam (install first: pip install opencv-python ultralytics):
python3 remote/hud_remote.py --node R1 --stream http://<phone-ip>:<port>/video \
        --backend yolo --pose 0,0,0 --fov 60

# second node:
python3 remote/hud_remote.py --node R2 --stream http://<phone2>:<port>/video --pose 3,0,180
```
`--pose x_m,y_m,yaw_deg` is the phone's **taped position/yaw in the room frame** (IF-1: origin at
the wearer's floor mark, +y = facing, yaw clockwise). `bearing_deg = atan((x−cx)/f)`, clockwise from
the node boresight; `f` from `--fov` (or `--focal` px); optional mono `range_m = f·1.7/box_h`.

## Quick check (matches ws6 brief)
```bash
# terminal A (or on the Pi):  python3 tools/recv.py 47103
# terminal B:                 python3 tools/sim/remote.py --pi 127.0.0.1 --node R1
# recv prints:  remote_contacts R1  1 contact  bearing -12.0  range 2.8 m
```
Stop the relay → fusion shows `NODE R1 LOST` within 1.5 s (Pi drops a node after 1500 ms of
silence, by receive time); restart → contacts back within 3 s.

## Status (2026-11-21)
- ✅ `remote_contacts` (IF-5.4) emitted; schema + `recv` format validated locally over UDP loopback.
- ✅ `stub` backend + `sim/remote.py` run with **zero dependencies** (good for fusion/HUD testing now).
- ⏳ Real detection backends need `pip install opencv-python` (+ `ultralytics` or `mediapipe`) on the relay Mac.
- ⏳ Pi-side (`qnxpi-felix.local:47103`) send pending: test when the relay Mac is on the `QNX` hotspot.
- `calib_f.py` (focal calibration) is a stretch; `--fov` is a good-enough default for the demo.
