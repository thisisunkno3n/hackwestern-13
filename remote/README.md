# Firewatch remote/ -- phones as the vision sensors (all-phone pivot)

Per `PIVOT-all-phone.md` (context repo): **no webcams**. Two phones run person detection in the
browser (MediaPipe) and stream detections into the RTOS. Browsers can't send UDP, so a small
**WS→UDP bridge** turns each phone's WebSocket into IF-5.4 `remote_contacts` to `hud_fusion`.

```
phone.html (getUserMedia -> MediaPipe pose)  --WS/WSS-->  ws_ingest.py  --UDP remote_contacts--> hud_fusion (:47103)
```

## Files
- `phone.html` -- the Firewatch phone sensor page. URL params: `?node=R1&pose=x,y,yaw&hfov=66`. Sends
  `{node_id, bearing_deg, conf, box, range_m}` at ~15 Hz, empty-contacts liveness when no one is seen.
  Styled to match the Firewatch HUD (drafting plates, one cut corner, cool colours only): a start
  screen with a "Start camera" button, the camera full screen, and a title block along the bottom
  (node, link, fps, bearing, conf, range, pose or tag state, video). A magenta banner appears when
  the Pi link has been down for 2.5 s; camera or permission problems show in plain words on the
  start screen. Fonts (Atkinson Hyperlegible Next and Mono) load from Google Fonts without blocking,
  with system fallbacks. Single file: copy it as is to the Pi (`~/seeker/remote/phone.html`):
  `scp remote/phone.html qnxuser@qnxpi-felix.local:~/seeker/remote/phone.html`, then reload the phones.

  Staying up unattended (the wall phone, R2):
  - The link reconnects by itself (0.5, 1, 2, 4, then every 5 s), gives up on a handshake that
    hangs for 6 s, and drops a socket whose send buffer stops draining for 5 s. Every new socket
    sends the role message first, with the last known pose if the live one is missing.
  - Screen Wake Lock (Safari 16.4+) keeps the screen on, and is taken again after the page comes
    back, on any touch, and every second. Back from a screen lock of 3 s or more, the page opens a
    fresh socket and restarts the camera if iOS stopped it. A magenta chip at the top says
    `Screen may sleep`, `Camera restarting` or `Detector restarting` when one of those is wrong.
    Still set Auto-Lock to Never on the wall phone as a backup.
  - A reload in the same tab starts the camera again without a tap, and a link re-opened as just
    `?node=R2` gets its missing keys (`&pose=` and the rest) back from that tab's last load of the
    same node. `?fresh=1` skips the restore, `?autostart=0` keeps the start screen.
  - Video goes out only while less than 64 KB is still unsent (frames are dropped, never queued; the
    Video cell reads `Busy`), and detections are skipped only past 256 KB.
  - R2 does not stream video even with `&video=1`, since it is not the view node and the bridge drops
    its frames. It streams when the relay sends `video_ctrl` with `stream_video: true`, or with
    `?video=always` (plain `ws_ingest.py --view-node R2` sends no `video_ctrl`).
- `ws_ingest.py` -- WS/WSS→UDP bridge; also serves `phone.html`. `--fusion <pi> --fusion-port 47103`.
- `hud_remote.py` -- *alternative* Mac-relay path (OpenCV on a phone IP-cam). Kept for the Cubie
  fallback / non-browser phones; the in-browser path above is primary post-pivot.

## Run (the bridge is on the Pi, or the Mac for testing)
```bash
# plain (desktop browser / loopback testing):
python3 remote/ws_ingest.py --fusion 127.0.0.1 --port 8080
# iPhone needs HTTPS for camera access -- self-signed is fine:
python3 remote/ws_ingest.py --fusion 127.0.0.1 --port 8080 --tls-cert cert.pem --tls-key key.pem
# phone opens https://<host>:8080/?node=R1   (accept the cert warning once)
```
Quick check: `python3 tools/recv.py 47103` prints `remote_contacts R1  1 contact  bearing -12.0  range 2.8 m`.

## Status
- ✅ phone(WS) → bridge → `remote_contacts` UDP validated locally over loopback.
- ✅ TLS/WSS option for iPhone camera access.
- ⏳ In-browser test on a real iPhone; end-to-end vs Felix's `hud_fusion` on the `QNX` hotspot.
