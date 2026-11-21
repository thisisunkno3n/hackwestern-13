# remote/ — phones as the vision sensors (all-phone pivot)

Per `PIVOT-all-phone.md` (context repo): **no webcams**. Two phones run person detection in the
browser (MediaPipe) and stream detections into the RTOS. Browsers can't send UDP, so a small
**WS→UDP bridge** turns each phone's WebSocket into IF-5.4 `remote_contacts` to `hud_fusion`.

```
phone.html (getUserMedia -> MediaPipe pose)  --WS/WSS-->  ws_ingest.py  --UDP remote_contacts--> hud_fusion (:47103)
```

## Files
- `phone.html` — the phone sensor page. URL params: `?node=R1&pose=x,y,yaw&hfov=66`. Sends
  `{node_id, bearing_deg, conf, box, range_m}` at ~15 Hz, empty-contacts liveness when no one is seen.
- `ws_ingest.py` — WS/WSS→UDP bridge; also serves `phone.html`. `--fusion <pi> --fusion-port 47103`.
- `hud_remote.py` — *alternative* Mac-relay path (OpenCV on a phone IP-cam). Kept for the Cubie
  fallback / non-browser phones; the in-browser path above is primary post-pivot.

## Run (the bridge is on the Pi, or the Mac for testing)
```bash
# plain (desktop browser / loopback testing):
python3 remote/ws_ingest.py --fusion 127.0.0.1 --port 8080
# iPhone needs HTTPS for camera access — self-signed is fine:
python3 remote/ws_ingest.py --fusion 127.0.0.1 --port 8443 --tls-cert cert.pem --tls-key key.pem
# phone opens https://<host>:8443/?node=R1   (accept the cert warning once)
```
Quick check: `python3 tools/recv.py 47103` prints `remote_contacts R1  1 contact  bearing -12.0  range 2.8 m`.

## Status
- ✅ phone(WS) → bridge → `remote_contacts` UDP validated locally over loopback.
- ✅ TLS/WSS option for iPhone camera access.
- ⏳ In-browser test on a real iPhone; end-to-end vs Felix's `hud_fusion` on the `QNX` hotspot.
