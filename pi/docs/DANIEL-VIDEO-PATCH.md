# Proposed patch for Daniel: live video from the helmet phone (Sat Nov 21, from Felix's lane)

For Felix to hand to Daniel. **Daniel's repo `thisisunkno3n/hackwestern-13` is his lane: we did
not push, open a PR or comment there.** Until Daniel takes this, `scripts/run-live.sh` gets the
same result on the Pi by running his unchanged `ws_ingest.py` through `scripts/seeker_bridge.py`.

## What went wrong at 3:11 PM ET

Live run on `qnxpi-felix.local`: the HUD stayed on the radar and the bridge never printed
`video: connected`. Root cause, reproduced on the host with headless Chrome and a fake camera
running his exact `a74c626` files:

1. **The R1 page never sent a video frame.** `VideoForwarder` only opens TCP 47106 when the
   first frame arrives, and logs `video: dest ... not up, retrying` when `hud_display` is not
   listening. Neither line appeared, so no binary frame reached the bridge. `hud_display` was
   listening (its flags parse and it always starts the 47106 listener).
2. **The R1 page ran without `&video=1`.** The bridge logged `node R1 connected pose=None`. The
   intended helmet URL (`...&pose=0,0,0&heading=1&video=1`) makes the bridge log
   `pose={'x_m': 0, 'y_m': 0, 'yaw_deg': None}`. So the page that ran had none of the `&...`
   part, only `?node=R1` (Safari autocomplete of an older URL, or retyped). R2 also logged
   `pose=None`, so its `&pose=0,2,180` was dropped too (harmless: fusion pinned R2 with
   `--pose`). A bare `https://localhost:8443/?node=R1` reproduces the exact Pi log.
3. **Nothing could turn video on or say why.** The phone does handle `video_ctrl`, but the
   bridge never sends it. The bridge connects to the HUD only after a frame, so the HUD cannot
   tell "no bridge" from "phone not sending".

With `&video=1` typed in full, his unchanged code works end to end into our `hud_display`
(`video: connected`, `first frame ... R1 640x480`, `frame drawn on the HUD`).

A fourth issue showed up in the same test: with `&heading=1`, if the iPhone motion prompt is
denied, `startHeading()` throws and the **whole page stops** (no camera, no detections, no
video).

## The patch

Two unified diffs. Tested to apply cleanly with `patch -p1` from the repo root on **`main`
`a74c626`** and on **`daniel/phone-smoothing` `b806419`**. Tested end to end in headless Chrome
on both, through his bridge into `hud_display`:

- a bare `?node=R1` URL now streams video;
- a denied compass keeps the page running;
- `b806419`'s contact changes (`range_src`, `R1-p<n>` ids, 16-contact cap) are unaffected.

**`remote/ws_ingest.py`**:

- `ws_send_text()`; on the view node's first message, send
  `{"type": "video_ctrl", "v": 2, "stream_video": true, "fps": 15}`.
- `VideoForwarder._run`: connect at start (not on the first frame), retry every second with one
  log line, notice when `hud_display` closes or restarts, and reconnect.
- Log `video: first frame from Rn` per connection (and say when a node other than
  `--view-node` sends video, which is dropped). Warn once if the view node has sent no video 10 s
  after connecting.

**`remote/phone.html`**:

- A denied compass shows `HDG DENIED` and carries on (yaw stays `null`), instead of aborting.
- A `VIDEO ON 15 FPS` / `VIDEO OFF` pill in the header.
- The start screen shows what the page parsed (`node R1  pose 0,0,0  heading on  video on`), so a
  URL that lost its `&...` part is obvious before tapping Start.

Apply (from `~/hackwestern-13`, after saving the two blocks below as `ws_ingest.diff` and
`phone.diff`):

```bash
patch -p1 --dry-run < ws_ingest.diff && patch -p1 < ws_ingest.diff
patch -p1 --dry-run < phone.diff && patch -p1 < phone.diff
python3 -m py_compile remote/ws_ingest.py
```

### `ws_ingest.diff`

```diff
--- a/remote/ws_ingest.py
+++ b/remote/ws_ingest.py
@@ -15,7 +15,7 @@
 Bridge -> hud_fusion (UDP :47103): IF-5.4 remote_contacts, t_mono_ms stamped on receive
 (cross-machine clock rule: the Pi ages by its own receive time).
 """
-import socket, threading, json, hashlib, base64, struct, os, time, argparse, sys, ssl
+import socket, threading, json, hashlib, base64, struct, os, time, argparse, sys, ssl, select
 
 WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
 CTYPES = {".html": "text/html", ".js": "application/javascript", ".css": "text/css"}
@@ -59,6 +59,14 @@
     return fin, op, data
 
 
+def ws_send_text(c, text):
+    """One unmasked server -> phone text frame (e.g. video_ctrl)."""
+    data = text.encode()
+    n = len(data)
+    head = bytes([0x81, n]) if n < 126 else bytes([0x81, 126]) + struct.pack(">H", n)
+    c.sendall(head + data)
+
+
 def ws_handshake(c, key):
     acc = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
     c.sendall(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
@@ -124,28 +132,36 @@
         return VHEADER.pack(b"SKV2", 2, 0, 0, node_b, ts_ms, w, h, seq, len(payload)) + payload
 
     def _run(self):
-        sock = None
+        # Connect at start (not on the first frame), so hud_display can tell "bridge up, phone
+        # not sending" from "no bridge", and reconnect when hud_display restarts.
+        sock, down_logged = None, False
         while not self.stop:
-            with self.cond:
-                while self.pending is None and not self.stop:
-                    self.cond.wait()
-                if self.stop:
-                    break
-                node, payload, width, height = self.pending
-                self.pending = None
             if sock is None:
                 try:
                     sock = socket.create_connection(self.dst, timeout=2)
                     sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                     LOG("video: connected to %s:%d" % self.dst)
+                    down_logged = False
                 except OSError as e:
                     sock = None
-                    LOG("video: dest %s:%d not up (%s), retrying" % (self.dst + (e,)))
-                    time.sleep(1)           # drop this frame; a fresher one will come
+                    if not down_logged:
+                        LOG("video: dest %s:%d not up (%s), retrying every 1 s" % (self.dst + (e,)))
+                        down_logged = True
+                    with self.cond:
+                        self.cond.wait(1.0)
+                        self.pending = None     # never queue old frames behind a dead link
                     continue
+            with self.cond:
+                if self.pending is None and not self.stop:
+                    self.cond.wait(0.5)
+                item, self.pending = self.pending, None
             try:
-                frame = self._frame(node, payload, width, height)
-                sock.sendall(frame)         # 2s timeout from create_connection guards a stall
+                if item is None:
+                    # hud_display never sends: readable means it closed the connection
+                    if select.select([sock], [], [], 0)[0] and sock.recv(1) == b"":
+                        raise ConnectionError("hud_display closed the connection")
+                    continue
+                sock.sendall(self._frame(*item))   # 2s timeout from create_connection
                 self.sent += 1
                 if self.sent % 30 == 0:
                     LOG("video: forwarded %d frames (%d dropped)" % (self.sent, self.dropped))
@@ -155,8 +171,7 @@
                 except Exception:
                     pass
                 sock = None
-                LOG("video: send failed (%s), reconnecting" % e)
-                time.sleep(1)
+                LOG("video: link lost (%s), reconnecting" % e)
 
 
 class Bridge:
@@ -186,6 +201,7 @@
         width = height = hfov_deg = None
         n = 0
         vbuf = None                          # accumulates a fragmented binary (video) message
+        vframes, t_conn, vwarned = 0, time.monotonic(), False
         while True:
             fin, op, data = ws_recv(c)
             if op == 0x8:
@@ -203,6 +219,10 @@
                 payload = bytes(vbuf)
                 vbuf = None
                 # forward video only from the wearer/view node; drop binary from other nodes
+                vframes += 1
+                if vframes == 1:
+                    LOG("video: first frame from %s%s" % (node_id, "" if node_id == self.view_node
+                        else " (not --view-node %s: dropped)" % self.view_node))
                 if self.video is not None and node_id == self.view_node:
                     self.video.submit(node_id, payload, width, height)
                 continue
@@ -213,7 +233,21 @@
             except Exception:
                 continue
             if m.get("node_id"):
+                if (m["node_id"] != node_id and m["node_id"] == self.view_node
+                        and self.video is not None):
+                    # the view node streams video even if its URL lost &video=1
+                    try:
+                        ws_send_text(c, json.dumps({"type": "video_ctrl", "v": 2,
+                                                    "stream_video": True, "fps": 15}))
+                        LOG("video: asked %s to stream video (video_ctrl)" % m["node_id"])
+                    except OSError:
+                        pass
                 node_id = m["node_id"]
+            if (node_id == self.view_node and not vframes and not vwarned
+                    and time.monotonic() - t_conn > 10):
+                vwarned = True
+                LOG("video: WARNING %s connected 10 s ago but sends no video (reload it with &video=1)"
+                    % node_id)
             if isinstance(m.get("pose"), dict):
                 pose = m["pose"]            # explicit pose:null leaves the last known value
             if m.get("width") is not None:
```

### `phone.diff`

```diff
--- a/remote/phone.html
+++ b/remote/phone.html
@@ -34,2 +34,3 @@
     <span class="pill" id="hdg" style="display:none">HDG --</span>
+    <span class="pill" id="vpill">VIDEO OFF</span>
     <button class="pill" id="zeroBtn" style="display:none;border:0;color:#07090c;background:#ffd23d;font-weight:600">Zero heading</button>
@@ -39,3 +40,6 @@
     <canvas id="ov"></canvas>
-    <div class="start" id="startWrap"><button id="startBtn">Start sensor</button></div>
+    <div class="start" id="startWrap" style="flex-direction:column;gap:12px">
+      <button id="startBtn">Start sensor</button>
+      <div id="params" style="font-size:13px;opacity:.8;text-align:center"></div>
+    </div>
   </div>
@@ -112,3 +116,6 @@
     const r = await DeviceOrientationEvent.requestPermission();
-    if (r !== "granted") throw new Error("compass permission denied");
+    if (r !== "granted") {   // keep the camera, detections and video going without a heading
+      el("hdg").style.display = "inline-block"; el("hdg").textContent = "HDG DENIED";
+      return;
+    }
   }
@@ -144,3 +151,10 @@
 }
-function startVideoTimer() { if (videoOn && !vtimer) vtimer = setInterval(sendVideo, Math.max(33, 1000 / VFPS)); }
+function startVideoTimer() {
+  if (videoOn && !vtimer) vtimer = setInterval(sendVideo, Math.max(33, 1000 / VFPS));
+  el("vpill").textContent = videoOn ? "VIDEO ON " + VFPS + " FPS" : "VIDEO OFF";
+  el("vpill").classList.toggle("on", videoOn);
+}
+// what this page will do, so a URL that lost its &... part (Safari autocomplete) is obvious
+el("params").textContent = "node " + NODE_ID + "  pose " + (POSE ? [POSE.x_m, POSE.y_m, POSE.yaw_deg].join(",") : "none") +
+  "  heading " + (HEADING ? "on" : "off") + "  video " + (videoOn ? "on" : "off (the Pi can turn it on)");
 
```

## Check on the Pi after applying

1. `hud_display` running, then `python3 ~/seeker/remote/ws_ingest.py --port 8080 --tls-cert
   ~/cert.pem --tls-key ~/key.pem` logs `video: connected to 127.0.0.1:47106` **before any
   phone connects**. The HUD banner changes from `NO VIDEO: BRIDGE NOT CONNECTED` to
   `NO VIDEO: PHONE NOT SENDING`.
2. Open `https://qnxpi-felix.local:8080/?node=R1` (no `&video=1` on purpose). The bridge logs
   `video: asked R1 to stream video (video_ctrl)` and then `video: first frame from R1`, and the
   phone pill reads `VIDEO ON 15 FPS`. `hud_display` logs `video: ok, R1 picture on the HUD`.
3. Open R1 with `&heading=1` and tap "Don't Allow" on the motion prompt: the page still starts
   and shows `HDG DENIED`.
4. Restart `hud_display`: the bridge logs `video: link lost (...)`, then `video: connected`
   within about a second, and video returns without reloading the phone.
