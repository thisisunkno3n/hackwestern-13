# Seeker HUD Interfaces v2 (Single Source of Truth)

Every workstream, lane doc, and code module links here and never redefines a contract.
All times are America/Toronto (ET). Names: processes `hud_*`, C symbols `seeker_*`.

## Interface Governance Rules (Approved Sat Nov 21, 2026)

1. **Contract first, code second.** Any change to a message, port, or field goes into
   `INTERFACES.md` before anyone codes against it.
2. **Fast merge with high-detail standard.** Anyone (Felix, Daniel, or their agents) can merge
   their own `INTERFACES.md` PR immediately without waiting for approval (speed first). Every
   change must be high quality and detailed enough for the other person's agent to pick up cold:
   state **what changed**, **why**, a **before/after example message**, **which code must follow**,
   and **how it was checked on QNX**.
3. **Every message carries a version field `"v"`.** Bump `"v"` when any field changes meaning.
   The Pi (`hud_fusion` and `hud_display`) warns loudly in stderr/logs on any unknown version
   instead of silently drawing junk.
4. **Units are in the field name:** `bearing_deg`, `range_m`, `ts_ms` (plus `t_mono_ms`, `x_m`,
   `y_m`, `yaw_deg`, `hfov_deg`, `closing_mps`, `age_ms`). Never guess between degrees and
   radians or seconds and milliseconds.
5. **One way to say "missing": `null`.** Never send `0`, `[0,0,0,0]`, or `0,0,0` as a sentinel
   for missing geometry, range, or pose.
6. **Add-only during the hackathon.** Never rename or remove existing fields mid-hackathon. Mark
   superseded fields as deprecated and keep them until after the demo.
7. **Unknown fields are ignored, not rejected.** Either lane can add fields without breaking the
   other lane's parser.
8. **Every message gets one real example** in this file, plus a one-line header naming sender,
   receiver, port/transport, and rate.
9. **One owner per message.** Phones (`phone.html`) own raw detections; the relay
   (`ws_ingest.py` / `hud_remote.py`) owns forwarded `remote_contacts` and `video_frame`; the Pi
   (`hud_fusion` / `hud_display`) owns `tracks` and display. The owner drafts any change first.
10. **Not done until tested on the real QNX Pi built with `qcc`.** Linux/laptop host tests are
    only a pre-filter. A contract PR must state how its consumer/producer was verified on
    `qnxpi-felix.local`.

---

## IF-1 Conventions and Runtime Topology (Post-Pivot v2)

| Topic | Rule |
| --- | --- |
| Architecture (all-phone pivot) | No helmet webcam (`hud_capture` and `hud_infer` retired). Two phones (`R1` = Daniel's iPhone 17 Pro, `R2` = Felix's iPhone 16) run MediaPipe in Safari (`remote/phone.html`) and stream detections (and video from the wearer-selected phone) over WSS to `remote/ws_ingest.py` **running on the Pi (`qnxpi-felix.local`, Python 3.14.0)**. `ws_ingest.py` forwards contacts and video over local loopback to `hud_fusion` and `hud_display` (C11, QNX Screen on `HDMI0`, 27-inch 4K monitor). `remote/hud_remote.py` on a Mac remains the backup relay. |
| Languages | Pi native stack (`hud_fusion`, `hud_display`, `hud_supervisor`, `hud_stress`, `common/`): **plain C11** built with **`qcc`** on QNX 8.0 (`SEEKER_TARGET=qnx`, host `laptop` build is pre-filter only). Relay (`ws_ingest.py`, `hud_remote.py`) and sims/tools: **Python 3** (Pi has Python 3.14.0). Phone client: HTML/JS ES module (`remote/phone.html`). |
| Bearing math (canonical) | **`atan((x - cx) / f)` everywhere** (`phone.html`, `ws_ingest.py`, `hud_remote.py`, `tools/sim/remote.py`), where `f = (width / 2.0) / tan(radians(hfov_deg) / 2.0)` and `x` is the person's horizontal pixel center. Positive `bearing_deg` is clockwise / right of camera boresight. Linear `(cx - 0.5) * hfov_deg` is deprecated and must be replaced. |
| Per-node FOV calibration | Each phone has its own `hfov_deg` in `config/seeker.json` (`R1` iPhone 17 Pro, `R2` iPhone 16), calibrated once by standing a person at a measured angle (e.g. 30 deg off boresight) and adjusting `hfov_deg` until `bearing_deg` matches. |
| Image dimensions | Every detection and video message **must** carry `width` and `height` (`w`, `h`) in pixels so orientation flips or Safari resolution changes never misplace bounding boxes. |
| Person tracking & IDs | `hud_fusion` on the Pi tracks people across frames and nodes and assigns stable track IDs. Phone-side contact IDs are optional and ignored by `hud_fusion`. |
| Clock rule | **Never compare `ts_ms` or `t_mono_ms` across machines.** `ws_ingest.py` stamps Pi-local monotonic ms on receive (`t_mono_ms` / `ts_ms`), and `hud_fusion` / `hud_display` age every node and frame strictly by Pi receive time. |
| Image coords | Origin top-left, `+x` right, `+y` down. `box` is `[x, y, w, h]` in pixels of `[width, height]`, or `null` when absent. |
| Room frame | Origin at the wearer's floor mark (or room origin). `+y` is forward ("north"), `+x` is right, `yaw_deg` is degrees clockwise from `+y`. Phones start at fixed measured poses (`x_m`, `y_m`, `yaw_deg`) and iterate toward handheld live pose updates (`pose` in messages). |
| Missing values | Always `null` (JSON) or explicit presence flag `0` (C structs). Never `0`, `[0,0,0,0]`, or `0,0,0`. |
| Logging | Human logs to stderr prefixed `[process]`. Machine logs are JSONL (IF-9). |
| Config | Everything tunable lives in `config/seeker.json` (IF-8). |

---

## IF-2 Repository Layout and Ownership

Code repo: **`thisisunkno3n/hackwestern-13`** (public).
Context repo: **`thisisunkno3n/hackwestern-13-context`** (private docs/contracts only).

---

## IF-3 Processes and Priorities (on `qnxpi-felix.local`)

| Process | Role | QNX Priority, Policy | Rate | Must survive |
| --- | --- | --- | --- | --- |
| `hud_display` | Reads latest `tracks` and latest decoded `video_frame`, draws HUD + radar inset, presents on `HDMI0` via QNX Screen. **Never blocks on network or JPEG decode.** | **50, FIFO** (highest) | 30 fps target (25+ fps pass) | Everything else dying |
| `hud_supervisor` | Spawns Pi processes, sets priorities, watches `/seeker_health` beats, restarts crashed/hung children | 45, FIFO | 20 Hz check | |
| `hud_fusion` | Listens on UDP `47103`, tracks/fuses `R1`+`R2`, enforces 1500 ms `LOST`, emits `tracks` on `47102` | 30, RR | 30 Hz | |
| `ws_ingest.py` | Python 3.14.0 HTTP/HTTPS + WS/WSS server on `:8080`, forwards UDP `47103` + TCP `47106` | normal (other/10) | ~15 Hz | |
| `hud_stress` | Demo CPU load: N spinning threads | 10, RR | | |

*Deprecated post-pivot:* `hud_capture` (was 40, FIFO) and `hud_infer` (was 11, RR).

---

## IF-4 Frame Ring (Deprecated Post-Pivot)

`/seeker_frames` POSIX shared memory from `hud_capture` is **deprecated** after the all-phone
pivot. Instead, `hud_display` maintains an internal lock-free/double-buffered latest-frame slot
fed by its TCP `47106` receiver thread (newest frame wins, never blocks the priority-50 draw loop).

---

## IF-5 Messages and Transports

### Ports Table

| Port | Proto | Message | Sender | Receiver | Rate |
| --- | --- | --- | --- | --- | --- |
| `8080` | TCP (HTTPS/WSS or HTTP/WS) | Phone pages + IF-5.0 JSON/binary | Phones (`R1`, `R2`) | `ws_ingest.py` (on Pi) | ~15 Hz |
| `47103` | UDP (`127.0.0.1` or LAN) | IF-5.4 `remote_contacts` (JSON, <= 8 KB) | `ws_ingest.py` (or `hud_remote.py` backup) | `hud_fusion` | ~15 Hz |
| `47106` | TCP (`127.0.0.1`) | IF-5.6 `video_frame` (36 B header + JPEG) | `ws_ingest.py` (selected `view_node`) | `hud_display` | 15 fps target |
| `47102` | UDP (`127.0.0.1`) | IF-5.2 `tracks` (JSON, <= 8 KB) | `hud_fusion` | `hud_display` | 30 Hz |
| `47104` | UDP (optional stretch) | IF-5.5 `head_pose` (JSON) | iPhone HeadBridge | `hud_fusion` | >= 20 Hz |
| `47105` | UDP (`127.0.0.1`) | IF-5.3 `control` (JSON) | keyboard / CLI tool | `hud_display` / `hud_supervisor` | on command |
| `8090` | TCP HTTP (optional) | MJPEG mirror of HUD | `hud_display` | laptop viewer | 15 fps |
| `47101` | UDP (DEPRECATED) | IF-5.1 `detections` | `hud_infer` (retired) | `hud_fusion` | retired |

Note: ports `8099` and `8443` from early drafts are **dropped**. Use `8080` everywhere.

---

### IF-5.0 Phone -> Relay WebSocket (`ws://` or `wss://<pi>:8080/ws`)

- **Header:** Sender: `remote/phone.html` (Safari on `R1` / `R2`) | Receiver: `remote/ws_ingest.py` (on Pi) | Port: TCP `8080` (WSS `/ws`) | Rate: ~15 Hz when detecting, >= 5 Hz liveness when empty.
- **Owner:** Daniel.
- **Rules:**
  - `width` and `height` are required on **every** message.
  - `bearing_deg` uses `atan((x - cx) / f)` with calibrated `hfov_deg`.
  - Missing `box`, `range_m`, or unknown `pose` must be `null`.
  - Video frames from the wearer-selected `view_node` are sent as **binary WebSocket frames**
    using the exact IF-5.6 `SKV2` 36-byte header + JPEG bytes below (only the active `view_node`
    streams video; relay may send `{"type":"video_ctrl","v":2,"stream_video":true,"width":1280,"height":720,"fps":15}`).

**Example (JSON text frame, detection present):**
```json
{
  "type": "phone_detections",
  "v": 2,
  "node_id": "R1",
  "ts_ms": 1763740800123,
  "width": 1280,
  "height": 720,
  "hfov_deg": 68.5,
  "pose": { "x_m": -2.0, "y_m": 3.0, "yaw_deg": 145.0 },
  "contacts": [
    {
      "cls": "person",
      "conf": 0.91,
      "box": [520, 140, 180, 460],
      "bearing_deg": -2.6,
      "range_m": 3.9
    }
  ]
}
```

**Example (JSON text frame, no person detected, liveness heartbeat):**
```json
{
  "type": "phone_detections",
  "v": 2,
  "node_id": "R2",
  "ts_ms": 1763740800323,
  "width": 1280,
  "height": 720,
  "hfov_deg": 66.0,
  "pose": null,
  "contacts": []
}
```

---

### IF-5.4 `remote_contacts` (Relay -> Fusion)

- **Header:** Sender: `remote/ws_ingest.py` (on Pi) or `remote/hud_remote.py` (Mac backup) | Receiver: `hud_fusion` | Port: UDP `47103` (`127.0.0.1:47103` primary) | Rate: ~15 Hz (empty `contacts: []` >= 5 Hz for liveness). Max 8 KB per datagram.
- **Owner:** Daniel (relay producer), shared contract with Felix (`hud_fusion` consumer).
- **v2 changes from v1:**
  - `v` bumped to `2` (`hud_fusion` accepts `v: 1` during transition, warns loudly on any unknown `v`).
  - Added required `width` and `height` (int px) at top level (and optional `hfov_deg`).
  - `ts_ms` added alongside `t_mono_ms` (add-only Rule 6; both hold Pi receive monotonic ms).
  - `pose` is `{"x_m": float, "y_m": float, "yaw_deg": float}` or `null` (never `0,0,0` as a sentinel for unknown).
  - `contacts[].box` is `[x, y, w, h]` or `null` (never `[0,0,0,0]`).
  - `contacts[].range_m` is float metres or `null` (never `0`).
  - `contacts[].id` is optional and **ignored** by `hud_fusion` (Pi tracks people itself).
  - Node timeout: `hud_fusion` marks `node_id` as `lost` after **1500 ms** without a datagram, measured strictly by Pi receive time.

**Example:**
```json
{
  "type": "remote_contacts",
  "v": 2,
  "src": "R1",
  "ts_ms": 4829105,
  "t_mono_ms": 4829105,
  "node_id": "R1",
  "width": 1280,
  "height": 720,
  "hfov_deg": 68.5,
  "pose": { "x_m": -2.0, "y_m": 3.0, "yaw_deg": 145.0 },
  "contacts": [
    {
      "cls": "person",
      "conf": 0.91,
      "box": [520, 140, 180, 460],
      "bearing_deg": -2.6,
      "range_m": 3.9
    }
  ]
}
```

---

### IF-5.6 `video_frame` (NEW: Relay -> Display, Local TCP)

- **Header:** Sender: `remote/ws_ingest.py` (on Pi, forwarding only the wearer-selected `view_node`) | Receiver: `hud_display` | Port: TCP `127.0.0.1:47106` | Rate: 15 fps target at `1280x720` (fallback `960x540` if WiFi or QNX JPEG decode lags).
- **Owner:** Daniel (`ws_ingest.py` sender), shared contract with Felix (`hud_display` receiver/decoder).
- **Transport justification:**
  - **Why local TCP (`127.0.0.1:47106`) instead of chunked UDP:** A `1280x720` JPEG at quality 75 is ~60 to 180 KB, which exceeds the 64 KB single UDP datagram limit and would require custom chunking, sequence reassembly, and loss recovery in C11. Because `ws_ingest.py` runs **on the Pi**, `127.0.0.1` loopback has no WiFi packet loss and negligible latency. Length-prefixed TCP gives zero-reassembly framing with a minimal `recv()` loop in C (`hud_display`).
  - **Freshness guarantee (no lag buildup):** Enable `TCP_NODELAY` on both ends. In `ws_ingest.py`, keep a single-slot latest-frame mailbox per connection so slow sends drop older frames instead of queueing. In `hud_display`, a dedicated receiver thread reads length-prefixed frames into a 2-slot double buffer so the priority-50 draw loop always renders the latest decoded frame and never blocks on I/O.
  - **Fallback guarantee:** If no frame arrives for `> 500 ms` (`VIDEO LOST`) or JPEG decode fails, `hud_display` switches seamlessly to radar-only full screen without freezing.

**Framing (36-byte big-endian binary header + raw JPEG bytes):**

| Offset | Size | Type | Field | Description |
| --- | --- | --- | --- | --- |
| `0` | 4 | `char[4]` | `magic` | ASCII `"SKV2"` (`0x53 0x4B 0x56 0x32`) |
| `4` | 1 | `uint8` | `v` | Protocol version (`2`). Receiver warns loudly on unknown `v` and drops stream/resyncs. |
| `5` | 1 | `uint8` | `flags` | Bit flags (`0` = standard baseline JPEG). |
| `6` | 2 | `uint16` | `reserved` | `0` (reserved for add-only extensions). |
| `8` | 8 | `char[8]` | `node_id` | NUL-padded ASCII node ID, e.g. `"R1      "`. |
| `16` | 8 | `uint64` | `ts_ms` | Monotonic ms stamped on receive by `ws_ingest.py` on the Pi. |
| `24` | 2 | `uint16` | `width` | Frame width in pixels (target `1280`, fallback `960`). |
| `26` | 2 | `uint16` | `height` | Frame height in pixels (target `720`, fallback `540`). |
| `28` | 4 | `uint32` | `seq` | Monotonic frame sequence number per `node_id`. |
| `32` | 4 | `uint32` | `payload_len` | Byte length of the JPEG image immediately following (max `2097152`, 2 MB). |
| `36` | `payload_len` | `uint8[]` | `jpeg_bytes` | Raw JPEG payload (`FF D8 ... FF D9`). |

**Example (header + payload description):**
```text
Hex header (36 bytes):
  53 4B 56 32              magic = "SKV2"
  02 00 00 00              v = 2, flags = 0, reserved = 0
  52 31 00 00 00 00 00 00  node_id = "R1"
  00 00 00 00 00 49 B0 21  ts_ms = 4829217
  05 00 02 D0              width = 1280, height = 720
  00 00 01 A4              seq = 420
  00 01 4A 20              payload_len = 84512 bytes
Followed immediately by 84512 bytes of JPEG (FF D8 ... FF D9).
```

**Receiver behaviour (`hud_display`, add-only clarification Sat Nov 21 evening; wire format
unchanged, `v` stays `2`):**
- Listens on TCP `47106` on `0.0.0.0` and `[::]` (IPv6 socket with `IPV6_V6ONLY=1`, IPv4 only if
  the stack refuses IPv6), so the primary `127.0.0.1` sender works unchanged and the Mac backup
  relay can also send over LAN or IPv6 link-local.
- One sender at a time; a new connection replaces the old one.
- Frames are back to back with nothing in between. Bad magic, `v != 2`, or `payload_len` outside
  `1..2097152`: logged loudly, then the receiver scans forward for the next `SKV2`.
- Only the newest complete frame is decoded; older ones are dropped, never queued. Any JPEG
  (baseline or progressive, any chroma subsampling) decodes. `ts_ms` is logged, never compared.
- Implementation and test sender: `drafts/pi-lane/VIDEO.md`, `scripts/video_send.py`. Checked on
  the host only so far (Linux, clang and gcc); QNX verification on `qnxpi-felix.local` pending.

---

### IF-5.2 `tracks` (Fusion -> Display)

- **Header:** Sender: `hud_fusion` | Receiver: `hud_display` | Port: UDP `127.0.0.1:47102` | Rate: 30 Hz (coast between `remote_contacts` arrivals). Max 8 KB per datagram.
- **Owner:** Felix.
- **v2 changes from v1:**
  - `v` bumped to `2` (`ts_ms` added alongside `t_mono_ms`; `nodes[]` array and per-track room/node fields promoted from draft `PROPOSED` into v2; `node_w`, `node_h` added so `hud_display` scales `node_box` accurately onto 1280x720 / 4K).
  - Unknown ages/coordinates use `null` (never `-1` or `0,0,0` placeholders).

**Example:**
```json
{
  "type": "tracks",
  "v": 2,
  "src": "hud_fusion",
  "ts_ms": 4829250,
  "t_mono_ms": 4829250,
  "basis_frame_seq": 1448,
  "heading_deg": null,
  "heading_src": "none",
  "wearer_yaw_deg": 0.0,
  "nodes": [
    {
      "node_id": "R1",
      "state": "ok",
      "age_ms": 42,
      "x_m": -2.0,
      "y_m": 3.0,
      "yaw_deg": 145.0,
      "hfov_deg": 68.5,
      "n_contacts": 1,
      "rate_hz": 14.9
    },
    {
      "node_id": "R2",
      "state": "lost",
      "age_ms": 1820,
      "x_m": 2.0,
      "y_m": 3.0,
      "yaw_deg": 215.0,
      "hfov_deg": 66.0,
      "n_contacts": 0,
      "rate_hz": 0.0
    }
  ],
  "tracks": [
    {
      "id": 1,
      "label": "R1-1",
      "cls": "person",
      "src": "remote",
      "visible": false,
      "range_m": 3.9,
      "range_src": "remote",
      "range_conf": 0.85,
      "bearing_deg": -2.6,
      "elev_deg": 0.0,
      "closing_mps": 0.1,
      "colour": "amber",
      "age_ms": 42,
      "node_id": "R1",
      "nodes": "R1",
      "node_bearing_deg": -2.6,
      "room_x_m": 0.12,
      "room_y_m": 0.21,
      "node_box": [520, 140, 180, 460],
      "node_w": 1280,
      "node_h": 720
    }
  ]
}
```

---

### IF-5.3 `control` (Operator / CLI -> Display & Supervisor)

- **Header:** Sender: keyboard / CLI script | Receiver: `hud_display`, `hud_supervisor` | Port: UDP `127.0.0.1:47105` | Rate: on command.
- **Owner:** Felix.
- **Commands (`cmd`):** `background` (`"wearer"` | `"radar"`), `view_node` (`"R1"` | `"R2"`), `mode` (`"video"` | `"visor"`), `mirror` (`"none"` | `"h"` | `"v"`), `declutter`, `stress` (`"on"` | `"off"`), `chaos` (kill `hud_fusion`), `prio_demo` (`"high"` | `"low"`), `record` (`"start"` | `"stop"`).

**Example:**
```json
{
  "type": "control",
  "v": 2,
  "src": "cli",
  "ts_ms": 4829300,
  "cmd": "view_node",
  "val": "R2"
}
```

---

### IF-5.5 `head_pose` (Optional Stretch WS8: iPhone -> Fusion)

- **Header:** Sender: iPhone HeadBridge | Receiver: `hud_fusion` | Port: UDP `:47104` | Rate: >= 20 Hz, stale after 300 ms.
- **Owner:** Daniel (sender), Felix (`hud_fusion` consumer).

**Example:**
```json
{
  "type": "head_pose",
  "v": 2,
  "src": "iphone",
  "ts_ms": 1763740800500,
  "yaw_deg": 12.4,
  "pitch_deg": -2.1,
  "roll_deg": 0.5,
  "heading_deg": 245.0
}
```

---

### IF-5.1 `detections` (DEPRECATED)

Retired after the all-phone pivot (`hud_infer` removed). Port `47101` is unused.

---

## IF-6 Health (POSIX Shared Memory `/seeker_health`)

One slot per native Pi process (`hud_fusion`, `hud_display`): `name`, `pid`, `last_beat_ms`,
`loop_ms`, `state` (`starting`, `ok`, `degraded`, `dead`), `restarts`. Each beats at least every
100 ms. `hud_supervisor` declares a process dead after 500 ms without a beat or on exit, restarts
with 500 ms backoff, and increments `restarts`. `hud_display` renders live process lights (`FUS`,
`HUD`, plus node status `R1`, `R2`).

---

## IF-7 Symbology, HUD Mode, and Colour Rules

- **Primary HUD Mode (`display.background: "wearer"`):** Full-screen live video from
  `display.view_node` (target 1280x720 @ 15 fps scaled fullscreen to the 27-inch 4K monitor on
  `HDMI0`) with a **tactical radar inset** in the bottom-right corner.
- **Fallback HUD Mode (`display.background: "radar"`):** Full-screen tactical radar room map
  with nodes (`R1`, `R2`), view cones, and fused contacts. Activated automatically if video is
  absent/stale (`> 500 ms`) or via `--background radar` / `control`.
- **EagleEye wearer view (`hud_display --view wearer`, or a `display.wearer` block; add-only Sat
  Nov 21; plain `--background wearer` keeps the mode above):** the wearer node
  (`display.wearer.node`, default `W`, a phone taped to the helmet; it is also `view_node`) is the
  camera. Each fused track with `room_x_m`/`room_y_m` is projected into that phone's image with the
  IF-1 pinhole (`f = (image_w / 2) / tan(hfov / 2)`) using the wearer pose (`hud_fusion`'s pose
  for the node when heard, else static). Track `nodes` containing the wearer id = **seen** (solid
  box, range colour); otherwise **x-ray** (dashed cyan box, `XRAY`, `VIA <nodes>`); outside the FOV
  = edge chevron with range and turn angle (`BEHIND` past 90 deg). Heading-up radar inset bottom
  right. Over the wearer node's video when decoded, else black (visor). No wire format changes.
  Details: `drafts/pi-lane/docs/EAGLEEYE.md`.

| Element | Spec |
| --- | --- |
| Target bracket | Corner-only brackets around `node_box` (scaled from `node_w x node_h` to display size) for contacts on `view_node`; colour from `colour`, label `R1-1 3.9m`. |
| Bearing ray (no range) | Dashed bearing ray on the radar map (and vertical tick on video) when `range_m` is `null`. |
| Other-phone / off-screen contact | Cyan diamond on horizon or edge chevron at relative bearing, plus always plotted on the radar inset. |
| Radar inset | Bottom-right, top-down room map (`radar_m` radius), nodes as squares with FOV cones, fused contacts as dots/diamonds (`R1+R2` when merged). |
| Heading tape | Top centre, 60 deg wide, `REL` mode when `heading_deg` is `null`. |
| Banners | `NODE R1 LOST` / `NODE R2 LOST` (after 1500 ms silence); `FUSION LOST` (when `hud_fusion` dies); `VIDEO LOST` (when `video_frame` silent > 500 ms, falling back to radar). |
| Status line | Bottom: `VID 15fps  FUS 30Hz  HUD 30fps  PRIO 50  R1 OK  R2 LOST` plus process lights. |

Colours: **red** within `near_m` (2.0 m) or closing faster than `closing_mps` (0.5 m/s);
**amber** within `mid_m` (5.0 m); **green** beyond; **cyan** secondary/other-node highlight when
`remote_by_range` is `false`; **grey** stale (`age_ms > stale_ms`, 500 ms).

---

## IF-8 Config (`config/seeker.json`)

Key ownership is strictly split: each section has one owner.

| Section | Keys | Owner |
| --- | --- | --- |
| `remote` (node specs & ingest) | `ws_port` (`8080`), `video` (`width`: 1280, `height`: 720, `fps`: 15, `jpeg_quality`: 75), `nodes[]` (`id`, `device`, `hfov_deg`, `x_m`, `y_m`, `yaw_deg`, `image_w`, `image_h`) | **Daniel** |
| `remote` (fusion policy) | `drop_ms` (1500), `assoc_m` (0.8), `pose_source` (`"message"` \| `"config"`) | **Felix** |
| `colours` | `near_m` (2.0), `mid_m` (5.0), `closing_mps` (0.5), `stale_ms` (500), `remote_by_range` (true/false) | **Felix** |
| `display` | `sink` (`"screen"`), `background` (`"wearer"` \| `"radar"`), `view_node` (`"R1"`), `video_port` (`47106`), `mirror` (`"none"`), `line_px` (2), `radar_m` (8.0), `size` (`[1280, 720]`), `fps` (30), `swap_rb` (false), `display_index` (0), `wearer` (optional object, EagleEye: `node` (`"W"`), `pose` (`[x_m, y_m, yaw_deg]`), `pose_source` (`"auto"` \| `"static"` \| `"fusion"`), `hfov_deg`, `image_w`, `image_h`, `cam_h_m` (1.7), `assoc_deg` (6), `assoc_range_m` (1.5), `yaw_alpha` (0.35), `visor` (false), `inset` (true), `walls` (`[[x1, y1, x2, y2]]`)) | **Felix** |
| `fusion` | `rate_hz` (30), `tracks_host` (`"127.0.0.1"`), `tracks_port` (`47102`), `wearer_yaw_deg` (0.0) | **Felix** |
| `prio` | `hud_display` (50 fifo), `hud_supervisor` (45 fifo), `hud_fusion` (30 rr), `hud_low` (10) | **Felix** |
| `health` | `beat_ms` (100), `dead_ms` (500), `backoff_ms` (500) | **Felix** |
| `supervisor` | `children[]` (`hud_fusion`, `hud_display`, optional `ws_ingest`) | **Felix** |

---

## IF-9 Logs and IF-10 Command Lines

- `logs/latency.jsonl`: `frame_seq`, `t_present_ms`, `hud_prio`, `stress`, `video_age_ms`, `tracks_age_ms`, `run`.
- `logs/events.jsonl`: supervisor restarts, node LOST/resumed, background fallbacks.
- Relay on Pi: `python3 remote/ws_ingest.py --fusion 127.0.0.1 --fusion-port 47103 --video-host 127.0.0.1 --video-port 47106 --port 8080 --tls-cert cert.pem --tls-key key.pem`
- Display on Pi: `build/hud_display --sink screen --background wearer --view-node R1`
- EagleEye on Pi: `build/hud_display --config config/seeker.eagleeye.json --view wearer [--wearer-node W] [--pose W=x,y,yaw] [--visor --mirror]`
- Supervisor on Pi: `sudo build/hud_supervisor --config config/seeker.json`
