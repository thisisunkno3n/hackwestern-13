#!/usr/bin/env python3
"""Fake phone.html for testing Daniel's remote/ws_ingest.py (PR #1) without a phone.

Connects to ws_ingest over a plain WebSocket and sends what phone.html sends: one
{"role":"sensor","node_id","pose"} hello, then {"node_id","bearing_deg","conf","box","range_m"}
at --rate Hz. --no-range sends range_m null (to exercise the HUD's range-less path).

  python3 scripts/ws_phone_sim.py --host 127.0.0.1 --port 8099 --node R2 --pose=2,3,215 --no-range
  python3 scripts/ws_phone_sim.py --port 8099 --node R2 --pose=0.66,-1.737,0 --tag-id 7   # R2 on tag 7
  python3 scripts/ws_phone_sim.py --port 8080 --node R1 --video --vfps 24 --queue-ms 60   # latency

--video also streams SKV2 frames like phone.html does: built on the phone, ts_ms = Date.now()
(epoch ms, not the Pi's clock), a test card JPEG (video_send.py). --queue-ms N holds a random
0..N ms between stamping and sending (uplink queueing for hud_display's latency line), --age-ms
N sets the optional SKV2_FLAG_AGE capture-to-send field. WebSocket pings from the bridge get a
pong, as Safari does.
"""
import argparse, base64, json, math, os, random, select, socket, struct, sys, time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
SKV2 = struct.Struct(">4sBBH8sQHHII")
SKV2_FLAG_AGE = 0x10


def ws_frame(op, data):
    """One masked client-to-server frame (the XOR in one big-int step, fast for JPEGs)."""
    mask = os.urandom(4)
    n = len(data)
    head = bytes([0x80 | op])
    if n < 126:
        head += bytes([0x80 | n])
    elif n < 65536:
        head += bytes([0x80 | 126]) + struct.pack(">H", n)
    else:
        head += bytes([0x80 | 127]) + struct.pack(">Q", n)
    if n:
        full = mask * ((n + 3) // 4)
        data = (int.from_bytes(data, "big") ^ int.from_bytes(full[:n], "big")).to_bytes(n, "big")
    return head + mask + data


def answer_pings(sock, buf):
    """Reads what the server sent (unmasked frames); pongs every ping. Returns the leftover."""
    while (hasattr(sock, "pending") and sock.pending()) or select.select([sock], [], [], 0)[0]:
        d = sock.recv(65536)
        if not d:
            raise ConnectionError("server closed")
        buf += d
    while len(buf) >= 2:
        op, n, off = buf[0] & 0x0F, buf[1] & 0x7F, 2
        if n == 126:
            if len(buf) < 4:
                break
            n, off = struct.unpack(">H", buf[2:4])[0], 4
        elif n == 127:
            if len(buf) < 10:
                break
            n, off = struct.unpack(">Q", buf[2:10])[0], 10
        if len(buf) < off + n:
            break
        data, buf = buf[off:off + n], buf[off + n:]
        if op == 0x9:
            sock.sendall(ws_frame(0xA, data))
    return buf


def ws_send(sock, text):
    data = text.encode()
    mask = os.urandom(4)
    head = bytes([0x81])
    n = len(data)
    if n < 126:
        head += bytes([0x80 | n])
    else:
        head += bytes([0x80 | 126]) + struct.pack(">H", n)
    sock.sendall(head + mask + bytes(b ^ mask[i & 3] for i, b in enumerate(data)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--node", default="R2")
    ap.add_argument("--pose", default="0,0,0")
    ap.add_argument("--hfov", type=float, default=66.0)
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--seconds", type=float, default=0)
    ap.add_argument("--no-range", action="store_true")
    ap.add_argument("--tls", action="store_true", help="wss:// (run-live.sh's HTTPS bridge; the "
                    "self-signed cert is not checked)")
    ap.add_argument("--video", action="store_true", help="also stream SKV2 video like phone.html")
    ap.add_argument("--vfps", type=float, default=15.0)
    ap.add_argument("--vsize", default="640x480")
    ap.add_argument("--queue-ms", type=float, default=0.0)
    ap.add_argument("--age-ms", type=int, default=None)
    ap.add_argument("--tag-id", type=int, default=None,
                    help="send --pose in every message as measured from this ArUco tag (pose.tag_id)")
    a = ap.parse_args()
    x, y, yaw = [float(v) for v in a.pose.split(",")]
    pose = {"x_m": x, "y_m": y, "yaw_deg": yaw}
    if a.tag_id is not None:
        pose["tag_id"] = a.tag_id

    s = socket.create_connection((a.host, a.port))
    if a.tls:
        import ssl
        ctx = ssl.create_default_context()
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        s = ctx.wrap_socket(s, server_hostname=a.host)
    key = base64.b64encode(os.urandom(16)).decode()
    s.sendall(("GET /ws HTTP/1.1\r\nHost: %s\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
               "Sec-WebSocket-Key: %s\r\nSec-WebSocket-Version: 13\r\n\r\n" % (a.host, key)).encode())
    resp = s.recv(1024)
    if b" 101 " not in resp.split(b"\r\n", 1)[0]:
        sys.exit("[ws_phone_sim] handshake failed: %r" % resp[:80])
    ws_send(s, json.dumps({"role": "sensor", "node_id": a.node, "pose": pose}))
    print("[ws_phone_sim] %s connected to ws://%s:%d/ws" % (a.node, a.host, a.port), file=sys.stderr)
    t0 = time.monotonic()
    W, H = 640, 480
    f = (W / 2) / math.tan(math.radians(a.hfov) / 2)
    cards, vw, vh = [], 0, 0
    if a.video:
        import video_send
        vw, vh = [int(v) for v in a.vsize.split("x")]
        cards = [video_send.test_card(vw, vh, i, 30) for i in range(30)]
    next_det = next_vid = t0
    vseq, rbuf = 0, b""
    while a.seconds <= 0 or time.monotonic() - t0 < a.seconds:
        now = time.monotonic()
        rbuf = answer_pings(s, rbuf)
        if a.video and now >= next_vid:
            next_vid += 1.0 / a.vfps
            ts = int(time.time() * 1000)  # Date.now()
            if a.queue_ms > 0:
                time.sleep(random.uniform(0, a.queue_ms) / 1000.0)
            flags, age = (SKV2_FLAG_AGE, a.age_ms) if a.age_ms is not None else (0, 0)
            jpg = cards[vseq % len(cards)]
            hdr = SKV2.pack(b"SKV2", 2, flags, age, a.node.encode()[:8].ljust(8, b"\0"), ts,
                            vw, vh, vseq, len(jpg))
            s.sendall(ws_frame(0x2, hdr + jpg))
            vseq += 1
        if now < next_det:
            time.sleep(max(0.0, min(next_det, next_vid if a.video else next_det) - now))
            continue
        next_det += 1.0 / a.rate
        t = now - t0
        cx = 0.5 + 0.3 * math.sin(t * 0.6)
        bh = int(H * 0.6)
        box = [int(cx * W - bh * 0.2), int(H * 0.2), int(bh * 0.4), bh]
        msg = {"node_id": a.node, "bearing_deg": (cx - 0.5) * a.hfov, "conf": 0.8, "box": box,
               "range_m": None if a.no_range else round(f * 1.7 / bh, 1)}
        if a.tag_id is not None:
            msg["pose"] = pose
        ws_send(s, json.dumps(msg))


if __name__ == "__main__":
    main()
