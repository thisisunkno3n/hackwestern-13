#!/usr/bin/env python3
"""Seeker all-phone pivot: WebSocket -> UDP remote_contacts bridge.

Browsers can't send UDP, so each phone's sensor page connects here over
WebSocket and streams detections; this bridge re-emits them as IF-5.4
`remote_contacts` UDP datagrams to hud_fusion on the Pi. Also serves the
static phone/viewer pages. Pure stdlib (runs on QNX Python).

  python3 remote/ws_ingest.py --fusion 127.0.0.1 --port 8080

Phone -> this bridge (WS /ws):
  first msg:  {"role":"sensor","node_id":"R1","pose":{"x_m":0,"y_m":0,"yaw_deg":0}}
  then:       {"contacts":[{"cls":"person","conf":0.9,"box":[x,y,w,h],"bearing_deg":-12.0,"range_m":2.8}]}
              (or a single {"bearing_deg":..,"conf":..} which is wrapped into one contact)
Bridge -> hud_fusion (UDP :47103): IF-5.4 remote_contacts, t_mono_ms stamped on receive
(cross-machine clock rule: the Pi ages by its own receive time).
"""
import socket, threading, json, hashlib, base64, struct, os, time, argparse, sys, ssl

WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
SEEKER_MAX_DATAGRAM = 8192   # IF-5 / common/seeker_msgs.h: max 8 KB per UDP datagram
CTYPES = {".html": "text/html", ".js": "application/javascript", ".css": "text/css"}
LOG = lambda *a: print("[ws_ingest]", *a, file=sys.stderr, flush=True)

VHEADER = struct.Struct(">4sBBH8sQHHII")   # IF-5.6 SKV2 video_frame header (36 bytes)


# ---- websocket framing (from server.py) ----
def recvall(c, n):
    b = b""
    while len(b) < n:
        d = c.recv(n - len(b))
        if not d:
            raise ConnectionError("closed")
        b += d
    return b


def ws_recv(c):
    b0, b1 = recvall(c, 2)
    fin = b0 & 0x80
    op = b0 & 0x0F
    masked = b1 & 0x80
    ln = b1 & 0x7F
    if ln == 126:
        ln = struct.unpack(">H", recvall(c, 2))[0]
    elif ln == 127:
        ln = struct.unpack(">Q", recvall(c, 8))[0]
    mask = recvall(c, 4) if masked else b"\0\0\0\0"
    data = recvall(c, ln)
    if masked and ln:
        if op in (0x0, 0x2):
            # binary / continuation (video): whole-buffer XOR, far faster than per-byte
            # for 60-180 KB JPEGs on the Pi. int.from_bytes keeps leading zeros via to_bytes(ln).
            full = mask * ((ln + 3) // 4)
            data = (int.from_bytes(data, "big") ^
                    int.from_bytes(full[:ln], "big")).to_bytes(ln, "big")
        else:
            data = bytes(data[i] ^ mask[i & 3] for i in range(ln))
    return fin, op, data


def ws_handshake(c, key):
    acc = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
    c.sendall(("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\n"
               "Connection: Upgrade\r\nSec-WebSocket-Accept: " + acc + "\r\n\r\n").encode())


def serve_static(c, path):
    path = path.split("?")[0]          # drop query string before matching
    if path in ("/", ""):
        path = "/phone.html"
    fn = os.path.basename(path)
    full = os.path.join(os.path.dirname(os.path.abspath(__file__)), fn)
    if os.path.isfile(full):
        body = open(full, "rb").read()
        ct = CTYPES.get(os.path.splitext(fn)[1], "application/octet-stream")
        c.sendall(("HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %d\r\n"
                   "Connection: close\r\n\r\n" % (ct, len(body))).encode() + body)
    else:
        c.sendall(b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")


class VideoForwarder:
    """IF-5.6 video relay: WS binary frames -> TCP 47106 (hud_display).

    One persistent TCP connection with TCP_NODELAY, reconnected ~1s after a drop or while
    the display is not listening yet. A single-slot latest-frame mailbox (freshness rule):
    if the TCP send can't keep up, older frames are dropped, never queued. A background
    thread does the SKV2 wrapping and sending so the WS receive loop never blocks on the
    socket. Never crashes the thread: every per-frame error is caught, logged and dropped.
    """

    def __init__(self, host, port):
        self.dst = (host, port)
        self.cond = threading.Condition()
        self.pending = None       # (node, payload, w, h) -- latest frame only
        self.seqs = {}            # per-node seq counter
        self.sent = self.dropped = 0
        self.stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def submit(self, node, payload, width, height):
        with self.cond:
            if self.pending is not None:
                self.dropped += 1          # a frame is already waiting: drop the old one
            self.pending = (node, payload, width, height)
            self.cond.notify()

    def _frame(self, node, payload, width, height):
        if payload[:4] == b"SKV2":
            return payload                 # phone already sent a full IF-5.6 frame
        seq = self.seqs.get(node, 0)
        self.seqs[node] = (seq + 1) & 0xFFFFFFFF
        try:
            w = max(0, min(65535, int(width or 0)))
        except (TypeError, ValueError):
            w = 0
        try:
            h = max(0, min(65535, int(height or 0)))
        except (TypeError, ValueError):
            h = 0
        node_b = node.encode("ascii", "replace")[:8].ljust(8, b"\0")
        ts_ms = int(time.monotonic() * 1000)   # restamp with Pi monotonic ms on receive
        return VHEADER.pack(b"SKV2", 2, 0, 0, node_b, ts_ms, w, h, seq, len(payload)) + payload

    def _run(self):
        sock = None
        while not self.stop:
            with self.cond:
                while self.pending is None and not self.stop:
                    self.cond.wait()
                if self.stop:
                    break
                node, payload, width, height = self.pending
                self.pending = None
            if sock is None:
                try:
                    sock = socket.create_connection(self.dst, timeout=2)
                    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    LOG("video: connected to %s:%d" % self.dst)
                except OSError as e:
                    sock = None
                    LOG("video: dest %s:%d not up (%s), retrying" % (self.dst + (e,)))
                    time.sleep(1)           # drop this frame; a fresher one will come
                    continue
            try:
                frame = self._frame(node, payload, width, height)
                sock.sendall(frame)         # 2s timeout from create_connection guards a stall
                self.sent += 1
                if self.sent % 30 == 0:
                    LOG("video: forwarded %d frames (%d dropped)" % (self.sent, self.dropped))
            except Exception as e:
                try:
                    sock.close()
                except Exception:
                    pass
                sock = None
                LOG("video: send failed (%s), reconnecting" % e)
                time.sleep(1)


class Bridge:
    def __init__(self, fusion_host, fusion_port, wire_v=2, view_node="R1", video=None):
        self.dst = (fusion_host, fusion_port)
        self.wire_v = wire_v          # 1 for fusion builds that only accept v1
        self.view_node = view_node    # only this node's binary frames are forwarded as video
        self.video = video            # VideoForwarder or None
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def emit(self, node_id, pose, contacts, width, height, hfov_deg):
        # stamp ids + assemble IF-5.4 remote_contacts. Missing values stay null.
        contacts = contacts[:16]   # Pi decoder limit, SEEKER_MAX_CONTACTS (common/seeker_msgs.h)
        for i, ct in enumerate(contacts, 1):
            ct.setdefault("id", "%s-%d" % (node_id, i))
            ct.setdefault("cls", "person")
            ct.setdefault("box", None)
            ct.setdefault("range_m", None)
            # `pts` (skeleton landmarks, phone.html) rides through verbatim when present; the C
            # fusion ignores the unknown key until it is parsed (IF-5, additive). Forwarded as-is.
        now_ms = int(time.monotonic() * 1000)   # Pi receive clock (IF-1 clock rule)
        msg = {"type": "remote_contacts", "v": self.wire_v, "src": node_id,
               "t_mono_ms": now_ms, "ts_ms": now_ms, "node_id": node_id,
               "pose": pose, "width": width, "height": height, "hfov_deg": hfov_deg,
               "contacts": contacts}
        wire = json.dumps(msg).encode()
        # Backward-compat guard: an oversized datagram is truncated by the C recv buffer
        # (buf[SEEKER_MAX_DATAGRAM + 1]) and then fails to parse, losing EVERY field including
        # bearing/range. The skeleton is optional, so if `pts` pushed us over the 8 KB limit,
        # drop only `pts` and re-encode, keeping the core contacts intact.
        if len(wire) > SEEKER_MAX_DATAGRAM:
            for ct in contacts:
                ct.pop("pts", None)
            wire = json.dumps(msg).encode()
        self.udp.sendto(wire, self.dst)

    def ws_loop(self, c):
        node_id, pose = None, None           # null until the phone gives a measured pose
        width = height = hfov_deg = None
        n = 0
        vbuf = None                          # accumulates a fragmented binary (video) message
        while True:
            fin, op, data = ws_recv(c)
            if op == 0x8:
                break
            if op == 0x9:
                c.sendall(bytes([0x8A, 0])); continue
            if op == 0xA:
                continue                     # pong: ignore
            if op == 0x2 or (op == 0x0 and vbuf is not None):
                # binary video frame (0x2) or its continuation (0x0). Control frames may be
                # interleaved; they don't touch vbuf, so reassembly survives them.
                vbuf = bytearray(data) if op == 0x2 else (vbuf + data)
                if not fin:
                    continue
                payload = bytes(vbuf)
                vbuf = None
                # forward video only from the wearer/view node; drop binary from other nodes
                if self.video is not None and node_id == self.view_node:
                    self.video.submit(node_id, payload, width, height)
                continue
            if op != 0x1:
                continue
            try:
                m = json.loads(data.decode())
            except Exception:
                continue
            if m.get("node_id"):
                node_id = m["node_id"]
            if "pose" in m:                 # forward phone's pose verbatim; explicit null -> null
                pose = m["pose"] if isinstance(m["pose"], dict) else None
            if m.get("width") is not None:
                width = m["width"]
            if m.get("height") is not None:
                height = m["height"]
            if m.get("hfov_deg") is not None:
                hfov_deg = m["hfov_deg"]
            if "contacts" in m:
                contacts = m["contacts"]
            elif "bearing_deg" in m:
                contacts = [{"conf": m.get("conf", 0.9), "box": m.get("box"),
                             "bearing_deg": m["bearing_deg"], "range_m": m.get("range_m"),
                             "pts": m.get("pts")}]
            else:
                LOG("node %s connected pose=%s" % (node_id, pose)); continue
            if node_id:
                self.emit(node_id, pose, contacts, width, height, hfov_deg)
                n += 1
                if n % 30 == 0:
                    LOG("node %s: forwarded %d msgs" % (node_id, n))


def handle(c, bridge, ctx):
    try:
        if ctx is not None:
            try:
                c = ctx.wrap_socket(c, server_side=True)
            except Exception:
                c.close(); return
        data = b""
        while b"\r\n\r\n" not in data:
            d = c.recv(1024)
            if not d:
                return
            data += d
        head = data.split(b"\r\n\r\n", 1)[0].decode("latin1").split("\r\n")
        path = (head[0].split() + ["", ""])[1]
        hdrs = {}
        for ln in head[1:]:
            if ":" in ln:
                k, v = ln.split(":", 1); hdrs[k.strip().lower()] = v.strip()
        if path == "/ws" and hdrs.get("upgrade", "").lower() == "websocket":
            ws_handshake(c, hdrs.get("sec-websocket-key", ""))
            bridge.ws_loop(c)
        else:
            serve_static(c, path)
    except Exception:
        pass
    finally:
        try:
            c.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fusion", default="127.0.0.1", help="hud_fusion host (the Pi itself usually)")
    ap.add_argument("--fusion-port", type=int, default=47103)
    ap.add_argument("--port", type=int, default=8080, help="WS/HTTP listen port")
    ap.add_argument("--tls-cert", default=None, help="PEM cert -> serve HTTPS/WSS (needed for iPhone camera)")
    ap.add_argument("--tls-key", default=None, help="PEM private key")
    ap.add_argument("--wire-version", type=int, default=2, choices=[1, 2],
                    help="remote_contacts version to stamp (1 for fusion builds that only accept v1)")
    ap.add_argument("--view-node", default="R1",
                    help="only this node's binary WS frames are forwarded as IF-5.6 video")
    ap.add_argument("--video-host", default="127.0.0.1", help="hud_display host for IF-5.6 video")
    ap.add_argument("--video-port", type=int, default=47106, help="hud_display TCP port for video")
    args = ap.parse_args()
    ctx = None
    if args.tls_cert and args.tls_key:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(args.tls_cert, args.tls_key)
    video = VideoForwarder(args.video_host, args.video_port)
    bridge = Bridge(args.fusion, args.fusion_port, args.wire_version, args.view_node, video)
    s = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
    except OSError:
        pass
    s.bind(("", args.port)); s.listen(16)
    scheme = "https" if ctx else "http"
    LOG("phones -> %s://:%d/  (sensor page)  ->  remote_contacts UDP %s:%d%s"
        % (scheme, args.port, args.fusion, args.fusion_port,
           "" if ctx else "   [no TLS: iPhone camera needs --tls-cert/--tls-key]"))
    LOG("video: node %s binary frames -> IF-5.6 TCP %s:%d"
        % (args.view_node, args.video_host, args.video_port))
    while True:
        c, _ = s.accept()
        threading.Thread(target=handle, args=(c, bridge, ctx), daemon=True).start()


if __name__ == "__main__":
    main()
