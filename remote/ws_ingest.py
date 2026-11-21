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
CTYPES = {".html": "text/html", ".js": "application/javascript", ".css": "text/css"}
LOG = lambda *a: print("[ws_ingest]", *a, file=sys.stderr, flush=True)


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
    op = b0 & 0x0F
    masked = b1 & 0x80
    ln = b1 & 0x7F
    if ln == 126:
        ln = struct.unpack(">H", recvall(c, 2))[0]
    elif ln == 127:
        ln = struct.unpack(">Q", recvall(c, 8))[0]
    mask = recvall(c, 4) if masked else b"\0\0\0\0"
    data = recvall(c, ln)
    if masked:
        data = bytes(data[i] ^ mask[i & 3] for i in range(ln))
    return op, data


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


class Bridge:
    def __init__(self, fusion_host, fusion_port, wire_v=2):
        self.dst = (fusion_host, fusion_port)
        self.wire_v = wire_v          # 1 for fusion builds that only accept v1
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def emit(self, node_id, pose, contacts, width, height, hfov_deg):
        # stamp ids + assemble IF-5.4 remote_contacts. Missing values stay null.
        for i, ct in enumerate(contacts, 1):
            ct.setdefault("id", "%s-%d" % (node_id, i))
            ct.setdefault("cls", "person")
            ct.setdefault("box", None)
            ct.setdefault("range_m", None)
        now_ms = int(time.monotonic() * 1000)   # Pi receive clock (IF-1 clock rule)
        msg = {"type": "remote_contacts", "v": self.wire_v, "src": node_id,
               "t_mono_ms": now_ms, "ts_ms": now_ms, "node_id": node_id,
               "pose": pose, "width": width, "height": height, "hfov_deg": hfov_deg,
               "contacts": contacts}
        self.udp.sendto(json.dumps(msg).encode(), self.dst)

    def ws_loop(self, c):
        node_id, pose = None, None           # null until the phone gives a measured pose
        width = height = hfov_deg = None
        n = 0
        while True:
            op, data = ws_recv(c)
            if op == 0x8:
                break
            if op == 0x9:
                c.sendall(bytes([0x8A, 0])); continue
            if op != 0x1:
                continue
            try:
                m = json.loads(data.decode())
            except Exception:
                continue
            if m.get("node_id"):
                node_id = m["node_id"]
            if isinstance(m.get("pose"), dict):
                pose = m["pose"]            # explicit pose:null leaves the last known value
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
                             "bearing_deg": m["bearing_deg"], "range_m": m.get("range_m")}]
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
    args = ap.parse_args()
    ctx = None
    if args.tls_cert and args.tls_key:
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        ctx.load_cert_chain(args.tls_cert, args.tls_key)
    bridge = Bridge(args.fusion, args.fusion_port, args.wire_version)
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
    while True:
        c, _ = s.accept()
        threading.Thread(target=handle, args=(c, bridge, ctx), daemon=True).start()


if __name__ == "__main__":
    main()
