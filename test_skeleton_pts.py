#!/usr/bin/env python3
"""Integration test: ws_ingest.py forwards the ADDITIVE skeleton `pts` through to the IF-5.4
remote_contacts UDP datagram, and its size guard protects the core fields.

Reuses the raw-stdlib WebSocket client from test_contacts_cap.py. Two assertions:

  1. A phone_detections contact carrying `pts` (COCO-17 flat int landmarks) is forwarded with the
     pts array byte-identical, alongside the unchanged bearing/range/box.
  2. A fat 16-contact message whose pts would push the datagram past SEEKER_MAX_DATAGRAM (8192 B)
     still arrives <= 8192 B with every bearing intact -- the guard drops only `pts`, never the
     core contact, so an oversized skeleton can never make the whole datagram unparseable.
"""
import base64
import json
import os
import socket
import struct
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WS_INGEST = os.path.join(HERE, "remote", "ws_ingest.py")
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
MAX_DGRAM = 8192


def free_tcp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def ws_connect(host, port, timeout=5.0):
    deadline = time.time() + timeout
    last = None
    while time.time() < deadline:
        try:
            c = socket.create_connection((host, port), timeout=2.0)
            break
        except OSError as e:
            last = e
            time.sleep(0.1)
    else:
        raise RuntimeError("could not connect to ws_ingest: %s" % last)
    key = base64.b64encode(os.urandom(16)).decode()
    req = ("GET /ws HTTP/1.1\r\nHost: %s:%d\r\nUpgrade: websocket\r\n"
           "Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n"
           "Sec-WebSocket-Version: 13\r\n\r\n" % (host, port, key))
    c.sendall(req.encode())
    resp = b""
    while b"\r\n\r\n" not in resp:
        d = c.recv(1024)
        if not d:
            raise RuntimeError("ws handshake closed early")
        resp += d
    acc = base64.b64encode(
        __import__("hashlib").sha1((key + WS_GUID).encode()).digest()).decode()
    if b"101" not in resp.split(b"\r\n", 1)[0] or acc.encode() not in resp:
        raise RuntimeError("bad ws handshake response: %r" % resp[:120])
    return c


def ws_send_text(c, text):
    payload = text.encode()
    ln = len(payload)
    header = bytearray([0x81])  # FIN + text opcode
    if ln < 126:
        header.append(0x80 | ln)
    elif ln < 65536:
        header.append(0x80 | 126)
        header += struct.pack(">H", ln)
    else:
        header.append(0x80 | 127)
        header += struct.pack(">Q", ln)
    mask = os.urandom(4)
    header += mask
    masked = bytes(payload[i] ^ mask[i & 3] for i in range(ln))
    c.sendall(bytes(header) + masked)


# a realistic COCO-17 flat int skeleton: 51 ints in [0..1000] / [0..100]
SKELETON = [500, 100, 99, 470, 90, 95, 530, 90, 95, 440, 95, 80, 560, 95, 80,
            430, 300, 98, 570, 300, 98, 410, 460, 90, 590, 460, 90, 400, 600, 85,
            600, 600, 85, 455, 560, 97, 545, 560, 97, 450, 780, 92, 550, 780, 92,
            448, 980, 88, 552, 980, 88]


def recv_msg(rx):
    data, _ = rx.recvfrom(65535)
    assert len(data) <= MAX_DGRAM, "datagram %d B exceeds SEEKER_MAX_DATAGRAM" % len(data)
    out = json.loads(data.decode())
    assert out.get("type") == "remote_contacts", "unexpected type: %r" % out.get("type")
    return out, len(data)


def main():
    udp_port = free_tcp_port()
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", udp_port))
    rx.settimeout(5.0)

    ws_port = free_tcp_port()
    proc = subprocess.Popen(
        [sys.executable, WS_INGEST,
         "--port", str(ws_port),
         "--fusion", "127.0.0.1", "--fusion-port", str(udp_port),
         "--video-port", str(free_tcp_port())],
        stderr=subprocess.PIPE)
    try:
        c = ws_connect("127.0.0.1", ws_port)

        # --- 1) a single contact carrying pts: forwarded byte-identical ---
        msg = {"type": "phone_detections", "v": 2, "node_id": "R2",
               "pose": None, "width": 640, "height": 480, "hfov_deg": 66,
               "contacts": [{"id": "R2-1", "bearing_deg": -12.5, "conf": 0.9,
                             "box": [100, 50, 60, 180], "range_m": 2.8,
                             "range_src": "full", "pts": SKELETON}]}
        ws_send_text(c, json.dumps(msg))
        out, _ = recv_msg(rx)
        got = out["contacts"][0]
        assert got.get("pts") == SKELETON, "pts not forwarded verbatim: %r" % got.get("pts")
        assert abs(got["bearing_deg"] - (-12.5)) < 1e-6, "bearing changed"
        assert got["box"] == [100, 50, 60, 180], "box changed"
        print("PASS 1: pts (%d ints) forwarded byte-identical with bearing/box intact" % len(SKELETON))

        # --- 2) fat 16-contact message: guard keeps it <= 8192 B, bearings survive ---
        contacts = [{"id": "R2-%d" % i, "bearing_deg": float(i), "conf": 0.9,
                     "box": [i, i, 40, 160], "range_m": 3.0, "range_src": "full",
                     # worst case: all off-frame 4-digit coords -> maximal pts bytes
                     "pts": [1234, -987, 100] * 17}
                    for i in range(16)]
        fat = {"type": "phone_detections", "v": 2, "node_id": "R2", "pose": None,
               "width": 640, "height": 480, "hfov_deg": 66, "contacts": contacts}
        raw_len = len(json.dumps({**fat, "type": "remote_contacts"}))
        ws_send_text(c, json.dumps(fat))
        out, dlen = recv_msg(rx)
        assert len(out["contacts"]) == 16, "expected 16 contacts, got %d" % len(out["contacts"])
        for i, ct in enumerate(out["contacts"]):
            assert abs(ct["bearing_deg"] - float(i)) < 1e-6, "bearing %d lost" % i
        dropped = "pts" not in out["contacts"][0]
        print("PASS 2: 16-contact fat message raw ~%d B -> delivered %d B (<= %d); "
              "bearings intact; pts %s"
              % (raw_len, dlen, MAX_DGRAM, "dropped by guard" if dropped else "kept"))
    finally:
        try:
            c.close()
        except Exception:
            pass
        proc.terminate()
        try:
            proc.wait(timeout=3)
        except Exception:
            proc.kill()
        rx.close()


if __name__ == "__main__":
    main()
