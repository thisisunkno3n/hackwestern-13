#!/usr/bin/env python3
"""Integration test for the Pi contacts cap (Task 4).

Starts remote/ws_ingest.py, opens a raw (stdlib-only) WebSocket client, sends a
single `phone_detections` message carrying 20 contacts, and asserts that the
forwarded IF-5.4 `remote_contacts` UDP datagram carries exactly 16 contacts
(SEEKER_MAX_CONTACTS in common/seeker_msgs.h). Prints PASS on success.
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


def free_tcp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def ws_connect(host, port, timeout=5.0):
    """Minimal RFC6455 client handshake. Returns a connected, upgraded socket."""
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
    """Send a masked client text frame (client frames MUST be masked)."""
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


def main():
    udp_port = free_tcp_port()  # a free port number; we bind it for UDP below
    # Bind our UDP receiver FIRST so the bridge's datagrams have a listener.
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx.bind(("127.0.0.1", udp_port))
    rx.settimeout(5.0)

    ws_port = free_tcp_port()
    proc = subprocess.Popen(
        [sys.executable, WS_INGEST,
         "--port", str(ws_port),
         "--fusion", "127.0.0.1", "--fusion-port", str(udp_port),
         "--video-port", str(free_tcp_port())],  # steer video TCP away from anything real
        stderr=subprocess.PIPE)
    try:
        c = ws_connect("127.0.0.1", ws_port)
        contacts = [{"id": "X", "bearing_deg": float(i), "conf": 0.9,
                     "box": [i, i, 10, 20], "range_m": 2.0, "range_src": "full"}
                    for i in range(20)]
        msg = {"type": "phone_detections", "v": 2, "node_id": "R1",
               "pose": None, "width": 640, "height": 480, "hfov_deg": 66,
               "contacts": contacts}
        ws_send_text(c, json.dumps(msg))

        data, _ = rx.recvfrom(65535)
        out = json.loads(data.decode())
        assert out.get("type") == "remote_contacts", "unexpected type: %r" % out.get("type")
        got = len(out.get("contacts", []))
        assert got == 16, "expected 16 contacts, got %d" % got
        # ids are stamped per index and capped
        assert out["contacts"][0]["id"] == "X", "phone-supplied id should be kept"
        print("PASS: 20 contacts in -> %d forwarded (cap = 16)" % got)
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
