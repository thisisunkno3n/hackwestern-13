#!/usr/bin/env python3
"""End-to-end test for IF-5.6 video forwarding in remote/ws_ingest.py (pure stdlib).

Binds a TCP listener on 127.0.0.1:47106 (the hud_display video dest), starts ws_ingest.py on a
spare WS port with no TLS, opens a raw WebSocket client, does the handshake, sends a sensor
hello then three client-masked binary frames (minimal valid JPEGs, one padded to ~120 KB so the
64-bit length path and the fast XOR unmask are exercised at a realistic size), and reads the
SKV2 frames back off the TCP socket.

Asserts, for each of the three frames: the message begins with b"SKV2", version byte == 2,
header width/height == the hello's 640/480, the embedded JPEG is byte-for-byte what was sent
(so it starts FF D8 / ends FF D9), and seq strictly increases across the three frames.

Prints PASS and exits 0 on success; prints the mismatch (and ws_ingest stderr) and exits 1.
"""
import base64
import hashlib
import json
import os
import socket
import struct
import subprocess
import sys
import time

WS_PORT = 8097
VIDEO_PORT = 47106
WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
VHEADER = struct.Struct(">4sBBH8sQHHII")   # IF-5.6 header, 36 bytes
HERE = os.path.dirname(os.path.abspath(__file__))
WS_INGEST = os.path.join(HERE, "remote", "ws_ingest.py")


def fail(proc, msg):
    print("FAIL:", msg)
    out = ""
    if proc is not None:
        try:
            proc.kill()                       # stop it so the stderr pipe reaches EOF
            out = proc.communicate(timeout=5)[1].decode("utf-8", "replace")
        except Exception:
            out = ""
    if out:
        print("--- ws_ingest stderr ---")
        print(out)
    sys.exit(1)


def make_jpeg(pad_to=0):
    """A minimal JPEG-shaped payload: SOI ... EOI, padded in the middle to pad_to bytes."""
    head = b"\xff\xd8\xff\xe0\x00\x10JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00"
    tail = b"\xff\xd9"
    body = b"\x5a" * max(0, pad_to - len(head) - len(tail))
    data = head + body + tail
    if len(data) < 300:                      # ensure > 125 bytes (extended length path)
        data = head + b"\x5a" * (300 - len(head) - len(tail)) + tail
    return data


def ws_frame_binary(payload):
    """A single FIN binary frame (opcode 0x2), client-masked with a random 4-byte mask."""
    b0 = 0x82                                # FIN + binary
    ln = len(payload)
    if ln < 126:
        hdr = struct.pack(">BB", b0, 0x80 | ln)
    elif ln < 65536:
        hdr = struct.pack(">BBH", b0, 0x80 | 126, ln)
    else:
        hdr = struct.pack(">BBQ", b0, 0x80 | 127, ln)
    mask = os.urandom(4)                      # non-zero mask: a broken unmask can't pass
    masked = bytes(payload[i] ^ mask[i & 3] for i in range(ln))
    return hdr + mask + masked


def ws_frame_text(obj):
    payload = json.dumps(obj).encode()
    ln = len(payload)
    assert ln < 126
    mask = os.urandom(4)
    masked = bytes(payload[i] ^ mask[i & 3] for i in range(ln))
    return struct.pack(">BB", 0x81, 0x80 | ln) + mask + masked


def recvn(sock, n):
    buf = b""
    while len(buf) < n:
        d = sock.recv(n - len(buf))
        if not d:
            raise ConnectionError("tcp closed after %d/%d bytes" % (len(buf), n))
        buf += d
    return buf


def main():
    # 1. TCP listener standing in for hud_display on 47106.
    lsn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    lsn.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    lsn.bind(("127.0.0.1", VIDEO_PORT))
    lsn.listen(1)
    lsn.settimeout(8)

    # 2. Start ws_ingest.py (no TLS).
    proc = subprocess.Popen(
        [sys.executable, WS_INGEST, "--port", str(WS_PORT),
         "--video-host", "127.0.0.1", "--video-port", str(VIDEO_PORT), "--view-node", "R1"],
        stderr=subprocess.PIPE)
    try:
        # wait for the WS port to accept connections
        deadline = time.time() + 8
        ws = None
        while time.time() < deadline:
            try:
                ws = socket.create_connection(("127.0.0.1", WS_PORT), timeout=1)
                break
            except OSError:
                if proc.poll() is not None:
                    fail(proc, "ws_ingest exited early")
                time.sleep(0.1)
        if ws is None:
            fail(proc, "ws_ingest WS port never came up")

        # 3. WebSocket handshake.
        key = base64.b64encode(os.urandom(16)).decode()
        ws.sendall(("GET /ws HTTP/1.1\r\nHost: 127.0.0.1\r\nUpgrade: websocket\r\n"
                    "Connection: Upgrade\r\nSec-WebSocket-Key: %s\r\n"
                    "Sec-WebSocket-Version: 13\r\n\r\n" % key).encode())
        resp = b""
        ws.settimeout(5)
        while b"\r\n\r\n" not in resp:
            d = ws.recv(1024)
            if not d:
                fail(proc, "no handshake response")
            resp += d
        acc = base64.b64encode(hashlib.sha1((key + WS_GUID).encode()).digest()).decode()
        if acc.encode() not in resp:
            fail(proc, "bad Sec-WebSocket-Accept in handshake: %r" % resp)

        # 4. Sensor hello (sets node_id + width/height), then three binary JPEG frames.
        ws.sendall(ws_frame_text({"role": "sensor", "node_id": "R1",
                                  "width": 640, "height": 480}))
        jpegs = [make_jpeg(), make_jpeg(400), make_jpeg(120000)]

        # 5. Accept the TCP connection ws_ingest makes on the first frame, read frames back.
        conn = None
        last_seq = None
        for idx, jpeg in enumerate(jpegs):
            ws.sendall(ws_frame_binary(jpeg))
            if conn is None:
                conn, _ = lsn.accept()
                conn.settimeout(8)
            hdr = recvn(conn, VHEADER.size)
            magic, ver, flags, _res, node, _ts, w, h, seq, plen = VHEADER.unpack(hdr)
            payload = recvn(conn, plen)
            if magic != b"SKV2":
                fail(proc, "frame %d magic %r != SKV2" % (idx, magic))
            if ver != 2:
                fail(proc, "frame %d version %d != 2" % (idx, ver))
            if (w, h) != (640, 480):
                fail(proc, "frame %d size %dx%d != 640x480" % (idx, w, h))
            if node.rstrip(b"\0") != b"R1":
                fail(proc, "frame %d node %r != R1" % (idx, node))
            if plen != len(jpeg) or payload != jpeg:
                fail(proc, "frame %d JPEG mismatch (len %d vs %d)" % (idx, plen, len(jpeg)))
            if payload[:2] != b"\xff\xd8" or payload[-2:] != b"\xff\xd9":
                fail(proc, "frame %d not FF D8..FF D9" % idx)
            if last_seq is not None and seq <= last_seq:
                fail(proc, "frame %d seq %d not > %d" % (idx, seq, last_seq))
            last_seq = seq

        # 6. A payload already SKV2-wrapped by the phone must be forwarded byte-for-byte
        #    (its ts_ms/seq preserved, not restamped).
        pre = VHEADER.pack(b"SKV2", 2, 0, 0, b"R1".ljust(8, b"\0"), 1234567, 320, 240, 9999,
                           len(jpegs[0])) + jpegs[0]
        ws.sendall(ws_frame_binary(pre))
        got = recvn(conn, len(pre))
        if got != pre:
            fail(proc, "SKV2 passthrough not byte-identical")

        print("PASS")
        return 0
    finally:
        try:
            proc.terminate()
            proc.wait(timeout=3)
        except Exception:
            try:
                proc.kill()
            except Exception:
                pass
        try:
            lsn.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
