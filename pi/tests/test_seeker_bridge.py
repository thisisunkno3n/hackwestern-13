#!/usr/bin/env python3
"""Host tests for scripts/seeker_bridge.py (no phone, no browser): SKV2 framing, eager connect
and reconnect of the video link, video_ctrl to the view node only, and the unchanged fallback.
Run: python3 tests/test_seeker_bridge.py"""
import os
import socket
import struct
import sys
import threading
import time
import json
import types
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"))
import seeker_bridge as sb  # noqa: E402

HDR = struct.Struct(">4sBBH8sQHHII")


def free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def listen(port):
    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("127.0.0.1", port))
    s.listen(4)
    s.settimeout(5)
    return s


def recv_exact(c, n):
    b = b""
    while len(b) < n:
        d = c.recv(n - len(b))
        if not d:
            raise ConnectionError("closed")
        b += d
    return b


def wait_for(pred, secs=5.0):
    end = time.monotonic() + secs
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.02)
    return False


class FakeSock:
    def __init__(self):
        self.out = b""

    def sendall(self, b):
        self.out += b


class TestFraming(unittest.TestCase):
    def test_text_frame(self):
        self.assertEqual(sb.ws_text_frame("hi"), b"\x81\x02hi")
        f = sb.ws_text_frame("x" * 200)
        self.assertEqual(f[:4], b"\x81\x7e\x00\xc8")
        self.assertEqual(len(f), 204)


class TestForwarder(unittest.TestCase):
    def test_connects_before_any_frame_and_reconnects(self):
        port = free_port()
        fwd = sb.SeekerVideoForwarder("127.0.0.1", port)  # hud_display not up yet
        try:
            time.sleep(0.3)
            self.assertFalse(fwd.connected)
            srv = listen(port)
            c, _ = srv.accept()  # connects with no frame submitted
            self.assertTrue(wait_for(lambda: fwd.connected))

            jpeg = b"\xff\xd8" + b"\x11" * 100 + b"\xff\xd9"
            fwd.submit("R1", jpeg, 640, 480)
            magic, v, flags, res, node, ts, w, h, seq, ln = HDR.unpack(recv_exact(c, 36))
            self.assertEqual((magic, v, flags, res), (b"SKV2", 2, 0, 0))
            self.assertEqual((node.rstrip(b"\0"), w, h, seq, ln), (b"R1", 640, 480, 0, len(jpeg)))
            self.assertEqual(recv_exact(c, ln), jpeg)

            phone_frame = HDR.pack(b"SKV2", 2, 0, 0, b"R1".ljust(8, b"\0"), 123, 32, 24, 77, 4) + b"abcd"
            fwd.submit("R1", phone_frame, 640, 480)  # phone-built SKV2 passes through
            self.assertEqual(recv_exact(c, len(phone_frame)), phone_frame)

            c.close()  # hud_display restarts
            srv.close()
            self.assertTrue(wait_for(lambda: not fwd.connected))
            srv = listen(port)
            c, _ = srv.accept()
            self.assertTrue(wait_for(lambda: fwd.connected))
            fwd.submit("R1", jpeg, 640, 480)
            self.assertEqual(HDR.unpack(recv_exact(c, 36))[8], 1)  # seq keeps counting
            c.close()
            srv.close()
        finally:
            fwd.close()


class TestInstall(unittest.TestCase):
    def make_mod(self, frames):
        it = iter(frames)

        def ws_recv(c):
            f = next(it)
            if isinstance(f, Exception):
                raise f
            return f
        return types.SimpleNamespace(ws_recv=ws_recv, main=lambda: None,
                                     VideoForwarder=lambda h, p: None)

    def test_video_ctrl_once_to_view_node(self):
        hello = (0x80, 0x1, b'{"role":"sensor","node_id":"R1","pose":null}')
        det = (0x80, 0x1, b'{"type":"phone_detections","node_id":"R1","contacts":[]}')
        mod = self.make_mod([hello, det, (0x80, 0x2, b"SKV2..."), ConnectionError("closed")])
        phones = sb.install(mod, "R1", True, 15)
        self.assertIsNotNone(phones)
        c = FakeSock()
        for _ in range(3):
            mod.ws_recv(c)
        self.assertEqual(c.out.count(b"video_ctrl"), 1)
        self.assertIn(b'"stream_video": true', c.out)
        st = phones.conns[id(c)]
        self.assertEqual((st["node"], st["msgs"], st["frames"]), ("R1", 1, 1))
        with self.assertRaises(ConnectionError):
            mod.ws_recv(c)
        self.assertNotIn(id(c), phones.conns)

    def test_no_ctrl_for_other_nodes_or_when_off(self):
        hello = (0x80, 0x1, b'{"role":"sensor","node_id":"R2"}')
        mod = self.make_mod([hello])
        sb.install(mod, "R1", True, 15)
        c = FakeSock()
        mod.ws_recv(c)
        self.assertNotIn(b"video_ctrl", c.out)
        mod = self.make_mod([(0x80, 0x1, b'{"node_id":"R1"}')])
        phones = sb.install(mod, "R1", False, 15)
        phones.ping = False
        c = FakeSock()
        mod.ws_recv(c)
        self.assertEqual(c.out, b"")

    def test_uplink_drain(self):
        mod = self.make_mod([(0x80, 0x1, b'{"role":"sensor","node_id":"R1"}')])
        phones = sb.install(mod, "R1", True, 24)
        c = FakeSock()
        mod.ws_recv(c)  # hello: video_ctrl on and the first ping

        def pong(age_s):
            phones.pong(c, sb.PING_MAGIC + struct.pack(">d", time.monotonic() - age_s))

        c.out = b""
        pong(0.5)
        pong(0.6)
        self.assertNotIn(b'"stream_video": false', c.out)  # two slow pongs: not yet
        pong(0.7)
        self.assertIn(b'"stream_video": false', c.out)  # three over 400 ms: drain
        c.out = b""
        pong(0.3)
        self.assertNotIn(b"stream_video", c.out)  # still slow: stays paused
        pong(0.05)
        self.assertIn(b'"stream_video": true', c.out)  # caught up: resume
        c.out = b""
        for a in (0.5, 0.6, 0.7):
            pong(a)
        self.assertNotIn(b"stream_video", c.out)  # at most one drain per DRAIN_GAP_S
        self.assertEqual(phones.conns[id(c)]["drains"], 1)
        phones.drain = False
        phones.conns[id(c)]["drain_t"] = -1e9
        for a in (0.5, 0.6, 0.7):
            pong(a)
        self.assertNotIn(b"stream_video", c.out)  # --no-drain

    def test_ping_rtt_in_status(self):
        hello = (0x80, 0x1, b'{"role":"sensor","node_id":"R1"}')
        mod = self.make_mod([hello])
        phones = sb.install(mod, "R1", False, 15)
        c = FakeSock()
        mod.ws_recv(c)  # named phone: first ping goes out at once
        self.assertEqual(c.out[0], 0x89)
        payload = c.out[2:2 + c.out[1]]
        self.assertTrue(payload.startswith(sb.PING_MAGIC))
        phones.pong(c, payload)
        phones.pong(c, b"junk")  # not ours: ignored
        st = phones.conns[id(c)]
        self.assertEqual(len(st["rtt"]), 1)
        self.assertTrue(0 <= st["rtt"][0] < 1000)
        lines = []
        orig = sb.log
        sb.log = lambda *a: lines.append(" ".join(str(x) for x in a))
        try:
            phones.status(10)
        finally:
            sb.log = orig
        self.assertTrue(any("ws rtt p50" in ln for ln in lines), lines)
        n = len(c.out)
        phones.maybe_ping(c)  # within a second: no second ping
        self.assertEqual(len(c.out), n)

    def test_forwarder_replaced_and_fallback(self):
        mod = self.make_mod([])
        sb.install(mod, "R1", True, 15)
        fwd = mod.VideoForwarder("127.0.0.1", free_port())
        self.assertIsInstance(fwd, sb.SeekerVideoForwarder)
        fwd.close()
        self.assertIsNone(sb.install(types.SimpleNamespace(main=lambda: None), "R1", True, 15))

    def test_missing_ws_ingest(self):
        self.assertEqual(sb.main(["--ws-ingest", "/nonexistent/ws_ingest.py"]), 2)


LIVE = b'{"type":"phone_detections","node_id":"R1","pose":{"x_m":0.6,"y_m":-2.0,"yaw_deg":-15},' \
       b'"contacts":[]}'
NULL = b'{"type":"phone_detections","node_id":"R1","pose":null,"contacts":[]}'
NOKEY = b'{"type":"phone_detections","node_id":"R1","contacts":[]}'


class FakeUdp:
    def __init__(self):
        self.sent = []

    def sendto(self, b, dst):
        self.sent.append(json.loads(b.decode()))


def fake_ingest(frames):
    """Stand-in for remote/ws_ingest.py with its pose rule: a dict pose is kept, pose null leaves
    the last one, and every message is emitted with the kept pose."""
    mod = types.SimpleNamespace()
    it = iter(frames)

    def ws_recv(c):
        f = next(it, None)
        return (True, 0x8, b"") if f is None else (True, 0x1, f)

    class Bridge:
        def __init__(self):
            self.out = []

        def emit(self, node_id, pose, contacts, width, height, hfov_deg):
            self.out.append(pose)

        def ws_loop(self, c):
            pose = None
            while True:
                _, op, data = mod.ws_recv(c)
                if op == 0x8:
                    return
                m = json.loads(data.decode())
                if isinstance(m.get("pose"), dict):
                    pose = m["pose"]
                self.emit(m["node_id"], pose, m["contacts"], 640, 480, 66)

    mod.ws_recv, mod.main, mod.Bridge = ws_recv, (lambda: None), Bridge
    return mod


class TestPoseNull(unittest.TestCase):
    def test_null_reaches_fusion_and_is_logged(self):
        mod = fake_ingest([LIVE, NULL, NULL, LIVE, NOKEY])
        phones = sb.install(mod, "R1", False, 15)
        b = mod.Bridge()
        b.ws_loop(FakeSock())
        live = {"x_m": 0.6, "y_m": -2.0, "yaw_deg": -15}
        # NOKEY (no "pose" key at all) keeps ws_ingest's value.
        self.assertEqual(b.out, [live, None, None, live, live])
        self.assertEqual(phones.conns, {})  # the close frame ended the connection

    def test_keep_ingest_pose(self):
        mod = fake_ingest([LIVE, NULL])
        sb.install(mod, "R1", False, 15, pass_null_pose=False)
        b = mod.Bridge()
        b.ws_loop(FakeSock())
        self.assertEqual(b.out[1], {"x_m": 0.6, "y_m": -2.0, "yaw_deg": -15})

    def test_status_and_transition_lines(self):
        mod = fake_ingest([])
        phones = sb.install(mod, "R1", False, 15)
        lines = []
        old = sb.log
        sb.log = lambda *a: lines.append(" ".join(str(x) for x in a))
        try:
            c = FakeSock()
            phones.text(c, NULL)
            phones.status(10)
            phones.text(c, LIVE)
            phones.text(c, LIVE)
            phones.status(10)
            phones.text(c, NULL)
            phones.text(c, NULL)
            phones.status(10)
        finally:
            sb.log = old
        text = "\n".join(lines)
        self.assertIn("pose null (no tag fix yet)", text)
        self.assertEqual(text.count("pose R1: live 0.60,-2.00,-15.0"), 1)
        self.assertIn("pose live 0.60,-2.00,-15.0", text)
        self.assertEqual(text.count("pose R1: null (tag out of view?)"), 1)
        self.assertIn("POSE HELD", text)
        self.assertRegex(text, r"pose null \d+ s \(held at 0.60,-2.00,-15.0\)")

    def test_against_daniels_ws_ingest(self):
        path = os.environ.get("WS_INGEST") or os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "..", "..", "..", "..", "hackwestern-13",
            "remote", "ws_ingest.py")
        if not os.path.isfile(path):
            self.skipTest("no ws_ingest.py at %s (set WS_INGEST)" % path)
        mod = sb.load(path)
        frames = iter([LIVE, NULL, LIVE])

        def recv(c, it=frames):
            f = next(it, None)
            return (True, 0x8, b"") if f is None else (True, 0x1, f)

        mod.ws_recv = recv
        sb.install(mod, "R1", False, 15)
        b = mod.Bridge("127.0.0.1", 47103)
        b.udp.close()
        b.udp = FakeUdp()
        b.ws_loop(FakeSock())
        poses = [m["pose"] for m in b.udp.sent]
        self.assertEqual(poses[1], None)
        self.assertEqual(poses[0], poses[2])
        self.assertEqual(poses[0]["x_m"], 0.6)


if __name__ == "__main__":
    unittest.main(verbosity=1)
