#!/usr/bin/env python3
"""Host tests for scripts/seeker_bridge.py (no phone, no browser): SKV2 framing, eager connect
and reconnect of the video link, video_ctrl to the view node only, and the unchanged fallback.
Run: python3 tests/test_seeker_bridge.py"""
import os
import select
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


def ws_ingest_path():
    """$WS_INGEST, else remote/ws_ingest.py of this repo (pi/tests -> ../../remote) or of the
    Pi's checkout (~/seeker/drafts/pi-lane/tests -> ~/seeker/remote)."""
    here = os.path.dirname(os.path.abspath(__file__))
    for p in (os.environ.get("WS_INGEST"),
              os.path.join(here, "..", "..", "remote", "ws_ingest.py"),
              os.path.join(here, "..", "..", "..", "remote", "ws_ingest.py")):
        if p and os.path.isfile(p):
            return p
    return None


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
        st = phones.conns[c]
        self.assertEqual((st["node"], st["msgs"], st["frames"]), ("R1", 1, 1))
        with self.assertRaises(ConnectionError):
            mod.ws_recv(c)
        self.assertNotIn(c, phones.conns)

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
        self.assertEqual(phones.conns[c]["drains"], 1)
        phones.drain = False
        phones.conns[c]["drain_t"] = -1e9
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
        st = phones.conns[c]
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
        path = ws_ingest_path()
        if path is None:
            self.skipTest("no ws_ingest.py (set WS_INGEST)")
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


HELLO_R2 = b'{"role":"sensor","node_id":"R2","pose":null}'
DET_R2 = b'{"type":"phone_detections","v":2,"node_id":"R2","pose":null,"width":640,' \
         b'"height":480,"hfov_deg":66,"contacts":[{"cls":"person","conf":0.9,"bearing_deg":5.0,' \
         b'"range_m":2.1,"box":[1,2,3,4]}]}'


def ws_client_frame(op, data):
    mask = os.urandom(4)
    n = len(data)
    head = bytes([0x80 | op])
    if n < 126:
        head += bytes([0x80 | n])
    elif n < 65536:
        head += bytes([0x80 | 126]) + struct.pack(">H", n)
    else:
        head += bytes([0x80 | 127]) + struct.pack(">Q", n)
    return head + mask + bytes(b ^ mask[i & 3] for i, b in enumerate(data))


class Phone:
    """A phone.html stand-in on a real socket: masked frames out, server frames parsed, pings
    answered (after pong_delay_s, like a hotspot RTT spike) only while pump() runs."""

    def __init__(self, port, tls=False):
        s = socket.create_connection(("127.0.0.1", port), timeout=5)
        if tls:
            import ssl
            ctx = ssl.create_default_context()
            ctx.check_hostname = False
            ctx.verify_mode = ssl.CERT_NONE
            s = ctx.wrap_socket(s, server_hostname="127.0.0.1")
        s.sendall(b"GET /ws HTTP/1.1\r\nHost: x\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                  b"Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\nSec-WebSocket-Version: 13\r\n\r\n")
        resp = b""
        while b"\r\n\r\n" not in resp:
            d = s.recv(1024)
            if not d:
                raise ConnectionError("handshake closed")
            resp += d
        assert b" 101 " in resp.split(b"\r\n", 1)[0], resp
        self.s, self.buf, self.frames, self.eof = s, resp.split(b"\r\n\r\n", 1)[1], [], False
        self.pongs = []

    def send(self, op, data):
        self.s.sendall(ws_client_frame(op, data))

    def _read(self, wait):
        pend = getattr(self.s, "pending", None)
        if not (pend and pend()) and not select.select([self.s], [], [], wait)[0]:
            return
        try:
            d = self.s.recv(65536)
        except (ConnectionError, OSError):
            d = b""
        if not d:
            self.eof = True
            return
        self.buf += d
        while len(self.buf) >= 2:
            op, n, off = self.buf[0] & 0x0F, self.buf[1] & 0x7F, 2
            if n == 126:
                if len(self.buf) < 4:
                    break
                n, off = struct.unpack(">H", self.buf[2:4])[0], 4
            if len(self.buf) < off + n:
                break
            data, self.buf = self.buf[off:off + n], self.buf[off + n:]
            self.frames.append((op, data))
            if op == 0x9:
                self.pongs.append(data)

    def pump(self, secs, pong_delay_s=0.0, det_every_s=None):
        """Reads for secs; answers pings pong_delay_s after they came; sends DET_R2 every
        det_every_s if set. Returns False once the bridge closed the socket."""
        end = time.monotonic() + secs
        due, next_det = [], time.monotonic()
        while time.monotonic() < end and not self.eof:
            self._read(0.02)
            now = time.monotonic()
            due += [(now + pong_delay_s, p) for p in self.pongs]
            self.pongs = []
            try:
                for t, p in [x for x in due if x[0] <= now]:
                    self.send(0xA, p)
                due = [x for x in due if x[0] > now]
                if det_every_s and now >= next_det:
                    next_det = now + det_every_s
                    self.send(0x1, DET_R2)
            except OSError:
                self.eof = True
        return not self.eof

    def texts(self):
        return [json.loads(d.decode()) for op, d in self.frames if op == 0x1]

    def close(self):
        try:
            self.s.close()
        except OSError:
            pass


class TestLiveLink(unittest.TestCase):
    """The real bridge (scripts/seeker_bridge.py running remote/ws_ingest.py) on real sockets:
    the R2 disconnects from the live run's bridge.log."""

    def start(self, *extra, tls=False):
        path = ws_ingest_path()
        if path is None:
            self.skipTest("no ws_ingest.py (set WS_INGEST)")
        import subprocess
        import tempfile
        self.tmp = tempfile.mkdtemp()
        self.port = free_port()
        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("127.0.0.1", 0))
        self.udp.settimeout(3)
        self.logf = os.path.join(self.tmp, "bridge.log")
        args = [sys.executable, "-u", os.path.join(os.path.dirname(sb.__file__), "seeker_bridge.py"),
                "--ws-ingest", path, "--port", str(self.port), "--fusion", "127.0.0.1",
                "--fusion-port", str(self.udp.getsockname()[1]), "--video-port", str(free_port()),
                "--view-node", "R1", "--status-s", "1"] + list(extra)
        if tls:
            import shutil
            if not shutil.which("openssl"):
                self.skipTest("no openssl to make a test certificate")
            cert, key = os.path.join(self.tmp, "c.pem"), os.path.join(self.tmp, "k.pem")
            subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days",
                            "1", "-subj", "/CN=127.0.0.1", "-keyout", key, "-out", cert],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
            args += ["--tls-cert", cert, "--tls-key", key]
        self.out = open(self.logf, "w")
        self.proc = subprocess.Popen(args, stdout=self.out, stderr=subprocess.STDOUT)
        self.addCleanup(self.stop)
        self.assertTrue(wait_for(lambda: "phones ->" in self.log(), 10), self.log())
        self.phones = []

    def stop(self):
        for p in getattr(self, "phones", []):
            p.close()
        self.proc.kill()
        self.proc.wait()
        self.out.close()
        self.udp.close()

    def log(self):
        with open(self.logf) as f:
            return f.read()

    def phone(self, tls=False):
        p = Phone(self.port, tls)
        self.phones.append(p)
        return p

    def closes(self, node="R2"):
        return [ln for ln in self.log().splitlines() if "] %s closed: " % node in ln]

    def half_open(self, tls):
        self.start("--idle-s", "2", tls=tls)
        p = self.phone(tls)
        p.send(0x1, HELLO_R2)
        p.pump(0.5, det_every_s=0.1)
        # The phone is gone (hotspot drop, phone locked) but no FIN or RST reaches the Pi.
        self.assertEqual(self.closes(), [])
        time.sleep(3.5)
        self.assertTrue(wait_for(lambda: self.closes(), 3), self.log())
        self.assertIn("no data from the phone for 2 s", self.closes()[0])
        self.assertIn("from 127.0.0.1:", self.closes()[0])
        self.assertFalse(p.pump(1.0))  # the bridge really closed the socket
        self.assertEqual(len(self.closes()), 1)

    def test_half_open_phone_is_closed_and_logged(self):
        self.half_open(False)

    def test_half_open_phone_is_closed_and_logged_tls(self):
        self.half_open(True)

    def test_reconnect_replaces_the_silent_old_link(self):
        self.start("--idle-s", "60")
        old = self.phone()
        old.send(0x1, HELLO_R2)
        old.pump(0.3, det_every_s=0.1)
        new = self.phone()  # Safari reconnects while the old socket is still open on the Pi
        new.send(0x1, HELLO_R2)
        self.assertTrue(new.pump(5.0, det_every_s=0.1))
        self.assertEqual(len(self.closes()), 1, self.log())
        self.assertIn("replaced by a newer R2 connection from 127.0.0.1:", self.closes()[0])
        self.assertFalse(old.pump(0.5))
        self.assertTrue(new.pump(1.5, det_every_s=0.1))
        self.assertEqual(len(self.closes()), 1)
        self.assertEqual(self.log().count("phone R2 connected from 127.0.0.1:"), 2)

    def test_hotspot_rtt_spikes_do_not_close_or_pause_r2(self):
        self.start("--idle-s", "3")
        p = self.phone()
        p.send(0x1, HELLO_R2)
        # No detections, only pongs that come back 700 ms late (the live run saw ~680 ms):
        # over DRAIN_RTT_MS every time, and the only data that keeps the link alive.
        self.assertTrue(p.pump(7.0, pong_delay_s=0.7))
        self.assertEqual(self.closes(), [], self.log())
        self.assertEqual([m for m in p.texts() if m.get("type") == "video_ctrl"], [])
        self.assertRegex(self.log(), r"status: R2 up \d+ s, .*ws rtt p50 [7-9]\d\d")
        self.assertNotIn("uplink backlog", self.log())

    def test_pin_survives_null_pose_and_stray_video_is_stopped(self):
        self.start("--pin", "R2=-1.5,-2,90")
        p = self.phone()
        p.send(0x1, HELLO_R2)
        p.send(0x1, DET_R2)
        got = json.loads(self.udp.recv(9000).decode())
        self.assertEqual(got["node_id"], "R2")
        self.assertEqual(got["pose"], {"x_m": -1.5, "y_m": -2.0, "yaw_deg": 90.0})
        hdr = HDR.pack(b"SKV2", 2, 0, 0, b"R2".ljust(8, b"\0"), 1, 32, 24, 0, 4)
        p.send(0x2, hdr + b"abcd")
        p.pump(1.0, det_every_s=0.2)
        ctrl = [m for m in p.texts() if m.get("type") == "video_ctrl"]
        self.assertEqual(len(ctrl), 1, p.texts())
        self.assertIs(ctrl[0]["stream_video"], False)
        p.send(0x2, hdr + b"abcd")  # within OTHER_VIDEO_S: not asked again
        p.pump(0.5)
        self.assertEqual(len([m for m in p.texts() if m.get("type") == "video_ctrl"]), 1)
        text = self.log()
        self.assertIn("pose R2: pinned on the Pi at -1.50,-2.00,90.0 (--pin); the phone sent null",
                      text)
        self.assertIn("asking R2 to stop (video_ctrl stream_video false)", text)
        self.assertTrue(wait_for(lambda: "pose pinned -1.50,-2.00,90.0 (on the Pi)" in self.log()))

    def test_close_frame_and_fin_reasons(self):
        self.start()
        p = self.phone()
        p.send(0x1, HELLO_R2)
        p.send(0x8, struct.pack(">H", 1001))
        self.assertTrue(wait_for(lambda: self.closes()), self.log())
        self.assertIn("the phone closed the WebSocket (close frame, code 1001)", self.closes()[0])
        q = self.phone()
        q.send(0x1, HELLO_R2)
        q.pump(0.3)
        q.close()
        self.assertTrue(wait_for(lambda: len(self.closes()) == 2), self.log())
        self.assertIn("without a WebSocket close frame", self.closes()[1])


class TestClosePaths(unittest.TestCase):
    def capture(self):
        lines = []
        orig = sb.log
        sb.log = lambda *a: lines.append(" ".join(str(x) for x in a))
        self.addCleanup(setattr, sb, "log", orig)
        return lines

    def test_bridge_error_is_logged_not_silent(self):
        lines = self.capture()
        mod = fake_ingest([LIVE, b"[1]"])  # a non-object message makes the fake loop raise
        phones = sb.install(mod, "R1", False, 15)
        with self.assertRaises(Exception):
            mod.Bridge().ws_loop(FakeSock())
        self.assertEqual(phones.conns, {})
        self.assertTrue(any(ln.startswith("R1 closed: bridge error in ws_ingest: AttributeError")
                            for ln in lines), lines)

    def test_udp_send_error_drops_the_message_not_the_phone(self):
        lines = self.capture()
        mod = fake_ingest([LIVE, LIVE, LIVE])
        calls = []

        def emit(self, node_id, pose, contacts, width, height, hfov_deg):
            calls.append(pose)
            raise OSError(55, "No buffer space available")
        mod.Bridge.emit = emit
        phones = sb.install(mod, "R1", False, 15)
        mod.Bridge().ws_loop(FakeSock())
        self.assertEqual(len(calls), 3)  # every message tried, the loop went on
        self.assertEqual(sum("not sent to hud_fusion" in ln for ln in lines), 1)  # rate-limited
        self.assertEqual(phones.conns, {})
        self.assertTrue(any("R1 closed: the phone closed the WebSocket" in ln for ln in lines))

    def test_failed_ping_closes_with_a_reason(self):
        class DeadSock(FakeSock):
            def sendall(self, b):
                raise BrokenPipeError(32, "Broken pipe")
        phones = sb.Phones("R1", True, 15)
        link = sb.Link(DeadSock(), phones, 0)
        phones.text(link, b'{"node_id":"R2"}')
        with self.assertRaises(sb.LinkClosed) as cm:
            phones.maybe_ping(link)
        self.assertIn("send to the phone failed: broken pipe", str(cm.exception))

    def test_drain_never_targets_r2(self):
        phones = sb.Phones("R1", True, 15)
        c = FakeSock()
        phones.text(c, b'{"node_id":"R2"}')
        for _ in range(6):
            phones.pong(c, sb.PING_MAGIC + struct.pack(">d", time.monotonic() - 0.9))
        phones.maybe_ping(c)
        self.assertNotIn(b"video_ctrl", c.out)
        self.assertIsNone(phones.conns[c]["paused_t"])

    def test_two_phones_warning_only_when_both_talk(self):
        lines = self.capture()
        phones = sb.Phones("R1", False, 15)
        old, new = FakeSock(), FakeSock()
        phones.text(old, b'{"node_id":"R2"}')
        phones.conns[old]["t0"] -= 10
        phones.conns[old]["rx_t"] -= 2  # last data before the reconnect: a dead link
        phones.text(new, b'{"node_id":"R2"}')
        phones.status(10)
        self.assertFalse(any("two live connections" in ln for ln in lines), lines)
        phones.conns[new]["t0"] -= 2
        phones.rx(old)  # still talking after the newer one connected: a second phone
        phones.status(10)
        self.assertEqual(sum("two live connections say they are R2" in ln for ln in lines), 1)

    def test_pin_parse(self):
        self.assertEqual(sb.parse_pin("R2=0,2,180"), ("R2", {"x_m": 0.0, "y_m": 2.0,
                                                             "yaw_deg": 180.0}))
        for bad in ("R2", "R2=1,2", "=1,2,3", "R2=a,b,c"):
            with self.assertRaises(ValueError):
                sb.parse_pin(bad)
        self.assertEqual(sb.main(["--pin", "R2=1,2", "--ws-ingest", __file__]), 2)


if __name__ == "__main__":
    unittest.main(verbosity=1)
