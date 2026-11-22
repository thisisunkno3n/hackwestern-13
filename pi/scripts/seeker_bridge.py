#!/usr/bin/env python3
"""seeker_bridge.py: runs Daniel's remote/ws_ingest.py unchanged, plus fixes for live video and pose.

  python3 scripts/seeker_bridge.py [--ws-ingest PATH] [--no-video-ctrl] [--video-fps N]
                                   [--status-s S] [--keep-ingest-pose] [--no-ping] [--no-drain]
                                   [--idle-s S] [--pin NODE=x,y,yaw]... [ws_ingest.py flags...]

  e.g. python3 scripts/seeker_bridge.py --ws-ingest ~/seeker/remote/ws_ingest.py \\
           --port 8080 --tls-cert ~/cert.pem --tls-key ~/key.pem --view-node R1 --pin R2=0,2,180

Every flag that is not one of the nine above goes to ws_ingest.py as is (--port, --tls-cert,
--tls-key, --view-node, --video-host, --video-port, --fusion, ...).

1. video_ctrl: when the view node's page connects, the bridge sends it
   {"type": "video_ctrl", "v": 2, "stream_video": true, "fps": 15}. phone.html (24686f3 and later)
   then streams video even if its URL lost &video=1 (Safari autocomplete does that).
2. Video link at once: ws_ingest.VideoForwarder is replaced by one with the same interface that
   connects to hud_display (tcp/47106) at start, retries every second, and notices when
   hud_display goes away. hud_display can then tell "bridge connected, phone not sending" from
   "no bridge" on its banner.
3. Status: one line per phone every --status-s seconds (messages, video frames and fps, link,
   pose), and a warning when the view node sends no video or another node's video is dropped.
4. Link round trip: every second the bridge pings each phone over its WebSocket (Safari answers
   in its network stack, not in the page's JavaScript). The pong rides the uplink behind any
   video the phone has queued, so the status line's "ws rtt p50 X min Y ms" shows uplink
   queueing (p50 - min) and min/2 is about the base Wi-Fi hop for the latency budget
   (docs/LIVE-RUN.md "Latency"). --no-ping turns this off.
5. Uplink backlog drain: phone.html sends video with no backpressure and its detections ride
   the same WebSocket, so once the uplink falls behind (a Wi-Fi dip, a busier scene) frames
   queue on the phone and everything, HUD positions included, stays late and choppy. When the
   view node's pongs come back over DRAIN_RTT_MS three times in a row, the bridge pauses its
   video (video_ctrl stream_video false), lets the queue drain until a pong is back under
   RESUME_RTT_MS (at most DRAIN_MAX_S), then resumes. At most once per DRAIN_GAP_S. --no-drain
   turns it off.
6. Pose null passes through: ws_ingest.py keeps a phone's last pose and re-sends it when the
   phone sends pose null, so hud_fusion could not tell "tag out of view" from a live pose. The
   bridge forwards what the phone sent in that message (null stays null); hud_fusion still
   holds the last pose and the HUD says POSE HELD. Logged once per change ("pose R1: null ...").
   --keep-ingest-pose turns this off.
7. Link liveness and close reasons: ws_ingest.py reads with no timeout and the pings above only
   go out after the phone sends something, so when a phone vanished without a FIN or RST
   reaching the Pi (hotspot drop, phone locked) its socket stayed open forever and nothing was
   logged; the phone then reconnected next to it. Now a quiet link is pinged every second from
   its own reader thread (Safari answers in its network stack), a link with no data at all for
   --idle-s (15 s; hotspot RTT spikes are under 1 s) is closed, a new connection from the same
   node replaces an old one that has gone silent, a failed or stalled write closes the link
   (a half-written frame would corrupt the stream), and every close logs one line:
   "R2 closed: <reason> (up N s, ...)". A failed UDP send to hud_fusion drops that message,
   not the phone.
8. Pins (--pin NODE=x,y,yaw, same format as hud_fusion --pose): every message from NODE goes
   to hud_fusion with that pose, whatever the phone sends (null after a reconnect whose URL
   lost &pose=, or another pose). run-live.sh passes its PINS here as well as to hud_fusion.
9. Stray video: a phone other than the view node that sends video is asked to stop
   (video_ctrl stream_video false, again every OTHER_VIDEO_S while it keeps sending), so it
   does not spend the hotspot uplink on frames the bridge drops. Off with --no-video-ctrl.

If ws_ingest.py lacks the hooks this needs (ws_recv, VideoForwarder, Bridge.emit, main), it
runs without that part, with a warning.
"""
import argparse
import importlib.util
import json
import os
import select
import socket
import ssl
import struct
import sys
import threading
import time

VHEADER = struct.Struct(">4sBBH8sQHHII")  # IF-5.6 SKV2 header, 36 bytes big-endian
WS_TEXT, WS_BINARY, WS_CONT, WS_CLOSE, WS_PONG = 0x1, 0x2, 0x0, 0x8, 0xA
PING_MAGIC = b"SKP1"
PING_S = 1.0
DRAIN_RTT_MS = 400.0
RESUME_RTT_MS = 150.0
DRAIN_MAX_S = 2.5
DRAIN_GAP_S = 15.0
IDLE_S = 15.0
REPLACE_QUIET_S = 3.0  # an old link this quiet is a dead one; two talking links are two phones
OTHER_VIDEO_S = 5.0
EMIT_WARN_S = 10.0


def log(*a):
    print("[seeker_bridge]", *a, file=sys.stderr, flush=True)


def pose_str(p):
    try:
        s = "%.2f,%.2f,%.1f" % (p["x_m"], p["y_m"], p["yaw_deg"])
        return s + (" (tag %s)" % p["tag_id"] if p.get("tag_id") is not None else "")
    except (KeyError, TypeError, ValueError, AttributeError):
        return json.dumps(p)


def ws_ping_frame(t):
    """One unmasked server-to-client WebSocket ping carrying our monotonic send time."""
    data = PING_MAGIC + struct.pack(">d", t)
    return bytes([0x89, len(data)]) + data


def pct(v, q):
    v = sorted(v)
    return v[min(len(v) - 1, int(q * (len(v) - 1) + 0.5))] if v else None


def ws_text_frame(text):
    """One unmasked server-to-client WebSocket text frame."""
    data = text.encode()
    n = len(data)
    if n < 126:
        head = bytes([0x81, n])
    elif n < 65536:
        head = bytes([0x81, 126]) + struct.pack(">H", n)
    else:
        head = bytes([0x81, 127]) + struct.pack(">Q", n)
    return head + data


def parse_pin(s):
    """'R2=0,2,180' -> ('R2', {"x_m": 0.0, "y_m": 2.0, "yaw_deg": 180.0}); ValueError if bad."""
    node, sep, v = s.partition("=")
    xs = [float(x) for x in v.split(",")] if sep else []
    if not node or len(xs) != 3:
        raise ValueError("--pin %s: want NODE=x_m,y_m,yaw_deg" % s)
    return node, {"x_m": xs[0], "y_m": xs[1], "yaw_deg": xs[2]}


class LinkClosed(Exception):
    """Ends one phone connection; the message is the reason that gets logged. Not an OSError,
    so the per-send `except OSError` paths cannot swallow it."""


def close_reason(e):
    if isinstance(e, LinkClosed):
        return str(e)
    if isinstance(e, ssl.SSLError):
        return "TLS error: %s" % e
    if isinstance(e, socket.timeout):
        return "socket timed out: %s" % e
    if isinstance(e, ConnectionResetError):
        return "connection reset (ECONNRESET): the phone or the hotspot dropped the link"
    if isinstance(e, BrokenPipeError):
        return "broken pipe (EPIPE): the phone had already closed the link"
    if type(e) is ConnectionError:
        return ("the phone closed the TCP connection without a WebSocket close frame (page "
                "reloaded, tab killed, or the phone left Wi-Fi)")
    if isinstance(e, OSError):
        return "socket error: %s" % e
    return "bridge error in ws_ingest: %s: %s" % (type(e).__name__, e)


class Link:
    """What ws_ingest's ws_loop gets instead of the phone's socket (same recv and sendall), so
    every read and write of a phone connection goes through the bridge on that connection's
    own reader thread. While no byte arrives, recv wakes once a second (Phones.quiet: ping,
    replaced by a newer connection, idle_s reached); the socket timeout is a backstop for a
    read or write stuck inside one call. Write errors raise LinkClosed instead of being
    ignored."""

    TICK_S = 1.0

    def __init__(self, sock, phones, idle_s):
        self.sock, self.phones, self.idle_s = sock, phones, idle_s
        self.rx_t = time.monotonic()
        if idle_s > 0:
            try:
                sock.settimeout(idle_s)
            except (AttributeError, OSError):
                pass

    def __getattr__(self, name):
        return getattr(self.sock, name)

    def recv(self, n):
        while True:
            pending = getattr(self.sock, "pending", None)
            if pending is not None and pending():
                break  # TLS already holds decrypted bytes; select would not see them
            try:
                if select.select([self.sock], [], [], self.TICK_S)[0]:
                    break
            except (OSError, ValueError) as e:
                raise LinkClosed("socket error while waiting for data: %s" % e)
            self.phones.quiet(self)
        try:
            d = self.sock.recv(n)
        except socket.timeout:
            raise LinkClosed("read stalled for %.0f s in the middle of a frame" % self.idle_s)
        self.rx_t = time.monotonic()
        return d

    def sendall(self, b):
        try:
            self.sock.sendall(b)
        except socket.timeout:
            raise LinkClosed("send to the phone stalled for %.0f s (its receive window stayed "
                             "full)" % self.idle_s)
        except OSError as e:
            raise LinkClosed("send to the phone failed: %s" % close_reason(e))


def peer_str(c):
    try:
        p = c.getpeername()
    except (AttributeError, OSError):
        return None
    host = p[0][7:] if str(p[0]).startswith("::ffff:") else p[0]
    return "%s:%d" % (host, p[1])


class SeekerVideoForwarder:
    """Drop-in for ws_ingest.VideoForwarder: same constructor and submit(), same SKV2 framing
    (phone-built SKV2 frames pass through, bare JPEGs are wrapped with Pi monotonic ts_ms and a
    per-node seq), single-slot newest-frame mailbox. Connects at start instead of on the first
    frame, and reconnects when hud_display closes or restarts."""

    def __init__(self, host, port):
        self.dst = (host, port)
        self.cond = threading.Condition()
        self.pending = None
        self.seqs = {}
        self.sent = self.dropped = 0
        self.connected = False
        self.stop = False
        self.sock = None
        self._down_logged = False
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()

    def submit(self, node, payload, width, height):
        with self.cond:
            if self.pending is not None:
                self.dropped += 1
            self.pending = (node, payload, width, height)
            self.cond.notify()

    def close(self):
        with self.cond:
            self.stop = True
            self.cond.notify()
        self.thread.join(3)
        self._drop_link(None)

    def _frame(self, node, payload, width, height):
        if payload[:4] == b"SKV2":
            return payload
        seq = self.seqs.get(node, 0)
        self.seqs[node] = (seq + 1) & 0xFFFFFFFF

        def u16(v):
            try:
                return max(0, min(65535, int(v or 0)))
            except (TypeError, ValueError):
                return 0
        node_b = node.encode("ascii", "replace")[:8].ljust(8, b"\0")
        ts_ms = int(time.monotonic() * 1000)
        return VHEADER.pack(b"SKV2", 2, 0, 0, node_b, ts_ms, u16(width), u16(height), seq,
                            len(payload)) + payload

    def _connect(self):
        try:
            s = socket.create_connection(self.dst, timeout=2)
            s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except OSError as e:
            if not self._down_logged:
                log("video: hud_display not listening on %s:%d yet (%s); retrying every second"
                    % (self.dst + (e.strerror or e,)))
                self._down_logged = True
            return
        self.sock = s
        self.connected = True
        self._down_logged = False
        log("video: connected to hud_display at %s:%d" % self.dst)

    def _drop_link(self, why):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        if self.connected and why:
            log("video: link to hud_display lost (%s), reconnecting" % why)
        self.connected = False

    def _peer_closed(self):
        """hud_display never sends, so a readable socket means it closed (or reset)."""
        try:
            r, _, _ = select.select([self.sock], [], [], 0)
            if not r:
                return False
            return self.sock.recv(1) == b""
        except OSError:
            return True

    def _run(self):
        while True:
            with self.cond:
                if self.stop:
                    return
            if self.sock is None:
                self._connect()
                if self.sock is None:
                    with self.cond:
                        self.cond.wait(1.0)
                        self.pending = None  # never queue old frames behind a dead link
                    continue
            with self.cond:
                if self.pending is None and not self.stop:
                    self.cond.wait(0.5)
                item, self.pending = self.pending, None
            if item is None:
                if self._peer_closed():
                    self._drop_link("hud_display closed the connection")
                continue
            try:
                self.sock.sendall(self._frame(*item))
                self.sent += 1
            except (OSError, ValueError) as e:
                self._drop_link("send failed: %s" % e)


class Phones:
    """Per-connection bookkeeping fed from the ws_recv hook (each connection's own thread)."""

    def __init__(self, view_node, video_ctrl, fps, forwarder=None, port=8080):
        self.view_node = view_node
        self.port = port
        self.video_ctrl = video_ctrl
        self.fps = fps
        self.fwd = forwarder
        self.lock = threading.Lock()
        # Keyed by the connection object, not id(): an id can be reused by the next socket.
        self.conns = {}
        self.tls = threading.local()  # .pose: (phone message had "pose", its value), per thread
        self.pass_null = False  # install() set Bridge.emit to forward pose null
        self.ping = True
        self.drain = True
        self.idle_s = IDLE_S
        self.pins = {}  # node -> pose dict (--pin)
        self.emit_warn_t = {}

    def _st(self, c):
        st = self.conns.get(c)
        if st is None:
            now = time.monotonic()
            st = {"node": None, "t0": now, "rx_t": now, "peer": peer_str(c), "msgs": 0,
                  "frames": 0, "frames_t": 0, "fps": 0.0, "ctrl_t": None, "warned": False,
                  "other_warned": False, "stop_t": None, "stops": 0, "pose": None,
                  "pose_live": None, "pose_t": None, "pin_logged": False, "dup_warned": False,
                  "ping_t": 0.0, "rtt": [], "rtt_recent": [], "paused_t": None,
                  "drain_t": -1e9, "drains": 0}
            self.conns[c] = st
        return st

    def opened(self, c):
        with self.lock:
            self._st(c)

    def rx(self, c):
        with self.lock:
            self._st(c)["rx_t"] = time.monotonic()

    def _newer(self, c, st):
        """Under self.lock: the state of a newer connection that says it is the same node."""
        for oc, ost in self.conns.items():
            if oc is not c and st["node"] and ost["node"] == st["node"] and ost["t0"] > st["t0"]:
                return ost
        return None

    def text(self, c, data):
        self.tls.pose = (False, None)
        try:
            m = json.loads(data.decode())
        except (ValueError, UnicodeDecodeError):
            return
        if not isinstance(m, dict):
            return
        if "pose" in m:
            self.tls.pose = (True, m["pose"] if isinstance(m["pose"], dict) else None)
        send_ctrl = False
        with self.lock:
            st = self._st(c)
            node = m.get("node_id")
            if node and node != st["node"]:
                st["node"] = node
                log("phone %s connected%s%s" % (
                    node, " from " + st["peer"] if st["peer"] else "",
                    " (view node: its video goes to the HUD)" if node == self.view_node else ""))
            if "contacts" in m or "bearing_deg" in m:
                st["msgs"] += 1
            if "pose" in m and st["node"] in self.pins:
                if not st["pin_logged"]:
                    st["pin_logged"] = True
                    sent = self.tls.pose[1]
                    log("pose %s: pinned on the Pi at %s (--pin); the phone sent %s, ignored"
                        % (st["node"], pose_str(self.pins[st["node"]]),
                           "null" if sent is None else pose_str(sent)))
            elif "pose" in m and st["node"]:
                self._pose(st, self.tls.pose[1])
            if (self.video_ctrl and st["node"] == self.view_node and st["ctrl_t"] is None):
                st["ctrl_t"] = time.monotonic()
                send_ctrl = True
        if send_ctrl:
            msg = {"type": "video_ctrl", "v": 2, "stream_video": True, "fps": self.fps}
            try:
                c.sendall(ws_text_frame(json.dumps(msg)))
                log("video: asked %s to stream video (video_ctrl, %d fps)" % (self.view_node,
                                                                              self.fps))
            except OSError as e:
                log("video: could not send video_ctrl to %s: %s" % (self.view_node, e))

    def _pose(self, st, pose):
        live = pose is not None
        if live:
            st["pose"], st["pose_t"] = pose, time.monotonic()
        if live == st["pose_live"]:
            return
        if live:
            log("pose %s: live %s" % (st["node"], pose_str(pose)))
        elif st["pose_live"]:
            log("pose %s: null (tag out of view?); %s" % (
                st["node"], ("hud_fusion holds %s, HUD says POSE HELD" if self.pass_null else
                             "ws_ingest keeps sending %s as live (--keep-ingest-pose)")
                % pose_str(st["pose"])))
        st["pose_live"] = live

    def binary(self, c):
        stop = False
        with self.lock:
            st = self._st(c)
            st["frames"] += 1
            node = st["node"]
            if st["frames"] == 1:
                log("video: first frame from %s" % (node or "an unnamed phone"))
            now = time.monotonic()
            if (node and node != self.view_node and self.video_ctrl and
                    (st["stop_t"] is None or now - st["stop_t"] >= OTHER_VIDEO_S)):
                st["stop_t"] = now
                st["stops"] += 1
                stop = True
            if node != self.view_node and not st["other_warned"]:
                st["other_warned"] = True
                log("video: %s sends video but only %s (--view-node) goes to the HUD; dropped%s"
                    % (node, self.view_node, ", asking %s to stop (video_ctrl stream_video "
                       "false)" % node if stop else ""))
            elif stop and st["stops"] == 2:
                log("video: %s still sends video after a stop request; asking again every %.0f s "
                    "(its phone.html may ignore video_ctrl)" % (node, OTHER_VIDEO_S))
        if stop:
            self._video_ctrl(c, False)

    def _video_ctrl(self, c, on):
        msg = {"type": "video_ctrl", "v": 2, "stream_video": on, "fps": self.fps}
        try:
            c.sendall(ws_text_frame(json.dumps(msg)))
        except OSError:
            pass

    def maybe_ping(self, c):
        """Called on the connection's own reader thread, the only one that writes to it."""
        if not self.ping:
            return
        now = time.monotonic()
        resume = False
        with self.lock:
            st = self._st(c)
            if st["paused_t"] is not None and now - st["paused_t"] > DRAIN_MAX_S:
                st["paused_t"] = None
                resume = True
        if resume:
            self._video_ctrl(c, True)
            log("video: %s video resumed after the drain (time limit)" % self.view_node)
        with self.lock:
            st = self._st(c)
            if not st["node"] or now - st["ping_t"] < PING_S:
                return
            st["ping_t"] = now
        try:
            c.sendall(ws_ping_frame(now))
        except OSError:
            pass

    def pong(self, c, data):
        if len(data) != 12 or data[:4] != PING_MAGIC:
            return
        now = time.monotonic()
        rtt = (now - struct.unpack(">d", data[4:])[0]) * 1000.0
        if not 0 <= rtt < 60000:
            return
        act = None
        with self.lock:
            st = self._st(c)
            st["rtt"].append(rtt)
            st["rtt_recent"] = (st["rtt_recent"] + [rtt])[-3:]
            if not (self.drain and self.video_ctrl and st["node"] == self.view_node):
                return
            if st["paused_t"] is not None:
                if rtt < RESUME_RTT_MS:
                    act = ("resume", now - st["paused_t"])
                    st["paused_t"] = None
            elif (len(st["rtt_recent"]) == 3 and min(st["rtt_recent"]) > DRAIN_RTT_MS and
                  now - st["drain_t"] > DRAIN_GAP_S):
                st["paused_t"] = st["drain_t"] = now
                st["drains"] += 1
                act = ("pause", min(st["rtt_recent"]))
        if act and act[0] == "pause":
            self._video_ctrl(c, False)
            log("video: %s uplink backlog (ws rtt over %.0f ms three times, %.0f ms): pausing its "
                "video to let the queue drain" % (self.view_node, DRAIN_RTT_MS, act[1]))
        elif act:
            self._video_ctrl(c, True)
            log("video: %s video resumed after a %.1f s drain (ws rtt %.0f ms)" % (self.view_node, act[1], rtt))

    def quiet(self, link):
        """A second with no byte from this phone (its reader thread, inside Link.recv)."""
        now = time.monotonic()
        silent = now - link.rx_t
        with self.lock:
            st = self.conns.get(link)
            newer = self._newer(link, st) if st else None
            node = st["node"] if st else None
        if newer is not None and silent >= REPLACE_QUIET_S:
            raise LinkClosed("replaced by a newer %s connection%s; this one had no data for %.0f s "
                             "(half-open: the phone reconnected and this socket never closed)"
                             % (node, " from " + newer["peer"] if newer["peer"] else "", silent))
        if link.idle_s > 0 and silent >= link.idle_s:
            raise LinkClosed("no data from the phone for %.0f s, pings unanswered (Wi-Fi or "
                             "hotspot dropped, phone locked, or Safari in the background)"
                             % link.idle_s)
        self.maybe_ping(link)

    def closed(self, c, reason="connection ended"):
        """Logs one line per connection, whichever path ends it first; later calls do nothing."""
        with self.lock:
            st = self.conns.pop(c, None)
        if st is None:
            return
        who = st["node"] or "an unnamed phone"
        log("%s closed: %s (up %.0f s, %d msgs, %d video frames%s)"
            % (who, reason, time.monotonic() - st["t0"], st["msgs"], st["frames"],
               ", from " + st["peer"] if st["peer"] else ""))

    def emit_failed(self, node, e):
        now = time.monotonic()
        with self.lock:
            if now - self.emit_warn_t.get(node, -1e9) < EMIT_WARN_S:
                return
            self.emit_warn_t[node] = now
        log("WARNING remote_contacts from %s not sent to hud_fusion (%s); that message is "
            "dropped, the phone link stays up" % (node, e))

    def status(self, period):
        now = time.monotonic()
        lines = []
        with self.lock:
            for c, st in self.conns.items():
                if not st["node"]:
                    continue
                dt = now - (st["frames_t"] or st["t0"])
                fps = (st["frames"] - st.get("frames_prev", 0)) / dt if dt > 0 else 0.0
                st["frames_prev"], st["frames_t"] = st["frames"], now
                line = "%s up %.0f s, %d msgs, video %d frames %.1f fps" % (
                    st["node"], now - st["t0"], st["msgs"], st["frames"], fps)
                if st["rtt"]:
                    line += ", ws rtt p50 %.0f min %.0f max %.0f ms" % (
                        pct(st["rtt"], 0.5), min(st["rtt"]), max(st["rtt"]))
                    st["rtt"] = []
                if st["drains"]:
                    line += ", %d uplink drain%s%s" % (st["drains"], "s" if st["drains"] > 1 else "",
                                                       " (video paused now)" if st["paused_t"] else "")
                twin = self._newer(c, st)
                if (twin is not None and not st["dup_warned"] and st["rx_t"] > twin["t0"] + 1.0
                        and now - st["rx_t"] < REPLACE_QUIET_S):
                    st["dup_warned"] = True
                    lines.append("WARNING two live connections say they are %s (%s and %s): two "
                                 "phones with the same ?node=? Both kept"
                                 % (st["node"], st["peer"], twin["peer"]))
                if now - st["rx_t"] >= 2 * PING_S + 1:
                    line += ", no data for %.0f s" % (now - st["rx_t"])
                if st["stops"]:
                    line += ", video stop asked %d time%s" % (st["stops"],
                                                              "s" if st["stops"] > 1 else "")
                if st["node"] in self.pins:
                    line += ", pose pinned %s (on the Pi)" % pose_str(self.pins[st["node"]])
                elif st["pose_live"]:
                    line += ", pose live %s" % pose_str(st["pose"])
                elif st["pose"] is not None:
                    line += ", pose null %.0f s (held at %s)" % (now - st["pose_t"],
                                                                 pose_str(st["pose"]))
                else:
                    line += ", pose null (no tag fix yet)"
                if st["node"] == self.view_node:
                    line += ", HUD link %s" % ("up" if self.fwd is None or self.fwd.connected
                                               else "DOWN")
                    since = st["ctrl_t"] if st["ctrl_t"] is not None else st["t0"]
                    if st["frames"] == 0 and now - since > 10 and not st["warned"]:
                        st["warned"] = True
                        lines.append(
                            "WARNING %s is connected but sends no video. Reload its page with "
                            "&video=1; if that does not help, its phone.html is older than "
                            "24686f3 (no video capture)" % st["node"])
                if st["peer"]:
                    line += ", from " + st["peer"]
                lines.append(line)
        if not lines:
            lines.append("no phones connected (open https://qnxpi-felix.local:%d/?node=...)"
                         % self.port)
        for ln in lines:
            log("status: " + ln)


def install(mod, view_node, video_ctrl, fps, port=8080, pass_null_pose=True):
    """Patches a loaded ws_ingest module in place. Returns the Phones tracker, or None."""
    if not hasattr(mod, "ws_recv") or not hasattr(mod, "main"):
        log("WARNING %s has no ws_recv/main; running it unchanged (no video_ctrl, no status)"
            % getattr(mod, "__file__", "ws_ingest"))
        return None
    fwd_holder = {}
    if hasattr(mod, "VideoForwarder"):
        def make_forwarder(host, port):
            fwd_holder["fwd"] = SeekerVideoForwarder(host, port)
            return fwd_holder["fwd"]
        mod.VideoForwarder = make_forwarder
    else:
        log("WARNING ws_ingest has no VideoForwarder; video link left as it is")
    phones = Phones(view_node, video_ctrl, fps, port=port)
    phones.fwd_holder = fwd_holder
    bridge = getattr(mod, "Bridge", None)
    if bridge is not None and hasattr(bridge, "emit"):
        orig_emit = bridge.emit

        def emit(self, node_id, pose, *a, **k):
            pin = phones.pins.get(node_id)
            if pin is not None:
                pose = dict(pin)
            elif phones.pass_null:
                had, raw = getattr(phones.tls, "pose", (False, None))
                pose = raw if had else pose
            try:
                return orig_emit(self, node_id, pose, *a, **k)
            except OSError as e:
                phones.emit_failed(node_id, e)
                return None

        bridge.emit = emit
        phones.pass_null = pass_null_pose
    else:
        log("WARNING ws_ingest has no Bridge.emit; pose null is not passed through (the HUD "
            "cannot show POSE HELD) and --pin does nothing")
    if bridge is not None and hasattr(bridge, "ws_loop"):
        orig_loop = bridge.ws_loop

        def ws_loop(self, c):
            link = Link(c, phones, phones.idle_s)
            phones.opened(link)
            try:
                return orig_loop(self, link)
            except Exception as e:
                phones.closed(link, close_reason(e))
                raise  # ws_ingest's handle() closes the socket
            finally:
                phones.closed(link, "ws_ingest left its loop")

        bridge.ws_loop = ws_loop
    else:
        log("WARNING ws_ingest has no Bridge.ws_loop; no idle timeout, a dead phone link stays "
            "open")
    orig = mod.ws_recv

    def ws_recv(c):
        try:
            fin, op, data = orig(c)
        except Exception as e:
            phones.closed(c, close_reason(e))
            raise
        phones.rx(c)
        if op == WS_TEXT:
            phones.text(c, data)
        elif op == WS_BINARY:
            phones.binary(c)
        elif op == WS_PONG:
            phones.pong(c, data)
        elif op == WS_CLOSE:
            code = struct.unpack(">H", data[:2])[0] if len(data) >= 2 else None
            phones.closed(c, "the phone closed the WebSocket (close frame%s)"
                          % (", code %d" % code if code is not None else ""))
        if op != WS_CLOSE:
            phones.maybe_ping(c)
        return fin, op, data

    mod.ws_recv = ws_recv
    return phones


def load(path):
    spec = importlib.util.spec_from_file_location("ws_ingest", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    ap = argparse.ArgumentParser(add_help=False)
    ap.add_argument("--ws-ingest", default=os.path.expanduser("~/seeker/remote/ws_ingest.py"))
    ap.add_argument("--no-video-ctrl", action="store_true")
    ap.add_argument("--video-fps", type=int, default=15)
    ap.add_argument("--status-s", type=float, default=10.0)
    ap.add_argument("--keep-ingest-pose", action="store_true")
    ap.add_argument("--no-ping", action="store_true")
    ap.add_argument("--no-drain", action="store_true")
    ap.add_argument("--idle-s", type=float, default=IDLE_S)
    ap.add_argument("--pin", action="append", default=[])
    ours, rest = ap.parse_known_args(argv)
    try:
        pins = dict(parse_pin(p) for p in ours.pin)
    except ValueError as e:
        log(str(e))
        return 2
    theirs = argparse.ArgumentParser(add_help=False)
    theirs.add_argument("--view-node", default="R1")
    theirs.add_argument("--port", type=int, default=8080)
    targs, _ = theirs.parse_known_args(rest)
    if not os.path.isfile(ours.ws_ingest):
        log("cannot find ws_ingest.py at %s (use --ws-ingest PATH)" % ours.ws_ingest)
        return 2
    mod = load(ours.ws_ingest)
    phones = install(mod, targs.view_node, not ours.no_video_ctrl, ours.video_fps, targs.port,
                     not ours.keep_ingest_pose)
    if phones is not None:
        phones.ping = not ours.no_ping
        phones.drain = not ours.no_drain
        phones.idle_s = ours.idle_s
        phones.pins = pins
    log("running %s %s (view node %s, video_ctrl %s, pose null %s, idle close %s, pins %s)"
        % (ours.ws_ingest, " ".join(rest), targs.view_node,
           "off" if ours.no_video_ctrl else "on",
           "as ws_ingest (last pose)" if ours.keep_ingest_pose else "passed to hud_fusion",
           "%.0f s" % ours.idle_s if ours.idle_s > 0 else "off",
           " ".join("%s=%s" % (n, pose_str(p)) for n, p in sorted(pins.items())) or "none"))
    if phones is not None and ours.status_s > 0:
        def status_loop():
            while True:
                time.sleep(ours.status_s)
                phones.fwd = phones.fwd_holder.get("fwd")
                phones.status(ours.status_s)
        threading.Thread(target=status_loop, daemon=True).start()
    sys.argv = [ours.ws_ingest] + rest
    try:
        mod.main()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
