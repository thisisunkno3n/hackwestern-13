#!/usr/bin/env python3
"""Test sender for IF-5.6 video_frame: streams SKV2-framed JPEGs to hud_display on TCP 47106.

Stands in for remote/ws_ingest.py's video forwarder (Daniel's lane) so the Pi lane can test the
receiver without phones. Pure stdlib, Python 3.8+ (the Pi has 3.14).

  python3 scripts/video_send.py                                # test card, 640x480 @ 15 fps
  python3 scripts/video_send.py --host qnxpi-felix.local --size 1280x720
  python3 scripts/video_send.py --host 'fe80::1234:56ff:fe78:9abc%en0'   # IPv6 link-local
  python3 scripts/video_send.py --jpeg a.jpg b.jpg             # loop real JPEG files
  python3 scripts/video_send.py --dir frames/ --fps 30
  python3 scripts/video_send.py --write card.jpg               # write one test card and exit

Each frame is the 36-byte big-endian header from INTERFACES.md IF-5.6 then the JPEG:
  magic "SKV2", v 2, flags 0, reserved 0, node_id (8 bytes, NUL padded), ts_ms (u64, this
  machine's monotonic ms), width (u16), height (u16), seq (u32), payload_len (u32).

The test card is drawn in flat 8x8 blocks so the built-in baseline JPEG writer below needs no
DCT: red top left, green top right, blue bottom left, white bottom right (check display.swap_rb
if red and blue trade places), an orange bar sweeping left to right, and a frame counter.
"""
import argparse
import glob
import os
import signal
import socket
import struct
import sys
import time

MAGIC = b"SKV2"
VERSION = 2
HEADER = struct.Struct(">4sBBH8sQHHII")  # 36 bytes
MAX_PAYLOAD = 2097152

# ---- minimal baseline JPEG writer for images made of flat 8x8 blocks ----
# With every AC coefficient zero, a block is just its DC term: DC = 8 * (value - 128) with a
# quantiser of 1. Standard luminance DC Huffman table (ITU T.81 Annex K.3); the AC table holds
# only EOB (code "0").
DC_BITS = [0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0]
DC_VALS = list(range(12))
AC_BITS = [1] + [0] * 15
AC_VALS = [0x00]


def huff_codes(bits, vals):
    codes, code, k = {}, 0, 0
    for length in range(1, 17):
        for _ in range(bits[length - 1]):
            codes[vals[k]] = (code, length)
            code += 1
            k += 1
        code <<= 1
    return codes


DC_CODES = huff_codes(DC_BITS, DC_VALS)
EOB = huff_codes(AC_BITS, AC_VALS)[0]


def segment(marker, payload):
    return struct.pack(">BBH", 0xFF, marker, len(payload) + 2) + payload


def ycc_dc(rgb):
    r, g, b = rgb
    y = 0.299 * r + 0.587 * g + 0.114 * b
    cb = -0.168736 * r - 0.331264 * g + 0.5 * b + 128
    cr = 0.5 * r - 0.418688 * g - 0.081312 * b + 128
    return tuple(max(-1024, min(1016, int(round(8 * (v - 128))))) for v in (y, cb, cr))


def encode_blocks(cols, rows, colour):
    """JPEG bytes for a (8 * cols) x (8 * rows) image where block (bx, by) is colour(bx, by)."""
    w, h = cols * 8, rows * 8
    head = b"\xFF\xD8"
    head += segment(0xE0, b"JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00")
    head += segment(0xDB, b"\x00" + b"\x01" * 64)
    head += segment(0xC0, struct.pack(">BHHB", 8, h, w, 3) + b"\x01\x11\x00\x02\x11\x00\x03\x11\x00")
    head += segment(0xC4, b"\x00" + bytes(DC_BITS) + bytes(DC_VALS) + b"\x10" + bytes(AC_BITS) +
                    bytes(AC_VALS))
    head += segment(0xDA, b"\x03\x01\x00\x02\x00\x03\x00\x00\x3F\x00")

    out = bytearray()
    acc = nbits = 0
    pred = [0, 0, 0]
    cache = {}
    eob_code, eob_len = EOB

    for by in range(rows):
        for bx in range(cols):
            c = colour(bx, by)
            dcs = cache.get(c)
            if dcs is None:
                dcs = cache[c] = ycc_dc(c)
            for ci in range(3):
                diff = dcs[ci] - pred[ci]
                pred[ci] = dcs[ci]
                cat = abs(diff).bit_length()
                code, length = DC_CODES[cat]
                acc = (acc << length) | code
                nbits += length
                if cat:
                    acc = (acc << cat) | (diff if diff > 0 else diff + (1 << cat) - 1)
                    nbits += cat
                acc = (acc << eob_len) | eob_code
                nbits += eob_len
                while nbits >= 8:
                    nbits -= 8
                    byte = (acc >> nbits) & 0xFF
                    out.append(byte)
                    if byte == 0xFF:
                        out.append(0)
                acc &= (1 << nbits) - 1
    if nbits:
        pad = 8 - nbits
        byte = ((acc << pad) | ((1 << pad) - 1)) & 0xFF
        out.append(byte)
        if byte == 0xFF:
            out.append(0)
    return head + bytes(out) + b"\xFF\xD9"


FONT = ["111101101101111", "010110010010111", "111001111100111", "111001111001111",
        "101101111001001", "111100111001111", "111100111101111", "111001001001001",
        "111101111101111", "111101111001111"]  # 3x5 digits


def test_card(w, h, phase, period):
    cols, rows = w // 8, h // 8
    cs = max(2, cols // 10)
    bar_w, bar_h = max(2, cols // 12), max(4, rows // 2)
    bar_x = cs + (cols - 2 * cs - bar_w) * phase // max(1, period - 1)
    bar_y = (rows - bar_h) // 2
    digits = "%02d" % (phase % 100)
    scale = max(1, rows // 30)
    dig_x = cols // 2 - (len(digits) * 4 * scale) // 2
    dig_y = max(cs + 1, rows // 6)

    def colour(bx, by):
        if by < cs and bx < cs:
            return (255, 0, 0)
        if by < cs and bx >= cols - cs:
            return (0, 255, 0)
        if by >= rows - cs and bx < cs:
            return (0, 0, 255)
        if by >= rows - cs and bx >= cols - cs:
            return (255, 255, 255)
        if bar_x <= bx < bar_x + bar_w and bar_y <= by < bar_y + bar_h:
            return (255, 140, 0)
        fx, fy = (bx - dig_x) // scale, (by - dig_y) // scale
        if 0 <= fy < 5 and 0 <= fx < len(digits) * 4 and fx % 4 < 3:
            if FONT[ord(digits[fx // 4]) - 48][fy * 3 + fx % 4] == "1":
                return (255, 255, 255)
        return (20, 30 + 40 * by // rows, 50 + 60 * by // rows)

    return encode_blocks(cols, rows, colour)


# ---- real JPEG files ----

def jpeg_size(data):
    """(width, height) from the first SOFn marker, or None."""
    if data[:2] != b"\xFF\xD8":
        return None
    i = 2
    while i + 9 < len(data):
        if data[i] != 0xFF:
            i += 1
            continue
        marker = data[i + 1]
        if marker == 0xFF:
            i += 1
            continue
        if marker in (0x01, 0xD8) or 0xD0 <= marker <= 0xD7:
            i += 2
            continue
        seg_len = struct.unpack(">H", data[i + 2:i + 4])[0]
        if 0xC0 <= marker <= 0xCF and marker not in (0xC4, 0xC8, 0xCC):
            h, w = struct.unpack(">HH", data[i + 5:i + 9])
            return w, h
        i += 2 + seg_len
    return None


def load_files(paths):
    frames = []
    for p in paths:
        with open(p, "rb") as f:
            data = f.read()
        size = jpeg_size(data)
        if not size:
            sys.exit("not a JPEG (no SOI or SOF marker): %s" % p)
        if len(data) > MAX_PAYLOAD:
            sys.exit("%s is %d bytes, above the IF-5.6 limit of %d" % (p, len(data), MAX_PAYLOAD))
        frames.append((data, size))
    return frames


def frame_bytes(node, seq, jpeg, size):
    w, h = size
    node_b = node.encode("ascii")[:8].ljust(8, b"\x00")
    ts_ms = int(time.monotonic() * 1000)
    return HEADER.pack(MAGIC, VERSION, 0, 0, node_b, ts_ms, w, h, seq & 0xFFFFFFFF, len(jpeg)) + jpeg


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--host", default="127.0.0.1", help="hud_display host (IPv4, IPv6 or name)")
    ap.add_argument("--port", type=int, default=47106)
    ap.add_argument("--node", default="R1", help="node_id in the header (8 ASCII chars max)")
    ap.add_argument("--fps", type=float, default=15.0)
    ap.add_argument("--size", default="640x480", help="test card WxH, rounded down to 8 px")
    ap.add_argument("--jpeg", nargs="+", help="JPEG files to loop instead of the test card")
    ap.add_argument("--dir", help="loop every *.jpg / *.jpeg in this folder (sorted)")
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = forever)")
    ap.add_argument("--frames", type=int, default=0, help="stop after this many frames (0 = no limit)")
    ap.add_argument("--once", action="store_true", help="exit instead of reconnecting")
    ap.add_argument("--junk", action="store_true",
                    help="send 50 junk bytes after connecting (exercises the receiver's resync)")
    ap.add_argument("--write", metavar="PATH", help="write one test card JPEG to PATH and exit")
    ap.add_argument("--first-delay", type=float, default=0, metavar="S",
                    help="wait S seconds after connecting before the first frame (a slow phone)")
    ap.add_argument("--exit-delay", type=float, default=0, metavar="S",
                    help="linger S seconds after closing the socket before exiting (models a slow "
                         "interpreter teardown; the connection is already gone)")
    args = ap.parse_args()

    stop = []

    def on_term(signum, frame):
        stop.append(signum)

    signal.signal(signal.SIGTERM, on_term)  # SIGTERM: send no more frames, close, exit 0

    w, h = (int(v) // 8 * 8 for v in args.size.lower().split("x"))
    period = max(2, int(round(args.fps * 2)))  # the bar crosses the screen in about 2 s
    if args.write:
        with open(args.write, "wb") as f:
            f.write(test_card(w, h, 0, period))
        print("wrote %s (%dx%d)" % (args.write, w, h))
        return 0

    files = list(args.jpeg or [])
    if args.dir:
        files += sorted(glob.glob(os.path.join(args.dir, "*.jpg")) +
                        glob.glob(os.path.join(args.dir, "*.jpeg")))
    frames = load_files(files) if files else None
    cards = {}

    def next_jpeg(i):
        if frames:
            return frames[i % len(frames)]
        phase = i % period
        if phase not in cards:
            cards[phase] = (test_card(w, h, phase, period), (w, h))
        return cards[phase]

    src = "%d JPEG file(s)" % len(frames) if frames else "test card %dx%d" % (w, h)
    print("[video_send] %s -> %s port %d as %s at %.1f fps" % (src, args.host, args.port, args.node,
                                                              args.fps), file=sys.stderr)
    t_end = time.monotonic() + args.seconds if args.seconds > 0 else None
    seq = sent = 0
    period_s = 1.0 / args.fps if args.fps > 0 else 0
    while not stop:
        try:
            sock = socket.create_connection((args.host, args.port), timeout=2)
        except OSError as e:
            if args.once:
                print("[video_send] connect failed: %s" % e, file=sys.stderr)
                return 1
            print("[video_send] connect failed (%s), retrying in 1 s" % e, file=sys.stderr)
            time.sleep(1)
            continue
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        print("[video_send] connected to %s" % (sock.getpeername(),), file=sys.stderr)
        try:
            if args.junk:
                sock.sendall(b"junk before the first header: resync me " + b"\x00" * 9)
            t_first = time.monotonic() + args.first_delay
            while not stop and time.monotonic() < t_first:
                time.sleep(0.05)
            t_next = time.monotonic()
            t_log, n_log = t_next, 0
            while True:
                now = time.monotonic()
                if stop or (t_end and now >= t_end) or (args.frames and sent >= args.frames):
                    print("[video_send] done: sent %d frames%s" % (sent, " (SIGTERM)" if stop else ""),
                          file=sys.stderr)
                    sock.close()
                    if args.exit_delay > 0:
                        time.sleep(args.exit_delay)
                    return 0
                jpeg, size = next_jpeg(seq)
                sock.sendall(frame_bytes(args.node, seq, jpeg, size))
                seq += 1
                sent += 1
                n_log += 1
                if now - t_log >= 1.0:
                    print("[video_send] %d frames, %.1f fps, last %d bytes" %
                          (sent, n_log / (now - t_log), len(jpeg)), file=sys.stderr)
                    t_log, n_log = now, 0
                t_next += period_s
                wait = t_next - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
                elif wait < -period_s:
                    t_next = time.monotonic()  # fell behind: skip ahead, never burst
        except OSError as e:
            sock.close()
            if args.once:
                print("[video_send] disconnected: %s" % e, file=sys.stderr)
                return 1
            print("[video_send] disconnected (%s), reconnecting in 1 s" % e, file=sys.stderr)
            time.sleep(1)
    print("[video_send] done: sent %d frames (SIGTERM)" % sent, file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
