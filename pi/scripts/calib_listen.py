#!/usr/bin/env python3
"""calib_listen.py: calibrate the fixed phones from what they send (run by scripts/calibrate.sh).

Listens on UDP 47103 (instead of hud_fusion, through the phone bridge) for --seconds and writes
sh lines for scripts/run-live.sh to --out (keeps lines it does not replace).

  pose            each phone sees tag 0 (?tag=1&tagid=0): the median of its tag poses is its
                  pinned room pose (x,y,yaw in the tag 0 frame).
  aim --mark X,Y  one person stands on a taped mark X,Y (metres from tag 0): each phone keeps
                  its x,y (from --out or the environment) and its yaw is solved from the
                  person's median bearing. For a phone that cannot keep tag 0 in view once aimed.
  yaw             a pinned phone keeps its x,y (from --out or the environment, e.g. R2_POSE) and
                  its yaw comes from where tag 0 sits in its picture: the tag's bearing in the
                  image is the same for both solutions of a flat tag's pose (the flips), so this
                  is steadier than the tag pose's own yaw. Link ?tag=1&tagid=0, tag 0 in view.
  tag --id N      tag N's room pose for the wearer's tag map: one phone (--nodes R1) held
                  still where it sees tag 0 and tag N, first with a ?tagid=0 link, then
                  (Enter) a ?tagid=N link. Tag N = (camera in tag 0) after (camera in tag N)
                  inverted. Merged into TAG_MAP.

Bearings are rescaled to the real FOV the same way hud_fusion --hfov does (R1_HFOV, R2_HFOV:
degrees or iphone17pro, iphone16, iphone-ultrawide).
"""
import argparse, json, math, os, socket, statistics, sys, time

PRESETS = {"iphone17pro": 24.0, "iphone16": 26.0, "iphone-ultrawide": 13.0}  # mm, 35 mm equiv.
VAR = {"R1": "WEARER_POSE", "R2": "R2_POSE"}


def real_hfov(spec, w, h):
    if not spec:
        return None
    try:
        return float(spec)
    except ValueError:
        f = PRESETS.get(spec)
        if f is None:
            sys.exit("unknown FOV preset %r (want degrees or %s)" % (spec, ", ".join(PRESETS)))
        half = 12.98 if h > w else 17.31  # 4:3 frame with the 35 mm diagonal
        return math.degrees(2 * math.atan(half / f))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def circ_median(angles):
    ref = angles[0]
    return wrap(ref + statistics.median(wrap(a - ref) for a in angles))


def read_env(path):
    vals = {}
    try:
        for line in open(path):
            line = line.strip()
            if "=" in line and not line.startswith("#"):
                k, v = line.split("=", 1)
                v = v.strip().strip('"')
                if v.startswith("${") and ":-" in v:  # NAME=${NAME:-value}
                    v = v.split(":-", 1)[1].rstrip("}").strip('"')
                vals[k.strip()] = v
    except OSError:
        pass
    return vals


def write_env(path, updates):
    keep = []
    try:
        keep = [l.rstrip("\n") for l in open(path)
                if not any(l.startswith(k + "=") for k in updates) and not l.startswith("# written")]
    except OSError:
        pass
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    with open(path, "w") as f:
        f.write("# written by scripts/calibrate.sh %s; run-live.sh reads it, env wins\n"
                % time.strftime("%Y-%m-%d %H:%M:%S"))
        for l in keep:
            f.write(l + "\n")
        for k, v in updates.items():
            f.write('%s=${%s:-%s}\n' % (k, k, v))


def collect_pose(s, node, seconds):
    """Median tag pose (x, y, yaw) one node sends for seconds, or None."""
    ps, end = [], time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            m = json.loads(s.recv(65536))
        except (socket.timeout, ValueError):
            continue
        p = m.get("pose")
        if m.get("node_id") == node and isinstance(p, dict) and \
                all(isinstance(p.get(k), (int, float)) for k in ("x_m", "y_m", "yaw_deg")):
            ps.append((p["x_m"], p["y_m"], p["yaw_deg"]))
    if len(ps) < 10:
        return None, len(ps)
    return (statistics.median(p[0] for p in ps), statistics.median(p[1] for p in ps),
            circ_median([p[2] for p in ps])), len(ps)


def tag_from_two_views(p0, pn):
    """Room pose of tag N from the same camera's pose in tag 0's frame (p0) and tag N's (pn):
    p0 = from_tag(T, pn), with from_tag(T, l) = T.xy + R(T.yaw) l.xy, yaw T.yaw + l.yaw."""
    yaw = wrap(p0[2] - pn[2])
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    x = p0[0] - (pn[0] * c + pn[1] * s)
    y = p0[1] - (-pn[0] * s + pn[1] * c)
    return x, y, yaw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["pose", "aim", "tag", "yaw"])
    ap.add_argument("--id", type=int, help="tag: the tag to measure (1..9)")
    ap.add_argument("--mark", help="aim: the person's taped spot x,y in metres from tag 0")
    ap.add_argument("--seconds", type=float, default=15)
    ap.add_argument("--out", default=os.path.expanduser("~/seeker-logs/calib.env"))
    ap.add_argument("--port", type=int, default=47103)
    ap.add_argument("--nodes", default="R1,R2")
    a = ap.parse_args()
    nodes = a.nodes.split(",")
    mark = None
    if a.mode == "aim":
        if not a.mark:
            sys.exit("aim needs --mark X,Y")
        mark = [float(v) for v in a.mark.split(",")]

    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("0.0.0.0", a.port))
    except OSError as e:
        sys.exit("cannot bind udp/%d (%s): stop hud_fusion first (sh scripts/stop-live.sh)" % (a.port, e))
    s.settimeout(0.5)
    if a.mode == "tag":
        if not a.id or a.id < 1:
            sys.exit("tag needs --id N (1..9)")
        node = nodes[0]
        print("calib: %s on its ?tagid=0 link, held still (%.0f s)" % (node, a.seconds))
        p0, n0 = collect_pose(s, node, a.seconds)
        if not p0:
            sys.exit("calib: only %d tag 0 poses from %s: is tag 0 in view?" % (n0, node))
        input("calib: now reload %s with ?tagid=%d (same spot, do not move it), wait for TAG %d "
              "on its pill, then press Enter " % (node, a.id, a.id))
        s.settimeout(0.5)
        while True:  # drop what queued up while waiting
            try:
                s.setblocking(False)
                s.recv(65536)
            except (BlockingIOError, OSError):
                break
        s.settimeout(0.5)
        pn, nn = collect_pose(s, node, a.seconds)
        if not pn:
            sys.exit("calib: only %d tag %d poses from %s: is tag %d in view?" % (nn, a.id, node, a.id))
        tx, ty, tyaw = tag_from_two_views(p0, pn)
        env = read_env(a.out)
        entries = {e.split("=", 1)[0]: e for e in (os.environ.get("TAG_MAP") or env.get("TAG_MAP", "")).split()}
        entries[str(a.id)] = "%d=%.2f,%.2f,%.1f" % (a.id, tx, ty, tyaw)
        print("calib: tag %d at room %.2f,%.2f, facing yaw %.1f (camera %.2f,%.2f,%.1f from tag 0, "
              "%.2f,%.2f,%.1f from tag %d)" % (a.id, tx, ty, tyaw, *p0, *pn, a.id))
        write_env(a.out, {"TAG_MAP": '"%s"' % " ".join(entries[k] for k in sorted(entries))})
        print("calib: wrote %s" % a.out)
        return 0
    poses = {n: [] for n in nodes}
    bearings = {n: [] for n in nodes}
    frame = {}
    end = time.monotonic() + a.seconds
    last_print = 0
    print("calib: listening %.0f s on udp/%d for %s (%s)" % (a.seconds, a.port, ", ".join(nodes), a.mode))
    while time.monotonic() < end:
        try:
            m = json.loads(s.recv(65536))
        except (socket.timeout, ValueError):
            continue
        n = m.get("node_id")
        if n not in poses:
            continue
        frame[n] = (m.get("width") or 640, m.get("height") or 480, m.get("hfov_deg") or 66.0)
        p = m.get("pose")
        if isinstance(p, dict) and all(isinstance(p.get(k), (int, float)) for k in ("x_m", "y_m", "yaw_deg")):
            if p.get("tag_id") in (None, 0):
                poses[n].append((p["x_m"], p["y_m"], p["yaw_deg"]))
        cs = m.get("contacts") or []
        if len(cs) == 1 and isinstance(cs[0].get("bearing_deg"), (int, float)):
            bearings[n].append(cs[0]["bearing_deg"])
        if time.monotonic() - last_print > 2:
            last_print = time.monotonic()
            print("  " + "  ".join("%s: %d poses %d bearings" % (k, len(poses[k]), len(bearings[k])) for k in nodes))

    env = read_env(a.out)
    updates, ok = {}, True
    for n in nodes:
        var = VAR.get(n, n + "_POSE")
        if a.mode == "yaw":
            ps = poses[n]
            pose = os.environ.get(var) or env.get(var)
            if not pose:
                print("calib: %s has no %s yet: set it (x,y measured, any yaw)" % (n, var))
                ok = False
                continue
            if len(ps) < 10:
                print("calib: %s sent %d tag 0 poses (need 10): link ?tag=1&tagid=0, tag 0 in view?" % (n, len(ps)))
                ok = False
                continue
            px, py, old_yaw = [float(v) for v in pose.split(",")]
            # Tag 0's bearing in the picture (same for both pose solutions), on the phone's FOV.
            rel = circ_median([wrap(math.degrees(math.atan2(-p[0], -p[1])) - p[2]) for p in ps])
            w, h, sent = frame.get(n, (640, 480, 66.0))
            real = real_hfov(os.environ.get(n + "_HFOV") or env.get(n + "_HFOV"), w, h)
            if real and abs(rel) < 80:
                rel = math.degrees(math.atan(math.tan(math.radians(rel)) *
                                             math.tan(math.radians(real / 2)) / math.tan(math.radians(sent / 2))))
            yaw = wrap(math.degrees(math.atan2(-px, -py)) - rel)
            print("calib: %s = %.2f,%.2f,%.1f  (yaw from tag 0 at %.1f deg in its picture, %d poses%s; "
                  "was %.1f)" % (var, px, py, yaw, rel, len(ps), ", FOV %.1f" % real if real else "",
                                 old_yaw))
            updates[var] = "%.2f,%.2f,%.1f" % (px, py, yaw)
            continue
        if a.mode == "pose":
            ps = poses[n]
            if len(ps) < 10:
                print("calib: %s sent %d tag 0 poses (need 10): is tag 0 in its view, link ?tag=1&tagid=0?" % (n, len(ps)))
                ok = False
                continue
            x = statistics.median(p[0] for p in ps)
            y = statistics.median(p[1] for p in ps)
            yaw = circ_median([p[2] for p in ps])
            spread = max(math.hypot(p[0] - x, p[1] - y) for p in ps)
            print("calib: %s = %.2f,%.2f,%.1f  (%d poses, spread %.2f m)%s" % (
                var, x, y, yaw, len(ps), spread,
                "  WARNING: phone moved or tag far; redo" if spread > 0.3 else ""))
            updates[var] = "%.2f,%.2f,%.1f" % (x, y, yaw)
        else:
            bs = bearings[n]
            pose = os.environ.get(var) or env.get(var)
            if not pose:
                print("calib: %s has no %s yet: run 'calibrate.sh pose' or set it" % (n, var))
                ok = False
                continue
            if len(bs) < 10:
                print("calib: %s saw one person in %d messages (need 10): is the person in its view, alone?" % (n, len(bs)))
                ok = False
                continue
            px, py, _ = [float(v) for v in pose.split(",")]
            b = statistics.median(bs)
            w, h, sent = frame.get(n, (640, 480, 66.0))
            real = real_hfov(os.environ.get(n + "_HFOV") or env.get(n + "_HFOV"), w, h)
            if real:
                b = math.degrees(math.atan(math.tan(math.radians(b)) *
                                           math.tan(math.radians(real / 2)) / math.tan(math.radians(sent / 2))))
            yaw = wrap(math.degrees(math.atan2(mark[0] - px, mark[1] - py)) - b)
            print("calib: %s = %.2f,%.2f,%.1f  (yaw from %d bearings, median %.1f deg%s)" % (
                var, px, py, yaw, len(bs), b, ", FOV %.1f" % real if real else ""))
            updates[var] = "%.2f,%.2f,%.1f" % (px, py, yaw)
    if updates:
        write_env(a.out, updates)
        print("calib: wrote %s" % a.out)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
