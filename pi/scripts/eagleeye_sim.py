#!/usr/bin/env python3
"""EagleEye scene sim: three phones, a wall, and people, as IF-5.4 v2 remote_contacts.

Stands in for ws_ingest.py + phones so hud_fusion and `hud_display --view wearer` can be demoed
and tested with no hardware. Pure stdlib (runs on the Pi's Python 3.14). Default scene matches
config/seeker.eagleeye.json (room frame IF-1: +y forward, +x right, yaw clockwise from +y):

    y
    6.0 |  R1 (-3.0, 5.5) yaw 104 . . . . . . A walks left/right at y = 4.5
    2.5 |=========== wall (-4, 2.5) to (4, 2.5) ===========
    0.0 |                 W (0, 0) yaw 0: the helmet phone      B circles W at 1.8 m
   -2.5 |                                 R2 (2.5, -2.5) yaw 315
        +--------------------------------------------------- x

A is always behind the wall, so only R1 sees A: x-ray in the wearer view. B is seen by the helmet
phone when in front (solid box, merged W+R2) and only by R2 otherwise (edge arrow, BEHIND).

  python3 scripts/eagleeye_sim.py                        # live scene to 127.0.0.1:47103
  python3 scripts/eagleeye_sim.py --wearer-turn 40       # helmet sweeps +-40 deg (live pose)
  python3 scripts/eagleeye_sim.py --at 0 --run-for 3     # frozen at t = 0 s (tests, screenshots)
  python3 scripts/eagleeye_sim.py --pi qnxpi-felix.local # from a laptop to the Pi
"""
import argparse, json, math, random, socket, sys, time

W_PX, H_PX = 1280, 720
NODES = {  # id: [x_m, y_m, yaw_deg, hfov_deg]
    "W": [0.0, 0.0, 0.0, 66.0],
    "R1": [-3.0, 5.5, 104.0, 66.0],
    "R2": [2.5, -2.5, 315.0, 66.0],
}
WALLS = [(-4.0, 2.5, 4.0, 2.5)]


def log(*a):
    print("[eagleeye_sim]", *a, file=sys.stderr, flush=True)


def wrap180(d):
    d = math.fmod(d, 360.0)
    if d > 180.0:
        d -= 360.0
    elif d <= -180.0:
        d += 360.0
    return d


def person_a(t):  # behind the wall
    return (2.2 * math.sin(0.35 * t) + 0.2, 4.5)


def person_b(t):  # circles the wearer, in front at t = 0
    th = 0.25 * t
    return (1.8 * math.sin(th), 1.8 * math.cos(th))


PEOPLE = {"A": person_a, "B": person_b}


def crosses(ax, ay, bx, by, wall):
    cx, cy, dx, dy = wall

    def orient(px, py, qx, qy, rx, ry):
        return (qx - px) * (ry - py) - (qy - py) * (rx - px)

    o1 = orient(ax, ay, bx, by, cx, cy)
    o2 = orient(ax, ay, bx, by, dx, dy)
    o3 = orient(cx, cy, dx, dy, ax, ay)
    o4 = orient(cx, cy, dx, dy, bx, by)
    return o1 * o2 < 0 and o3 * o4 < 0


def observe(node, person, noise, with_range):
    nx, ny, yaw, hfov = node
    px, py = person
    dx, dy = px - nx, py - ny
    rng = math.hypot(dx, dy)
    if rng < 0.3 or rng > 10.0:
        return None
    b = wrap180(math.degrees(math.atan2(dx, dy)) - yaw)
    if abs(b) > hfov / 2.0 or any(crosses(nx, ny, px, py, w) for w in WALLS):
        return None
    f = (W_PX / 2.0) / math.tan(math.radians(hfov) / 2.0)
    u = W_PX / 2.0 + f * math.tan(math.radians(b))
    r_meas = rng * (1.0 + random.gauss(0.0, noise)) if noise > 0 else rng
    bh = min(H_PX * 0.98, f * 1.7 / r_meas)
    bw = bh * 0.4
    x0, x1 = max(0, int(u - bw / 2)), min(W_PX - 1, int(u + bw / 2))
    y0 = max(0, int(H_PX / 2 - bh * 0.45))
    return {
        "cls": "person",
        "conf": 0.9,
        "box": [x0, y0, max(1, x1 - x0), int(bh)],
        "bearing_deg": round(math.degrees(math.atan((u - W_PX / 2.0) / f)), 2),  # IF-1 atan
        "range_m": round(r_meas, 2) if with_range else None,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pi", "--host", dest="host", default="127.0.0.1", help="hud_fusion host")
    ap.add_argument("--port", type=int, default=47103)
    ap.add_argument("--rate", type=float, default=15.0, help="Hz per node (IF-5.4: ~15)")
    ap.add_argument("--nodes", default="W,R1,R2", help="which phones send (drop W: helmet off)")
    ap.add_argument("--people", default="A,B", help="A = behind the wall, B = circles the wearer")
    ap.add_argument("--speed", type=float, default=1.0, help="scene time multiplier")
    ap.add_argument("--at", type=float, default=None, help="freeze the scene at this time (s)")
    ap.add_argument("--wearer-yaw", type=float, default=0.0, help="helmet heading (deg)")
    ap.add_argument("--wearer-turn", type=float, default=0.0, help="sweep +- this many deg")
    ap.add_argument("--turn-period", type=float, default=12.0, help="sweep period (s)")
    ap.add_argument("--no-range", default="", help="nodes that send range_m null, e.g. W")
    ap.add_argument("--noise", type=float, default=0.0, help="relative range noise (0.05 = 5%%)")
    ap.add_argument("--run-for", type=float, default=0.0, help="stop after s (0 = forever)")
    ap.add_argument("--print-every", type=float, default=2.0)
    args = ap.parse_args()

    ids = [n for n in args.nodes.split(",") if n]
    for n in ids:
        if n not in NODES:
            sys.exit("unknown node %s (have %s)" % (n, ",".join(NODES)))
    people = [p for p in args.people.split(",") if p]
    no_range = set(n for n in args.no_range.split(",") if n)
    ai = socket.getaddrinfo(args.host, args.port, 0, socket.SOCK_DGRAM)[0]
    sock = socket.socket(ai[0], socket.SOCK_DGRAM)
    dst = ai[4]
    log("nodes %s, people %s -> %s:%d at %.0f Hz%s" % (
        ",".join(ids), ",".join(people), args.host, args.port, args.rate,
        "" if args.at is None else ", frozen at t=%.1f s" % args.at))

    t0 = time.monotonic()
    next_print = 0.0
    period = 1.0 / args.rate
    sent = 0
    try:
        while True:
            now = time.monotonic()
            el = now - t0
            if args.run_for > 0 and el >= args.run_for:
                break
            t = args.at if args.at is not None else el * args.speed
            wyaw = args.wearer_yaw + args.wearer_turn * math.sin(2 * math.pi * t / args.turn_period)
            nodes = {k: list(v) for k, v in NODES.items()}
            nodes["W"][2] = wyaw
            pos = {p: PEOPLE[p](t) for p in people}
            seen_by = {p: [] for p in people}
            ms = int(now * 1000)
            for nid in ids:
                nx, ny, yaw, hfov = nodes[nid]
                contacts = []
                for p in people:
                    c = observe(nodes[nid], pos[p], args.noise, nid not in no_range)
                    if c:
                        c["id"] = "%s-%d" % (nid, len(contacts) + 1)
                        contacts.append(c)
                        seen_by[p].append(nid)
                msg = {"type": "remote_contacts", "v": 2, "src": nid, "ts_ms": ms, "t_mono_ms": ms,
                       "node_id": nid, "width": W_PX, "height": H_PX, "hfov_deg": hfov,
                       "pose": {"x_m": nx, "y_m": ny, "yaw_deg": round(yaw, 2)},
                       "contacts": contacts}
                sock.sendto(json.dumps(msg).encode(), dst)
                sent += 1
            if el >= next_print:
                next_print = el + args.print_every
                log("t=%.1f W yaw %.0f | %s" % (t, wyaw, " | ".join(
                    "%s (%.1f,%.1f) %s" % (p, pos[p][0], pos[p][1], ",".join(seen_by[p]) or "nobody")
                    for p in people)))
            dt = period - (time.monotonic() - now)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        pass
    log("stopped after %d datagrams" % sent)


if __name__ == "__main__":
    main()
