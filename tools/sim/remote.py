#!/usr/bin/env python3
"""WS6 remote-node simulator (no camera, no OpenCV).

Replays `remote_contacts` (IF-5.4) to the Pi's hud_fusion so fusion/HUD can be
tested with no phone. Drives one or more people on scripted left<->right paths.

  python3 tools/sim/remote.py --pi qnxpi-felix.local --node R1
  python3 tools/sim/remote.py --pi 127.0.0.1 --node R2 --people 2 --pose 3,0,180
"""
import argparse, json, math, socket, sys, time

LOG = lambda *a: print("[sim.remote]", *a, file=sys.stderr, flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pi", default="qnxpi-felix.local")
    ap.add_argument("--port", type=int, default=47103)
    ap.add_argument("--node", default="R1")
    ap.add_argument("--pose", default=None, help="node taped pose 'x,y,yaw_deg'; omit for null")
    ap.add_argument("--people", type=int, default=1)
    ap.add_argument("--fov", type=float, default=60.0)
    ap.add_argument("--rate", type=float, default=15.0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    args = ap.parse_args()

    pose = None
    if args.pose:
        px, py, yaw = [float(v) for v in args.pose.split(",")]
        pose = {"x_m": px, "y_m": py, "yaw_deg": yaw}
    W, H = args.width, args.height
    f = (W / 2.0) / math.tan(math.radians(args.fov) / 2.0)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    LOG("node %s -> %s:%d  %d person(s)  pose=%s" % (args.node, args.pi, args.port, args.people, pose))

    t0 = time.monotonic()
    period = 1.0 / args.rate
    n = 0
    try:
        while True:
            now = time.monotonic()
            t = now - t0
            contacts = []
            for p in range(args.people):
                phase = p * 1.7
                cx = W * (0.5 + 0.42 * math.sin(t * 0.5 + phase))
                bh = H * (0.5 + 0.08 * math.sin(t * 0.9 + phase))   # range wobble
                bw = bh * 0.4
                x = cx - bw / 2
                contacts.append({
                    "id": "%s-%d" % (args.node, p + 1),
                    "cls": "person",
                    "conf": 0.9,
                    "box": [int(x), int(H * 0.3), int(bw), int(bh)],
                    "bearing_deg": round(math.degrees(math.atan((cx - W / 2) / f)), 1),
                    "range_m": round(f * 1.7 / bh, 2),
                })
            now_ms = int(now * 1000)
            msg = {"type": "remote_contacts", "v": 2, "src": args.node,
                   "t_mono_ms": now_ms, "ts_ms": now_ms, "node_id": args.node,
                   "pose": pose, "width": W, "height": H, "hfov_deg": args.fov,
                   "contacts": contacts}
            sock.sendto(json.dumps(msg).encode(), (args.pi, args.port))
            n += 1
            if n % 30 == 0:
                LOG("sent %d msgs" % n)
            dt = period - (time.monotonic() - now)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        LOG("stopped")


if __name__ == "__main__":
    main()
