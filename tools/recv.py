#!/usr/bin/env python3
"""Tiny UDP receiver for Seeker messages (IF-5). Prints one line per datagram.

  python3 tools/recv.py 47103      # watch remote_contacts from hud_remote nodes

On the helmet Pi this is the quick check for WS6:
  remote_contacts R1  1 contact  bearing -12.0  range 2.8 m
"""
import json, socket, sys, time


def fmt(m):
    t = m.get("type", "?")
    if t == "remote_contacts":
        cs = m.get("contacts", [])
        head = "remote_contacts %-3s %d contact%s" % (m.get("node_id", "?"), len(cs),
                                                      "" if len(cs) == 1 else "s")
        if cs:
            c = cs[0]
            rng = c.get("range_m")
            head += "  bearing %.1f  range %s" % (
                c.get("bearing_deg", 0.0), ("%.1f m" % rng) if rng is not None else "n/a")
        return head
    return "%s %s" % (t, json.dumps({k: v for k, v in m.items() if k not in ("type",)}))


def main():
    port = int(sys.argv[1]) if len(sys.argv) > 1 else 47103
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("", port))
    print("[recv] listening on udp/%d" % port, flush=True)
    last = {}
    while True:
        data, addr = s.recvfrom(65535)
        try:
            m = json.loads(data.decode())
        except Exception:
            print("[recv] non-JSON %d bytes from %s" % (len(data), addr[0])); continue
        node = m.get("node_id") or m.get("src") or addr[0]
        now = time.monotonic()
        if node in last and now - last[node] > 1.5:
            print("[recv] NODE %s resumed" % node)
        last[node] = now
        print("[recv] %s" % fmt(m), flush=True)


if __name__ == "__main__":
    main()
