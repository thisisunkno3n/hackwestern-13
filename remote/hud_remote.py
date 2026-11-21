#!/usr/bin/env python3
"""WS6 Seeker remote node relay.

A Mac reads a phone's IP-camera stream, detects people, and sends ONLY
detections (box, bearing, range) as `remote_contacts` JSON over UDP to the
helmet Pi's hud_fusion (qnxpi-felix.local:47103). The Mac is a processing
relay, not a camera. One node_id (R1, R2, ...) per phone stream.

Contract: IF-5.4 remote_contacts, IF-1 room frame + cross-machine clock rule.

Backends (chosen by --backend, default auto):
  stub      synthetic walking person, NO dependencies  -> works right now
  hog       OpenCV HOG people detector     (needs: opencv-python)
  yolo      YOLOv8n person class           (needs: ultralytics, opencv-python)
  mediapipe single-person pose bbox        (needs: mediapipe, opencv-python)

Examples:
  python3 remote/hud_remote.py --pi qnxpi-felix.local --node R1 --source stub
  python3 remote/hud_remote.py --node R1 --stream http://192.168.4.5:8080/video --backend hog
  python3 remote/hud_remote.py --node R2 --stream webcam:0 --pose 3.0,0,180
"""
import argparse, json, math, socket, sys, time

LOG = lambda *a: print("[hud_remote]", *a, file=sys.stderr, flush=True)


def t_mono_ms():
    return int(time.monotonic() * 1000)


def focal_px(width, fov_deg, focal_override):
    if focal_override:
        return float(focal_override)
    return (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)


def bearing_deg(cx_box, width, f):
    """Clockwise from the node's boresight (image +x = right = positive)."""
    return math.degrees(math.atan((cx_box - width / 2.0) / f))


def mono_range_m(box_h, f, person_h):
    if not box_h or box_h <= 0:
        return None
    return round(f * person_h / box_h, 2)


def make_contacts(boxes, width, f, person_h, node):
    """boxes: list of (x,y,w,h,conf). -> IF-5.4 contact dicts."""
    out = []
    for i, (x, y, w, h, conf) in enumerate(boxes, 1):
        out.append({
            "id": "%s-%d" % (node, i),
            "cls": "person",
            "conf": round(float(conf), 3),
            "box": [int(x), int(y), int(w), int(h)],
            "bearing_deg": round(bearing_deg(x + w / 2.0, width, f), 1),
            "range_m": mono_range_m(h, f, person_h),
        })
    return out


# ----------------- detection backends -----------------
class StubBackend:
    """No deps: one person walking left<->right across a virtual frame."""
    def __init__(self, width, height):
        self.w, self.h = width, height
        self.t0 = time.monotonic()
    def frame_size(self):
        return self.w, self.h
    def detect(self):
        t = time.monotonic() - self.t0
        cx = self.w * (0.5 + 0.42 * math.sin(t * 0.6))        # sweep across view
        bh = self.h * 0.55
        bw = bh * 0.4
        return [(cx - bw / 2, self.h * 0.3, bw, bh, 0.95)]


class CVBackend:
    """OpenCV-based backends (hog/yolo/mediapipe) over a VideoCapture stream."""
    def __init__(self, stream, backend, conf):
        import cv2  # lazy
        self.cv2 = cv2
        self.backend = backend
        self.conf = conf
        src = 0 if stream in ("webcam:0", "0") else \
              int(stream.split(":")[1]) if stream.startswith("webcam:") else stream
        self.cap = cv2.VideoCapture(src)
        if not self.cap.isOpened():
            raise RuntimeError("cannot open stream: %r" % stream)
        self.w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 1280
        self.h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 720
        self._init_detector()
    def frame_size(self):
        return self.w, self.h
    def _init_detector(self):
        cv2 = self.cv2
        if self.backend == "hog":
            self.hog = cv2.HOGDescriptor()
            self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
        elif self.backend == "yolo":
            from ultralytics import YOLO
            self.yolo = YOLO("yolov8n.pt")
        elif self.backend == "mediapipe":
            import mediapipe as mp
            self.pose = mp.solutions.pose.Pose(model_complexity=0)
        else:
            raise ValueError("unknown backend " + self.backend)
    def detect(self):
        ok, frame = self.cap.read()
        if not ok:
            return []
        self.h, self.w = frame.shape[0], frame.shape[1]
        if self.backend == "hog":
            rects, weights = self.hog.detectMultiScale(frame, winStride=(8, 8))
            return [(x, y, w, h, float(wt)) for (x, y, w, h), wt in zip(rects, weights)
                    if wt >= self.conf]
        if self.backend == "yolo":
            res = self.yolo(frame, verbose=False, classes=[0], conf=self.conf)[0]
            out = []
            for b in res.boxes:
                x1, y1, x2, y2 = b.xyxy[0].tolist()
                out.append((x1, y1, x2 - x1, y2 - y1, float(b.conf[0])))
            return out
        if self.backend == "mediapipe":
            rgb = self.cv2.cvtColor(frame, self.cv2.COLOR_BGR2RGB)
            r = self.pose.process(rgb)
            if not r.pose_landmarks:
                return []
            xs = [lm.x * self.w for lm in r.pose_landmarks.landmark]
            ys = [lm.y * self.h for lm in r.pose_landmarks.landmark]
            x0, y0, x1, y1 = min(xs), min(ys), max(xs), max(ys)
            return [(x0, y0, x1 - x0, y1 - y0, 0.9)]
        return []


def pick_backend(args):
    if args.source == "stub" or args.backend == "stub":
        return StubBackend(args.width, args.height)
    order = [args.backend] if args.backend != "auto" else ["yolo", "hog", "mediapipe"]
    last = None
    for b in order:
        try:
            return CVBackend(args.stream or args.source, b, args.conf)
        except Exception as e:
            last = e; LOG("backend %s unavailable: %s" % (b, e))
    raise SystemExit("no usable backend (install opencv-python / ultralytics / mediapipe, "
                     "or use --source stub).  last error: %s" % last)


def main():
    ap = argparse.ArgumentParser(description="Seeker WS6 remote node relay")
    ap.add_argument("--pi", default="qnxpi-felix.local", help="helmet Pi host")
    ap.add_argument("--port", type=int, default=47103)
    ap.add_argument("--node", default="R1", help="node id (R1, R2, ...)")
    ap.add_argument("--stream", default=None, help="phone IP-cam URL or webcam:IDX")
    ap.add_argument("--source", default=None, help="'stub' for no-camera synthetic mode")
    ap.add_argument("--backend", default="auto", choices=["auto", "stub", "hog", "yolo", "mediapipe"])
    ap.add_argument("--pose", default=None, help="node taped pose 'x_m,y_m,yaw_deg' (room frame); omit for null")
    ap.add_argument("--fov", type=float, default=60.0, help="camera horizontal FOV (deg)")
    ap.add_argument("--focal", type=float, default=None, help="focal length px (overrides --fov)")
    ap.add_argument("--person-height", type=float, default=1.7, dest="ph")
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--rate", type=float, default=15.0, help="max send Hz")
    ap.add_argument("--width", type=int, default=1280, help="stub frame width")
    ap.add_argument("--height", type=int, default=720, help="stub frame height")
    args = ap.parse_args()
    if not args.stream and not args.source:
        args.source = "stub"

    pose = None
    if args.pose:
        px, py, yaw = [float(v) for v in args.pose.split(",")]
        pose = {"x_m": px, "y_m": py, "yaw_deg": yaw}
    det = pick_backend(args)
    W, H = det.frame_size()
    f = focal_px(W, args.fov, args.focal)
    ai = socket.getaddrinfo(args.pi, args.port, 0, socket.SOCK_DGRAM)[0]  # IPv4 or IPv6
    sock = socket.socket(ai[0], socket.SOCK_DGRAM)
    dst = ai[4]
    LOG("node %s -> %s:%d  source=%s  %dx%d  f=%.1fpx  pose=%s"
        % (args.node, args.pi, args.port, args.source or args.stream, W, H, f, pose))

    period = 1.0 / args.rate
    n = 0
    try:
        while True:
            t = time.monotonic()
            W, H = det.frame_size()
            f = focal_px(W, args.fov, args.focal)
            boxes = det.detect()
            contacts = make_contacts(boxes, W, f, args.ph, args.node)
            now_ms = t_mono_ms()
            msg = {"type": "remote_contacts", "v": 2, "src": args.node,
                   "t_mono_ms": now_ms, "ts_ms": now_ms, "node_id": args.node,
                   "pose": pose, "width": W, "height": H, "hfov_deg": args.fov,
                   "contacts": contacts}
            sock.sendto(json.dumps(msg).encode(), dst)
            n += 1
            if n % 30 == 0:
                LOG("sent %d msgs, last had %d contact(s)" % (n, len(contacts)))
            dt = period - (time.monotonic() - t)
            if dt > 0:
                time.sleep(dt)
    except KeyboardInterrupt:
        LOG("stopped (node %s) -> Pi will show NODE %s LOST within 1.5 s" % (args.node, args.node))


if __name__ == "__main__":
    main()
