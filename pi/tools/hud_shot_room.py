"""Made-up R1 camera frame (1280x720) for tools/hud_shot.c, host only (needs Pillow):

  python3 tools/hud_shot_room.py room.png [x y [down]]  # person at room (x, y), default -1.3 2.3
  python3 tools/hud_shot_room.py room-close.png -0.35 -0.6
  python3 tools/hud_shot_room.py room-down.png -1.0 4.5 down   # lying on the floor

Then convert to binary PPM (P6) for hud_shot --bg / --bg-close. The frame: a room seen from 1.7 m, a wall slab on the
right 2 m ahead (x 0.3..2.5 m, the person behind it is hidden), one person visible on the left.
Geometry matches tools/hud_shot.c: wearer at (0,-2) facing +y, hfov 66."""
import math, random, sys
from PIL import Image, ImageDraw, ImageFilter

W, H = 1280, 720
F = (W / 2) / math.tan(math.radians(33))
CY, CAM = H / 2, 1.7

def proj(x, y, h):  # room (x, y forward from wearer at y=-2), height h -> pixel
    z = y + 2.0
    return (W / 2 + F * x / z, CY - F * (h - CAM) / z)

img = Image.new("RGB", (W, H))
d = ImageDraw.Draw(img)
random.seed(4)

# far wall z=7.5, floor, ceiling, left wall
zb = 7.5
_, v_floor = proj(0, zb - 2, 0)
_, v_ceil = proj(0, zb - 2, 2.7)
for v in range(H):  # base gradient
    if v < v_ceil:
        t = v / v_ceil
        c = (int(196 - 30 * t), int(194 - 30 * t), int(190 - 30 * t))
    elif v < v_floor:
        t = (v - v_ceil) / (v_floor - v_ceil)
        c = (int(178 - 22 * t), int(172 - 22 * t), int(160 - 22 * t))
    else:
        t = (v - v_floor) / (H - v_floor)
        c = (int(118 - 40 * t), int(96 - 34 * t), int(78 - 28 * t))  # warm wood floor
    d.line([(0, v), (W, v)], fill=c)
# left wall plane x=-3.2 from z=1 to z=7.5
lw = [proj(-3.2, 1 - 2, 2.7), proj(-3.2, zb - 2, 2.7), proj(-3.2, zb - 2, 0), proj(-3.2, 1 - 2, 0)]
d.polygon(lw, fill=(160, 156, 148))
# floor boards: perspective lines
for i in range(-14, 20):
    x = i * 0.18
    a, b = proj(x, -1.2, 0), proj(x, zb - 2, 0)
    d.line([a, b], fill=(96, 76, 60), width=1)
# door in the far wall
dl, dt = proj(-1.9, zb - 2, 2.05), proj(-1.0, zb - 2, 0)
d.rectangle([dl[0], dl[1], dt[0], dt[1]], fill=(120, 104, 88))
d.rectangle([dl[0] + 4, dl[1] + 4, dt[0] - 4, dt[1]], fill=(140, 122, 102))
# picture frame and a window glow
pf = [proj(0.2, zb - 2, 1.9), proj(1.0, zb - 2, 1.35)]
d.rectangle([pf[0][0], pf[0][1], pf[1][0], pf[1][1]], fill=(70, 74, 80))
d.rectangle([pf[0][0] + 5, pf[0][1] + 5, pf[1][0] - 5, pf[1][1] - 5], fill=(120, 140, 150))
# ceiling light panel
cl = [proj(-0.8, 2.5, 2.7), proj(0.8, 2.5, 2.7), proj(0.8, 3.6, 2.7), proj(-0.8, 3.6, 2.7)]
d.polygon(cl, fill=(250, 248, 238))

# person on the left at (-1.3, 2.3): z = 4.3
px = float(sys.argv[2]) if len(sys.argv) > 2 else -1.3
py = float(sys.argv[3]) if len(sys.argv) > 3 else 2.3
def P(dx, h):
    return proj(px + dx, py, h)
skin, jacket, jeans, hair = (196, 150, 122), (44, 58, 78), (58, 70, 96), (40, 30, 24)
if len(sys.argv) > 4 and sys.argv[4] == "down":  # lying along x, head to the left
    d.polygon([P(0.35, 0.22), P(0.95, 0.16), P(0.95, 0.02), P(0.35, 0.02)], fill=jeans)
    d.polygon([P(-0.25, 0.34), P(0.38, 0.30), P(0.38, 0.03), P(-0.25, 0.04)], fill=jacket)
    d.polygon([P(-0.15, 0.36), P(0.25, 0.40), P(0.25, 0.32), P(-0.15, 0.30)], fill=jacket)
    hc = P(-0.38, 0.13); r = F * 0.11 / (py + 2)
    d.ellipse([hc[0] - r, hc[1] - r * 0.85, hc[0] + r, hc[1] + r * 0.85], fill=skin)
    d.chord([hc[0] - r * 1.05, hc[1] - r * 0.9, hc[0] + r * 0.2, hc[1] + r * 0.9], 90, 270, fill=hair)
else:
    d.polygon([P(-0.11, 0.9), P(0.11, 0.9), P(0.12, 0.0), P(0.02, 0.0), P(0.0, 0.8), P(-0.02, 0.0), P(-0.12, 0.0)], fill=jeans)
    d.polygon([P(-0.22, 1.42), P(0.22, 1.42), P(0.2, 0.86), P(-0.2, 0.86)], fill=jacket)
    d.polygon([P(-0.22, 1.42), P(-0.27, 1.05), P(-0.24, 0.82), P(-0.19, 0.84), P(-0.2, 1.3)], fill=jacket)
    d.polygon([P(0.22, 1.42), P(0.28, 1.08), P(0.26, 0.84), P(0.21, 0.85), P(0.2, 1.3)], fill=jacket)
    for sx, h0 in ((-0.255, 0.83), (0.245, 0.85)):
        c = P(sx, h0); r = F * 0.04 / (py + 2)
        d.ellipse([c[0] - r, c[1] - r, c[0] + r, c[1] + r], fill=skin)
    nk = P(0, 1.47); r = F * 0.05 / (py + 2)
    d.rectangle([nk[0] - r, nk[1] - r, nk[0] + r, nk[1] + r * 1.4], fill=skin)
    hc = P(0, 1.585); r = F * 0.105 / (py + 2)
    d.ellipse([hc[0] - r * 0.85, hc[1] - r, hc[0] + r * 0.85, hc[1] + r], fill=skin)
    d.chord([hc[0] - r * 0.9, hc[1] - r * 1.08, hc[0] + r * 0.9, hc[1] + r * 0.5], 180, 360, fill=hair)

# floor shadow under the person
sh = P(0, 0)
img2 = Image.new("L", (W, H), 0)
sw = F * 0.4 / (py + 2)
ImageDraw.Draw(img2).ellipse([sh[0] - sw, sh[1] - sw * 0.15, sh[0] + sw, sh[1] + sw * 0.15], fill=110)
img.paste((30, 24, 20), mask=img2.filter(ImageFilter.GaussianBlur(10)))

# near wall slab: face at y=0 (z=2) from x=0.3 to 2.5, 2.4 m tall, plus its end face
fx0, _ = proj(0.3, 0, 0)
ex, _ = proj(0.3, 0.12, 0)
for u in range(int(ex), W):
    t = (u - fx0) / (W - fx0) if u >= fx0 else 0
    base = (206, 202, 192) if u >= fx0 else (150, 146, 138)  # end face is in shadow
    shade = 1.0 - 0.18 * t
    col = tuple(int(c * shade) for c in base)
    d.line([(u, 0), (u, H)], fill=col)
d.line([(fx0, 0), (fx0, H)], fill=(232, 230, 222), width=2)  # lit corner edge

img = img.filter(ImageFilter.GaussianBlur(1.1))
# light falloff + vignette + sensor noise
px_ = img.load()
for v in range(H):
    for u in range(W):
        dx, dy = (u - W / 2) / (W / 2), (v - H / 2) / (H / 2)
        k = 1.0 - 0.38 * (dx * dx + dy * dy) ** 1.1
        n = random.gauss(0, 3.2)
        r, g, b = px_[u, v]
        px_[u, v] = (max(0, min(255, int(r * k + n))), max(0, min(255, int(g * k + n))),
                     max(0, min(255, int(b * k + n * 1.2))))
img.save(sys.argv[1])
print("wrote", sys.argv[1], "focal", round(F, 1))
