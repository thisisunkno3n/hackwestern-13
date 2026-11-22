"""Camera frames and the scene timeline for hud_shot --demo (host only, needs Pillow):

  python3 tools/hud_demo_frames.py DIR

Writes DIR/scene.txt and DIR/f000.ppm ... (30 fps, 1280x720). One line per frame in scene.txt:
  t_ms seen seen_x seen_y hidden hidden_down
The story (7.5 s): an empty room with the plan up; a person R1 sees walks into view at 0.3 s;
someone appears behind the wall (seen by R2) at 1.6 s; the seen person walks in close from
3.0 s to 4.8 s; the person behind the wall goes down at 5.4 s. hud_shot draws the HUD over each
frame from the same timeline, so the picture and the tracks agree."""
import os, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import hud_shot_room as room

FPS, SECONDS = 30, 7.5
APPEAR, HIDDEN, WALK0, WALK1, DOWN = 300, 1600, 3000, 4800, 5400
FAR, NEAR = (-1.3, 2.3), (-0.45, -0.55)


def smooth(t):
    return t * t * (3 - 2 * t)


def at(t):
    """(seen, x, y, hidden, hidden_down) at t ms."""
    seen = t >= APPEAR
    a = 0.0 if t < WALK0 else 1.0 if t >= WALK1 else smooth((t - WALK0) / (WALK1 - WALK0))
    x = FAR[0] + (NEAR[0] - FAR[0]) * a
    y = FAR[1] + (NEAR[1] - FAR[1]) * a
    return seen, x, y, t >= HIDDEN, t >= DOWN


def main():
    out = sys.argv[1]
    os.makedirs(out, exist_ok=True)
    n = int(FPS * SECONDS)
    lines = []
    cache = {}
    for i in range(n):
        t = round(i * 1000 / FPS)
        seen, x, y, hid, down = at(t)
        key = (seen, round(x, 4), round(y, 4))
        if key not in cache:  # the picture only changes while someone moves
            cache.clear()
            cache[key] = room.frame((x, y, False) if seen else None, seed=i)
        cache[key].save(os.path.join(out, "f%03d.ppm" % i))
        lines.append("%d %d %.4f %.4f %d %d" % (t, seen, x, y, hid, down))
    with open(os.path.join(out, "scene.txt"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("wrote %d frames and scene.txt to %s" % (n, out))


if __name__ == "__main__":
    main()
