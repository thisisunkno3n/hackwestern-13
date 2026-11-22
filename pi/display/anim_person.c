#include "anim_person.h"

#include <math.h>
#include <string.h>

/* Slot states: FREE, LIVE (drawn in view on a recent frame), ECHO (gone from view, its echo is
 * fading), MEMO (echo over, remembered for AP_FORGET_MS so a return does not replay the plate). */
enum { AP_FREE = 0, AP_LIVE, AP_ECHO, AP_MEMO };

typedef struct {
    int state;
    int id;
    int direct;      /* kind of the last drawn marks */
    int seen;        /* registered this frame */
    int stored;      /* item/track/box below are valid */
    int64_t born_ms; /* leader and plate timing */
    int64_t kind_ms; /* mark intro timing (restarts when seen / x-ray flips) */
    int64_t last_ms; /* last frame it was drawn in view */
    wv_item it;
    seeker_track k;
    int box[4];
} ap_slot;

static int g_on = 1;
static ap_slot g_slot[AP_SLOTS];
static int64_t g_now;
static float g_march;

void anim_person_reset(void)
{
    memset(g_slot, 0, sizeof g_slot);
    g_now = 0;
    g_march = 0;
}

void anim_person_set_enabled(int on)
{
    g_on = on ? 1 : 0;
    anim_person_reset();
}

int anim_person_enabled(void)
{
    return g_on;
}

static float clamp01(float v)
{
    return v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v;
}

static float ease_out(float t) /* cubic */
{
    t = clamp01(t);
    float a = 1.0f - t;
    return 1.0f - a * a * a;
}

static float prog(int64_t from, int64_t start_ms, int dur_ms)
{
    return clamp01((float)(g_now - from - start_ms) / (float)dur_ms);
}

void anim_person_begin(int64_t now_ms, float u)
{
    if (!g_on)
        return;
    /* A clock that runs backwards (a new scene in a tool, a test) starts the table over. */
    if (now_ms < g_now)
        anim_person_reset();
    g_now = now_ms;
    /* Dash periods in the plan marks are 17, 18 and 13 design px: wrapping the clock at the time
     * their common multiple (3978 px) takes keeps the phase continuous, in exact integer ms. */
    const int64_t wrap = (int64_t)(3978.0 * 1000.0 / AP_MARCH_PX_S + 0.5);
    int64_t m = now_ms % wrap;
    g_march = (float)(m < 0 ? m + wrap : m) * (AP_MARCH_PX_S / 1000.0f) * u;
    for (int i = 0; i < AP_SLOTS; i++)
        g_slot[i].seen = 0;
}

static ap_slot *find(int id)
{
    for (int i = 0; i < AP_SLOTS; i++)
        if (g_slot[i].state != AP_FREE && g_slot[i].id == id)
            return &g_slot[i];
    return NULL;
}

/* A slot for a new id: a free one, else the oldest remembered one, else the oldest echo (an echo
 * is still on screen, so it goes last). Live tracks are never evicted. */
static ap_slot *alloc(void)
{
    static const int order[3] = { AP_FREE, AP_MEMO, AP_ECHO };
    for (int o = 0; o < 3; o++) {
        ap_slot *best = NULL;
        for (int i = 0; i < AP_SLOTS; i++) {
            ap_slot *s = &g_slot[i];
            if (s->state == order[o] && (!best || s->last_ms < best->last_ms))
                best = s;
        }
        if (best)
            return best;
    }
    return NULL;
}

static ap_fx fx_of(const ap_slot *s)
{
    ap_fx f;
    memset(&f, 0, sizeof f);
    f.mark_a = 1.0f;
    f.trace = f.hatch = f.leader = f.plate = 1.0f;
    if (s->direct)
        f.trace = ease_out(prog(s->kind_ms, 0, AP_TRACE_MS));
    else {
        f.mark_a = AP_XRAY_A0 + (1.0f - AP_XRAY_A0) * ease_out(prog(s->kind_ms, 0, AP_XRAY_FADE_MS));
        f.hatch = ease_out(prog(s->kind_ms, 0, AP_HATCH_MS));
        f.march = g_march;
    }
    f.leader = ease_out(prog(s->born_ms, AP_TRACE_MS, AP_LEADER_MS));
    f.plate = ease_out(prog(s->born_ms, AP_TRACE_MS + AP_LEADER_MS, AP_PLATE_MS));
    f.active = f.trace < 1.0f || f.mark_a < 1.0f || f.hatch < 1.0f || f.leader < 1.0f || f.plate < 1.0f ||
               f.march != 0.0f;
    return f;
}

static ap_fx none(void)
{
    ap_fx f;
    memset(&f, 0, sizeof f);
    f.mark_a = f.trace = f.hatch = f.leader = f.plate = 1.0f;
    return f;
}

ap_fx anim_person_track(int id, int direct)
{
    if (!g_on)
        return none();
    direct = direct ? 1 : 0;
    ap_slot *s = find(id);
    if (s && s->state == AP_MEMO && g_now - s->last_ms > AP_FORGET_MS)
        s->state = AP_FREE; /* forgotten: a new arrival (also covers fusion reusing the id) */
    if (!s || s->state == AP_FREE) {
        s = s ? s : alloc();
        if (!s)
            return none(); /* table full of live tracks: plain drawing */
        memset(s, 0, sizeof *s);
        s->id = id;
        s->direct = direct;
        s->born_ms = s->kind_ms = g_now;
    } else if (s->state == AP_MEMO) {
        /* Back after its echo: the mark replays its short intro, the plate stays up. */
        s->born_ms = g_now - (AP_TRACE_MS + AP_LEADER_MS + AP_PLATE_MS);
        s->kind_ms = g_now;
        s->direct = direct;
    } else if (s->direct != direct) {
        s->direct = direct; /* walked behind a wall or out of it: new marks, same plate */
        s->kind_ms = g_now;
    }
    s->state = AP_LIVE;
    s->seen = 1;
    s->last_ms = g_now;
    return fx_of(s);
}

ap_fx anim_person_get(int id)
{
    if (!g_on)
        return none();
    ap_slot *s = find(id);
    return s && s->state == AP_LIVE && s->seen ? fx_of(s) : none();
}

void anim_person_store(int id, const wv_item *it, const seeker_track *k, const int box[4])
{
    if (!g_on)
        return;
    ap_slot *s = find(id);
    if (!s || s->state != AP_LIVE)
        return;
    s->it = *it;
    s->k = *k;
    memcpy(s->box, box, sizeof s->box);
    s->stored = 1;
}

void anim_person_end(void)
{
    if (!g_on)
        return;
    for (int i = 0; i < AP_SLOTS; i++) {
        ap_slot *s = &g_slot[i];
        if (s->state == AP_LIVE && !s->seen)
            s->state = AP_ECHO;
        if (s->state == AP_ECHO && g_now - s->last_ms >= AP_LOST_MS)
            s->state = AP_MEMO;
        if (s->state == AP_MEMO && g_now - s->last_ms > AP_FORGET_MS)
            s->state = AP_FREE;
    }
}

int anim_person_echo(int i, wv_item *it, seeker_track *k, int box[4], ap_fx *fx)
{
    if (!g_on || i < 0 || i >= AP_SLOTS)
        return 0;
    const ap_slot *s = &g_slot[i];
    if (s->state != AP_ECHO || !s->stored)
        return 0;
    float a = AP_ECHO_A0 * (1.0f - (float)(g_now - s->last_ms) / (float)AP_LOST_MS);
    if (a <= 0.0f)
        return 0;
    *it = s->it;
    *k = s->k;
    memcpy(box, s->box, sizeof s->box);
    *fx = none();
    fx->active = 1;
    fx->echo = 1;
    fx->mark_a = a;
    fx->march = s->direct ? 0.0f : g_march;
    return 1;
}

/* ---- drawing ---- */

int ap_alpha(const ap_fx *fx, int a)
{
    if (!fx || !fx->active)
        return a;
    return (int)lroundf((float)a * clamp01(fx->mark_a));
}

/* Straight runs of a seen frame, drawn halos first then colour, so no halo cuts a stroke. */
typedef struct {
    float x0, y0, x1, y1;
} ap_run;

/* Halo under one run: AP_HALO_W wider, reaching as far past each end as to the sides. */
static void halo(canvas *c, float u, const ap_run *r, float w, int a)
{
    float e = AP_HALO_W * u * 0.5f, dx = r->x1 - r->x0, dy = r->y1 - r->y0, L = sqrtf(dx * dx + dy * dy);
    if (L > 1e-4f)
        dx = dx / L * e, dy = dy / L * e;
    else
        dx = dy = 0;
    draw_line_aa(c, r->x0 - dx, r->y0 - dy, r->x1 + dx, r->y1 + dy, w + 2 * e, DRAW_CAP_BUTT, RGB(0, 0, 0),
                 AP_HALO_A * a / 255);
}

static void runs(canvas *c, float u, const ap_run *r, int n, float w, uint32_t col, int a)
{
    if (a <= 0)
        return;
    for (int i = 0; i < n; i++)
        halo(c, u, &r[i], w, a);
    for (int i = 0; i < n; i++)
        draw_line_aa(c, r[i].x0, r[i].y0, r[i].x1, r[i].y1, w, DRAW_CAP_BUTT, col, a);
}

void ap_seen_frame(canvas *c, float u, int xi, int yi, int wi, int hi, uint32_t col, int a, const ap_fx *fx)
{
    float x = (float)xi, y = (float)yi, w = (float)wi, h = (float)hi, sw = 2.0f * u, hw = sw * 0.5f;
    int am = ap_alpha(fx, a);
    float tr = fx->echo ? 1.0f : fx->trace;
    float my = y + h * 0.5f, t8 = 8 * u;
    ap_run r[10];
    int n = 0;
    if (tr >= 1.0f) {
        /* Closed (an echo): the plan frame and ticks, faded, on their halo. */
        draw_poly_style hs = { 0 };
        hs.edge = RGB(0, 0, 0);
        hs.edge_a = AP_HALO_A * am / 255;
        hs.edge_w = sw + AP_HALO_W * u;
        draw_cut_rect(c, x, y, w, h, 0, &hs);
        r[n++] = (ap_run){ x - t8, my, x + t8, my };
        r[n++] = (ap_run){ x + w - t8, my, x + w + t8, my };
        for (int i = 0; i < n; i++)
            halo(c, u, &r[i], sw, am);
        draw_poly_style fs = { 0 };
        fs.edge = col;
        fs.edge_a = am;
        fs.edge_w = sw;
        draw_cut_rect(c, x, y, w, h, 0, &fs);
        for (int i = 0; i < n; i++)
            draw_line_aa(c, r[i].x0, r[i].y0, r[i].x1, r[i].y1, sw, DRAW_CAP_BUTT, col, am);
        return;
    }
    /* The whole frame, faint, so its true size is there on the first frame. */
    draw_poly_style fs = { 0 };
    fs.edge = col;
    fs.edge_a = (int)lroundf((float)am * AP_FAINT_A);
    fs.edge_w = sw;
    draw_cut_rect(c, x, y, w, h, 0, &fs);
    /* Corner segments at full strength, growing along each edge to meet at its middle. The
     * horizontal run owns the corner square so the two runs of a corner never overlap. */
    float f = AP_TRACE_C0 + (1.0f - AP_TRACE_C0) * tr;
    float lx = w * 0.5f * f, ly = h * 0.5f * f;
    float xs[2] = { x, x + w }, ys[2] = { y, y + h };
    for (int cx = 0; cx < 2; cx++)
        for (int cy = 0; cy < 2; cy++) {
            float px = xs[cx], py = ys[cy], dx = cx ? -1.0f : 1.0f, dy = cy ? -1.0f : 1.0f;
            r[n++] = (ap_run){ px - dx * hw, py, px + dx * lx, py };
            r[n++] = (ap_run){ px, py + dy * hw, px, py + dy * ly };
        }
    /* Mid-side ticks from the first frame: they mark where the person's centre is. */
    r[n++] = (ap_run){ x - t8, my, x + t8, my };
    r[n++] = (ap_run){ x + w - t8, my, x + w + t8, my };
    runs(c, u, r, n, sw, col, am);
}

void ap_hatch_rect(canvas *c, float x, float y, float w, float h, const draw_poly_style *hs, const ap_fx *fx)
{
    if (!fx->active || fx->hatch >= 1.0f || fx->echo) {
        draw_cut_rect(c, x, y, w, h, 0, hs);
        return;
    }
    /* Keep the part of the rectangle with (px - x) + (y + h - py) <= k: a front at 45 degrees
     * across the hatch lines, moving from the bottom-left corner to the top-right one. That corner
     * stays in, so the polygon's left and bottom (draw_poly's hatch origin) never move and the
     * lines do not crawl while the front sweeps. */
    float k = fx->hatch * (w + h);
    if (k <= 0.5f)
        return;
    float in[8] = { x, y + h, x, y, x + w, y, x + w, y + h }, out[16];
    int n = 0;
    for (int i = 0; i < 4; i++) {
        float ax = in[2 * i], ay = in[2 * i + 1], bx = in[2 * ((i + 1) % 4)], by = in[2 * ((i + 1) % 4) + 1];
        float fa = (ax - x) + (y + h - ay) - k, fb = (bx - x) + (y + h - by) - k;
        if (fa <= 0) {
            out[2 * n] = ax, out[2 * n + 1] = ay, n++;
        }
        if ((fa < 0) != (fb < 0) && fa != fb) {
            float t = fa / (fa - fb);
            out[2 * n] = ax + t * (bx - ax), out[2 * n + 1] = ay + t * (by - ay), n++;
        }
    }
    if (n >= 3)
        draw_poly(c, out, n, hs);
}

void ap_leader(canvas *c, float u, float x0, float y0, float x1, float y1, uint32_t col, const ap_fx *fx)
{
    if (!fx->active || fx->leader >= 1.0f) {
        plan_leader(c, u, x0, y0, x1, y1, col);
        return;
    }
    if (fx->leader <= 0.0f)
        return;
    /* plan_leader's path (45 degree run then level, or straight first when too steep), drawn up
     * to `leader` of its length from the person. */
    float lw = 1.5f * u, dy = fabsf(y1 - y0), dx = fabsf(x1 - x0), sx = x1 >= x0 ? 1.0f : -1.0f;
    float kx, ky;
    if (dy <= dx)
        kx = x0 + sx * dy, ky = y1;
    else
        kx = x0, ky = y1 + (y0 < y1 ? -dx : dx);
    float l1 = hypotf(kx - x0, ky - y0), l2 = hypotf(x1 - kx, y1 - ky), d = fx->leader * (l1 + l2);
    ap_run r[2];
    int n = 0;
    if (d <= l1) {
        float t = l1 > 0 ? d / l1 : 1.0f;
        r[n++] = (ap_run){ x0, y0, x0 + t * (kx - x0), y0 + t * (ky - y0) };
    } else {
        float t = l2 > 0 ? (d - l1) / l2 : 1.0f;
        r[n++] = (ap_run){ x0, y0, kx, ky };
        r[n++] = (ap_run){ kx, ky, kx + t * (x1 - kx), ky + t * (y1 - ky) };
    }
    /* Halos first (as on the person strokes), then the line and the start square. */
    int s = (int)lroundf(6 * u), hs = s + (int)lroundf(AP_HALO_W * u);
    for (int i = 0; i < n; i++)
        draw_line_aa(c, r[i].x0, r[i].y0, r[i].x1, r[i].y1, lw + AP_HALO_W * u, DRAW_CAP_SQUARE, RGB(0, 0, 0),
                     AP_HALO_A);
    draw_fill_a(c, (int)lroundf(x0) - hs / 2, (int)lroundf(y0) - hs / 2, hs, hs, RGB(0, 0, 0), AP_HALO_A);
    for (int i = 0; i < n; i++)
        draw_line_aa(c, r[i].x0, r[i].y0, r[i].x1, r[i].y1, lw, DRAW_CAP_SQUARE, col, 255);
    draw_fill_a(c, (int)lroundf(x0) - s / 2, (int)lroundf(y0) - s / 2, s, s, col, 255);
}

void ap_plate(canvas *c, float u, int x, int y, const plan_line *l, int n, uint32_t edge, int red_hatch,
              int sdx, int sdy, const ap_fx *fx)
{
    if (!fx->active || fx->plate >= 1.0f) {
        plan_plate(c, u, x, y, l, n, edge, red_hatch);
        return;
    }
    if (fx->plate <= 0.0f)
        return;
    float back = (1.0f - fx->plate) * AP_SLIDE_PX * u;
    plan_plate_a(c, u, x + (int)lroundf((float)sdx * back), y + (int)lroundf((float)sdy * back), l, n, edge,
                 red_hatch, (int)lroundf(255.0f * fx->plate));
}
