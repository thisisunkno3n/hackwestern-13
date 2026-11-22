#include "anim_map.h"

#include <math.h>
#include <string.h>

/* Timings, ms. */
#define AM_PING_MS 450      /* new person ring */
#define AM_DOWN_EVERY 1400  /* down ring period */
#define AM_DOWN_MS 1000     /* one down ring's life */
#define AM_SCAN_EVERY 3000  /* scan period */
#define AM_SCAN_MS 1800     /* one sweep; the rest of the period is still */
#define AM_TRAIL_RAD 0.12f  /* the soft wedge behind the scan line, about 7 degrees */
#define AM_DRAW_MS 300      /* first-show draw-in, all pieces done by then */
#define AM_PIECE_MS 220     /* one piece's own draw-in */
#define AM_MARCH_MS 600     /* hidden dashes move one dash + gap in this time */
#define AM_FORGET_MS 1000   /* a person off the map this long is new when seen again */
#define AM_DOWN_HOLD_MS 500 /* a down that drops out this briefly keeps its ring rhythm */
#define AM_SHOW_GAP_MS 1000 /* a view not drawn this long draws in again */

#define AM_MAX 64 /* people remembered; the oldest unseen is dropped when full */

typedef struct {
    int id;
    int64_t first_ms, last_ms;    /* first and newest frame on the map */
    int64_t down_ms, down_last;   /* when the current down began, newest down frame (0: never) */
} am_person;

static int g_on = 1;
static int64_t g_now;        /* this frame's time, 0 = no clock */
static am_person g_tab[AM_MAX];
static int g_n;
static int64_t g_any_last;   /* newest frame any plan was drawn (0: never) */
static int64_t g_full_last;  /* newest frame the full-screen plan was drawn (0: never) */
static int64_t g_inset_show, g_full_show;
static int64_t g_show;       /* first-show time of the view being drawn */
static int g_header;         /* view being drawn: 1 inset, 0 full screen */

static int live(void)
{
    return g_on && g_now > 0;
}

void anim_map_set_enabled(int on)
{
    g_on = on != 0;
}

int anim_map_enabled(void)
{
    return g_on;
}

void anim_map_frame(int64_t now_ms)
{
    if (now_ms < g_now) /* time went back (a new shot sequence): start over */
        anim_map_reset();
    g_now = now_ms > 0 ? now_ms : 0;
}

void anim_map_reset(void)
{
    memset(g_tab, 0, sizeof g_tab);
    g_n = 0;
    g_now = g_any_last = g_full_last = g_inset_show = g_full_show = g_show = 0;
    g_header = 0;
}

/* The person's map colour, the same choice plan.c makes for the square: the one place to change
 * when the plan palette changes. */
static uint32_t am_colour(const plan_mark *m)
{
    return m->close ? P_RED : m->kind == PLAN_SEEN ? P_INK : P_HIDDEN;
}

static float clamp01(float v)
{
    return v < 0 ? 0 : v > 1 ? 1 : v;
}

static float ease_out(float p) /* cubic */
{
    float q = 1.0f - clamp01(p);
    return 1.0f - q * q * q;
}

static am_person *find(int id)
{
    for (int i = 0; i < g_n; i++)
        if (g_tab[i].id == id)
            return &g_tab[i];
    return NULL;
}

void anim_map_begin(int header, const plan_mark *marks, int n)
{
    if (!live())
        return;
    /* First show: the inset after no plan at all for a while (boot); the full-screen plan
     * whenever it was not up a moment ago (the camera pausing). */
    if (header) {
        if (!g_any_last || g_now - g_any_last > AM_SHOW_GAP_MS)
            g_inset_show = g_now;
        g_show = g_inset_show;
    } else {
        if (!g_full_last || g_now - g_full_last > AM_SHOW_GAP_MS)
            g_full_show = g_now;
        g_show = g_full_show;
        g_full_last = g_now;
    }
    g_any_last = g_now;
    g_header = header;

    /* Forget people gone for a while: their id seen again (or reused) pings as new. */
    int k = 0;
    for (int i = 0; i < g_n; i++)
        if (g_now - g_tab[i].last_ms <= AM_FORGET_MS)
            g_tab[k++] = g_tab[i];
    g_n = k;
    for (int i = 0; i < n; i++) {
        const plan_mark *m = &marks[i];
        if (m->id <= 0 || m->kind == PLAN_RAY)
            continue;
        am_person *p = find(m->id);
        if (!p) {
            if (g_n < AM_MAX)
                p = &g_tab[g_n++];
            else { /* full: reuse the longest unseen */
                p = &g_tab[0];
                for (int j = 1; j < g_n; j++)
                    if (g_tab[j].last_ms < p->last_ms)
                        p = &g_tab[j];
            }
            memset(p, 0, sizeof *p);
            p->id = m->id;
            p->first_ms = g_now;
        }
        p->last_ms = g_now;
        if (m->down) {
            if (!p->down_last || g_now - p->down_last > AM_DOWN_HOLD_MS)
                p->down_ms = g_now;
            p->down_last = g_now;
        }
    }
}

float anim_map_draw_in(int part, int index, int count)
{
    if (!live())
        return 1.0f;
    float start = part == ANIM_MAP_PHONE ? 40.0f
                  : part == ANIM_MAP_WALL && count > 1
                      ? (float)(AM_DRAW_MS - AM_PIECE_MS) * (float)index / (float)(count - 1)
                      : 0.0f;
    float p = ((float)(g_now - g_show) - start) / (float)AM_PIECE_MS;
    return p >= 1.0f ? 1.0f : ease_out(p);
}

float anim_map_grow(int part, float full, float shown)
{
    float k = anim_map_draw_in(part, 0, 1);
    return k >= 1.0f ? full : fminf(full, shown) * k;
}

static int S(float u, float v)
{
    return (int)lroundf(v * u);
}

int anim_map_scale_bar(canvas *c, float u, int sx, int sy, int seg, int bh)
{
    if (!live())
        return 0;
    float p = ((float)(g_now - g_show) - 60.0f) / (float)(AM_DRAW_MS - 60);
    if (p >= 1.0f)
        return 0;
    float k = ease_out(p);
    /* The solid metre fills left to right, then the open metre's outline follows. */
    int w1 = (int)lroundf((float)seg * clamp01(k * 2.0f));
    float w2 = (float)seg * clamp01(k * 2.0f - 1.0f);
    if (w1 > 0)
        draw_fill_a(c, sx, sy, w1, bh, P_INK, 255);
    if (w2 > 1.0f) {
        draw_poly_style bs = { 0 };
        bs.edge = P_INK;
        bs.edge_a = 255;
        bs.edge_w = 1.0f * u;
        draw_cut_rect(c, (float)(sx + seg), (float)sy, w2, (float)bh, 0, &bs);
    }
    int lp = S(u, 11), ly = sy + S(u, 8), a = (int)lroundf((float)P_INK2_A * k);
    draw_ui_text_top(c, sx - S(u, 3), ly, lp, DRAW_FONT_MONO_REGULAR, P_INK, a, "0");
    if (k > 0.5f)
        draw_ui_text_top(c, sx + seg - S(u, 3), ly, lp, DRAW_FONT_MONO_REGULAR, P_INK, a, "1");
    if (w2 >= (float)seg * 0.99f)
        draw_ui_text_top(c, sx + 2 * seg - S(u, 3), ly, lp, DRAW_FONT_MONO_REGULAR, P_INK, a, "2 m");
    return 1;
}

/* Anti-aliased butt-capped line that only visits pixels near it (draw_line_aa walks the whole
 * bounding box, too slow for a long diagonal), clipped to the canvas. */
static void am_line(canvas *c, float x0, float y0, float x1, float y1, float width, uint32_t col,
                    int alpha)
{
    float dx = x1 - x0, dy = y1 - y0, len = sqrtf(dx * dx + dy * dy);
    if (alpha <= 0 || len < 0.5f)
        return;
    float ux = dx / len, uy = dy / len, hw = width * 0.5f;
    int rows = fabsf(dy) >= fabsf(dx); /* step along the longer axis */
    float slope = rows ? dx / dy : dy / dx, span = (hw + 1.5f) / (rows ? fabsf(uy) : fabsf(ux));
    float a0 = rows ? fminf(y0, y1) : fminf(x0, x1), a1 = rows ? fmaxf(y0, y1) : fmaxf(x0, x1);
    int lim = rows ? c->h : c->w, olim = rows ? c->w : c->h;
    int j0 = (int)floorf(a0 - hw - 1), j1 = (int)ceilf(a1 + hw + 1);
    if (j0 < 0) j0 = 0;
    if (j1 > lim) j1 = lim;
    for (int j = j0; j < j1; j++) {
        float m = (float)j + 0.5f;
        float o = rows ? x0 + (m - y0) * slope : y0 + (m - x0) * slope;
        int i0 = (int)floorf(o - span), i1 = (int)ceilf(o + span);
        if (i0 < 0) i0 = 0;
        if (i1 > olim) i1 = olim;
        for (int i = i0; i < i1; i++) {
            float px = (rows ? (float)i : (float)j) + 0.5f - x0;
            float py = (rows ? (float)j : (float)i) + 0.5f - y0;
            float along = px * ux + py * uy, across = fabsf(px * uy - py * ux);
            float end = fmaxf(-along, along - len);
            float cov = fminf(clamp01(hw + 0.5f - across), clamp01(0.5f - end));
            if (cov > 0) {
                uint32_t *d = rows ? &c->px[(size_t)j * (size_t)c->stride + (size_t)i]
                                   : &c->px[(size_t)i * (size_t)c->stride + (size_t)j];
                *d = draw_mix(*d, col, (int)((float)alpha * cov + 0.5f));
            }
        }
    }
}

/* Soft trail behind the scan line: the wedge between headings a0 and a1 (radians clockwise
 * from up) out of the apex, clear at a0 rising to `alpha` at a1. Rows only, so only for a cone
 * pointing up (both headings within 78 degrees of up); a wider cone just has the line. */
static void am_trail(canvas *c, float ax, float ay, float a0, float a1, float far, int alpha)
{
    float c0 = cosf(a0), c1 = cosf(a1);
    if (alpha <= 0 || c0 < 0.2f || c1 < 0.2f)
        return;
    float t0 = sinf(a0) / c0, t1 = sinf(a1) / c1; /* x per row up, each edge */
    int j1 = (int)floorf(ay), j0 = (int)floorf(ay - far * fminf(c0, c1));
    if (j0 < 0) j0 = 0;
    if (j1 > c->h) j1 = c->h;
    for (int j = j0; j < j1; j++) {
        float up = ay - ((float)j + 0.5f), xa = ax + up * t0, xb = ax + up * t1;
        if (xb - xa < 1.0f)
            continue;
        int i0 = (int)ceilf(xa - 0.5f), i1 = (int)floorf(xb - 0.5f);
        if (i0 < 0) i0 = 0;
        if (i1 >= c->w) i1 = c->w - 1;
        uint32_t *row = c->px + (size_t)j * (size_t)c->stride;
        float inv = 1.0f / (xb - xa);
        for (int i = i0; i <= i1; i++) {
            float f = ((float)i + 0.5f - xa) * inv;
            int al = (int)((float)alpha * f * f + 0.5f);
            if (al > 0)
                row[i] = draw_mix(row[i], P_INK, al);
        }
    }
}

void anim_map_scan(canvas *v, float u, float ax, float ay, float half, float far)
{
    if (!live() || half <= 0)
        return;
    int64_t e = g_now - (g_show + AM_DRAW_MS); /* the first sweep starts as the draw-in ends */
    if (e < 0)
        return;
    int64_t ph = e % AM_SCAN_EVERY;
    if (ph >= AM_SCAN_MS)
        return;
    float p = (float)ph / (float)AM_SCAN_MS;
    float s = p * p * (3.0f - 2.0f * p);            /* smoothstep across the cone */
    float env = sinf(3.14159265f * p);              /* fades in and out at the cone edges */
    float a = -half + 2.0f * half * s;
    am_trail(v, ax, ay, fmaxf(a - AM_TRAIL_RAD, -half), a, far, (int)lroundf(24.0f * env)); /* inside the cone */
    am_line(v, ax, ay, ax + far * sinf(a), ay - far * cosf(a), 1.4f * u, P_INK, (int)lroundf(60.0f * env));
}

void anim_map_rings(canvas *v, float u, const plan_mark *m, float px, float py)
{
    if (!live() || m->id <= 0 || m->kind == PLAN_RAY)
        return;
    const am_person *p = find(m->id);
    if (!p)
        return;
    uint32_t col = am_colour(m);
    float r0 = 7.5f * u, th = 1.5f * u;
    int down = m->down && p->down_ms > 0;
    /* New on the map: one ring (a person already down when first seen gets the down ring). */
    int64_t e = g_now - p->first_ms;
    if (e >= 0 && e < AM_PING_MS && !(down && p->down_ms == p->first_ms)) {
        float q = (float)e / (float)AM_PING_MS, r = r0 + (g_header ? 16.0f : 24.0f) * u * ease_out(q);
        draw_circle_aa(v, px, py, r, th, 0, 0, col, (int)lroundf(235.0f * powf(1.0f - q, 1.4f)));
    }
    /* Down: a slow ring from the moment they went down, over and over. */
    if (down) {
        int64_t d = (g_now - p->down_ms) % AM_DOWN_EVERY;
        if (d >= 0 && d < AM_DOWN_MS) {
            float q = (float)d / (float)AM_DOWN_MS, r = r0 + (g_header ? 20.0f : 30.0f) * u * ease_out(q);
            draw_circle_aa(v, px, py, r, th, 0, 0, col, (int)lroundf(220.0f * (1.0f - q) * (1.0f - q)));
        }
    }
}

/* Stretch [lo, hi] of the square's perimeter (clockwise from the top left), side by side; a
 * piece that reaches a corner runs half a line width past it so the bend is closed. */
static void am_perimeter(canvas *v, float x, float y, float s, float lo, float hi, float width,
                         uint32_t col, int alpha)
{
    static const float kx[5] = { 0, 1, 1, 0, 0 }, ky[5] = { 0, 0, 1, 1, 0 };
    float hw = width * 0.5f / s;
    for (int side = 0; side < 4; side++) {
        float t0 = (fmaxf(lo, s * (float)side) - s * (float)side) / s;
        float t1 = (fminf(hi, s * (float)(side + 1)) - s * (float)side) / s;
        if (t1 <= t0)
            continue;
        if (t0 <= 0) t0 = -hw;
        if (t1 >= 1) t1 = 1 + hw;
        float ex = kx[side + 1] - kx[side], ey = ky[side + 1] - ky[side];
        float bx = x + s * kx[side], by = y + s * ky[side];
        draw_line_aa(v, bx + ex * s * t0, by + ey * s * t0, bx + ex * s * t1, by + ey * s * t1, width,
                     DRAW_CAP_BUTT, col, alpha);
    }
}

int anim_map_hidden_rect(canvas *v, float x, float y, float s, float width, float dash, float gap,
                         uint32_t col, int alpha)
{
    float period = dash + gap;
    if (!live() || s <= 0 || period <= 0)
        return 0;
    /* One dash pattern walked round the whole square, shifted by the time; a dash running over a
     * corner bends round it, the one cut by the start wraps to the end. */
    float P = 4.0f * s, off = (float)(g_now % AM_MARCH_MS) / (float)AM_MARCH_MS * period;
    for (float a = off - period; a < P; a += period) {
        float b = fminf(a + dash, P);
        if (a < 0) {
            am_perimeter(v, x, y, s, a + P, P, width, col, alpha);
            am_perimeter(v, x, y, s, 0, b, width, col, alpha);
        } else
            am_perimeter(v, x, y, s, a, b, width, col, alpha);
    }
    return 1;
}
