#include "anim_alert.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

static int g_on = 1;
static int64_t g_now;     /* this frame's time, 0 = none */
static unsigned g_frame;  /* frames since the last reset */

static int live(void)
{
    return g_on && g_now > 0;
}

static float clamp01(float v)
{
    return v < 0 ? 0 : v > 1 ? 1 : v;
}

static float ease_out(float t) /* cubic: fast start, gentle landing */
{
    float s = 1.0f - clamp01(t);
    return 1.0f - s * s * s;
}

static int S(float u, float v)
{
    return (int)lroundf(v * u);
}

/* ---- per-track state ---- */

typedef struct {
    int id; /* 0 = free */
    int64_t seen_ms;
    int alert, down;
    int64_t alert_ms; /* alert became true */
    int64_t leave_ms; /* alert became false (0 = not leaving) */
    int64_t down_ms;  /* down became true */
    int skip_sweep;   /* re-entered while the hatch was still fading: keep it full */
    unsigned frame;   /* last frame the state was updated */
} track_slot;

static track_slot g_tracks[ANIM_ALERT_SLOTS];

/* ---- title block cells ---- */

typedef struct {
    int have;
    char cur[64], old[64];
    uint32_t cur_col, old_col;
    int64_t t; /* when old was replaced */
} cell_slot;

static cell_slot g_cells[5];

static struct {
    int late;
    int64_t on_ms, off_ms;
} g_late;

/* ---- warning plates ---- */

#define WARN_SLOTS 8
typedef struct {
    int used;
    char text[2][96];
    plan_line l[2];
    int n, x, y;
    uint32_t edge;
    int64_t in_ms, out_ms;
    unsigned seen;
} warn_slot;

static warn_slot g_warn[WARN_SLOTS];

void anim_alert_set_enabled(int on)
{
    g_on = on != 0;
}

int anim_alert_enabled(void)
{
    return g_on;
}

void anim_alert_reset(void)
{
    memset(g_tracks, 0, sizeof g_tracks);
    memset(g_cells, 0, sizeof g_cells);
    memset(&g_late, 0, sizeof g_late);
    memset(g_warn, 0, sizeof g_warn);
    g_now = 0;
    g_frame = 0;
}

void anim_alert_frame(int64_t now_ms)
{
    if (now_ms > 0 && now_ms < g_now) { /* time went back: start over */
        anim_alert_reset();
    }
    g_now = now_ms > 0 ? now_ms : 0;
    g_frame++;
    if (!g_now)
        return;
    for (int i = 0; i < ANIM_ALERT_SLOTS; i++)
        if (g_tracks[i].id && g_now - g_tracks[i].seen_ms > ANIM_FORGET_MS)
            g_tracks[i].id = 0;
}

anim_fx anim_alert_track(int id, int alert, int down)
{
    anim_fx f = { 0, alert, down && alert, 0, 1, -1, 1 };
    if (!live() || id <= 0)
        return f;
    track_slot *s = NULL, *spare = NULL;
    for (int i = 0; i < ANIM_ALERT_SLOTS && !s; i++) {
        if (g_tracks[i].id == id)
            s = &g_tracks[i];
        else if (!g_tracks[i].id && !spare)
            spare = &g_tracks[i];
    }
    if (!s) {
        if (!spare)
            return f; /* table full: plain look for this one */
        memset(spare, 0, sizeof *spare);
        spare->id = id;
        s = spare;
    }
    s->seen_ms = g_now;
    down = down && alert;
    if (s->frame != g_frame || alert != s->alert || down != s->down) {
        if (alert && !s->alert) {
            s->skip_sweep = s->leave_ms && g_now - s->leave_ms < ANIM_LEAVE_MS;
            s->alert_ms = g_now;
            s->leave_ms = 0;
        } else if (!alert && s->alert) {
            s->leave_ms = g_now;
        }
        if (down && !s->down)
            s->down_ms = g_now;
        s->alert = alert;
        s->down = down;
        s->frame = g_frame;
    }
    f.on = 1;
    if (alert) {
        float p = (float)(g_now - s->alert_ms) / ANIM_HATCH_MS;
        f.hatch = s->skip_sweep ? 1.0f : p >= 1 ? 1.0f : ANIM_HATCH_START + (1 - ANIM_HATCH_START) * ease_out(p);
    } else if (s->leave_ms) {
        int64_t lt = g_now - s->leave_ms;
        if (lt < ANIM_LEAVE_MS)
            f.leave = 1.0f - (float)lt / ANIM_LEAVE_MS;
        else
            s->leave_ms = 0;
    }
    if (down) {
        int64_t dt = g_now - s->down_ms;
        if (dt < ANIM_RING_MS) {
            f.ring = (float)dt / ANIM_RING_MS;
        } else {
            float ph = (float)(dt - ANIM_RING_MS) / ANIM_PULSE_MS * 6.28318531f;
            f.pulse = 1.0f - (1.0f - ANIM_PULSE_MIN) * 0.5f * (1.0f - cosf(ph)); /* 1 at the start */
        }
    }
    return f;
}

uint32_t anim_alert_tint(const anim_fx *f, uint32_t col)
{
    if (!f || !f->on || f->alert || f->leave <= 0)
        return col;
    return draw_mix(col, P_ALERT, (int)lroundf(f->leave * 255.0f));
}

int anim_alert_alpha(const anim_fx *f, int alpha)
{
    if (!f || !f->on || f->pulse >= 1.0f)
        return alpha;
    return (int)lroundf((float)alpha * f->pulse);
}

/* Rectangle outline of width `ew` with its halo. */
static void outline(canvas *c, float u, float x, float y, float w, float h, float ew, uint32_t col, int a)
{
    if (a <= 0)
        return;
    draw_poly_style st = { 0 };
    st.edge = col;
    st.edge_a = a;
    st.edge_w = ew;
    plan_halo_style(&st, u, a);
    draw_cut_rect(c, x, y, w, h, 0, &st);
}

void anim_alert_ring(canvas *c, float u, const anim_fx *f, float x, float y, float w, float h, float k)
{
    if (!f || !f->on || f->ring < 0)
        return;
    /* Out from 6 px to 40 px off the frame, fading from 90%. */
    float d = (6.0f + 34.0f * ease_out(f->ring)) * u, fade = 1.0f - f->ring;
    outline(c, u, x - d, y - d, w + 2 * d, h + 2 * d, 2.0f * u, P_ALERT,
            (int)lroundf(230.0f * fade * sqrtf(fade) * k));
}

void anim_alert_hull(canvas *c, float u, const anim_fx *f, float x, float y, float w, float h, float k)
{
    if (!f || !f->on || !f->down)
        return;
    /* One solid outline (no halo, no dashes): the hull's own dashes are already on their halo. */
    draw_poly_style st = { 0 };
    st.edge = P_ALERT;
    st.edge_a = (int)lroundf((float)anim_alert_alpha(f, 255) * k);
    st.edge_w = 2.0f * u;
    if (st.edge_a > 0)
        draw_cut_rect(c, x, y, w, h, 0, &st);
    anim_alert_ring(c, u, f, x, y, w, h, k);
}

/* Convex polygon clipped to the half-plane (px - x0) + (y1 - py) <= k: a front at 45 degrees,
 * across the hatch lines, growing from the bottom-left corner (x0, y1). That corner is always
 * inside, so draw_poly's hatch phase (anchored at the bounding box's left and bottom) stays put
 * and the lines grow in place instead of crawling. */
static int clip_sweep(const float *in, int n, float x0, float y1, float k, float *out)
{
    int m = 0;
    for (int i = 0; i < n; i++) {
        const float *a = in + 2 * i, *b = in + 2 * ((i + 1) % n);
        float da = (a[0] - x0) + (y1 - a[1]) - k, db = (b[0] - x0) + (y1 - b[1]) - k;
        if (da <= 0) {
            out[2 * m] = a[0], out[2 * m + 1] = a[1];
            m++;
        }
        if ((da < 0 && db > 0) || (da > 0 && db < 0)) {
            float t = da / (da - db);
            out[2 * m] = a[0] + t * (b[0] - a[0]);
            out[2 * m + 1] = a[1] + t * (b[1] - a[1]);
            m++;
        }
    }
    return m;
}

void anim_alert_hatch_rect(canvas *c, float x, float y, float w, float h, float cut,
                           const draw_poly_style *hs, float sweep, float k)
{
    if (sweep >= 1.0f && k >= 1.0f) {
        draw_cut_rect(c, x, y, w, h, cut, hs);
        return;
    }
    draw_poly_style st = *hs;
    st.hatch_a = (int)lroundf((float)hs->hatch_a * clamp01(k));
    if (st.hatch_a <= 0 || sweep <= 0)
        return;
    float p[10];
    int n;
    if (cut > 0) {
        float q[10] = { x, y, x + w - cut, y, x + w, y + cut, x + w, y + h, x, y + h };
        memcpy(p, q, sizeof q);
        n = 5;
    } else {
        float q[8] = { x, y, x + w, y, x + w, y + h, x, y + h };
        memcpy(p, q, sizeof q);
        n = 4;
    }
    if (sweep >= 1.0f) {
        draw_poly(c, p, n, &st);
        return;
    }
    float o[14];
    int m = clip_sweep(p, n, x, y + h, sweep * (w + h), o);
    if (m >= 3)
        draw_poly(c, o, m, &st);
}

/* plan_plate in layers (halo, fill, hatch with sweep and fade, edge, words), all scaled by
 * alpha: for plates in a transition. Steady plates go through plan_plate itself. */
static void plate_draw(canvas *c, float u, int x, int y, const plan_line *l, int n, uint32_t edge,
                       float sweep, float k, int alpha)
{
    int w, h;
    plan_plate_size(u, l, n, &w, &h);
    float X = (float)x, Y = (float)y, W = (float)w, H = (float)h, cut = 10 * u;
    plan_halo_rect(c, u, X, Y, W, H, cut, 1.5f * u, alpha);
    draw_poly_style fs = { 0 };
    fs.fill = P_PLATE;
    fs.fill_a = P_PLATE_A * alpha / 255;
    draw_cut_rect(c, X, Y, W, H, cut, &fs);
    if (k > 0) {
        draw_poly_style hs = { 0 };
        hs.hatch = P_ALERT;
        hs.hatch_a = 71 * alpha / 255; /* plan_plate's 28% */
        hs.hatch_gap = 9 * u;
        hs.hatch_w = 2 * u;
        anim_alert_hatch_rect(c, X, Y, W, H, cut, &hs, sweep, k);
    }
    draw_poly_style es = { 0 };
    es.edge = edge;
    es.edge_a = alpha;
    es.edge_w = 1.5f * u;
    draw_cut_rect(c, X, Y, W, H, cut, &es);
    int ty = y + S(u, 9);
    for (int i = 0; i < n; i++) {
        int px = S(u, (float)l[i].size);
        draw_ui_text_top(c, x + S(u, 12), ty, px, l[i].font, l[i].col, l[i].alpha * alpha / 255, l[i].text);
        ty += (int)lroundf((float)px * 1.15f) + S(u, 3);
    }
}

void anim_alert_plate(canvas *c, float u, int x, int y, const plan_line *l, int n, uint32_t edge,
                      int red_hatch, const anim_fx *f)
{
    float sweep = 1.0f, k = 0.0f;
    if (f && f->on) {
        if (red_hatch) {
            sweep = f->alert ? f->hatch : 1.0f;
            k = 1.0f;
        } else if (f->leave > 0) {
            k = f->leave;
        }
    }
    if ((red_hatch && sweep >= 1.0f) || (!red_hatch && k <= 0)) {
        plan_plate(c, u, x, y, l, n, edge, red_hatch);
        return;
    }
    plate_draw(c, u, x, y, l, n, edge, sweep, k, 255);
}

/* ---- warning plates ---- */

void anim_alert_warn(canvas *c, float u, int x, int y, const plan_line *l, int n, uint32_t edge)
{
    if (!live() || n < 1 || n > 2) {
        plan_plate(c, u, x, y, l, n, edge, 0);
        return;
    }
    warn_slot *s = NULL, *spare = NULL, *oldest = NULL;
    for (int i = 0; i < WARN_SLOTS && !s; i++) {
        warn_slot *w = &g_warn[i];
        if (w->used && !strcmp(w->text[0], l[0].text))
            s = w;
        else if (!w->used && !spare)
            spare = w;
        else if (w->used && (!oldest || w->seen < oldest->seen))
            oldest = w;
    }
    int fresh = !s || s->out_ms || s->seen + 1 < g_frame;
    if (!s)
        s = spare ? spare : oldest;
    if (fresh) {
        memset(s, 0, sizeof *s);
        s->used = 1;
        s->in_ms = g_now;
    }
    s->n = n;
    for (int i = 0; i < n; i++) {
        snprintf(s->text[i], sizeof s->text[i], "%s", l[i].text ? l[i].text : "");
        s->l[i] = l[i];
        s->l[i].text = s->text[i];
    }
    s->x = x, s->y = y, s->edge = edge, s->seen = g_frame;
    float p = (float)(g_now - s->in_ms) / ANIM_PLATE_MS;
    if (p >= 1.0f) {
        plan_plate(c, u, x, y, l, n, edge, 0);
        return;
    }
    float e = ease_out(p);
    int a = (int)lroundf(255.0f * (ANIM_PLATE_START + (1.0f - ANIM_PLATE_START) * e));
    plate_draw(c, u, x, y - (int)lroundf(ANIM_PLATE_DY * u * (1.0f - e)), l, n, edge, 1, 0, a);
}

void anim_alert_warn_flush(canvas *c, float u)
{
    if (!live())
        return;
    for (int i = 0; i < WARN_SLOTS; i++) {
        warn_slot *s = &g_warn[i];
        if (!s->used || s->seen == g_frame)
            continue;
        if (!s->out_ms)
            s->out_ms = g_now;
        float p = (float)(g_now - s->out_ms) / ANIM_PLATE_MS;
        if (p >= 1.0f) {
            s->used = 0;
            continue;
        }
        float e = ease_out(p);
        int a = (int)lroundf(255.0f * (1.0f - p));
        plate_draw(c, u, s->x, s->y - (int)lroundf(ANIM_PLATE_DY * u * e), s->l, s->n, s->edge, 1, 0, a);
    }
}

/* ---- title block ---- */

int anim_alert_late(int late, float *sweep, float *k)
{
    *sweep = 1.0f;
    *k = 1.0f;
    if (!live())
        return late;
    if (late && !g_late.late)
        g_late.on_ms = g_now, g_late.off_ms = 0;
    else if (!late && g_late.late)
        g_late.off_ms = g_now;
    g_late.late = late;
    if (late) {
        float p = (float)(g_now - g_late.on_ms) / ANIM_HATCH_MS;
        *sweep = p >= 1 ? 1.0f : ANIM_HATCH_START + (1 - ANIM_HATCH_START) * ease_out(p);
        return 1;
    }
    if (g_late.off_ms && g_now - g_late.off_ms < ANIM_LEAVE_MS) {
        *k = 1.0f - (float)(g_now - g_late.off_ms) / ANIM_LEAVE_MS;
        return 1;
    }
    g_late.off_ms = 0;
    return 0;
}

/* Same words once every run of digits is one '#': "Video 1.2 s late" and "Video 1.3 s late",
 * "00:04:02" and "00:04:03", "R2, 28 fps" and "R2, 29 fps". */
static int digits_only_change(const char *a, const char *b)
{
    while (*a && *b) {
        int da = *a >= '0' && *a <= '9', db = *b >= '0' && *b <= '9';
        if (da != db)
            return 0;
        if (da) {
            while (*a >= '0' && *a <= '9')
                a++;
            while (*b >= '0' && *b <= '9')
                b++;
            continue;
        }
        if (*a++ != *b++)
            return 0;
    }
    return !*a && !*b;
}

void anim_alert_cell(canvas *c, int i, int x, int y, int px, uint32_t col, const char *text, int cx,
                     int cy, int cw, int ch)
{
    if (!live() || i < 0 || i >= 5 || !text) {
        draw_ui_text_top(c, x, y, px, DRAW_FONT_MONO, col, 255, text ? text : "");
        return;
    }
    cell_slot *s = &g_cells[i];
    if (!s->have) {
        s->have = 1;
        snprintf(s->cur, sizeof s->cur, "%s", text);
        s->cur_col = col;
    } else if (strcmp(s->cur, text) != 0 || s->cur_col != col) {
        if (s->cur_col != col || !digits_only_change(s->cur, text)) {
            memcpy(s->old, s->cur, sizeof s->old);
            s->old_col = s->cur_col;
            s->t = g_now;
        }
        snprintf(s->cur, sizeof s->cur, "%s", text);
        s->cur_col = col;
    }
    float p = s->old[0] ? (float)(g_now - s->t) / ANIM_CELL_MS : 1.0f;
    if (p >= 1.0f) {
        s->old[0] = '\0';
        draw_ui_text_top(c, x, y, px, DRAW_FONT_MONO, col, 255, text);
        return;
    }
    /* The outgoing words stay inside this cell (a shorter new value narrows it). */
    int x0 = cx < 0 ? 0 : cx, y0 = cy < 0 ? 0 : cy;
    int x1 = cx + cw > c->w ? c->w : cx + cw, y1 = cy + ch > c->h ? c->h : cy + ch;
    if (x1 - x0 > 2 && y1 - y0 > 2) {
        canvas v = { c->px + (size_t)y0 * (size_t)c->stride + (size_t)x0, x1 - x0, y1 - y0, c->stride };
        float q = 1.0f - p; /* the old words go quicker than the new ones arrive */
        draw_ui_text_top(&v, x - x0, y - y0, px, DRAW_FONT_MONO, s->old_col,
                         (int)lroundf(255.0f * (1.0f - ANIM_CELL_START) * q * q), s->old);
    }
    draw_ui_text_top(c, x, y, px, DRAW_FONT_MONO, col,
                     (int)lroundf(255.0f * (ANIM_CELL_START + (1.0f - ANIM_CELL_START) * p)), text);
}
