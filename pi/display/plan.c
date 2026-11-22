#include "plan.h"
#include "anim_alert.h"
#include "anim_map.h"
#include "video_rx.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#define DEG2RAD 0.017453292519943295f

float plan_u(const canvas *c)
{
    return c && c->h > 0 ? (float)c->h / 720.0f : 1.0f;
}

static int S(float u, float v) /* design px to frame px */
{
    return (int)lroundf(v * u);
}

/* ---- person strokes with a halo ---- */

static int halo_a(int alpha)
{
    return (P_HALO_A * (alpha > 255 ? 255 : alpha) + 127) / 255;
}

void plan_halo_line(canvas *c, float u, float x0, float y0, float x1, float y1, float width, int cap,
                    int alpha)
{
    if (alpha <= 0)
        return;
    float e = P_HALO_W * u * 0.5f;
    if (cap == DRAW_CAP_BUTT) { /* reach past each end as far as it does to the sides */
        float dx = x1 - x0, dy = y1 - y0, L = sqrtf(dx * dx + dy * dy);
        if (L > 1e-4f) {
            dx = dx / L * e, dy = dy / L * e;
            x0 -= dx, y0 -= dy, x1 += dx, y1 += dy;
        }
    }
    draw_line_aa(c, x0, y0, x1, y1, width + 2 * e, cap, P_HALO, halo_a(alpha));
}

void plan_halo_dash(canvas *c, float u, float x0, float y0, float x1, float y1, float width,
                    float dash, float gap, int alpha)
{
    float dx = x1 - x0, dy = y1 - y0, len = sqrtf(dx * dx + dy * dy);
    if (len < 1e-4f || dash <= 0) {
        plan_halo_line(c, u, x0, y0, x1, y1, width, DRAW_CAP_BUTT, alpha);
        return;
    }
    if (gap < 0)
        gap = 0;
    float ux = dx / len, uy = dy / len;
    /* Exactly draw_dashed_aa's dashes, phase included (draw_dash_start), so halos march with them. */
    for (float a = draw_dash_start(dash, gap); a < len && a < 20000.0f; a += dash + gap) {
        float b = a + dash < len ? a + dash : len, s = a > 0 ? a : 0;
        if (b > s)
            plan_halo_line(c, u, x0 + ux * s, y0 + uy * s, x0 + ux * b, y0 + uy * b, width, DRAW_CAP_BUTT, alpha);
    }
}

void plan_stroke(canvas *c, float u, float x0, float y0, float x1, float y1, float width, int cap,
               uint32_t col, int alpha)
{
    plan_halo_line(c, u, x0, y0, x1, y1, width, cap, alpha);
    draw_line_aa(c, x0, y0, x1, y1, width, cap, col, alpha);
}

void plan_dash(canvas *c, float u, float x0, float y0, float x1, float y1, float width, float dash,
               float gap, uint32_t col, int alpha)
{
    plan_halo_dash(c, u, x0, y0, x1, y1, width, dash, gap, alpha);
    draw_dashed_aa(c, x0, y0, x1, y1, width, dash, gap, DRAW_CAP_BUTT, col, alpha);
}

void plan_dash_rect(canvas *c, float u, float x, float y, float w, float h, float width, float dash,
                    float gap, uint32_t col, int alpha)
{
    plan_halo_dash(c, u, x, y, x + w, y, width, dash, gap, alpha);
    plan_halo_dash(c, u, x + w, y, x + w, y + h, width, dash, gap, alpha);
    plan_halo_dash(c, u, x + w, y + h, x, y + h, width, dash, gap, alpha);
    plan_halo_dash(c, u, x, y + h, x, y, width, dash, gap, alpha);
    draw_dashed_rect(c, x, y, w, h, width, dash, gap, col, alpha);
}

void plan_square(canvas *c, float u, float cx, float cy, float s, uint32_t col, int alpha)
{
    float hs = s + P_HALO_W * u;
    draw_fill_a(c, (int)lroundf(cx - hs / 2), (int)lroundf(cy - hs / 2), (int)lroundf(hs), (int)lroundf(hs),
                P_HALO, halo_a(alpha));
    draw_fill_a(c, (int)lroundf(cx - s / 2), (int)lroundf(cy - s / 2), (int)lroundf(s), (int)lroundf(s), col,
                alpha);
}

void plan_halo_rect(canvas *c, float u, float x, float y, float w, float h, float cut, float ew,
                    int alpha)
{
    if (alpha <= 0)
        return;
    draw_poly_style hs = { 0 };
    hs.edge = P_HALO;
    hs.edge_a = halo_a(alpha);
    hs.edge_w = ew + P_HALO_W * u;
    draw_cut_rect(c, x, y, w, h, cut, &hs);
}

/* ---- plate and leader ---- */

void plan_plate_size(float u, const plan_line *l, int n, int *w, int *h)
{
    int mw = 0, hh = S(u, 9) * 2 - S(u, 3);
    for (int i = 0; i < n; i++) {
        int px = S(u, (float)l[i].size), tw = draw_ui_width(px, l[i].font, l[i].text);
        if (tw > mw)
            mw = tw;
        hh += (int)lroundf((float)px * 1.15f) + S(u, 3);
    }
    *w = mw + S(u, 12) * 2 + S(u, 10);
    *h = hh;
}

void plan_plate(canvas *c, float u, int x, int y, const plan_line *l, int n, uint32_t edge,
                int red_hatch)
{
    plan_plate_a(c, u, x, y, l, n, edge, red_hatch, 255);
}

void plan_plate_a(canvas *c, float u, int x, int y, const plan_line *l, int n, uint32_t edge,
                  int red_hatch, int alpha)
{
    int w, h;
    plan_plate_size(u, l, n, &w, &h);
    draw_poly_style st = { 0 };
    st.fill = P_PLATE;
    st.fill_a = P_PLATE_A * alpha / 255;
    if (red_hatch) {
        st.hatch = P_ALERT;
        st.hatch_a = 71 * alpha / 255; /* 28% */
        st.hatch_gap = 9 * u;
        st.hatch_w = 2 * u;
    }
    st.edge = edge;
    st.edge_a = alpha;
    st.edge_w = 1.5f * u;
    plan_halo_rect(c, u, (float)x, (float)y, (float)w, (float)h, 10 * u, st.edge_w, alpha);
    draw_cut_rect(c, (float)x, (float)y, (float)w, (float)h, 10 * u, &st);
    int ty = y + S(u, 9);
    for (int i = 0; i < n; i++) {
        int px = S(u, (float)l[i].size);
        draw_ui_text_top(c, x + S(u, 12), ty, px, l[i].font, l[i].col, l[i].alpha * alpha / 255, l[i].text);
        ty += (int)lroundf((float)px * 1.15f) + S(u, 3);
    }
}

void plan_leader(canvas *c, float u, float x0, float y0, float x1, float y1, uint32_t col)
{
    float lw = 1.5f * u, dy = fabsf(y1 - y0), dx = fabsf(x1 - x0), sx = x1 >= x0 ? 1.0f : -1.0f;
    if (dy <= dx) { /* 45 degree run, then level */
        float kx = x0 + sx * dy;
        plan_halo_line(c, u, x0, y0, kx, y1, lw, DRAW_CAP_SQUARE, 255);
        plan_halo_line(c, u, kx, y1, x1, y1, lw, DRAW_CAP_SQUARE, 255);
        draw_line_aa(c, x0, y0, kx, y1, lw, DRAW_CAP_SQUARE, col, 255);
        draw_line_aa(c, kx, y1, x1, y1, lw, DRAW_CAP_SQUARE, col, 255);
    } else { /* too steep for one 45 degree run: straight down or up first */
        float ky = y1 + (y0 < y1 ? -dx : dx);
        plan_halo_line(c, u, x0, y0, x0, ky, lw, DRAW_CAP_SQUARE, 255);
        plan_halo_line(c, u, x0, ky, x1, y1, lw, DRAW_CAP_SQUARE, 255);
        draw_line_aa(c, x0, y0, x0, ky, lw, DRAW_CAP_SQUARE, col, 255);
        draw_line_aa(c, x0, ky, x1, y1, lw, DRAW_CAP_SQUARE, col, 255);
    }
    int s = S(u, 6), hs = s + S(u, P_HALO_W);
    draw_fill_a(c, (int)lroundf(x0) - hs / 2, (int)lroundf(y0) - hs / 2, hs, hs, P_HALO, P_HALO_A);
    draw_fill_a(c, (int)lroundf(x0) - s / 2, (int)lroundf(y0) - s / 2, s, s, col, 255);
}

/* ---- title block ---- */

int plan_title_h(float u)
{
    return S(u, 54);
}

int plan_title_block(canvas *c, float u, const char *camera, int camera_late,
                     const seeker_tracks *t, const char *wearer_node, const char *position,
                     const char *delay)
{
    char eyes[64] = "", clock[16];
    int others = 0;
    for (int i = 0; t && i < t->n_nodes; i++)
        if (!wearer_node || strcmp(t->nodes[i].node_id, wearer_node) != 0)
            others++;
    size_t o = 0;
    for (int i = 0; t && i < t->n_nodes && o + 20 < sizeof eyes; i++) {
        const seeker_node_status *n = &t->nodes[i];
        if (wearer_node && !strcmp(n->node_id, wearer_node))
            continue;
        const char *sep = o ? ", " : "";
        if (!strcmp(n->state, "ok"))
            o += (size_t)snprintf(eyes + o, sizeof eyes - o, others == 1 ? "%s%s, %.0f fps" : "%s%s %.0f fps",
                                  sep, n->node_id, n->rate_hz);
        else
            o += (size_t)snprintf(eyes + o, sizeof eyes - o, "%s%s %s", sep, n->node_id,
                                  !strcmp(n->state, "lost") ? "lost" : "waiting");
    }
    if (!o)
        snprintf(eyes, sizeof eyes, "None");
    time_t now = time(NULL);
    struct tm tmv;
    if (localtime_r(&now, &tmv))
        strftime(clock, sizeof clock, "%H:%M:%S", &tmv);
    else
        snprintf(clock, sizeof clock, "00:00:00");

    const char *keys[5] = { "Camera", "Eyes", "Position", "Time", "Delay" };
    const char *vals[5] = { camera, eyes, position, clock, delay };
    int nc = delay && delay[0] ? 5 : 4;
    int ws[5], total = 0, pad = S(u, 12), vpx = S(u, 15), kpx = S(u, 13);
    for (int i = 0; i < nc; i++) {
        int tw = draw_ui_width(vpx, DRAW_FONT_MONO, vals[i]);
        int kw = draw_ui_width(kpx, DRAW_FONT_SANS_MEDIUM, keys[i]);
        if (tw < kw)
            tw = kw;
        if (tw < S(u, 60))
            tw = S(u, 60);
        ws[i] = tw + pad * 2;
        total += ws[i];
    }
    if (!c)
        return total;
    int h = plan_title_h(u), x0 = S(u, 24), y = c->h - S(u, 24) - h;
    draw_poly_style st = { 0 };
    st.fill = P_PLATE;
    st.fill_a = P_PLATE_A;
    draw_cut_rect(c, (float)x0, (float)y, (float)total, (float)h, 12 * u, &st);
    float hsw, hk;
    if (anim_alert_late(camera_late, &hsw, &hk)) {
        draw_poly_style hs = { 0 };
        hs.hatch = P_ALERT;
        hs.hatch_a = 77; /* 30% */
        hs.hatch_gap = 9 * u;
        hs.hatch_w = 2 * u;
        anim_alert_hatch_rect(c, (float)x0, (float)y, (float)ws[0], (float)h, 0, &hs, hsw, hk);
    }
    draw_poly_style es = { 0 };
    es.edge = P_INK;
    es.edge_a = 140; /* 55% */
    es.edge_w = 1.5f * u;
    draw_cut_rect(c, (float)x0, (float)y, (float)total, (float)h, 12 * u, &es);
    int x = x0;
    for (int i = 0; i < nc; i++) {
        if (i)
            draw_line_aa(c, (float)x, (float)y, (float)x, (float)(y + h), 1.0f * u, DRAW_CAP_BUTT, P_INK, 56);
        draw_ui_text_top(c, x + pad, y + S(u, 8), kpx, DRAW_FONT_SANS_MEDIUM, P_INK, P_INK2_A, keys[i]);
        anim_alert_cell(c, i, x + pad, y + S(u, 27), vpx, i == 0 && camera_late ? P_ALERT : P_INK, vals[i], x, y,
                        ws[i], h);
        x += ws[i];
    }
    return total;
}

/* ---- plan ---- */

typedef struct {
    float ax, ay, k, yaw;
    const wv_pose *p;
} plan_geo;

void plan_rel(const wv_pose *wearer, float yaw_deg, float x, float y, float *right_m, float *ahead_m)
{
    float a = yaw_deg * DEG2RAD, dx = x - wearer->x_m, dy = y - wearer->y_m;
    *ahead_m = dx * sinf(a) + dy * cosf(a);
    *right_m = dx * cosf(a) - dy * sinf(a);
}

static void to_map(const plan_geo *g, float x, float y, float *sx, float *sy)
{
    float fwd, right;
    plan_rel(g->p, g->yaw, x, y, &right, &fwd);
    *sx = g->ax + right * g->k;
    *sy = g->ay - fwd * g->k;
}

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

void plan_map(canvas *c, float u, int x, int y, int w, int h, float ppm, float wy, int header,
              const wv_pose *wearer, const char *wearer_node, float hfov_deg, const wv_config *wc,
              const seeker_tracks *t, const plan_mark *marks, int n)
{
    anim_map_begin(header, marks, n);
    if (header) {
        draw_poly_style st = { 0 };
        st.fill = P_PLATE;
        st.fill_a = P_PLATE_A;
        st.edge = P_INK;
        st.edge_a = 140;
        st.edge_w = 1.5f * u;
        draw_cut_rect(c, (float)x, (float)y, (float)w, (float)h, 12 * u, &st);
        draw_ui_text_top(c, x + S(u, 12), y + S(u, 9), S(u, 14), DRAW_FONT_SANS_SEMIBOLD, P_INK, 255, "Plan");
    }
    /* Scale bar: 0 1 2 m, one block per metre. */
    {
        int seg = (int)lroundf(ppm), bh = S(u, 5);
        int sx = x + w - 2 * seg - S(u, 52), sy = y + S(u, 15);
        if (anim_map_scale_bar(c, u, sx, sy, seg, bh))
            goto scale_done;
        draw_fill_a(c, sx, sy, seg, bh, P_INK, 255);
        draw_poly_style bs = { 0 };
        bs.edge = P_INK;
        bs.edge_a = 255;
        bs.edge_w = 1.0f * u;
        draw_cut_rect(c, (float)(sx + seg), (float)sy, (float)seg, (float)bh, 0, &bs);
        int lp = S(u, 11), ly = sy + S(u, 8);
        draw_ui_text_top(c, sx - S(u, 3), ly, lp, DRAW_FONT_MONO_REGULAR, P_INK, P_INK2_A, "0");
        draw_ui_text_top(c, sx + seg - S(u, 3), ly, lp, DRAW_FONT_MONO_REGULAR, P_INK, P_INK2_A, "1");
        draw_ui_text_top(c, sx + 2 * seg - S(u, 3), ly, lp, DRAW_FONT_MONO_REGULAR, P_INK, P_INK2_A, "2 m");
    scale_done:;
    }
    /* Everything else is clipped to the drawing area: a canvas view onto it. */
    int top = header ? S(u, 40) : S(u, 40);
    int cx0 = x + 1, cy0 = y + top, cw = w - 2, ch = h - top - 1;
    if (cx0 < 0) { cw += cx0; cx0 = 0; }
    if (cy0 < 0) { ch += cy0; cy0 = 0; }
    if (cx0 + cw > c->w) cw = c->w - cx0;
    if (cy0 + ch > c->h) ch = c->h - cy0;
    if (cw <= 4 || ch <= 4)
        return;
    canvas v = { c->px + (size_t)cy0 * (size_t)c->stride + (size_t)cx0, cw, ch, c->stride };
    /* Rings (anim_map) may spill past the drawing area onto the whole plate. */
    int rx0 = x + 1 > 0 ? x + 1 : 0, ry0 = y + 1 > 0 ? y + 1 : 0;
    int rx1 = x + w - 1 < c->w ? x + w - 1 : c->w, ry1 = y + h - 1 < c->h ? y + h - 1 : c->h;
    canvas rv = { c->px + (size_t)ry0 * (size_t)c->stride + (size_t)rx0, rx1 - rx0, ry1 - ry0, c->stride };
    plan_geo g = { (float)w * 0.5f - 1.0f, (float)(y + (int)lroundf((float)h * wy)) - (float)cy0, ppm,
                   wearer->yaw_deg, wearer };
    float far = (float)(cw + ch) * 2.0f;

    /* Wearer view cone. */
    float half = hfov_deg * 0.5f * DEG2RAD, fk = anim_map_grow(ANIM_MAP_CONE, far, g.ay / cosf(half));
    for (int s = -1; s <= 1 && fk > 0; s += 2)
        draw_dashed_aa(&v, g.ax, g.ay, g.ax + s * fk * sinf(half), g.ay - fk * cosf(half), 1.0f * u, 6 * u,
                       5 * u, DRAW_CAP_BUTT, P_INK, 115);
    anim_map_scan(&v, u, g.ax, g.ay, half, far);
    /* Walls in poche: light fill, dense hatch, solid outline. */
    for (int i = 0; wc && i < wc->n_walls; i++) {
        float ax, ay, bx, by;
        to_map(&g, wc->walls[i][0], wc->walls[i][1], &ax, &ay);
        to_map(&g, wc->walls[i][2], wc->walls[i][3], &bx, &by);
        float wk = anim_map_draw_in(ANIM_MAP_WALL, i, wc->n_walls);
        if (wk < 1.0f) { /* drawing in: the wall grows from its first end */
            bx = ax + (bx - ax) * wk;
            by = ay + (by - ay) * wk;
        }
        float dx = bx - ax, dy = by - ay, L = sqrtf(dx * dx + dy * dy);
        if (L < 1)
            continue;
        float hw = 4.5f * u, nx = -dy / L * hw, ny = dx / L * hw;
        float p[8] = { ax + nx, ay + ny, bx + nx, by + ny, bx - nx, by - ny, ax - nx, ay - ny };
        draw_poly_style st = { 0 };
        st.fill = P_INK;
        st.fill_a = 46;
        st.hatch = P_INK;
        st.hatch_a = 179;
        st.hatch_gap = 5 * u;
        st.hatch_w = 1.2f * u;
        st.edge = P_INK;
        st.edge_a = 255;
        st.edge_w = 1.5f * u;
        draw_poly(&v, p, 4, &st);
    }
    /* Other phones: a circle with a dashed view cone and the id. */
    for (int i = 0; t && i < t->n_nodes; i++) {
        const seeker_node_status *nd = &t->nodes[i];
        if (wearer_node && !strcmp(nd->node_id, wearer_node))
            continue;
        float nx, ny;
        to_map(&g, nd->x_m, nd->y_m, &nx, &ny);
        nx = clampf(nx, 8 * u, (float)cw - 8 * u);
        ny = clampf(ny, 8 * u, (float)ch - 8 * u);
        int a = !strcmp(nd->state, "ok") ? 255 : 120;
        float cone = 3.5f * ppm * anim_map_draw_in(ANIM_MAP_PHONE, 0, 1), hf = (nd->hfov_deg > 1 ? nd->hfov_deg : 66) * 0.5f;
        for (int s = -1; s <= 1; s += 2) {
            float h2 = (nd->yaw_deg - wearer->yaw_deg + s * hf) * DEG2RAD;
            draw_dashed_aa(&v, nx, ny, nx + cone * sinf(h2), ny - cone * cosf(h2), 1.0f * u, 3 * u, 4 * u,
                           DRAW_CAP_BUTT, P_HIDDEN, a / 2);
        }
        draw_circle_aa(&v, nx, ny, 5 * u, 1.5f * u, 0, 0, P_HIDDEN, a);
        draw_ui_text_top(&v, (int)lroundf(nx - 8 * u), (int)lroundf(ny + 9 * u), S(u, 11), DRAW_FONT_MONO_REGULAR,
                         P_HIDDEN, a, nd->node_id);
    }
    /* People. */
    for (int i = 0; i < n; i++) {
        const plan_mark *m = &marks[i];
        float px, py;
        to_map(&g, m->x, m->y, &px, &py);
        if (m->kind == PLAN_RAY) {
            float a = (m->ray_deg - wearer->yaw_deg) * DEG2RAD;
            plan_dash(&v, u, px, py, px + far * sinf(a), py - far * cosf(a), 1.5f * u, 6 * u, 5 * u, P_HIDDEN, 200);
            continue;
        }
        px = clampf(px, 7 * u, (float)cw - 7 * u);
        py = clampf(py, 7 * u, (float)ch - 7 * u);
        float s = 10 * u;
        anim_map_rings(&rv, u, m, px + (float)(cx0 - rx0), py + (float)(cy0 - ry0));
        if (m->kind == PLAN_SEEN)
            plan_square(&v, u, px, py, s, m->close ? P_ALERT : P_SEEN, 255);
        else {
            /* Behind a wall: the ring (above), then a halo that marches with the dashes, then the
             * square; the static dashed square (with its halo) when the map is not animating. */
            uint32_t hc = m->close ? P_ALERT : P_HIDDEN;
            if (anim_map_hidden_rect(&v, px - s / 2, py - s / 2, s, 2 * u + P_HALO_W * u, 3 * u, 2 * u, P_HALO,
                                     halo_a(255)))
                anim_map_hidden_rect(&v, px - s / 2, py - s / 2, s, 2 * u, 3 * u, 2 * u, hc, 255);
            else
                plan_dash_rect(&v, u, px - s / 2, py - s / 2, s, s, 2 * u, 3 * u, 2 * u, hc, 255);
        }
        if (m->label[0])
            draw_ui_text_top(&v, (int)lroundf(px + 10 * u), (int)lroundf(py - 9 * u), S(u, 13), DRAW_FONT_MONO,
                             m->close ? P_ALERT : m->kind == PLAN_SEEN ? P_SEEN : P_HIDDEN, 255, m->label);
    }
    /* The wearer: a filled triangle pointing ahead, with its id. */
    float tri[6] = { g.ax, g.ay - 9 * u, g.ax + 7 * u, g.ay + 6 * u, g.ax - 7 * u, g.ay + 6 * u };
    draw_poly_style ts = { 0 };
    ts.fill = P_INK;
    ts.fill_a = 255;
    draw_poly(&v, tri, 3, &ts);
    draw_ui_text_top(&v, (int)lroundf(g.ax + 10 * u), (int)lroundf(g.ay - 2 * u), S(u, 11), DRAW_FONT_MONO_REGULAR, P_INK,
                     P_INK2_A, wearer_node && wearer_node[0] ? wearer_node : "You");
}

/* ---- banners ---- */

int plan_banner(canvas *c, float u, int y, const char *title, const char *value, uint32_t edge,
                uint32_t title_col)
{
    plan_line l[2] = { { title, DRAW_FONT_SANS_SEMIBOLD, 19, title_col, 255 },
                       { value, DRAW_FONT_MONO, 15, P_INK, P_INK2_A } };
    int n = value && value[0] ? 2 : 1, w, h;
    plan_plate_size(u, l, n, &w, &h);
    anim_alert_warn(c, u, (c->w - w) / 2, y, l, n, edge);
    return h;
}

const char *plan_video_why(int why)
{
    switch (why) {
    case VIDEO_WHY_OFF: return "video is off";
    case VIDEO_WHY_NO_LINK: return "phone bridge not connected";
    case VIDEO_WHY_NO_FRAMES: return "phone not sending video";
    case VIDEO_WHY_BAD_DATA: return "unexpected video frames";
    case VIDEO_WHY_DECODE: return "picture did not decode";
    case VIDEO_WHY_STALLED: return "video stopped";
    case VIDEO_WHY_OTHER_NODE: return "only another phone's camera";
    default: return "no video";
    }
}
