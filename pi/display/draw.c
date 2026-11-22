#include "draw.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

/* Classic 5x7 column-major glyphs for ASCII 0x20 to 0x5F (bit 0 is the top row). */
static const uint8_t FONT[64][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},
    {0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},
    {0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},
    {0x00,0x41,0x22,0x1C,0x00},{0x08,0x2A,0x1C,0x2A,0x08},{0x08,0x08,0x3E,0x08,0x08},
    {0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},
    {0x20,0x10,0x08,0x04,0x02},{0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},
    {0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},
    {0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},
    {0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},
    {0x00,0x56,0x36,0x00,0x00},{0x00,0x08,0x14,0x22,0x41},{0x14,0x14,0x14,0x14,0x14},
    {0x41,0x22,0x14,0x08,0x00},{0x02,0x01,0x51,0x09,0x06},{0x32,0x49,0x79,0x41,0x3E},
    {0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},
    {0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x01,0x01},
    {0x3E,0x41,0x41,0x51,0x32},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},
    {0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},
    {0x7F,0x02,0x04,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},
    {0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},
    {0x1F,0x20,0x40,0x20,0x1F},{0x7F,0x20,0x18,0x20,0x7F},{0x63,0x14,0x08,0x14,0x63},
    {0x03,0x04,0x78,0x04,0x03},{0x61,0x51,0x49,0x45,0x43},{0x00,0x00,0x7F,0x41,0x41},
    {0x02,0x04,0x08,0x10,0x20},{0x41,0x41,0x7F,0x00,0x00},{0x04,0x02,0x01,0x02,0x04},
    {0x40,0x40,0x40,0x40,0x40},
};

static inline void put(canvas *c, int x, int y, uint32_t col)
{
    if ((unsigned)x < (unsigned)c->w && (unsigned)y < (unsigned)c->h)
        c->px[y * c->stride + x] = col;
}

void draw_fill(canvas *c, int x, int y, int w, int h, uint32_t col)
{
    int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int x1 = x + w > c->w ? c->w : x + w, y1 = y + h > c->h ? c->h : y + h;
    for (int j = y0; j < y1; j++)
        for (int i = x0; i < x1; i++)
            c->px[j * c->stride + i] = col;
}

void draw_dim(canvas *c, int x, int y, int w, int h)
{
    int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int x1 = x + w > c->w ? c->w : x + w, y1 = y + h > c->h ? c->h : y + h;
    for (int j = y0; j < y1; j++) {
        uint32_t *p = c->px + (size_t)j * (size_t)c->stride;
        for (int i = x0; i < x1; i++)
            p[i] = (p[i] >> 1) & 0x007f7f7fu;
    }
}

void draw_clear(canvas *c, uint32_t col)
{
    draw_fill(c, 0, 0, c->w, c->h, col);
}

static void dot(canvas *c, int x, int y, int thick, uint32_t col)
{
    if (thick <= 1) {
        put(c, x, y, col);
        return;
    }
    int o = thick / 2;
    draw_fill(c, x - o, y - o, thick, thick, col);
}

static void line_impl(canvas *c, int x0, int y0, int x1, int y1, int thick, int dash,
                      uint32_t col)
{
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, step = 0;
    for (int guard = 0; guard < 20000; guard++) {
        if (!dash || (step / dash) % 2 == 0)
            dot(c, x0, y0, thick, col);
        step++;
        if (x0 == x1 && y0 == y1)
            break;
        int e2 = 2 * err;
        if (e2 >= dy) {
            err += dy;
            x0 += sx;
        }
        if (e2 <= dx) {
            err += dx;
            y0 += sy;
        }
    }
}

void draw_line(canvas *c, int x0, int y0, int x1, int y1, int thick, uint32_t col)
{
    line_impl(c, x0, y0, x1, y1, thick, 0, col);
}

void draw_dashed(canvas *c, int x0, int y0, int x1, int y1, int thick, int dash, uint32_t col)
{
    line_impl(c, x0, y0, x1, y1, thick, dash > 0 ? dash : 6, col);
}

void draw_rect(canvas *c, int x, int y, int w, int h, int thick, uint32_t col)
{
    draw_fill(c, x, y, w, thick, col);
    draw_fill(c, x, y + h - thick, w, thick, col);
    draw_fill(c, x, y, thick, h, col);
    draw_fill(c, x + w - thick, y, thick, h, col);
}

void draw_circle(canvas *c, int cx, int cy, int r, int thick, uint32_t col)
{
    int x = r, y = 0, err = 1 - r;
    while (x >= y) {
        int pts[8][2] = { {x, y}, {y, x}, {-y, x}, {-x, y}, {-x, -y}, {-y, -x}, {y, -x}, {x, -y} };
        for (int i = 0; i < 8; i++)
            dot(c, cx + pts[i][0], cy + pts[i][1], thick, col);
        y++;
        if (err < 0) {
            err += 2 * y + 1;
        } else {
            x--;
            err += 2 * (y - x) + 1;
        }
    }
}

void draw_diamond(canvas *c, int cx, int cy, int r, int thick, uint32_t col)
{
    draw_line(c, cx, cy - r, cx + r, cy, thick, col);
    draw_line(c, cx + r, cy, cx, cy + r, thick, col);
    draw_line(c, cx, cy + r, cx - r, cy, thick, col);
    draw_line(c, cx - r, cy, cx, cy - r, thick, col);
}

void draw_brackets(canvas *c, int x, int y, int w, int h, int thick, uint32_t col)
{
    int lx = w / 4, ly = h / 4;
    draw_fill(c, x, y, lx, thick, col);
    draw_fill(c, x, y, thick, ly, col);
    draw_fill(c, x + w - lx, y, lx, thick, col);
    draw_fill(c, x + w - thick, y, thick, ly, col);
    draw_fill(c, x, y + h - thick, lx, thick, col);
    draw_fill(c, x, y + h - ly, thick, ly, col);
    draw_fill(c, x + w - lx, y + h - thick, lx, thick, col);
    draw_fill(c, x + w - thick, y + h - ly, thick, ly, col);
}

int draw_text_width(int scale, const char *s)
{
    return (int)strlen(s) * 6 * scale;
}

int draw_text(canvas *c, int x, int y, int scale, uint32_t col, const char *s)
{
    for (; *s; s++) {
        int ch = (unsigned char)*s;
        if (ch >= 'a' && ch <= 'z')
            ch -= 32;
        if (ch < 0x20 || ch > 0x5F)
            ch = '?';
        const uint8_t *g = FONT[ch - 0x20];
        for (int col_i = 0; col_i < 5; col_i++)
            for (int row = 0; row < 7; row++)
                if (g[col_i] & (1u << row))
                    draw_fill(c, x + col_i * scale, y + row * scale, scale, scale, col);
        x += 6 * scale;
    }
    return x;
}

uint32_t draw_colour_named(const char *name)
{
    if (!strcmp(name, "red"))
        return C_RED;
    if (!strcmp(name, "amber"))
        return C_AMBER;
    if (!strcmp(name, "green"))
        return C_GREEN;
    if (!strcmp(name, "cyan"))
        return C_CYAN;
    return C_GREY;
}

/* ---- theme ---- */

static int g_theme = DRAW_THEME_CLASSIC;

void draw_set_theme(int theme)
{
    g_theme = theme == DRAW_THEME_PLAN ? DRAW_THEME_PLAN : DRAW_THEME_CLASSIC;
}

int draw_theme(void)
{
    return g_theme;
}

int draw_theme_parse(const char *s)
{
    if (s && !strcmp(s, "plan"))
        return DRAW_THEME_PLAN;
    if (s && !strcmp(s, "classic"))
        return DRAW_THEME_CLASSIC;
    return -1;
}

/* ---- alpha blending and anti-aliased lines ---- */

static float g_dash_phase; /* draw_set_dash_phase */

void draw_set_dash_phase(float px)
{
    g_dash_phase = px;
}

float draw_dash_start(float dash, float gap)
{
    float period = dash + (gap > 0 ? gap : 0);
    if (g_dash_phase == 0.0f || period <= 0)
        return 0.0f;
    return fmodf(g_dash_phase, period) - period;
}

uint32_t draw_mix(uint32_t d, uint32_t s, int a)
{
    if (a <= 0)
        return d;
    if (a >= 255)
        return s & 0x00ffffffu;
    unsigned ia = 255u - (unsigned)a, ua = (unsigned)a;
    unsigned r = (((d >> 16) & 255u) * ia + ((s >> 16) & 255u) * ua + 127u) / 255u;
    unsigned g = (((d >> 8) & 255u) * ia + ((s >> 8) & 255u) * ua + 127u) / 255u;
    unsigned b = ((d & 255u) * ia + (s & 255u) * ua + 127u) / 255u;
    return (r << 16) | (g << 8) | b;
}

static inline float clampf01(float v)
{
    return v < 0.0f ? 0.0f : v > 1.0f ? 1.0f : v;
}

void draw_fill_a(canvas *c, int x, int y, int w, int h, uint32_t col, int alpha)
{
    int x0 = x < 0 ? 0 : x, y0 = y < 0 ? 0 : y;
    int x1 = x + w > c->w ? c->w : x + w, y1 = y + h > c->h ? c->h : y + h;
    for (int j = y0; j < y1; j++)
        for (int i = x0; i < x1; i++)
            c->px[j * c->stride + i] = draw_mix(c->px[j * c->stride + i], col, alpha);
}

void draw_line_aa(canvas *c, float x0, float y0, float x1, float y1, float width, int cap,
                  uint32_t col, int alpha)
{
    if (alpha <= 0 || width <= 0)
        return;
    float hw = width * 0.5f, dx = x1 - x0, dy = y1 - y0, len = sqrtf(dx * dx + dy * dy);
    float ux = len > 1e-4f ? dx / len : 1.0f, uy = len > 1e-4f ? dy / len : 0.0f;
    float ext = cap == DRAW_CAP_BUTT ? 0.0f : hw; /* how far the shape reaches past each end */
    int bx0 = (int)floorf(fminf(x0, x1) - hw - 1), bx1 = (int)ceilf(fmaxf(x0, x1) + hw + 1);
    int by0 = (int)floorf(fminf(y0, y1) - hw - 1), by1 = (int)ceilf(fmaxf(y0, y1) + hw + 1);
    if (bx0 < 0) bx0 = 0;
    if (by0 < 0) by0 = 0;
    if (bx1 > c->w) bx1 = c->w;
    if (by1 > c->h) by1 = c->h;
    for (int j = by0; j < by1; j++) {
        float py = (float)j + 0.5f - y0;
        uint32_t *row = c->px + (size_t)j * (size_t)c->stride;
        for (int i = bx0; i < bx1; i++) {
            float px = (float)i + 0.5f - x0;
            float along = px * ux + py * uy, across = fabsf(px * uy - py * ux), cov;
            if (cap == DRAW_CAP_ROUND) {
                float t = along < 0 ? 0 : along > len ? len : along;
                float ex = px - t * ux, ey = py - t * uy;
                cov = clampf01(hw + 0.5f - sqrtf(ex * ex + ey * ey));
            } else {
                float end = fmaxf(-ext - along, along - len - ext); /* < 0 inside the ends */
                cov = fminf(clampf01(hw + 0.5f - across), clampf01(0.5f - end));
            }
            if (cov > 0)
                row[i] = draw_mix(row[i], col, (int)(alpha * cov + 0.5f));
        }
    }
}

void draw_dashed_rect(canvas *c, float x, float y, float w, float h, float width, float dash,
                      float gap, uint32_t col, int alpha)
{
    draw_dashed_aa(c, x, y, x + w, y, width, dash, gap, DRAW_CAP_BUTT, col, alpha);
    draw_dashed_aa(c, x + w, y, x + w, y + h, width, dash, gap, DRAW_CAP_BUTT, col, alpha);
    draw_dashed_aa(c, x + w, y + h, x, y + h, width, dash, gap, DRAW_CAP_BUTT, col, alpha);
    draw_dashed_aa(c, x, y + h, x, y, width, dash, gap, DRAW_CAP_BUTT, col, alpha);
}

void draw_circle_aa(canvas *c, float cx, float cy, float r, float thick, float dash, float gap,
                    uint32_t col, int alpha)
{
    if (alpha <= 0 || r <= 0)
        return;
    float reach = r + thick * 0.5f + 1.0f;
    int x0 = (int)floorf(cx - reach), x1 = (int)ceilf(cx + reach);
    int y0 = (int)floorf(cy - reach), y1 = (int)ceilf(cy + reach);
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 > c->w) x1 = c->w;
    if (y1 > c->h) y1 = c->h;
    float period = dash + gap;
    for (int j = y0; j < y1; j++)
        for (int i = x0; i < x1; i++) {
            float ex = (float)i + 0.5f - cx, ey = (float)j + 0.5f - cy;
            float d = sqrtf(ex * ex + ey * ey);
            float cov = clampf01(thick * 0.5f + 0.5f - fabsf(d - r));
            if (cov <= 0)
                continue;
            if (dash > 0 && period > 0) {
                float s = (atan2f(ey, ex) + 3.14159265f) * r; /* arc length from the left */
                float m = fmodf(s - g_dash_phase, period);
                if ((m < 0 ? m + period : m) >= dash)
                    continue;
            }
            c->px[j * c->stride + i] = draw_mix(c->px[j * c->stride + i], col, (int)(alpha * cov + 0.5f));
        }
}

void draw_poly(canvas *c, const float *xy, int n, const draw_poly_style *st)
{
    if (n < 3 || n > 16)
        return;
    float nx[16], ny[16], off[16], area = 0;
    for (int i = 0; i < n; i++) {
        int k = (i + 1) % n;
        area += xy[2 * i] * xy[2 * k + 1] - xy[2 * k] * xy[2 * i + 1];
    }
    float sgn = area > 0 ? 1.0f : -1.0f; /* outward normals for either winding */
    float bx0 = 1e9f, by0 = 1e9f, bx1 = -1e9f, by1 = -1e9f;
    for (int i = 0; i < n; i++) {
        int k = (i + 1) % n;
        float ex = xy[2 * k] - xy[2 * i], ey = xy[2 * k + 1] - xy[2 * i + 1];
        float L = sqrtf(ex * ex + ey * ey);
        if (L < 1e-6f)
            L = 1e-6f;
        nx[i] = sgn * ey / L;
        ny[i] = -sgn * ex / L;
        off[i] = nx[i] * xy[2 * i] + ny[i] * xy[2 * i + 1];
        bx0 = fminf(bx0, xy[2 * i]);
        by0 = fminf(by0, xy[2 * i + 1]);
        bx1 = fmaxf(bx1, xy[2 * i]);
        by1 = fmaxf(by1, xy[2 * i + 1]);
    }
    float hw = st->edge_a > 0 ? st->edge_w * 0.5f : 0.0f;
    int ix0 = (int)floorf(bx0 - hw - 1), ix1 = (int)ceilf(bx1 + hw + 1);
    int iy0 = (int)floorf(by0 - hw - 1), iy1 = (int)ceilf(by1 + hw + 1);
    if (ix0 < 0) ix0 = 0;
    if (iy0 < 0) iy0 = 0;
    if (ix1 > c->w) ix1 = c->w;
    if (iy1 > c->h) iy1 = c->h;
    float gap = st->hatch_gap > 1 ? st->hatch_gap : 9, hwid = st->hatch_w > 0 ? st->hatch_w : 2;
    /* Each row: scan in from both ends to the inside run (a convex shape's inside is one run per
     * row). Inside it, deeper than the edge's reach, coverage is 1 and the edge adds nothing, so
     * an outline skips the run and a fill or hatch takes the short path (same pixels). */
    int outline_only = st->fill_a <= 0 && st->hatch_a <= 0;
    float inner = -(hw + 1.5f);
    for (int j = iy0; j < iy1; j++) {
        float py = (float)j + 0.5f;
        uint32_t *row = c->px + (size_t)j * (size_t)c->stride;
        int stop_l = ix1, from_r = ix1;
        {
            for (int i = ix0; i < ix1; i++) { /* find the first inside pixel from the left */
                float px = (float)i + 0.5f, d = -1e9f;
                for (int e = 0; e < n; e++) {
                    float de = nx[e] * px + ny[e] * py - off[e];
                    if (de > d)
                        d = de;
                }
                if (d < inner) {
                    stop_l = i;
                    break;
                }
            }
            from_r = stop_l;
            for (int i = ix1 - 1; i > stop_l; i--) { /* and from the right */
                float px = (float)i + 0.5f, d = -1e9f;
                for (int e = 0; e < n; e++) {
                    float de = nx[e] * px + ny[e] * py - off[e];
                    if (de > d)
                        d = de;
                }
                if (d < inner) {
                    from_r = i + 1;
                    break;
                }
            }
        }
        for (int i = ix0; i < ix1; i++) {
            if (i >= stop_l && i < from_r) {
                if (outline_only) {
                    i = from_r - 1;
                    continue;
                }
                uint32_t v = row[i];
                if (st->fill_a > 0)
                    v = draw_mix(v, st->fill, (int)(st->fill_a * 1.0f + 0.5f));
                if (st->hatch_a > 0) {
                    float s = fmodf((float)i + 0.5f + py - bx0 - by1, gap);
                    if (s < 0)
                        s += gap;
                    float across = fminf(s, gap - s) * 0.70710678f;
                    float hc = clampf01(hwid * 0.5f + 0.5f - across);
                    if (hc > 0)
                        v = draw_mix(v, st->hatch, (int)(st->hatch_a * hc * 1.0f + 0.5f));
                }
                row[i] = v;
                continue;
            }
            float px = (float)i + 0.5f, d = -1e9f;
            for (int e = 0; e < n; e++) {
                float de = nx[e] * px + ny[e] * py - off[e];
                if (de > d)
                    d = de;
            }
            float in = clampf01(0.5f - d);
            uint32_t v = row[i];
            if (in > 0 && st->fill_a > 0)
                v = draw_mix(v, st->fill, (int)(st->fill_a * in + 0.5f));
            if (in > 0 && st->hatch_a > 0) {
                /* Lines of constant x + y, `gap` apart along x; distance across is /sqrt2. */
                float s = fmodf(px + py - bx0 - by1, gap);
                if (s < 0)
                    s += gap;
                float across = fminf(s, gap - s) * 0.70710678f;
                float hc = clampf01(hwid * 0.5f + 0.5f - across);
                if (hc > 0)
                    v = draw_mix(v, st->hatch, (int)(st->hatch_a * hc * in + 0.5f));
            }
            if (st->edge_a > 0) {
                float ec = clampf01(hw + 0.5f - fabsf(d));
                if (ec > 0)
                    v = draw_mix(v, st->edge, (int)(st->edge_a * ec + 0.5f));
            }
            row[i] = v;
        }
    }
}

void draw_cut_rect(canvas *c, float x, float y, float w, float h, float cut,
                   const draw_poly_style *st)
{
    if (cut <= 0) {
        float r[8] = { x, y, x + w, y, x + w, y + h, x, y + h };
        draw_poly(c, r, 4, st);
        return;
    }
    float p[10] = { x, y, x + w - cut, y, x + w, y + cut, x + w, y + h, x, y + h };
    draw_poly(c, p, 5, st);
}

void draw_dashed_aa(canvas *c, float x0, float y0, float x1, float y1, float width, float dash,
                    float gap, int cap, uint32_t col, int alpha)
{
    float dx = x1 - x0, dy = y1 - y0, len = sqrtf(dx * dx + dy * dy);
    if (len < 1e-4f || dash <= 0) {
        draw_line_aa(c, x0, y0, x1, y1, width, cap, col, alpha);
        return;
    }
    if (gap < 0)
        gap = 0;
    float ux = dx / len, uy = dy / len;
    for (float a = draw_dash_start(dash, gap); a < len && a < 20000.0f; a += dash + gap) {
        float b = a + dash < len ? a + dash : len, s = a > 0 ? a : 0;
        if (b > s)
            draw_line_aa(c, x0 + ux * s, y0 + uy * s, x0 + ux * b, y0 + uy * b, width, cap, col, alpha);
    }
}
