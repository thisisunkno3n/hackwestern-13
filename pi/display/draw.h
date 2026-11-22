#ifndef SEEKER_DRAW_H
#define SEEKER_DRAW_H

/* Software raster for the HUD: 32-bit 0x00RRGGBB pixels, no dependencies. */

#include <stdint.h>

typedef struct {
    uint32_t *px;
    int w, h;
    int stride; /* in pixels */
} canvas;

#define RGB(r, g, b) (((uint32_t)(r) << 16) | ((uint32_t)(g) << 8) | (uint32_t)(b))
#define C_BLACK RGB(0, 0, 0)
#define C_HUD RGB(60, 255, 120)    /* HUD green for symbology */
#define C_DIM RGB(20, 90, 45)
#define C_RED RGB(255, 50, 40)
#define C_AMBER RGB(255, 180, 0)
#define C_GREEN RGB(60, 230, 90)
#define C_CYAN RGB(0, 230, 255)
#define C_GREY RGB(120, 120, 120)
#define C_WHITE RGB(255, 255, 255)

void draw_clear(canvas *c, uint32_t col);
void draw_line(canvas *c, int x0, int y0, int x1, int y1, int thick, uint32_t col);
void draw_dashed(canvas *c, int x0, int y0, int x1, int y1, int thick, int dash, uint32_t col);
void draw_rect(canvas *c, int x, int y, int w, int h, int thick, uint32_t col);
void draw_fill(canvas *c, int x, int y, int w, int h, uint32_t col);
/* Halves the brightness of a rectangle (stale video under live symbology). */
void draw_dim(canvas *c, int x, int y, int w, int h);
void draw_circle(canvas *c, int cx, int cy, int r, int thick, uint32_t col);
void draw_diamond(canvas *c, int cx, int cy, int r, int thick, uint32_t col);
void draw_brackets(canvas *c, int x, int y, int w, int h, int thick, uint32_t col);
/* 5x7 glyphs scaled by `scale`; lowercase is drawn as uppercase. Returns the end x. */
int draw_text(canvas *c, int x, int y, int scale, uint32_t col, const char *s);
int draw_text_width(int scale, const char *s);
uint32_t draw_colour_named(const char *name);

/* ---- theme ----
 * classic: the original look (5x7 capitals, brackets, solid colours). The library default, so
 * tests and other callers see exactly the old pixels.
 * plan:    drafting look (hud_display's default, --theme plan). Scaffolding only so far: the
 *          primitives below exist, the views still draw the classic marks. */
enum { DRAW_THEME_CLASSIC = 0, DRAW_THEME_PLAN = 1 };
void draw_set_theme(int theme);
int draw_theme(void);
/* "plan" or "classic"; -1 for anything else. */
int draw_theme_parse(const char *s);

/* ---- alpha blending and anti-aliased lines. alpha 0..255 over 0x00RRGGBB pixels. ---- */
uint32_t draw_mix(uint32_t dst, uint32_t src, int alpha);
void draw_fill_a(canvas *c, int x, int y, int w, int h, uint32_t col, int alpha);

/* Line ends: round (half a disc), square (extends width/2 past the end), butt (stops at it). */
enum { DRAW_CAP_ROUND = 0, DRAW_CAP_SQUARE = 1, DRAW_CAP_BUTT = 2 };
/* Anti-aliased line from (x0,y0) to (x1,y1), `width` px across. */
void draw_line_aa(canvas *c, float x0, float y0, float x1, float y1, float width, int cap,
                  uint32_t col, int alpha);
/* Dashed: `dash` px on, `gap` px off, starting with a dash at (x0,y0); every dash gets the cap
 * (with round or square caps the visible gap is gap - width). */
void draw_dashed_aa(canvas *c, float x0, float y0, float x1, float y1, float width, float dash,
                    float gap, int cap, uint32_t col, int alpha);
/* Rectangle outline of dashed lines (butt caps), one per side. */
void draw_dashed_rect(canvas *c, float x, float y, float w, float h, float width, float dash,
                      float gap, uint32_t col, int alpha);
/* Circle outline, `thick` px, centred on radius r; dash > 0 makes it dashed along the arc. */
void draw_circle_aa(canvas *c, float cx, float cy, float r, float thick, float dash, float gap,
                    uint32_t col, int alpha);

/* Convex polygon (xy = x0,y0,x1,y1,..., n points, either winding) in one pass, sharp corners:
 * fill, then 45 degree hatch (lines rising to the right, `hatch_gap` px apart along x, 2 px
 * wide), then an outline of `edge_w` px centred on the boundary. An alpha of 0 skips a layer. */
typedef struct {
    uint32_t fill;
    int fill_a;
    uint32_t hatch;
    int hatch_a;
    float hatch_gap, hatch_w;
    uint32_t edge;
    int edge_a;
    float edge_w;
} draw_poly_style;
void draw_poly(canvas *c, const float *xy, int n, const draw_poly_style *st);
/* Rectangle with its top-right corner cut at 45 degrees by `cut` px (the plan theme's plate). */
void draw_cut_rect(canvas *c, float x, float y, float w, float h, float cut,
                   const draw_poly_style *st);

/* ---- TTF text (draw_font.c, plan theme): Atkinson Hyperlegible, third_party/atkinson ----
 * Baseline-positioned, `px` = font size in pixels (em, like CSS font-size). Accepts UTF-8; the
 * baked set is ASCII plus the degree, plus-minus, middle dot and multiplication signs. Glyphs
 * are baked the first time a (font, size) is used; draw_ui_init bakes the HUD's sizes up front. */
/* Sans in three weights for words; mono for numbers: medium (values), regular (small map labels). */
enum { DRAW_FONT_SANS = 0, DRAW_FONT_SANS_MEDIUM, DRAW_FONT_SANS_SEMIBOLD, DRAW_FONT_MONO,
       DRAW_FONT_MONO_REGULAR, DRAW_FONT_N };
void draw_ui_init(int canvas_h);
int draw_ui_text(canvas *c, int x, int baseline, int px, int font, uint32_t col, int alpha,
                 const char *s);
int draw_ui_width(int px, int font, const char *s);
int draw_ui_cap(int px, int font); /* cap height in px (from the font's H) */
/* Text with its em box top at y (like a canvas textBaseline "top"). Returns the end x. */
int draw_ui_text_top(canvas *c, int x, int y, int px, int font, uint32_t col, int alpha,
                     const char *s);

#endif
