#ifndef SEEKER_PLAN_H
#define SEEKER_PLAN_H

/*
 * Plan theme (hud_display --theme plan): a building drawing laid over the camera. A solid line is
 * someone the wearer's camera sees; a dashed hidden line is someone behind a wall. Labels sit on
 * plates with one 45 degree cut corner (top right), joined to the person by a leader line. Red
 * only when someone is close. Sentence case everywhere.
 *
 * Sizes are given in px for a 720 px tall frame and scaled by u = frame height / 720.
 */

#include "draw.h"
#include "seeker_config.h"
#include "seeker_msgs.h"
#include "wearer_view.h"

#define P_INK RGB(244, 242, 236)
#define P_INK2_A 158 /* secondary text: ink at 62% */
#define P_PLATE RGB(12, 13, 14)
#define P_PLATE_A 214 /* plate fill at 84% */
#define P_HIDDEN RGB(255, 227, 163)
#define P_RED RGB(255, 90, 78)
#define P_GROUND RGB(17, 18, 20) /* behind the full-screen plan */

float plan_u(const canvas *c);

/* One line of a plate: text, font (DRAW_FONT_*), size in px at 720, colour, alpha. */
typedef struct {
    const char *text;
    int font;
    int size;
    uint32_t col;
    int alpha;
} plan_line;

/* Plate size for these lines (padding 12 x 9, 3 px between lines, 10 px for the cut). */
void plan_plate_size(float u, const plan_line *l, int n, int *w, int *h);
/* Plate with its top-left at (x, y): fill, optional red hatch (close), cut-corner edge, text. */
void plan_plate(canvas *c, float u, int x, int y, const plan_line *l, int n, uint32_t edge,
                int red_hatch);
/* Leader from (x0, y0) with a 6 px square at the start, a 45 degree elbow, then level to
 * (x1, y1). */
void plan_leader(canvas *c, float u, float x0, float y0, float x1, float y1, uint32_t col);

/* Title block, bottom left: Camera / Eyes / Position / Time, plus Delay when `delay` is given
 * (the SEEKER_LAT readout). `t` gives the other phones and their rates. Returns its width
 * (draws nothing when c is NULL). Height: plan_title_h(u). */
int plan_title_block(canvas *c, float u, const char *camera, int camera_late,
                     const seeker_tracks *t, const char *wearer_node, const char *position,
                     const char *delay);
int plan_title_h(float u);

/* A person or sighting on the plan, in the room frame. */
enum { PLAN_SEEN = 0, PLAN_HIDDEN, PLAN_RAY };
typedef struct {
    int kind;
    int close;          /* draw red: a seen person close by, or anyone down */
    float x, y;         /* room position, or the ray's origin (the phone) */
    float ray_deg;      /* PLAN_RAY: room heading of the ray */
    char label[24];     /* full-screen plan only: e.g. "3.4 m" (distance from the wearer) */
    int id;             /* fusion's track id (0: untracked), for the map animations (anim_map.h) */
    int down;           /* posture down and fusion live: the slow repeating ring */
} plan_mark;

/* Plan of the room around the wearer in the rectangle (x, y, w, h): heading up, wearer at
 * (w / 2, h * wy) inside it, `ppm` px per metre. `header`: "Plan" title, 0 1 2 m scale bar and
 * the cut-corner plate (the corner inset); without it the rectangle is drawn on the screen as is
 * (full-screen fallback). Walls come from `wc`, phones from `t`. */
void plan_map(canvas *c, float u, int x, int y, int w, int h, float ppm, float wy, int header,
              const wv_pose *wearer, const char *wearer_node, float hfov_deg, const wv_config *wc,
              const seeker_tracks *t, const plan_mark *marks, int n);

/* A room point as the plan draws it around the wearer (heading yaw_deg up): metres to the right
 * (screen right; negative is left) and ahead (screen up). */
void plan_rel(const wv_pose *wearer, float yaw_deg, float x, float y, float *right_m, float *ahead_m);

/* Centred plate at y (top) for a warning; returns its height. */
int plan_banner(canvas *c, float u, int y, const char *title, const char *value, uint32_t edge,
                uint32_t title_col);

/* Sentence-case reason for the video banner (VIDEO_WHY_*), lower case after the first word. */
const char *plan_video_why(int why);

#endif
