#ifndef SEEKER_ANIM_MAP_H
#define SEEKER_ANIM_MAP_H

/*
 * Plan map animations (plan theme only: the corner inset and the full-screen plan, plan.c).
 * Decoration only, never position: every marker is drawn at its fused place on the frame it
 * exists, at once, with no easing. What moves:
 *   - a person new on the map: one ring ping from their square, 450 ms, in their colour;
 *   - a person down: a slow ring every 1.4 s from the moment they went down;
 *   - the wearer's view cone: a faint scan line sweeping across it every 3 s, under the marks;
 *   - hidden people's dashed squares: the dashes march slowly round the square;
 *   - first show (boot, or the full-screen plan appearing): cones, walls and the scale bar draw
 *     in over 300 ms; people are there from the first frame.
 * Time comes only from anim_map_frame (one timestamp per rendered frame: hud_render passes
 * hud_state.now_ms, the same frame time as the alert and person animations);
 * nothing here reads a clock or counts frames, so any moment can be rendered again exactly.
 * Disabled, or with no frame time given yet, plan_map draws exactly what it drew before.
 */

#include "draw.h"
#include "plan.h"

#include <stdint.h>

/* On by default. Off: plan output is pixel-identical to the unanimated plan. */
void anim_map_set_enabled(int on);
int anim_map_enabled(void);

/* The frame's time in ms (any monotonic clock), once per rendered frame (hud_render does it).
 * A value <= 0 means no clock: that frame draws the settled, unanimated plan. */
void anim_map_frame(int64_t now_ms);

/* Forget every person, the view's first-show time and the frame time (tests, shot tools). */
void anim_map_reset(void);

/* ---- hooks called by plan_map (plan.c) ---- */

/* Start of a plan_map call: `header` 1 = corner inset, 0 = full-screen plan. Updates the
 * per-person table (keyed by plan_mark.id, ids <= 0 untracked) and the first-show time. */
void anim_map_begin(int header, const plan_mark *marks, int n);

/* Draw-in progress 0..1 of a piece of static geometry; exactly 1.0f once settled or when off. */
enum { ANIM_MAP_CONE = 0, ANIM_MAP_WALL, ANIM_MAP_PHONE };
float anim_map_draw_in(int part, int index, int count);
/* A line drawing in from its start: `full` once settled or when off (exactly), else the length
 * reached so far out of `shown`, the part of it that is on screen. */
float anim_map_grow(int part, float full, float shown);

/* Scale bar while it draws in: draws it and returns 1; returns 0 (draws nothing) once settled,
 * so plan_map draws its own. Arguments as plan_map's scale bar. */
int anim_map_scale_bar(canvas *c, float u, int sx, int sy, int seg, int bh);

/* Scan line across the wearer's cone from its apex (ax, ay), half angle `half` (radians),
 * `far` px long. Call after the cone and before walls, phones and people. */
void anim_map_scan(canvas *v, float u, float ax, float ay, float half, float far);

/* Ping and down rings for one person at screen (px, py) of the view; call just before the
 * person's square is drawn. */
void anim_map_rings(canvas *v, float u, const plan_mark *m, float px, float py);

/* A hidden person's dashed square with marching dashes: draws it and returns 1, or returns 0
 * (draws nothing) when off, so plan_map draws its own draw_dashed_rect. */
int anim_map_hidden_rect(canvas *v, float x, float y, float s, float width, float dash, float gap,
                         uint32_t col, int alpha);

#endif
