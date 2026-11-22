#ifndef SEEKER_ANIM_ALERT_H
#define SEEKER_ANIM_ALERT_H

/*
 * State-change and alert animations for the plan theme. Decoration only: an alert that becomes
 * true this frame is drawn in the alert colour this frame, and nobody's position or size is ever
 * eased here. What moves is alpha, colour blend, hatch progress, rings, pulses and plate offsets.
 *
 *   entering an alert   frame, plate edge and words snap to P_ALERT; the plate hatch sweeps in
 *                       diagonally from the bottom-left corner (starts at 20%, full by 200 ms)
 *   person down         plus one ring ping out from the frame (500 ms), then a calm breathing
 *                       pulse of the frame, period 1.2 s, alpha 70% to 100% (never below 70%).
 *                       Behind a wall: the same ring and pulse on the hull, as an alert outline
 *   leaving an alert    colours blend back from P_ALERT to normal and the hatch fades, 200 ms
 *   title block         a cell whose words change crossfades old (40% to 0) to new (60% to
 *                       100%) over 150 ms; a change of digits only (the clock, "1.2 s late",
 *                       fps) swaps at once so counters stay crisp. The "Video 1.2 s late"
 *                       hatch sweeps in like a plate hatch and fades out
 *   warning plates      slide in 8 px from above while fading 60% to 100% (150 ms), and out
 *                       the same way (100% to 0, up 8 px)
 *
 * Time comes from one timestamp per rendered frame, anim_alert_frame(now_ms), called by
 * hud_render with hud_state.now_ms. No clock is read here, so tools/hud_shot can render any
 * moment. now_ms 0 (never set, or a caller without a frame time) or anim_alert_set_enabled(0)
 * draws exactly the plain plan theme.
 */

#include "draw.h"
#include "plan.h"

#include <stdint.h>

#define ANIM_ALERT_SLOTS 32      /* people tracked at once (fixed table, keyed by fusion track id) */
#define ANIM_FORGET_MS 1000      /* a track id unseen this long is forgotten (a reused id starts fresh) */
#define ANIM_HATCH_MS 200        /* plate hatch sweep on entering an alert */
#define ANIM_HATCH_START 0.2f    /* sweep progress on the entering frame */
#define ANIM_LEAVE_MS 200        /* blend back to normal after an alert ends */
#define ANIM_RING_MS 500         /* person down: one ring ping */
#define ANIM_PULSE_MS 1200       /* then the breathing period */
#define ANIM_PULSE_MIN 0.70f     /* lowest frame alpha factor of the pulse */
#define ANIM_CELL_MS 150         /* title block crossfade */
#define ANIM_CELL_START 0.60f    /* new value's alpha factor on the changing frame */
#define ANIM_PLATE_MS 150        /* warning plate slide and fade */
#define ANIM_PLATE_DY 8.0f       /* px at 720 */
#define ANIM_PLATE_START 0.60f   /* warning plate alpha factor on its first frame */

void anim_alert_set_enabled(int on); /* default on */
int anim_alert_enabled(void);
void anim_alert_reset(void);         /* forget every track, cell and plate; frame time 0 */
/* Once per rendered frame, before drawing. 0 = no frame time: draw the plain plan theme. A time
 * earlier than the last one resets the state (hud_shot jumping back). */
void anim_alert_frame(int64_t now_ms);

/* One person's decoration this frame. on == 0: draw as the plain plan theme. */
typedef struct {
    int on;
    int alert;   /* the alert is true this frame */
    int down;    /* person down (and the alert is on) */
    float leave; /* 1 to 0 over ANIM_LEAVE_MS after the alert ended, else 0 */
    float hatch; /* plate hatch sweep progress, 0 to 1 */
    float ring;  /* ring ping progress 0 to 1, or -1 for none */
    float pulse; /* frame alpha factor, ANIM_PULSE_MIN to 1 */
} anim_fx;

/* State for fusion track `id` (<= 0: no state, plain look). Idempotent within a frame, so the
 * marks, the label and an edge chevron of the same person can each ask. */
anim_fx anim_alert_track(int id, int alert, int down);

/* Hooks for the plan theme's drawing; each is the plain plan drawing when f is off. */
uint32_t anim_alert_tint(const anim_fx *f, uint32_t col); /* blends col toward P_ALERT while leaving */
int anim_alert_alpha(const anim_fx *f, int alpha);        /* the down pulse */
void anim_alert_ring(canvas *c, float u, const anim_fx *f, float x, float y, float w, float h);
/* Behind a wall and down: ring plus a pulsing alert outline on the hull. */
void anim_alert_hull(canvas *c, float u, const anim_fx *f, float x, float y, float w, float h);
/* plan_plate with the hatch sweep (entering) or fade (leaving). */
void anim_alert_plate(canvas *c, float u, int x, int y, const plan_line *l, int n, uint32_t edge,
                      int red_hatch, const anim_fx *f);

/* Warning plates (the wearer view's top-left plate and plan_banner): keyed by the title text.
 * Draws the plate sliding and fading in; anim_alert_warn_flush draws the plates that were up last
 * frame but not this one, sliding and fading out. Call the flush once per frame, last. */
void anim_alert_warn(canvas *c, float u, int x, int y, const plan_line *l, int n, uint32_t edge);
void anim_alert_warn_flush(canvas *c, float u);

/* Title block. anim_alert_late: should the camera cell's hatch be drawn (late, or fading out),
 * with its sweep progress and alpha factor. anim_alert_hatch_rect: draw_cut_rect of a hatch-only
 * style, clipped to the sweep and scaled by k (exactly draw_cut_rect at sweep 1, k 1).
 * anim_alert_cell: value text of cell i (0 to 4) at (x, y) top, crossfading within the cell rect
 * (cx, cy, cw, ch); exactly draw_ui_text_top in the steady state. */
int anim_alert_late(int late, float *sweep, float *k);
void anim_alert_hatch_rect(canvas *c, float x, float y, float w, float h, float cut,
                           const draw_poly_style *hs, float sweep, float k);
void anim_alert_cell(canvas *c, int i, int x, int y, int px, uint32_t col, const char *text, int cx,
                     int cy, int cw, int ch);

#endif
