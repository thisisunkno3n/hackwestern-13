#ifndef SEEKER_ANIM_PERSON_H
#define SEEKER_ANIM_PERSON_H

/*
 * Person lifecycle animations for the plan theme's wearer view (display/wearer_view.c).
 *
 * Decoration only: a person's frame, skeleton and hull are always drawn at this frame's place and
 * size (whatever fusion and WV_BOX_EASE give). What animates is how the arrival and the loss are
 * dressed, timed in ms from the one render timestamp the caller passes to anim_person_begin:
 *
 *   appear, seen     full frame faint at once, corner segments at full strength trace along the
 *                    edges and close it (AP_TRACE_MS), then the leader draws out from the person
 *                    (AP_LEADER_MS), then the plate fades in and slides AP_SLIDE_PX into place
 *                    (AP_PLATE_MS).
 *   appear, x-ray    skeleton and hull start at AP_XRAY_A0 and reach full in AP_XRAY_FADE_MS, the
 *                    hull hatch sweeps in from the bottom-left corner (AP_HATCH_MS), leader and
 *                    plate as above.
 *   lost             a track that is gone (not in this frame's tracks at all) leaves an echo of
 *                    its last frame or skeleton (no plate), alpha falling from AP_ECHO_A0 to 0
 *                    over AP_LOST_MS. One still tracked but not drawn in view (anim_person_keep)
 *                    leaves none.
 *   look change      not animated here: seen to x-ray or back is the handoff crossfade's.
 *   ambient          hidden-person dashes march at AP_MARCH_PX_S.
 *
 * Text and numbers are never animated. A track that comes back with the same id within
 * AP_FORGET_MS does not replay its leader and plate (only the mark's short intro, and not even
 * that while its echo is still showing). State is a fixed table keyed by fusion's track id.
 * anim_person_set_enabled(0) makes every hook a no-op, so the output equals plain plan drawing.
 */

#include <stdint.h>

#include "draw.h"
#include "plan.h"
#include "wearer_view.h"

#define AP_TRACE_MS 180
#define AP_LEADER_MS 120
#define AP_PLATE_MS 120
#define AP_XRAY_FADE_MS 150
#define AP_HATCH_MS 220
#define AP_LOST_MS 220
#define AP_FORGET_MS 2000
#define AP_XRAY_A0 0.6f   /* x-ray marks' alpha on their first frame */
#define AP_ECHO_A0 0.5f   /* an echo's alpha when the track has just gone */
#define AP_TRACE_C0 0.22f /* corner segments' length on the first frame, of half an edge */
#define AP_FAINT_A 0.35f  /* the full frame under the tracing corners */
#define AP_SLIDE_PX 6.0f  /* plate slide, design px at 720 */
#define AP_MARCH_PX_S 12.0f
#define AP_SLOTS 32

/* Person stroke halo (dark line under each stroke): the plan palette's P_HALO_A / P_HALO_W when
 * plan.h has them, so the palette stays in one place. */
#ifndef AP_HALO_A
#ifdef P_HALO_A
#define AP_HALO_A P_HALO_A
#else
#define AP_HALO_A 166 /* rgba(0,0,0,0.65) */
#endif
#endif
#ifndef AP_HALO_W
#ifdef P_HALO_W
#define AP_HALO_W P_HALO_W
#else
#define AP_HALO_W 3.0f /* px wider than the stroke, at 720 */
#endif
#endif

/* What the hooks do for one person this frame. active == 0: draw exactly the plain plan marks. */
typedef struct {
    int active;
    int echo;     /* drawing a lost track's echo (marks only) */
    float trace;  /* seen frame: 0..1 corner trace (1 closed) */
    float mark_a; /* alpha factor for the marks */
    float hatch;  /* x-ray hull hatch sweep 0..1 */
    float leader; /* leader draw progress 0..1 */
    float plate;  /* plate fade and slide 0..1 */
    float march;  /* dash phase in frame px for hidden-person dashes (0 none) */
} ap_fx;

void anim_person_set_enabled(int on); /* default on; toggling clears the table */
int anim_person_enabled(void);
void anim_person_reset(void);

/* Frame bracket: begin with the frame's timestamp; register each person drawn in view with
 * anim_person_track (returns its fx), anim_person_store after its marks; then anim_person_end
 * turns the ones not drawn this frame into echoes and frees expired ones. */
void anim_person_begin(int64_t now_ms, float u);
ap_fx anim_person_track(int id, int direct);
ap_fx anim_person_get(int id); /* fx of a track registered this frame (labels), else inactive */
void anim_person_store(int id, const wv_item *it, const seeker_track *k, const int box[4]);
void anim_person_end(void);
/* Slot i (0..AP_SLOTS-1) as an echo to draw this frame: returns 1 and fills the stored item,
 * track, box and the echo's fx; 0 if slot i is not a drawable echo. */
int anim_person_echo(int i, wv_item *it, seeker_track *k, int box[4], ap_fx *fx);

/* A track that still exists this frame but is not drawn in view (a duplicate, out of the view, no
 * bearing): no echo, and no new arrival when it comes back into view. */
void anim_person_keep(int id);
/* Hidden-person dash phase this frame (frame px, 0 when off), for marks drawn without an fx. */
float anim_person_march(void);
/* Handoff crossfade (wearer_view.h, WV_XFADE_MS): while it runs, it alone sets the opacity of the
 * old and new looks. A look change never restarts the intro; ap_crossfading also drops the mark
 * fade-in and corner trace of a track coming back (same id, after its echo) in the other look. */
ap_fx ap_crossfading(ap_fx fx);

/* Drawing hooks (each one is the plain plan call when fx is inactive or settled). */
int ap_alpha(const ap_fx *fx, int a);
/* Seen frame: rectangle outline 2u wide plus the mid-side ticks, traced and faded per fx. */
void ap_seen_frame(canvas *c, float u, int x, int y, int w, int h, uint32_t col, int a, const ap_fx *fx);
/* Hatch-only hull (style hs) on the rectangle, swept in from the bottom-left corner. */
void ap_hatch_rect(canvas *c, float x, float y, float w, float h, const draw_poly_style *hs, const ap_fx *fx);
void ap_leader(canvas *c, float u, float x0, float y0, float x1, float y1, uint32_t col, const ap_fx *fx);
/* Plate at its final place (x, y); while fading it is drawn (sdx, sdy) * AP_SLIDE_PX short of it. */
void ap_plate(canvas *c, float u, int x, int y, const plan_line *l, int n, uint32_t edge, int red_hatch,
              int sdx, int sdy, const ap_fx *fx);

#endif
