#ifndef SEEKER_HUD_RENDER_H
#define SEEKER_HUD_RENDER_H

/*
 * IF-7 symbology after the pivot. Two backgrounds (display.background):
 *   radar  (default) full-screen top-down room map: wearer at the taped origin facing up, phones
 *          as squares with view cones, contacts as diamonds; range-less contacts as dashed rays.
 *   wearer the IF-5.6 video from the relay's view node, letterboxed full screen, with symbology
 *          in that phone's frame (brackets from its boxes, horizon diamonds, edge chevrons for
 *          the other phone) and a radar inset bottom right. A frame 0.5 to 2 s old stays up,
 *          dimmed, tagged VIDEO STALE; with no frame for 2 s it falls back to the full radar
 *          under a banner that says why (video_why_text: no bridge, phone not sending, bad
 *          frames, JPEG decode failed, VIDEO LOST).
 * Both draw the heading tape (REL, since heading_deg is null without head pose), NODE Rn LOST
 * banners and the status line with process lights.
 */

#include "draw.h"
#include "seeker_config.h"
#include "seeker_health.h"
#include "seeker_msgs.h"
#include "video_rx.h"
#include "wearer_view.h"

#include <stddef.h>

typedef struct {
    const seeker_tracks *t; /* newest tracks, or NULL if none ever arrived */
    int tracks_age_ms;      /* receive age of t on this machine */
    seeker_health_table *health;
    float fps;
    int hud_prio;
    const char *background;
    const char *view_node;
    const video_frame *video; /* drawn only when VIDEO_SHOWN(video_state); dimmed when STALE */
    int video_state;          /* VIDEO_* from video_rx_state */
    float video_fps;
    /* EagleEye: when set, background "wearer" is the projected wearer view (wearer_view.h),
     * over the video when VIDEO_SHOWN(video_state), else on black (visor). */
    wv_view *wearer;
    int video_why; /* VIDEO_WHY_* from video_rx_why, for the banner text (0 with the video not
                    * shown means unknown: the plain NO VIDEO / VIDEO LOST text) */
    int video_frozen; /* video_rx_frozen(.., VIDEO_FREEZE_MS): the wearer view keeps the last
                       * picture (dark) through a VIDEO LOST instead of switching to radar */
    const char *lat_text; /* latency readout drawn top right (SEEKER_LAT=1), NULL for none */
    int64_t now_ms; /* this frame's time for the plan theme's animations (one per frame), 0 = none */
} hud_state;

/* Returns 1 if this frame has the video picture composited into it, else 0 (radar fallback,
 * EagleEye on black, or another node's video that the wearer view leaves out). */
int hud_render(canvas *c, const seeker_config *cfg, const hud_state *st);

/* One line describing a rendered frame, for the PPM sink's header comment, e.g.
 * "seeker hud frame=42 video=ok drawn=1 node=R1 seq=17". node and seq are the newest frame from
 * the receiver ("-" and 0 before the first). Scripts wait on this instead of sleeping. */
void hud_frame_note(char *out, size_t n, long frame, int video_state, int drawn,
                    const video_frame *f);

#endif
