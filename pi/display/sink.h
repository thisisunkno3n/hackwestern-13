#ifndef SEEKER_SINK_H
#define SEEKER_SINK_H

/*
 * Display sinks for hud_display. `screen` (QNX Screen, fullscreen on HDMI0) is the Pi sink and the
 * only QNX-only file here (sink_screen.c, IF-1). `ppm` writes snapshots for laptops and tests,
 * `none` draws and discards (timing runs). IF-1 names `imshow` for laptops; it is left out of this
 * draft so nothing needs OpenCV.
 */

#include "draw.h"
#include "seeker_config.h"

typedef struct seeker_sink seeker_sink;
struct seeker_sink {
    int w, h;
    int no_vsync;     /* screen: swap interval 0 (posts do not wait for vsync) */
    float blit_ms;    /* last present: copying the canvas into the output buffer */
    float post_ms;    /* last present: waiting for a free buffer plus the post (vsync) */
    int buf_w, buf_h; /* screen: render at this size and let Screen scale it to the display
                       * (fewer pixels to draw and copy on a 4K monitor); 0: the display size */
    int mirror_h, mirror_v, swap_rb;
    int (*present)(seeker_sink *s, const canvas *c);
    void (*close)(seeker_sink *s);
    void *priv;
    char ppm_path[256];
    int ppm_every;      /* write every Nth frame, and any frame whose note state changed */
    char ppm_note[160]; /* set per frame by the caller; written as the PPM's "# " comment line */
    char ppm_state[160]; /* ppm_note from " video=" up to " seq=" at the last write */
    long frames;
};

/* Fills `s` for `kind`. Returns 0, or -1 if the sink is unknown or failed to open. */
int seeker_sink_open(seeker_sink *s, const char *kind, const seeker_config *cfg);

/* Copies a canvas into a 32-bit destination, applying mirror and R/B swap. */
void seeker_sink_blit(const seeker_sink *s, const canvas *c, uint32_t *dst, int dst_stride_px);

int seeker_sink_write_ppm(const char *path, const canvas *c);
/* Same, with `note` (NULL or "" for none) as a "# note" line right after "P6". Bytes outside
 * printable ASCII become '?', so the header stays one line whatever a relay puts in node_id.
 * Written to path.tmp and renamed, so a reader never sees a half-written file. */
int seeker_sink_write_ppm_note(const char *path, const canvas *c, const char *note);

/* Provided by sink_screen.c when built with Screen (real or host stub), else by sink_noscreen.c. */
int seeker_sink_screen_open(seeker_sink *s, const seeker_config *cfg);

#endif
