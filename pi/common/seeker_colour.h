#ifndef SEEKER_COLOUR_H
#define SEEKER_COLOUR_H

/*
 * Threat colour of one person (IF-7 red / amber / green by range and approach speed), with
 * hysteresis so a noisy range does not make the HUD flicker. Used by hud_fusion (the tracks'
 * colour field: radar fallback, plain wearer background) and by the EagleEye wearer view (range
 * from the wearer), one state per track.
 *
 * - Approach speed: least-squares slope of the range over the last SEEKER_COLOUR_RATE_WIN_MS,
 *   not a tick-to-tick difference (ranges arrive in ~15 Hz steps, so those were 0 or a spike).
 * - Red when nearer than near_m (leaves above near_m + SEEKER_COLOUR_RANGE_HYST_M), or while
 *   approaching fast: above SEEKER_COLOUR_FAST_ENTER x closing_mps for SEEKER_COLOUR_FAST_MS,
 *   until under SEEKER_COLOUR_FAST_EXIT x closing_mps.
 * - Amber nearer than mid_m (leaves above mid_m + SEEKER_COLOUR_RANGE_HYST_M), else green.
 * - A colour stays at least SEEKER_COLOUR_HOLD_MS before it changes.
 */

#include "seeker_config.h"

#include <stdint.h>

#define SEEKER_COLOUR_RATE_WIN_MS 700  /* approach speed: slope over this window */
#define SEEKER_COLOUR_RATE_MIN_MS 300  /* ... once it spans at least this long */
#define SEEKER_COLOUR_RANGE_HYST_M 0.3f
#define SEEKER_COLOUR_FAST_ENTER 1.2f  /* x closing_mps (0.5): 0.6 m/s */
#define SEEKER_COLOUR_FAST_EXIT 0.6f   /* x closing_mps: 0.3 m/s */
#define SEEKER_COLOUR_FAST_MS 300      /* fast approach must last this long */
#define SEEKER_COLOUR_HOLD_MS 400      /* minimum time per colour */
#define SEEKER_COLOUR_RESET_MS 1000    /* no update for this long: start over */
#define SEEKER_COLOUR_N 32             /* range samples kept (30 Hz x 0.7 s fits) */

enum { SEEKER_GREEN, SEEKER_AMBER, SEEKER_RED };

typedef struct {
    int valid;
    int level;              /* SEEKER_GREEN / AMBER / RED */
    int64_t level_ms;       /* when the level last changed */
    int fast;               /* approaching fast (with hysteresis) */
    int64_t fast_since_ms;  /* above the enter speed since (0: not) */
    float closing_mps;      /* smoothed approach speed, positive = coming closer */
    int n, head;            /* ring of recent (t, range) samples */
    int64_t t[SEEKER_COLOUR_N];
    float r[SEEKER_COLOUR_N];
} seeker_colour;

/* One update with the person's current range (m) at now_ms. Returns the level to draw. */
int seeker_colour_update(seeker_colour *s, float range_m, int64_t now_ms, const seeker_config *cfg);
/* "red", "amber", "green" (the tracks colour field). */
const char *seeker_colour_name(int level);

#endif
