#include "seeker_colour.h"

#include <string.h>

/* Least-squares slope (m/s) of the samples in the window, or 0 if they span too short a time. */
static float range_slope(const seeker_colour *s, int64_t now_ms)
{
    double st = 0, sr = 0, stt = 0, str = 0;
    int n = 0;
    int64_t t0 = 0, oldest = now_ms;
    for (int i = 0; i < s->n; i++) {
        int k = (s->head - 1 - i + SEEKER_COLOUR_N) % SEEKER_COLOUR_N;
        if (now_ms - s->t[k] > SEEKER_COLOUR_RATE_WIN_MS)
            break;
        if (n == 0)
            t0 = s->t[k];
        double t = (double)(s->t[k] - t0) / 1000.0, r = s->r[k];
        st += t;
        sr += r;
        stt += t * t;
        str += t * r;
        oldest = s->t[k];
        n++;
    }
    if (n < 3 || now_ms - oldest < SEEKER_COLOUR_RATE_MIN_MS)
        return 0;
    double den = n * stt - st * st;
    return den > 1e-9 ? (float)((n * str - st * sr) / den) : 0.0f;
}

int seeker_colour_update(seeker_colour *s, float range_m, int64_t now_ms, const seeker_config *cfg)
{
    if (s->valid && (now_ms < s->t[(s->head - 1 + SEEKER_COLOUR_N) % SEEKER_COLOUR_N] ||
                     now_ms - s->t[(s->head - 1 + SEEKER_COLOUR_N) % SEEKER_COLOUR_N] >
                         SEEKER_COLOUR_RESET_MS))
        s->valid = 0;
    if (!s->valid) {
        memset(s, 0, sizeof *s);
        s->valid = 1;
        s->level = -1;
    }
    s->t[s->head] = now_ms;
    s->r[s->head] = range_m;
    s->head = (s->head + 1) % SEEKER_COLOUR_N;
    if (s->n < SEEKER_COLOUR_N)
        s->n++;

    s->closing_mps = -range_slope(s, now_ms);
    float enter = SEEKER_COLOUR_FAST_ENTER * cfg->closing_mps;
    float leave = SEEKER_COLOUR_FAST_EXIT * cfg->closing_mps;
    if (s->fast) {
        if (s->closing_mps < leave)
            s->fast = 0;
    } else if (s->closing_mps > enter) {
        if (!s->fast_since_ms)
            s->fast_since_ms = now_ms;
        if (now_ms - s->fast_since_ms >= SEEKER_COLOUR_FAST_MS)
            s->fast = 1;
    } else {
        s->fast_since_ms = 0;
    }
    if (s->fast)
        s->fast_since_ms = 0;

    float near = cfg->near_m + (s->level == SEEKER_RED ? SEEKER_COLOUR_RANGE_HYST_M : 0);
    float mid = cfg->mid_m + (s->level >= SEEKER_AMBER ? SEEKER_COLOUR_RANGE_HYST_M : 0);
    int want = range_m < near || s->fast ? SEEKER_RED : range_m < mid ? SEEKER_AMBER : SEEKER_GREEN;
    if (s->level < 0 || (want != s->level && now_ms - s->level_ms >= SEEKER_COLOUR_HOLD_MS)) {
        s->level = want;
        s->level_ms = now_ms;
    }
    return s->level;
}

const char *seeker_colour_name(int level)
{
    return level == SEEKER_RED ? "red" : level == SEEKER_AMBER ? "amber" : "green";
}
