#include "seeker_skel.h"

#include <math.h>
#include <string.h>

#define DEG2RAD 0.017453292519943295f
#define TWO_PI 6.283185307179586f

enum { L_SH = 5, R_SH = 6, L_HIP = 11, R_HIP = 12 };

int seeker_skel_local(const seeker_contact *c, int w, int h, float hfov_deg, float range_m,
                      float out[SEEKER_N_JOINTS][3])
{
    if (!c->has_pts || w <= 0 || h <= 0 || !(range_m > 0.1f))
        return 0;
    float hfov = hfov_deg > 1.0f && hfov_deg < 179.0f ? hfov_deg : 66.0f;
    float f = ((float)w * 0.5f) / tanf(hfov * 0.5f * DEG2RAD);
    float hpx = f * SEEKER_SKEL_PERSON_M / range_m; /* px height of the person at this range */
    float ax, ay;
    const float (*p)[3] = c->pts;
    if (p[L_HIP][2] >= SEEKER_SKEL_VIS_MIN && p[R_HIP][2] >= SEEKER_SKEL_VIS_MIN) {
        ax = (p[L_HIP][0] + p[R_HIP][0]) * 0.5f * (float)w;
        ay = (p[L_HIP][1] + p[R_HIP][1]) * 0.5f * (float)h;
    } else if (p[L_SH][2] >= SEEKER_SKEL_VIS_MIN && p[R_SH][2] >= SEEKER_SKEL_VIS_MIN) {
        /* Hips hidden (behind a desk, cut by the frame): shoulder line minus the torso, so the
         * anchor does not jump to the box centre and drag every joint with it. */
        ax = (p[L_SH][0] + p[R_SH][0]) * 0.5f * (float)w;
        ay = (p[L_SH][1] + p[R_SH][1]) * 0.5f * (float)h + 0.29f * hpx;
    } else if (c->has_box) {
        ax = c->box[0] + c->box[2] * 0.5f;
        ay = c->box[1] + c->box[3] * 0.5f;
    } else {
        return 0;
    }
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        out[j][0] = (p[j][0] * (float)w - ax) / hpx;
        out[j][1] = (p[j][1] * (float)h - ay) / hpx;
        out[j][2] = p[j][2];
    }
    return 1;
}

static float oe_alpha(float cutoff_hz, float dt_s)
{
    float tau = 1.0f / (TWO_PI * cutoff_hz);
    return 1.0f / (1.0f + tau / dt_s);
}

/* One Euro step for one coordinate: x and dx are the filter state. */
static void one_euro(float *x, float *dx, float z, float dt_s)
{
    float raw_d = (z - *x) / dt_s;
    *dx += oe_alpha(SEEKER_SKEL_D_CUTOFF_HZ, dt_s) * (raw_d - *dx);
    float cutoff = SEEKER_SKEL_MIN_CUTOFF_HZ + SEEKER_SKEL_BETA * fabsf(*dx);
    *x += oe_alpha(cutoff, dt_s) * (z - *x);
}

static void restart_joint(seeker_skel *s, int j, const float *z, int64_t rx_ms)
{
    s->x[j] = z[0];
    s->y[j] = z[1];
    s->dx[j] = s->dy[j] = 0;
    s->vis[j] = z[2];
    s->joint_ms[j] = rx_ms;
    s->pend_ms[j] = 0;
}

/* SEEKER_SKEL_CONFIRM: 1 when a jump of joint j to z agrees with the one waiting, else it waits. */
static int jump_confirmed(seeker_skel *s, int j, const float *z, int64_t rx_ms)
{
    if (s->pend_ms[j] && rx_ms - s->pend_ms[j] <= SEEKER_SKEL_JOINT_HOLD_MS &&
        fabsf(z[0] - s->pend_x[j]) <= 0.5f * SEEKER_SKEL_SNAP && fabsf(z[1] - s->pend_y[j]) <= 0.5f * SEEKER_SKEL_SNAP)
        return 1;
    s->pend_x[j] = z[0];
    s->pend_y[j] = z[1];
    s->pend_ms[j] = rx_ms;
    return 0;
}

static void update(seeker_skel *s, const float (*raw)[3], int64_t rx_ms, int smooth)
{
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        const float *z = raw[j];
        if (z[2] < SEEKER_SKEL_VIS_MIN)
            continue; /* held (or hidden once the hold runs out) */
        int fresh = s->joint_ms[j] > 0 && rx_ms - s->joint_ms[j] <= SEEKER_SKEL_JOINT_HOLD_MS;
        int jump = fabsf(z[0] - s->x[j]) > SEEKER_SKEL_SNAP || fabsf(z[1] - s->y[j]) > SEEKER_SKEL_SNAP;
        if (smooth && fresh && jump && (smooth & SEEKER_SKEL_CONFIRM) && !jump_confirmed(s, j, z, rx_ms))
            continue; /* held where it was */
        if (!smooth || !fresh || jump) {
            restart_joint(s, j, z, rx_ms);
            continue;
        }
        s->pend_ms[j] = 0;
        float dt = (float)(rx_ms - s->joint_ms[j]) / 1000.0f;
        if (dt < 0.01f)
            dt = 0.01f;
        one_euro(&s->x[j], &s->dx[j], z[0], dt);
        one_euro(&s->y[j], &s->dy[j], z[1], dt);
        s->vis[j] += 0.5f * (z[2] - s->vis[j]);
        s->joint_ms[j] = rx_ms;
    }
}

int seeker_skel_step(seeker_skel *s, const char *node, long seq, int64_t rx_ms,
                     const float (*raw)[3], int smooth, float out[SEEKER_N_JOINTS][3])
{
    if (s->valid && strncmp(s->node, node, sizeof s->node) != 0)
        memset(s, 0, sizeof *s); /* another phone's view: its joints are not comparable */
    if (!s->valid || seq != s->seq) {
        if (raw) {
            if (!s->valid) {
                memset(s, 0, sizeof *s);
                s->valid = 1;
                strncpy(s->node, node, sizeof s->node - 1);
                s->t_ms = rx_ms;
            }
            update(s, raw, rx_ms, smooth);
            s->pts_ms = rx_ms;
        }
        if (s->valid) {
            s->seq = seq;
            s->t_ms = rx_ms;
        }
    }
    if (!s->valid || s->t_ms - s->pts_ms > SEEKER_SKEL_HOLD_MS)
        return 0;
    int shown = 0;
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        int live = s->joint_ms[j] > 0 && s->t_ms - s->joint_ms[j] <= SEEKER_SKEL_JOINT_HOLD_MS;
        out[j][0] = s->x[j];
        out[j][1] = s->y[j];
        out[j][2] = live ? fmaxf(s->vis[j], SEEKER_SKEL_VIS_MIN) : 0.0f;
        shown += live;
    }
    return shown > 0;
}
