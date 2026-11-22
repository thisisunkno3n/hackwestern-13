/* Crouch replay: a jointed 3D person (1.75 m, feet on the floor) seen by two phones through a
 * pinhole camera, the phone's own gates and ranging (mirrors remote/phone.html: confmin, the
 * upright gate, eye-to-ankle range, One Euro), the real hud_fusion core, the real tracks datagram
 * and the real wearer view, on a simulated clock.
 *
 *   test_crouch [seed] [--old-phone] [--table]     (ctest "crouch"; no sockets, no screen)
 *
 * --old-phone runs the phones with the upright gate as it was (every box under 1.5 tall per wide
 * dropped), to show what the gate did to a crouched person. --table prints the posture features
 * and labels of the still poses and checks nothing.
 *
 * Room: R1, the wearer, at 0,-2 facing +y; R2 at -2.2,0.6 facing the person from their side; the
 * person at 0.3,1.0 (3 m from R1, 2.5 m from R2). Cameras 1.3 m up (chest height), tilted 8 deg
 * down, 640x480, 66 deg. Phones report at 15 Hz, 20 to 40 ms late, landmarks jitter by 0.8% of
 * the body height; R1 misses 12% of its frames.
 *
 * The person: thigh and shin 0.43 m, hip to shoulder 0.50 m. A crouch of depth d folds the thigh
 * 110 d deg from vertical (knees forward and a little out), leans the shin 38 d deg and the
 * torso 40 d deg, hands to the knees. d 0.65 is an ordinary crouch (thighs 72 deg from vertical,
 * knee angle 84 deg, hips 0.59 m up); d 0.75 has the thighs about level (hips 0.47 m); d 1 is
 * sitting on the heels (hips 0.26 m up, the pose main needed before it said CROUCH).
 *
 * Checks (each failed on main before the crouch fixes, see pi/docs/LIVE-RUN.md):
 *   posture   a crouch from d 0.7 reads CROUCH from the front, three quarters and the side, at
 *             2.5 to 4 m, also side on with the far leg's ankle hidden; standing, a slight knee
 *             bend, walking, a stair step and bending over never do.
 *   dup       wv_classify: another phone's track far along its own ray (a long size range) is
 *             the seen person's duplicate; a second person on another ray, in front of the seen
 *             one or behind a wall is not.
 *   box       a seen person's box in a frame the wearer's phone missed is that phone's last box,
 *             not a projected standing-person box; a crouched seen person without one gets a
 *             box as wide and 0.6 as tall.
 *   replay    stand, crouch (0.5 s down to d 0.65, or to d 1), stay, stand up, facing R1 and
 *             facing R2: CROUCH about 0.5 s after being down, one track and no x-ray copy on
 *             every tick and frame, the SEEN box about as wide as the standing one and shorter
 *             on every frame; walking, stair steps and bending over stay one SEEN person, never
 *             CROUCH.
 */

#include "anim.h"
#include "draw.h"
#include "seeker_config.h"
#include "seeker_fusion.h"
#include "seeker_msgs.h"
#include "seeker_posture.h"
#include "seeker_skel.h"
#include "wearer_view.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f
#ifndef WV_CROUCH_BOX_H /* so this file also builds against a tree without the fix, to see it fail */
#define WV_CROUCH_BOX_H 0.6f
#endif
#define NJ SEEKER_N_JOINTS
#define NB (NJ + 6) /* plus fingertips, toes and heels: phone.html boxes all 33 landmarks */

static unsigned g_seed;
static double urand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return ((g_seed >> 8) & 0xffffff) / 16777216.0 + 1e-9;
}
static double gauss(void)
{
    return sqrt(-2.0 * log(urand())) * cos(6.283185307179586 * urand());
}

static int g_fails, g_checks;
#define WANT(c, ...)                                                                               \
    do {                                                                                           \
        g_checks++;                                                                                \
        if (!(c)) {                                                                                \
            printf("FAIL ");                                                                       \
            printf(__VA_ARGS__);                                                                   \
            printf("\n");                                                                          \
            g_fails++;                                                                             \
        }                                                                                          \
    } while (0)

/* ---- the person ---- */

enum { NOSE, LEYE, REYE, LEAR, REAR, LSH, RSH, LEL, REL, LWR, RWR, LHIP, RHIP, LKN, RKN, LAN, RAN };
#define L_THIGH 0.43f
#define L_SHIN 0.43f
#define L_TORSO 0.50f
#define ANKLE_H 0.07f

typedef struct {
    float thigh[2], knee[2]; /* per leg (left, right): thigh forward of straight down, knee flexion, deg */
    float splay;             /* knees out to the side, m */
    float lean;              /* torso forward of vertical, deg */
    float hands;             /* 0 arms hanging, 1 hands on the knees */
} pose_t;

/* Joints in the person's own frame: x to their right, y up from the floor, z the way they face.
 * The lower foot stands on the floor under the hips' start (z 0 at the standing ankle). */
static void body3(const pose_t *p, float J[NB][3])
{
    float kn[2][3], an[2][3], hy = 0, hz = 0;
    for (int s = 0; s < 2; s++) { /* legs from a hip at the origin */
        float a = p->thigh[s] * DEG2RAD, b = (p->thigh[s] - p->knee[s]) * DEG2RAD;
        float side = s ? 1.0f : -1.0f;
        kn[s][0] = side * (0.09f + p->splay), kn[s][1] = -L_THIGH * cosf(a), kn[s][2] = L_THIGH * sinf(a);
        an[s][0] = side * (0.10f + 0.3f * p->splay), an[s][1] = kn[s][1] - L_SHIN * cosf(b);
        an[s][2] = kn[s][2] + L_SHIN * sinf(b);
        if (!s || ANKLE_H - an[s][1] > hy)
            hy = ANKLE_H - an[s][1], hz = -an[s][2];
    }
    for (int s = 0; s < 2; s++) {
        float side = s ? 1.0f : -1.0f;
        float *h = J[LHIP + s], *k = J[LKN + s], *a = J[LAN + s];
        h[0] = side * 0.09f, h[1] = hy, h[2] = hz;
        k[0] = kn[s][0], k[1] = hy + kn[s][1], k[2] = hz + kn[s][2];
        a[0] = an[s][0], a[1] = hy + an[s][1], a[2] = hz + an[s][2];
    }
    float l = p->lean * DEG2RAD, sy = hy + L_TORSO * cosf(l), sz = hz + L_TORSO * sinf(l);
    float nl = 0.5f * l, ny = sy + 0.20f * cosf(nl), nz = sz + 0.20f * sinf(nl) + 0.08f;
    J[NOSE][0] = 0, J[NOSE][1] = ny, J[NOSE][2] = nz;
    for (int s = 0; s < 2; s++) {
        float side = s ? 1.0f : -1.0f;
        float *e = J[LEYE + s], *r = J[LEAR + s], *sh = J[LSH + s], *el = J[LEL + s], *wr = J[LWR + s];
        e[0] = side * 0.03f, e[1] = ny + 0.03f, e[2] = nz - 0.02f;
        r[0] = side * 0.07f, r[1] = ny + 0.01f, r[2] = nz - 0.10f;
        sh[0] = side * 0.19f, sh[1] = sy, sh[2] = sz;
        float hang[3] = { side * 0.24f, sy - 0.55f, sz + 0.05f };
        for (int a = 0; a < 3; a++)
            wr[a] = hang[a] + p->hands * (J[LKN + s][a] - hang[a]);
        el[0] = 0.5f * (sh[0] + wr[0]) + side * (0.03f + 0.07f * p->hands);
        el[1] = 0.5f * (sh[1] + wr[1]);
        el[2] = 0.5f * (sh[2] + wr[2]) - 0.05f * p->hands;
        float *fi = J[NJ + s], *toe = J[NJ + 2 + s], *heel = J[NJ + 4 + s];
        for (int a = 0; a < 3; a++)
            fi[a] = wr[a] + 0.35f * (wr[a] - el[a]);
        const float *an = J[LAN + s];
        toe[0] = an[0] + side * 0.03f, toe[1] = 0.01f + an[1] - ANKLE_H, toe[2] = an[2] + 0.17f;
        heel[0] = an[0], heel[1] = 0.01f + an[1] - ANKLE_H, heel[2] = an[2] - 0.06f;
    }
}

static void pose_crouch(float d, pose_t *p)
{
    memset(p, 0, sizeof *p);
    for (int s = 0; s < 2; s++) {
        p->thigh[s] = 110.0f * d;
        p->knee[s] = 148.0f * d; /* the shin leans 38 d deg the other way */
    }
    p->splay = 0.10f * d;
    p->lean = 40.0f * d;
    p->hands = d;
}

/* Walking on the spot, phase 0..1 per stride: thighs swing +-25 deg, the swing knee folds 60. */
static void pose_walk(float ph, pose_t *p)
{
    memset(p, 0, sizeof *p);
    for (int s = 0; s < 2; s++) {
        float a = 6.2831853f * (ph + 0.5f * (float)s);
        p->thigh[s] = 25.0f * sinf(a);
        float sw = sinf(a + 2.2f);
        p->knee[s] = 8.0f + 55.0f * (sw > 0 ? sw : 0);
    }
    p->lean = 4.0f;
}

/* A stair step: one foot a step up (thigh 65 deg forward, shin vertical), the other leg straight. */
static void pose_stair(int up, pose_t *p)
{
    memset(p, 0, sizeof *p);
    p->thigh[up] = 65.0f, p->knee[up] = 65.0f;
    p->thigh[!up] = -5.0f, p->knee[!up] = 5.0f;
    p->lean = 12.0f;
}

static void pose_bend(float deg, pose_t *p)
{
    memset(p, 0, sizeof *p);
    p->thigh[0] = p->thigh[1] = -6.0f; /* hips back over straight legs */
    p->lean = deg;
    p->hands = 0.2f;
}

/* ---- camera and phone (remote/phone.html) ---- */

typedef struct {
    float x, y, yaw; /* room pose */
    float h, pitch;  /* lens height m, tilt deg (up positive) */
} cam_t;
enum { IMG_W = 640, IMG_H = 480 };
#define HFOV 66.0f
static float focal(void)
{
    return 0.5f * IMG_W / tanf(0.5f * HFOV * DEG2RAD);
}

typedef struct {
    float far_vis;  /* knee and ankle visibility of the leg behind the other in a side view */
    float far_knee; /* its knee alone (-1: far_vis) */
} vis_t;
static const vis_t VIS_ALL = { 0.9f, -1.0f };

/* Normalized image landmarks of the person standing at (px, py) facing face_deg (room yaw). */
static void project(const float J[NB][3], float px, float py, float face_deg, const cam_t *c,
                    const vis_t *vs, float noise, float lm[NB][3])
{
    float fa = face_deg * DEG2RAD, ca = c->yaw * DEG2RAD, q = -c->pitch * DEG2RAD, f = focal();
    float depth[NB];
    for (int j = 0; j < NB; j++) {
        /* facing (sin, cos); the person's right is (cos, -sin) */
        float rx = px + J[j][2] * sinf(fa) + J[j][0] * cosf(fa) - c->x;
        float ry = py + J[j][2] * cosf(fa) - J[j][0] * sinf(fa) - c->y;
        float Z = rx * sinf(ca) + ry * cosf(ca), X = rx * cosf(ca) - ry * sinf(ca), Y = c->h - J[j][1];
        float Z2 = Z * cosf(q) + Y * sinf(q), Y2 = Y * cosf(q) - Z * sinf(q);
        depth[j] = Z2;
        float hpx = f * 1.75f / Z2;
        lm[j][0] = (0.5f * IMG_W + f * X / Z2 + noise * hpx * (float)gauss()) / IMG_W;
        lm[j][1] = (0.5f * IMG_H + f * Y2 / Z2 + noise * hpx * (float)gauss()) / IMG_H;
        lm[j][2] = 0.9f;
        if (lm[j][0] < 0 || lm[j][0] > 1 || lm[j][1] < 0 || lm[j][1] > 1)
            lm[j][2] = 0.2f; /* off the frame: the pose model still places it, unsure */
    }
    /* side view (hips more than 60% closed up in the picture): the far leg is behind the near one */
    float hipw = fabsf(lm[LHIP][0] - lm[RHIP][0]) * IMG_W, full = f * 0.18f / depth[LHIP];
    if (hipw < 0.6f * full) {
        int far = depth[LHIP] > depth[RHIP] ? 0 : 1;
        lm[LAN + far][2] = fminf(lm[LAN + far][2], vs->far_vis);
        lm[LKN + far][2] = fminf(lm[LKN + far][2], vs->far_knee >= 0 ? vs->far_knee : vs->far_vis);
        lm[NJ + 2 + far][2] = lm[NJ + 4 + far][2] = lm[LAN + far][2];
    }
}

typedef struct {
    int on;
    float x, dx, t;
} one_euro;
static float oe_alpha(float cutoff, float dt)
{
    float tau = 1.0f / (6.2831853f * cutoff);
    return 1.0f / (1.0f + tau / dt);
}
static float oe_step(one_euro *o, float x, float t)
{
    if (!o->on) {
        o->on = 1, o->x = x, o->dx = 0, o->t = t;
        return x;
    }
    float dt = fmaxf(1e-3f, t - o->t);
    o->t = t;
    o->dx += oe_alpha(1.0f, dt) * ((x - o->x) / dt - o->dx);
    o->x += oe_alpha(1.0f + 0.007f * fabsf(o->dx), dt) * (x - o->x);
    return o->x;
}

typedef struct {
    one_euro fb, fh;
    int old_gate;    /* the upright gate as it was: every box under 1.5 tall per wide dropped */
    long dropped_gate, dropped_conf, sent;
} phone_t;

/* phone.html isUpright. body: shoulders, hips and one whole leg all shown (visibility over 0.5). */
static int phone_upright(const phone_t *ph, const float lm[NB][3], const float box[4])
{
    const float v = 0.5f;
    int leg = (lm[LKN][2] > v && lm[LAN][2] > v) || (lm[RKN][2] > v && lm[RAN][2] > v);
    int body = lm[LSH][2] > v && lm[RSH][2] > v && lm[LHIP][2] > v && lm[RHIP][2] > v && leg;
    float min_aspect = !ph->old_gate && body ? 0.7f : 1.5f;
    if (box[2] > 0 && box[3] / box[2] < min_aspect)
        return 0;
    int hip = lm[LHIP][2] > 0.3f && lm[RHIP][2] > 0.3f, sh = lm[LSH][2] > 0.3f && lm[RSH][2] > 0.3f;
    float hip_y = 0.5f * (lm[LHIP][1] + lm[RHIP][1]), sh_y = 0.5f * (lm[LSH][1] + lm[RSH][1]);
    float ank_y = -1;
    for (int s = 0; s < 2; s++)
        if (lm[LAN + s][2] > v)
            ank_y = fmaxf(ank_y, lm[LAN + s][1]);
    if (sh && hip && !(sh_y < hip_y))
        return 0;
    if (lm[NOSE][2] > 0.3f && hip && !(lm[NOSE][1] < hip_y))
        return 0;
    if (hip && ank_y >= 0 && !(hip_y < ank_y))
        return 0;
    return 1;
}

/* One phone frame: the contact phone.html would send for these landmarks, or 0 when it drops it. */
static int phone_contact(phone_t *ph, const char *node, const float lm[NB][3], float t_s, seeker_contact *c)
{
    const float v = 0.5f, f = focal();
    float conf = fminf(1.0f, 0.5f * (lm[LHIP][2] + lm[RHIP][2]));
    if (conf < 0.6f) {
        ph->dropped_conf++;
        return 0;
    }
    float x0 = 1, y0 = 1, x1 = 0, y1 = 0;
    for (int j = 0; j < NB; j++)
        if (lm[j][2] >= 0.3f) {
            x0 = fminf(x0, lm[j][0]), x1 = fmaxf(x1, lm[j][0]);
            y0 = fminf(y0, lm[j][1]), y1 = fmaxf(y1, lm[j][1]);
        }
    float box[4] = { roundf(x0 * IMG_W), roundf(y0 * IMG_H), roundf((x1 - x0) * IMG_W),
                     roundf((y1 - y0) * IMG_H) };
    if (!phone_upright(ph, lm, box)) {
        ph->dropped_gate++;
        return 0;
    }
    float hpx = box[3];
    int na = (lm[LAN][2] > v) + (lm[RAN][2] > v), ne = (lm[LEYE][2] > v) + (lm[REYE][2] > v);
    if (na && (ne || lm[NOSE][2] > v)) {
        float tx = 0, ty = 0, ax = 0, ay = 0;
        if (ne) {
            for (int s = 0; s < 2; s++)
                if (lm[LEYE + s][2] > v)
                    tx += lm[LEYE + s][0] / (float)ne, ty += lm[LEYE + s][1] / (float)ne;
        } else {
            tx = lm[NOSE][0], ty = lm[NOSE][1];
        }
        for (int s = 0; s < 2; s++)
            if (lm[LAN + s][2] > v)
                ax += lm[LAN + s][0] / (float)na, ay += lm[LAN + s][1] / (float)na;
        hpx = hypotf((tx - ax) * IMG_W, (ty - ay) * IMG_H) / 0.90f;
    } else if (lm[LSH][2] > v && lm[RSH][2] > v && lm[LHIP][2] > v && lm[RHIP][2] > v) {
        hpx = hypotf(0.5f * (lm[LSH][0] + lm[RSH][0] - lm[LHIP][0] - lm[RHIP][0]) * IMG_W,
                     0.5f * (lm[LSH][1] + lm[RSH][1] - lm[LHIP][1] - lm[RHIP][1]) * IMG_H) / 0.30f;
    }
    float b = atanf((0.5f * (lm[LHIP][0] + lm[RHIP][0]) * IMG_W - 0.5f * IMG_W) / f) * RAD2DEG;
    b = oe_step(&ph->fb, b, t_s);
    hpx = oe_step(&ph->fh, hpx, t_s);
    memset(c, 0, sizeof *c);
    snprintf(c->id, sizeof c->id, "%s-p1", node);
    snprintf(c->cls, sizeof c->cls, "person");
    c->bearing_deg = roundf(b * 10) / 10;
    c->conf = conf;
    c->has_box = 1;
    memcpy(c->box, box, sizeof box);
    c->has_range = 1;
    c->range_m = roundf(f * 1.7f / hpx * 100) / 100;
    c->has_pts = 1;
    for (int j = 0; j < NJ; j++) { /* packPts: x, y to 0.001, visibility to 0.01 */
        c->pts[j][0] = roundf(lm[j][0] * 1000) / 1000;
        c->pts[j][1] = roundf(lm[j][1] * 1000) / 1000;
        c->pts[j][2] = roundf(lm[j][2] * 100) / 100;
    }
    ph->sent++;
    return 1;
}

/* ---- still poses: features and labels ---- */

static const cam_t CAM0 = { 0, 0, 0, 1.3f, -8.0f };

/* The label fusion's posture gives a still pose seen from az_deg (0 facing the camera, 90 side
 * on) at dist_m, from the label cur, and its features. Person-local joints as hud_fusion makes
 * them (seeker_skel_local at the phone's own range, the frame-edge mask). */
static int still_label(const pose_t *p, float az_deg, float dist_m, const vis_t *vs, int cur,
                       seeker_posture_feat *ft)
{
    float J[NB][3], lm[NB][3], loc[NJ][3];
    body3(p, J);
    project((const float (*)[3])J, 0, dist_m, 180.0f + az_deg, &CAM0, vs, 0, lm);
    seeker_contact c;
    memset(&c, 0, sizeof c);
    c.has_pts = 1;
    memcpy(c.pts, lm, sizeof c.pts);
    if (!seeker_skel_local(&c, IMG_W, IMG_H, HFOV, dist_m, loc))
        return -1;
    seeker_posture_mask_offframe(loc, (const float (*)[3])lm);
    seeker_posture_feat tmp;
    if (!ft)
        ft = &tmp;
    if (!seeker_posture_features((const float (*)[3])loc, ft))
        return SEEKER_POSTURE_UNKNOWN;
    return seeker_posture_classify(ft, cur);
}

static const float AZ[] = { 0, 30, 60, 90 }, DIST[] = { 2.5f, 3.0f, 4.0f };
#define N_AZ (int)(sizeof AZ / sizeof AZ[0])
#define N_DIST (int)(sizeof DIST / sizeof DIST[0])

static float g_table_m = 3.0f;
static void table(void)
{
    static const float depth[] = { 0, 0.2f, 0.4f, 0.5f, 0.6f, 0.65f, 0.75f, 0.9f, 1.0f };
    printf("still poses from STAND (camera 1.3 m up, 8 deg down, %.1f m): label and features\n", (double)g_table_m);
    for (int a = 0; a < N_AZ; a++)
        for (size_t d = 0; d < sizeof depth / sizeof depth[0]; d++) {
            pose_t p;
            pose_crouch(depth[d], &p);
            seeker_posture_feat f;
            int l = still_label(&p, AZ[a], g_table_m, &VIS_ALL, SEEKER_POSTURE_STAND, &f);
            char why[160];
            seeker_posture_describe(&f, why, sizeof why);
            printf("  az %2.0f depth %.2f: %-7s %s\n", AZ[a], depth[d], seeker_posture_name(l), why);
        }
}

static void test_still(void)
{
    int miss = 0, n = 0, bad = 0, nb = 0;
    for (int a = 0; a < N_AZ; a++)
        for (int d = 0; d < N_DIST; d++) {
            pose_t p;
            seeker_posture_feat f;
            char why[160];
            /* an ordinary crouch and a full one, from standing: the votes need CROUCH itself */
            static const float deep[] = { 0.7f, 0.75f, 1.0f };
            for (int k = 0; k < 3; k++) {
                pose_crouch(deep[k], &p);
                int l = still_label(&p, AZ[a], DIST[d], &VIS_ALL, SEEKER_POSTURE_STAND, &f);
                seeker_posture_describe(&f, why, sizeof why);
                n++;
                if (l != SEEKER_POSTURE_CROUCH) {
                    miss++;
                    printf("  crouch depth %.2f az %.0f at %.1f m reads %s (%s)\n", deep[k], AZ[a], DIST[d],
                           seeker_posture_name(l), why);
                }
            }
            /* never CROUCH, from STAND or from UNKNOWN: standing, a slight knee bend, a stair step
             * (either foot up), bending over, every phase of a walk */
            pose_t np[32];
            int nn = 0;
            pose_crouch(0, &np[nn++]);
            pose_crouch(0.2f, &np[nn++]);
            pose_stair(0, &np[nn++]);
            pose_stair(1, &np[nn++]);
            pose_bend(40, &np[nn++]);
            pose_bend(70, &np[nn++]);
            for (int w = 0; w < 16; w++)
                pose_walk((float)w / 16.0f, &np[nn++]);
            for (int k = 0; k < nn; k++)
                for (int cur = SEEKER_POSTURE_UNKNOWN; cur <= SEEKER_POSTURE_STAND; cur++) {
                    int l = still_label(&np[k], AZ[a], DIST[d], &VIS_ALL, cur, &f);
                    nb++;
                    if (l == SEEKER_POSTURE_CROUCH) {
                        bad++;
                        seeker_posture_describe(&f, why, sizeof why);
                        printf("  upright pose %d az %.0f at %.1f m reads crouch (%s)\n", k, AZ[a], DIST[d], why);
                    }
                }
            /* standing reads STAND */
            pose_crouch(0, &p);
            WANT(still_label(&p, AZ[a], DIST[d], &VIS_ALL, SEEKER_POSTURE_UNKNOWN, NULL) == SEEKER_POSTURE_STAND,
                 "standing az %.0f at %.1f m is STAND", AZ[a], DIST[d]);
        }
    printf("still poses: ordinary and full crouch read CROUCH in %d of %d views; upright poses read "
           "CROUCH in %d of %d\n", n - miss, n, bad, nb);
    WANT(miss == 0, "an ordinary crouch reads CROUCH from every view (%d of %d missed)", miss, n);
    WANT(bad == 0, "standing, knee bend, stair step, bending and walking never read CROUCH (%d did)", bad);

    /* side on with the far ankle hidden behind the near leg (its knee still shows) */
    pose_t p;
    vis_t far_ankle = { 0.3f, 0.8f }, far_leg = { 0.2f, 0.2f };
    seeker_posture_feat f;
    char why[160];
    pose_crouch(0.65f, &p);
    int l = still_label(&p, 90, 3.0f, &far_ankle, SEEKER_POSTURE_STAND, &f);
    seeker_posture_describe(&f, why, sizeof why);
    printf("side crouch, far ankle hidden: %s (%s)\n", seeker_posture_name(l), why);
    WANT(l == SEEKER_POSTURE_CROUCH, "side crouch with the far ankle hidden reads CROUCH");
    /* a stair step from the side with the lower leg's ankle hidden stays as it was, and with the
     * whole far leg hidden one folded leg is still no evidence */
    for (int up = 0; up < 2; up++) {
        pose_stair(up, &p);
        WANT(still_label(&p, 90, 3.0f, &far_ankle, SEEKER_POSTURE_STAND, NULL) == SEEKER_POSTURE_STAND,
             "stair step from the side, far ankle hidden, stays STAND (foot %d up)", up);
        WANT(still_label(&p, 90, 3.0f, &far_leg, SEEKER_POSTURE_STAND, NULL) == SEEKER_POSTURE_STAND,
             "stair step from the side, far leg hidden, stays STAND (foot %d up)", up);
    }
}

/* ---- replay ---- */

typedef struct {
    float x, y, yaw;
} pose3;
static const pose3 R1 = { 0.0f, -2.0f, 0.0f }, R2 = { -2.2f, 0.6f, 81.0f };
static const float PERSON[2] = { 0.3f, 1.0f };
#define CAM_H 1.3f
#define CAM_PITCH -8.0f

enum { SC_FACE_R1, SC_FACE_R2, SC_DEEP, SC_WALK, SC_STAIRS, SC_BEND, N_SC };
static const char *const SC_NAME[] = { "crouch facing R1", "crouch facing R2 (side on to R1)",
                                       "full crouch facing R1", "walking on the spot", "stair steps",
                                       "bending over" };
#define T_DOWN0 2000 /* starts going down */
#define T_DOWN1 2500 /* is down */
#define T_UP0 5000
#define T_UP1 5500
#define T_END 7000
#define DEPTH 0.65f

static void scene_pose(int sc, int64_t t, pose_t *p, float *face)
{
    float to_r1 = atan2f(R1.x - PERSON[0], R1.y - PERSON[1]) * RAD2DEG;
    float to_r2 = atan2f(R2.x - PERSON[0], R2.y - PERSON[1]) * RAD2DEG;
    *face = sc == SC_FACE_R2 ? to_r2 : to_r1;
    float s = t < T_DOWN0 ? 0 : t < T_DOWN1 ? (float)(t - T_DOWN0) / (T_DOWN1 - T_DOWN0)
              : t < T_UP0 ? 1 : t < T_UP1 ? 1 - (float)(t - T_UP0) / (T_UP1 - T_UP0) : 0;
    s = s * s * (3 - 2 * s);
    switch (sc) {
    case SC_WALK:
        pose_walk((float)(t % 1000) / 1000.0f, p);
        break;
    case SC_STAIRS:
        pose_stair((int)(t / 600) & 1, p);
        break;
    case SC_BEND:
        pose_bend(65.0f * s, p);
        break;
    default:
        pose_crouch((sc == SC_DEEP ? 1.0f : DEPTH) * s, p);
    }
}

typedef struct {
    int ticks, not_one, ids;        /* fusion ticks from 500 ms; ticks without exactly one track */
    int frames, xray, none;         /* frames; x-ray people drawn; frames with nobody drawn */
    int crouch_ticks, down_ticks;   /* ticks labelled CROUCH while down (T_DOWN1 + 500 to T_UP0) */
    int crouch_elsewhere;           /* ticks labelled CROUCH while standing (before T_DOWN0) */
    int64_t t_crouch, t_stand;      /* first CROUCH label after T_DOWN0, first STAND after T_UP0 (-1) */
    float stand_w, stand_h;         /* SEEN box while standing (median of 1 s to T_DOWN0), canvas px */
    float min_w, max_w, max_h;      /* SEEN box while down, over the standing one */
    int thin;                       /* frames while down with the box under 0.6 of the standing width */
    int tall;                       /* ... or over 0.85 of the standing height */
    float max_err;                  /* farthest the one track is from the person, m */
    long r1_gate, r1_sent;
} metrics;

static int cmp_f(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return x < y ? -1 : x > y;
}

static void run(int sc, int old_phone, const char *cfg_path, metrics *out)
{
    static seeker_config cfg;
    seeker_config_load(&cfg, cfg_path);
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    seeker_pose p1 = { R1.x, R1.y, R1.yaw }, p2 = { R2.x, R2.y, R2.yaw };
    seeker_fusion_set_pose(&f, "R1", &p1, "cli", 1);
    seeker_fusion_set_pose(&f, "R2", &p2, "cli", 1);

    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    snprintf(v.cfg.node, sizeof v.cfg.node, "R1");
    v.cfg.pose.x_m = R1.x, v.cfg.pose.y_m = R1.y, v.cfg.pose.yaw_deg = R1.yaw;
    v.cfg.cam.image_w = IMG_W, v.cfg.cam.image_h = IMG_H;
    v.cfg.cam.cam_h_m = CAM_H, v.cfg.cam.pitch_deg = CAM_PITCH;
    v.cfg.inset = 0;
    enum { W = 640, H = 480 };
    static uint32_t px[W * H];
    canvas c = { px, W, H, W };

    enum { MAXM = 512, MAXB = 256 };
    static seeker_remote_contacts q[MAXM];
    static int64_t q_at[MAXM];
    int nq = 0;
    int64_t next_gen[2] = { 0, 33 };
    static seeker_tracks tracks, latest;
    char buf[SEEKER_MAX_DATAGRAM], err[64];
    int have = 0, ids[8], nids = 0;
    phone_t ph[2];
    memset(ph, 0, sizeof ph);
    ph[0].old_gate = ph[1].old_gate = old_phone;
    static float sw[MAXB], sh[MAXB];
    int nsw = 0;
    static int dw[MAXB], dh[MAXB];
    int nd = 0;
    memset(out, 0, sizeof *out);
    out->t_crouch = out->t_stand = -1;

    for (int64_t now = 0; now < T_END; now++) {
        for (int n = 0; n < 2; n++)
            if (now >= next_gen[n] && nq < MAXM) {
                const pose3 *rp = n ? &R2 : &R1;
                cam_t cam = { rp->x, rp->y, rp->yaw, CAM_H, CAM_PITCH };
                seeker_remote_contacts *m = &q[nq];
                memset(m, 0, sizeof *m);
                snprintf(m->node_id, sizeof m->node_id, "%s", n ? "R2" : "R1");
                m->v = 2;
                m->t_mono_ms = now + (n ? 90000 : 50000);
                m->width = IMG_W, m->height = IMG_H, m->hfov_deg = HFOV;
                pose_t bp;
                float face, J[NB][3], lm[NB][3];
                scene_pose(sc, now, &bp, &face);
                body3(&bp, J);
                project((const float (*)[3])J, PERSON[0], PERSON[1], face, &cam, &VIS_ALL, 0.008f, lm);
                int miss = !n && urand() < 0.12; /* R1's own missed frames */
                if (!miss && phone_contact(&ph[n], m->node_id, (const float (*)[3])lm, (float)now / 1000.0f, &m->c[0]))
                    m->n = 1;
                q_at[nq++] = now + 20 + (int64_t)(20 * urand());
                next_gen[n] = now + 66 + (int64_t)(4 * gauss());
            }
        for (int i = 0; i < nq; i++)
            if (q_at[i] == now)
                seeker_fusion_ingest(&f, &q[i], now);
        if (now % 33 == 0) {
            seeker_fusion_tick(&f, now, &tracks);
            int len = seeker_encode_tracks(&tracks, buf, sizeof buf);
            if (len > 0 && seeker_decode_tracks(buf, len, &latest, err, sizeof err) == 0)
                have = 1;
            if (now >= 500) {
                out->ticks++;
                out->not_one += latest.n != 1;
                for (int i = 0; i < latest.n; i++) {
                    const seeker_track *k = &latest.tracks[i];
                    int known = 0;
                    for (int j = 0; j < nids; j++)
                        known |= ids[j] == k->id;
                    if (!known && nids < 8)
                        ids[nids++] = k->id;
                    if (k->has_room)
                        out->max_err = fmaxf(out->max_err, hypotf(k->room_x_m - PERSON[0], k->room_y_m - PERSON[1]));
                }
                int lab = latest.n == 1 ? latest.tracks[0].posture : -1;
                if (lab == SEEKER_POSTURE_CROUCH) {
                    if (now < T_DOWN0)
                        out->crouch_elsewhere++;
                    if (now >= T_DOWN0 && out->t_crouch < 0)
                        out->t_crouch = now;
                }
                if (now >= T_DOWN1 + 500 && now < T_UP0) {
                    out->down_ticks++;
                    out->crouch_ticks += lab == SEEKER_POSTURE_CROUCH;
                }
                if (lab == SEEKER_POSTURE_STAND && now >= T_UP0 && out->t_stand < 0)
                    out->t_stand = now;
            }
        }
        if (now % 33 != 11 || !have)
            continue;
        v.clock_ms = now + 1;
        wv_render(&c, &cfg, &v, &latest, 0, NULL);
        if (now < 500)
            continue;
        out->frames++;
        out->none += v.last.n_drawn == 0;
        for (int i = 0; i < v.last.n_drawn; i++) {
            if (!v.last.drawn[i].direct) {
                out->xray++;
                continue;
            }
            if (now >= 1000 && now < T_DOWN0 && nsw < MAXB)
                sw[nsw] = (float)v.last.drawn[i].w, sh[nsw++] = (float)v.last.drawn[i].h;
            if (now >= T_DOWN1 + 300 && now < T_UP0 && nd < MAXB)
                dw[nd] = v.last.drawn[i].w, dh[nd++] = v.last.drawn[i].h;
        }
    }
    out->ids = nids;
    if (nsw > 0) {
        qsort(sw, (size_t)nsw, sizeof sw[0], cmp_f);
        qsort(sh, (size_t)nsw, sizeof sh[0], cmp_f);
        out->stand_w = sw[nsw / 2], out->stand_h = sh[nsw / 2];
        out->min_w = 1e9f;
        for (int i = 0; i < nd; i++) {
            float rw = (float)dw[i] / out->stand_w, rh = (float)dh[i] / out->stand_h;
            out->min_w = fminf(out->min_w, rw), out->max_w = fmaxf(out->max_w, rw);
            out->max_h = fmaxf(out->max_h, rh);
            out->thin += rw < 0.6f;
            out->tall += rh > 0.85f;
        }
    }
    out->r1_gate = ph[0].dropped_gate, out->r1_sent = ph[0].sent;
}

static void test_replay(const char *cfg_path, unsigned seed, int old_phone)
{
    printf("replay (%s phone gate): stand, down at %d to %d ms (depth %.2f), up at %d to %d ms\n",
           old_phone ? "old" : "new", T_DOWN0, T_DOWN1, (double)DEPTH, T_UP0, T_UP1);
    for (int sc = 0; sc < N_SC; sc++) {
        metrics m;
        g_seed = seed + (unsigned)sc;
        run(sc, old_phone, cfg_path, &m);
        int crouch_sc = sc <= SC_DEEP;
        printf("%s\n  tracks: %d ids, %d of %d ticks not exactly one, farthest %.2f m off; frames %d: x-ray "
               "drawn %d, nobody drawn %d; R1 sent %ld, upright gate dropped %ld\n",
               SC_NAME[sc], m.ids, m.not_one, m.ticks, (double)m.max_err, m.frames, m.xray, m.none, m.r1_sent,
               m.r1_gate);
        if (crouch_sc) {
            printf("  posture: CROUCH %lld ms after the crouch starts (%lld ms after being down), CROUCH %d "
                   "of %d ticks while down, STAND %lld ms after standing up starts, %d CROUCH ticks before\n",
                   (long long)(m.t_crouch < 0 ? -1 : m.t_crouch - T_DOWN0),
                   (long long)(m.t_crouch < 0 ? -1 : m.t_crouch - T_DOWN1), m.crouch_ticks, m.down_ticks,
                   (long long)(m.t_stand < 0 ? -1 : m.t_stand - T_UP0), m.crouch_elsewhere);
            printf("  SEEN box: standing %.0f x %.0f px; down: width %.2f to %.2f, height up to %.2f of "
                   "standing; frames under 0.6 wide %d, over 0.85 tall %d\n",
                   (double)m.stand_w, (double)m.stand_h, (double)m.min_w, (double)m.max_w, (double)m.max_h,
                   m.thin, m.tall);
        } else {
            printf("  posture: %d CROUCH ticks\n", m.crouch_elsewhere + (m.t_crouch >= 0));
        }
        if (old_phone)
            continue;
        WANT(m.ids == 1 && m.not_one == 0, "%s: exactly one track on every tick (%d ids, %d ticks off)",
             SC_NAME[sc], m.ids, m.not_one);
        WANT(m.xray == 0 && m.none == 0, "%s: SEEN on every frame, no x-ray copy (%d x-ray, %d empty)",
             SC_NAME[sc], m.xray, m.none);
        /* the phone smooths the pixel height it ranges from, so its range trails a person
         * standing up by about 0.2 s: up to 0.7 m for a tick or two when only one phone has them */
        WANT(m.max_err < 0.8f, "%s: the track stays on the person (%.2f m)", SC_NAME[sc], (double)m.max_err);
        if (crouch_sc) {
            WANT(m.t_crouch >= 0 && m.t_crouch - T_DOWN1 <= 600,
                 "%s: CROUCH about 0.5 s after being down (%lld ms)", SC_NAME[sc],
                 (long long)(m.t_crouch < 0 ? -1 : m.t_crouch - T_DOWN1));
            WANT(m.crouch_ticks >= m.down_ticks - 3 && m.down_ticks > 0, "%s: CROUCH the whole time down (%d of %d)",
                 SC_NAME[sc], m.crouch_ticks, m.down_ticks);
            WANT(m.crouch_elsewhere == 0, "%s: no CROUCH while standing", SC_NAME[sc]);
            WANT(m.t_stand >= 0 && m.t_stand - T_UP1 <= 700, "%s: STAND again after standing up", SC_NAME[sc]);
            /* side on, a standing person is narrow and a crouched one is not: no upper bound there */
            WANT(m.thin == 0 && m.min_w >= 0.6f && (sc == SC_FACE_R2 || m.max_w <= 1.9f),
                 "%s: the crouched SEEN box is about as wide as the standing one (%.2f to %.2f, %d thin frames)",
                 SC_NAME[sc], (double)m.min_w, (double)m.max_w, m.thin);
            WANT(m.tall == 0, "%s: the crouched SEEN box is shorter than the standing one (up to %.2f, %d tall frames)",
                 SC_NAME[sc], (double)m.max_h, m.tall);
        } else {
            WANT(m.crouch_elsewhere == 0 && m.t_crouch < 0, "%s: never CROUCH", SC_NAME[sc]);
        }
    }
}

/* ---- the seen / other-phone duplicate check (wv_classify) ---- */

static void dup_track(seeker_track *k, int id, const char *node, const pose3 *np, float x, float y)
{
    memset(k, 0, sizeof *k);
    k->id = id;
    snprintf(k->node_id, sizeof k->node_id, "%s", node);
    snprintf(k->nodes, sizeof k->nodes, "%s", node);
    k->has_room = 1, k->room_x_m = x, k->room_y_m = y;
    float b = atan2f(x - np->x, y - np->y) * RAD2DEG - np->yaw;
    k->node_bearing_deg = b > 180 ? b - 360 : b <= -180 ? b + 360 : b;
}

/* R1 sees the person at (px, py); R2's own track of someone sits at k2 times R2's distance to
 * (qx, qy) along R2's ray. Returns 1 when wv_classify calls R2's track a duplicate. */
static int dup_case(float px, float py, float qx, float qy, float k2, int handoff, int wall, int both)
{
    static seeker_tracks t;
    memset(&t, 0, sizeof t);
    t.n = 2;
    dup_track(&t.tracks[0], 1, "R1", &R1, px, py);
    if (both)
        snprintf(t.tracks[0].nodes, sizeof t.tracks[0].nodes, "R1+R2");
    dup_track(&t.tracks[1], 2, "R2", &R2, R2.x + k2 * (qx - R2.x), R2.y + k2 * (qy - R2.y));
    t.n_nodes = 2;
    for (int i = 0; i < 2; i++) {
        const pose3 *np = i ? &R2 : &R1;
        snprintf(t.nodes[i].node_id, sizeof t.nodes[i].node_id, "%s", i ? "R2" : "R1");
        snprintf(t.nodes[i].state, sizeof t.nodes[i].state, "ok");
        t.nodes[i].x_m = np->x, t.nodes[i].y_m = np->y, t.nodes[i].yaw_deg = np->yaw;
    }
    wv_config w;
    wv_config_defaults(&w);
    snprintf(w.node, sizeof w.node, "R1");
    w.pose.x_m = R1.x, w.pose.y_m = R1.y, w.pose.yaw_deg = R1.yaw;
    w.cam.image_w = IMG_W, w.cam.image_h = IMG_H;
    w.handoff = handoff;
    if (wall) { /* a wall across the room 0.5 m behind the person */
        w.n_walls = 1;
        w.walls[0][0] = -3, w.walls[0][1] = py + 0.5f, w.walls[0][2] = 3, w.walls[0][3] = py + 0.5f;
    }
    wv_item it[4];
    int n = wv_classify(&t, &w, &w.pose, it, 4);
    return n == 2 && it[1].dup_of == 0;
}

static void test_dup(void)
{
    const float px = PERSON[0], py = PERSON[1];
    /* R2's size range on a person who is not standing: up to 2x long, the track far along R2's
     * ray and 1 to 2.5 m to the right of the person as R1 sees them */
    static const float longk[] = { 1.3f, 1.6f, 1.9f, 2.1f };
    int miss = 0;
    for (int i = 0; i < 4; i++)
        miss += !dup_case(px, py, px, py, longk[i], 1, 0, 0);
    printf("duplicate check: R2's track 1.3 to 2.1x long on its ray: merged with the seen person in %d of 4\n",
           4 - miss);
    WANT(miss == 0, "an other-phone track far along its own ray is the seen person's duplicate (%d of 4 missed)", miss);
    WANT(dup_case(px, py, px, py, 1.0f, 1, 0, 0) && dup_case(px, py, px, py, 0.85f, 1, 0, 0),
         "an other-phone track on the seen person is their duplicate");
    WANT(dup_case(px, py, px, py, 1.6f, 0, 0, 0) == 0, "--handoff off keeps the bearing and range match alone");
    /* not duplicates: another person 1.6 m away (R2's ray misses the seen one), a person
     * much nearer R2 than the seen one on the same ray (short, not long), a second contact of a
     * phone that already sees the seen person, and someone past a wall behind the seen person */
    WANT(!dup_case(px, py, px - 1.0f, py + 1.2f, 1.0f, 1, 0, 0), "a second person off R2's ray to the seen one stays x-ray");
    WANT(!dup_case(px, py, px, py, 0.5f, 1, 0, 0), "a person much nearer R2 on the same ray stays x-ray");
    /* R2's ray crosses R1's line 1 m in front of the seen person and its track is 2x past the
     * crossing: both ranges would have to be far out at once */
    WANT(!dup_case(px, py + 1.0f, px, py, 2.0f, 1, 0, 0), "a hidden person whose ray passes in front of the seen one stays x-ray");
    WANT(!dup_case(px, py, px, py, 1.6f, 1, 0, 1), "a second contact of a phone that sees the seen person stays x-ray");
    WANT(!dup_case(px, py, px + 0.6f, py + 1.2f, 1.0f, 1, 1, 0), "a person behind a wall stays x-ray");
}

/* ---- the SEEN box (wv_render) ---- */

/* One frame of one track at the person's place; returns the drawn box (canvas px) in b. */
static int box_frame(wv_view *v, const seeker_config *cfg, int64_t t_ms, const char *nodes, const char *primary,
                     const float *node_box, int posture, int b[4])
{
    static seeker_tracks t;
    memset(&t, 0, sizeof t);
    t.n = 1;
    seeker_track *k = &t.tracks[0];
    dup_track(k, 7, primary, strcmp(primary, "R1") ? &R2 : &R1, PERSON[0], PERSON[1]);
    snprintf(k->nodes, sizeof k->nodes, "%s", nodes);
    snprintf(k->colour, sizeof k->colour, "cyan");
    k->posture = posture;
    if (node_box) {
        k->has_node_box = 1;
        memcpy(k->node_box, node_box, sizeof k->node_box);
    }
    t.n_nodes = 2;
    for (int i = 0; i < 2; i++) {
        const pose3 *np = i ? &R2 : &R1;
        snprintf(t.nodes[i].node_id, sizeof t.nodes[i].node_id, "%s", i ? "R2" : "R1");
        snprintf(t.nodes[i].state, sizeof t.nodes[i].state, "ok");
        t.nodes[i].x_m = np->x, t.nodes[i].y_m = np->y, t.nodes[i].yaw_deg = np->yaw;
    }
    enum { W = 640, H = 480 };
    static uint32_t px[W * H];
    canvas c = { px, W, H, W };
    v->clock_ms = t_ms;
    wv_render(&c, cfg, v, &t, 0, NULL);
    if (v->last.n_drawn != 1)
        return -1;
    b[0] = v->last.drawn[0].x, b[1] = v->last.drawn[0].y, b[2] = v->last.drawn[0].w, b[3] = v->last.drawn[0].h;
    return v->last.drawn[0].direct;
}

static void box_view(wv_view *v)
{
    memset(v, 0, sizeof *v);
    wv_config_defaults(&v->cfg);
    snprintf(v->cfg.node, sizeof v->cfg.node, "R1");
    v->cfg.pose.x_m = R1.x, v->cfg.pose.y_m = R1.y, v->cfg.pose.yaw_deg = R1.yaw;
    v->cfg.cam.image_w = IMG_W, v->cfg.cam.image_h = IMG_H;
    v->cfg.cam.cam_h_m = CAM_H, v->cfg.cam.pitch_deg = CAM_PITCH;
    v->cfg.inset = 0;
    v->box_ease_off = 1;
}

static void test_box(const char *cfg_path)
{
    static seeker_config cfg;
    seeker_config_load(&cfg, cfg_path);
    static wv_view v;
    /* R1's own box of a crouched person: 130 px wide, 150 tall (a standing one here: 85 x 280) */
    const float crouch_box[4] = { 280, 230, 130, 150 }, r2_box[4] = { 250, 200, 110, 170 };
    int b0[4], b1[4], b2[4], b3[4];
    /* frame 1: R1 sees them. frames 2, 3: R1's phone missed them (its confidence cut, the upright
     * gate), R2 leads the track; drawn as seen through the hold. */
    box_view(&v);
    int d0 = box_frame(&v, &cfg, 1000, "R1+R2", "R1", crouch_box, SEEKER_POSTURE_CROUCH, b0);
    int d1 = box_frame(&v, &cfg, 1066, "R2", "R2", r2_box, SEEKER_POSTURE_CROUCH, b1);
    int d2 = box_frame(&v, &cfg, 1300, "R2", "R2", r2_box, SEEKER_POSTURE_CROUCH, b2);
    printf("SEEN box: R1's own %d x %d; in a frame R1 missed %d x %d, 300 ms on %d x %d\n", b0[2], b0[3], b1[2],
           b1[3], b2[2], b2[3]);
    WANT(d0 == 1 && b0[2] == 130 && b0[3] == 150, "a seen person's box is the wearer phone's own box");
    WANT(d1 == 1 && b1[2] == b0[2] && b1[3] == b0[3] && b1[0] == b0[0] && b1[1] == b0[1],
         "a frame the wearer's phone missed keeps its last box (%d x %d, was %d x %d)", b1[2], b1[3], b0[2], b0[3]);
    WANT(d2 == 1 && b2[2] == b0[2] && b2[3] == b0[3], "and still 300 ms on, while the person stays seen");
    /* seen, but the wearer's phone never boxed them (another phone leads): the projected box,
     * a standing person's; crouched, as wide and WV_CROUCH_BOX_H as tall, on the floor */
    box_view(&v);
    int ds = box_frame(&v, &cfg, 1000, "R1+R2", "R2", r2_box, SEEKER_POSTURE_STAND, b2);
    box_view(&v);
    int dc = box_frame(&v, &cfg, 1000, "R1+R2", "R2", r2_box, SEEKER_POSTURE_CROUCH, b3);
    printf("SEEN box without one from R1: standing %d x %d, crouched %d x %d\n", b2[2], b2[3], b3[2], b3[3]);
    WANT(ds == 1 && dc == 1 && b3[2] == b2[2], "a crouched seen person's projected box is as wide as a standing one");
    WANT(abs(b3[3] - (int)lroundf(WV_CROUCH_BOX_H * (float)b2[3])) <= 2 && abs(b3[1] + b3[3] - b2[1] - b2[3]) <= 1,
         "... and %.1f as tall, from the floor (%d of %d px)", (double)WV_CROUCH_BOX_H, b3[3], b2[3]);
}

int main(int argc, char **argv)
{
    unsigned seed = 11;
    int old_phone = 0, tab = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--old-phone"))
            old_phone = 1;
        else if (!strcmp(argv[i], "--table")) {
            tab = 1;
            if (i + 1 < argc && atof(argv[i + 1]) > 0.5)
                g_table_m = (float)atof(argv[++i]);
        }
        else
            seed = (unsigned)atoi(argv[i]);
    }
    char cfg_path[512] = "config/seeker.live.json";
#ifdef SEEKER_SOURCE_DIR
    snprintf(cfg_path, sizeof cfg_path, "%s/config/seeker.live.json", SEEKER_SOURCE_DIR);
    if (access(cfg_path, R_OK) != 0)
        snprintf(cfg_path, sizeof cfg_path, "config/seeker.live.json");
#endif
    if (tab) {
        table();
        return 0;
    }
    draw_set_theme(DRAW_THEME_PLAN);
    anim_set_enabled(0);
    g_seed = seed;
    test_still();
    test_dup();
    test_box(cfg_path);
    test_replay(cfg_path, seed, old_phone);
    printf("%d checks, %d failed\n", g_checks, g_fails);
    return g_fails ? 1 : 0;
}
