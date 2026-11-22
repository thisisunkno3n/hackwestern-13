#include "seeker_posture.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define RAD2DEG 57.29577951308232f

enum { L_SH = 5, R_SH = 6, L_HIP = 11, R_HIP = 12, L_KN = 13, R_KN = 14, L_AN = 15, R_AN = 16 };

static seeker_posture_tune g_tune = { 0.5f, 120.0f, 0.45f, 0.41f, 150.0f, 0.6f, 0, 6, 9 };

void seeker_posture_tune_defaults(seeker_posture_tune *t)
{
    static const seeker_posture_tune d = { 0.5f, 120.0f, 0.45f, 0.41f, 150.0f, 0.6f, 0, 6, 9 };
    *t = d;
}

static int env_float(const char *name, float lo, float hi, float *out, char *bad, int badsz)
{
    const char *v = getenv(name);
    if (!v || !*v)
        return 0;
    char *end;
    double d = strtod(v, &end);
    if (end == v || *end || !(d >= lo && d <= hi)) {
        if (bad && badsz > 0) {
            size_t l = strlen(bad);
            snprintf(bad + l, (size_t)badsz - l, "%s%s", l ? "," : "", name);
        }
        return 0;
    }
    *out = (float)d;
    return 1;
}

int seeker_posture_tune_env(seeker_posture_tune *t, char *bad, int badsz)
{
    if (bad && badsz > 0)
        bad[0] = 0;
    int n = 0;
    n += env_float("SEEKER_POSTURE_LEG_VIS", 0.0f, 1.0f, &t->leg_vis, bad, badsz);
    n += env_float("SEEKER_POSTURE_CROUCH_KNEE", 30.0f, 180.0f, &t->crouch_knee, bad, badsz);
    n += env_float("SEEKER_POSTURE_CROUCH_THIGH", 0.0f, 1.5f, &t->crouch_thigh, bad, badsz);
    n += env_float("SEEKER_POSTURE_CROUCH_LEG", 0.0f, 1.0f, &t->crouch_leg, bad, badsz);
    n += env_float("SEEKER_POSTURE_STAND_KNEE", 30.0f, 180.0f, &t->stand_knee, bad, badsz);
    n += env_float("SEEKER_POSTURE_STAND_THIGH", 0.0f, 1.5f, &t->stand_thigh, bad, badsz);
    float one = (float)t->one_leg;
    if (env_float("SEEKER_POSTURE_ONE_LEG", 0.0f, 1.0f, &one, bad, badsz)) {
        t->one_leg = one >= 0.5f;
        n++;
    }
    const char *v = getenv("SEEKER_POSTURE_VOTES");
    if (v && *v) {
        int a, b;
        char c;
        if (sscanf(v, "%d/%d%c", &a, &b, &c) == 2 && b >= 1 && b <= SEEKER_POSTURE_VOTES_MAX &&
            a >= 1 && a <= b) {
            t->vote_n = a;
            t->vote_m = b;
            n++;
        } else if (bad && badsz > 0) {
            size_t l = strlen(bad);
            snprintf(bad + l, (size_t)badsz - l, "%sSEEKER_POSTURE_VOTES", l ? "," : "");
        }
    }
    return n;
}

void seeker_posture_set_tune(const seeker_posture_tune *t)
{
    g_tune = *t;
}

const seeker_posture_tune *seeker_posture_get_tune(void)
{
    return &g_tune;
}

void seeker_posture_tune_describe(const seeker_posture_tune *t, char *buf, int n)
{
    snprintf(buf, (size_t)n,
             "legs need knee and ankle at visibility %.2f inside the frame (no ankles: unknown); "
             "a leg is folded under %.0f deg, thigh drop %.2f or leg height %.2f, supporting over "
             "%.0f deg and %.2f; CROUCH needs both legs folded%s, STAND one supporting leg (stairs); "
             "a new label needs %d of the last %d frames (unknown %d of %d), held %d ms",
             t->leg_vis, t->crouch_knee, t->crouch_thigh, t->crouch_leg, t->stand_knee, t->stand_thigh,
             t->one_leg ? " (or one deeply folded)" : "", t->vote_n, t->vote_m,
             SEEKER_POSTURE_UNKNOWN_N, SEEKER_POSTURE_UNKNOWN_M, SEEKER_POSTURE_HOLD_MS);
}

static int vis(const float (*p)[3], int j, float min)
{
    return p[j][2] >= min;
}

/* Mean of the visible joints of a left/right pair. */
static int pair_mid(const float (*p)[3], int a, int b, float min, float *x, float *y)
{
    int va = vis(p, a, min), vb = vis(p, b, min), n = va + vb;
    if (!n)
        return 0;
    *x = ((va ? p[a][0] : 0) + (vb ? p[b][0] : 0)) / (float)n;
    *y = ((va ? p[a][1] : 0) + (vb ? p[b][1] : 0)) / (float)n;
    return 1;
}

static float angle_at(const float *a, const float *k, const float *b)
{
    float ux = a[0] - k[0], uy = a[1] - k[1], vx = b[0] - k[0], vy = b[1] - k[1];
    float nu = hypotf(ux, uy), nv = hypotf(vx, vy);
    if (nu < 1e-4f || nv < 1e-4f)
        return 180.0f;
    float c = (ux * vx + uy * vy) / (nu * nv);
    c = c < -1 ? -1 : c > 1 ? 1 : c;
    return acosf(c) * RAD2DEG;
}

int seeker_posture_features(const float (*p)[3], seeker_posture_feat *f)
{
    const seeker_posture_tune *t = &g_tune;
    memset(f, 0, sizeof *f);
    float sx, sy, hx, hy;
    if (!pair_mid(p, L_SH, R_SH, SEEKER_POSTURE_VIS_MIN, &sx, &sy) ||
        !pair_mid(p, L_HIP, R_HIP, SEEKER_POSTURE_VIS_MIN, &hx, &hy))
        return 0;
    float tx = hx - sx, ty = hy - sy, tl = hypotf(tx, ty);
    if (tl < 0.03f)
        return 0; /* shoulders on the hips: not a usable torso */
    f->has_torso = 1;
    float sw = vis(p, L_SH, SEEKER_POSTURE_VIS_MIN) && vis(p, R_SH, SEEKER_POSTURE_VIS_MIN)
                   ? hypotf(p[L_SH][0] - p[R_SH][0], p[L_SH][1] - p[R_SH][1]) : 0.0f;
    f->torso_deg = atan2f(fabsf(tx), ty) * RAD2DEG; /* 0 hips below shoulders, 180 inverted */
    f->has_knees = vis(p, L_KN, t->leg_vis) || vis(p, R_KN, t->leg_vis);

    static const int legs[2][3] = { { L_HIP, L_KN, L_AN }, { R_HIP, R_KN, R_AN } };
    for (int s = 0; s < 2; s++) {
        const float *hp = p[legs[s][0]], *kn = p[legs[s][1]], *an = p[legs[s][2]];
        if (!vis(p, legs[s][0], SEEKER_POSTURE_VIS_MIN) || !vis(p, legs[s][1], t->leg_vis) ||
            !vis(p, legs[s][2], t->leg_vis))
            continue;
        float shin = hypotf(an[0] - kn[0], an[1] - kn[1]);
        float thigh = hypotf(kn[0] - hp[0], kn[1] - hp[1]);
        if (shin < 0.3f * tl)
            continue; /* ankle on the knee: an invented (hidden) lower leg */
        seeker_posture_leg *l = &f->leg[s];
        l->ok = 1;
        l->knee_deg = angle_at(hp, kn, an);
        l->thigh_drop = (kn[1] - hp[1]) / tl;
        l->thigh_ratio = thigh / tl;
        l->seg = fmaxf(thigh, shin);
        l->leg_h = an[1] - hp[1];
        if (!f->n_legs++ || l->knee_deg > f->knee_deg) {
            f->knee_deg = l->knee_deg;
            f->thigh_drop = l->thigh_drop;
        }
    }
    f->has_knee_angle = f->n_legs > 0;
    /* The person's height in these units, from whatever the view shortens least: thigh or shin
     * (side on), the torso, the shoulder line (facing the camera). */
    float body = fmaxf(tl / 0.29f, sw / 0.25f);
    for (int s = 0; s < 2; s++)
        if (f->leg[s].ok)
            body = fmaxf(body, f->leg[s].seg / 0.245f);
    /* only against a whole torso: with a shoulder or hip hidden or off the frame the torso is the
     * line to the one that shows, tilted and too long, and every leg would look short */
    int whole = vis(p, L_SH, SEEKER_POSTURE_VIS_MIN) && vis(p, R_SH, SEEKER_POSTURE_VIS_MIN) &&
                vis(p, L_HIP, SEEKER_POSTURE_VIS_MIN) && vis(p, R_HIP, SEEKER_POSTURE_VIS_MIN);
    for (int s = 0; s < 2; s++)
        f->leg[s].leg_h = whole ? f->leg[s].leg_h / body : 9.0f;
    /* One full leg and the other one's ankle hidden (behind the near leg in a side view): its
     * thigh still shows, against the full leg's length. */
    for (int s = 0; s < 2 && f->n_legs == 1; s++) {
        seeker_posture_leg *l = &f->leg[s];
        const seeker_posture_leg *o = &f->leg[!s];
        if (l->ok || !o->ok || !vis(p, legs[s][0], SEEKER_POSTURE_VIS_MIN) || !vis(p, legs[s][1], t->leg_vis))
            continue;
        l->half = 1;
        l->thigh_h = (p[legs[s][1]][1] - p[legs[s][0]][1]) / o->seg;
    }
    if (!f->n_legs && f->has_knees) {
        float kx, ky;
        pair_mid(p, L_KN, R_KN, t->leg_vis, &kx, &ky);
        f->thigh_drop = (ky - hy) / tl;
    }
    if (f->has_knees) {
        static const int body[] = { L_SH, R_SH, L_HIP, R_HIP, L_KN, R_KN, L_AN, R_AN };
        float x0 = 1e9f, x1 = -1e9f, y0 = 1e9f, y1 = -1e9f;
        for (size_t i = 0; i < sizeof body / sizeof body[0]; i++)
            if (vis(p, body[i], i < 4 ? SEEKER_POSTURE_VIS_MIN : t->leg_vis)) {
                x0 = fminf(x0, p[body[i]][0]);
                x1 = fmaxf(x1, p[body[i]][0]);
                y0 = fminf(y0, p[body[i]][1]);
                y1 = fmaxf(y1, p[body[i]][1]);
            }
        f->has_flat = 1;
        f->flat = (x1 - x0) / fmaxf(y1 - y0, 0.02f);
    }
    return 1;
}

int seeker_posture_classify(const seeker_posture_feat *f, int cur)
{
    const seeker_posture_tune *t = &g_tune;
    if (!f->has_torso)
        return SEEKER_POSTURE_UNKNOWN;
    int crouch = cur == SEEKER_POSTURE_CROUCH, stand = cur == SEEKER_POSTURE_STAND;
    int down = cur == SEEKER_POSTURE_DOWN;
    int support = 0, folded = 0, deep = 0, half_folded = 0;
    for (int s = 0; s < 2; s++) {
        const seeker_posture_leg *l = &f->leg[s];
        if (l->half && l->thigh_h < 0.7f + (crouch ? 0.1f : 0.0f))
            half_folded++;
        if (!l->ok)
            continue;
        /* low: a leg folded under the body is short in the picture even when its knee points at
         * the camera and the 2D knee angle stays straight; with the knee nearer the hip than on
         * a hanging thigh, so a body scale thrown off by guessed joints is not enough */
        int low = l->leg_h < t->crouch_leg + (crouch ? 0.02f : 0.0f) &&
                  l->thigh_drop < SEEKER_POSTURE_LOW_THIGH + (crouch ? 0.05f : 0.0f);
        if (!low && l->knee_deg > t->stand_knee - (stand ? 10.0f : 0.0f) &&
            l->thigh_drop > t->stand_thigh - (stand ? 0.1f : 0.0f))
            support++;
        if (low || l->knee_deg < t->crouch_knee + (crouch ? 15.0f : 0.0f) ||
            l->thigh_drop < t->crouch_thigh + (crouch ? 0.1f : 0.0f))
            folded++;
        if (l->knee_deg < 100.0f && l->thigh_drop < 0.3f)
            deep++;
    }
    int flat = f->torso_deg > (down ? 50.0f : 65.0f) ||
               (f->has_flat && f->flat > (down ? 1.3f : 1.6f));
    if (flat && !support)
        return SEEKER_POSTURE_DOWN;
    if (support) {
        if (f->torso_deg < (stand ? 45.0f : 35.0f))
            return SEEKER_POSTURE_STAND;
        /* bent over on straight legs: neither crouching nor down */
        return crouch || down ? SEEKER_POSTURE_UNKNOWN : cur;
    }
    if (!f->n_legs)
        return SEEKER_POSTURE_UNKNOWN; /* no full leg (ankles hidden): standing or crouching, no guess */
    if (f->n_legs == 2 && folded == 2)
        return SEEKER_POSTURE_CROUCH;
    if (f->n_legs == 1 && folded == 1 && half_folded)
        return SEEKER_POSTURE_CROUCH; /* side on: the far ankle hidden, both thighs folded */
    if (f->n_legs == 1 && t->one_leg && deep)
        return SEEKER_POSTURE_CROUCH;
    return cur; /* in a dead band, or one folded leg (a stair step as often as a crouch) */
}

void seeker_posture_mask_offframe(float (*pts)[3], const float (*img)[3])
{
    for (int j = 0; j < 17; j++)
        if (!(img[j][0] > 0.005f && img[j][0] < 0.995f && img[j][1] > 0.005f &&
              img[j][1] < 0.995f))
            pts[j][2] = 0.0f;
}

/* Changes whenever the smoother has a new skeleton; a held one repeats exactly. */
static float signature(const float (*p)[3])
{
    float s = 0;
    for (int j = 0; j < 17; j++)
        s += (float)(j + 1) * (0.37f * p[j][0] + 0.61f * p[j][1] + 0.23f * p[j][2]);
    return s;
}

static int count_last(const seeker_posture *s, int label, int m)
{
    int n = 0;
    for (int i = 0; i < m && i < s->nvotes; i++) {
        int k = (s->head - 1 - i + SEEKER_POSTURE_VOTES_MAX) % SEEKER_POSTURE_VOTES_MAX;
        n += s->votes[k] == label;
    }
    return n;
}

int seeker_posture_update(seeker_posture *s, const float (*pts)[3], int64_t now_ms)
{
    const seeker_posture_tune *t = &g_tune;
    if (s->valid && (now_ms < s->t_ms || now_ms - s->t_ms > SEEKER_POSTURE_RESET_MS))
        s->valid = 0;
    if (!s->valid) {
        memset(s, 0, sizeof *s);
        s->valid = 1;
        s->label = s->cand = SEEKER_POSTURE_UNKNOWN;
        s->label_ms = now_ms;
    }
    s->t_ms = now_ms;
    float sig = pts ? signature(pts) : 0.0f;
    int64_t since = now_ms - s->vote_ms;
    if (s->nvotes) {
        int fresh = pts && sig != s->sig;
        if (since < (fresh ? SEEKER_POSTURE_VOTE_MIN_MS : SEEKER_POSTURE_VOTE_REPEAT_MS))
            return s->label;
    }
    s->sig = sig;
    s->vote_ms = now_ms;
    int c = SEEKER_POSTURE_UNKNOWN;
    if (pts && seeker_posture_features(pts, &s->feat))
        c = seeker_posture_classify(&s->feat, s->label);
    else
        memset(&s->feat, 0, sizeof s->feat);
    s->cand = c;
    s->votes[s->head] = c;
    s->head = (s->head + 1) % SEEKER_POSTURE_VOTES_MAX;
    if (s->nvotes < SEEKER_POSTURE_VOTES_MAX)
        s->nvotes++;
    if (now_ms - s->label_ms < SEEKER_POSTURE_HOLD_MS)
        return s->label;
    for (int l = SEEKER_POSTURE_UNKNOWN; l <= SEEKER_POSTURE_DOWN; l++) {
        if (l == s->label)
            continue;
        int need = l == SEEKER_POSTURE_UNKNOWN ? SEEKER_POSTURE_UNKNOWN_N : t->vote_n;
        int of = l == SEEKER_POSTURE_UNKNOWN ? SEEKER_POSTURE_UNKNOWN_M : t->vote_m;
        if (count_last(s, l, of) >= need) {
            s->label = l;
            s->label_ms = now_ms;
            break;
        }
    }
    return s->label;
}

void seeker_posture_describe(const seeker_posture_feat *f, char *buf, int n)
{
    if (!f->has_torso) {
        snprintf(buf, (size_t)n, "no skeleton");
        return;
    }
    int o = snprintf(buf, (size_t)n, "%.0f deg torso, ", f->torso_deg);
    if (o < 0 || o >= n)
        return;
    if (!f->n_legs) {
        snprintf(buf + o, (size_t)(n - o), f->has_knees ? "knees but no full leg (ankles hidden)"
                                                        : "no legs");
        return;
    }
    o += snprintf(buf + o, (size_t)(n - o), "legs:");
    for (int s = 0; s < 2 && o < n; s++) {
        const seeker_posture_leg *l = &f->leg[s];
        if (l->ok)
            o += snprintf(buf + o, (size_t)(n - o), " %s knee %.0f deg thigh drop %.2f len %.2f leg %.2f",
                          s ? "R" : "L", l->knee_deg, l->thigh_drop, l->thigh_ratio, l->leg_h);
        else if (l->half)
            o += snprintf(buf + o, (size_t)(n - o), " %s ankle hidden, thigh %.2f", s ? "R" : "L", l->thigh_h);
        else
            o += snprintf(buf + o, (size_t)(n - o), " %s hidden", s ? "R" : "L");
    }
}

const char *seeker_posture_name(int label)
{
    switch (label) {
    case SEEKER_POSTURE_STAND: return "stand";
    case SEEKER_POSTURE_CROUCH: return "crouch";
    case SEEKER_POSTURE_DOWN: return "down";
    default: return "unknown";
    }
}

int seeker_posture_parse(const char *name)
{
    for (int i = SEEKER_POSTURE_UNKNOWN; i <= SEEKER_POSTURE_DOWN; i++)
        if (!strcmp(name, seeker_posture_name(i)))
            return i;
    return -1;
}

const char *seeker_posture_word(int label)
{
    switch (label) {
    case SEEKER_POSTURE_STAND: return "STAND";
    case SEEKER_POSTURE_CROUCH: return "CROUCH";
    case SEEKER_POSTURE_DOWN: return "DOWN";
    default: return "";
    }
}
