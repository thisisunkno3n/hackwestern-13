#include "seeker_triangulate.h"
#include "seeker_assign.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#define DEG2RAD 0.017453292519943295
#define RAD2DEG 57.29577951308232
#define MIN_DIST_M 0.5 /* sigma floor: a contact right at the node still has some error */

void seeker_tri_defaults(seeker_tri_opts *o)
{
    o->bearing_sigma_deg = 3.0f;
    o->range_sigma_frac = 0.20f;
    o->range_sigma_k = 0.05f;
    o->min_cross_deg = 15.0f;
    o->min_cross_bearing_only_deg = 5.0f;
    o->max_range_m = 20.0f;
    o->range_gate_frac = 0.5f;
    o->assoc_m = 0.8f;
    o->prior_sigma_m = 0.5f;
    o->prior_cap = 9.0f;
}

const char *seeker_tri_method_name(seeker_tri_method m)
{
    switch (m) {
    case SEEKER_TRI_SINGLE: return "single";
    case SEEKER_TRI_PARALLEL: return "parallel";
    case SEEKER_TRI_INTERSECT: return "intersect";
    default: return "none";
    }
}

static double conf_of(const seeker_ray *r)
{
    double c = r->conf > 0 ? r->conf : 0.5;
    return c < 0.1 ? 0.1 : c > 1.0 ? 1.0 : c;
}

static void unit(const seeker_ray *r, double *ux, double *uy)
{
    double a = r->bearing_deg * DEG2RAD;
    *ux = sin(a);
    *uy = cos(a);
}

static double sigma_cross(const seeker_tri_opts *o, const seeker_ray *r, double dist)
{
    return fmax(dist, MIN_DIST_M) * (o->bearing_sigma_deg * DEG2RAD) / conf_of(r);
}

static double range_sigma(const seeker_tri_opts *o, double range, double conf)
{
    double d = fmax(range, MIN_DIST_M);
    double lin = o->range_sigma_frac * d, quad = o->range_sigma_k * d * d;
    return fmax(sqrt(lin * lin + quad * quad), 0.01) / conf;
}

static double sigma_range(const seeker_tri_opts *o, const seeker_ray *r)
{
    return range_sigma(o, r->range_m, conf_of(r));
}

float seeker_tri_range_sigma(const seeker_tri_opts *o, float range_m, float conf)
{
    seeker_ray r = { 0 };
    r.conf = conf;
    return (float)range_sigma(o, range_m, conf_of(&r));
}

/* 2x2 normal equations for sum w * (v . P - target)^2. */
typedef struct {
    double a, b, c, rx, ry;
} normal_eq;

static void add_term(normal_eq *m, double vx, double vy, double w, double target)
{
    m->a += w * vx * vx;
    m->b += w * vx * vy;
    m->c += w * vy * vy;
    m->rx += w * vx * target;
    m->ry += w * vy * target;
}

static int solve_eq(const normal_eq *m, double *x, double *y)
{
    double det = m->a * m->c - m->b * m->b;
    if (!(det > 1e-9 * m->a * m->c) || !(det > 1e-12))
        return -1;
    *x = (m->c * m->rx - m->b * m->ry) / det;
    *y = (m->a * m->ry - m->b * m->rx) / det;
    return 0;
}

/* Distance along ray i to P (negative behind the node) and signed cross-ray offset. */
static void offsets(const seeker_ray *r, double x, double y, double *along, double *cross)
{
    double ux, uy;
    unit(r, &ux, &uy);
    double dx = x - r->x_m, dy = y - r->y_m;
    *along = dx * ux + dy * uy;
    *cross = dx * uy - dy * ux;
}

static double cost_at(const seeker_ray *r, int n, const seeker_tri_opts *o, double x, double y)
{
    double cost = 0;
    for (int i = 0; i < n; i++) {
        double along, cross;
        offsets(&r[i], x, y, &along, &cross);
        double sc = sigma_cross(o, &r[i], along);
        cost += (cross / sc) * (cross / sc);
        if (r[i].has_range) {
            double e = (along - r[i].range_m) / sigma_range(o, &r[i]);
            cost += e * e;
        }
    }
    return cost;
}

static seeker_tri_method finish(seeker_fix *out, seeker_tri_method m, const seeker_ray *r, int n,
                                const seeker_tri_opts *o, double x, double y, double sigma)
{
    out->method = m;
    out->x_m = (float)x;
    out->y_m = (float)y;
    out->cost = (float)cost_at(r, n, o, x, y);
    out->sigma_m = (float)sigma;
    return m;
}

seeker_tri_method seeker_tri_solve(const seeker_ray *r, int n, const seeker_tri_opts *o,
                                   seeker_fix *out)
{
    memset(out, 0, sizeof *out);
    if (n <= 0)
        return SEEKER_TRI_NONE;

    double best_sin = 0;
    for (int i = 0; i < n; i++)
        for (int j = i + 1; j < n; j++) {
            double s = fabs(sin((r[i].bearing_deg - r[j].bearing_deg) * DEG2RAD));
            if (s > best_sin)
                best_sin = s;
        }
    out->cross_deg = (float)(asin(fmin(best_sin, 1.0)) * RAD2DEG);
    int any_range = 0;
    for (int i = 0; i < n; i++)
        any_range |= r[i].has_range;
    double min_cross = any_range ? o->min_cross_deg : o->min_cross_bearing_only_deg;

    if (n >= 2 && best_sin >= sin(min_cross * DEG2RAD)) {
        /* Unweighted line intersection first, for each ray's distance to the person. */
        normal_eq m0 = { 0 };
        for (int i = 0; i < n; i++) {
            double ux, uy;
            unit(&r[i], &ux, &uy);
            add_term(&m0, uy, -ux, 1.0, uy * r[i].x_m - ux * r[i].y_m);
        }
        double x0, y0;
        int ahead = solve_eq(&m0, &x0, &y0) == 0;
        for (int i = 0; ahead && i < n; i++) {
            double along, cross;
            offsets(&r[i], x0, y0, &along, &cross);
            ahead = along > 0;
        }
        if (ahead) {
            normal_eq m = { 0 };
            for (int i = 0; i < n; i++) {
                double ux, uy, along, cross;
                unit(&r[i], &ux, &uy);
                offsets(&r[i], x0, y0, &along, &cross);
                double sc = sigma_cross(o, &r[i], along);
                add_term(&m, uy, -ux, 1.0 / (sc * sc), uy * r[i].x_m - ux * r[i].y_m);
                if (r[i].has_range) {
                    double sr = sigma_range(o, &r[i]);
                    add_term(&m, ux, uy, 1.0 / (sr * sr),
                             ux * r[i].x_m + uy * r[i].y_m + r[i].range_m);
                }
            }
            double x, y;
            if (solve_eq(&m, &x, &y) == 0)
                return finish(out, SEEKER_TRI_INTERSECT, r, n, o, x, y,
                              sqrt((m.a + m.c) / (m.a * m.c - m.b * m.b)));
        }
    }

    /* Parallel, diverging or single: weighted mean of the range points. */
    double sw = 0, sx = 0, sy = 0;
    for (int i = 0; i < n; i++) {
        if (!r[i].has_range)
            continue;
        double ux, uy, sr = sigma_range(o, &r[i]);
        unit(&r[i], &ux, &uy);
        double w = 1.0 / (sr * sr);
        sw += w;
        sx += w * (r[i].x_m + r[i].range_m * ux);
        sy += w * (r[i].y_m + r[i].range_m * uy);
    }
    if (sw <= 0)
        return SEEKER_TRI_NONE;
    return finish(out, n == 1 ? SEEKER_TRI_SINGLE : SEEKER_TRI_PARALLEL, r, n, o, sx / sw,
                  sy / sw, sqrt(1.0 / sw));
}

int seeker_tri_consistent(const seeker_ray *r, int n, const seeker_tri_opts *o,
                          const seeker_fix *fx)
{
    if (fx->method == SEEKER_TRI_NONE)
        return 0;
    for (int i = 0; i < n; i++) {
        double along, cross;
        offsets(&r[i], fx->x_m, fx->y_m, &along, &cross);
        if (along < 0.1 || hypot(fx->x_m - r[i].x_m, fx->y_m - r[i].y_m) > o->max_range_m)
            return 0;
        if (fabs(cross) > fmax(o->assoc_m, 3.0 * sigma_cross(o, &r[i], along)))
            return 0;
        if (r[i].has_range &&
            fabs(along - r[i].range_m) > fmax(o->assoc_m, o->range_gate_frac * r[i].range_m))
            return 0;
    }
    return 1;
}

static void solve_members(const seeker_ray *r, const int *mem, int k, const seeker_tri_opts *o,
                          seeker_fix *fx, int *ok)
{
    seeker_ray sub[SEEKER_TRI_MAX_MEMBERS];
    for (int i = 0; i < k; i++)
        sub[i] = r[mem[i]];
    seeker_tri_solve(sub, k, o, fx);
    if (ok)
        *ok = seeker_tri_consistent(sub, k, o, fx);
}

static float prior_cost(const seeker_tri_opts *o, const seeker_tri_prior *pr, int npr,
                        const seeker_fix *fx)
{
    if (npr <= 0 || fx->method == SEEKER_TRI_NONE || !(o->prior_sigma_m > 0))
        return 0;
    double best = o->prior_cap, s2 = (double)o->prior_sigma_m * o->prior_sigma_m;
    for (int k = 0; k < npr; k++) {
        double dx = fx->x_m - pr[k].x_m, dy = fx->y_m - pr[k].y_m;
        double d2 = (dx * dx + dy * dy) / s2;
        if (d2 < best)
            best = d2;
    }
    return (float)best;
}

static void add_member(seeker_tri_cluster *c, int i)
{
    int at = c->n++;
    while (at > 0 && c->member[at - 1] > i) {
        c->member[at] = c->member[at - 1];
        at--;
    }
    c->member[at] = i;
}

int seeker_tri_group_prior(const seeker_ray *r, int n, const seeker_tri_opts *o,
                           const seeker_tri_prior *pr, int npr, seeker_tri_cluster *out,
                           int max_out)
{
    int nf = n < SEEKER_TRI_MAX_RAYS ? n : SEEKER_TRI_MAX_RAYS;
    static seeker_tri_cluster work[SEEKER_TRI_MAX_RAYS];
    static double cost[SEEKER_TRI_MAX_RAYS * SEEKER_TRI_MAX_RAYS];
    static unsigned char allowed[SEEKER_TRI_MAX_RAYS * SEEKER_TRI_MAX_RAYS];
    unsigned char done[SEEKER_TRI_MAX_RAYS] = { 0 };
    int nc = 0;

    /* One node at a time: its rays are assigned to the clusters built so far (each cluster takes
     * at most one ray, so two contacts of one node are never fused), cost = chi-square increase
     * (cross-ray residual + range disagreement) + distance to the nearest track prior. Rays left
     * over start their own cluster. */
    for (int s = 0; s < nf; s++) {
        if (done[s])
            continue;
        int cols[SEEKER_TRI_MAX_RAYS], m = 0;
        for (int i = s; i < nf; i++)
            if (!done[i] && r[i].node == r[s].node) {
                cols[m++] = i;
                done[i] = 1;
            }
        int rows = nc, row_col[SEEKER_TRI_MAX_RAYS];
        for (int k = 0; k < rows; k++)
            for (int j = 0; j < m; j++) {
                seeker_tri_cluster *c = &work[k];
                int at = k * m + j, ok = 0;
                allowed[at] = 0;
                cost[at] = 0;
                if (c->n >= SEEKER_TRI_MAX_MEMBERS)
                    continue;
                int mem[SEEKER_TRI_MAX_MEMBERS];
                memcpy(mem, c->member, sizeof(int) * (size_t)c->n);
                mem[c->n] = cols[j];
                seeker_fix fx;
                solve_members(r, mem, c->n + 1, o, &fx, &ok);
                allowed[at] = (unsigned char)ok;
                cost[at] = fx.cost - c->fix.cost + prior_cost(o, pr, npr, &fx);
            }
        unsigned char taken[SEEKER_TRI_MAX_RAYS] = { 0 };
        if (rows > 0 && seeker_assign(cost, allowed, rows, m, row_col) > 0)
            for (int k = 0; k < rows; k++)
                if (row_col[k] >= 0) {
                    add_member(&work[k], cols[row_col[k]]);
                    solve_members(r, work[k].member, work[k].n, o, &work[k].fix, NULL);
                    taken[row_col[k]] = 1;
                }
        for (int j = 0; j < m; j++)
            if (!taken[j]) {
                seeker_tri_cluster *c = &work[nc++];
                c->n = 1;
                c->member[0] = cols[j];
                seeker_tri_solve(&r[cols[j]], 1, o, &c->fix);
            }
    }
    int no = 0;
    for (int k = 0; k < nc && no < max_out; k++)
        out[no++] = work[k];
    for (int i = nf; i < n && no < max_out; i++) {
        seeker_tri_cluster *c = &out[no++];
        c->n = 1;
        c->member[0] = i;
        seeker_tri_solve(&r[i], 1, o, &c->fix);
    }
    for (int a = 1; a < no; a++) {
        seeker_tri_cluster tmp = out[a];
        int b = a;
        while (b > 0 && out[b - 1].member[0] > tmp.member[0]) {
            out[b] = out[b - 1];
            b--;
        }
        out[b] = tmp;
    }
    return no;
}

int seeker_tri_group(const seeker_ray *r, int n, const seeker_tri_opts *o,
                     seeker_tri_cluster *out, int max_out)
{
    return seeker_tri_group_prior(r, n, o, NULL, 0, out, max_out);
}
