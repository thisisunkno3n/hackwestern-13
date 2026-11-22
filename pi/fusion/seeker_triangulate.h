#ifndef SEEKER_TRIANGULATE_H
#define SEEKER_TRIANGULATE_H

/*
 * Multi-phone triangulation in the IF-1 room frame (+y forward, +x right, bearings clockwise from
 * +y). Each contact is a ray from its node's pose. Rays seen by different nodes that agree on one
 * point are fused into one fix (checkpoint 5: two phones, one marker).
 *
 * The fix is a weighted least-squares point, which is the inverse-variance blend of the bearing
 * intersection and the size-based (mono) ranges: every ray adds a cross-ray term (bearing error,
 * sigma grows with distance) and, when range_m is known, an along-ray term. The mono range sigma is
 * sqrt((range_sigma_frac * r)^2 + (range_sigma_k * r^2)^2): the linear part is the person-height
 * guess, the r^2 part is box-height pixel jitter (r = f * H / h, so dr = r^2 / (f * H) * dh). Both
 * sigmas are divided by the contact's confidence. With two well-crossed bearings the intersection
 * dominates and the ranges barely move it. Fallbacks:
 *   - bearings closer than min_cross_deg to parallel, or diverging (no crossing in front of both
 *     nodes): confidence-weighted mean of the range points, so near-parallel rays use the size
 *     ranges and phones facing each other across a person still fuse. Bearing-only rays have no
 *     range point, so they still intersect down to min_cross_bearing_only_deg;
 *   - one node: its own range point; a lone bearing-only contact has no fix.
 */

#define SEEKER_TRI_MAX_RAYS 64   /* rays past this are never fused (each stays its own cluster) */
#define SEEKER_TRI_MAX_MEMBERS 4 /* nodes fused into one cluster */

typedef struct {
    float x_m, y_m;     /* node position, room frame */
    float bearing_deg;  /* room-frame bearing (node yaw + contact bearing) */
    int has_range;
    float range_m;
    float conf;         /* detector confidence 0..1; 0 means unknown (treated as 0.5) */
    int node;           /* caller's node index: rays from the same node are never fused */
    int partial;        /* part of a body (a hand or shoulder round a wall edge): its size range
                         * is the visible part's, so it is ignored when another ray in the set
                         * has a whole-body range, and its bearing counts partial_bearing_k
                         * times less sure */
} seeker_ray;

typedef struct {
    float bearing_sigma_deg; /* per-ray bearing error, 1 sigma */
    float range_sigma_frac;  /* mono range error, linear part: fraction of range_m, 1 sigma */
    float range_sigma_k;     /* mono range error, quadratic part: k * range_m^2 (1/m), 1 sigma */
    float min_cross_deg;     /* bearings closer than this to parallel use the range fallback */
    float min_cross_bearing_only_deg; /* same, when no ray in the cluster has a range */
    float max_range_m;       /* a fix farther than this from any of its nodes is rejected */
    float range_gate_frac;   /* fix may sit this fraction of range_m off a ray's range */
    float assoc_m;           /* floor for the range and cross-ray gates (remote.assoc_m) */
    float prior_sigma_m;     /* grouping: a fix this far from a track prior costs 1 (chi-square) */
    float prior_cap;         /* grouping: prior cost cap, so a new person is not shut out */
    float partial_bearing_k; /* bearing sigma factor for a partial ray (its hip midpoint is a guess) */
} seeker_tri_opts;

/* Predicted position of an existing track, to break ties between pairings (ghost crossings). */
typedef struct {
    float x_m, y_m;
} seeker_tri_prior;

typedef enum {
    SEEKER_TRI_NONE = 0,  /* no position (lone bearing-only contact, or nothing to solve) */
    SEEKER_TRI_SINGLE,    /* one node's range point */
    SEEKER_TRI_PARALLEL,  /* range-weighted fallback for parallel or diverging bearings */
    SEEKER_TRI_INTERSECT, /* weighted bearing intersection */
} seeker_tri_method;

typedef struct {
    seeker_tri_method method;
    float x_m, y_m;
    float cost;      /* weighted squared residual (chi-square); lower is a better match */
    float cross_deg; /* widest angle between the fused bearings, 0..90 */
    float sigma_m;   /* 1-sigma position error from the weights (sqrt of the covariance trace) */
} seeker_fix;

typedef struct {
    int n;
    int member[SEEKER_TRI_MAX_MEMBERS]; /* ray indices, ascending */
    seeker_fix fix;
} seeker_tri_cluster;

void seeker_tri_defaults(seeker_tri_opts *o);
/* 1-sigma mono (size-based) range error the solver assumes for a ray with this range and conf. */
float seeker_tri_range_sigma(const seeker_tri_opts *o, float range_m, float conf);
/* Solves one point from n rays (n >= 1). Returns out->method. */
seeker_tri_method seeker_tri_solve(const seeker_ray *r, int n, const seeker_tri_opts *o,
                                   seeker_fix *out);
/* 1 if fx lies in front of every ray, within max_range_m, and inside each ray's cross-ray and
 * range gates (a partial ray's range gate only when no whole-body ray in the set has a range). */
int seeker_tri_consistent(const seeker_ray *r, int n, const seeker_tri_opts *o,
                          const seeker_fix *fx);
/* Groups n rays into clusters (each ray in exactly one, at most one ray per node per cluster).
 * Nodes are taken in order of first appearance; each node's rays are assigned to the clusters so
 * far by gated minimum-cost assignment (seeker_assign.h), not greedy nearest, so in the 2x2 case
 * (two people, two phones) the pairing with the lower total cost wins over the two ghost
 * crossings. Cost = chi-square increase (cross-ray residual + range disagreement) + the distance
 * to the nearest prior (prior_sigma_m, capped at prior_cap). Clusters come out in order of their
 * lowest ray index. Returns the cluster count (<= n and <= max_out). Not reentrant. */
int seeker_tri_group_prior(const seeker_ray *r, int n, const seeker_tri_opts *o,
                           const seeker_tri_prior *pr, int npr, seeker_tri_cluster *out,
                           int max_out);
/* seeker_tri_group_prior with no priors. */
int seeker_tri_group(const seeker_ray *r, int n, const seeker_tri_opts *o,
                     seeker_tri_cluster *out, int max_out);
const char *seeker_tri_method_name(seeker_tri_method m);

#endif
