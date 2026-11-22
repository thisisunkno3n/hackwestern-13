#include "seeker_fusion.h"
#include "seeker_assign.h"
#include "seeker_log.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f
/* A moving wearer back from a hold, or going home, eases instead of jumping. */
#define POSE_BLEND_TAU_MS 150.0f
#define POSE_BLEND_MAX_MS 800
#define POSE_BLEND_MIN_M 0.10f   /* smaller differences are taken at once */
#define POSE_BLEND_MIN_DEG 3.0f

float seeker_wrap180(float deg)
{
    while (deg > 180.0f)
        deg -= 360.0f;
    while (deg <= -180.0f)
        deg += 360.0f;
    return deg;
}

void seeker_node_to_room(float nx, float ny, float nyaw, float bearing, float range, float *x,
                         float *y)
{
    seeker_pose p = { nx, ny, nyaw };
    seeker_pose_to_world(&p, bearing, range, x, y);
}

void seeker_fusion_init(seeker_fusion *f, const seeker_config *cfg)
{
    memset(f, 0, sizeof *f);
    f->cfg = cfg;
    f->next_id = 1;
    f->pose_gate = 1;
    f->skel_smooth = 1;
    f->pose_blend = 1;
    f->pose_smooth = 1;
    f->pose_flip = 1;
    f->range_hfov = 1;
    f->handoff = 1;
    f->crouch_range = 1;
    f->occl_guard = 1;
    seeker_tri_defaults(&f->tri);
    f->tri.assoc_m = cfg->assoc_m;
    f->tri.bearing_sigma_deg = cfg->tri_bearing_sigma_deg;
    f->tri.range_sigma_frac = cfg->tri_range_sigma_frac;
    f->tri.range_sigma_k = cfg->tri_range_sigma_k;
    f->tri.min_cross_deg = cfg->tri_min_cross_deg;
    f->tri.min_cross_bearing_only_deg = cfg->tri_min_cross_bearing_only_deg;
    f->tri.prior_sigma_m = cfg->tri_track_prior_sigma_m;
    f->tri.prior_cap = cfg->tri_track_prior_cap;
    for (int i = 0; i < cfg->n_nodes && i < SEEKER_MAX_NODES; i++) {
        seeker_fusion_node *n = &f->nodes[f->n_nodes++];
        snprintf(n->id, sizeof n->id, "%s", cfg->nodes[i].id);
        n->state = NODE_WAIT;
        n->x_m = cfg->nodes[i].x_m;
        n->y_m = cfg->nodes[i].y_m;
        n->yaw_deg = cfg->nodes[i].yaw_deg;
        n->hfov_deg = cfg->nodes[i].hfov_deg;
        n->image_w = cfg->nodes[i].image_w;
        n->image_h = cfg->nodes[i].image_h;
        snprintf(n->pose_src, sizeof n->pose_src, "config");
        n->last_tag = n->warned_tag = -1;
    }
}

static seeker_fusion_node *find_node(seeker_fusion *f, const char *id, int create)
{
    for (int i = 0; i < f->n_nodes; i++)
        if (strcmp(f->nodes[i].id, id) == 0)
            return &f->nodes[i];
    if (!create || f->n_nodes >= SEEKER_MAX_NODES)
        return NULL;
    seeker_fusion_node *n = &f->nodes[f->n_nodes++];
    memset(n, 0, sizeof *n);
    snprintf(n->id, sizeof n->id, "%s", id);
    n->hfov_deg = 60.0f;
    n->image_w = 640;
    n->image_h = 480;
    n->last_tag = n->warned_tag = -1;
    snprintf(n->pose_src, sizeof n->pose_src, "default");
    seeker_log("unconfigured node %s joined; add it to remote.nodes for its view cone", id);
    return n;
}

int seeker_fusion_set_pose(seeker_fusion *f, const char *id, const seeker_pose *p, const char *src,
                           int pin)
{
    seeker_fusion_node *n = find_node(f, id, 1);
    if (!n)
        return -1;
    n->x_m = p->x_m;
    n->y_m = p->y_m;
    n->yaw_deg = p->yaw_deg;
    n->pose_pinned = pin;
    n->blending = 0;
    if (!pin && !strcmp(src, "init")) { /* the wearer's usual spot: home after a long hold */
        n->has_home = 1;
        n->home = *p;
    }
    snprintf(n->pose_src, sizeof n->pose_src, "%s", src);
    return (int)(n - f->nodes);
}

int seeker_fusion_set_tag_frame(seeker_fusion *f, const char *id, const seeker_pose *tag)
{
    seeker_fusion_node *n = find_node(f, id, 1);
    if (!n)
        return -1;
    n->has_tag_frame = 1;
    n->tag_frame = *tag;
    seeker_pose in_front = { 0.0f, -2.0f, 0.0f }, start;
    seeker_pose_from_tag(tag, &in_front, &start);
    return seeker_fusion_set_pose(f, id, &start, "init", 0);
}

int seeker_hfov_preset(const char *name, float *landscape, float *portrait)
{
    static const struct {
        const char *name;
        float focal_mm; /* 35 mm equivalent */
    } lens[] = {
        { "iphone17pro", 24.0f },      /* support.apple.com/en-us/125090: Fusion Main 24 mm */
        { "iphone16", 26.0f },         /* apple.com/iphone-16/specs: Fusion Main 26 mm */
        { "iphone-ultrawide", 13.0f }, /* both: Ultra Wide 13 mm */
    };
    for (size_t i = 0; i < sizeof lens / sizeof lens[0]; i++)
        if (!strcmp(name, lens[i].name)) {
            /* 4:3 frame with the 35 mm diagonal (43.27 mm): 34.62 x 25.96 mm */
            *landscape = 2.0f * atanf(17.31f / lens[i].focal_mm) * RAD2DEG;
            *portrait = 2.0f * atanf(12.98f / lens[i].focal_mm) * RAD2DEG;
            return 0;
        }
    return -1;
}

int seeker_fusion_relative(const seeker_fusion *f, const char *from, const char *id, float *right_m,
                           float *ahead_m)
{
    const seeker_fusion_node *a = NULL, *b = NULL;
    for (int i = 0; i < f->n_nodes; i++) {
        if (!strcmp(f->nodes[i].id, from))
            a = &f->nodes[i];
        if (!strcmp(f->nodes[i].id, id))
            b = &f->nodes[i];
    }
    if (!a || !b)
        return -1;
    float yaw = a->yaw_deg * DEG2RAD, dx = b->x_m - a->x_m, dy = b->y_m - a->y_m;
    *right_m = dx * cosf(yaw) - dy * sinf(yaw);
    *ahead_m = dx * sinf(yaw) + dy * cosf(yaw);
    return 0;
}

int seeker_fusion_set_hfov(seeker_fusion *f, const char *id, const char *spec)
{
    float land, port;
    char *end;
    double v = strtod(spec, &end);
    if (end != spec && *end == '\0') {
        if (!(v > 10.0 && v < 170.0))
            return -1;
        land = port = (float)v;
    } else if (seeker_hfov_preset(spec, &land, &port) != 0) {
        return -1;
    }
    seeker_fusion_node *n = find_node(f, id, 1);
    if (!n)
        return -1;
    n->hfov_set = land;
    n->hfov_set_portrait = port;
    n->hfov_deg = land;
    return (int)(n - f->nodes);
}

/* --hfov: rescale the bearings the phone computed with its own FOV to the real one. */
static float contact_ground_range(const seeker_fusion_node *n, const seeker_contact *c);
static int contact_partial(const seeker_contact *c, int w, int h);
static int contact_frame_cut(const seeker_contact *c);

/* The phone's bearings and box-height ranges with its real FOV (--hfov), the real people's
 * height (--person-h, phone.html assumes SEEKER_SKEL_PERSON_M) and a per-phone factor
 * (--range-scale). Range from box height is focal x height / box px, so it scales by
 * tan(sent / 2) / tan(real / 2) unless the phone's range was calibrated (--no-range-hfov). */
static void apply_hfov(const seeker_fusion *f, seeker_fusion_node *n, const seeker_remote_contacts *m)
{
    float rk = (n->range_k > 0.0f ? n->range_k : 1.0f) *
               (f->person_h > 0.5f ? f->person_h / SEEKER_SKEL_PERSON_M : 1.0f);
    if (n->hfov_set > 0) {
        int portrait = n->image_w > 0 && n->image_h > n->image_w;
        float real = portrait ? n->hfov_set_portrait : n->hfov_set;
        float sent = m->hfov_deg > 1.0f ? m->hfov_deg : 66.0f; /* phone.html default */
        n->hfov_deg = real;
        if (fabsf(real - sent) >= 0.05f) {
            float k = tanf(real * 0.5f * DEG2RAD) / tanf(sent * 0.5f * DEG2RAD);
            for (int i = 0; i < n->last.n; i++)
                n->last.c[i].bearing_deg = atanf(tanf(n->last.c[i].bearing_deg * DEG2RAD) * k) * RAD2DEG;
            if (f->range_hfov)
                rk /= k;
        }
    }
    if (fabsf(rk - 1.0f) >= 1e-4f)
        for (int i = 0; i < n->last.n; i++)
            if (n->last.c[i].has_range)
                n->last.c[i].range_m *= rk;
    if (n->cam_h <= 0)
        return;
    for (int i = 0; i < n->last.n; i++) {
        seeker_contact *c = &n->last.c[i];
        float rg = contact_ground_range(n, c);
        if (rg <= 0)
            continue;
        if (m->t_mono_ms - n->ground_log_ms >= 5000) { /* the distance check, both ways */
            n->ground_log_ms = m->t_mono_ms;
            char box[16] = "none";
            if (c->has_range)
                snprintf(box, sizeof box, "%.2f m", c->range_m);
            seeker_log("NODE %s range check %s: box height %s, ground plane %.2f m (camera %.2f m, "
                       "tilt %.0f deg)%s", n->id, c->id, box, rg, n->cam_h, n->cam_pitch,
                       f->ground_range ? ", averaged" : "");
        }
        if (f->ground_range) {
            c->range_m = c->has_range ? 0.5f * (c->range_m + rg) : rg;
            c->has_range = 1;
        }
    }
}

float seeker_ground_range(float v_feet, int w, int h, float hfov_deg, float cam_h, float pitch_deg)
{
    if (w <= 0 || h <= 0 || cam_h <= 0.1f)
        return 0;
    float f = ((float)w * 0.5f) / tanf((hfov_deg > 1.0f ? hfov_deg : 66.0f) * 0.5f * DEG2RAD);
    float dep = atanf((v_feet - (float)h * 0.5f) / f) - pitch_deg * DEG2RAD;
    if (dep < 2.0f * DEG2RAD)
        return 0;
    return cam_h / tanf(dep);
}

/* Feet in view: an ankle (COCO 15, 16) clearly seen and the box bottom inside the frame. */
static float contact_ground_range(const seeker_fusion_node *n, const seeker_contact *c)
{
    if (!c->has_box || !c->has_pts || n->image_w <= 0 || n->image_h <= 0)
        return 0;
    if (c->pts[15][2] < 0.5f && c->pts[16][2] < 0.5f)
        return 0;
    float v = c->box[1] + c->box[3];
    if (v > 0.98f * (float)n->image_h)
        return 0;
    return seeker_ground_range(v, n->image_w, n->image_h, n->hfov_deg, n->cam_h, n->cam_pitch);
}

int seeker_fusion_set_cam(seeker_fusion *f, const char *id, float cam_h, float pitch_deg)
{
    seeker_fusion_node *n = find_node(f, id, 1);
    if (!n || !(cam_h > 0.1f && cam_h < 5.0f) || !(pitch_deg > -60.0f && pitch_deg < 60.0f))
        return -1;
    n->cam_h = cam_h;
    n->cam_pitch = pitch_deg;
    return 0;
}

int seeker_fusion_set_range_scale(seeker_fusion *f, const char *id, float k)
{
    seeker_fusion_node *n = find_node(f, id, 1);
    if (!n || !(k > 0.2f && k < 5.0f))
        return -1;
    n->range_k = k;
    return 0;
}

static const seeker_pose *find_tag(const seeker_fusion *f, int tag_id)
{
    for (int i = 0; i < f->n_tags; i++)
        if (f->tags[i].id == tag_id)
            return &f->tags[i].pose;
    return NULL;
}

int seeker_fusion_set_tag(seeker_fusion *f, int tag_id, const seeker_pose *room)
{
    if (tag_id <= 0)
        return -1;
    seeker_pose *p = (seeker_pose *)find_tag(f, tag_id);
    if (!p) {
        if (f->n_tags >= SEEKER_MAX_TAGS)
            return -1;
        f->tags[f->n_tags].id = tag_id;
        p = &f->tags[f->n_tags++].pose;
    }
    *p = *room;
    return 0;
}

static int pose_jumped(float x0, float y0, float yaw0, float x1, float y1, float yaw1)
{
    return hypotf(x1 - x0, y1 - y0) > SEEKER_POSE_JUMP_M ||
           fabsf(seeker_wrap180(yaw1 - yaw0)) > SEEKER_POSE_JUMP_DEG;
}

/* The other solution of a flat tag's pose, in the tag's own frame (SEEKER_POSE_FLIP_M). */
static seeker_pose pose_mirror(const seeker_pose *p)
{
    float b = atan2f(-p->x_m, -p->y_m) * RAD2DEG; /* camera to tag, clockwise from +y */
    seeker_pose m = { -p->x_m, p->y_m, seeker_wrap180(p->yaw_deg - 2.0f * b) };
    return m;
}

/* 1 when b is more than SEEKER_POSE_FLIP_M or SEEKER_POSE_FLIP_DEG from a. */
static int pose_near_flip(const seeker_pose *a, const seeker_pose *b)
{
    return hypotf(a->x_m - b->x_m, a->y_m - b->y_m) > SEEKER_POSE_FLIP_M ||
           fabsf(seeker_wrap180(a->yaw_deg - b->yaw_deg)) > SEEKER_POSE_FLIP_DEG;
}

static int count_bits(unsigned v)
{
    int n = 0;
    for (; v; v &= v - 1)
        n++;
    return n;
}

/* No live pose: "held" (position and yaw kept), "heading" (yaw from the compass), "home" (eased
 * to the --init-pose after pose_home_ms). */
static int pose_is_held(const seeker_fusion_node *n)
{
    return !strcmp(n->pose_src, "held") || !strcmp(n->pose_src, "heading") ||
           !strcmp(n->pose_src, "home");
}

static void start_blend(seeker_fusion_node *n, float x, float y, float yaw, int with_yaw,
                        int64_t now)
{
    n->target = (seeker_pose){ x, y, yaw };
    n->blend_yaw = with_yaw;
    if (!n->blending)
        n->blend_start_ms = n->blend_ms = now;
    n->blending = 1;
}

static float pose_oe_alpha(float cutoff_hz, float dt_s)
{
    float tau = 1.0f / (6.2831853f * cutoff_hz);
    return 1.0f / (1.0f + tau / dt_s);
}

static void take_message_pose(seeker_fusion *f, seeker_fusion_node *n,
                              const seeker_remote_contacts *m, int64_t now)
{
    if (n->pose_pinned || (f->cfg->pose_from_config && !n->has_tag_frame))
        return;
    int take_xy = m->has_xy || m->has_pose;
    int take_yaw = m->has_yaw || m->has_pose;
    seeker_pose msg = { m->x_m, m->y_m, m->yaw_deg };
    const seeker_pose local_msg = msg; /* as the phone sent it: in its tag's frame */
    int msg_zero = msg.x_m == 0 && msg.y_m == 0 && msg.yaw_deg == 0;
    const seeker_pose *frame = NULL; /* NULL: the pose is in the room frame */
    int tag = m->has_tag_id ? m->tag_id : -1;
    if (tag > 0) {
        frame = find_tag(f, tag);
        if (!frame && (take_xy || take_yaw)) {
            if (n->warned_tag != tag)
                seeker_log("NODE %s sends its pose from tag %d, which has no room pose "
                           "(hud_fusion --tag-pose %d=x,y,yaw): taken as pose null", n->id, tag,
                           tag);
            n->warned_tag = tag;
            take_xy = take_yaw = 0;
        }
    } else if (tag < 0 && n->has_tag_frame) {
        frame = &n->tag_frame;
    }
    if (frame) {
        seeker_pose local = msg;
        seeker_pose_from_tag(frame, &local, &msg);
    }
    if (!take_xy && !take_yaw) {
        if (!strcmp(n->pose_src, "message")) {
            if (n->blending) { /* lost the tag while blending back in: hold where the blend is */
                n->blending = 0;
            }
            snprintf(n->pose_src, sizeof n->pose_src, "held");
            n->held_ms = now;
            seeker_log("NODE %s pose null (tag out of view?): holding its last pose %.2f,%.2f,%.1f%s",
                       n->id, n->x_m, n->y_m, n->yaw_deg,
                       n->has_head_off && m->has_heading ? ", yaw from the compass" : "");
        }
        if (pose_is_held(n) && m->has_heading && n->has_head_off) {
            /* The wearer keeps turning: follow the compass, offset learned from the tags. */
            float yaw = seeker_wrap180(m->heading_deg + n->head_off);
            if (!n->blending || !n->blend_yaw) /* light smoothing: compasses are noisy */
                n->yaw_deg = seeker_wrap180(n->yaw_deg + 0.35f * seeker_wrap180(yaw - n->yaw_deg));
            if (!strcmp(n->pose_src, "held"))
                snprintf(n->pose_src, sizeof n->pose_src, "heading");
        }
        n->pose_jumps = 0;
        return;
    }
    float nx = take_xy ? msg.x_m : n->x_m;
    float ny = take_xy ? msg.y_m : n->y_m;
    float nyaw = take_yaw ? msg.yaw_deg : n->yaw_deg;
    if (f->pose_gate && take_xy && take_yaw && pose_is_held(n) && now - n->held_ms >= SEEKER_POSE_REACQ_MS) {
        seeker_pose cand = { nx, ny, nyaw }, mir = cand, ml = pose_mirror(&local_msg);
        mir = ml;
        if (frame)
            seeker_pose_from_tag(frame, &ml, &mir);
        int agree = 0;
        if (n->reacq_have) {
            const seeker_pose *q = &n->reacq;
            if (!pose_near_flip(q, &cand))
                agree = 1;
            else if (!pose_near_flip(q, &mir)) {
                agree = 1;
                cand = mir; /* the second came as the first's mirror: keep the first's solution */
            }
        }
        if (!agree && !(n->reacq_have && now - n->reacq_ms >= SEEKER_POSE_REACQ_WAIT_MS)) {
            if (!n->reacq_have)
                seeker_log("NODE %s tag back after %.1f s without it: %.2f,%.2f,%.1f, waiting for a "
                           "second pose that agrees", n->id, (double)(now - n->held_ms) / 1000.0,
                           nx, ny, nyaw);
            n->reacq = cand;
            n->reacq_ms = n->reacq_have ? n->reacq_ms : now;
            n->reacq_have = 1;
            n->pose_jumps = 0;
            return;
        }
        n->reacq_have = 0;
        n->reacqs++;
        n->oe_valid = 0; /* filters and flip history start over from the pose taken back */
        n->flip_hist = 0;
        n->pose_jumps = 0;
        nx = cand.x_m;
        ny = cand.y_m;
        nyaw = cand.yaw_deg;
        seeker_log("NODE %s pose back after %.1f s: %.2f,%.2f,%.1f (%s)", n->id,
                   (double)(now - n->held_ms) / 1000.0, nx, ny, nyaw,
                   agree ? "two poses agreed" : "no agreement in time, newest taken");
    } else if (take_xy && take_yaw) {
        n->reacq_have = 0;
    }
    /* The jump gate compares with where the pose is going (a blend's target), not the eased
     * pose on screen. */
    const seeker_pose cur = n->blending ? n->target : (seeker_pose){ n->x_m, n->y_m, n->yaw_deg };
    int flip_switch = 0;
    if (f->pose_gate && f->pose_flip && take_xy && take_yaw && !strcmp(n->pose_src, "message") &&
        tag == n->last_tag) {
        int flipped = 0;
        seeker_pose mir = { 0, 0, 0 };
        if (pose_jumped(cur.x_m, cur.y_m, cur.yaw_deg, nx, ny, nyaw)) {
            seeker_pose ml = pose_mirror(&local_msg);
            mir = ml;
            if (frame)
                seeker_pose_from_tag(frame, &ml, &mir);
            flipped = hypotf(mir.x_m - cur.x_m, mir.y_m - cur.y_m) <= SEEKER_POSE_FLIP_M &&
                      fabsf(seeker_wrap180(mir.yaw_deg - cur.yaw_deg)) <= SEEKER_POSE_FLIP_DEG;
        }
        n->flip_hist = ((n->flip_hist << 1) | (unsigned)flipped) & ((1u << SEEKER_POSE_FLIP_WINDOW) - 1u);
        if (flipped && count_bits(n->flip_hist) >= SEEKER_POSE_FLIP_SWITCH) {
            seeker_log("NODE %s tag pose: the other solution won %d of the last %d frames, switching to "
                       "it: %.2f,%.2f,%.1f (was %.2f,%.2f,%.1f)", n->id, count_bits(n->flip_hist),
                       SEEKER_POSE_FLIP_WINDOW, nx, ny, nyaw, cur.x_m, cur.y_m, cur.yaw_deg);
            n->flip_hist = 0;
            flip_switch = 1;
        } else if (flipped) {
            if (n->flips % 50 == 0)
                seeker_log("NODE %s tag pose flipped to its mirror %.2f,%.2f,%.1f (planar tag "
                           "ambiguity): kept %.2f,%.2f,%.1f (%ld flips so far)", n->id, nx, ny,
                           nyaw, mir.x_m, mir.y_m, mir.yaw_deg, n->flips + 1);
            n->flips++;
            nx = mir.x_m;
            ny = mir.y_m;
            nyaw = mir.yaw_deg;
        }
    }
    if (f->pose_gate && !flip_switch && take_xy && take_yaw && !strcmp(n->pose_src, "message") &&
        pose_jumped(cur.x_m, cur.y_m, cur.yaw_deg, nx, ny, nyaw)) {
        if (n->pose_jumps > 0 && pose_jumped(n->jump_pose.x_m, n->jump_pose.y_m,
                                             n->jump_pose.yaw_deg, nx, ny, nyaw))
            n->pose_jumps = 0; /* not the same new place as the last jump: start over */
        n->jump_pose = (seeker_pose){ nx, ny, nyaw };
        if (++n->pose_jumps < SEEKER_POSE_JUMP_N) {
            float jump_m = hypotf(nx - n->x_m, ny - n->y_m);
            float jump_deg = fabsf(seeker_wrap180(nyaw - n->yaw_deg));
            if (f->debug || (tag != n->last_tag && n->pose_jumps == 1))
                seeker_log("NODE %s pose jump %.2f m %.0f deg to %.2f,%.2f,%.1f%s ignored (%d of %d)",
                           n->id, jump_m, jump_deg, nx, ny, nyaw,
                           tag != n->last_tag ? " (tag switch)" : "", n->pose_jumps,
                           SEEKER_POSE_JUMP_N);
            return;
        }
    }
    n->pose_jumps = 0;
    if (take_xy && take_yaw && msg_zero && !n->warned_zero_pose &&
        strcmp(n->pose_src, "message") != 0 && (n->x_m != 0 || n->y_m != 0 || n->yaw_deg != 0)) {
        n->warned_zero_pose = 1;
        seeker_log("NODE %s sends pose 0,0,0 (old relay default?), replacing %s pose %.2f,%.2f,%.1f;"
                   " pin it with --pose %s=x,y,yaw or set remote.pose_source \"config\"",
                   n->id, n->pose_src, n->x_m, n->y_m, n->yaw_deg, n->id);
    }
    const char *nsrc = (take_xy && take_yaw) ? "message" : "partial";
    if (fabsf(nx - n->x_m) > 0.1f || fabsf(ny - n->y_m) > 0.1f ||
        fabsf(seeker_wrap180(nyaw - n->yaw_deg)) > 2.0f || strcmp(n->pose_src, nsrc) ||
        tag != n->last_tag) {
        char via[64] = "";
        if (tag == 0)
            snprintf(via, sizeof via, " (tag 0)");
        else if (frame && tag > 0)
            snprintf(via, sizeof via, " (tag %d frame %.2f,%.2f,%.1f)", tag, m->x_m, m->y_m,
                     m->yaw_deg);
        else if (frame)
            snprintf(via, sizeof via, " (tag frame %.2f,%.2f,%.1f)", m->x_m, m->y_m, m->yaw_deg);
        seeker_log("NODE %s pose from message%s: %.2f,%.2f,%.1f%s%s", n->id,
                   (take_xy && take_yaw) ? "" : " (partial)", nx, ny, nyaw, via,
                   pose_is_held(n) ? " (live again after hold, blending in)" : "");
    }
    if (take_xy && take_yaw && m->has_heading) { /* learn compass -> room yaw while tags show */
        float off = seeker_wrap180(nyaw - m->heading_deg);
        n->head_off = n->has_head_off ? seeker_wrap180(n->head_off + 0.2f * seeker_wrap180(off - n->head_off))
                                      : off;
        n->has_head_off = 1;
    }
    /* Back from a hold (or home): ease in instead of jumping; while easing, new poses only move
     * the target. A small difference is taken at once. */
    if (f->pose_blend && (pose_is_held(n) || n->blending) &&
        (n->blending || hypotf(nx - n->x_m, ny - n->y_m) > POSE_BLEND_MIN_M ||
         fabsf(seeker_wrap180(nyaw - n->yaw_deg)) > POSE_BLEND_MIN_DEG)) {
        start_blend(n, nx, ny, nyaw, 1, now);
    } else if (f->pose_smooth && take_xy && take_yaw && !strcmp(n->pose_src, "message") &&
               tag == n->last_tag && n->oe_valid && now > n->oe_ms) {
        float dt = (float)(now - n->oe_ms) / 1000.0f;
        if (dt > 0.5f)
            dt = 0.5f;
        float ex = nx - n->x_m, ey = ny - n->y_m, eyaw = seeker_wrap180(nyaw - n->yaw_deg);
        float ad = pose_oe_alpha(SEEKER_POSE_D_CUTOFF_HZ, dt);
        n->oe_dx += ad * (ex / dt - n->oe_dx);
        n->oe_dy += ad * (ey / dt - n->oe_dy);
        n->oe_dyaw += ad * (eyaw / dt - n->oe_dyaw);
        float ap = pose_oe_alpha(SEEKER_POSE_MIN_CUTOFF_HZ +
                                     SEEKER_POSE_BETA_M * hypotf(n->oe_dx, n->oe_dy), dt);
        float ay = pose_oe_alpha(SEEKER_POSE_MIN_CUTOFF_HZ + SEEKER_POSE_BETA_DEG * fabsf(n->oe_dyaw),
                                 dt);
        n->x_m += ap * ex;
        n->y_m += ap * ey;
        n->yaw_deg = seeker_wrap180(n->yaw_deg + ay * eyaw);
        n->oe_ms = now;
    } else {
        n->x_m = nx;
        n->y_m = ny;
        n->yaw_deg = nyaw;
        n->oe_valid = take_xy && take_yaw;
        n->oe_dx = n->oe_dy = n->oe_dyaw = 0;
        n->oe_ms = now;
    }
    n->last_tag = tag;
    snprintf(n->pose_src, sizeof n->pose_src, "%s", nsrc);
}

/* Once per tick: ease a blending pose toward its target (exponential, POSE_BLEND_TAU_MS, done
 * when close or after POSE_BLEND_MAX_MS), and after pose_home_ms without a live pose ease a
 * node with a home (its --init-pose) back there. */
static void step_pose(seeker_fusion *f, seeker_fusion_node *n, int64_t now)
{
    const seeker_config *cfg = f->cfg;
    if (n->pose_pinned)
        return;
    if (pose_is_held(n) && strcmp(n->pose_src, "home") != 0 && n->has_home &&
        cfg->pose_home_ms > 0 && now - n->held_ms > cfg->pose_home_ms) {
        int compass = !strcmp(n->pose_src, "heading");
        start_blend(n, n->home.x_m, n->home.y_m, n->home.yaw_deg, !compass, now);
        snprintf(n->pose_src, sizeof n->pose_src, "home");
        seeker_log("NODE %s no tag for %d s: easing to its home pose %.2f,%.2f%s", n->id,
                   cfg->pose_home_ms / 1000, n->home.x_m, n->home.y_m,
                   compass ? " (yaw from the compass)" : "");
    }
    if (!n->blending)
        return;
    float dt = (float)(now - n->blend_ms);
    n->blend_ms = now;
    float a = dt <= 0 ? 0 : 1.0f - expf(-dt / POSE_BLEND_TAU_MS);
    n->x_m += a * (n->target.x_m - n->x_m);
    n->y_m += a * (n->target.y_m - n->y_m);
    if (n->blend_yaw)
        n->yaw_deg = seeker_wrap180(n->yaw_deg + a * seeker_wrap180(n->target.yaw_deg - n->yaw_deg));
    float d = hypotf(n->target.x_m - n->x_m, n->target.y_m - n->y_m);
    float dy = n->blend_yaw ? fabsf(seeker_wrap180(n->target.yaw_deg - n->yaw_deg)) : 0;
    if ((d < 0.02f && dy < 0.5f) || now - n->blend_start_ms > POSE_BLEND_MAX_MS) {
        n->x_m = n->target.x_m;
        n->y_m = n->target.y_m;
        if (n->blend_yaw)
            n->yaw_deg = n->target.yaw_deg;
        n->blending = 0;
    }
}

/* SEEKER_CROUCH_EYE_ANKLE_FRAC: the factor for contact c's range (1 unless phone.html ranged it
 * from eye to ankle and its landmarks are crouched); why gets the posture features. */
static float crouch_range_k(const seeker_contact *c, int w, int h, char *why, int whysz)
{
    if (!c->has_pts || !c->has_range || w <= 0 || h <= 0)
        return 1.0f;
    const float(*p)[3] = c->pts;
    const float v = 0.5f; /* phone.html VIS_MIN */
    int top = p[0][2] > v || p[1][2] > v || p[2][2] > v;
    int ankle = p[15][2] > v || p[16][2] > v;
    if (!top || !ankle)
        return 1.0f; /* ranged from the torso or the box */
    float px[SEEKER_N_JOINTS][3];
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        px[j][0] = p[j][0] * (float)w;
        px[j][1] = p[j][1] * (float)h;
        px[j][2] = p[j][2];
    }
    seeker_posture_mask_offframe(px, p);
    seeker_posture_feat ft;
    if (!seeker_posture_features((const float(*)[3])px, &ft) ||
        seeker_posture_classify(&ft, SEEKER_POSTURE_UNKNOWN) != SEEKER_POSTURE_CROUCH)
        return 1.0f;
    seeker_posture_describe(&ft, why, whysz);
    return SEEKER_CROUCH_EYE_ANKLE_FRAC / SEEKER_STAND_EYE_ANKLE_FRAC;
}

static void apply_crouch_range(seeker_fusion_node *n, int64_t now_ms)
{
    for (int i = 0; i < n->last.n; i++) {
        seeker_contact *c = &n->last.c[i];
        char why[160] = "";
        float k = crouch_range_k(c, n->image_w, n->image_h, why, sizeof why);
        if (k == 1.0f)
            continue;
        if (!n->crouch_ranges || now_ms - n->crouch_log_ms >= 5000) {
            n->crouch_log_ms = now_ms;
            seeker_log("NODE %s crouch range %s: eye-to-ankle range %.2f m taken as crouched, "
                       "%.2f m (x%.2f; %s; %ld so far)", n->id, c->id, c->range_m, c->range_m * k,
                       k, why, n->crouch_ranges + 1);
        }
        c->range_m *= k;
        n->crouch_ranges++;
    }
}

/* Which of the node's new contacts show only part of a body (SEEKER_HANDOFF_*), with the
 * occlusion guard's hold on a whole sighting where the node just saw a part (SEEKER_OCCL_*). */
static void mark_partial(seeker_fusion *f, seeker_fusion_node *n, int64_t now_ms)
{
    for (int i = 0; i < n->last.n; i++) {
        const seeker_contact *c = &n->last.c[i];
        int part = f->handoff && contact_partial(c, n->image_w, n->image_h);
        /* SEEKER_PART_CUT: the frame cuts them, no wall does. The node sees this person. */
        int cut = part && contact_frame_cut(c);
        if (f->handoff && f->occl_guard) {
            int near = n->frag_ms && now_ms - n->frag_ms <= SEEKER_OCCL_HOLD_MS &&
                       fabsf(seeker_wrap180(c->bearing_deg - n->frag_bearing)) <= SEEKER_OCCL_BEARING_DEG;
            if (part && !cut) {
                n->frag_ms = now_ms;
                n->frag_bearing = c->bearing_deg;
                n->frag_whole = 0;
            } else if (!part && near && ++n->frag_whole < SEEKER_OCCL_WHOLE_N) {
                part = 2;
                if (n->frag_whole == 1)
                    seeker_log("NODE %s occl: whole sighting %s at %.1f deg held partial, part of someone "
                               "seen there %lld ms ago (%ld held)",
                               n->id, c->id, c->bearing_deg, (long long)(now_ms - n->frag_ms), n->frag_held + 1);
                n->frag_held++;
            }
            part |= near || part ? 4 : 0;
        }
        n->part[i] = (unsigned char)(part | (cut ? SEEKER_PART_CUT : 0));
        n->cut_n += cut;
        if ((part & 3) && (!n->part_log_ms || now_ms - n->part_log_ms >= 5000)) {
            /* Why a person in a phone's view is not one it "sees": the numbers the rules read. */
            const float(*p)[3] = c->pts;
            n->part_log_ms = now_ms;
            seeker_log("NODE %s part: %s at %.1f deg %s (head %.2f, shoulders %.2f %.2f, hips %.2f %.2f, "
                       "ankles %.2f %.2f at rows %.2f %.2f; %ld cut by the frame so far)",
                       n->id, c->id, c->bearing_deg,
                       cut ? "is cut by the frame: still seen by this phone, ranged by bearing"
                       : part & 2 ? "is whole but held as part of a body (occlusion guard)"
                                  : "shows part of a body: ranged by bearing, not one of the track's phones while another sees them whole",
                       fmaxf(p[0][2], fmaxf(p[1][2], p[2][2])), p[5][2], p[6][2], p[11][2], p[12][2],
                       p[15][2], p[16][2], p[15][1], p[16][1], n->cut_n);
        }
    }
}

int seeker_fusion_ingest(seeker_fusion *f, const seeker_remote_contacts *m, int64_t now_ms)
{
    seeker_fusion_node *n = find_node(f, m->node_id, 1);
    if (!n)
        return -1;
    if (n->state == NODE_LOST) {
        seeker_log("NODE %s resumed after %lld ms", n->id, (long long)(now_ms - n->last_rx_ms));
        char ev[96];
        snprintf(ev, sizeof ev, "\"event\":\"node_resumed\",\"node_id\":\"%s\"", n->id);
        seeker_event(ev);
    } else if (n->state == NODE_WAIT) {
        seeker_log("NODE %s first datagram", n->id);
        n->first_rx_ms = now_ms;
    }
    if (n->msgs > 0 && now_ms > n->last_rx_ms) {
        float inst = 1000.0f / (float)(now_ms - n->last_rx_ms);
        n->rate_hz = n->rate_hz <= 0 ? inst : 0.9f * n->rate_hz + 0.1f * inst;
    }
    n->state = NODE_OK;
    n->last_rx_ms = now_ms;
    n->msgs++;
    n->last = *m;
    if (m->has_pitch) {
        n->has_pitch = 1;
        n->pitch_deg = m->pitch_deg;
        n->pitch_ms = now_ms;
    }
    take_message_pose(f, n, m, now_ms);
    if (m->width > 0 && m->height > 0) {
        n->image_w = m->width;
        n->image_h = m->height;
    }
    if (m->hfov_deg > 0)
        n->hfov_deg = m->hfov_deg;
    if (f->crouch_range)
        apply_crouch_range(n, now_ms);
    apply_hfov(f, n, m);
    mark_partial(f, n, now_ms);
    return (int)(n - f->nodes);
}

/* The track that last carried this label and is not taken this tick, else a fresh one in the
 * oldest free slot (never one taken this tick). */
static seeker_fusion_mem *track_for_label(seeker_fusion *f, const char *label, int64_t now)
{
    seeker_fusion_mem *oldest = NULL;
    for (int i = 0; i < SEEKER_FUSION_MEM; i++) {
        seeker_fusion_mem *m = &f->mem[i];
        if (m->matched)
            continue;
        if (m->label[0] && strcmp(m->label, label) == 0)
            return m;
        if (!oldest || m->seen_ms < oldest->seen_ms)
            oldest = m;
    }
    if (!oldest)
        return NULL; /* more clusters than slots: cannot happen with 16 tracks and 32 slots */
    memset(oldest, 0, sizeof *oldest);
    snprintf(oldest->label, sizeof oldest->label, "%s", label);
    oldest->id = f->next_id++;
    oldest->seen_ms = now;
    return oldest;
}

/* A position-matched track takes the cluster's current label; a coasting track holding the same
 * stale label gives it up (it keeps coasting by position). */
static void relabel(seeker_fusion *f, seeker_fusion_mem *m, const char *label)
{
    for (int i = 0; i < SEEKER_FUSION_MEM; i++)
        if (&f->mem[i] != m && !f->mem[i].matched && strcmp(f->mem[i].label, label) == 0)
            f->mem[i].label[0] = '\0';
    snprintf(m->label, sizeof m->label, "%s", label);
}

#define AB_MAX_SPEED_MPS 4.0f /* faster than anyone walks indoors; caps a bad velocity */
#define PREDICT_MIN_MPS 0.2f  /* glide only above this: still people do not drift on noise */

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

int seeker_ab_update(seeker_ab *s, float zx, float zy, float dt_s, const seeker_config *cfg)
{
    float max_dt = (float)cfg->smooth_max_dt_ms / 1000.0f;
    float dt = clampf(dt_s, 0.005f, max_dt > 0.005f ? max_dt : 0.005f);
    float px = s->x + s->vx * dt, py = s->y + s->vy * dt;
    if (!s->valid || hypotf(zx - px, zy - py) > cfg->smooth_gate_m) {
        s->valid = 1;
        s->x = zx;
        s->y = zy;
        s->vx = s->vy = 0;
        return 1;
    }
    float rx = zx - px, ry = zy - py;
    s->x = px + cfg->smooth_alpha * rx;
    s->y = py + cfg->smooth_alpha * ry;
    s->vx = clampf(s->vx + cfg->smooth_beta / dt * rx, -AB_MAX_SPEED_MPS, AB_MAX_SPEED_MPS);
    s->vy = clampf(s->vy + cfg->smooth_beta / dt * ry, -AB_MAX_SPEED_MPS, AB_MAX_SPEED_MPS);
    return 0;
}

int seeker_range_update(seeker_rf *s, float z, float dt_s, const seeker_config *cfg)
{
    float max_dt = (float)cfg->smooth_max_dt_ms / 1000.0f;
    float dt = clampf(dt_s, 0.005f, max_dt > 0.005f ? max_dt : 0.005f);
    float p = s->r + s->v * dt;
    float gate = cfg->range_smooth_gate_frac * p;
    if (gate < cfg->smooth_gate_m)
        gate = cfg->smooth_gate_m;
    if (!s->valid || fabsf(z - p) > gate) {
        s->valid = 1;
        s->r = z;
        s->v = 0;
        return 1;
    }
    float res = z - p;
    s->r = p + cfg->range_smooth_alpha * res;
    if (s->r < 0.05f)
        s->r = 0.05f;
    s->v = clampf(s->v + cfg->range_smooth_beta / dt * res, -AB_MAX_SPEED_MPS, AB_MAX_SPEED_MPS);
    return 0;
}

static long nodes_seq(const seeker_fusion_node *const *nodes, int nn)
{
    long seq = 0;
    for (int i = 0; i < nn; i++)
        seq += nodes[i]->msgs;
    return seq;
}

/* Runs the track's filter once per new datagram (the 30 Hz tick outruns the ~15 Hz phones) and
 * replaces (x, y) with the filtered position. nodes[0] is the track's primary node: dt comes from
 * its message clock (t_mono_ms, else ts_ms), or from Pi receive time when that clock is missing or
 * not increasing. `single` is the ray of a single-node range fix (else NULL): its range goes
 * through the range filter and the room filter state follows it, so a switch to a fused fix (or
 * back) continues from the current position instead of snapping. */
static void smooth_track(seeker_fusion *f, seeker_fusion_mem *mm,
                         const seeker_fusion_node *const *nodes, int nn, const seeker_ray *single,
                         int64_t now_ms, float *x, float *y)
{
    const seeker_config *cfg = f->cfg;
    const seeker_fusion_node *n = nodes[0];
    long seq = nodes_seq(nodes, nn);
    int64_t rx = 0;
    for (int i = 0; i < nn; i++)
        if (nodes[i]->last_rx_ms > rx)
            rx = nodes[i]->last_rx_ms;
    if (now_ms - mm->seen_ms > cfg->smooth_reset_ms)
        mm->ab.valid = mm->rf.valid = 0; /* new, back from LOST, or out of view for a while */
    if (mm->ab.valid && seq == mm->ab_seq) {
        *x = mm->ab.x;
        *y = mm->ab.y;
        return;
    }
    int64_t dt_ms = n->last.t_mono_ms - mm->ab_msg_ms;
    int pnode = (int)(n - f->nodes) + 1;
    if (n->last.t_mono_ms <= 0 || dt_ms <= 0 || dt_ms > cfg->drop_ms ||
        (f->handoff && mm->ab_node && mm->ab_node != pnode)) /* another phone's clock */
        dt_ms = rx - mm->ab_rx_ms;
    float dt = (float)dt_ms / 1000.0f;
    int was_valid = mm->ab.valid;
    float ox = mm->ab.x, oy = mm->ab.y;
    if (single && cfg->range_smooth && single->range_m > 0) {
        float ux = sinf(single->bearing_deg * DEG2RAD), uy = cosf(single->bearing_deg * DEG2RAD);
        if (f->handoff && mm->rf.valid && mm->rf_node != single->node + 1)
            mm->rf.valid = 0; /* its range was from another phone */
        mm->rf_node = single->node + 1;
        if (!mm->rf.valid && mm->ab.valid) /* was a fused fix: start from where the track is */
            mm->rf = (seeker_rf){ 1, hypotf(mm->ab.x - single->x_m, mm->ab.y - single->y_m), 0 };
        int rf_valid = mm->rf.valid;
        float orr = mm->rf.r;
        if (seeker_range_update(&mm->rf, single->range_m, dt, cfg) && rf_valid && f->debug)
            seeker_log("track %d range snapped %.1f m (gate %.1f m)", mm->id,
                       fabsf(single->range_m - orr),
                       fmaxf(cfg->smooth_gate_m, cfg->range_smooth_gate_frac * orr));
        *x = single->x_m + mm->rf.r * ux;
        *y = single->y_m + mm->rf.r * uy;
        mm->ab = (seeker_ab){ 1, *x, *y, mm->rf.v * ux, mm->rf.v * uy };
    } else {
        mm->rf.valid = 0;
        if (seeker_ab_update(&mm->ab, *x, *y, dt, cfg) && was_valid && f->debug)
            seeker_log("track %d snapped %.1f m (gate %.1f m)", mm->id, hypotf(*x - ox, *y - oy),
                       cfg->smooth_gate_m);
        *x = mm->ab.x;
        *y = mm->ab.y;
    }
    /* Glide velocity for predict_ms: an average of successive filtered positions. Reset when
     * the filter snapped or the track is new. */
    if (was_valid && mm->pv_valid && dt > 0.01f && hypotf(*x - mm->pv_x, *y - mm->pv_y) < cfg->smooth_gate_m) {
        float ivx = clampf((*x - mm->pv_x) / dt, -AB_MAX_SPEED_MPS, AB_MAX_SPEED_MPS);
        float ivy = clampf((*y - mm->pv_y) / dt, -AB_MAX_SPEED_MPS, AB_MAX_SPEED_MPS);
        mm->pvx += 0.4f * (ivx - mm->pvx);
        mm->pvy += 0.4f * (ivy - mm->pvy);
    } else {
        mm->pvx = mm->pvy = 0;
    }
    mm->pv_valid = 1;
    mm->pv_x = *x;
    mm->pv_y = *y;
    mm->ab_seq = seq;
    mm->ab_node = pnode;
    mm->ab_msg_ms = n->last.t_mono_ms;
    mm->ab_rx_ms = rx;
}

/* Where a live track's person should be now: its filter state moved on by its velocity (only
 * with smoothing, for at most smooth_reset_ms), else its last sent position. */
static void predict(const seeker_fusion *f, const seeker_fusion_mem *m, int64_t now_ms, float *x,
                    float *y)
{
    const seeker_config *cfg = f->cfg;
    if (cfg->smooth && m->ab.valid) {
        float h = clampf((float)(now_ms - m->ab_rx_ms) / 1000.0f, 0,
                         (float)cfg->smooth_reset_ms / 1000.0f);
        *x = m->ab.x + m->ab.vx * h;
        *y = m->ab.y + m->ab.vy * h;
    } else {
        *x = m->px;
        *y = m->py;
    }
}

/* --debug: one line per track per new datagram, raw fix against what is sent. */
static void debug_track(const seeker_fusion *f, const seeker_track *k, const seeker_fix *fx,
                        const seeker_ray *single)
{
    if (!k->has_room) {
        seeker_log("track %d %s %s bearing-only %.1f deg (no fix)", k->id, k->nodes, k->label,
                   k->node_bearing_deg);
    } else if (single) {
        float r = hypotf(k->room_x_m - single->x_m, k->room_y_m - single->y_m);
        seeker_log("track %d %s single size range %.2f -> %.2f m (sigma %.2f m) bearing %.1f "
                   "range %.2f m",
                   k->id, k->nodes, single->range_m, r,
                   seeker_tri_range_sigma(&f->tri, single->range_m, single->conf), k->bearing_deg,
                   k->range_m);
    } else {
        seeker_log("track %d %s %s cross %.0f deg sigma %.2f m fix %.2f,%.2f -> %.2f,%.2f "
                   "range %.2f m bearing %.1f",
                   k->id, k->nodes, seeker_tri_method_name(fx->method), fx->cross_deg, fx->sigma_m,
                   fx->x_m, fx->y_m, k->room_x_m, k->room_y_m, k->range_m, k->bearing_deg);
    }
}

/* level: the track's seeker_colour level (range and approach speed with hysteresis), -1 without
 * a range. */
static const char *colour_for(const seeker_config *c, const seeker_track *k, int level)
{
    if (k->age_ms > c->stale_ms)
        return "grey";
    if (!c->remote_by_range || level < 0)
        return "cyan"; /* IF-7: remote-only, not visible to the wearer */
    return seeker_colour_name(level);
}

/* Joins up to SEEKER_TRI_MAX_MEMBERS strings with '+', shortening each so the result fits. */
static void join_plus(char *dst, int dstsz, const char *const *parts, int n)
{
    int each = n > 0 ? (dstsz - 1 - (n - 1)) / n : 0, len = 0;
    dst[0] = '\0';
    for (int i = 0; i < n && len < dstsz - 1; i++)
        len += snprintf(dst + len, (size_t)(dstsz - len), "%s%.*s", i ? "+" : "", each, parts[i]);
}

/* Part of a body (SEEKER_HANDOFF_*): phone.html ranges from eyes to ankles, else the torso,
 * else the box of the visible landmarks; the last is the visible part's height, not the
 * person's. Such a contact is partial unless its box still looks like a whole person (tall,
 * narrow, clear of the frame's sides). Without landmarks there is nothing to tell: whole. */
static int contact_partial(const seeker_contact *c, int w, int h)
{
    if (!c->has_pts)
        return 0;
    const float(*p)[3] = c->pts;
    const float v = 0.5f;
    int head = p[0][2] > v || p[1][2] > v || p[2][2] > v;
    int ankle = p[15][2] > v || p[16][2] > v;
    int torso = p[5][2] > v && p[6][2] > v && p[11][2] > v && p[12][2] > v;
    if ((head && ankle) || torso)
        return 0;
    if (c->has_box && w > 0 && h > 0 && c->box[3] >= 2.2f * c->box[2] && c->box[3] >= 0.3f * (float)h &&
        c->box[0] > 0.01f * (float)w && c->box[0] + c->box[2] < 0.99f * (float)w)
        return 0;
    return 1;
}

/* SEEKER_PART_CUT: a partial contact that is an upper body the bottom of the frame cuts: the head
 * and both shoulders seen, both ankles placed at the bottom edge or below it. Nothing hides this
 * person from the phone, they stand too close for its frame (within about 2.7 m of a landscape
 * phone at chest height). Their size range is still that of a part (the box of what shows). */
static int contact_frame_cut(const seeker_contact *c)
{
    const float(*p)[3] = c->pts;
    const float v = 0.5f;
    return c->has_pts && (p[0][2] > v || p[1][2] > v || p[2][2] > v) && p[5][2] > v && p[6][2] > v &&
           p[15][1] >= SEEKER_PART_CUT_ROW && p[16][1] >= SEEKER_PART_CUT_ROW;
}

static const char *wearer_id(const seeker_config *cfg)
{
    return cfg->wearer_node[0] ? cfg->wearer_node : cfg->view_node;
}

/* Fills one track's identity fields from a triangulation cluster of contacts (ray index -> node,
 * contact). Returns the member count; nodes[0] is the primary node, then the others. The track's
 * node list names only the nodes that see the whole person (all of them when none does); mask
 * gets the same nodes as bits, and wearer_part is set for a lone partial contact of the wearer. */
static int place_cluster(const seeker_fusion *f, const seeker_tri_cluster *cl, const seeker_ray *rays,
                         const int *ray_node, const int *ray_idx, int64_t now_ms, seeker_track *k,
                         const seeker_fusion_node **nodes, const seeker_contact **primary_contact,
                         unsigned *mask, int *wearer_part)
{
    const seeker_config *cfg = f->cfg;
    int whole = 0;
    for (int m = 0; m < cl->n; m++)
        whole += !rays[cl->member[m]].partial;
    /* Prefer the wearer's view_node so its box drives the brackets, unless it sees only a part. */
    int primary = cl->member[0];
    for (int m = 0; m < cl->n && whole; m++)
        if (!rays[cl->member[m]].partial) {
            primary = cl->member[m];
            break;
        }
    const char *ids[SEEKER_TRI_MAX_MEMBERS], *labels[SEEKER_TRI_MAX_MEMBERS];
    int nid = 0;
    k->age_ms = 0;
    *mask = 0;
    *wearer_part = 0;
    for (int m = 0; m < cl->n; m++) {
        const seeker_fusion_node *mn = &f->nodes[ray_node[cl->member[m]]];
        /* Not one of the track's nodes: a view of part of them while another sees them whole,
         * unless it is only the frame that cuts them (SEEKER_PART_CUT: that phone sees them). */
        int part = whole > 0 && rays[cl->member[m]].partial &&
                   !(mn->part[ray_idx[cl->member[m]]] & SEEKER_PART_CUT);
        if (!part) {
            ids[nid++] = mn->id;
            *mask |= 1u << ray_node[cl->member[m]];
        }
        labels[m] = mn->last.c[ray_idx[cl->member[m]]].id;
        if (!strcmp(mn->id, cfg->view_node) && !part)
            primary = cl->member[m];
        if (cl->n == 1 && rays[cl->member[m]].partial && !strcmp(mn->id, wearer_id(cfg)))
            *wearer_part = 1;
        int age = (int)(now_ms - mn->last_rx_ms);
        if (age > k->age_ms)
            k->age_ms = age;
    }
    const seeker_fusion_node *n = &f->nodes[ray_node[primary]];
    const seeker_contact *c = &n->last.c[ray_idx[primary]];
    *primary_contact = c;

    snprintf(k->cls, sizeof k->cls, "%s", c->cls);
    snprintf(k->src, sizeof k->src, "remote");
    snprintf(k->node_id, sizeof k->node_id, "%s", n->id);
    k->visible = 0; /* no wearer camera after the pivot */
    k->node_bearing_deg = c->bearing_deg;
    k->has_node_box = c->has_box;
    memcpy(k->node_box, c->box, sizeof k->node_box);
    join_plus(k->nodes, sizeof k->nodes, ids, nid);
    join_plus(k->label, sizeof k->label, labels, cl->n);
    switch (cl->fix.method) {
    case SEEKER_TRI_INTERSECT: /* wider crossing angle, better fix */
        k->range_conf = 0.5f + 0.4f * sinf(cl->fix.cross_deg * DEG2RAD);
        break;
    case SEEKER_TRI_PARALLEL:
        k->range_conf = 0.6f;
        break;
    default:
        k->range_conf = 0.5f; /* single-phone mono range */
    }
    int nn = 0;
    nodes[nn++] = n;
    for (int m = 0; m < cl->n; m++)
        if (cl->member[m] != primary)
            nodes[nn++] = &f->nodes[ray_node[cl->member[m]]];
    return nn;
}

/* ---- handoff between phones (SEEKER_HANDOFF_*) ---- */

static float conf_k(float conf)
{
    float c = conf > 0 ? conf : 0.5f;
    return c < 0.1f ? 0.1f : c > 1.0f ? 1.0f : c;
}

static float bearing_sigma_m(const seeker_fusion *f, const seeker_ray *r, float dist)
{
    float k = r->partial && f->tri.partial_bearing_k > 1.0f ? f->tri.partial_bearing_k : 1.0f;
    return fmaxf(dist, 0.5f) * f->tri.bearing_sigma_deg * DEG2RAD * k / conf_k(r->conf);
}

/* Covariance of a fix: a lone phone's range point is long along its ray (the size range) and
 * narrow across it (the bearing); a fused fix is round. The wearer's partial sighting placed on
 * its bearing keeps the track's own (its distance is the track's). */
static void fix_cov(const seeker_fusion *f, const seeker_fix *fx, const seeker_ray *single, int wpart,
                    float *cxx, float *cxy, float *cyy)
{
    if (wpart)
        return;
    if (single && single->has_range) {
        float b = single->bearing_deg * DEG2RAD, ux = sinf(b), uy = cosf(b);
        float sa = seeker_tri_range_sigma(&f->tri, single->range_m, single->conf);
        float sc = bearing_sigma_m(f, single, single->range_m);
        sa *= sa;
        sc *= sc;
        *cxx = sa * ux * ux + sc * uy * uy;
        *cxy = (sa - sc) * ux * uy;
        *cyy = sa * uy * uy + sc * ux * ux;
        return;
    }
    *cxx = *cyy = fx->sigma_m * fx->sigma_m;
    *cxy = 0;
}

/* Does any '+'-separated contact id of a also appear in b? */
static int labels_share(const char *a, const char *b)
{
    for (const char *p = a; *p;) {
        const char *e = strchr(p, '+');
        size_t n = e ? (size_t)(e - p) : strlen(p);
        for (const char *q = b; n && *q;) {
            const char *qe = strchr(q, '+');
            size_t qn = qe ? (size_t)(qe - q) : strlen(q);
            if (qn == n && !strncmp(p, q, n))
                return 1;
            if (!qe)
                break;
            q = qe + 1;
        }
        if (!e)
            break;
        p = e + 1;
    }
    return 0;
}

/* Gate and cost of cluster cl for live track m predicted at pr. A lone partial sighting of the
 * wearer goes by bearing: the track's predicted spot must sit on its ray. Anything else with a
 * position: Mahalanobis distance with the fix's and the track's covariance plus track_sigma_m
 * of motion, within track_gate_sigmas (one more when the node set changes) and
 * track_gate_max_m; a shared phone contact id is enough within track_gate_max_m. The old
 * round gate still lets a pair through. */
static void handoff_gate(const seeker_fusion *f, const seeker_fusion_mem *m, const seeker_tri_prior *pr,
                         const seeker_tri_cluster *cl, const seeker_ray *rays, unsigned mask, int wpart,
                         const char *label, unsigned char *tok, double *cost)
{
    const seeker_config *cfg = f->cfg;
    float ts2 = cfg->track_sigma_m * cfg->track_sigma_m;
    if (wpart) {
        const seeker_ray *ry = &rays[cl->member[0]];
        float b = ry->bearing_deg * DEG2RAD, ux = sinf(b), uy = cosf(b);
        float vx = pr->x_m - ry->x_m, vy = pr->y_m - ry->y_m;
        float along = vx * ux + vy * uy, cross = vx * uy - vy * ux;
        if (along < 0.3f || fabsf(atan2f(cross, along)) * RAD2DEG > SEEKER_HANDOFF_BEARING_DEG)
            return;
        float sb = bearing_sigma_m(f, ry, along);
        float tn = m->cxx * uy * uy - 2.0f * m->cxy * ux * uy + m->cyy * ux * ux;
        float s2 = sb * sb + tn + ts2, g = cfg->track_gate_sigmas + 1.0f;
        *tok = cross * cross <= g * g * s2 && fabsf(cross) <= cfg->track_gate_max_m;
        *cost = cross * cross / s2;
        return;
    }
    const seeker_fix *fx = &cl->fix;
    if (fx->method == SEEKER_TRI_NONE)
        return;
    float dx = fx->x_m - pr->x_m, dy = fx->y_m - pr->y_m, d = hypotf(dx, dy);
    float s2 = ts2 + fx->sigma_m * fx->sigma_m;
    float gate = fminf(cfg->track_gate_sigmas * sqrtf(s2 > 1e-6f ? s2 : 1e-6f), cfg->track_gate_max_m);
    if (gate < cfg->smooth_gate_m)
        gate = cfg->smooth_gate_m;
    float fxx, fxy, fyy;
    fix_cov(f, fx, fx->method == SEEKER_TRI_SINGLE ? &rays[cl->member[0]] : NULL, 0, &fxx, &fxy, &fyy);
    float a = fxx + m->cxx + ts2, bb = fxy + m->cxy, c = fyy + m->cyy + ts2;
    float det = a * c - bb * bb;
    float m2 = det > 1e-9f ? (c * dx * dx - 2.0f * bb * dx * dy + a * dy * dy) / det
                           : d * d / (s2 > 1e-6f ? s2 : 1e-6f);
    float g = cfg->track_gate_sigmas + (mask != m->node_mask ? 1.0f : 0.0f);
    int shared = labels_share(m->label, label);
    *tok = d <= gate || (d <= cfg->track_gate_max_m && (m2 <= g * g || shared));
    *cost = m2 * (shared ? 0.5 : 1.0);
}

/* The wearer's partial sighting continuing track m: on its bearing, at the track's distance. */
static void bearing_fix(const seeker_fusion *f, const seeker_tri_prior *pr, const seeker_ray *ry, seeker_fix *fx)
{
    float b = ry->bearing_deg * DEG2RAD, ux = sinf(b), uy = cosf(b);
    float along = (pr->x_m - ry->x_m) * ux + (pr->y_m - ry->y_m) * uy;
    fx->method = SEEKER_TRI_PARALLEL; /* a position, filtered like a fused one */
    fx->x_m = ry->x_m + along * ux;
    fx->y_m = ry->y_m + along * uy;
    fx->sigma_m = bearing_sigma_m(f, ry, along);
    fx->cost = 0;
}

/* Node n's learned range ratio now: faded toward 1 since it was learned. */
static float range_ratio(const seeker_fusion_mem *mm, int n, int64_t now_ms)
{
    if (!mm->rk_ms[n])
        return 1.0f;
    float e = expf(-(float)(now_ms - mm->rk_ms[n]) / SEEKER_HANDOFF_RK_TAU_MS);
    return 1.0f + (mm->rk[n] - 1.0f) * e;
}

/* While two phones fix the person, each one's distance to the fix over its own size range. */
static void learn_range_ratio(const seeker_fusion *f, seeker_fusion_mem *mm, const seeker_tri_cluster *cl,
                              const seeker_ray *rays, int64_t now_ms)
{
    for (int i = 0; i < cl->n; i++)
        if (now_ms - f->nodes[rays[cl->member[i]].node].last_rx_ms > SEEKER_HANDOFF_FRESH_MS)
            return; /* one phone's view is older: the person moved in between */
    for (int i = 0; i < cl->n; i++) {
        const seeker_ray *r = &rays[cl->member[i]];
        if (!r->has_range || r->partial || r->range_m < 0.3f || r->node < 0 || r->node >= SEEKER_MAX_NODES)
            continue;
        float q = clampf(hypotf(cl->fix.x_m - r->x_m, cl->fix.y_m - r->y_m) / r->range_m, 0.5f, 2.0f);
        int n = r->node;
        float k = range_ratio(mm, n, now_ms);
        mm->rk[n] = mm->rk_ms[n] ? k + 0.2f * (q - k) : q; /* the first fused fix is the best yet */
        mm->rk_ms[n] = now_ms;
    }
}

/* The track's node set changes (another phone takes over or joins) and the new fix is more than
 * SEEKER_HANDOFF_MIN_M from where the track was drawn: the filters move to the new fix and keep
 * their velocity (a change of measurement is not motion: no velocity kick, no slow slide, no lag
 * on someone still walking), and (hx, hy) gets the old position for the glide. Returns 1 then. */
static int handoff_start(const seeker_fusion *f, seeker_fusion_mem *mm, unsigned mask, float x, float y,
                         int64_t now_ms, float *hx, float *hy)
{
    if (!mm->node_mask || mask == mm->node_mask || !mm->ab.valid || !mm->has_pos ||
        now_ms - mm->seen_ms > f->cfg->smooth_reset_ms)
        return 0;
    predict(f, mm, now_ms, hx, hy);
    if (f->occl_guard && f->cfg->smooth && mm->ab.valid) {
        /* Where it was drawn (moved on at most predict_ms): a person unseen behind a wall has not
         * walked on at the old speed into it. */
        float h = clampf((float)(now_ms - mm->ab_rx_ms) / 1000.0f, 0, (float)f->cfg->predict_ms / 1000.0f);
        *hx = mm->ab.x + mm->ab.vx * h;
        *hy = mm->ab.y + mm->ab.vy * h;
    }
    if (mm->ho_ms && now_ms - mm->ho_ms < SEEKER_HANDOFF_MS) {
        float e = expf(-(float)(now_ms - mm->ho_ms) / SEEKER_HANDOFF_TAU_MS);
        *hx += mm->ho_x * e;
        *hy += mm->ho_y * e;
    }
    if (hypotf(x - *hx, y - *hy) < SEEKER_HANDOFF_MIN_M)
        return 0;
    float h = clampf((float)(now_ms - mm->ab_rx_ms) / 1000.0f, 0, (float)f->cfg->smooth_reset_ms / 1000.0f);
    mm->ab.x = x - mm->ab.vx * h; /* so the filter's prediction for now is the new fix */
    mm->ab.y = y - mm->ab.vy * h;
    mm->rf.valid = 0; /* restarts at the new fix's distance from its phone */
    mm->pv_x = mm->ab.x;
    mm->pv_y = mm->ab.y;
    return 1;
}

/* ---- occlusion guard (SEEKER_OCCL_*) ---- */

/* Joints a contact shows at SEEKER_OCCL_FRAG_VIS inside its frame. */
static int skel_joints(const seeker_contact *c)
{
    int n = 0;
    for (int j = 0; c->has_pts && j < SEEKER_N_JOINTS; j++)
        n += c->pts[j][2] >= SEEKER_OCCL_FRAG_VIS && c->pts[j][0] > 0.005f && c->pts[j][0] < 0.995f &&
             c->pts[j][1] > 0.005f && c->pts[j][1] < 0.995f;
    return n;
}

/* The ray of cluster cl whose view the track's skeleton comes from: the primary's (prim), or with
 * every view partial the most complete one (a tie stays with the view the skeleton is in). */
static int skel_ray(const seeker_fusion *f, const seeker_fusion_mem *mm, const seeker_tri_cluster *cl,
                    const seeker_ray *rays, const int *ray_node, const int *ray_idx, int prim)
{
    int best = prim, best_n = -1;
    for (int m = 0; m < cl->n; m++) {
        int r = cl->member[m];
        if (!rays[r].partial)
            return prim;
        const seeker_fusion_node *n = &f->nodes[ray_node[r]];
        const seeker_contact *c = &n->last.c[ray_idx[r]];
        int nj = skel_joints(c) * 2 + (mm->skel.valid && !strncmp(n->id, mm->skel.node, sizeof mm->skel.node));
        if (c->has_pts && nj > best_n)
            best_n = nj, best = r;
    }
    return best;
}

/* Joints not to take from contact c (its person-local skeleton raw): placed on or past the frame's
 * edge, and, from a partial view or one near where its node saw a part in the last
 * SEEKER_OCCL_HOLD_MS, under SEEKER_OCCL_FRAG_VIS (the pose model's guesses). */
static void occl_mask(float (*raw)[3], const seeker_contact *c, int partial)
{
    seeker_posture_mask_offframe(raw, (const float (*)[3])c->pts);
    for (int j = 0; partial && j < SEEKER_N_JOINTS; j++)
        if (raw[j][2] < SEEKER_OCCL_FRAG_VIS)
            raw[j][2] = 0;
}

void seeker_fusion_tick(seeker_fusion *f, int64_t now_ms, seeker_tracks *out)
{
    const seeker_config *cfg = f->cfg;
    memset(out, 0, sizeof *out);
    snprintf(out->src, sizeof out->src, "hud_fusion");
    out->t_mono_ms = now_ms;
    out->basis_frame_seq = ++f->tick;
    snprintf(out->heading_src, sizeof out->heading_src, "none");
    out->wearer_yaw_deg = cfg->wearer_yaw_deg;
    for (int i = 0; i < f->n_nodes; i++)
        step_pose(f, &f->nodes[i], now_ms);
    /* Ranges, bearings and colours from the wearer node's pose when configured (the fixed or
     * tag-located helmet phone), else from the room origin facing wearer_yaw_deg. */
    float wx = 0, wy = 0;
    const seeker_fusion_node *wn = cfg->wearer_node[0] ? find_node(f, cfg->wearer_node, 0) : NULL;
    if (wn) {
        wx = wn->x_m;
        wy = wn->y_m;
        out->wearer_yaw_deg = wn->yaw_deg;
        snprintf(out->wearer_node, sizeof out->wearer_node, "%s", wn->id);
    }

    enum { MAX_RAYS = SEEKER_MAX_NODES * SEEKER_MAX_CONTACTS };
    seeker_ray rays[MAX_RAYS];
    int ray_node[MAX_RAYS], ray_idx[MAX_RAYS];
    int nr = 0;
    for (int i = 0; i < f->n_nodes; i++) {
        seeker_fusion_node *n = &f->nodes[i];
        if (n->state == NODE_OK && now_ms - n->last_rx_ms > cfg->drop_ms) {
            n->state = NODE_LOST;
            seeker_log("NODE %s LOST (%lld ms silent)", n->id, (long long)(now_ms - n->last_rx_ms));
            char ev[96];
            snprintf(ev, sizeof ev, "\"event\":\"node_lost\",\"node_id\":\"%s\"", n->id);
            seeker_event(ev);
        }
        if (out->n_nodes < SEEKER_MAX_NODES) {
            seeker_node_status *s = &out->nodes[out->n_nodes++];
            snprintf(s->node_id, sizeof s->node_id, "%s", n->id);
            snprintf(s->state, sizeof s->state, "%s",
                     n->state == NODE_OK ? "ok" : n->state == NODE_LOST ? "lost" : "wait");
            s->age_ms = n->msgs ? (int)(now_ms - n->last_rx_ms) : -1;
            s->x_m = n->x_m;
            s->y_m = n->y_m;
            s->yaw_deg = n->yaw_deg;
            s->hfov_deg = n->hfov_deg;
            s->n_contacts = n->state == NODE_OK ? n->last.n : 0;
            s->rate_hz = n->rate_hz;
            snprintf(s->pose_src, sizeof s->pose_src, "%s", n->pose_src);
            s->has_pitch = n->has_pitch && now_ms - n->pitch_ms < 2000;
            s->pitch_deg = n->pitch_deg;
        }
        if (n->state != NODE_OK)
            continue; /* IF-5.4: a dropped node's contacts are dropped too */
        seeker_pose pose = { n->x_m, n->y_m, n->yaw_deg };
        for (int k = 0; k < n->last.n && nr < MAX_RAYS; k++) {
            const seeker_contact *c = &n->last.c[k];
            seeker_ray *r = &rays[nr];
            r->x_m = n->x_m;
            r->y_m = n->y_m;
            r->bearing_deg = seeker_pose_world_bearing(&pose, c->bearing_deg);
            r->has_range = c->has_range;
            r->range_m = c->range_m;
            r->conf = c->conf;
            r->node = i;
            r->partial = f->handoff && (n->part[k] & 1);
            ray_node[nr] = i;
            ray_idx[nr++] = k;
        }
    }

    /* Live tracks (sent within drop_ms, coasting or not) and where they should be now. */
    int live[SEEKER_FUSION_MEM], nlive = 0;
    seeker_tri_prior pri[SEEKER_FUSION_MEM];
    for (int i = 0; i < SEEKER_FUSION_MEM; i++) {
        seeker_fusion_mem *m = &f->mem[i];
        m->matched = 0;
        if (m->id && m->has_pos && now_ms - m->seen_ms <= cfg->drop_ms) {
            predict(f, m, now_ms, &pri[nlive].x_m, &pri[nlive].y_m);
            live[nlive++] = i;
        }
    }

    /* Cross-node triangulation: the same person seen by R1 and R2 becomes one cluster. */
    static seeker_tri_cluster cls[MAX_RAYS];
    int ncl = seeker_tri_group_prior(rays, nr, &f->tri, pri, nlive, cls, MAX_RAYS);
    for (int i = 0; i < nr; i++) /* a held whole sighting: part of a body from here on */
        rays[i].partial |= (f->nodes[ray_node[i]].part[ray_idx[i]] & 2) != 0;
    int nout = ncl < SEEKER_MAX_TRACKS ? ncl : SEEKER_MAX_TRACKS;
    const seeker_fusion_node *mnodes[SEEKER_MAX_TRACKS][SEEKER_TRI_MAX_MEMBERS];
    int nms[SEEKER_MAX_TRACKS], wpart[SEEKER_MAX_TRACKS];
    unsigned cmask[SEEKER_MAX_TRACKS];
    const seeker_contact *pcon[SEEKER_MAX_TRACKS];
    for (int a = 0; a < nout; a++)
        nms[a] = place_cluster(f, &cls[a], rays, ray_node, ray_idx, now_ms, &out->tracks[a],
                               mnodes[a], &pcon[a], &cmask[a], &wpart[a]);
    out->n = nout;

    /* Track identity: fixes to live tracks by predicted position, minimum total cost. */
    double tcost[SEEKER_FUSION_MEM * SEEKER_MAX_TRACKS];
    unsigned char tok[SEEKER_FUSION_MEM * SEEKER_MAX_TRACKS];
    int row_col[SEEKER_FUSION_MEM], cl_mem[SEEKER_MAX_TRACKS];
    for (int a = 0; a < nout; a++)
        cl_mem[a] = -1;
    for (int r = 0; r < nlive; r++)
        for (int a = 0; a < nout; a++) {
            const seeker_fix *fx = &cls[a].fix;
            int at = r * nout + a;
            tok[at] = 0;
            tcost[at] = 0;
            if (f->handoff) {
                handoff_gate(f, &f->mem[live[r]], &pri[r], &cls[a], rays, cmask[a], wpart[a],
                             out->tracks[a].label, &tok[at], &tcost[at]);
                continue;
            }
            if (fx->method == SEEKER_TRI_NONE)
                continue;
            float d = hypotf(fx->x_m - pri[r].x_m, fx->y_m - pri[r].y_m);
            float s2 = cfg->track_sigma_m * cfg->track_sigma_m + fx->sigma_m * fx->sigma_m;
            float s = sqrtf(s2 > 1e-6f ? s2 : 1e-6f);
            float gate = fminf(cfg->track_gate_sigmas * s, cfg->track_gate_max_m);
            if (gate < cfg->smooth_gate_m)
                gate = cfg->smooth_gate_m;
            tok[at] = d <= gate;
            tcost[at] = d * d / (s * s);
        }
    if (nlive > 0 && nout > 0 && seeker_assign(tcost, tok, nlive, nout, row_col) > 0)
        for (int r = 0; r < nlive; r++)
            if (row_col[r] >= 0) {
                cl_mem[row_col[r]] = live[r];
                f->mem[live[r]].matched = 1;
                if (wpart[row_col[r]]) { /* on the wearer's bearing, at the track's distance */
                    seeker_tri_prior at = pri[r];
                    const seeker_fusion_mem *lm = &f->mem[live[r]];
                    if (f->occl_guard && lm->ab.valid)
                        at.x_m = lm->ab.x, at.y_m = lm->ab.y;
                    bearing_fix(f, &at, &rays[cls[row_col[r]].member[0]], &cls[row_col[r]].fix);
                }
            }
    for (int a = 0; a < nout; a++)
        if (cl_mem[a] >= 0)
            relabel(f, &f->mem[cl_mem[a]], out->tracks[a].label);
    int drop[SEEKER_MAX_TRACKS] = { 0 };
    for (int a = 0; a < nout; a++) {
        if (cl_mem[a] >= 0)
            continue;
        if (wpart[a]) { /* the wearer sees part of someone no track is near: no new person */
            drop[a] = 1;
            continue;
        }
        seeker_fusion_mem *m = track_for_label(f, out->tracks[a].label, now_ms);
        if (m) {
            m->matched = 1;
            cl_mem[a] = (int)(m - f->mem);
        }
    }

    for (int a = 0; a < nout; a++) {
        seeker_track *k = &out->tracks[a];
        int nm = nms[a];
        if (cl_mem[a] < 0) {
            k->id = 0;
            continue;
        }
        seeker_fusion_mem *mm = &f->mem[cl_mem[a]];
        k->id = mm->id;
        float x = cls[a].fix.x_m, y = cls[a].fix.y_m;
        int has_room = cls[a].fix.method != SEEKER_TRI_NONE;
        const seeker_ray *single =
            cls[a].fix.method == SEEKER_TRI_SINGLE ? &rays[cls[a].member[0]] : NULL;
        seeker_ray scaled;
        float hx = 0, hy = 0;
        int ho = 0;
        if (f->handoff && has_room) {
            if (cls[a].fix.method == SEEKER_TRI_INTERSECT)
                learn_range_ratio(f, mm, &cls[a], rays, now_ms);
            if (single && single->has_range && mm->rk_ms[single->node] > 0) {
                scaled = *single;
                scaled.range_m *= range_ratio(mm, single->node, now_ms);
                seeker_node_to_room(scaled.x_m, scaled.y_m, 0, scaled.bearing_deg, scaled.range_m, &x, &y);
                single = &scaled;
            }
            ho = handoff_start(f, mm, cmask[a], x, y, now_ms, &hx, &hy);
        }
        if (has_room && cfg->smooth)
            smooth_track(f, mm, mnodes[a], nm, single, now_ms, &x, &y);
        if (ho) { /* glide from where the person was drawn to the new fix */
            mm->ho_x = hx - x;
            mm->ho_y = hy - y;
            mm->ho_ms = now_ms;
            mm->handoffs++;
            if (f->debug)
                seeker_log("track %d handoff to %s: %.2f m from where it was, gliding over %d ms",
                           mm->id, k->nodes, hypotf(mm->ho_x, mm->ho_y), SEEKER_HANDOFF_MS);
        }
        if (f->handoff && has_room) {
            fix_cov(f, &cls[a].fix, single, wpart[a], &mm->cxx, &mm->cxy, &mm->cyy);
            mm->node_mask = cmask[a];
        }
        mm->has_pos = has_room;
        mm->px = x;
        mm->py = y;
        mm->pos_ms = now_ms;
        if (has_room && cfg->smooth && cfg->predict_ms > 0 && mm->pv_valid &&
            hypotf(mm->pvx, mm->pvy) > PREDICT_MIN_MPS) {
            /* Phones report at about 15 Hz, the HUD draws at 30: without this every person
             * moves in two-frame steps. Glide velocity, horizon capped; a person standing
             * still (under PREDICT_MIN_MPS of noise) is not pushed around. */
            float h = clampf((float)(now_ms - mm->ab_rx_ms) / 1000.0f, 0,
                             (float)cfg->predict_ms / 1000.0f);
            x += mm->pvx * h;
            y += mm->pvy * h;
        }
        if (has_room && mm->ho_ms && now_ms - mm->ho_ms < SEEKER_HANDOFF_MS) {
            float e = expf(-(float)(now_ms - mm->ho_ms) / SEEKER_HANDOFF_TAU_MS);
            x += mm->ho_x * e;
            y += mm->ho_y * e;
        }
        if (has_room) {
            k->has_room = 1;
            k->room_x_m = x;
            k->room_y_m = y;
            k->has_range = 1;
            k->range_m = hypotf(x - wx, y - wy);
            snprintf(k->range_src, sizeof k->range_src, "remote");
            k->has_bearing = 1;
            k->bearing_deg = seeker_wrap180(atan2f(x - wx, y - wy) * RAD2DEG - out->wearer_yaw_deg);
        }
        {
            const seeker_fusion_node *pn = mnodes[a][0];
            const seeker_contact *sc = pcon[a];
            int sr = -1, guard = f->handoff && f->occl_guard;
            for (int m = 0; guard && m < cls[a].n && sr < 0; m++) {
                int r = cls[a].member[m];
                if (&f->nodes[ray_node[r]].last.c[ray_idx[r]] == pcon[a])
                    sr = skel_ray(f, mm, &cls[a], rays, ray_node, ray_idx, r);
            }
            if (sr >= 0) {
                pn = &f->nodes[ray_node[sr]];
                sc = &pn->last.c[ray_idx[sr]];
            }
            float raw[SEEKER_N_JOINTS][3];
            int have = has_room &&
                       seeker_skel_local(sc, pn->image_w, pn->image_h, pn->hfov_deg,
                                         hypotf(x - pn->x_m, y - pn->y_m), raw);
            if (have && sr >= 0)
                occl_mask(raw, sc, pn->part[ray_idx[sr]] != 0);
            k->has_pts = seeker_skel_step(&mm->skel, pn->id, pn->msgs, pn->last_rx_ms,
                                          have ? (const float (*)[3])raw : NULL,
                                          f->skel_smooth ? 1 | (guard ? SEEKER_SKEL_CONFIRM : 0) : 0,
                                          k->pts);
            int was = mm->posture.label;
            float pp[SEEKER_N_JOINTS][3];
            if (k->has_pts) {
                memcpy(pp, k->pts, sizeof pp);
                if (have)
                    seeker_posture_mask_offframe(pp, (const float (*)[3])sc->pts);
            }
            k->posture = seeker_posture_update(
                &mm->posture, k->has_pts ? (const float (*)[3])pp : NULL, now_ms);
            if (k->posture != was) {
                char why[160];
                seeker_posture_describe(&mm->posture.feat, why, sizeof why);
                seeker_log("track %d %s posture %s -> %s (%s)", mm->id, k->nodes,
                           seeker_posture_name(was), seeker_posture_name(k->posture), why);
            }
        }

        int level = -1;
        if (k->has_range) {
            level = seeker_colour_update(&mm->colour, k->range_m, now_ms, cfg);
            k->closing_mps = mm->colour.closing_mps;
        }
        mm->seen_ms = now_ms;
        snprintf(k->colour, sizeof k->colour, "%s", colour_for(cfg, k, level));
        long seq = nodes_seq(mnodes[a], nm);
        if (f->debug && seq != mm->dbg_seq) {
            mm->dbg_seq = seq;
            debug_track(f, k, &cls[a].fix, single);
        }
    }
    int kept = 0;
    for (int a = 0; a < nout; a++)
        if (!drop[a]) {
            if (kept != a)
                out->tracks[kept] = out->tracks[a];
            kept++;
        }
    out->n = kept;
}
