/* Walk-behind-wall regression: a person walks from R1's view behind the divider's edge while R2
 * sees them whole, through the real hud_fusion core, the real tracks datagram and the real wearer
 * view (plan theme), on a simulated clock; then direct checks of each part of fusion's occlusion
 * guard (seeker_fusion.h SEEKER_OCCL_*) and of the HUD's crossfade. Each replay scene runs on 10
 * seeds, twice per seed: before (the guard off, hud_fusion --no-occl-guard, and the outgoing
 * ghost of a crossfade drawn from the track's pts as the old HUD did) and after (the defaults).
 *
 *   test_occlusion [seed]     (ctest "occlusion"; seeds seed..seed+9, default 11; host and QNX
 *                              alike, no sockets, no screen)
 *
 * Room as in test_handoff.c: R1, the wearer, pinned at 0,-2 facing the divider; R2 pinned at
 * -1.59,-1.4 yaw 49; the divider is the wall -0.2,-0.6 to 1.4,-0.6 (its left edge is 8 deg left of
 * R1's boresight). Phones report at 15 Hz, 20 to 40 ms late, bearings +-0.7 deg; R2's range reads
 * 25% long, R1's 15% short; R1 misses 12% of its frames.
 *
 * In R1's frame every joint right of the wall's edge is hidden, and the pose model still places it
 * (MediaPipe in VIDEO mode tracks on from the last frame): pushed on into the wall and fanned out
 * (arms up, legs down: the '<' Felix saw), the hips next to what still shows at visibility 0.45
 * to 0.75, the rest at 0.1 to 0.5; in 8% of frames it snaps to a confident guess of the whole body
 * (hips 0.6 to 0.9, the rest 0.55 to 0.85), head and ankles over 0.5. R1's contact follows
 * phone.html: hip midpoint bearing (the guessed hip too), confidence the mean hip visibility
 * (under 0.6 dropped), box of the joints at 0.3, the upright gate (box 1.5 times taller than
 * wide), range from eyes to ankles, else the torso, else the box; R1 finds nobody once fewer
 * than 2 joints show.
 *
 *   walk-behind       stands, walks right at 0.8 m/s behind the edge, on behind the wall.
 *   edge linger       the same, but stays mostly behind the edge (one side showing) for 1.6 s.
 *   edge linger, R2 edge  R2 turned so the person is at the right edge of its frame: joints past
 *                     the edge are placed further out at visibility 0.8 (the pose model's habit).
 *
 * Metrics, summed or worst over the seeds: ghost frames (an x-ray skeleton drawn, the outgoing
 * ghost of a crossfade included), the worst shape error of a drawn ghost (largest distance of a
 * drawn joint from the true body, person heights), ghost frames with a joint more than 0.2 off
 * (splayed), the largest one-frame jump of a drawn joint, and, while R1 sends a cut contact, the
 * largest step of the sent position in one 33 ms tick and its error against the truth (mean of
 * the seeds' p50, worst). */

#include "anim.h"
#include "draw.h"
#include "seeker_config.h"
#include "seeker_fusion.h"
#include "seeker_msgs.h"
#include "seeker_skel.h"
#include "wearer_view.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f
#define N_SEEDS 10

static int fails;
#define WANT(c, what)                                                                              \
    do {                                                                                           \
        if (!(c)) {                                                                                \
            printf("FAIL %s\n", what);                                                             \
            fails++;                                                                               \
        }                                                                                          \
    } while (0)

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

/* COCO-17, person-height units from the mid-hip (x right, y down), standing. */
static const float BODY[SEEKER_N_JOINTS][2] = {
    { 0.00f, -0.43f }, { -0.02f, -0.45f }, { 0.02f, -0.45f }, { -0.05f, -0.44f }, { 0.05f, -0.44f },
    { -0.12f, -0.29f }, { 0.12f, -0.29f }, { -0.15f, -0.12f }, { 0.16f, -0.13f }, { -0.16f, 0.03f },
    { 0.17f, 0.03f },  { -0.08f, 0.00f }, { 0.08f, 0.00f },  { -0.08f, 0.25f }, { 0.08f, 0.25f },
    { -0.08f, 0.48f }, { 0.08f, 0.48f },
};

typedef struct {
    float x, y, yaw;
} pose3;
static const pose3 R1 = { 0.0f, -2.0f, 0.0f }, R2_NEAR = { -1.59f, -1.4f, 49.0f },
                   R2_EDGE = { -1.59f, -1.4f, 15.0f };
static const float WALL[4] = { -0.2f, -0.6f, 1.4f, -0.6f };

enum { WALK = 0, WALK_EDGE, WALK_R2_EDGE, N_SCENES };
static const char *const SCENE[] = { "walk-behind", "edge linger", "edge linger, R2 edge" };

static const char *cfg_path(void)
{
    static char p[512] = "config/seeker.live.json";
#ifdef SEEKER_SOURCE_DIR
    snprintf(p, sizeof p, "%s/config/seeker.live.json", SEEKER_SOURCE_DIR);
    if (access(p, R_OK) != 0)
        snprintf(p, sizeof p, "config/seeker.live.json");
#endif
    return p;
}

/* Stand at x -1.0 until 1 s, walk right at 0.8 m/s; at the edge (the linger scenes) stay mostly
 * behind it (one side showing, swaying 3 cm) for 1.6 s, then walk on behind the wall. */
static void truth(int scene, int64_t t, float *x, float *y)
{
    const float ax = -1.0f, ay = -0.1f, ex = -0.17f, ey = 0.05f, bx = 0.75f, by = 0.25f;
    float t0 = 1000, t1 = t0 + 1000.0f * hypotf(ex - ax, ey - ay) / 0.8f, t2 = scene == WALK ? t1 : t1 + 1600;
    float t3 = t2 + 1000.0f * hypotf(bx - ex, by - ey) / 0.8f, tt = (float)t;
    if (tt < t0) {
        *x = ax, *y = ay;
    } else if (tt < t1) {
        float s = (tt - t0) / (t1 - t0);
        *x = ax + (ex - ax) * s, *y = ay + (ey - ay) * s;
    } else if (tt < t2) {
        *x = ex + 0.03f * sinf((tt - t1) * 0.004f), *y = ey;
    } else {
        float s = tt < t3 ? (tt - t2) / (t3 - t2) : 1.0f;
        *x = ex + (bx - ex) * s, *y = ey + (by - ey) * s;
    }
}

static float wrap180(float b)
{
    while (b > 180)
        b -= 360;
    while (b <= -180)
        b += 360;
    return b;
}

static float focal(void)
{
    return 320.0f / tanf(33.0f * DEG2RAD);
}

/* phone.html's contact from 17 image-normalized landmarks: box of the joints at 0.3, hip midpoint
 * bearing, range from eyes to ankles, else the torso, else the box. 0 when its gates drop it. */
static int phone_contact(const char *node, const float (*pts)[3], float range_bias, seeker_contact *c)
{
    float f = focal();
    if (0.5f * (pts[11][2] + pts[12][2]) < 0.6f)
        return 0; /* CONF_MIN */
    float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
    for (int j = 0; j < SEEKER_N_JOINTS; j++)
        if (pts[j][2] >= 0.3f) {
            x0 = fminf(x0, pts[j][0] * 640.0f), x1 = fmaxf(x1, pts[j][0] * 640.0f);
            y0 = fminf(y0, pts[j][1] * 480.0f), y1 = fmaxf(y1, pts[j][1] * 480.0f);
        }
    if (y1 - y0 < 1.5f * (x1 - x0))
        return 0; /* upright gate */
    memset(c, 0, sizeof *c);
    snprintf(c->id, sizeof c->id, "%s-p1", node);
    snprintf(c->cls, sizeof c->cls, "person");
    memcpy(c->pts, pts, sizeof c->pts);
    c->has_pts = 1;
    c->has_box = 1;
    c->box[0] = x0, c->box[1] = y0, c->box[2] = x1 - x0, c->box[3] = y1 - y0;
    c->conf = 0.5f * (pts[11][2] + pts[12][2]);
    float hu = 0.5f * (pts[11][0] + pts[12][0]) * 640.0f;
    c->bearing_deg = atanf((hu - 320.0f) / f) * RAD2DEG;
    float hp;
    int eye = pts[1][2] > 0.5f || pts[2][2] > 0.5f, ank = pts[15][2] > 0.5f || pts[16][2] > 0.5f;
    if (eye && ank) {
        int e = pts[1][2] > 0.5f ? 1 : 2, a = pts[15][2] > 0.5f ? 15 : 16;
        hp = hypotf((pts[e][0] - pts[a][0]) * 640.0f, (pts[e][1] - pts[a][1]) * 480.0f) / 0.9f;
    } else if (pts[5][2] > 0.5f && pts[6][2] > 0.5f && pts[11][2] > 0.5f && pts[12][2] > 0.5f) {
        hp = hypotf((pts[5][0] + pts[6][0] - pts[11][0] - pts[12][0]) * 320.0f,
                    (pts[5][1] + pts[6][1] - pts[11][1] - pts[12][1]) * 240.0f) / 0.3f;
    } else {
        hp = c->box[3];
    }
    c->has_range = hp > 10.0f;
    c->range_m = f * 1.7f / hp * (1.0f + range_bias);
    return 1;
}

static void msg_head(seeker_remote_contacts *m, const char *node, int64_t t)
{
    memset(m, 0, sizeof *m);
    snprintf(m->node_id, sizeof m->node_id, "%s", node);
    m->v = 2;
    m->t_mono_ms = t + (!strcmp(node, "R1") ? 50000 : 90000); /* each phone has its own clock */
    m->width = 640;
    m->height = 480;
    m->hfov_deg = 66;
}

/* One replay message. hidden (R1) gets how many joints the wall hid. */
static void make_msg(int scene, const char *node, pose3 p, int64_t t, seeker_remote_contacts *m, int *hidden)
{
    msg_head(m, node, t);
    int r1 = !strcmp(node, "R1");
    float x, y;
    truth(scene, t, &x, &y);
    float r = hypotf(x - p.x, y - p.y), b = wrap180(atan2f(x - p.x, y - p.y) * RAD2DEG - p.yaw);
    if ((r1 && urand() < 0.12) || fabsf(b) > 33.0f)
        return; /* under the phone's confidence cut this frame, or out of its view */
    float f = focal(), hpx = f * 1.7f / r, u = 320.0f + f * tanf(b * DEG2RAD);
    float ue = 320.0f + f * tanf((atan2f(WALL[0] - R1.x, WALL[1] - R1.y) * RAD2DEG - R1.yaw) * DEG2RAD);
    float pts[SEEKER_N_JOINTS][3];
    int nhid = 0, spike = urand() < 0.08;
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        float px = u + (BODY[j][0] + 0.01f * (float)gauss()) * hpx;
        float py = 240.0f + (BODY[j][1] + 0.01f * (float)gauss()) * hpx;
        float v = 0.9f;
        if (r1 && px > ue) { /* behind the wall: guessed, pushed on into it and fanned out */
            float d = (px - ue) / hpx, k = 0.5f + 0.7f * (float)urand();
            int arm = j >= 7 && j <= 10, leg = j >= 13, core = j == 5 || j == 6 || j == 11 || j == 12;
            px += (0.8f * d + (arm || leg ? 0.06f : 0.0f)) * k * hpx;
            py += (arm ? -0.7f * d - 0.05f : leg ? 0.25f * d + 0.03f : 0.0f) * k * hpx;
            float lo = core ? (spike ? 0.6f : 0.45f) : (spike ? 0.55f : 0.1f);
            v = lo + (core ? 0.3f : spike ? 0.3f : 0.4f) * (float)urand();
            nhid++;
        }
        if (!r1 && px > 640.0f) { /* past R2's frame edge: placed further out, confidently */
            px += (0.1f + 0.25f * (float)urand()) * hpx;
            py += (j >= 7 && j <= 10 ? -0.15f : 0.1f) * (float)urand() * hpx;
            v = 0.8f;
        }
        pts[j][0] = px / 640.0f;
        pts[j][1] = py / 480.0f;
        pts[j][2] = v;
    }
    *hidden = nhid;
    if (SEEKER_N_JOINTS - nhid < 2)
        return; /* too little of them left for the pose model */
    if (phone_contact(node, (const float (*)[3])pts, r1 ? -0.15f : 0.25f, &m->c[m->n])) {
        m->c[m->n].bearing_deg += 0.7f * (float)gauss();
        m->c[m->n].range_m *= 1.0f + 0.04f * (float)gauss();
        m->n++;
    }
}

typedef struct {
    int ghost_frames, splay_frames, cut_ticks;
    double shape_max, joint_jump, max_step, err_p50, err_max;
} metrics;

static int cmp_d(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return x < y ? -1 : x > y;
}

/* Largest distance of a drawn joint (visibility 0.3) from the true body, person heights. */
static double shape_err(const float (*p)[3])
{
    double e = 0;
    for (int j = 0; j < SEEKER_N_JOINTS; j++)
        if (p[j][2] >= 0.3f)
            e = fmax(e, hypot(p[j][0] - BODY[j][0], p[j][1] - BODY[j][1]));
    return e;
}

static void view_init(wv_view *v)
{
    memset(v, 0, sizeof *v);
    wv_config_defaults(&v->cfg);
    snprintf(v->cfg.node, sizeof v->cfg.node, "R1");
    v->cfg.pose.x_m = R1.x, v->cfg.pose.y_m = R1.y, v->cfg.pose.yaw_deg = R1.yaw;
    v->cfg.cam.image_w = 640, v->cfg.cam.image_h = 480;
    v->cfg.inset = 0;
    v->cfg.n_walls = 1;
    memcpy(v->cfg.walls[0], WALL, sizeof WALL);
    v->cfg.handoff = 1;
}

static void run(int scene, int guard, metrics *out)
{
    static seeker_config cfg;
    seeker_config_load(&cfg, cfg_path());
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    f.occl_guard = guard;
    const pose3 R2 = scene == WALK_R2_EDGE ? R2_EDGE : R2_NEAR;
    seeker_pose p1 = { R1.x, R1.y, R1.yaw }, p2 = { R2.x, R2.y, R2.yaw };
    seeker_fusion_set_pose(&f, "R1", &p1, "cli", 1);
    seeker_fusion_set_pose(&f, "R2", &p2, "cli", 1);
    static wv_view v;
    view_init(&v);
    enum { W = 640, H = 360 };
    static uint32_t px[W * H];
    canvas c = { px, W, H, W };

    enum { MAXM = 512, MAXE = 400 };
    static seeker_remote_contacts q[MAXM];
    static int64_t q_at[MAXM];
    int nq = 0, have = 0, cut = 0, have_last = 0, have_prev = 0, nerr = 0;
    int64_t next_gen[2] = { 0, 33 };
    static seeker_tracks tracks, latest;
    char buf[SEEKER_MAX_DATAGRAM], err[64];
    float last_x = 0, last_y = 0;
    static float prev[SEEKER_N_JOINTS][3];
    static double errs[MAXE];
    memset(out, 0, sizeof *out);

    for (int64_t now = 0; now < 7000; now++) {
        for (int n = 0; n < 2; n++)
            if (now >= next_gen[n] && nq < MAXM) {
                int hid = 0;
                make_msg(scene, n ? "R2" : "R1", n ? R2 : R1, now, &q[nq], &hid);
                if (!n)
                    cut = hid > 0 && q[nq].n > 0; /* R1 sends someone the wall cuts */
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
            out->cut_ticks += cut;
            if (latest.n == 1 && latest.tracks[0].has_room) {
                const seeker_track *k = &latest.tracks[0];
                float tx, ty;
                truth(scene, now, &tx, &ty);
                if (have_last && cut)
                    out->max_step = fmax(out->max_step, hypotf(k->room_x_m - last_x, k->room_y_m - last_y));
                if (cut && nerr < MAXE)
                    errs[nerr++] = hypotf(k->room_x_m - tx, k->room_y_m - ty);
                last_x = k->room_x_m, last_y = k->room_y_m, have_last = 1;
            } else {
                have_last = 0;
            }
        }
        if (now % 33 != 11 || !have)
            continue;
        v.clock_ms = now + 1;
        wv_render(&c, &cfg, &v, &latest, 0, NULL);
        if (latest.n != 1 || v.last.n_skel == 0 || !latest.tracks[0].has_pts) {
            have_prev = 0;
            continue;
        }
        const seeker_track *k = &latest.tracks[0];
        /* The ghost drawn: the track's pts, but in a crossfade to seen the outgoing ghost is the
         * one last drawn as x-ray (the old HUD drew this frame's pts there too). */
        const float(*gp)[3] = (const float(*)[3])k->pts;
        for (int j = 0; guard && j < WV_COLOUR_SLOTS; j++) {
            const wv_colour_slot *s = &v.colours[j];
            if (s->id == k->id && s->shown_direct && v.last.n_fading && s->ghost_valid)
                gp = (const float(*)[3])s->ghost_pts;
        }
        double e = shape_err(gp);
        out->ghost_frames++;
        out->splay_frames += e > 0.2;
        out->shape_max = fmax(out->shape_max, e);
        for (int j = 0; have_prev && j < SEEKER_N_JOINTS; j++)
            if (gp[j][2] >= 0.3f && prev[j][2] >= 0.3f)
                out->joint_jump = fmax(out->joint_jump, hypot(gp[j][0] - prev[j][0], gp[j][1] - prev[j][1]));
        memcpy(prev, gp, sizeof prev);
        have_prev = 1;
    }
    if (nerr > 0) {
        qsort(errs, (size_t)nerr, sizeof errs[0], cmp_d);
        out->err_p50 = errs[nerr / 2];
        out->err_max = errs[nerr - 1];
    }
}

static void add(metrics *sum, const metrics *m)
{
    sum->ghost_frames += m->ghost_frames;
    sum->splay_frames += m->splay_frames;
    sum->cut_ticks += m->cut_ticks;
    sum->shape_max = fmax(sum->shape_max, m->shape_max);
    sum->joint_jump = fmax(sum->joint_jump, m->joint_jump);
    sum->max_step = fmax(sum->max_step, m->max_step);
    sum->err_p50 += m->err_p50 / N_SEEDS;
    sum->err_max = fmax(sum->err_max, m->err_max);
}

static void print(const char *tag, const metrics *m)
{
    printf("  %-6s ghost frames %4d, worst shape %.2f, splayed %2d, joint jump %.2f, max step %.2f m, "
           "error p50 %.2f max %.2f m (%d ticks R1 sees them cut)\n",
           tag, m->ghost_frames, m->shape_max, m->splay_frames, m->joint_jump, m->max_step, m->err_p50,
           m->err_max, m->cut_ticks);
}

static void replay(unsigned seed)
{
    metrics m[N_SCENES][2];
    memset(m, 0, sizeof m);
    for (int s = 0; s < N_SCENES; s++)
        for (unsigned d = 0; d < N_SEEDS; d++)
            for (int g = 0; g < 2; g++) {
                metrics one;
                g_seed = seed + d + (unsigned)s;
                run(s, g, &one);
                add(&m[s][g], &one);
            }
    printf("walk-behind-wall replay, seeds %u..%u (before = --no-occl-guard and the old crossfade):\n", seed,
           seed + N_SEEDS - 1);
    int splay_before = 0;
    for (int s = 0; s < N_SCENES; s++) {
        printf("%s\n", SCENE[s]);
        print("before", &m[s][0]);
        print("after", &m[s][1]);
        splay_before += m[s][0].splay_frames;
    }
    WANT(splay_before > 0, "replay: before the guard, the ghost splays (Felix's '<')");
    for (int s = 0; s < N_SCENES; s++) {
        const metrics *b = &m[s][0], *a = &m[s][1];
        char what[160];
        snprintf(what, sizeof what, "%s: no splayed ghost (every drawn joint within 0.2 of the body)", SCENE[s]);
        WANT(a->splay_frames == 0 && a->shape_max < 0.2, what);
        snprintf(what, sizeof what, "%s: no drawn joint jumps 0.1 person heights in a frame", SCENE[s]);
        WANT(a->joint_jump < 0.1, what);
        snprintf(what, sizeof what, "%s: the ghost is drawn at least as long as before", SCENE[s]);
        WANT(a->ghost_frames >= b->ghost_frames, what);
        snprintf(what, sizeof what, "%s: the position steps and errs no more than before (5 cm, 2 cm p50)", SCENE[s]);
        WANT(a->max_step <= b->max_step + 0.05 && a->max_step < 0.25 && a->err_max <= b->err_max + 0.05 &&
                 a->err_p50 <= b->err_p50 + 0.02,
             what);
    }
}

/* ---- direct checks ---- */

static void body_pts(float pts[SEEKER_N_JOINTS][3], float u, float r, float vis)
{
    float hpx = focal() * 1.7f / r;
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        pts[j][0] = (u + BODY[j][0] * hpx) / 640.0f;
        pts[j][1] = (240.0f + BODY[j][1] * hpx) / 480.0f;
        pts[j][2] = vis;
    }
}

/* A whole-body contact of the person at room x, y as node p sees them (exact). */
static int see(const char *node, pose3 p, float x, float y, seeker_contact *c)
{
    float r = hypotf(x - p.x, y - p.y), b = wrap180(atan2f(x - p.x, y - p.y) * RAD2DEG - p.yaw);
    float pts[SEEKER_N_JOINTS][3];
    body_pts(pts, 320.0f + focal() * tanf(b * DEG2RAD), r, 0.9f);
    if (!phone_contact(node, (const float (*)[3])pts, 0, c))
        return 0;
    c->bearing_deg = b;
    c->range_m = r;
    return 1;
}

/* A whole sighting near where R1 just saw part of someone is held partial for
 * SEEKER_OCCL_WHOLE_N - 1 messages; another bearing, or one after SEEKER_OCCL_HOLD_MS, is not. */
static void check_hold(void)
{
    static seeker_config cfg;
    seeker_config_load(&cfg, cfg_path());
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    seeker_pose p1 = { R1.x, R1.y, R1.yaw };
    int i = seeker_fusion_set_pose(&f, "R1", &p1, "cli", 1);
    seeker_remote_contacts m;
    int64_t t = 1000;
    int part[8];
    for (int k = 0; k < 8; k++, t += 66) {
        msg_head(&m, "R1", t);
        float pts[SEEKER_N_JOINTS][3];
        body_pts(pts, 300.0f, 2.2f, 0.9f);
        if (k == 0 || k == 4) /* the left shoulder, arm, hip and knee show; the right side is guessed */
            for (int j = 0; j < SEEKER_N_JOINTS; j++)
                pts[j][2] = j == 5 || j == 7 || j == 9 || j == 11 || j == 13 ? 0.9f
                            : j == 12                                       ? 0.6f
                            : j == 6                                        ? 0.35f
                                                                            : 0.2f;
        if (k == 7) /* far from the part, another person */
            for (int j = 0; j < SEEKER_N_JOINTS; j++)
                pts[j][0] -= 0.25f;
        m.n = phone_contact("R1", (const float (*)[3])pts, 0, &m.c[0]);
        seeker_fusion_ingest(&f, &m, t);
        part[k] = m.n ? f.nodes[i].part[0] : -1;
    }
    printf("hold: R1's contacts part %d, then whole %d %d %d, part %d, whole %d %d, elsewhere %d (1 part, "
           "2 held, 4 near)\n",
           part[0], part[1], part[2], part[3], part[4], part[5], part[6], part[7]);
    WANT(part[0] == 5 && part[4] == 5, "hold: a side-cut contact is partial");
    WANT(part[1] == 6 && part[2] == 6 && part[3] == 4 && part[5] == 6 && part[6] == 6,
         "hold: whole sightings near it are held partial twice, then whole (still near)");
    WANT(part[7] == 0, "hold: a whole sighting 15 deg away is whole at once");
    msg_head(&m, "R1", t += SEEKER_OCCL_HOLD_MS + 100);
    float pts[SEEKER_N_JOINTS][3];
    body_pts(pts, 300.0f, 2.2f, 0.9f);
    m.n = phone_contact("R1", (const float (*)[3])pts, 0, &m.c[0]);
    seeker_fusion_ingest(&f, &m, t);
    WANT(m.n == 1 && f.nodes[i].part[0] == 0, "hold: after SEEKER_OCCL_HOLD_MS a whole sighting is whole");
}

/* R2 alone sees someone whose right arm is past its frame's edge (placed there at visibility
 * 0.9): the sent skeleton hides those joints; without the guard it drew them. */
static void check_offframe(void)
{
    static seeker_config cfg;
    seeker_config_load(&cfg, cfg_path());
    int drawn[2] = { 0, 0 };
    for (int g = 0; g < 2; g++) {
        static seeker_fusion f;
        seeker_fusion_init(&f, &cfg);
        f.occl_guard = g;
        seeker_pose p2 = { R2_NEAR.x, R2_NEAR.y, R2_NEAR.yaw };
        seeker_fusion_set_pose(&f, "R2", &p2, "cli", 1);
        static seeker_tracks t;
        for (int k = 0; k < 15; k++) {
            seeker_remote_contacts m;
            msg_head(&m, "R2", 1000 + 66 * k);
            float pts[SEEKER_N_JOINTS][3];
            body_pts(pts, 570.0f, 2.0f, 0.9f);
            for (int j = 6; j <= 10; j += 2)
                pts[j][0] = 1.0f + 0.01f * (float)(j - 4);
            m.n = phone_contact("R2", (const float (*)[3])pts, 0, &m.c[0]);
            if (!m.n) {
                WANT(0, "offframe: the test contact passes phone.html's gates");
                return;
            }
            seeker_fusion_ingest(&f, &m, 1000 + 66 * k);
            seeker_fusion_tick(&f, 1000 + 66 * k + 5, &t);
        }
        for (int j = 6; t.n == 1 && t.tracks[0].has_pts && j <= 10; j += 2)
            drawn[g] += t.tracks[0].pts[j][2] >= 0.3f;
    }
    printf("off-frame: right arm joints drawn, guard off %d, on %d (of 3)\n", drawn[0], drawn[1]);
    WANT(drawn[0] == 3 && drawn[1] == 0, "offframe: joints past the frame's edge are not drawn");
}

/* Someone seen by R1 and R2 walks at 1 m/s, is lost by both for 400 ms and stops; R2 picks them up
 * again where they stopped. The handoff glide starts where they were drawn, not where the old
 * speed would have taken them (0.4 m on, into the wall). */
static void check_glide_origin(void)
{
    static seeker_config cfg;
    seeker_config_load(&cfg, cfg_path());
    double over[2] = { 0, 0 }, jump[2] = { 0, 0 };
    for (int g = 0; g < 2; g++) {
        static seeker_fusion f;
        seeker_fusion_init(&f, &cfg);
        f.occl_guard = g;
        seeker_pose p1 = { R1.x, R1.y, R1.yaw }, p2 = { R2_NEAR.x, R2_NEAR.y, R2_NEAR.yaw };
        seeker_fusion_set_pose(&f, "R1", &p1, "cli", 1);
        seeker_fusion_set_pose(&f, "R2", &p2, "cli", 1);
        static seeker_tracks t;
        const float y = 0.1f, stop_x = -0.05f;
        float last_x = 0;
        int have = 0;
        for (int64_t now = 0; now < 3000; now += 33) {
            float x = now < 1200 ? -1.25f + (float)now / 1000.0f : stop_x;
            if (now % 66 == 0)
                for (int n = 0; n < 2; n++) { /* R1 lost them behind the wall, R2 for 400 ms */
                    seeker_remote_contacts m;
                    msg_head(&m, n ? "R2" : "R1", now);
                    if (now < 1200 || (n && now >= 1600))
                        m.n = see(n ? "R2" : "R1", n ? R2_NEAR : R1, x, y, &m.c[0]);
                    seeker_fusion_ingest(&f, &m, now);
                }
            seeker_fusion_tick(&f, now + 5, &t);
            if (t.n != 1 || !t.tracks[0].has_room)
                continue;
            float sx = t.tracks[0].room_x_m;
            if (now >= 1600) {
                over[g] = fmax(over[g], sx - stop_x);
                if (have)
                    jump[g] = fmax(jump[g], fabsf(sx - last_x));
            }
            last_x = sx;
            have = 1;
        }
    }
    printf("glide origin after a 400 ms loss: past the stop %.2f m (guard off) %.2f m (on), largest step %.2f / %.2f m\n",
           over[0], over[1], jump[0], jump[1]);
    WANT(over[0] > 0.2 && over[1] < 0.15 && jump[1] < jump[0],
         "glide: the ghost does not walk on unseen into the wall and back");
}

/* The HUD: an x-ray ghost turns seen and the track's pts become the wearer's own (guessed) view in
 * the same frame. The outgoing ghost of the crossfade is drawn as it was: the frame is the same,
 * pixel for pixel, as one where the pts did not change. */
static void check_crossfade(void)
{
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    enum { W = 640, H = 360 };
    static uint32_t px[2][W * H];
    int fading = 0, skel = 0;
    for (int run = 0; run < 2; run++) {
        static wv_view v;
        view_init(&v);
        canvas c = { px[run], W, H, W };
        static seeker_tracks t;
        memset(&t, 0, sizeof t);
        snprintf(t.src, sizeof t.src, "hud_fusion");
        t.n_nodes = 2;
        snprintf(t.nodes[0].node_id, sizeof t.nodes[0].node_id, "R1");
        snprintf(t.nodes[1].node_id, sizeof t.nodes[1].node_id, "R2");
        for (int n = 0; n < 2; n++) {
            snprintf(t.nodes[n].state, sizeof t.nodes[n].state, "ok");
            snprintf(t.nodes[n].pose_src, sizeof t.nodes[n].pose_src, "cli");
        }
        t.nodes[0].x_m = R1.x, t.nodes[0].y_m = R1.y;
        t.nodes[1].x_m = R2_NEAR.x, t.nodes[1].y_m = R2_NEAR.y, t.nodes[1].yaw_deg = R2_NEAR.yaw;
        snprintf(t.wearer_node, sizeof t.wearer_node, "R1");
        t.n = 1;
        seeker_track *k = &t.tracks[0];
        k->id = 7;
        snprintf(k->cls, sizeof k->cls, "person");
        snprintf(k->colour, sizeof k->colour, "cyan");
        k->has_room = k->has_pts = 1;
        k->room_x_m = 0.1f, k->room_y_m = 0.3f;
        k->posture = 1;
        for (int i = 0; i < 12; i++) {
            int seen = i == 11;
            snprintf(k->nodes, sizeof k->nodes, "%s", seen ? "R1+R2" : "R2");
            snprintf(k->node_id, sizeof k->node_id, "%s", seen ? "R1" : "R2");
            for (int j = 0; j < SEEKER_N_JOINTS; j++) {
                int splay = seen && run == 0 && (j % 2 == 0) && j >= 6; /* the wearer's guessed side */
                k->pts[j][0] = BODY[j][0] + (splay ? 0.3f : 0.0f);
                k->pts[j][1] = BODY[j][1] - (splay && j <= 10 ? 0.2f : 0.0f);
                k->pts[j][2] = 0.9f;
            }
            v.clock_ms = 1000 + 33 * i;
            wv_render(&c, &cfg, &v, &t, 0, NULL);
        }
        fading += v.last.n_fading;
        skel += v.last.n_skel;
    }
    int same = !memcmp(px[0], px[1], sizeof px[0]);
    printf("crossfade: x-ray to seen, outgoing ghost drawn %d of 2, frames %s\n", skel,
           same ? "identical" : "differ");
    WANT(fading == 2 && skel == 2, "crossfade: the outgoing ghost is drawn");
    WANT(same, "crossfade: the outgoing ghost keeps its last x-ray shape, not the new pts");
}

/* SEEKER_SKEL_CONFIRM: a one-message jump of a joint past SEEKER_SKEL_SNAP is never drawn; two
 * that agree restart it there. */
static void check_confirm(void)
{
    static seeker_skel s;
    memset(&s, 0, sizeof s);
    float raw[SEEKER_N_JOINTS][3], out[SEEKER_N_JOINTS][3];
    const float (*rp)[3] = (const float (*)[3])raw;
    for (int j = 0; j < SEEKER_N_JOINTS; j++)
        raw[j][0] = BODY[j][0], raw[j][1] = BODY[j][1], raw[j][2] = 0.9f;
    int64_t t = 1000;
    long seq = 1;
    const int fl = 1 | SEEKER_SKEL_CONFIRM;
    for (int i = 0; i < 5; i++)
        seeker_skel_step(&s, "R2", seq++, t += 66, rp, fl, out);
    raw[10][0] = BODY[10][0] + 0.8f; /* one wild frame */
    seeker_skel_step(&s, "R2", seq++, t += 66, rp, fl, out);
    float wild = out[10][0];
    raw[10][0] = BODY[10][0];
    seeker_skel_step(&s, "R2", seq++, t += 66, rp, fl, out);
    float back = out[10][0];
    raw[10][0] = BODY[10][0] + 0.8f; /* a real move: two frames agree */
    seeker_skel_step(&s, "R2", seq++, t += 66, rp, fl, out);
    float first = out[10][0];
    raw[10][0] = BODY[10][0] + 0.82f;
    seeker_skel_step(&s, "R2", seq++, t += 66, rp, fl, out);
    printf("confirm: wild frame drawn at %+.2f, after it %+.2f; a real move %+.2f then %+.2f (person heights)\n",
           wild - BODY[10][0], back - BODY[10][0], first - BODY[10][0], out[10][0] - BODY[10][0]);
    WANT(fabsf(wild - BODY[10][0]) < 0.02f && fabsf(back - BODY[10][0]) < 0.02f, "confirm: one wild frame is not drawn");
    WANT(fabsf(first - BODY[10][0]) < 0.02f && fabsf(out[10][0] - BODY[10][0] - 0.82f) < 0.02f,
         "confirm: a jump two frames agree on restarts there");
}

int main(int argc, char **argv)
{
    unsigned seed = argc > 1 ? (unsigned)atoi(argv[1]) : 11;
    draw_set_theme(DRAW_THEME_PLAN);
    anim_set_enabled(1);
    replay(seed);
    anim_set_enabled(0);
    check_hold();
    check_offframe();
    check_glide_origin();
    check_crossfade();
    check_confirm();
    printf("%s\n", fails ? "occlusion targets missed" : "all occlusion targets met");
    return fails ? 1 : 0;
}
