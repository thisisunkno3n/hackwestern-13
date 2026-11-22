/* Smoothness simulation: two synthetic phones (jittery detections, bursty and late delivery,
 * video stalls, occlusion behind a divider, a crouch) through the real hud_fusion core, the real
 * tracks datagram and the real EagleEye wearer-view renderer, on a simulated clock. Prints
 * smoothness metrics for the old settings and the current ones, and fails if the current ones
 * miss their targets.
 *
 *   sim_smooth [seconds]       (ctest "smoothness"; host and QNX alike, no sockets, no screen)
 *
 * Scene (room frame, tag 0 at the origin): R1, the fixed wearer, at 0,-2 facing the divider
 * (y = 0); R2 at -2.4,0.5 facing +x, behind it. Person A walks behind the divider at y = 1.0
 * (only R2 sees A: XRAY on the HUD), crouches from 9 to 12 s; person B stands at 0.6,-0.6 in
 * front of R1 (both phones see B). Phones report at 15 Hz with timing jitter, stall and then
 * burst; R1's video stalls for 1.2 s at 6 s and 3.5 s at 14 s.
 *
 * Metrics: A's box (XRAY, moving): interval between frames where it moves (p50/p95/max ms)
 * and jerk (RMS second difference, px/frame^2); B's box (SEEN, still): position jitter (px
 * std); mode flips (wearer view <-> full radar); label flips (A's posture after it settles,
 * B's colour); video age at each HUD frame (p50/p95/max ms). */

#include "hud_render.h"
#include "seeker_config.h"
#include "seeker_fusion.h"
#include "seeker_msgs.h"
#include "video_rx.h"
#include "wearer_view.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f

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

static const float STAND[SEEKER_N_JOINTS][2] = {
    { 0.00f, -0.43f }, { -0.02f, -0.45f }, { 0.02f, -0.45f }, { -0.05f, -0.44f }, { 0.05f, -0.44f },
    { -0.12f, -0.29f }, { 0.12f, -0.29f }, { -0.15f, -0.12f }, { 0.15f, -0.12f }, { -0.16f, 0.03f },
    { 0.16f, 0.03f }, { -0.08f, 0.00f }, { 0.08f, 0.00f }, { -0.08f, 0.25f }, { 0.08f, 0.25f },
    { -0.08f, 0.48f }, { 0.08f, 0.48f },
};
static const float CROUCH[SEEKER_N_JOINTS][2] = {
    { 0.00f, -0.36f }, { -0.02f, -0.38f }, { 0.02f, -0.38f }, { -0.05f, -0.37f }, { 0.05f, -0.37f },
    { -0.12f, -0.22f }, { 0.12f, -0.22f }, { -0.16f, -0.08f }, { 0.16f, -0.08f }, { -0.12f, 0.02f },
    { 0.12f, 0.02f }, { -0.09f, 0.00f }, { 0.09f, 0.00f }, { -0.18f, 0.03f }, { 0.18f, 0.03f },
    { -0.12f, 0.24f }, { 0.12f, 0.24f },
};

typedef struct {
    float x, y, yaw;
} pose3;
static const pose3 R1 = { 0.0f, -2.0f, 0.0f }, R2 = { -2.4f, 0.5f, 80.0f };

static void person_a(int64_t t, float *x, float *y, int *crouch)
{
    float s = (float)(t % 8000) / 1000.0f; /* 0.5 m/s across and back over 8 s */
    float u = s < 4.0f ? s : 8.0f - s;
    *x = -1.0f + 0.5f * u;
    *y = 1.0f;
    *crouch = t >= 9000 && t < 12000;
}

/* One phone message: node at p sees the people it can (A only from R2: the divider). */
static void make_msg(const char *node, pose3 p, int64_t t, seeker_remote_contacts *m)
{
    memset(m, 0, sizeof *m);
    snprintf(m->node_id, sizeof m->node_id, "%s", node);
    m->v = 2;
    m->t_mono_ms = t;
    m->width = 640;
    m->height = 480;
    m->hfov_deg = 66;
    float f = 320.0f / tanf(33.0f * DEG2RAD);
    float ax, ay;
    int crouch;
    person_a(t, &ax, &ay, &crouch);
    struct {
        float x, y;
        int crouch, seen;
        const char *id;
    } who[2] = { { ax, ay, crouch, !strcmp(node, "R2"), "p1" }, { 0.6f, -0.6f, 0, 1, "p2" } };
    for (int i = 0; i < 2; i++) {
        if (!who[i].seen)
            continue;
        float dx = who[i].x - p.x, dy = who[i].y - p.y, r = hypotf(dx, dy);
        float b = atan2f(dx, dy) * RAD2DEG - p.yaw;
        while (b > 180)
            b -= 360;
        while (b <= -180)
            b += 360;
        if (fabsf(b) > 31.0f)
            continue;
        seeker_contact *c = &m->c[m->n++];
        snprintf(c->id, sizeof c->id, "%s-%s", node, who[i].id);
        snprintf(c->cls, sizeof c->cls, "person");
        c->conf = 0.9f;
        c->bearing_deg = b + 0.7f * (float)gauss();
        c->has_range = 1;
        c->range_m = r * (1.0f + 0.08f * (float)gauss());
        float u = 320.0f + f * tanf(b * DEG2RAD), hpx = f * 1.7f / r;
        c->has_box = 1;
        c->box[0] = u - 0.18f * hpx + 3.0f * (float)gauss();
        c->box[1] = 240.0f - 0.5f * hpx + 3.0f * (float)gauss();
        c->box[2] = 0.36f * hpx;
        c->box[3] = hpx;
        if (c->box[1] < 0) { /* the phone's box spans visible landmarks: inside the frame */
            c->box[3] += c->box[1];
            c->box[1] = 0;
        }
        if (c->box[1] + c->box[3] > 480.0f)
            c->box[3] = 480.0f - c->box[1];
        const float (*tm)[2] = who[i].crouch ? CROUCH : STAND;
        for (int j = 0; j < SEEKER_N_JOINTS; j++) {
            c->pts[j][0] = (u + (tm[j][0] + 0.02f * (float)gauss()) * hpx) / 640.0f;
            c->pts[j][1] = (240.0f + (tm[j][1] + 0.02f * (float)gauss()) * hpx) / 480.0f;
            c->pts[j][2] = 0.9f;
        }
        c->has_pts = 1;
    }
}

/* Delivery: a few ms of jitter, plus stalls that hold messages and release them in a burst. */
static int64_t deliver(int64_t t, const int64_t (*stall)[2], int ns)
{
    for (int i = 0; i < ns; i++)
        if (t >= stall[i][0] && t < stall[i][1])
            return stall[i][1] + (int64_t)(5 * urand());
    return t + 12 + (int64_t)(10 * fabs(gauss()));
}

typedef struct {
    double p50, p95, max;
} pct;

static int cmp_d(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return x < y ? -1 : x > y;
}

static pct percentiles(double *v, int n)
{
    pct p = { 0, 0, 0 };
    if (n <= 0)
        return p;
    qsort(v, (size_t)n, sizeof *v, cmp_d);
    p.p50 = v[n / 2];
    p.p95 = v[(int)(0.95 * (n - 1))];
    p.max = v[n - 1];
    return p;
}

typedef struct {
    pct a_update, video_age;
    double a_jerk, b_jitter;
    int mode_flips, a_posture_flips, b_colour_flips, frames, a_frames;
} metrics;

enum { MAXF = 2000 };

static void run(int seconds, int improved, const char *cfg_path, metrics *out)
{
    static seeker_config cfg;
    seeker_config_load(&cfg, cfg_path);
    if (!improved)
        cfg.predict_ms = 0;
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    seeker_pose p1 = { R1.x, R1.y, R1.yaw }, p2 = { R2.x, R2.y, R2.yaw };
    seeker_fusion_set_pose(&f, "R1", &p1, "cli", 1);
    seeker_fusion_set_pose(&f, "R2", &p2, "cli", 1);

    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    snprintf(v.cfg.node, sizeof v.cfg.node, "R1");
    v.cfg.pose.x_m = R1.x;
    v.cfg.pose.y_m = R1.y;
    v.cfg.pose.yaw_deg = R1.yaw;
    v.cfg.cam.image_w = 640;
    v.cfg.cam.image_h = 480;
    v.cfg.radar_fallback = 1;
    v.cfg.inset = 0;
    v.box_ease_off = !improved;
    enum { W = 640, H = 360 };
    static uint32_t px[W * H], vid[W * H];
    canvas c = { px, W, H, W };
    video_frame vf;
    memset(&vf, 0, sizeof vf);
    vf.px = vid;
    vf.w = vf.stride = W;
    vf.h = H;
    vf.src_w = 640;
    vf.src_h = 480;
    vf.rect[0] = 80, vf.rect[2] = 480, vf.rect[3] = 360;
    snprintf(vf.node_id, sizeof vf.node_id, "R1");
    vf.t_rx_ms = -100000;
    video_stats vs;
    memset(&vs, 0, sizeof vs);
    vs.connected = 1;

    const int64_t end = (int64_t)seconds * 1000;
    static const int64_t st_r1[][2] = { { 8000, 8500 } }, st_r2[][2] = { { 5000, 5600 }, { 12000, 12800 } };
    static const int64_t st_vid[][2] = { { 6000, 7200 }, { 14000, 17500 } };
    /* Pending messages: generation every 66 ms with jitter, delivered later. */
    enum { MAXM = 4096 };
    static seeker_remote_contacts q[MAXM];
    static int64_t q_at[MAXM];
    int nq = 0;
    int64_t next_gen[2] = { 0, 33 }, next_vid = 5, vid_at = 0;
    static seeker_tracks tracks, latest;
    int have_tracks = 0;
    char buf[SEEKER_MAX_DATAGRAM], err[64];

    static double a_x[MAXF], a_t[MAXF], upd[MAXF], age[MAXF], b_x[MAXF];
    int na = 0, nupd = 0, nage = 0, nb = 0;
    int last_rendered = -1, last_a_post = -1, last_b_level = -1;
    double last_move_t = -1, prev_ax = 1e9;
    memset(out, 0, sizeof *out);

    for (int64_t now = 0; now < end; now++) {
        for (int n = 0; n < 2; n++)
            if (now >= next_gen[n] && nq < MAXM) {
                make_msg(n ? "R2" : "R1", n ? R2 : R1, now, &q[nq]);
                q_at[nq] = n ? deliver(now, st_r2, 2) : deliver(now, st_r1, 1);
                nq++;
                next_gen[n] = now + 66 + (int64_t)(6 * gauss());
            }
        if (now >= next_vid) {
            int64_t at = deliver(now, st_vid, 2);
            if (at <= now + 40) /* a stalled stream drops what it buffered */
                vid_at = at;
            next_vid = now + 66;
        }
        if (vid_at && now >= vid_at) {
            vf.t_rx_ms = vid_at;
            vid_at = 0;
        }
        for (int i = 0; i < nq; i++)
            if (q_at[i] == now)
                seeker_fusion_ingest(&f, &q[i], now);
        if (now % 33 == 0) {
            seeker_fusion_tick(&f, now, &tracks);
            int len = seeker_encode_tracks(&tracks, buf, sizeof buf);
            if (len > 0 && seeker_decode_tracks(buf, len, &latest, err, sizeof err) == 0)
                have_tracks = 1;
        }
        if (now % 33 != 11 || !have_tracks)
            continue;
        /* One HUD frame. */
        hud_state hs;
        memset(&hs, 0, sizeof hs);
        hs.t = &latest;
        hs.tracks_age_ms = 11;
        hs.background = "wearer";
        hs.view_node = "R1";
        hs.video = vf.t_rx_ms <= now ? &vf : NULL;
        hs.video_state = video_rx_state(hs.video, &vs, now);
        hs.video_why = video_rx_why(hs.video, &vs, now);
        hs.video_frozen = improved && video_rx_frozen(hs.video, &vs, now, VIDEO_FREEZE_MS);
        hs.wearer = &v;
        v.clock_ms = now + 1;
        hud_render(&c, &cfg, &hs);
        out->frames++;
        if (hs.video && vf.t_rx_ms > 0 && nage < MAXF)
            age[nage++] = (double)(now - vf.t_rx_ms);
        /* Flips once the picture has been up (the start, before any video, is not one). */
        if (last_rendered >= 0 && v.last.rendered != last_rendered)
            out->mode_flips++;
        if (last_rendered >= 0 || v.last.rendered)
            last_rendered = v.last.rendered;
        for (int i = 0; i < latest.n; i++) {
            const seeker_track *k = &latest.tracks[i];
            if (k->has_room && k->room_y_m > 0.2f) { /* A */
                if (k->posture != SEEKER_POSTURE_UNKNOWN) {
                    if (last_a_post >= 0 && k->posture != last_a_post)
                        out->a_posture_flips++;
                    last_a_post = k->posture;
                }
            }
        }
        if (!v.last.rendered)
            continue;
        for (int i = 0; i < v.last.n_drawn; i++) {
            double cx = v.last.drawn[i].x + v.last.drawn[i].w / 2.0;
            const seeker_track *k = NULL;
            for (int j = 0; j < latest.n; j++)
                if (latest.tracks[j].id == v.last.drawn[i].id)
                    k = &latest.tracks[j];
            if (!k || !k->has_room)
                continue;
            if (k->room_y_m > 0.2f) { /* A, moving behind the divider */
                if (now > 1500 && now < 8500 && na < MAXF) {
                    a_x[na] = cx;
                    a_t[na++] = (double)now;
                    if (fabs(cx - prev_ax) >= 1.0) {
                        if (last_move_t >= 0 && nupd < MAXF)
                            upd[nupd++] = now - last_move_t;
                        last_move_t = (double)now;
                    }
                    prev_ax = cx;
                }
            } else if (v.last.drawn[i].direct) { /* B, still, while R1 sees B */
                if (nb < MAXF)
                    b_x[nb++] = cx;
                if (last_b_level >= 0 && v.last.drawn[i].level != last_b_level)
                    out->b_colour_flips++;
                last_b_level = v.last.drawn[i].level;
            }
        }
    }
    out->a_frames = na;
    out->a_update = percentiles(upd, nupd);
    out->video_age = percentiles(age, nage);
    double jerk = 0;
    int nj = 0;
    for (int i = 1; i + 1 < na; i++)
        if (a_t[i + 1] - a_t[i - 1] < 80) { /* consecutive frames */
            double d2 = a_x[i + 1] - 2 * a_x[i] + a_x[i - 1];
            jerk += d2 * d2;
            nj++;
        }
    out->a_jerk = nj ? sqrt(jerk / nj) : 0;
    double mean = 0, var = 0;
    for (int i = 0; i < nb; i++)
        mean += b_x[i];
    mean = nb ? mean / nb : 0;
    for (int i = 0; i < nb; i++)
        var += (b_x[i] - mean) * (b_x[i] - mean);
    out->b_jitter = nb ? sqrt(var / nb) : 0;
}

/* ---- moving wearer: R1 walks and looks around, tag 0 drops out for 1, 5 and 20 s ---- */

static void r1_truth(int64_t t, pose3 *p)
{
    float s = (float)t / 1000.0f;
    p->x = sinf(s * 0.25f);            /* +-1 m along the divider */
    p->y = -2.5f + 0.3f * sinf(s * 0.11f);
    p->yaw = 20.0f * sinf(s * 0.4f);   /* looking left and right */
}

static const int64_t DROP[3][2] = { { 5000, 6000 }, { 10000, 15000 }, { 18000, 38000 } };

static int tag_visible(int64_t t)
{
    for (int i = 0; i < 3; i++)
        if (t >= DROP[i][0] && t < DROP[i][1])
            return 0;
    return 1;
}

typedef struct {
    pct pos_err[3], brg_err[3], yaw_err[3]; /* per dropout */
    double jump_px[3];          /* biggest frame-to-frame jump of the x-ray box after each */
    double max_jump_px;
} walk_metrics;

static void run_walk(int improved, const char *cfg_path, walk_metrics *out)
{
    static seeker_config cfg;
    seeker_config_load(&cfg, cfg_path);
    if (!improved)
        cfg.pose_home_ms = 0;
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    f.pose_blend = improved;
    seeker_pose home = { 0.0f, -2.5f, 0.0f }, p2 = { R2.x, R2.y, R2.yaw };
    seeker_fusion_set_pose(&f, "R1", &home, "init", 0); /* WEARER_POSE, not pinned */
    seeker_fusion_set_pose(&f, "R2", &p2, "cli", 1);
    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    snprintf(v.cfg.node, sizeof v.cfg.node, "R1");
    v.cfg.pose_src = WV_POSE_AUTO;
    v.cfg.cam.image_w = 640;
    v.cfg.cam.image_h = 480;
    v.cfg.inset = 0;
    v.cfg.radar_fallback = 0;
    enum { W = 640, H = 360 };
    static uint32_t px[W * H];
    canvas c = { px, W, H, W };
    static seeker_tracks tracks, latest;
    char buf[SEEKER_MAX_DATAGRAM], err[64];
    const float bx = 0.3f, by = 1.0f; /* the hidden person, seen by R2 only */
    static double pe[3][MAXF], be[3][MAXF], ye[3][MAXF];
    int np[3] = { 0, 0, 0 };
    double prev_x = -1;
    memset(out, 0, sizeof *out);
    int64_t next[2] = { 0, 33 };
    for (int64_t now = 0; now < 40000; now++) {
        if (now >= next[0]) { /* R1: tag pose or null, and (after) its compass heading */
            seeker_remote_contacts m;
            memset(&m, 0, sizeof m);
            snprintf(m.node_id, sizeof m.node_id, "R1");
            m.v = 2;
            m.width = 640, m.height = 480, m.hfov_deg = 66;
            pose3 tr;
            r1_truth(now, &tr);
            if (tag_visible(now)) {
                m.has_pose = m.has_xy = m.has_yaw = 1;
                m.x_m = tr.x + 0.03f * (float)gauss();
                m.y_m = tr.y + 0.03f * (float)gauss();
                m.yaw_deg = tr.yaw + 1.5f * (float)gauss();
            }
            if (improved) { /* compass: room yaw minus a 25 deg magnetic offset, noisy */
                m.has_heading = 1;
                m.heading_deg = tr.yaw - 25.0f + 2.0f * (float)gauss();
            }
            seeker_fusion_ingest(&f, &m, now + 15);
            next[0] = now + 66;
        }
        if (now >= next[1]) {
            seeker_remote_contacts m = { 0 };
            pose3 r2 = R2;
            snprintf(m.node_id, sizeof m.node_id, "R2");
            m.v = 2;
            m.width = 640, m.height = 480, m.hfov_deg = 66, m.n = 1;
            float dx = bx - r2.x, dy = by - r2.y;
            m.c[0].bearing_deg = atan2f(dx, dy) * RAD2DEG - r2.yaw + 0.5f * (float)gauss();
            m.c[0].has_range = 1;
            m.c[0].range_m = hypotf(dx, dy) * (1.0f + 0.03f * (float)gauss());
            m.c[0].conf = 0.9f;
            snprintf(m.c[0].id, sizeof m.c[0].id, "R2-p1");
            seeker_fusion_ingest(&f, &m, now + 15);
            next[1] = now + 66;
        }
        if (now % 33 == 0) {
            seeker_fusion_tick(&f, now, &tracks);
            int len = seeker_encode_tracks(&tracks, buf, sizeof buf);
            if (len > 0)
                seeker_decode_tracks(buf, len, &latest, err, sizeof err);
        }
        if (now % 33 != 11 || now < 1000)
            continue;
        v.clock_ms = now + 1;
        wv_render(&c, &cfg, &v, &latest, 0, NULL);
        pose3 tr;
        r1_truth(now, &tr);
        const seeker_node_status *e = NULL;
        for (int i = 0; i < latest.n_nodes; i++)
            if (!strcmp(latest.nodes[i].node_id, "R1"))
                e = &latest.nodes[i];
        if (!e)
            continue;
        float true_b = atan2f(bx - tr.x, by - tr.y) * RAD2DEG - tr.yaw;
        float est_b = atan2f(bx - e->x_m, by - e->y_m) * RAD2DEG - e->yaw_deg;
        double berr = fabs(fmod(est_b - true_b + 540.0, 360.0) - 180.0);
        double perr = hypot(e->x_m - tr.x, e->y_m - tr.y);
        double cx = -1;
        for (int i = 0; i < v.last.n_drawn; i++)
            if (!v.last.drawn[i].direct)
                cx = v.last.drawn[i].x + v.last.drawn[i].w / 2.0;
        double jump = cx >= 0 && prev_x >= 0 ? fabs(cx - prev_x) : 0;
        prev_x = cx;
        if (jump > out->max_jump_px)
            out->max_jump_px = jump;
        for (int d = 0; d < 3; d++) {
            if (now >= DROP[d][0] && now < DROP[d][1] && np[d] < MAXF) {
                pe[d][np[d]] = perr;
                ye[d][np[d]] = fabs(fmod(e->yaw_deg - tr.yaw + 540.0, 360.0) - 180.0);
                be[d][np[d]++] = berr;
            }
            if (now >= DROP[d][1] && now < DROP[d][1] + 1000 && jump > out->jump_px[d])
                out->jump_px[d] = jump; /* the tag is back */
        }
    }
    for (int d = 0; d < 3; d++) {
        out->pos_err[d] = percentiles(pe[d], np[d]);
        out->yaw_err[d] = percentiles(ye[d], np[d]);
        out->brg_err[d] = percentiles(be[d], np[d]);
    }
}

static void print_walk(const char *name, const walk_metrics *m)
{
    static const char *const what[3] = { "1 s", "5 s", "20 s" };
    for (int d = 0; d < 3; d++)
        printf("%-7s tag gone %-4s: R1 position err p50 %.2f p95 %.2f m, yaw err p50 %4.1f p95 "
               "%4.1f deg; hidden person's bearing err p50 %4.1f p95 %4.1f max %4.1f deg; box "
               "jump when the tag is back %5.1f px\n",
               name, what[d], m->pos_err[d].p50, m->pos_err[d].p95, m->yaw_err[d].p50,
               m->yaw_err[d].p95, m->brg_err[d].p50, m->brg_err[d].p95, m->brg_err[d].max,
               m->jump_px[d]);
}

static void print(const char *name, const metrics *m)
{
    printf("%-7s A moves every p50 %3.0f / p95 %3.0f / max %4.0f ms, jerk %5.2f px/f2 | "
           "B jitter %4.2f px | mode flips %d | A posture flips %d, B colour flips %d | "
           "video age p50 %3.0f p95 %4.0f max %5.0f ms (%d frames)\n",
           name, m->a_update.p50, m->a_update.p95, m->a_update.max, m->a_jerk, m->b_jitter,
           m->mode_flips, m->a_posture_flips, m->b_colour_flips, m->video_age.p50,
           m->video_age.p95, m->video_age.max, m->frames);
}

int main(int argc, char **argv)
{
    int seconds = argc > 1 ? atoi(argv[1]) : 20;
    if (seconds < 18)
        seconds = 18;
    char cfg_path[512] = "config/seeker.live.json";
#ifdef SEEKER_SOURCE_DIR
    snprintf(cfg_path, sizeof cfg_path, "%s/config/seeker.live.json", SEEKER_SOURCE_DIR);
    if (access(cfg_path, R_OK) != 0)
        snprintf(cfg_path, sizeof cfg_path, "config/seeker.live.json");
#endif
    metrics before, after;
    g_seed = 7;
    run(seconds, 0, cfg_path, &before);
    g_seed = 7;
    run(seconds, 1, cfg_path, &after);
    printf("smoothness sim, %d s, same seed (before = main: no prediction, no box easing, radar "
           "after 2 s):\n", seconds);
    print("before", &before);
    print("after", &after);
    int fails = 0;
#define WANT(c, what)                                                                              \
    do {                                                                                           \
        if (!(c)) {                                                                                \
            printf("FAIL %s\n", what);                                                             \
            fails++;                                                                               \
        }                                                                                          \
    } while (0)
    WANT(after.a_frames > 100, "the x-ray person was drawn");
    WANT(after.a_update.p50 <= 40, "A moves on every HUD frame (p50 <= 40 ms)");
    WANT(after.a_update.p50 < before.a_update.p50, "A moves more often than before");
    WANT(after.a_jerk < before.a_jerk, "A's motion is smoother (less jerk) than before");
    WANT(after.mode_flips == 0, "no wearer/radar flips through 1.2 s and 3.5 s video stalls");
    WANT(after.a_posture_flips == 2, "A's posture changes exactly twice (crouch, stand)");
    WANT(after.b_colour_flips == 0, "B's colour never flips");
    WANT(after.b_jitter < before.b_jitter, "B's box shakes less than before");
    walk_metrics wb, wa;
    g_seed = 11;
    run_walk(0, cfg_path, &wb);
    g_seed = 11;
    run_walk(1, cfg_path, &wa);
    printf("moving wearer, 40 s, tag 0 gone for 1 s at 5 s, 5 s at 10 s, 20 s at 18 s (before = "
           "main: hold, jump back, no compass; after = compass yaw, blend back, home after "
           "10 s):\n");
    print_walk("before", &wb);
    print_walk("after", &wa);
    WANT(wa.yaw_err[1].p95 < wb.yaw_err[1].p95 && wa.yaw_err[2].p95 < wb.yaw_err[2].p95,
         "the compass keeps the yaw through 5 s and 20 s dropouts");
    WANT(wa.brg_err[2].p95 < wb.brg_err[2].p95, "20 s dropout: smaller bearing error than before");
    WANT(wa.pos_err[2].p50 < wb.pos_err[2].p50, "20 s dropout: home is closer than a stale hold");
    WANT(wa.jump_px[1] < wb.jump_px[1] && wa.jump_px[2] < wb.jump_px[2],
         "smaller box jumps when the tag comes back");
    printf(fails ? "%d target(s) missed\n" : "all smoothness targets met\n", fails);
    return fails ? 1 : 0;
}
