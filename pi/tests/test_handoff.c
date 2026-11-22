/* Handoff replay: an x-ray person (R2 only, behind the divider) whom R1 then sees, through the
 * real hud_fusion core, the real tracks datagram and the real wearer view (plan theme), on a
 * simulated clock. Each scene runs twice on the same seed: before (hud_fusion --no-handoff,
 * hud_display --handoff off) and after (the defaults). Fails if the defaults miss their targets.
 *
 *   test_handoff [seed]     (ctest "handoff"; host and QNX alike, no sockets, no screen)
 *
 * Room (tag 0 frame, Morrissette setup): R1, the wearer, pinned at 0,-2 facing the divider; R2
 * pinned at -1.59,-1.4 yaw 49; the divider is the wall -0.2,-0.6 to 1.4,-0.6. Phones report at
 * 15 Hz, 20 to 40 ms late, bearings +-0.7 deg; R2's box-height range reads 25% long, R1's 15%
 * short (both +-4%); R1 misses 12% of its frames (its confidence cut).
 *
 *   step-out        the person stands behind the divider (R2 only), walks out at 0.84 m/s into
 *                   R1's view at about 2.6 s and stops in front of it; R2 keeps seeing them.
 *   step-out, R2 lost  the same walk, but R2 loses the person as R1 picks them up.
 *   step-out, far R2   the same walk seen by an R2 at -4.2,-3.8 yaw 45 (about 6 m away) whose
 *                   range reads 30% long: the fused fix lands 1.9 m from the x-ray one.
 *   hand-out        the person stays behind the divider edge (R2 sees all of them); R1 sees only
 *                   a head, a shoulder and an arm in 70% of its frames (hips guessed by the pose
 *                   model, box-height range about 2x long).
 *   partial alone   R1 sees only such a part and nobody else sees anyone.
 *   crouch          the person stands at -0.7,0.5 in both phones' view and crouches at 1.5 s
 *                   (0.4 s down, then stays down). The phones range as phone.html does, from
 *                   eye-to-ankle / 0.90: crouched that reads 1.75x long, so R2's range is 2.2x
 *                   the truth. Before is current main's defaults with --no-crouch-range.
 *   walk-in, R2 part first  the reverse of step-out, in a room where R2 covers only the space
 *                   behind the divider: the person walks from R1's view behind it (R1 loses
 *                   them at the edge), and for its first 500 ms R2 sees only a head, a shoulder
 *                   and an arm (box-height range about 2x long), then the whole person. Before
 *                   is the defaults with hud_fusion --no-part-keep.
 *
 * Metrics: track ids the person had, ticks with two tracks (one person shown twice), frames
 * drawn as seen, look changes (seen <-> x-ray) after R1 first saw them, crossfade frames,
 * the largest step of the sent position in one 33 ms tick, and the sent position's error
 * against the truth from 400 ms after R1 first saw them, past the 350 ms glide (p50 / p95).
 * The seen frame against R1's own detection box (the one in R1's picture), in box heights: how
 * far its centre is off on the first frame R1 sees an x-ray person, and how far the frame of a
 * person held as seen (R1 missed them) is from R1's last box, in place or in height.
 * Walk-in, for 1.5 s from R2's first sighting: the farthest the sent position is from the person,
 * its largest step in one tick, and the tallest ghost drawn (joints' extent, person heights). */

#include "anim.h"
#include "draw.h"
#include "seeker_config.h"
#include "seeker_fusion.h"
#include "seeker_msgs.h"
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

/* COCO-17, person-height units from the mid-hip (x right, y down): standing, right arm a little
 * out (a hand round the divider's edge). */
static const float BODY[SEEKER_N_JOINTS][2] = {
    { 0.00f, -0.43f }, { -0.02f, -0.45f }, { 0.02f, -0.45f }, { -0.05f, -0.44f }, { 0.05f, -0.44f },
    { -0.12f, -0.29f }, { 0.12f, -0.29f }, { -0.15f, -0.12f }, { 0.16f, -0.13f }, { -0.16f, 0.03f },
    { 0.22f, 0.00f }, { -0.08f, 0.00f }, { 0.08f, 0.00f }, { -0.08f, 0.25f }, { 0.08f, 0.25f },
    { -0.08f, 0.48f }, { 0.08f, 0.48f },
};
/* The same person in a deep crouch facing the camera: torso leaning in, thighs toward the camera
 * (knees out, about level with the hips), hands on the knees. Eye to ankle 0.53 of the standing
 * height (standing: 0.93). */
static const float CROUCH_BODY[SEEKER_N_JOINTS][2] = {
    { 0.00f, -0.31f }, { -0.02f, -0.33f }, { 0.02f, -0.33f }, { -0.05f, -0.32f }, { 0.05f, -0.32f },
    { -0.12f, -0.21f }, { 0.12f, -0.21f }, { -0.17f, -0.08f }, { 0.17f, -0.08f }, { -0.14f, 0.03f },
    { 0.14f, 0.03f }, { -0.08f, 0.00f }, { 0.08f, 0.00f }, { -0.16f, 0.03f }, { 0.16f, 0.03f },
    { -0.10f, 0.20f }, { 0.10f, 0.20f },
};
/* What R1 sees of someone leaning out at the divider's edge: head, right shoulder and arm; the
 * pose model guesses the hips (just over phone.html's 0.6 cut), the rest is hidden. */
static const float PART_VIS[SEEKER_N_JOINTS] = { 0.9f, 0.9f, 0.9f, 0.2f, 0.8f, 0.2f, 0.9f, 0.1f, 0.9f,
                                                 0.1f, 0.9f, 0.65f, 0.65f, 0.1f, 0.1f, 0.1f, 0.1f };

typedef struct {
    float x, y, yaw;
} pose3;
static const pose3 R1 = { 0.0f, -2.0f, 0.0f }, R2_NEAR = { -1.59f, -1.4f, 49.0f },
                   R2_FAR = { -4.2f, -3.8f, 45.0f };
static const float WALL[4] = { -0.2f, -0.6f, 1.4f, -0.6f };

enum { STEP_OUT = 0, STEP_OUT_R2_LOST, STEP_OUT_FAR, HAND_OUT, PARTIAL_ALONE, CROUCH, WALK_IN, N_SCENES };
static const char *const SCENE[] = { "step-out", "step-out, R2 lost", "step-out, far R2", "hand-out",
                                     "partial alone", "crouch", "walk-in, R2 part first" };
#define WALK_IN_PART_MS 500 /* walk-in: R2 sees only part of the person this long */
static int64_t g_r2_since;  /* walk-in: when the person came into R2's view (-1: not yet) */
#define CROUCH_T0 1500 /* the crouch starts, and is down CROUCH_DOWN_MS later */
#define CROUCH_DOWN_MS 400

/* How far into the crouch the person is at t, 0 standing to 1 down. */
static float crouch_at(int scene, int64_t t)
{
    if (scene != CROUCH)
        return 0;
    float s = (float)(t - CROUCH_T0) / (float)CROUCH_DOWN_MS;
    return s < 0 ? 0 : s > 1 ? 1 : s;
}

/* phone.html's person height for a body (all joints seen): eye midpoint to ankle midpoint / 0.90. */
static float phone_height(const float (*b)[2])
{
    return hypotf(0.5f * (b[1][0] + b[2][0] - b[15][0] - b[16][0]),
                  0.5f * (b[1][1] + b[2][1] - b[15][1] - b[16][1])) / 0.90f;
}

static float orient(float ax, float ay, float bx, float by, float cx, float cy)
{
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

static int behind_wall(pose3 p, float x, float y)
{
    return orient(p.x, p.y, x, y, WALL[0], WALL[1]) * orient(p.x, p.y, x, y, WALL[2], WALL[3]) < 0 &&
           orient(WALL[0], WALL[1], WALL[2], WALL[3], p.x, p.y) *
                   orient(WALL[0], WALL[1], WALL[2], WALL[3], x, y) <
               0;
}

static void truth(int scene, int64_t t, float *x, float *y)
{
    if (scene == HAND_OUT || scene == PARTIAL_ALONE) {
        *x = -0.05f;
        *y = 0.0f;
        return;
    }
    if (scene == CROUCH) {
        *x = -0.7f;
        *y = 0.5f;
        return;
    }
    float s = (float)(t - 1500) / 1500.0f;
    s = s < 0 ? 0 : s > 1 ? 1 : s;
    if (scene == WALK_IN)
        s = 1.0f - s; /* the same path, from R1's view to behind the divider */
    *x = 0.6f - 1.2f * s;
    *y = 0.2f - 0.4f * s;
}

/* Who sees the person whole (2), in part (1) or not (0) at time t. */
static int sees(int scene, const char *node, int64_t t, float x, float y)
{
    int r1 = !strcmp(node, "R1");
    if (scene == PARTIAL_ALONE)
        return r1 && t >= 500 && urand() < 0.7 ? 1 : 0;
    if (scene == HAND_OUT)
        return r1 ? (t >= 1000 && urand() < 0.7 ? 1 : 0) : 2;
    int r1_view = !behind_wall(R1, x, y);
    if (scene == WALK_IN && !r1) { /* R2 covers only the space behind the divider */
        if (r1_view)
            return 0;
        if (g_r2_since < 0)
            g_r2_since = t;
        return t - g_r2_since < WALK_IN_PART_MS ? 1 : 2;
    }
    if (r1)
        return r1_view && urand() >= 0.12 ? 2 : 0;
    return scene == STEP_OUT_R2_LOST && r1_view ? 0 : 2;
}

static void make_msg(int scene, const char *node, pose3 p, int64_t t, seeker_remote_contacts *m)
{
    memset(m, 0, sizeof *m);
    snprintf(m->node_id, sizeof m->node_id, "%s", node);
    m->v = 2;
    m->t_mono_ms = t + (!strcmp(node, "R1") ? 50000 : 90000); /* each phone has its own clock */
    m->width = 640;
    m->height = 480;
    m->hfov_deg = 66;
    float x, y;
    truth(scene, t, &x, &y);
    int how = sees(scene, node, t, x, y);
    float dx = x - p.x, dy = y - p.y, r = hypotf(dx, dy);
    float b = atan2f(dx, dy) * RAD2DEG - p.yaw;
    while (b > 180)
        b -= 360;
    while (b <= -180)
        b += 360;
    if (!how || fabsf(b) > 31.0f)
        return;
    float f = 320.0f / tanf(33.0f * DEG2RAD), hpx = f * 1.7f / r;
    float u = 320.0f + f * tanf(b * DEG2RAD);
    seeker_contact *c = &m->c[m->n++];
    snprintf(c->id, sizeof c->id, "%s-p1", node);
    snprintf(c->cls, sizeof c->cls, "person");
    float body[SEEKER_N_JOINTS][2], s = crouch_at(scene, t);
    for (int j = 0; j < SEEKER_N_JOINTS; j++)
        for (int a = 0; a < 2; a++)
            body[j][a] = BODY[j][a] + s * (CROUCH_BODY[j][a] - BODY[j][a]);
    /* the feet stay on the floor: the hips drop as the person goes down */
    float hip_v = 240.0f + (0.5f * (BODY[15][1] + BODY[16][1]) - 0.5f * (body[15][1] + body[16][1])) * hpx;
    float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        c->pts[j][0] = (u + (body[j][0] + 0.01f * (float)gauss()) * hpx) / 640.0f;
        c->pts[j][1] = (hip_v + (body[j][1] + 0.01f * (float)gauss()) * hpx) / 480.0f;
        c->pts[j][2] = how == 2 ? 0.9f : PART_VIS[j];
        if (c->pts[j][2] >= 0.3f) {
            x0 = fminf(x0, c->pts[j][0] * 640.0f), x1 = fmaxf(x1, c->pts[j][0] * 640.0f);
            y0 = fminf(y0, c->pts[j][1] * 480.0f), y1 = fmaxf(y1, c->pts[j][1] * 480.0f);
        }
    }
    c->has_pts = 1;
    c->has_box = 1;
    c->box[0] = x0, c->box[1] = y0, c->box[2] = x1 - x0, c->box[3] = y1 - y0;
    c->conf = how == 2 ? 0.9f : 0.65f;
    c->has_range = 1;
    if (how == 2) {
        float bias = strcmp(node, "R2") ? -0.15f : scene == STEP_OUT_FAR ? 0.30f : 0.25f;
        c->bearing_deg = b + 0.7f * (float)gauss();
        c->range_m = r * (1.0f + bias + 0.04f * (float)gauss());
        if (s > 0) /* phone.html's eye-to-ankle height shrinks with the crouch */
            c->range_m *= phone_height(BODY) / phone_height((const float (*)[2])body);
    } else { /* phone.html: hip midpoint bearing (a guess, 1.5 deg off), box-height range */
        c->bearing_deg = b + 1.5f + 0.7f * (float)gauss();
        c->range_m = f * 1.7f / c->box[3];
    }
}

typedef struct {
    int ids, two_ticks, seen_frames, look_flips, fade_frames, tracks_partial_alone;
    double max_step, err_p50, err_p95;
    /* crouch scene, from CROUCH_SETTLED_MS: ticks, ticks with two tracks, ticks the one track is
     * CROUCH, frames, x-ray people drawn, and the farthest any track is from the person (m) and
     * to the right of them as R1 sees it (m, R1 faces +y) */
    int c_ticks, c_two, c_crouch, c_frames, c_xray;
    double c_far, c_right;
    /* the seen frame against R1's own box, in box heights: its centre on the first frame after an
     * x-ray look, and a held frame's centre or height against R1's last box; held frames */
    double first_off, held_off;
    int held_frames;
    /* walk-in, for WALK_IN_WINDOW_MS from R2's first sighting: farthest the sent position is from
     * the person and its largest step in a tick (m), ticks with two tracks, tallest ghost drawn */
    double w_err, w_step, w_tall;
    int w_two;
} metrics;
#define WALK_IN_WINDOW_MS 1500
#define CROUCH_SETTLED_MS (CROUCH_T0 + CROUCH_DOWN_MS + 400)

static int cmp_d(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return x < y ? -1 : x > y;
}

/* R1's image px to the 640x360 canvas (a 640x480 picture, pillarboxed). */
static float canvas_x(float u)
{
    return 80.0f + 0.75f * u;
}

static void run(int scene, int handoff, int crouch_range, int part_keep, const char *cfg_path, metrics *out)
{
    static seeker_config cfg;
    seeker_config_load(&cfg, cfg_path);
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    f.handoff = handoff;
    f.crouch_range = crouch_range;
    f.part_keep = part_keep;
    g_r2_since = -1;
    const pose3 R2 = scene == STEP_OUT_FAR ? R2_FAR : R2_NEAR;
    seeker_pose p1 = { R1.x, R1.y, R1.yaw }, p2 = { R2.x, R2.y, R2.yaw };
    seeker_fusion_set_pose(&f, "R1", &p1, "cli", 1);
    seeker_fusion_set_pose(&f, "R2", &p2, "cli", 1);

    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    snprintf(v.cfg.node, sizeof v.cfg.node, "R1");
    v.cfg.pose.x_m = R1.x, v.cfg.pose.y_m = R1.y, v.cfg.pose.yaw_deg = R1.yaw;
    v.cfg.cam.image_w = 640, v.cfg.cam.image_h = 480;
    v.cfg.inset = 0;
    v.cfg.n_walls = 1;
    memcpy(v.cfg.walls[0], WALL, sizeof WALL);
    v.cfg.handoff = handoff;
    enum { W = 640, H = 360 };
    static uint32_t px[W * H];
    canvas c = { px, W, H, W };

    enum { MAXM = 512, MAXE = 400 };
    static seeker_remote_contacts q[MAXM];
    static int64_t q_at[MAXM];
    int nq = 0;
    int64_t next_gen[2] = { 0, 33 };
    static seeker_tracks tracks, latest;
    char buf[SEEKER_MAX_DATAGRAM], err[64];
    int ids[8], nids = 0, have = 0, last_look = -1;
    int64_t r1_first = -1, r2_first = -1;
    float last_x = 0, last_y = 0;
    int have_last = 0;
    int prev_look = -1, have_own = 0; /* the look last drawn (0 x-ray, 1 seen); R1's last box */
    float own_cx = 0, own_h = 0;      /* (canvas px): its centre x and its height */
    static double errs[MAXE];
    int nerr = 0;
    memset(out, 0, sizeof *out);
    const int64_t end = scene <= STEP_OUT_FAR || scene == CROUCH || scene == WALK_IN ? 5000 : 4000;

    for (int64_t now = 0; now < end; now++) {
        for (int n = 0; n < 2; n++)
            if (now >= next_gen[n] && nq < MAXM) {
                make_msg(scene, n ? "R2" : "R1", n ? R2 : R1, now, &q[nq]);
                if (!n && q[nq].n && r1_first < 0)
                    r1_first = now;
                if (n && q[nq].n && r2_first < 0)
                    r2_first = now;
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
            if (scene == PARTIAL_ALONE)
                out->tracks_partial_alone += latest.n;
            out->two_ticks += latest.n > 1;
            for (int i = 0; i < latest.n; i++) {
                const seeker_track *k = &latest.tracks[i];
                int known = 0;
                for (int j = 0; j < nids; j++)
                    known |= ids[j] == k->id;
                if (!known && nids < 8)
                    ids[nids++] = k->id;
            }
            if (scene == CROUCH && now >= CROUCH_SETTLED_MS) {
                float tx, ty;
                truth(scene, now, &tx, &ty);
                out->c_ticks++;
                out->c_two += latest.n > 1;
                out->c_crouch += latest.n == 1 && latest.tracks[0].posture == SEEKER_POSTURE_CROUCH;
                for (int i = 0; i < latest.n; i++)
                    if (latest.tracks[i].has_room) {
                        const seeker_track *k = &latest.tracks[i];
                        out->c_far = fmax(out->c_far, hypotf(k->room_x_m - tx, k->room_y_m - ty));
                        out->c_right = fmax(out->c_right, k->room_x_m - tx);
                    }
            }
            int walk_in = scene == WALK_IN && r2_first >= 0 && now >= r2_first && now <= r2_first + WALK_IN_WINDOW_MS;
            out->w_two += walk_in && latest.n > 1;
            if (latest.n == 1 && latest.tracks[0].has_room) {
                const seeker_track *k = &latest.tracks[0];
                if (walk_in) {
                    float tx, ty;
                    truth(scene, now, &tx, &ty);
                    out->w_err = fmax(out->w_err, hypotf(k->room_x_m - tx, k->room_y_m - ty));
                    if (have_last)
                        out->w_step = fmax(out->w_step, hypotf(k->room_x_m - last_x, k->room_y_m - last_y));
                }
                if (have_last)
                    out->max_step = fmax(out->max_step, hypotf(k->room_x_m - last_x, k->room_y_m - last_y));
                last_x = k->room_x_m, last_y = k->room_y_m, have_last = 1;
                float tx, ty;
                truth(scene, now, &tx, &ty);
                int64_t from = scene == CROUCH ? CROUCH_SETTLED_MS : r1_first + 400;
                if (r1_first >= 0 && now >= from && nerr < MAXE)
                    errs[nerr++] = hypotf(k->room_x_m - tx, k->room_y_m - ty);
            } else {
                have_last = 0;
            }
        }
        if (now % 33 != 11 || !have)
            continue;
        v.clock_ms = now + 1;
        wv_render(&c, &cfg, &v, &latest, 0, NULL);
        for (int i = 0; i < v.last.n_drawn; i++)
            out->seen_frames += v.last.drawn[i].direct;
        out->fade_frames += v.last.n_fading > 0;
        if (scene == CROUCH && now >= CROUCH_SETTLED_MS) {
            out->c_frames++;
            for (int i = 0; i < v.last.n_drawn; i++)
                out->c_xray += !v.last.drawn[i].direct;
        }
        if (v.last.n_drawn == 1 && r1_first >= 0) {
            int look = v.last.drawn[0].direct;
            if (last_look >= 0 && look != last_look)
                out->look_flips++;
            last_look = look;
        }
        /* The seen frame against the box R1 itself measured (the track's node_box when R1 is its
         * primary node: what the display had this frame). */
        if (v.last.n_drawn == 1 && latest.n == 1) {
            const seeker_track *k = &latest.tracks[0];
            float cx = (float)v.last.drawn[0].x + 0.5f * (float)v.last.drawn[0].w;
            int own = !strcmp(k->node_id, "R1") && k->has_node_box && k->node_box[3] > 0;
            if (own) {
                own_cx = canvas_x(k->node_box[0] + 0.5f * k->node_box[2]);
                own_h = 0.75f * k->node_box[3];
                have_own = 1;
                if (v.last.drawn[0].direct && prev_look == 0)
                    out->first_off = fmax(out->first_off, fabsf(cx - own_cx) / own_h);
            } else if (v.last.drawn[0].direct && v.last.n_held > 0 && have_own) {
                out->held_frames++;
                out->held_off = fmax(out->held_off, fmaxf(fabsf(cx - own_cx), fabsf((float)v.last.drawn[0].h - own_h)) / own_h);
            }
            if (scene == WALK_IN && !v.last.drawn[0].direct && k->has_pts && r2_first >= 0 &&
                now <= r2_first + WALK_IN_WINDOW_MS) {
                float lo = 1e9f, hi = -1e9f;
                for (int j = 0; j < SEEKER_N_JOINTS; j++)
                    if (k->pts[j][2] >= 0.3f)
                        lo = fminf(lo, k->pts[j][1]), hi = fmaxf(hi, k->pts[j][1]);
                if (hi > lo)
                    out->w_tall = fmax(out->w_tall, hi - lo);
            }
        }
        prev_look = v.last.n_drawn == 1 ? v.last.drawn[0].direct : -1;
    }
    out->ids = nids;
    if (nerr > 0) {
        qsort(errs, (size_t)nerr, sizeof errs[0], cmp_d);
        out->err_p50 = errs[nerr / 2];
        out->err_p95 = errs[(int)(0.95 * (nerr - 1))];
    }
}

static void print(const char *tag, const metrics *m)
{
    printf("  %-6s ids %d, two-track ticks %3d, seen frames %3d, look changes %2d, crossfade frames %2d, "
           "max step %.2f m, error p50 %.2f p95 %.2f m; seen frame off R1's box: first %.2f, held %.2f "
           "box heights (%d held)\n",
           tag, m->ids, m->two_ticks, m->seen_frames, m->look_flips, m->fade_frames, m->max_step,
           m->err_p50, m->err_p95, m->first_off, m->held_off, m->held_frames);
}

int main(int argc, char **argv)
{
    unsigned seed = argc > 1 ? (unsigned)atoi(argv[1]) : 11;
    char cfg_path[512] = "config/seeker.live.json";
#ifdef SEEKER_SOURCE_DIR
    snprintf(cfg_path, sizeof cfg_path, "%s/config/seeker.live.json", SEEKER_SOURCE_DIR);
    if (access(cfg_path, R_OK) != 0)
        snprintf(cfg_path, sizeof cfg_path, "config/seeker.live.json");
#endif
    draw_set_theme(DRAW_THEME_PLAN);
    int fails = 0, flips[2] = { 0, 0 }, fades[2] = { 0, 0 };
#define WANT(c, what)                                                                              \
    do {                                                                                           \
        if (!(c)) {                                                                                \
            printf("FAIL %s\n", what);                                                             \
            fails++;                                                                               \
        }                                                                                          \
    } while (0)
    /* The whole replay with the plan animations on, then off: the handoff must not depend on them. */
    for (int an = 1; an >= 0; an--) {
        anim_set_enabled(an);
        metrics m[N_SCENES][2];
        for (int s = 0; s < N_SCENES; s++)
            for (int h = 0; h < 2; h++) {
                g_seed = seed + (unsigned)s;
                /* the crouch scene's before is the handoff on, the crouch height model off; the
                 * walk-in scene's is the handoff on, part_keep off */
                run(s, s == CROUCH || s == WALK_IN ? 1 : h, s == WALK_IN ? 1 : h, s == WALK_IN ? h : 1, cfg_path,
                    &m[s][h]);
            }
        flips[an] = m[STEP_OUT][1].look_flips;
        fades[an] = m[STEP_OUT][1].fade_frames;
        printf("handoff replay, animations %s, same seed per scene (before = --no-handoff / --handoff off;\n"
               "crouch: before = --no-crouch-range; walk-in: before = --no-part-keep):\n",
               an ? "on" : "off");
        for (int s = 0; s < N_SCENES; s++) {
            printf("%s\n", SCENE[s]);
            print("before", &m[s][0]);
            print("after", &m[s][1]);
        }
        for (int h = 0; h < 2; h++) {
            const metrics *cm = &m[CROUCH][h];
            printf("  %-6s crouched %d ticks: two tracks %d, one track CROUCH %d; %d frames, x-ray drawn %d; "
                   "farthest track %.2f m from the person, %.2f m right of them\n",
                   h ? "after" : "before", cm->c_ticks, cm->c_two, cm->c_crouch, cm->c_frames, cm->c_xray,
                   cm->c_far, cm->c_right);
        }
        for (int h = 0; h < 2; h++) {
            const metrics *wm = &m[WALK_IN][h];
            printf("  %-6s walk-in, %d ms from R2's first sighting: farthest %.2f m from the person, largest step "
                   "%.2f m, tallest ghost %.2f person heights, two-track ticks %d\n",
                   h ? "after" : "before", WALK_IN_WINDOW_MS, wm->w_err, wm->w_step, wm->w_tall, wm->w_two);
        }
        const metrics *so = &m[STEP_OUT][1], *sl = &m[STEP_OUT_R2_LOST][1], *sf = &m[STEP_OUT_FAR][1];
        const metrics *ho = &m[HAND_OUT][1];
        WANT(so->ids == 1 && so->two_ticks == 0, "step-out: one track id, never two people");
        WANT(so->look_flips == 1, "step-out: x-ray to seen once, no flicker on R1's missed frames");
        WANT(so->fade_frames >= 3 && so->fade_frames <= 12, "step-out: the look crossfades (about 250 ms)");
        WANT(so->max_step < 0.3 && so->max_step < m[STEP_OUT][0].max_step && so->err_p95 < 0.15,
             "step-out: glides onto the fused fix and stays on it");
        WANT(so->err_p95 <= m[STEP_OUT][0].err_p95 + 0.03, "step-out: as close to the truth as before (3 cm)");
        WANT(sl->ids == 1 && sl->two_ticks == 0, "step-out, R2 lost: one track id, never two people");
        /* R1 takes over: R2's 25% long range is gone. R1's 15% short one (0.28 m at 1.9 m) is left
         * unless the phones' last and first messages overlap for a fused fix, as they do here (the
         * phones are not in step), which teaches R1's range ratio. */
        WANT(sl->max_step < 0.35 && sl->err_p95 < 0.45 && sl->err_p50 < m[STEP_OUT_R2_LOST][0].err_p50,
             "step-out, R2 lost: no jump at the phone switch, R1's range from R1");
        WANT(sf->ids == 1 && sf->two_ticks == 0 && sf->look_flips == 1,
             "step-out, far R2: the 1.9 m correction keeps the track (one id, seen once)");
        /* 1.9 m to correct: a 90 ms glide moves 31% of it in the first 33 ms tick, then less. */
        WANT(sf->max_step < 0.7 && sf->max_step < 0.5 * m[STEP_OUT_FAR][0].max_step && sf->err_p95 < 0.35,
             "step-out, far R2: glides onto the fused fix instead of jumping");
        WANT(ho->ids == 1 && ho->two_ticks == 0 && ho->seen_frames == 0,
             "hand-out: one x-ray person, R1's part sighting neither a second person nor SEEN");
        WANT(ho->err_p50 < m[HAND_OUT][0].err_p50, "hand-out: R1's bearing pulls the ghost onto the person");
        WANT(m[PARTIAL_ALONE][1].tracks_partial_alone == 0, "partial alone: R1's part sighting starts no track");
        const metrics *cr = &m[CROUCH][1];
        WANT(cr->ids == 1 && cr->two_ticks == 0 && cr->c_two == 0,
             "crouch: one track id, never a second copy");
        WANT(cr->c_frames > 0 && cr->c_xray == 0 && cr->seen_frames > 0,
             "crouch: SEEN all the way down, no x-ray copy drawn");
        WANT(cr->c_ticks > 0 && cr->c_crouch >= cr->c_ticks - 3, "crouch: posture CROUCH once down");
        WANT(cr->c_far < 0.35 && cr->c_right < 0.25 && cr->err_p95 < 0.25,
             "crouch: the track stays on the person (no copy off to the right)");
        /* The seen frame is R1's own box: at once when R1 first sees an x-ray person (not eased
         * there from the ghost's place), and still while R1 misses a frame (not the fused
         * position projected back into R1's picture). The eased box lags a walker a little. */
        WANT(so->first_off < 0.06 && sl->first_off < 0.06 && sf->first_off < 0.06,
             "step-out: the first seen frame is on R1's own box, not eased from the ghost");
        /* which frames R1 misses is drawn from the seed: some seeds give a scene no held frame */
        WANT(so->held_frames + sf->held_frames > 0 && so->held_off < 0.12 && sf->held_off < 0.12,
             "step-out: a person held as seen stays in R1's last box");
        const metrics *wi = &m[WALK_IN][1], *wb = &m[WALK_IN][0];
        WANT(wi->ids == 1 && wi->w_two == 0, "walk-in: one track id, never two people");
        WANT(wi->w_err < 1.0 && wb->w_err > 1.5,
             "walk-in: R2's part sighting does not throw the person along R2's ray");
        /* Kept by bearing while R2 sees a part, the person may have walked on along R2's ray: the
         * correction when R2 sees them whole glides (31% of it in the first tick, as step-out). */
        WANT(wi->w_step < 0.6 && wi->w_step < wb->w_step, "walk-in: no jump when R2 picks them up or sees them whole");
        /* R2's range reads 25% long, so its ghost is that much taller than the 0.93 of the body. */
        WANT(wi->w_tall > 0 && wi->w_tall < 1.4 && wb->w_tall > 1.6, "walk-in: the ghost is person sized");
        WANT(wi->held_frames > 0 && wi->held_off < 0.12,
             "walk-in: the seen frame stays where R1 last saw them until the ghost takes over");
    }
    WANT(flips[0] == flips[1] && fades[0] == fades[1], "step-out: same look change and crossfade, animations on or off");
    printf("%s\n", fails ? "handoff targets missed" : "all handoff targets met");
    return fails ? 1 : 0;
}
