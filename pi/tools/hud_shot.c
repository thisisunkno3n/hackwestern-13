/*
 * hud_shot: renders the EagleEye wearer view of a made-up scene to PNG files, so the look can be
 * checked on a laptop without phones or a Pi (host only, not a QNX test).
 *
 *   build/hud_shot --bg room.ppm [--bg-close room-close.ppm] [--bg-down room-down.ppm]
 *                  --out shots/ [--size 1280x720] [--classic-all] [--reps 20]
 *
 * --bg is a binary PPM (P6) camera frame for R1. Scene: wearer R1 fixed at (0,-2) facing +y, a
 * wall from (0.3,0) to (2.5,0), one person R1 sees at (-1.3,2.3), one person behind the wall at
 * (1.1,2.6) that only R2 sees (with a COCO-17 skeleton). States: normal, close (the seen person
 * walks in to 1.4 m; --bg-close is the matching frame), video-late, map-fallback,
 * xray-no-skeleton, down (the seen person lying at (-1.0,4.5); --bg-down), down-behind-wall (the
 * person behind the wall lying, posture down). Writes plan-<state>.png and classic-normal.png (--classic-all: every state
 * for classic too, for the pixel check against main). Built with -DHUD_SHOT_CLASSIC_ONLY it
 * renders classic only (it then compiles against older trees without the plan theme).
 *
 * --seq DIR renders the plan theme's alert animations (display/anim_alert.h) instead: six
 * sequences, one PNG every 33 ms in DIR/<name>/f000.png..., driven only by hud_state.now_ms, so
 * every frame is a chosen moment: close (someone walks in fast, then back out), down (the seen
 * person goes down, gets up at 3 s), down-behind-wall (the same behind the wall), video-late (the
 * video stalls 1.5 s), title-change (R2 lost for 1.2 s: Eyes cell and the "R2 lost" plate),
 * warning (R1 position held for 1.2 s: the top-left plate and the Position cell). Each sequence
 * is rendered again with the animations off and the per-frame cost of both is printed.
 */

#include "draw.h"
#include "hud_render.h"
#include "seeker_config.h"
#include "seeker_health.h"
#include "seeker_time.h"
#include "wearer_view.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif
#include "../third_party/stb/stb_image_write.h"
#if defined(__GNUC__) || defined(__clang__)
#pragma GCC diagnostic pop
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#ifndef HUD_SHOT_CLASSIC_ONLY
#include "anim_alert.h"
#endif

static uint32_t *load_ppm(const char *path, int *w, int *h)
{
    FILE *f = fopen(path, "rb");
    if (!f)
        return NULL;
    int maxv = 0;
    if (fscanf(f, "P6 %d %d %d", w, h, &maxv) != 3 || maxv != 255 || *w <= 0 || *h <= 0) {
        fclose(f);
        return NULL;
    }
    fgetc(f);
    size_t n = (size_t)*w * (size_t)*h;
    unsigned char *rgb = malloc(n * 3);
    uint32_t *px = malloc(n * 4);
    if (!rgb || !px || fread(rgb, 3, n, f) != n) {
        fclose(f);
        free(rgb);
        free(px);
        return NULL;
    }
    fclose(f);
    for (size_t i = 0; i < n; i++)
        px[i] = RGB(rgb[3 * i], rgb[3 * i + 1], rgb[3 * i + 2]);
    free(rgb);
    return px;
}

/* Nearest-neighbour scale into the canvas size (video_rx does this on the Pi). */
static uint32_t *scale_to(const uint32_t *src, int sw, int sh, int dw, int dh)
{
    uint32_t *d = malloc((size_t)dw * (size_t)dh * 4);
    for (int y = 0; d && y < dh; y++)
        for (int x = 0; x < dw; x++)
            d[(size_t)y * dw + x] = src[(size_t)(y * sh / dh) * sw + (size_t)(x * sw / dw)];
    return d;
}

static int write_png(const char *dir, const char *name, const canvas *c)
{
    char path[512];
    snprintf(path, sizeof path, "%s/%s.png", dir, name);
    unsigned char *rgb = malloc((size_t)c->w * (size_t)c->h * 3);
    if (!rgb)
        return -1;
    for (int y = 0; y < c->h; y++)
        for (int x = 0; x < c->w; x++) {
            uint32_t p = c->px[(size_t)y * c->stride + x];
            unsigned char *o = rgb + ((size_t)y * c->w + x) * 3;
            o[0] = (unsigned char)(p >> 16);
            o[1] = (unsigned char)(p >> 8);
            o[2] = (unsigned char)p;
        }
    int ok = stbi_write_png(path, c->w, c->h, 3, rgb, c->w * 3);
    free(rgb);
    printf("%s %s\n", ok ? "wrote" : "FAILED", path);
    return ok ? 0 : -1;
}

static void add_node(seeker_tracks *t, const char *id, float x, float y, float yaw, float hz)
{
    seeker_node_status *n = &t->nodes[t->n_nodes++];
    memset(n, 0, sizeof *n);
    snprintf(n->node_id, sizeof n->node_id, "%s", id);
    snprintf(n->state, sizeof n->state, "ok");
    snprintf(n->pose_src, sizeof n->pose_src, "cli");
    n->x_m = x;
    n->y_m = y;
    n->yaw_deg = yaw;
    n->hfov_deg = 66;
    n->rate_hz = hz;
    n->n_contacts = 1;
}

static seeker_track *add_track(seeker_tracks *t, int id, const char *nodes, float x, float y,
                               const char *colour)
{
    seeker_track *k = &t->tracks[t->n++];
    memset(k, 0, sizeof *k);
    k->id = id;
    snprintf(k->label, sizeof k->label, "%s-%d", nodes, id);
    snprintf(k->cls, sizeof k->cls, "person");
    snprintf(k->nodes, sizeof k->nodes, "%s", nodes);
    snprintf(k->node_id, sizeof k->node_id, "%s", nodes);
    snprintf(k->colour, sizeof k->colour, "%s", colour);
    k->visible = 1;
    k->has_room = 1;
    k->room_x_m = x;
    k->room_y_m = y;
    k->has_range = 1;
    k->range_m = 2.4f;
    k->range_conf = 0.9f;
    k->age_ms = 40;
    return k;
}

/* A person lying on the floor in R1's frame: x from -0.55 to +0.95 m, 0.4 m high. */
static void lying_box(float x, float y, float box[4])
{
    const float F = 640.0f / tanf(33.0f * 0.017453292519943295f), z = y + 2.0f;
    float u0 = 640.0f + F * (x - 0.55f) / z, u1 = 640.0f + F * (x + 0.95f) / z;
    float top = 360.0f - F * (0.4f - 1.7f) / z, bot = 360.0f + F * 1.7f / z;
    if (bot > 720.0f)
        bot = 720.0f;
    box[0] = u0;
    box[1] = top;
    box[2] = u1 - u0;
    box[3] = bot - top;
}

/* Standing person facing the camera, COCO-17, person-height units from the mid-hip. */
static const float POSE[SEEKER_N_JOINTS][2] = {
    { 0.0f, -0.47f },   { 0.02f, -0.485f },  { -0.02f, -0.485f }, { 0.045f, -0.475f },
    { -0.045f, -0.475f }, { 0.12f, -0.33f },  { -0.12f, -0.33f },  { 0.165f, -0.17f },
    { -0.17f, -0.16f }, { 0.175f, -0.02f },   { -0.14f, 0.0f },    { 0.065f, 0.0f },
    { -0.065f, 0.0f },  { 0.075f, 0.25f },    { -0.08f, 0.245f },  { 0.08f, 0.49f },
    { -0.09f, 0.485f },
};

/* R1's own detection box for a person at room (x, y): the 1280x720 pinhole frame, hfov 66,
 * camera 1.7 m up at (0,-2) facing +y; 0.6 m wide, 1.75 m tall, clipped to the frame. */
static void person_box(float x, float y, float box[4])
{
    const float F = 640.0f / tanf(33.0f * 0.017453292519943295f), z = y + 2.0f;
    float cx = 640.0f + F * x / z, top = 360.0f - F * (1.75f - 1.7f) / z, h = F * 1.75f / z;
    if (top + h > 720.0f)
        h = 720.0f - top;
    box[2] = F * 0.6f / z;
    box[0] = cx - box[2] / 2;
    box[1] = top;
    box[3] = h;
}

#ifndef HUD_SHOT_CLASSIC_ONLY
/* ---- --seq: alert animation sequences ---- */

typedef struct {
    seeker_tracks t;
    wv_view v;
    hud_state hs;
    seeker_track *seen, *hid;
} seq_scene;

static void seq_setup(seq_scene *sc, video_frame *f, seeker_health_table *health, uint32_t *bg)
{
    memset(sc, 0, sizeof *sc);
    seeker_tracks *t = &sc->t;
    add_node(t, "R1", 0, -2, 0, 15);
    add_node(t, "R2", 3.6f, 2.6f, -90, 28);
    sc->seen = add_track(t, 1, "R1", -1.3f, 2.3f, "amber");
    sc->seen->has_node_box = 1;
    person_box(sc->seen->room_x_m, sc->seen->room_y_m, sc->seen->node_box);
    sc->hid = add_track(t, 2, "R2", 1.1f, 2.6f, "cyan");
    sc->hid->has_pts = 1;
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        sc->hid->pts[j][0] = POSE[j][0];
        sc->hid->pts[j][1] = POSE[j][1];
        sc->hid->pts[j][2] = 0.9f;
    }
    wv_view *v = &sc->v;
    wv_config_defaults(&v->cfg);
    snprintf(v->cfg.node, sizeof v->cfg.node, "R1");
    v->cfg.pose.x_m = 0;
    v->cfg.pose.y_m = -2;
    v->cfg.pose_src = WV_POSE_STATIC;
    v->cfg.n_walls = 1;
    v->cfg.walls[0][0] = 0.3f;
    v->cfg.walls[0][1] = 0;
    v->cfg.walls[0][2] = 2.5f;
    v->cfg.walls[0][3] = 0;
    v->cfg.radar_fallback = 1;
    v->cfg.from_config = 1;
    hud_state *hs = &sc->hs;
    hs->t = t;
    hs->tracks_age_ms = 14;
    hs->health = health;
    hs->fps = 30;
    hs->background = "wearer";
    hs->view_node = "R1";
    hs->video = f;
    hs->video_fps = 15;
    hs->wearer = v;
    hs->video_state = VIDEO_OK;
    hs->video_why = VIDEO_WHY_OK;
    f->px = bg;
}

static void seq_lie(seeker_track *k, int down)
{
    k->posture = down ? SEEKER_POSTURE_DOWN : SEEKER_POSTURE_UNKNOWN;
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        k->pts[j][0] = down ? POSE[j][1] : POSE[j][0];
        k->pts[j][1] = down ? -POSE[j][0] * 0.4f : POSE[j][1];
    }
}

static float lerpf(float a, float b, float t)
{
    return a + (b - a) * (t < 0 ? 0 : t > 1 ? 1 : t);
}

/* The scene at time t (ms from the start) of sequence `which`. */
static void seq_at(int which, seq_scene *sc, video_frame *f, int t, uint32_t *vid_far, uint32_t *vid_down)
{
    seeker_track *seen = sc->seen, *hid = sc->hid;
    f->px = vid_far;
    f->t_rx_ms = seeker_now_ms() - 30;
    sc->hs.video_state = VIDEO_OK;
    switch (which) {
    case 0: /* close: 1 m/s in from 2.4 m to 1.4 m, hold, back out to 3.4 m */
        seen->room_x_m = -0.35f;
        seen->room_y_m = t < 1700 ? lerpf(0.4f, -0.6f, (t - 300) / 800.0f) : lerpf(-0.6f, 1.4f, (t - 1700) / 400.0f);
        person_box(seen->room_x_m, seen->room_y_m, seen->node_box);
        break;
    case 1: /* down in view */
        f->px = vid_down;
        seen->room_x_m = -1.0f;
        seen->room_y_m = 4.5f;
        if (t >= 300 && t < 3000) {
            seen->posture = SEEKER_POSTURE_DOWN;
            lying_box(seen->room_x_m, seen->room_y_m, seen->node_box);
        } else {
            seen->posture = SEEKER_POSTURE_UNKNOWN;
            person_box(seen->room_x_m, seen->room_y_m, seen->node_box);
        }
        break;
    case 2: /* down behind the wall */
        seq_lie(hid, t >= 300 && t < 3000);
        break;
    case 3: /* video stalls for 1.5 s */
        if (t >= 300 && t < 1800) {
            sc->hs.video_state = VIDEO_STALE;
            f->t_rx_ms = seeker_now_ms() - (500 + (t - 300));
        }
        break;
    case 4: /* R2 lost for 1.2 s */
        snprintf(sc->t.nodes[1].state, sizeof sc->t.nodes[1].state, "%s", t >= 300 && t < 1500 ? "lost" : "ok");
        break;
    case 5: /* R1 loses its marker for 1.2 s: position held */
        sc->v.cfg.pose_src = WV_POSE_FUSION;
        snprintf(sc->t.nodes[0].pose_src, sizeof sc->t.nodes[0].pose_src, "%s",
                 t >= 300 && t < 1500 ? "held" : "cli");
        break;
    }
}

static double now_us(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e6 + (double)ts.tv_nsec / 1e3;
}

/* The module's own cost on its worst frame, every effect at once at 1280x720 scale: a plate
 * mid hatch sweep, a person-down ring around a big frame, the pulsing hull outline and its ring,
 * a title cell crossfading, the late hatch sweeping, a warning plate sliding in and one sliding
 * out. Min over 200 runs of the hooks alone, minus the plain plan drawing of the same plate,
 * cell and warning plate. */
static void bench_module(canvas *cv)
{
    float u = plan_u(cv);
    plan_line l[2] = { { "Person down", DRAW_FONT_SANS_SEMIBOLD, 19, P_ALERT, 255 },
                       { "1.4 m, coming closer", DRAW_FONT_MONO, 15, P_ALERT, 255 } };
    plan_line w2[2] = { { "R1 position held", DRAW_FONT_SANS_SEMIBOLD, 19, P_INK, 255 },
                        { "phone lost its marker", DRAW_FONT_MONO, 15, P_INK, P_INK2_A } };
    plan_line w3[1] = { { "R2 lost", DRAW_FONT_SANS_SEMIBOLD, 19, P_ALERT, 255 } };
    draw_poly_style hs = { 0 };
    hs.hatch = P_ALERT;
    hs.hatch_a = 77;
    hs.hatch_gap = 9 * u;
    hs.hatch_w = 2 * u;
    double best[2] = { 1e18, 1e18 };
    for (int pass = 0; pass < 2; pass++) /* 0: animated hooks, 1: the plain drawing they replace */
        for (int rep = 0; rep < 200; rep++) {
            anim_alert_reset();
            anim_alert_set_enabled(pass == 0);
            anim_alert_frame(1000);
            anim_alert_track(1, 0, 0);
            anim_alert_track(2, 0, 0);
            anim_alert_cell(cv, 0, 40, 670, (int)lroundf(15 * u), P_INK, "R1 live", 24, 642, 160, 54);
            anim_alert_late(0, &(float){ 0 }, &(float){ 0 });
            anim_alert_warn(cv, u, 400, 24, w3, 1, P_ALERT);
            anim_alert_warn_flush(cv, u);
            anim_alert_frame(1100);
            double t0 = now_us();
            anim_fx f1 = anim_alert_track(1, 1, 1), f2 = anim_alert_track(2, 1, 1);
            anim_alert_ring(cv, u, &f1, 200, 200, 420, 480);
            anim_alert_hull(cv, u, &f2, 700, 420, 420, 200);
            anim_alert_plate(cv, u, 660, 300, l, 2, P_ALERT, 1, &f1);
            anim_alert_cell(cv, 0, 40, 670, (int)lroundf(15 * u), P_ALERT, "Video 1.2 s late", 24, 642, 200, 54);
            float sw, k;
            if (anim_alert_late(1, &sw, &k))
                anim_alert_hatch_rect(cv, 24, 642, 200, 54, 0, &hs, sw, k);
            anim_alert_warn(cv, u, 24, 24, w2, 2, P_ALERT);
            anim_alert_warn_flush(cv, u); /* "R2 lost" sliding out */
            double d = now_us() - t0;
            if (d < best[pass])
                best[pass] = d;
        }
    anim_alert_set_enabled(1);
    anim_alert_reset();
    printf("module worst frame: hooks %.1f us, plain drawing they replace %.1f us, animation cost %.1f us\n",
           best[0], best[1], best[0] - best[1]);
}

static int run_seq(const char *dir, canvas *cv, seeker_config *cfg, seeker_health_table *health, video_frame *f,
                   uint32_t *vid_far, uint32_t *vid_down)
{
    static const char *const names[] = { "close", "down", "down-behind-wall", "video-late", "title-change",
                                         "warning" };
    static const int dur[] = { 2400, 3600, 3600, 2600, 2600, 2400 };
    static seq_scene sc;
    static double cost[2][256];
    draw_set_theme(DRAW_THEME_PLAN);
    draw_ui_init(cv->h);
    mkdir(dir, 0755);
    for (int q = 0; q < 6; q++) {
        char sub[512];
        snprintf(sub, sizeof sub, "%s/%s", dir, names[q]);
        mkdir(sub, 0755);
        int nf = dur[q] / 33 + 1;
        if (nf > 256)
            nf = 256;
        for (int pass = 0; pass < 2; pass++) { /* animations on (frames written), then off */
            anim_alert_set_enabled(pass == 0);
            anim_alert_reset();
            seq_setup(&sc, f, health, vid_far);
            for (int k = 0; k < nf; k++) {
                int t = k * 33;
                seq_at(q, &sc, f, t, vid_far, vid_down);
                for (int i = 0; i < 3; i++)
                    health->slot[i].last_beat_ms = seeker_now_ms();
                sc.hs.now_ms = 100000 + t;
                sc.v.clock_ms = 100000 + t;
                double best = 1e18;
                for (int rep = 0; rep < 5; rep++) { /* min of 5: the same moment, idempotent */
                    double t0 = now_us();
                    hud_render(cv, cfg, &sc.hs);
                    double d = now_us() - t0;
                    if (d < best)
                        best = d;
                }
                cost[pass][k] = best;
                if (pass == 0) {
                    char name[600];
                    snprintf(name, sizeof name, "%s/f%03d", names[q], k);
                    if (write_png(dir, name, cv) != 0)
                        return 1;
                }
            }
        }
        anim_alert_set_enabled(1);
        double on = 0, off = 0, worst = -1e18;
        for (int k = 0; k < nf; k++) {
            on += cost[0][k];
            off += cost[1][k];
            if (cost[0][k] - cost[1][k] > worst)
                worst = cost[0][k] - cost[1][k];
        }
        printf("seq %-17s %3d frames  anim on %7.1f us  off %7.1f us  diff %6.1f us (max frame diff %6.1f us)\n",
               names[q], nf, on / nf, off / nf, (on - off) / nf, worst);
    }
    bench_module(cv);
    return 0;
}
#endif

int main(int argc, char **argv)
{
    const char *bg = NULL, *bg_close = NULL, *bg_down = NULL, *out = ".", *seq = NULL;
    int W = 1280, H = 720, classic_all = 0, reps = 20;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--bg") && i + 1 < argc) bg = argv[++i];
        else if (!strcmp(argv[i], "--bg-close") && i + 1 < argc) bg_close = argv[++i];
        else if (!strcmp(argv[i], "--bg-down") && i + 1 < argc) bg_down = argv[++i];
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
        else if (!strcmp(argv[i], "--seq") && i + 1 < argc) seq = argv[++i];
        else if (!strcmp(argv[i], "--size") && i + 1 < argc) sscanf(argv[++i], "%dx%d", &W, &H);
        else if (!strcmp(argv[i], "--reps") && i + 1 < argc) reps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--classic-all")) classic_all = 1;
        else {
            fprintf(stderr, "usage: %s --bg frame.ppm [--bg-close frame.ppm] [--out dir] [--size WxH] "
                            "[--classic-all] [--reps n] [--seq dir]\n", argv[0]);
            return 2;
        }
    }
    int bw = 0, bh = 0, cw = 0, chh = 0;
    uint32_t *src = bg ? load_ppm(bg, &bw, &bh) : NULL;
    if (!src) {
        fprintf(stderr, "cannot read P6 PPM %s\n", bg ? bg : "(none)");
        return 1;
    }
    uint32_t *src_close = bg_close ? load_ppm(bg_close, &cw, &chh) : NULL;
    int dw = 0, dh = 0;
    uint32_t *src_down = bg_down ? load_ppm(bg_down, &dw, &dh) : NULL;
    uint32_t *vid_far = scale_to(src, bw, bh, W, H);
    uint32_t *vid_close = src_close ? scale_to(src_close, cw, chh, W, H) : vid_far;
    uint32_t *vid_down = src_down ? scale_to(src_down, dw, dh, W, H) : vid_far;
    canvas cv = { calloc((size_t)W * H, 4), W, H, W };
    if (!vid_far || !vid_close || !cv.px)
        return 1;

    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    snprintf(cfg.background, sizeof cfg.background, "wearer");
    static seeker_health_table health;
    memset(&health, 0, sizeof health);
    const char *procs[3] = { "hud_fusion", "hud_display", "hud_supervisor" };
    for (int i = 0; i < 3; i++) {
        snprintf(health.slot[i].name, sizeof health.slot[i].name, "%s", procs[i]);
        health.slot[i].state = SH_OK;
    }
    video_frame f;
    memset(&f, 0, sizeof f);
    f.w = f.stride = W;
    f.h = H;
    f.src_w = 1280;
    f.src_h = 720;
    f.rect[2] = W;
    f.rect[3] = H;
    snprintf(f.node_id, sizeof f.node_id, "R1");

#ifndef HUD_SHOT_CLASSIC_ONLY
    if (seq) {
        int rc = run_seq(seq, &cv, &cfg, &health, &f, vid_far, vid_down);
        free(cv.px);
        return rc;
    }
#else
    (void)seq;
#endif
    static const char *const states[] = { "normal", "close", "video-late", "map-fallback",
                                          "xray-no-skeleton", "down", "down-behind-wall" };
    const int n_states = (int)(sizeof states / sizeof states[0]);
#ifdef HUD_SHOT_CLASSIC_ONLY
    const int n_themes = 1;
#else
    const int n_themes = 2;
#endif
    for (int th = 0; th < n_themes; th++) {
        const char *theme = th ? "plan" : "classic";
#ifndef HUD_SHOT_CLASSIC_ONLY
        draw_set_theme(th ? DRAW_THEME_PLAN : DRAW_THEME_CLASSIC);
        if (th)
            draw_ui_init(H);
#endif
        for (int s = 0; s < n_states; s++) {
            if (!th && !classic_all && s != 0)
                continue;
            static seeker_tracks t;
            memset(&t, 0, sizeof t);
            add_node(&t, "R1", 0, -2, 0, 15);
            add_node(&t, "R2", 3.6f, 2.6f, -90, 28);
            seeker_track *seen = add_track(&t, 1, "R1", -1.3f, 2.3f, "amber");
            seen->has_node_box = 1;
            person_box(seen->room_x_m, seen->room_y_m, seen->node_box);
            seeker_track *hid = add_track(&t, 2, "R2", 1.1f, 2.6f, "cyan");
            hid->has_pts = s != 4;
            for (int j = 0; j < SEEKER_N_JOINTS; j++) {
                hid->pts[j][0] = POSE[j][0];
                hid->pts[j][1] = POSE[j][1];
                hid->pts[j][2] = 0.9f;
            }
            if (s == 5) { /* the seen person is down on the floor further in */
                seen->room_x_m = -1.0f;
                seen->room_y_m = 4.5f;
                seen->posture = SEEKER_POSTURE_DOWN;
                lying_box(seen->room_x_m, seen->room_y_m, seen->node_box);
            }
            if (s == 6) { /* the person behind the wall is down: lying, head to the left */
                hid->posture = SEEKER_POSTURE_DOWN;
                for (int j = 0; j < SEEKER_N_JOINTS; j++) {
                    hid->pts[j][0] = POSE[j][1];
                    hid->pts[j][1] = -POSE[j][0] * 0.4f;
                }
            }
            static wv_view v;
            memset(&v, 0, sizeof v);
            wv_config_defaults(&v.cfg);
            snprintf(v.cfg.node, sizeof v.cfg.node, "R1");
            v.cfg.pose.x_m = 0;
            v.cfg.pose.y_m = -2;
            v.cfg.pose.yaw_deg = 0;
            v.cfg.pose_src = WV_POSE_STATIC;
            v.cfg.n_walls = 1;
            v.cfg.walls[0][0] = 0.3f;
            v.cfg.walls[0][1] = 0;
            v.cfg.walls[0][2] = 2.5f;
            v.cfg.walls[0][3] = 0;
            v.cfg.radar_fallback = 1;
            v.cfg.from_config = 1;
            hud_state hs;
            memset(&hs, 0, sizeof hs);
            hs.t = &t;
            hs.tracks_age_ms = 14;
            hs.health = &health;
            hs.fps = 30;
            hs.background = "wearer";
            hs.view_node = "R1";
            hs.video = &f;
            hs.video_fps = 15;
            hs.wearer = &v;
            hs.video_state = VIDEO_OK;
            hs.video_why = VIDEO_WHY_OK;
            f.px = s == 1 ? vid_close : s == 5 ? vid_down : vid_far;
            f.t_rx_ms = seeker_now_ms();
            if (s == 2) {
                hs.video_state = VIDEO_STALE;
                f.t_rx_ms = seeker_now_ms() - 1200;
            } else if (s == 3) {
                hs.video_state = VIDEO_LOST;
                hs.video_why = VIDEO_WHY_NO_FRAMES;
            }
            for (int i = 0; i < 3; i++)
                health.slot[i].last_beat_ms = seeker_now_ms();
            /* Settle the per-person colour state like a live HUD; "close" walks the seen person
             * in from 2.4 m to 1.4 m over 1 s, so the approach is fast. */
            int steps = s == 1 ? 11 : 2;
            for (int k = 0; k < steps; k++) {
                if (s == 1) {
                    float a = (float)k / (float)(steps - 1);
                    seen->room_x_m = -0.35f;
                    seen->room_y_m = 0.4f - 1.0f * a;
                    person_box(seen->room_x_m, seen->room_y_m, seen->node_box);
                }
                v.clock_ms = 1000 + 100 * k;
                hud_render(&cv, &cfg, &hs);
            }
            /* Save this frame first: the timing loop below repeats it at a fixed clock, which
             * would let the approach speed decay. */
            char name[64];
            snprintf(name, sizeof name, "%s-%s", theme, states[s]);
            int nd = v.last.n_direct, nx = v.last.n_xray, ns = v.last.n_skel;
            write_png(out, name, &cv);
            int64_t t0 = seeker_now_ms();
            for (int k = 0; k < reps; k++)
                hud_render(&cv, &cfg, &hs);
            double ms = reps > 0 ? (double)(seeker_now_ms() - t0) / reps : 0;
            printf("%-24s %dx%d  %.2f ms per frame  (seen %d, behind wall %d, skeletons %d)\n", name, W, H,
                   ms, nd, nx, ns);
        }
    }
    free(src);
    free(src_close);
    free(src_down);
    if (vid_close != vid_far)
        free(vid_close);
    if (vid_down != vid_far)
        free(vid_down);
    free(vid_far);
    free(cv.px);
    return 0;
}
