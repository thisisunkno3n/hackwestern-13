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

int main(int argc, char **argv)
{
    const char *bg = NULL, *bg_close = NULL, *bg_down = NULL, *out = ".";
    int W = 1280, H = 720, classic_all = 0, reps = 20;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--bg") && i + 1 < argc) bg = argv[++i];
        else if (!strcmp(argv[i], "--bg-close") && i + 1 < argc) bg_close = argv[++i];
        else if (!strcmp(argv[i], "--bg-down") && i + 1 < argc) bg_down = argv[++i];
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
        else if (!strcmp(argv[i], "--size") && i + 1 < argc) sscanf(argv[++i], "%dx%d", &W, &H);
        else if (!strcmp(argv[i], "--reps") && i + 1 < argc) reps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--classic-all")) classic_all = 1;
        else {
            fprintf(stderr, "usage: %s --bg frame.ppm [--bg-close frame.ppm] [--out dir] [--size WxH] "
                            "[--classic-all] [--reps n]\n", argv[0]);
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
