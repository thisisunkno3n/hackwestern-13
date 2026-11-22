/*
 * hud_anim_map_shot: renders the plan map animations (display/anim_map.h) of a made-up scene at
 * chosen times, as numbered PNGs, and times them (host only, not a QNX test).
 *
 *   build/hud_anim_map_shot --bg room.ppm --out dir [--size 1280x720] [--fps 30]
 *                           [--seq appear|down|scan|drawin|boot|all|none] [--bench n]
 *
 * Scene as tools/hud_shot.c: wearer R1 fixed at (0,-2) facing +y, R2 at (3.6,2.6), a wall from
 * (0.3,0) to (2.5,0), person 1 seen by R1 at (-1.3,2.3), person 2 behind the wall (only R2 sees
 * them) at (1.1,2.6). Each sequence starts from anim_map_reset() at 10000 ms and renders
 * pre-roll frames first (not saved), then writes dir/<seq>/f0000.png ... one per 1000/fps ms:
 *   appear  inset; person 3 (seen) appears at +200 ms, person 4 (behind the wall) at +800 ms
 *   down    inset; person 1 goes down (lying at (-1.0,4.5)) at +200 ms, 3.2 s
 *   scan    inset; 3.4 s of the cone scan and the marching hidden dashes
 *   drawin  inset for 1.5 s, then the video is lost: the full-screen plan draws in, 0.7 s
 *   boot    no pre-roll: the inset's first frames, 0.7 s
 * --bench n: hud_render time at the frame size with the animations on and off (inset and full
 * screen), people replaced every 450 ms so rings are always running. tools/anim_map_media.sh
 * turns the frames into GIFs.
 */

#include "anim_map.h"
#include "draw.h"
#include "hud_render.h"
#include "seeker_config.h"
#include "seeker_health.h"
#include "seeker_posture.h"
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

#define T0 10000

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

static uint32_t *scale_to(const uint32_t *src, int sw, int sh, int dw, int dh)
{
    uint32_t *d = malloc((size_t)dw * (size_t)dh * 4);
    for (int y = 0; d && y < dh; y++)
        for (int x = 0; x < dw; x++)
            d[(size_t)y * dw + x] = src[(size_t)(y * sh / dh) * sw + (size_t)(x * sw / dw)];
    return d;
}

static int write_png(const char *path, const canvas *c)
{
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

/* R1's detection box for a person at room (x, y), standing or lying (as hud_shot). */
static void person_box(float x, float y, int lying, float box[4])
{
    const float F = 640.0f / tanf(33.0f * 0.017453292519943295f), z = y + 2.0f;
    if (lying) {
        float u0 = 640.0f + F * (x - 0.55f) / z, u1 = 640.0f + F * (x + 0.95f) / z;
        float top = 360.0f - F * (0.4f - 1.7f) / z, bot = 360.0f + F * 1.7f / z;
        box[0] = u0;
        box[1] = top;
        box[2] = u1 - u0;
        box[3] = (bot > 720.0f ? 720.0f : bot) - top;
        return;
    }
    float cx = 640.0f + F * x / z, top = 360.0f - F * (1.75f - 1.7f) / z, h = F * 1.75f / z;
    if (top + h > 720.0f)
        h = 720.0f - top;
    box[2] = F * 0.6f / z;
    box[0] = cx - box[2] / 2;
    box[1] = top;
    box[3] = h;
}

typedef struct {
    int id;
    const char *nodes; /* "R1": the wearer sees them; "R2": behind the wall */
    float x, y;
    int down;
} person;

static void scene(seeker_tracks *t, const person *p, int n)
{
    memset(t, 0, sizeof *t);
    add_node(t, "R1", 0, -2, 0, 15);
    add_node(t, "R2", 3.6f, 2.6f, -90, 28);
    for (int i = 0; i < n; i++) {
        seeker_track *k = &t->tracks[t->n++];
        memset(k, 0, sizeof *k);
        k->id = p[i].id;
        snprintf(k->label, sizeof k->label, "%s-%d", p[i].nodes, p[i].id);
        snprintf(k->cls, sizeof k->cls, "person");
        snprintf(k->nodes, sizeof k->nodes, "%s", p[i].nodes);
        snprintf(k->node_id, sizeof k->node_id, "%s", p[i].nodes);
        snprintf(k->colour, sizeof k->colour, "amber");
        k->visible = 1;
        k->has_room = 1;
        k->room_x_m = p[i].x;
        k->room_y_m = p[i].y;
        k->has_range = 1;
        k->range_m = 2.4f;
        k->range_conf = 0.9f;
        k->age_ms = 40;
        k->posture = p[i].down ? SEEKER_POSTURE_DOWN : SEEKER_POSTURE_STAND;
        if (!strcmp(p[i].nodes, "R1")) {
            k->has_node_box = 1;
            person_box(p[i].x, p[i].y, p[i].down, k->node_box);
        }
    }
}

static canvas cv;
static seeker_config cfg;
static seeker_health_table health;
static video_frame vf;
static wv_view v;
static seeker_tracks tracks;

static void view_reset(void)
{
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
}

/* One frame at time t (ms): the only clock the animations see. */
static void frame(int64_t t, const person *p, int n, int lost)
{
    scene(&tracks, p, n);
    hud_state hs;
    memset(&hs, 0, sizeof hs);
    hs.t = &tracks;
    hs.tracks_age_ms = 14;
    hs.health = &health;
    hs.fps = 30;
    hs.background = "wearer";
    hs.view_node = "R1";
    hs.video = &vf;
    hs.video_fps = 15;
    hs.wearer = &v;
    hs.video_state = lost ? VIDEO_LOST : VIDEO_OK;
    hs.video_why = lost ? VIDEO_WHY_NO_FRAMES : VIDEO_WHY_OK;
    vf.t_rx_ms = seeker_now_ms();
    for (int i = 0; i < 3; i++)
        health.slot[i].last_beat_ms = seeker_now_ms();
    hs.now_ms = t; /* the one frame time: hud_render hands it to the map and person modules */
    hud_render(&cv, &cfg, &hs);
}

static const person BASE[2] = { { 1, "R1", -1.3f, 2.3f, 0 }, { 2, "R2", 1.1f, 2.6f, 0 } };

/* Scene of sequence `seq` at ms `e` after the save start; returns the people count. */
static int people_at(const char *seq, int64_t e, person *p, int *lost)
{
    memcpy(p, BASE, sizeof BASE);
    int n = 2;
    *lost = 0;
    if (!strcmp(seq, "appear")) {
        if (e >= 200)
            p[n++] = (person){ 3, "R1", 0.6f, 1.0f, 0 };
        if (e >= 800)
            p[n++] = (person){ 4, "R2", 1.8f, 1.6f, 0 };
    } else if (!strcmp(seq, "down")) {
        if (e >= 200)
            p[0] = (person){ 1, "R1", -1.0f, 4.5f, 1 };
    } else if (!strcmp(seq, "drawin")) {
        *lost = e >= 0;
    }
    return n;
}

static int run_seq(const char *out, const char *seq, int fps)
{
    int pre = !strcmp(seq, "boot") ? 0 : !strcmp(seq, "drawin") ? 1500 : 1000;
    int dur = !strcmp(seq, "down") ? 3200 : !strcmp(seq, "scan") ? 3400 : !strcmp(seq, "appear") ? 1600 : 700;
    char dir[512], path[600];
    snprintf(dir, sizeof dir, "%s/%s", out, seq);
    mkdir(out, 0755);
    mkdir(dir, 0755);
    anim_map_reset();
    view_reset();
    int saved = 0;
    for (int i = 0;; i++) {
        int64_t e = (int64_t)i * 1000 / fps - pre; /* ms from the save start */
        if (e >= dur)
            break;
        person p[8];
        int lost, n = people_at(seq, e, p, &lost);
        frame(T0 + pre + e, p, n, lost);
        if (e >= 0) {
            snprintf(path, sizeof path, "%s/f%04d.png", dir, saved++);
            if (write_png(path, &cv) != 0)
                return -1;
        }
    }
    printf("%-7s %3d frames in %s\n", seq, saved, dir);
    return 0;
}

static double ms_now(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec * 1e3 + (double)ts.tv_nsec / 1e6;
}

static int cmp_d(const void *a, const void *b)
{
    double x = *(const double *)a, y = *(const double *)b;
    return x < y ? -1 : x > y;
}

/* Each frame time is rendered twice, animations off then on, so drift and the video blit's
 * noise cancel in the paired difference; the median of the differences is reported. */
static void bench(int frames)
{
    double *dd = malloc(sizeof(double) * (size_t)frames);
    if (!dd)
        return;
    for (int lost = 0; lost < 2; lost++) {
        anim_map_reset();
        view_reset();
        double sum[2] = { 0, 0 }, mx[2] = { 0, 0 };
        for (int i = 0; i < frames; i++) {
            int64_t t = T0 + (int64_t)i * 33;
            int base = 1 + (int)((t - T0) / 450) * 5; /* five new ids every 450 ms */
            person p[5] = { { base, "R1", -1.3f, 2.3f, 0 },     { base + 1, "R2", 1.1f, 2.6f, 0 },
                            { base + 2, "R1", 0.6f, 1.0f, 0 },  { base + 3, "R2", 1.8f, 1.6f, 0 },
                            { base + 4, "R1", -1.0f, 4.5f, 1 } };
            double d[2];
            for (int on = 0; on < 2; on++) {
                anim_map_set_enabled(on);
                double a = ms_now();
                frame(t, p, 5, lost);
                d[on] = ms_now() - a;
                sum[on] += d[on];
                if (d[on] > mx[on])
                    mx[on] = d[on];
            }
            dd[i] = d[1] - d[0];
        }
        qsort(dd, (size_t)frames, sizeof *dd, cmp_d);
        const char *name = lost ? "full-screen" : "inset";
        printf("bench %-11s %dx%d %d frames: off mean %.3f max %.3f ms, on mean %.3f max %.3f ms\n", name, cv.w,
               cv.h, frames, sum[0] / frames, mx[0], sum[1] / frames, mx[1]);
        printf("bench %-11s animations add %.3f ms per frame (median of paired frames, 90th pct %.3f)\n", name,
               dd[frames / 2], dd[frames * 9 / 10]);
    }
    free(dd);
    anim_map_set_enabled(1);
}

int main(int argc, char **argv)
{
    const char *bg = NULL, *out = ".", *seq = "all";
    int W = 1280, H = 720, fps = 30, nbench = 0;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--bg") && i + 1 < argc) bg = argv[++i];
        else if (!strcmp(argv[i], "--out") && i + 1 < argc) out = argv[++i];
        else if (!strcmp(argv[i], "--size") && i + 1 < argc) sscanf(argv[++i], "%dx%d", &W, &H);
        else if (!strcmp(argv[i], "--fps") && i + 1 < argc) fps = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--seq") && i + 1 < argc) seq = argv[++i];
        else if (!strcmp(argv[i], "--bench") && i + 1 < argc) nbench = atoi(argv[++i]);
        else {
            fprintf(stderr, "usage: %s --bg frame.ppm [--out dir] [--size WxH] [--fps n] "
                            "[--seq appear|down|scan|drawin|boot|all|none] [--bench n]\n", argv[0]);
            return 2;
        }
    }
    int bw = 0, bh = 0;
    uint32_t *src = bg ? load_ppm(bg, &bw, &bh) : NULL;
    if (!src || fps <= 0) {
        fprintf(stderr, "cannot read P6 PPM %s\n", bg ? bg : "(none)");
        return 1;
    }
    uint32_t *vid = scale_to(src, bw, bh, W, H);
    cv = (canvas){ calloc((size_t)W * H, 4), W, H, W };
    if (!vid || !cv.px)
        return 1;
    seeker_config_defaults(&cfg);
    snprintf(cfg.background, sizeof cfg.background, "wearer");
    const char *procs[3] = { "hud_fusion", "hud_display", "hud_supervisor" };
    for (int i = 0; i < 3; i++) {
        snprintf(health.slot[i].name, sizeof health.slot[i].name, "%s", procs[i]);
        health.slot[i].state = SH_OK;
    }
    vf.w = vf.stride = W;
    vf.h = H;
    vf.src_w = 1280;
    vf.src_h = 720;
    vf.rect[2] = W;
    vf.rect[3] = H;
    vf.px = vid;
    snprintf(vf.node_id, sizeof vf.node_id, "R1");
    draw_set_theme(DRAW_THEME_PLAN);
    draw_ui_init(H);

    static const char *const all[] = { "appear", "down", "scan", "drawin", "boot" };
    for (int i = 0; i < 5; i++)
        if ((!strcmp(seq, "all") || !strcmp(seq, all[i])) && run_seq(out, all[i], fps) != 0)
            return 1;
    if (nbench > 0)
        bench(nbench);
    free(src);
    free(vid);
    free(cv.px);
    return 0;
}
