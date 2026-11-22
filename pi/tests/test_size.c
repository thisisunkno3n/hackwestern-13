/* Drawn size check: a standing person at 2, 3 and 5 m from the wearer, in view (SEEN box) and
 * behind a wall (XRAY hull and ghost skeleton), through the real hud_fusion core, the real tracks
 * datagram and the real wearer view (plan theme, over an R1 video frame), on a simulated clock.
 *
 *   test_size [-v]     (ctest "size"; host and QNX alike, no sockets, no screen; -v: fusion's log)
 *
 * What is right. A person of height P at depth z in front of a level pinhole camera of horizontal
 * FOV H and image width W is f * P / z px tall, f = (W / 2) / tan(H / 2); on the HUD that is times
 * the video scale (canvas px per image px). So:
 *   SEEN box       R1's own detection box, scaled from the detection image to the picture
 *   XRAY hull      f1 * P / z1 tall (R1's camera, the person's true depth from R1), less what
 *                  falls off the picture
 *   XRAY skeleton  eyes to ankles, 0.90 of f1 * P / z1
 * each within 10 percent.
 *
 * The phones are phone.html: bearing from the hip midpoint and range from eye to ankle over 0.90
 * of 1.7 m (else the torso over 0.30, else the box), both with the FOV the page assumes (66), box
 * of the landmarks at visibility 0.3, COCO-17 pts; nobody sent when the hips are not seen or the
 * box is not upright. Each camera images the person with its real FOV from its real height;
 * joints past the frame come back at visibility 0.1.
 *
 * Setups:
 *   default    config/seeker.live.json as it is; cameras really 66 deg at 1.7 m, person 1.7 m
 *   run-live   scripts/run-live.sh defaults: --hfov R1=iphone16 R2=iphone17pro (ranges rescaled),
 *              --person-h 1.75 on both programs, --wearer-cam-h 1.30, --cam-h R2=1.27,
 *              --pitch-auto on; cameras really those lenses at those heights, person 1.75 m
 *   run-live, portrait   the same with both phones upright (480x640)
 *
 * Room: R1, the wearer, pinned at 0,0 facing +y; R2 pinned at 6.5,3.4 facing -x; a wall from
 * 0.05,1 to 3,1. In view: the person at -0.6 right, r away. Behind the wall: at 0.5 right (R2
 * only). Close to R1 the frame cuts a standing person's legs; where R1's page then sends nobody
 * (not upright in its frame) they are an x-ray person and are checked as one.
 *
 * Then the live cases the sizes went wrong in (see the comments at each). */

#include "anim.h"
#include "draw.h"
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

static int fails, verbose;

/* COCO-17, person-height units from the mid-hip (x right, y down), standing; the mid-hip is 0.52
 * of the height above the floor. Eyes to ankles is 0.90 (phone.html EYE_ANKLE_FRAC). */
static const float BODY[SEEKER_N_JOINTS][2] = {
    { 0.00f, -0.40f }, { -0.02f, -0.42f }, { 0.02f, -0.42f }, { -0.05f, -0.41f }, { 0.05f, -0.41f },
    { -0.12f, -0.30f }, { 0.12f, -0.30f }, { -0.15f, -0.12f }, { 0.16f, -0.13f }, { -0.16f, 0.03f },
    { 0.17f, 0.03f },  { -0.08f, 0.00f }, { 0.08f, 0.00f },  { -0.08f, 0.25f }, { 0.08f, 0.25f },
    { -0.08f, 0.48f }, { 0.08f, 0.48f },
};
/* The same person in a deep crouch facing the camera (tests/test_handoff.c CROUCH_BODY), the
 * mid-hip 0.24 of the standing height above the floor. */
static const float CROUCH_BODY[SEEKER_N_JOINTS][2] = {
    { 0.00f, -0.31f }, { -0.02f, -0.33f }, { 0.02f, -0.33f }, { -0.05f, -0.32f }, { 0.05f, -0.32f },
    { -0.12f, -0.21f }, { 0.12f, -0.21f }, { -0.17f, -0.08f }, { 0.17f, -0.08f }, { -0.14f, 0.03f },
    { 0.14f, 0.03f }, { -0.08f, 0.00f }, { 0.08f, 0.00f }, { -0.16f, 0.03f }, { 0.16f, 0.03f },
    { -0.10f, 0.20f }, { 0.10f, 0.20f },
};
#define HIP_FRAC 0.52f
#define CROUCH_HIP_FRAC 0.24f
#define EYE_ANKLE 0.90f
#define PHONE_HFOV 66.0f /* phone.html's default, what it computes bearings and ranges with */

typedef struct {
    float x, y, yaw; /* room pose */
    float h;         /* lens height above the floor, m */
    float hfov;      /* the lens's real horizontal FOV, deg */
    int w, h_px;     /* its frame (640x480 landscape, 480x640 portrait) */
} camera;

typedef struct {
    const char *name;
    int live;       /* run-live.sh's knobs */
    float person_h; /* how tall the people really are */
    camera r1, r2;
} setup;

static const float WALL[4] = { 0.05f, 1.0f, 3.0f, 1.0f };

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

static float focal(int w, float hfov)
{
    return ((float)w / 2.0f) / tanf(hfov * 0.5f * DEG2RAD);
}

/* The person at room px,py as camera cam images them (facing it), then phone.html's contact from
 * those landmarks: joints past the frame come back at visibility 0.1; the contact is dropped when
 * the hips are not seen (CONF_MIN) or the box is not upright; box of the joints at 0.3; range from
 * eyes to ankles, else the torso, else the box. Returns 0 when the page sends nobody. */
static int phone_sees(const camera *cam, const char *node, int pid, float px, float py, float person_h,
                      int crouch, seeker_contact *c)
{
    const float (*body)[2] = crouch ? CROUCH_BODY : BODY;
    float hip = crouch ? CROUCH_HIP_FRAC : HIP_FRAC;
    float a = cam->yaw * DEG2RAD, dx = px - cam->x, dy = py - cam->y;
    float z = dx * sinf(a) + dy * cosf(a), xr = dx * cosf(a) - dy * sinf(a);
    if (z < 0.5f)
        return 0;
    float W = (float)cam->w, H = (float)cam->h_px, f = focal(cam->w, cam->hfov), fp = focal(cam->w, PHONE_HFOV);
    float pts[SEEKER_N_JOINTS][3], x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f;
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        float u = W / 2.0f + f * (xr + body[j][0] * person_h) / z;
        float v = H / 2.0f - f * ((hip - body[j][1]) * person_h - cam->h) / z;
        int in = u >= 0 && u <= W && v >= 0 && v <= H;
        pts[j][0] = u / W, pts[j][1] = v / H, pts[j][2] = in ? 0.9f : 0.1f;
        if (in)
            x0 = fminf(x0, u), x1 = fmaxf(x1, u), y0 = fminf(y0, v), y1 = fmaxf(y1, v);
    }
    if (0.5f * (pts[11][2] + pts[12][2]) < 0.6f || y1 - y0 < 1.5f * (x1 - x0))
        return 0;
    memset(c, 0, sizeof *c);
    snprintf(c->id, sizeof c->id, "%s-p%d", node, pid);
    snprintf(c->cls, sizeof c->cls, "person");
    memcpy(c->pts, pts, sizeof c->pts);
    c->has_pts = 1;
    c->has_box = 1;
    c->box[0] = roundf(x0), c->box[1] = roundf(y0), c->box[2] = roundf(x1 - x0), c->box[3] = roundf(y1 - y0);
    c->conf = 0.9f;
    float hu = 0.5f * (pts[11][0] + pts[12][0]) * W;
    c->bearing_deg = atanf((hu - W / 2.0f) / fp) * RAD2DEG;
    float hpx;
    if (pts[1][2] > 0.5f && pts[2][2] > 0.5f && pts[15][2] > 0.5f && pts[16][2] > 0.5f)
        hpx = hypotf((pts[1][0] + pts[2][0] - pts[15][0] - pts[16][0]) * 0.5f * W,
                     (pts[1][1] + pts[2][1] - pts[15][1] - pts[16][1]) * 0.5f * H) / EYE_ANKLE;
    else if (pts[5][2] > 0.5f && pts[6][2] > 0.5f)
        hpx = hypotf((pts[5][0] + pts[6][0] - pts[11][0] - pts[12][0]) * 0.5f * W,
                     (pts[5][1] + pts[6][1] - pts[11][1] - pts[12][1]) * 0.5f * H) / 0.30f;
    else
        hpx = c->box[3];
    c->has_range = 1;
    c->range_m = roundf(fp * 1.7f / hpx * 100.0f) / 100.0f;
    return 1;
}

typedef struct {
    float x, y;
    int seen;   /* R1 sees them (not behind the wall) */
    int crouch; /* crouched from 1.5 s on */
} person;

/* What a live phone does to R1's view of someone it sees: no contact in `miss` of its messages
 * (the page's confidence cut), and in `dip` of them one hip under visibility 0.5 (the pose model
 * on a body the frame cuts at the legs), which fusion reads as part of a body. */
typedef struct {
    float miss, dip;
    int handoff, guard; /* fusion and HUD handoff, fusion occlusion guard (1: the defaults) */
} live_opts;
static const live_opts CLEAN = { 0, 0, 1, 1 };

static unsigned g_seed;
static float urand(void)
{
    g_seed = g_seed * 1103515245u + 12345u;
    return (float)((g_seed >> 8) & 0xffffff) / 16777216.0f;
}

typedef struct {
    int frames;         /* frames the person was drawn in the measured second */
    int direct;         /* of those, drawn as SEEN */
    float box_h, box_w; /* mean drawn box, canvas px */
    float skel_h;       /* mean ghost skeleton top to bottom joint, canvas px (0: none drawn) */
    float want_h, want_w, want_skel;
    int big;            /* frames the box was over 1.10 of want_h */
    int ghost;          /* frames a ghost skeleton was drawn for them */
    float worst;        /* largest box height over want_h in one frame */
    int sent;           /* R1's page sent them (0: an x-ray person, wants as for one) */
    float pitch;        /* the tilt the x-ray projection ended up with, deg */
} drawn;

/* Runs the scene for 5 s and measures what is drawn for each person in the last second. */
static void run(const setup *su, const live_opts *lo, const person *ps, int np, drawn *out)
{
    static seeker_config cfg;
    seeker_config_load(&cfg, cfg_path());
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    f.handoff = lo->handoff;
    f.occl_guard = lo->handoff && lo->guard;
    g_seed = 7;
    seeker_pose p1 = { su->r1.x, su->r1.y, su->r1.yaw }, p2 = { su->r2.x, su->r2.y, su->r2.yaw };
    seeker_fusion_set_pose(&f, "R1", &p1, "cli", 1);
    seeker_fusion_set_pose(&f, "R2", &p2, "cli", 1);
    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_load(&v.cfg, &cfg, cfg_path(), "R1");
    v.cfg.inset = 0;
    v.cfg.handoff = lo->handoff;
    v.cfg.n_walls = 1;
    memcpy(v.cfg.walls[0], WALL, sizeof WALL);
    if (su->live) { /* scripts/run-live.sh, its defaults */
        seeker_fusion_set_hfov(&f, "R1", "iphone16");
        seeker_fusion_set_hfov(&f, "R2", "iphone17pro");
        f.person_h = 1.75f;
        seeker_fusion_set_cam(&f, "R2", 1.27f, 0.0f);
        v.cfg.cam.cam_h_m = 1.30f;
        v.cfg.cam.pitch_deg = 0.0f;
        v.cfg.cam.person_h_m = 1.75f;
        v.cfg.xray_up_deg = 0.0f;
        v.cfg.pitch_auto = 1;
    }
    enum { CW = 1280, CH = 720 };
    static uint32_t px[CW * CH], vid[CW * CH];
    canvas c = { px, CW, CH, CW };
    video_frame vf;
    memset(&vf, 0, sizeof vf);
    vf.px = vid, vf.w = CW, vf.h = CH, vf.stride = CW;
    vf.src_w = su->r1.w, vf.src_h = su->r1.h_px;
    vf.rect[2] = CH * su->r1.w / su->r1.h_px, vf.rect[3] = CH, vf.rect[0] = (CW - vf.rect[2]) / 2;
    snprintf(vf.node_id, sizeof vf.node_id, "R1");
    float scale = (float)vf.rect[3] / (float)su->r1.h_px;

    memset(out, 0, sizeof *out * (size_t)np);
    float r1_box_h[8] = { 0 }, r1_box_w[8] = { 0 };
    static seeker_tracks tracks, latest;
    char buf[SEEKER_MAX_DATAGRAM], err[64];
    int have = 0;
    for (int64_t now = 0; now < 5000; now++) {
        for (int n = 0; n < 2; n++) {
            if (now % 66 != (n ? 33 : 0))
                continue;
            const camera *cam = n ? &su->r2 : &su->r1;
            seeker_remote_contacts m;
            memset(&m, 0, sizeof m);
            snprintf(m.node_id, sizeof m.node_id, "%s", n ? "R2" : "R1");
            m.v = 2;
            m.t_mono_ms = now + (n ? 90000 : 50000);
            m.width = cam->w, m.height = cam->h_px, m.hfov_deg = PHONE_HFOV;
            for (int i = 0; i < np; i++) {
                if (!n && !ps[i].seen)
                    continue;
                if (!phone_sees(cam, m.node_id, i + 1, ps[i].x, ps[i].y, su->person_h, ps[i].crouch && now >= 1500, &m.c[m.n]))
                    continue;
                if (!n) {
                    r1_box_h[i] = m.c[m.n].box[3], r1_box_w[i] = m.c[m.n].box[2];
                    float u = urand();
                    if (u < lo->miss)
                        continue;
                    if (u < lo->miss + lo->dip)
                        m.c[m.n].pts[11][2] = 0.45f;
                }
                m.n++;
            }
            seeker_fusion_ingest(&f, &m, now);
        }
        if (now % 33 == 0) {
            seeker_fusion_tick(&f, now, &tracks);
            int len = seeker_encode_tracks(&tracks, buf, sizeof buf);
            if (len > 0 && seeker_decode_tracks(buf, len, &latest, err, sizeof err) == 0)
                have = 1;
        }
        if (now % 33 != 11 || !have)
            continue;
        v.clock_ms = now + 1;
        wv_render(&c, &cfg, &v, &latest, 0, &vf);
        if (now < 4000)
            continue;
        /* Which drawn box is which person: the track nearest to them in the room. */
        for (int i = 0; i < np; i++) {
            int id = -1;
            float best = 1.0f;
            for (int k = 0; k < latest.n; k++) {
                const seeker_track *t = &latest.tracks[k];
                float d = t->has_room ? hypotf(t->room_x_m - ps[i].x, t->room_y_m - ps[i].y) : 9.0f;
                if (d < best)
                    best = d, id = t->id;
            }
            for (int d = 0; d < v.last.n_drawn; d++) {
                if (v.last.drawn[d].id != id)
                    continue;
                out[i].frames++;
                out[i].direct += v.last.drawn[d].direct;
                out[i].box_h += (float)v.last.drawn[d].h;
                out[i].box_w += (float)v.last.drawn[d].w;
                out[i].skel_h += (float)(v.last.drawn[d].skel_y1 - v.last.drawn[d].skel_y0);
                out[i].ghost += v.last.drawn[d].skel_y1 > v.last.drawn[d].skel_y0;
                if (r1_box_h[i] > 0) {
                    float k = (float)v.last.drawn[d].h / (r1_box_h[i] * scale);
                    out[i].big += k > 1.10f;
                    out[i].worst = fmaxf(out[i].worst, k);
                }
            }
            out[i].pitch = v.last.pitch_deg;
        }
    }
    for (int i = 0; i < np; i++) {
        if (out[i].frames) {
            out[i].box_h /= (float)out[i].frames;
            out[i].box_w /= (float)out[i].frames;
            out[i].skel_h /= (float)out[i].frames;
        }
        float a = su->r1.yaw * DEG2RAD;
        float z = (ps[i].x - su->r1.x) * sinf(a) + (ps[i].y - su->r1.y) * cosf(a);
        float f1 = focal(su->r1.w, su->r1.hfov) * scale, mid = (float)CH / 2.0f;
        float tall = f1 * su->person_h / z;
        /* the hull is the person from the floor to the top of the head, kept on the picture */
        float top = fmaxf(mid - f1 * (su->person_h - su->r1.h) / z, -4.0f);
        float bot = fminf(mid + f1 * su->r1.h / z, (float)CH + 4.0f);
        out[i].sent = r1_box_h[i] > 0;
        if (out[i].sent) {
            out[i].want_h = r1_box_h[i] * scale;
            out[i].want_w = r1_box_w[i] * scale;
        } else {
            out[i].want_h = bot - top;
            out[i].want_skel = EYE_ANKLE * tall;
        }
    }
}

static int within(float got, float want, float tol)
{
    return want > 0 && fabsf(got / want - 1.0f) <= tol;
}

static void check(const setup *su, const char *what, const person *pp, const drawn *d)
{
    person seen_as = *pp, *p = &seen_as;
    char note[64];
    snprintf(note, sizeof note, "%s%s", what, p->seen && !d->sent ? ", R1 drops" : "");
    what = note;
    p->seen = p->seen && d->sent;
    int ok = d->frames >= 25 && (p->seen ? d->direct == d->frames : d->direct == 0) &&
             within(d->box_h, d->want_h, 0.10f);
    if (p->seen)
        ok = ok && within(d->box_w, d->want_w, 0.10f);
    else
        ok = ok && within(d->skel_h, d->want_skel, 0.10f);
    float r = hypotf(p->x - su->r1.x, p->y - su->r1.y);
    if (p->seen)
        printf("%s %-18s %-24s %.1f m SEEN box %3.0f x %3.0f px, want %3.0f x %3.0f (x%.2f tall)\n",
               ok ? "ok  " : "FAIL", su->name, what, r, d->box_w, d->box_h, d->want_w, d->want_h,
               d->want_h > 0 ? d->box_h / d->want_h : 0);
    else
        printf("%s %-18s %-24s %.1f m XRAY hull %3.0f px tall, want %3.0f (x%.2f); skeleton %3.0f, want %3.0f "
               "(x%.2f); tilt %+.1f deg\n",
               ok ? "ok  " : "FAIL", su->name, what, r, d->box_h, d->want_h,
               d->want_h > 0 ? d->box_h / d->want_h : 0, d->skel_h, d->want_skel,
               d->want_skel > 0 ? d->skel_h / d->want_skel : 0, d->pitch);
    if (!ok && (d->frames < 25 || (p->seen ? d->direct != d->frames : d->direct != 0)))
        printf("     drawn in %d frames of 30, %d as SEEN\n", d->frames, d->direct);
    fails += !ok;
}

int main(int argc, char **argv)
{
    verbose = argc > 1 && !strcmp(argv[1], "-v");
    if (!verbose) {
        FILE *n = freopen("/dev/null", "w", stderr); /* fusion's log lines */
        (void)n;
    }
    draw_set_theme(DRAW_THEME_PLAN);
    anim_set_enabled(0);
    float l16, p16, l17, p17;
    seeker_hfov_preset("iphone16", &l16, &p16);
    seeker_hfov_preset("iphone17pro", &l17, &p17);
    const setup setups[] = {
        { "default", 0, 1.70f, { 0, 0, 0, 1.70f, 66.0f, 640, 480 }, { 6.5f, 3.4f, -90.0f, 1.70f, 66.0f, 640, 480 } },
        { "run-live", 1, 1.75f, { 0, 0, 0, 1.30f, l16, 640, 480 }, { 6.5f, 3.4f, -90.0f, 1.27f, l17, 640, 480 } },
        { "run-live, portrait", 1, 1.75f, { 0, 0, 0, 1.30f, p16, 480, 640 }, { 6.5f, 3.4f, -90.0f, 1.27f, p17, 480, 640 } },
    };
    static const float ranges[] = { 2.0f, 3.0f, 5.0f };
    printf("drawn sizes on a 1280x720 HUD over R1's picture (each within 10 percent):\n");
    for (size_t s = 0; s < sizeof setups / sizeof setups[0]; s++) {
        const setup *su = &setups[s];
        for (size_t i = 0; i < sizeof ranges / sizeof ranges[0]; i++) {
            float r = ranges[i];
            drawn d[2];
            person in = { -0.6f, sqrtf(r * r - 0.36f), 1, 0 }, hid = { 0.5f, sqrtf(r * r - 0.25f), 0, 0 };
            run(su, &CLEAN, &in, 1, d);
            check(su, "in view", &in, &d[0]);
            run(su, &CLEAN, &hid, 1, d);
            check(su, "behind the wall", &hid, &d[0]);
            /* Both at once: R1's own sighting of the one in view teaches pitch-auto the tilt the
             * hidden one is then drawn with. */
            person both[2] = { in, hid };
            run(su, &CLEAN, both, 2, d);
            check(su, "in view (two people)", &both[0], &d[0]);
            check(su, "behind (two people)", &both[1], &d[1]);
        }
    }
    /* Live: someone standing close in front of R1, the frame cutting their legs (ankles below the
     * picture), R2 seeing all of them. R1's page sends nobody in 12 percent of its messages and
     * one hip under visibility 0.5 in 30 percent (the pose model at the frame's edge). They are in
     * plain view: SEEN, in R1's own box, every frame, and never an x-ray ghost. */
    static const live_opts LIVE = { 0.12f, 0.30f, 1, 1 };
    static const float cut_m[] = { 2.6f, 2.2f, 2.6f }; /* where the frame cuts the legs */
    for (size_t s = 0; s < sizeof setups / sizeof setups[0]; s++) {
        const setup *su = &setups[s];
        drawn d[2];
        person in = { -0.4f, cut_m[s], 1, 0 };
        run(su, &LIVE, &in, 1, d);
        int ok = d[0].frames >= 25 && d[0].direct == d[0].frames && d[0].ghost == 0 && d[0].big == 0;
        printf("%s %-18s %-24s %.1f m SEEN in %d of %d frames, x-ray ghost in %d; box %3.0f px tall, want %3.0f "
               "(R1's own), over 1.10 of it in %d frames (worst x%.2f)\n",
               ok ? "ok  " : "FAIL", su->name, "legs cut, live", cut_m[s], d[0].direct, d[0].frames, d[0].ghost,
               d[0].box_h, d[0].want_h, d[0].big, d[0].worst);
        fails += !ok;
        /* The tilt pitch-auto learns must come from whole standing people only: someone R1 sees
         * crouched (or cut by the frame) is not where the projected standing box is, and the tilt
         * that explains it draws everyone behind the wall too big. */
        person two[2] = { { -0.6f, 2.9f, 1, 1 }, { 0.5f, 2.9f, 0, 0 } };
        run(su, &CLEAN, two, 2, d);
        check(su, "behind, other crouched", &two[1], &d[1]);
        person cut[2] = { { -0.4f, cut_m[s < 2 ? s : 1], 1, 0 }, { 0.5f, 2.9f, 0, 0 } };
        run(su, &CLEAN, cut, 2, d);
        check(su, "behind, other legs cut", &cut[1], &d[1]);
    }
    printf("%s\n", fails ? "size targets missed" : "all size targets met");
    return fails ? 1 : 0;
}
