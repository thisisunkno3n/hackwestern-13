/* Host unit tests for the JSON reader, IF-5.4 decode, tracks encode, node poses, triangulation,
 * the fusion rules, the IF-5.6 video path (SKV2 framing, JPEG decode, letterbox scale, TCP
 * receiver) and the EagleEye wearer view (projection, x-ray classification, mirror). */

#include "anim.h"
#include "anim_alert.h"
#include "anim_map.h"
#include "hud_render.h"
#include "latency.h"
#include "anim_person.h"
#include "plan.h"
#include "projection.h"
#include "seeker_config.h"
#include "seeker_fusion.h"
#include "seeker_json.h"
#include "seeker_msgs.h"
#include "seeker_pose.h"
#include "seeker_triangulate.h"
#include "seeker_time.h"
#include "seeker_udp.h"
#include "sink.h"
#include "video_decode.h"
#include "video_proto.h"
#include "video_rx.h"
#include "wearer_view.h"

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

/* stb_image_write only makes test JPEGs here; hud_display never links it. */
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wall"
#pragma clang diagnostic ignored "-Wextra"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wsign-compare"
#endif
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STBI_WRITE_NO_STDIO
#include "../third_party/stb/stb_image_write.h"
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

static int g_fail, g_pass;

#define CHECK(cond)                                                                                \
    do {                                                                                           \
        if (cond) {                                                                                \
            g_pass++;                                                                              \
        } else {                                                                                   \
            g_fail++;                                                                              \
            fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);                        \
        }                                                                                          \
    } while (0)
#define NEAR(a, b, eps) (fabs((double)(a) - (double)(b)) < (eps))

/* Exactly what tools/sim/remote.py (main bc1ae5a) sends. */
static const char *SIM =
    "{\"type\": \"remote_contacts\", \"v\": 1, \"src\": \"R1\", \"t_mono_ms\": 123456789, "
    "\"node_id\": \"R1\", \"pose\": {\"x_m\": 0.0, \"y_m\": 0.0, \"yaw_deg\": 0.0}, "
    "\"contacts\": [{\"id\": \"R1-1\", \"cls\": \"person\", \"conf\": 0.9, "
    "\"box\": [540, 216, 144, 360], \"bearing_deg\": -12.0, \"range_m\": 2.8}]}";

/* What remote/ws_ingest.py (PR #1) emits for a phone.html bearing-only message. */
static const char *WS =
    "{\"type\": \"remote_contacts\", \"v\": 1, \"src\": \"R2\", \"t_mono_ms\": 5, "
    "\"node_id\": \"R2\", \"pose\": {\"x_m\": 2, \"y_m\": 3, \"yaw_deg\": 215}, "
    "\"contacts\": [{\"conf\": 0.9, \"box\": [0, 0, 0, 0], \"bearing_deg\": 4.5, "
    "\"range_m\": null, \"id\": \"R2-1\", \"cls\": \"person\"}]}";

static void test_json(void)
{
    sj_tok t[64];
    const char *s = "{\"a\": [1, 2.5, -3e2], \"b\": {\"c\": \"x\\\"y\"}, \"d\": null, \"e\": true}";
    int n = sj_parse(s, (int)strlen(s), t, 64);
    CHECK(n > 0);
    int a = sj_get(s, t, 0, "a");
    CHECK(t[a].size == 3);
    CHECK(NEAR(sj_num(s, t, sj_index(t, a, 2), 0), -300, 1e-9));
    char out[8];
    CHECK(sj_str(s, t, sj_get(s, t, sj_get(s, t, 0, "b"), "c"), out, sizeof out) == 0);
    CHECK(!strcmp(out, "x\"y"));
    CHECK(sj_is_null(t, sj_get(s, t, 0, "d")));
    CHECK(sj_bool(s, t, sj_get(s, t, 0, "e"), 0) == 1);
    CHECK(sj_get(s, t, 0, "missing") == -1);
    const char *bad[] = { "{", "{\"a\":}", "[1,]", "{\"a\":1} x", "nul", "" };
    for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; i++)
        CHECK(sj_parse(bad[i], (int)strlen(bad[i]), t, 64) < 0);
    CHECK(sj_parse("[1,2,3]", 7, t, 2) < 0); /* token budget enforced */
}

static void test_decode(void)
{
    seeker_remote_contacts m;
    char err[64];
    CHECK(seeker_decode_remote_contacts(SIM, (int)strlen(SIM), &m, err, sizeof err) == 0);
    CHECK(!strcmp(m.node_id, "R1") && m.n == 1 && m.has_pose);
    CHECK(m.c[0].has_box && m.c[0].has_range && NEAR(m.c[0].range_m, 2.8, 1e-5));
    CHECK(NEAR(m.c[0].bearing_deg, -12.0, 1e-5));

    CHECK(seeker_decode_remote_contacts(WS, (int)strlen(WS), &m, err, sizeof err) == 0);
    CHECK(m.n == 1 && !m.c[0].has_box && !m.c[0].has_range);
    CHECK(NEAR(m.yaw_deg, 215, 1e-5));

    const char *empty = "{\"type\":\"remote_contacts\",\"v\":1,\"node_id\":\"R1\",\"contacts\":[]}";
    CHECK(seeker_decode_remote_contacts(empty, (int)strlen(empty), &m, err, sizeof err) == 0);
    CHECK(m.n == 0 && !m.has_pose);

    const char *no_node = "{\"type\":\"remote_contacts\",\"v\":1,\"contacts\":[]}";
    CHECK(seeker_decode_remote_contacts(no_node, (int)strlen(no_node), &m, err, sizeof err) < 0);
    const char *v3 = "{\"type\":\"remote_contacts\",\"v\":3,\"node_id\":\"R1\",\"contacts\":[]}";
    CHECK(seeker_decode_remote_contacts(v3, (int)strlen(v3), &m, err, sizeof err) < 0);
    CHECK(strstr(err, "unsupported v 3") != NULL);
    const char *nob = "{\"type\":\"remote_contacts\",\"node_id\":\"R1\","
                      "\"contacts\":[{\"conf\":0.5},{\"bearing_deg\":3}],\"extra\":{\"x\":[1]}}";
    CHECK(seeker_decode_remote_contacts(nob, (int)strlen(nob), &m, err, sizeof err) == 0);
    CHECK(m.n == 1 && !strcmp(m.c[0].id, "R1-1")); /* bearing-less contact skipped, id filled */
}

/* INTERFACES.md v2, byte for byte what remote/ws_ingest.py (daniel/v2-conform, PR #2) emits
 * for phone.html: json.dumps of the bridge dict, contacts with id and cls appended. */
static const char *V2_ONE = /* 292 bytes, pose null */
    "{\"type\": \"remote_contacts\", \"v\": 2, \"src\": \"R1\", \"t_mono_ms\": 4829105, "
    "\"ts_ms\": 4829105, \"node_id\": \"R1\", \"pose\": null, \"width\": 640, \"height\": 480, "
    "\"hfov_deg\": 66.0, \"contacts\": [{\"bearing_deg\": -8.3, \"conf\": 0.87, "
    "\"box\": [212, 96, 118, 301], \"range_m\": 3.1, \"id\": \"R1-1\", \"cls\": \"person\"}]}";
static const char *V2_TWO = /* 430 bytes, pose set, second contact all nulls */
    "{\"type\": \"remote_contacts\", \"v\": 2, \"src\": \"R2\", \"t_mono_ms\": 4829105, "
    "\"ts_ms\": 4829105, \"node_id\": \"R2\", \"pose\": {\"x_m\": -2.0, \"y_m\": 3.0, "
    "\"yaw_deg\": 145.0}, \"width\": 1280, \"height\": 720, \"hfov_deg\": 68.5, \"contacts\": "
    "[{\"bearing_deg\": -8.3, \"conf\": 0.87, \"box\": [212, 96, 118, 301], \"range_m\": 3.1, "
    "\"id\": \"R2-1\", \"cls\": \"person\"}, {\"bearing_deg\": 21.4, \"conf\": 0.62, "
    "\"box\": null, \"range_m\": null, \"id\": \"R2-2\", \"cls\": \"person\"}]}";
/* Before the phone reports its frame: every optional field null, ts_ms only. */
static const char *V2_NULLS =
    "{\"type\": \"remote_contacts\", \"v\": 2, \"src\": \"R1\", \"ts_ms\": 77, "
    "\"node_id\": \"R1\", \"pose\": null, \"width\": null, \"height\": null, \"hfov_deg\": null, "
    "\"contacts\": [{\"bearing_deg\": 4.5, \"conf\": null, \"box\": null, \"range_m\": null, "
    "\"id\": null, \"cls\": null}, {\"bearing_deg\": null, \"conf\": 0.5}]}";

static void test_decode_v2(void)
{
    seeker_remote_contacts m;
    char err[64] = "";
    CHECK(strlen(V2_ONE) == 292 && strlen(V2_TWO) == 430);
    CHECK(seeker_decode_remote_contacts(V2_ONE, (int)strlen(V2_ONE), &m, err, sizeof err) == 0);
    CHECK(m.v == 2 && m.n == 1 && !m.has_pose && m.t_mono_ms == 4829105);
    CHECK(m.width == 640 && m.height == 480 && NEAR(m.hfov_deg, 66.0, 1e-5));
    CHECK(m.c[0].has_box && m.c[0].has_range && NEAR(m.c[0].range_m, 3.1, 1e-5));

    CHECK(seeker_decode_remote_contacts(V2_TWO, (int)strlen(V2_TWO), &m, err, sizeof err) == 0);
    CHECK(m.n == 2 && m.has_pose && NEAR(m.yaw_deg, 145, 1e-5) && NEAR(m.x_m, -2, 1e-5));
    CHECK(NEAR(m.c[1].bearing_deg, 21.4, 1e-4) && !m.c[1].has_box && !m.c[1].has_range);
    CHECK(!strcmp(m.c[1].id, "R2-2") && NEAR(m.hfov_deg, 68.5, 1e-5));

    CHECK(seeker_decode_remote_contacts(V2_NULLS, (int)strlen(V2_NULLS), &m, err, sizeof err) == 0);
    CHECK(m.n == 1 && m.t_mono_ms == 77 && !m.has_pose && m.width == 0 && m.hfov_deg == 0);
    CHECK(!strcmp(m.c[0].id, "R1-1") && !strcmp(m.c[0].cls, "person") && m.c[0].conf == 0);

    const char *cnull = "{\"type\":\"remote_contacts\",\"v\":2,\"node_id\":\"R1\",\"contacts\":null}";
    CHECK(seeker_decode_remote_contacts(cnull, (int)strlen(cnull), &m, err, sizeof err) == 0);
    CHECK(m.n == 0);

    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    seeker_decode_remote_contacts(V2_TWO, (int)strlen(V2_TWO), &m, err, sizeof err);
    int i = seeker_fusion_ingest(&f, &m, 1000);
    CHECK(i >= 0 && f.nodes[i].image_w == 1280 && NEAR(f.nodes[i].hfov_deg, 68.5, 1e-5));
    CHECK(NEAR(f.nodes[i].yaw_deg, 145, 1e-5));
    static seeker_tracks t;
    seeker_fusion_tick(&f, 1010, &t);
    CHECK(t.n == 2);
}

/* phone.html on daniel/phone-smoothing (b806419) through ws_ingest: phone ids "R1-p<n>", a new
 * range_src field (string or null), up to 16 contacts (maxppl default 5). */
static int phone_smoothing_msg(char *buf, int size, int n, const char *id_prefix)
{
    int len = snprintf(buf, (size_t)size,
                       "{\"type\": \"remote_contacts\", \"v\": 2, \"src\": \"R1\", \"t_mono_ms\": 900, "
                       "\"ts_ms\": 900, \"node_id\": \"R1\", \"pose\": {\"x_m\": 0, \"y_m\": 0, "
                       "\"yaw_deg\": null}, \"width\": 480, \"height\": 640, \"hfov_deg\": 66.0, "
                       "\"contacts\": [");
    static const char *const src[] = { "\"full\"", "\"torso\"", "\"box\"", "null" };
    for (int i = 0; i < n && len < size; i++)
        len += snprintf(buf + len, (size_t)(size - len),
                        "%s{\"id\": \"%s%d\", \"bearing_deg\": %.1f, \"conf\": 0.91, "
                        "\"box\": [%d, 80, 60, 300], \"range_m\": %s, \"range_src\": %s, "
                        "\"cls\": \"person\"}",
                        i ? ", " : "", id_prefix, 100 + i, -25.0 + 4.0 * i, 10 + 28 * i,
                        i % 4 == 3 ? "null" : "3.27", src[i % 4]);
    len += snprintf(buf + len, (size_t)(size - len), "]}");
    return len;
}

static void test_decode_phone_smoothing(void)
{
    static char buf[SEEKER_MAX_DATAGRAM];
    seeker_remote_contacts m;
    char err[64] = "";
    int len = phone_smoothing_msg(buf, sizeof buf, 6, "R1-p");
    CHECK(seeker_decode_remote_contacts(buf, len, &m, err, sizeof err) == 0);
    CHECK(m.n == 6 && !strcmp(m.c[0].id, "R1-p100") && !strcmp(m.c[5].id, "R1-p105"));
    CHECK(m.c[0].has_range && NEAR(m.c[0].range_m, 3.27, 1e-5) && !m.c[3].has_range);
    CHECK(m.c[2].has_box && NEAR(m.c[4].bearing_deg, -9.0, 1e-5) && m.has_pose == 0);

    /* 20 contacts (a phone without the ws_ingest cap): the first 16 are kept, still under 8 KB. */
    len = phone_smoothing_msg(buf, sizeof buf, 20, "R1-p");
    CHECK(len < SEEKER_MAX_DATAGRAM);
    CHECK(seeker_decode_remote_contacts(buf, len, &m, err, sizeof err) == 0 &&
          m.n == SEEKER_MAX_CONTACTS && !strcmp(m.c[15].id, "R1-p115"));

    /* A 23-character id is kept whole; a longer one is cut to 23 characters, never rejected. */
    len = phone_smoothing_msg(buf, sizeof buf, 2, "R1-p1234567890123456");
    CHECK(seeker_decode_remote_contacts(buf, len, &m, err, sizeof err) == 0 && m.n == 2);
    CHECK(!strcmp(m.c[0].id, "R1-p1234567890123456100"));
    len = phone_smoothing_msg(buf, sizeof buf, 1, "R1-p12345678901234567890");
    CHECK(seeker_decode_remote_contacts(buf, len, &m, err, sizeof err) == 0 && m.n == 1 &&
          !strcmp(m.c[0].id, "R1-p1234567890123456789"));

    /* Fusion keeps every one of 6 people as its own track. */
    static seeker_config cfg;
    static seeker_fusion f;
    static seeker_tracks t;
    seeker_config_defaults(&cfg);
    seeker_fusion_init(&f, &cfg);
    len = phone_smoothing_msg(buf, sizeof buf, 6, "R1-p");
    seeker_decode_remote_contacts(buf, len, &m, err, sizeof err);
    seeker_fusion_ingest(&f, &m, 1000);
    seeker_fusion_tick(&f, 1000, &t);
    int ids_distinct = 1;
    for (int i = 0; i < t.n; i++)
        for (int j = i + 1; j < t.n; j++)
            ids_distinct &= t.tracks[i].id != t.tracks[j].id;
    CHECK(t.n == 6 && ids_distinct && !strcmp(t.tracks[5].label, "R1-p105"));
}

/* ---- track smoothing: R1 at the origin facing +y, one person straight ahead at `range` ---- */

static void sm_put(seeker_fusion *f, float range, int64_t t_ms)
{
    seeker_remote_contacts m;
    memset(&m, 0, sizeof m);
    snprintf(m.node_id, sizeof m.node_id, "R1");
    m.v = 2;
    m.t_mono_ms = t_ms;
    m.has_pose = 1;
    m.n = 1;
    snprintf(m.c[0].id, sizeof m.c[0].id, "R1-1");
    snprintf(m.c[0].cls, sizeof m.c[0].cls, "person");
    m.c[0].has_range = 1;
    m.c[0].range_m = range;
    seeker_fusion_ingest(f, &m, t_ms);
}

/* Feeds one datagram at t_ms, ticks, returns the track's room y (or -1 with no track). */
static float sm_step(seeker_fusion *f, float range, int64_t t_ms)
{
    static seeker_tracks t;
    sm_put(f, range, t_ms);
    seeker_fusion_tick(f, t_ms, &t);
    return t.n == 1 && t.tracks[0].has_room ? t.tracks[0].room_y_m : -1.0f;
}

static void test_smoothing(void)
{
    static seeker_config cfg;
    static seeker_fusion f;
    static seeker_tracks t;
    const int64_t P = 66; /* 15 Hz phones */
    seeker_config_defaults(&cfg);
    CHECK(cfg.smooth && NEAR(cfg.smooth_alpha, 0.5, 1e-6) && NEAR(cfg.smooth_gate_m, 1.5, 1e-6));

    /* New track snaps to its first fix; steady input stays put. */
    seeker_fusion_init(&f, &cfg);
    int64_t now = 1000;
    CHECK(NEAR(sm_step(&f, 3.0f, now), 3.0, 1e-5));
    float y = 0;
    for (int i = 0; i < 30; i++)
        y = sm_step(&f, 3.0f, now += P);
    CHECK(NEAR(y, 3.0, 1e-4));
    seeker_fusion_tick(&f, now, &t);
    CHECK(NEAR(t.tracks[0].range_m, 3.0, 1e-4) && NEAR(t.tracks[0].bearing_deg, 0, 1e-3));

    /* A small step is smoothed, then converges. A tick between datagrams moves the track on by
     * its velocity (fusion.predict_ms), toward the new place and not past it; with prediction
     * off it stays put. */
    y = sm_step(&f, 3.5f, now += P);
    CHECK(y > 3.05f && y < 3.45f);
    seeker_fusion_tick(&f, now + 33, &t);
    CHECK(t.tracks[0].room_y_m > y && t.tracks[0].room_y_m < 3.5f);
    cfg.predict_ms = 0;
    seeker_fusion_tick(&f, now + 33, &t);
    CHECK(NEAR(t.tracks[0].room_y_m, y, 1e-6));
    cfg.predict_ms = 100;
    for (int i = 0; i < 40; i++)
        y = sm_step(&f, 3.5f, now += P);
    CHECK(NEAR(y, 3.5, 0.01));

    /* A jump past the 1.5 m gate snaps instead of sliding across the room. */
    CHECK(NEAR(sm_step(&f, 6.0f, now += P), 6.0, 1e-5));

    /* Walking at 1.2 m/s: the constant-velocity filter catches up (small lag). */
    float truth = 2.0f;
    seeker_fusion_init(&f, &cfg);
    for (int i = 0; i < 40; i++, truth += 1.2f * (float)P / 1000.0f)
        y = sm_step(&f, truth, now += P);
    CHECK(fabsf(y - (truth - 1.2f * (float)P / 1000.0f)) < 0.05f);

    /* Noisy fixes (+-0.3 m) come out with much less variance. */
    seeker_fusion_init(&f, &cfg);
    unsigned rng = 12345;
    double vin = 0, vout = 0;
    for (int i = 0; i < 300; i++) {
        rng = rng * 1103515245u + 12345u;
        float z = 3.0f + 0.6f * ((float)((rng >> 16) & 0x7fff) / 32767.0f - 0.5f);
        y = sm_step(&f, z, now += P);
        if (i >= 20) {
            vin += (z - 3.0) * (z - 3.0);
            vout += (y - 3.0) * (y - 3.0);
        }
    }
    CHECK(vout < 0.7 * vin);

    /* LOST resets: 1.6 s of silence, then a fix within the gate is taken as is. */
    seeker_fusion_init(&f, &cfg);
    for (int i = 0; i < 20; i++)
        sm_step(&f, 3.0f, now += P);
    seeker_fusion_tick(&f, now + 1600, &t);
    CHECK(t.n == 0 && !strcmp(t.nodes[0].state, "lost"));
    now += 1700;
    CHECK(NEAR(sm_step(&f, 3.8f, now), 3.8, 1e-5));

    /* dt is clamped: 10 s and 0 s act like 0.25 s and 5 ms, never inf or NaN. */
    seeker_ab ab = { 1, 0, 0, 0, 2.0f };
    CHECK(seeker_ab_update(&ab, 0, 0.5f, 10.0f, &cfg) == 0); /* exactly on the 0.25 s prediction */
    CHECK(NEAR(ab.y, 0.5, 1e-6) && NEAR(ab.vy, 2.0, 1e-6));
    CHECK(seeker_ab_update(&ab, 0, 0.6f, 0.0f, &cfg) == 0 && isfinite(ab.vy) && ab.vy <= 4.0f);
    CHECK(seeker_ab_update(&ab, 0, 0.6f, -3.0f, &cfg) == 0 && isfinite(ab.y));

    /* --no-smooth passes raw positions through. */
    cfg.smooth = 0;
    seeker_fusion_init(&f, &cfg);
    sm_step(&f, 3.0f, now += P);
    CHECK(NEAR(sm_step(&f, 3.5f, now += P), 3.5, 1e-5));
}

static void test_geometry(void)
{
    float x, y;
    seeker_node_to_room(0, 0, 0, 0, 2, &x, &y); /* straight ahead (+y) */
    CHECK(NEAR(x, 0, 1e-5) && NEAR(y, 2, 1e-5));
    seeker_node_to_room(0, 0, 90, 0, 2, &x, &y); /* yaw 90 cw from +y = +x */
    CHECK(NEAR(x, 2, 1e-5) && NEAR(y, 0, 1e-5));
    seeker_node_to_room(-2, 3, 90, 90, 1, &x, &y); /* 180: towards -y */
    CHECK(NEAR(x, -2, 1e-5) && NEAR(y, 2, 1e-5));
    CHECK(NEAR(seeker_wrap180(190), -170, 1e-5) && NEAR(seeker_wrap180(-180), 180, 1e-5));
}

static void test_fusion(void)
{
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    static seeker_tracks t;
    seeker_remote_contacts m;
    char err[64];
    seeker_decode_remote_contacts(SIM, (int)strlen(SIM), &m, err, sizeof err);

    seeker_fusion_tick(&f, 0, &t);
    CHECK(t.n == 0 && t.n_nodes == 2 && !strcmp(t.nodes[0].state, "wait"));

    seeker_fusion_ingest(&f, &m, 1000);
    seeker_fusion_tick(&f, 1010, &t);
    CHECK(t.n == 1 && !strcmp(t.nodes[0].state, "ok"));
    CHECK(t.tracks[0].has_range && NEAR(t.tracks[0].range_m, 2.8, 1e-3));
    CHECK(NEAR(t.tracks[0].bearing_deg, -12.0, 1e-3)); /* node at origin facing +y = wearer */
    CHECK(!strcmp(t.tracks[0].colour, "cyan"));
    int id = t.tracks[0].id;

    seeker_fusion_tick(&f, 1000 + 1500, &t); /* exactly 1500 ms: not yet lost */
    CHECK(!strcmp(t.nodes[0].state, "ok"));
    CHECK(!strcmp(t.tracks[0].colour, "grey")); /* but stale after 500 ms */
    seeker_fusion_tick(&f, 1000 + 1501, &t);
    CHECK(!strcmp(t.nodes[0].state, "lost") && t.n == 0);

    seeker_fusion_ingest(&f, &m, 4000);
    seeker_fusion_tick(&f, 4001, &t);
    CHECK(!strcmp(t.nodes[0].state, "ok") && t.n == 1 && t.tracks[0].id == id);

    /* R2 sees the same person from (2,3): merged into one track. Person at room (0, 2.8)? R1 is
     * at the origin facing +y with bearing -12, so the person is at (-0.582, 2.739). */
    float px = -2.8f * sinf(12.0f * 0.0174533f), py = 2.8f * cosf(12.0f * 0.0174533f);
    float dx = px - 2.0f, dy = py - 3.0f;
    seeker_remote_contacts r2;
    memset(&r2, 0, sizeof r2);
    snprintf(r2.node_id, sizeof r2.node_id, "R2");
    r2.has_pose = 1;
    r2.x_m = 2;
    r2.y_m = 3;
    r2.yaw_deg = 215;
    r2.n = 1;
    snprintf(r2.c[0].id, sizeof r2.c[0].id, "R2-1");
    snprintf(r2.c[0].cls, sizeof r2.c[0].cls, "person");
    r2.c[0].bearing_deg = seeker_wrap180(atan2f(dx, dy) * 57.29578f - 215.0f);
    r2.c[0].has_range = 1;
    r2.c[0].range_m = hypotf(dx, dy) + 0.3f; /* 0.3 m of mono error */
    seeker_fusion_ingest(&f, &r2, 4010);
    seeker_fusion_tick(&f, 4011, &t);
    CHECK(t.n == 1 && !strcmp(t.tracks[0].nodes, "R1+R2"));

    /* R2 bearing-only (range null): its bearing still crosses R1's ray at the person. */
    r2.c[0].has_range = 0;
    seeker_fusion_ingest(&f, &r2, 4020);
    seeker_fusion_tick(&f, 4021, &t);
    CHECK(t.n == 1 && !strcmp(t.tracks[0].nodes, "R1+R2") && t.tracks[0].has_room);
    CHECK(NEAR(t.tracks[0].room_x_m, px, 0.02) && NEAR(t.tracks[0].room_y_m, py, 0.02));

    /* R2 bearing-only and pointing away from R1's person: no fix, kept on its own. */
    r2.c[0].bearing_deg = seeker_wrap180(r2.c[0].bearing_deg + 60.0f);
    seeker_fusion_ingest(&f, &r2, 4030);
    seeker_fusion_tick(&f, 4031, &t);
    CHECK(t.n == 2);
    int nr = t.tracks[0].has_range ? 1 : 0;
    CHECK(!t.tracks[nr].has_room && !t.tracks[nr].has_bearing && !t.tracks[nr].has_range);
    CHECK(!strcmp(t.tracks[nr].node_id, "R2"));
}

/* Room-frame bearing and range from a node pose to a person at (x, y). */
static void aim(const seeker_pose *p, float x, float y, float *bearing, float *range)
{
    float dx = x - p->x_m, dy = y - p->y_m;
    *bearing = seeker_wrap180(atan2f(dx, dy) * 57.29578f - p->yaw_deg);
    *range = hypotf(dx, dy);
}

static seeker_ray ray_to(const seeker_pose *p, float x, float y, int with_range, float range_err,
                         float conf, int node)
{
    float b, r;
    aim(p, x, y, &b, &r);
    seeker_ray ray = { p->x_m, p->y_m, seeker_pose_world_bearing(p, b), with_range, r + range_err,
                       conf, node, 0 };
    return ray;
}

static void test_pose(void)
{
    char id[SEEKER_NODE_LEN], err[96];
    seeker_pose p;
    CHECK(seeker_pose_parse("R1=-2,3,145", id, &p, err, sizeof err) == 0);
    CHECK(!strcmp(id, "R1") && NEAR(p.x_m, -2, 1e-6) && NEAR(p.y_m, 3, 1e-6) &&
          NEAR(p.yaw_deg, 145, 1e-6));
    CHECK(seeker_pose_parse("  R2 = 2.5 , -0.25 ,215.5 ", id, &p, err, sizeof err) == 0);
    CHECK(!strcmp(id, "R2") && NEAR(p.x_m, 2.5, 1e-6) && NEAR(p.y_m, -0.25, 1e-6) &&
          NEAR(p.yaw_deg, 215.5, 1e-6));
    const char *bad[] = { "R1", "R1=1,2", "R1=1,2,3,4", "R1=a,b,c", "=1,2,3", "TOOLONGID=1,2,3",
                          "R1=1,2,nan", "R1=1e999,0,0", "R 1=1,2,3", "R1=1,,3", "" };
    for (unsigned i = 0; i < sizeof bad / sizeof bad[0]; i++)
        CHECK(seeker_pose_parse(bad[i], id, &p, err, sizeof err) < 0 && err[0]);

    static seeker_pose_table t;
    memset(&t, 0, sizeof t);
    seeker_pose a = { 1, 2, 3 }, b = { 4, 5, 6 };
    CHECK(seeker_pose_table_set(&t, "R1", &a, "file") == 0);
    CHECK(seeker_pose_table_set(&t, "R1", &b, "cli") == 0); /* replaces, does not add */
    CHECK(t.n == 1 && NEAR(seeker_pose_table_get(&t, "R1")->pose.x_m, 4, 1e-6));
    CHECK(!strcmp(seeker_pose_table_get(&t, "R1")->src, "cli") && !seeker_pose_table_get(&t, "R9"));

    char path[64] = "/tmp/seeker_poses_XXXXXX";
    int fd = mkstemp(path);
    if (fd < 0) { /* no writable /tmp on this image: use the working folder */
        snprintf(path, sizeof path, "seeker_poses_XXXXXX");
        fd = mkstemp(path);
    }
    FILE *fp = fd >= 0 ? fdopen(fd, "w") : NULL;
    CHECK(fp != NULL);
    if (!fp)
        return;
    fputs("# checkpoint 5 room\n\nR1=-2.0,3.0,145  # left corner\n  R2 = 2,3,215\n", fp);
    fclose(fp);
    memset(&t, 0, sizeof t);
    CHECK(seeker_pose_load_file(&t, path, err, sizeof err) == 2);
    CHECK(t.n == 2 && NEAR(seeker_pose_table_get(&t, "R2")->pose.yaw_deg, 215, 1e-6));
    CHECK(!strcmp(t.e[0].src, "file"));
    fp = fopen(path, "w");
    CHECK(fp != NULL);
    if (!fp)
        return;
    fputs("R1=0,0,0\nR2=oops\n", fp);
    fclose(fp);
    CHECK(seeker_pose_load_file(&t, path, err, sizeof err) < 0 && strstr(err, ":2:") != NULL);
    unlink(path);
    CHECK(seeker_pose_load_file(&t, "/nonexistent/poses.txt", err, sizeof err) < 0);

    /* Transform: yaw 145 from (-2, 3), bearing -2.6, range 3.9 (the IF-5.4 example). */
    seeker_pose r1 = { -2, 3, 145 };
    float x, y;
    seeker_pose_to_world(&r1, -2.6f, 3.9f, &x, &y);
    CHECK(NEAR(x, -2 + 3.9 * sin(142.4 * 0.017453292519943295), 1e-4));
    CHECK(NEAR(y, 3 + 3.9 * cos(142.4 * 0.017453292519943295), 1e-4));
    float x2, y2;
    seeker_node_to_room(-2, 3, 145, -2.6f, 3.9f, &x2, &y2);
    CHECK(NEAR(x, x2, 1e-6) && NEAR(y, y2, 1e-6));
    seeker_pose r2 = { 2, 3, 215 };
    CHECK(NEAR(seeker_pose_world_bearing(&r2, 10), -135, 1e-4)); /* 225 wraps to -135 */
    CHECK(NEAR(seeker_pose_world_bearing(&r1, 40), 185 - 360, 1e-4));
    CHECK(NEAR(seeker_pose_world_bearing(&r1, -145), 0, 1e-4));
}

static void test_triangulate(void)
{
    seeker_tri_opts o;
    seeker_tri_defaults(&o);
    seeker_pose r1 = { -2, 3, 145 }, r2 = { 2, 3, 215 };
    const float px = 0.3f, py = 0.5f;
    seeker_fix fx;

    /* Intersection: two bearing-only rays meet exactly at the person. */
    seeker_ray two[2] = { ray_to(&r1, px, py, 0, 0, 0.9f, 0), ray_to(&r2, px, py, 0, 0, 0.8f, 1) };
    CHECK(seeker_tri_solve(two, 2, &o, &fx) == SEEKER_TRI_INTERSECT);
    CHECK(NEAR(fx.x_m, px, 1e-3) && NEAR(fx.y_m, py, 1e-3) && fx.cost < 1e-6);
    CHECK(fx.cross_deg > 45 && seeker_tri_consistent(two, 2, &o, &fx));

    /* With mono ranges (R1 0.6 m long): the crossing bearings still pin the point. */
    two[0] = ray_to(&r1, px, py, 1, 0.6f, 0.9f, 0);
    two[1] = ray_to(&r2, px, py, 1, -0.2f, 0.8f, 1);
    CHECK(seeker_tri_solve(two, 2, &o, &fx) == SEEKER_TRI_INTERSECT);
    CHECK(hypotf(fx.x_m - px, fx.y_m - py) < 0.15f && seeker_tri_consistent(two, 2, &o, &fx));

    /* R1 says 1.6 m but the bearings cross at 3.4 m: range gate fails (different people). */
    two[0] = ray_to(&r1, px, py, 1, -1.8f, 0.9f, 0);
    seeker_tri_solve(two, 2, &o, &fx);
    CHECK(!seeker_tri_consistent(two, 2, &o, &fx));

    /* Parallel fallback: phones face each other across the person (anti-parallel bearings). */
    seeker_pose a = { -2, 0, 90 }, b = { 2, 0, 270 };
    seeker_ray par[2] = { ray_to(&a, 0.5f, 0, 1, 0.2f, 0.9f, 0), ray_to(&b, 0.5f, 0, 1, -0.1f, 0.9f, 1) };
    CHECK(seeker_tri_solve(par, 2, &o, &fx) == SEEKER_TRI_PARALLEL);
    CHECK(NEAR(fx.y_m, 0, 1e-3) && fx.x_m > 0.5f && fx.x_m < 0.75f);
    CHECK(fx.cross_deg < 1 && seeker_tri_consistent(par, 2, &o, &fx));
    /* Weighted by range and confidence: the near, confident range point wins. */
    par[0] = (seeker_ray){ -2, 0, 90, 1, 2.9f, 1.0f, 0, 0 }; /* says (0.9, 0), confident */
    par[1] = (seeker_ray){ 2, 0, -90, 1, 2.0f, 0.3f, 1, 0 }; /* says (0, 0), low confidence */
    CHECK(seeker_tri_solve(par, 2, &o, &fx) == SEEKER_TRI_PARALLEL);
    CHECK(fx.x_m > 0.7f && fx.x_m < 0.9f); /* near (0.9, 0), not the midpoint 0.45 */
    /* Parallel bearing-only rays (same direction) have no fix. */
    seeker_pose c = { -1, 0, 0 }, d = { 1, 0, 0 };
    seeker_ray same[2] = { ray_to(&c, -1, 4, 0, 0, 0.9f, 0), ray_to(&d, 1, 4, 0, 0, 0.9f, 1) };
    CHECK(seeker_tri_solve(same, 2, &o, &fx) == SEEKER_TRI_NONE);
    /* Diverging bearing-only rays (crossing behind both phones) have no fix either. */
    seeker_ray div[2] = { ray_to(&c, -3, 4, 0, 0, 0.9f, 0), ray_to(&d, 3, 4, 0, 0, 0.9f, 1) };
    CHECK(seeker_tri_solve(div, 2, &o, &fx) == SEEKER_TRI_NONE && !seeker_tri_consistent(div, 2, &o, &fx));

    /* Single node: its range point, or nothing when bearing-only. */
    seeker_ray one = ray_to(&r1, px, py, 1, 0, 0.9f, 0);
    CHECK(seeker_tri_solve(&one, 1, &o, &fx) == SEEKER_TRI_SINGLE);
    CHECK(NEAR(fx.x_m, px, 1e-4) && NEAR(fx.y_m, py, 1e-4));
    one.has_range = 0;
    CHECK(seeker_tri_solve(&one, 1, &o, &fx) == SEEKER_TRI_NONE);

    /* Grouping: two people, both phones, noisy ranges; true pairs win over ghost crossings. */
    const float qx = -0.8f, qy = -1.5f;
    seeker_ray rs[5] = {
        ray_to(&r1, px, py, 1, 0.3f, 0.9f, 0), ray_to(&r1, qx, qy, 1, -0.4f, 0.8f, 0),
        ray_to(&r2, qx, qy, 1, 0.5f, 0.7f, 1), ray_to(&r2, px, py, 1, -0.3f, 0.9f, 1),
    };
    seeker_pose r3 = { 0, -5, 0 };
    rs[4] = ray_to(&r3, qx, qy, 0, 0, 0.9f, 2); /* a third phone sees the second person */
    seeker_tri_cluster cl[5];
    int n = seeker_tri_group(rs, 5, &o, cl, 5);
    CHECK(n == 2);
    CHECK(cl[0].n == 2 && cl[0].member[0] == 0 && cl[0].member[1] == 3);
    CHECK(cl[1].n == 3 && cl[1].member[0] == 1 && cl[1].member[1] == 2 && cl[1].member[2] == 4);
    CHECK(hypotf(cl[0].fix.x_m - px, cl[0].fix.y_m - py) < 0.2f);
    CHECK(hypotf(cl[1].fix.x_m - qx, cl[1].fix.y_m - qy) < 0.2f);
    /* Two contacts from one node are never fused, even on the same spot. */
    seeker_ray dup[2] = { ray_to(&r1, px, py, 1, 0, 0.9f, 0), ray_to(&r1, px, py, 1, 0, 0.9f, 0) };
    CHECK(seeker_tri_group(dup, 2, &o, cl, 2) == 2 && cl[0].n == 1 && cl[1].n == 1);
}

static seeker_remote_contacts phone_msg(const char *node, const seeker_pose *truth, float x, float y,
                                        int with_range)
{
    seeker_remote_contacts m;
    memset(&m, 0, sizeof m);
    snprintf(m.node_id, sizeof m.node_id, "%s", node);
    m.v = 2;
    m.n = 1;
    snprintf(m.c[0].id, sizeof m.c[0].id, "%s-1", node);
    snprintf(m.c[0].cls, sizeof m.c[0].cls, "person");
    m.c[0].conf = 0.9f;
    aim(truth, x, y, &m.c[0].bearing_deg, &m.c[0].range_m);
    m.c[0].has_range = with_range;
    return m;
}

/* Checkpoint 5 through the fusion core: pinned poses, two phones, one fused marker. */
static void test_fusion_poses(void)
{
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    static seeker_tracks t;
    seeker_pose r1 = { -1.5f, 2.0f, 150 }, r2 = { 1.0f, 2.5f, 200 };
    CHECK(seeker_fusion_set_pose(&f, "R1", &r1, "cli", 1) == 0);
    CHECK(seeker_fusion_set_pose(&f, "R2", &r2, "file", 1) == 1);
    const float px = -0.4f, py = -0.6f;

    /* phone.html with no ?pose= on main still sends 0,0,0; a pinned pose ignores it. */
    seeker_remote_contacts m1 = phone_msg("R1", &r1, px, py, 0);
    seeker_remote_contacts m2 = phone_msg("R2", &r2, px, py, 0);
    m1.has_pose = m2.has_pose = 1;
    seeker_fusion_ingest(&f, &m1, 1000);
    seeker_fusion_ingest(&f, &m2, 1005);
    CHECK(NEAR(f.nodes[0].yaw_deg, 150, 1e-6) && !strcmp(f.nodes[0].pose_src, "cli"));
    seeker_fusion_tick(&f, 1010, &t);
    CHECK(t.n == 1 && !strcmp(t.tracks[0].nodes, "R1+R2") && t.tracks[0].has_room);
    CHECK(NEAR(t.tracks[0].room_x_m, px, 1e-3) && NEAR(t.tracks[0].room_y_m, py, 1e-3));
    CHECK(NEAR(t.tracks[0].range_m, hypotf(px, py), 1e-3) && t.tracks[0].has_bearing);
    CHECK(t.tracks[0].range_conf > 0.5f && !strcmp(t.tracks[0].label, "R1-1+R2-1"));
    CHECK(!strcmp(t.tracks[0].node_id, "R1")); /* view_node R1 is primary */
    int id = t.tracks[0].id;

    /* Same person a moment later keeps the same fused id. */
    seeker_fusion_ingest(&f, &m1, 1050);
    seeker_fusion_ingest(&f, &m2, 1055);
    seeker_fusion_tick(&f, 1060, &t);
    CHECK(t.n == 1 && t.tracks[0].id == id);

    /* R2 goes LOST: single-node fallback on R1, its mono range point. */
    m1 = phone_msg("R1", &r1, px, py, 1);
    seeker_fusion_ingest(&f, &m1, 2000);
    seeker_fusion_ingest(&f, &m1, 2600);
    seeker_fusion_tick(&f, 2600, &t);
    CHECK(!strcmp(t.nodes[1].state, "lost") && t.n == 1 && !strcmp(t.tracks[0].nodes, "R1"));
    CHECK(NEAR(t.tracks[0].room_x_m, px, 1e-3) && NEAR(t.tracks[0].room_y_m, py, 1e-3));
    /* R2 resumes: fused again. */
    seeker_fusion_ingest(&f, &m2, 2700);
    seeker_fusion_tick(&f, 2701, &t);
    CHECK(!strcmp(t.nodes[1].state, "ok") && t.n == 1 && !strcmp(t.tracks[0].nodes, "R1+R2"));

    /* Unpinned: a message pose replaces the config pose, unless pose_source is "config". */
    static seeker_fusion g;
    seeker_fusion_init(&g, &cfg);
    seeker_remote_contacts mp = phone_msg("R2", &r2, px, py, 0);
    mp.has_pose = 1;
    mp.x_m = r2.x_m;
    mp.y_m = r2.y_m;
    mp.yaw_deg = r2.yaw_deg;
    seeker_fusion_ingest(&g, &mp, 100);
    CHECK(NEAR(g.nodes[1].x_m, 1.0, 1e-6) && !strcmp(g.nodes[1].pose_src, "message"));
    cfg.pose_from_config = 1;
    seeker_fusion_init(&g, &cfg);
    seeker_fusion_ingest(&g, &mp, 100);
    CHECK(NEAR(g.nodes[1].x_m, 2.0, 1e-6) && !strcmp(g.nodes[1].pose_src, "config"));
    /* A pose for an unconfigured node adds it. */
    seeker_pose r3 = { 0, -4, 0 };
    CHECK(seeker_fusion_set_pose(&g, "R3", &r3, "cli", 1) == 2 && g.n_nodes == 3);
}

/* ---- precision: triangulation weighting, range smoothing, close-together people ---- */

static unsigned g_rng = 2024;
static double urand(void)
{
    g_rng = g_rng * 1103515245u + 12345u;
    return ((g_rng >> 8) & 0xffffff) / 16777216.0 + 1e-9;
}
static double gauss(void)
{
    return sqrt(-2.0 * log(urand())) * cos(6.283185307179586 * urand());
}

/* Room-frame ray from a node position to (x, y) with fractional range error and bearing error. */
static seeker_ray noisy_ray(float nx, float ny, float x, float y, float range_frac_err,
                            float bearing_err_deg, int with_range, int node)
{
    float dx = x - nx, dy = y - ny;
    seeker_ray r = { nx, ny, atan2f(dx, dy) * 57.29578f + bearing_err_deg, with_range,
                     hypotf(dx, dy) * (1.0f + range_frac_err), 0.9f, node, 0 };
    return r;
}

static void test_tri_weighting(void)
{
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    seeker_tri_opts o;
    seeker_tri_defaults(&o);
    /* Config defaults are the solver defaults, and init copies the fusion.tri_* keys. */
    CHECK(NEAR(cfg.tri_bearing_sigma_deg, o.bearing_sigma_deg, 1e-6) &&
          NEAR(cfg.tri_range_sigma_frac, o.range_sigma_frac, 1e-6) &&
          NEAR(cfg.tri_range_sigma_k, o.range_sigma_k, 1e-6) &&
          NEAR(cfg.tri_min_cross_deg, o.min_cross_deg, 1e-6) &&
          NEAR(cfg.tri_min_cross_bearing_only_deg, o.min_cross_bearing_only_deg, 1e-6) &&
          NEAR(cfg.tri_track_prior_sigma_m, o.prior_sigma_m, 1e-6) &&
          NEAR(cfg.tri_track_prior_cap, o.prior_cap, 1e-6));
    static seeker_fusion f;
    cfg.tri_range_sigma_k = 0.11f;
    cfg.tri_min_cross_deg = 20.0f;
    seeker_fusion_init(&f, &cfg);
    CHECK(NEAR(f.tri.range_sigma_k, 0.11, 1e-6) && NEAR(f.tri.min_cross_deg, 20, 1e-6));

    /* Mono range sigma grows with range squared: doubling the range more than doubles it. */
    float s3 = seeker_tri_range_sigma(&o, 3.0f, 1.0f), s6 = seeker_tri_range_sigma(&o, 6.0f, 1.0f);
    CHECK(NEAR(s3, sqrt(0.6 * 0.6 + 0.45 * 0.45), 1e-4) && s6 > 2.5f * s3);

    /* Two phones 4 m apart, person 3.6 m from each (67 deg crossing). Bearings +-2 deg (1 sigma),
     * size ranges +-20% (1 sigma). The fused fix is far tighter than either phone's range point. */
    const float px = 0.0f, py = 0.0f;
    double e_tri = 0, e_size = 0;
    int n_int = 0;
    const int N = 400;
    g_rng = 7;
    for (int i = 0; i < N; i++) {
        seeker_ray two[2] = {
            noisy_ray(-2, 3, px, py, 0.2f * (float)gauss(), 2.0f * (float)gauss(), 1, 0),
            noisy_ray(2, 3, px, py, 0.2f * (float)gauss(), 2.0f * (float)gauss(), 1, 1),
        };
        seeker_fix fx, one;
        n_int += seeker_tri_solve(two, 2, &o, &fx) == SEEKER_TRI_INTERSECT;
        seeker_tri_solve(&two[0], 1, &o, &one);
        e_tri += pow(hypot(fx.x_m - px, fx.y_m - py), 2);
        e_size += pow(hypot(one.x_m - px, one.y_m - py), 2);
    }
    e_tri = sqrt(e_tri / N);
    e_size = sqrt(e_size / N);
    printf("two-node fix: triangulated rms %.3f m, size-range rms %.3f m\n", e_tri, e_size);
    CHECK(n_int == N && e_tri < 0.35 * e_size && e_tri < 0.25);

    /* Rays 10 deg from parallel: ranged rays use the size ranges (bearing intersection is poor);
     * bearing-only rays still intersect down to 5 deg. */
    seeker_ray nar[2] = { noisy_ray(-0.3f, -3, 0, 0.4f, 0, 0, 1, 0),
                          noisy_ray(0.3f, -3, 0, 0.4f, 0, 0, 1, 1) };
    seeker_fix fx;
    CHECK(seeker_tri_solve(nar, 2, &o, &fx) == SEEKER_TRI_PARALLEL && fx.cross_deg < 15);
    CHECK(NEAR(fx.x_m, 0, 0.05) && NEAR(fx.y_m, 0.4, 0.05));
    nar[0].has_range = nar[1].has_range = 0;
    CHECK(seeker_tri_solve(nar, 2, &o, &fx) == SEEKER_TRI_INTERSECT);
    /* At a wide crossing the fix sigma is small; a single phone's is its range sigma. */
    seeker_ray wide[2] = { noisy_ray(-2, 3, px, py, 0, 0, 1, 0), noisy_ray(2, 3, px, py, 0, 0, 1, 1) };
    seeker_tri_solve(wide, 2, &o, &fx);
    seeker_fix one;
    seeker_tri_solve(&wide[0], 1, &o, &one);
    CHECK(fx.sigma_m > 0 && fx.sigma_m < 0.4f * one.sigma_m);
}

/* One node at the origin facing +y, one person at `range`; returns the sent range (or -1). */
static float rs_step(seeker_fusion *f, float range, int64_t t_ms)
{
    static seeker_tracks t;
    sm_put(f, range, t_ms);
    seeker_fusion_tick(f, t_ms, &t);
    return t.n == 1 && t.tracks[0].has_range ? t.tracks[0].range_m : -1.0f;
}

static double range_std_ratio(const seeker_config *cfg, double sigma_frac, unsigned seed)
{
    static seeker_fusion f;
    seeker_fusion_init(&f, cfg);
    g_rng = seed;
    int64_t now = 100000;
    double vin = 0, vout = 0;
    for (int i = 0; i < 400; i++) {
        float z = 3.0f * (1.0f + (float)(sigma_frac * gauss()));
        float y = rs_step(&f, z, now += 66);
        if (i >= 40) {
            vin += (z - 3.0) * (z - 3.0);
            vout += (y - 3.0) * (y - 3.0);
        }
    }
    return sqrt(vout / vin);
}

static void test_range_smoothing(void)
{
    static seeker_config cfg;
    static seeker_fusion f;
    seeker_config_defaults(&cfg);
    CHECK(cfg.range_smooth && NEAR(cfg.range_smooth_alpha, 0.2, 1e-6) &&
          NEAR(cfg.range_smooth_beta, 0.015, 1e-6) && NEAR(cfg.range_smooth_gate_frac, 0.65, 1e-6));

    /* Noisy single-phone range, +-20% as the 95% band (sigma 10%), 15 Hz: jitter drops > 2x. */
    double r10 = range_std_ratio(&cfg, 0.10, 11), r10b = range_std_ratio(&cfg, 0.10, 12);
    double r20 = range_std_ratio(&cfg, 0.20, 11);
    CHECK(r10 < 0.5 && r10b < 0.5);

    /* A clean 1 m step (3 m to 4 m): 90% within 500 ms, then settles. */
    seeker_fusion_init(&f, &cfg);
    int64_t now = 200000, t_step = 0;
    float y = 0, peak = 0;
    for (int i = 0; i < 30; i++)
        y = rs_step(&f, 3.0f, now += 66);
    int t90 = -1, t_settle = -1;
    t_step = now;
    for (int i = 0; i < 60; i++) {
        y = rs_step(&f, 4.0f, now += 66);
        if (y > peak)
            peak = y;
        if (t90 < 0 && y >= 3.9f)
            t90 = (int)(now - t_step);
        if (fabsf(y - 4.0f) > 0.1f)
            t_settle = -1;
        else if (t_settle < 0)
            t_settle = (int)(now - t_step);
    }
    printf("range filter (alpha %.2f beta %.3f, 15 Hz): jitter x%.2f at 10%% sigma (x%.2f at 20%%),"
           " 90%% of a 1 m step in %d ms, overshoot %.2f m, within 0.1 m from %d ms\n",
           cfg.range_smooth_alpha, cfg.range_smooth_beta, r10, r20, t90, peak - 4.0f, t_settle);
    CHECK(t90 > 0 && t90 <= 500 && NEAR(y, 4.0, 0.01) && peak - 4.0f < 0.25f);

    /* Fused (two-phone) tracks keep the room filter: the range filter only drives mono ranges. */
    static seeker_tracks t;
    seeker_fusion_init(&f, &cfg);
    seeker_pose p1 = { -2, 3, 145 }, p2 = { 2, 3, 215 };
    seeker_fusion_set_pose(&f, "R1", &p1, "cli", 1);
    seeker_fusion_set_pose(&f, "R2", &p2, "cli", 1);
    seeker_remote_contacts m1 = phone_msg("R1", &p1, 0, 0, 1), m2 = phone_msg("R2", &p2, 0, 0, 1);
    seeker_fusion_ingest(&f, &m1, 1000);
    seeker_fusion_ingest(&f, &m2, 1000);
    seeker_fusion_tick(&f, 1000, &t);
    CHECK(t.n == 1 && !strcmp(t.tracks[0].nodes, "R1+R2") && !f.mem[0].rf.valid);

    /* Fused, then R2 goes quiet and is dropped: the single-phone track keeps its id and starts
     * from the fused position instead of jumping to R1's +30% mono range point. */
    int id = t.tracks[0].id;
    m1.c[0].range_m *= 1.3f;
    float raw_x, raw_y;
    seeker_pose_to_world(&p1, m1.c[0].bearing_deg, m1.c[0].range_m, &raw_x, &raw_y);
    float last_x = 0, last_y = 0, jump = -1, gap = 0;
    for (int64_t now = 1066; now <= 4500; now += 66) {
        float lx = t.tracks[0].room_x_m, ly = t.tracks[0].room_y_m;
        int was_fused = t.n == 1 && strchr(t.tracks[0].nodes, '+') != NULL;
        seeker_fusion_ingest(&f, &m1, now);
        seeker_fusion_tick(&f, now, &t);
        if (was_fused && t.n == 1 && !strcmp(t.tracks[0].nodes, "R1") && jump < 0) {
            jump = hypotf(t.tracks[0].room_x_m - lx, t.tracks[0].room_y_m - ly);
            gap = hypotf(raw_x - lx, raw_y - ly);
        }
        last_x = t.tracks[0].room_x_m;
        last_y = t.tracks[0].room_y_m;
    }
    CHECK(t.n == 1 && !strcmp(t.tracks[0].nodes, "R1") && t.tracks[0].id == id);
    CHECK(jump >= 0 && gap > 0.5f && jump < 0.35f * gap);
    CHECK(hypotf(last_x - raw_x, last_y - raw_y) < 0.6f * gap); /* and converges toward it */

    /* --debug prints one line per track per datagram (smoke test: no crash). */
    f.debug = 1;
    seeker_fusion_ingest(&f, &m1, 2666);
    seeker_fusion_tick(&f, 2666, &t);
    seeker_fusion_tick(&f, 2690, &t); /* no new datagram: no second line */
    f.debug = 0;
}

static void put_contacts(seeker_fusion *f, const char *node, const seeker_pose *p, const float *xs,
                         const float *ys, int n, const char *const *labels, int with_range,
                         int64_t t_ms)
{
    seeker_remote_contacts m;
    memset(&m, 0, sizeof m);
    snprintf(m.node_id, sizeof m.node_id, "%s", node);
    m.v = 2;
    m.t_mono_ms = t_ms;
    m.n = n;
    for (int i = 0; i < n; i++) {
        snprintf(m.c[i].id, sizeof m.c[i].id, "%s", labels[i]);
        snprintf(m.c[i].cls, sizeof m.c[i].cls, "person");
        m.c[i].conf = 0.9f;
        aim(p, xs[i], ys[i], &m.c[i].bearing_deg, &m.c[i].range_m);
        m.c[i].has_range = with_range;
    }
    seeker_fusion_ingest(f, &m, t_ms);
}

static int track_near(const seeker_tracks *t, float x, float y, float within)
{
    int best = -1;
    float bd = within;
    for (int i = 0; i < t->n; i++) {
        if (!t->tracks[i].has_room)
            continue;
        float d = hypotf(t->tracks[i].room_x_m - x, t->tracks[i].room_y_m - y);
        if (d < bd) {
            bd = d;
            best = i;
        }
    }
    return best;
}

static void test_close_people(void)
{
    static seeker_config cfg;
    static seeker_fusion f;
    static seeker_tracks t;
    seeker_config_defaults(&cfg);
    seeker_tri_opts o;
    seeker_tri_defaults(&o);

    /* 2x2 ghost case: the cheapest single pair is a ghost crossing (A from R1 with B from R2), so
     * greedy pairing would also force the other ghost. The assignment takes the true pairs. */
    seeker_ray rs[4] = {
        noisy_ray(-2, 3, -0.25f, 0, 0.35f, 0, 1, 0), noisy_ray(-2, 3, 0.25f, 0, 0.36f, 0, 1, 0),
        noisy_ray(2, 3, -0.25f, 0, 0.23f, 0, 1, 1), noisy_ray(2, 3, 0.25f, 0, 0.02f, 0, 1, 1),
    };
    seeker_fix fx;
    seeker_ray p_aa[2] = { rs[0], rs[2] }, p_bb[2] = { rs[1], rs[3] };
    seeker_ray p_ab[2] = { rs[0], rs[3] }, p_ba[2] = { rs[1], rs[2] };
    seeker_tri_solve(p_aa, 2, &o, &fx);
    float c_aa = fx.cost;
    seeker_tri_solve(p_bb, 2, &o, &fx);
    float c_bb = fx.cost;
    seeker_tri_solve(p_ab, 2, &o, &fx);
    float c_ab = fx.cost;
    seeker_tri_solve(p_ba, 2, &o, &fx);
    float c_ba = fx.cost;
    CHECK(c_ab < c_aa && c_ab < c_bb && c_aa + c_bb < c_ab + c_ba); /* the trap is set */
    seeker_tri_cluster cl[4];
    CHECK(seeker_tri_group(rs, 4, &o, cl, 4) == 2);
    CHECK(cl[0].n == 2 && cl[0].member[0] == 0 && cl[0].member[1] == 2);
    CHECK(cl[1].n == 2 && cl[1].member[0] == 1 && cl[1].member[1] == 3);
    /* With track priors at the people, the true pairing is preferred even more. */
    seeker_tri_prior pr[2] = { { -0.25f, 0 }, { 0.25f, 0 } };
    CHECK(seeker_tri_group_prior(rs, 4, &o, pr, 2, cl, 4) == 2 && cl[0].member[1] == 2);

    /* Two people 0.5 m apart, both phones see both: two tracks, each on its person. */
    seeker_fusion_init(&f, &cfg);
    seeker_pose p1 = { -2, 3, 145 }, p2 = { 2, 3, 215 };
    seeker_fusion_set_pose(&f, "R1", &p1, "cli", 1);
    seeker_fusion_set_pose(&f, "R2", &p2, "cli", 1);
    const float ax = -0.25f, bx = 0.25f, yy = 0.0f;
    float xs[2] = { ax, bx }, ys[2] = { yy, yy };
    const char *l1[2] = { "R1-1", "R1-2" }, *l2[2] = { "R2-1", "R2-2" };
    for (int i = 0; i < 5; i++) {
        put_contacts(&f, "R1", &p1, xs, ys, 2, l1, 1, 1000 + 66 * i);
        put_contacts(&f, "R2", &p2, xs, ys, 2, l2, 1, 1000 + 66 * i);
        seeker_fusion_tick(&f, 1000 + 66 * i, &t);
    }
    int ia = track_near(&t, ax, yy, 0.15f), ib = track_near(&t, bx, yy, 0.15f);
    CHECK(t.n == 2 && ia >= 0 && ib >= 0 && ia != ib);
    CHECK(t.tracks[0].id != t.tracks[1].id && strchr(t.tracks[0].nodes, '+') &&
          strchr(t.tracks[1].nodes, '+'));

    /* Same-camera exclusion: two contacts of R1 0.15 m apart are never one track, with or
     * without R2 seeing one of them. */
    seeker_fusion_init(&f, &cfg);
    seeker_fusion_set_pose(&f, "R1", &p1, "cli", 1);
    seeker_fusion_set_pose(&f, "R2", &p2, "cli", 1);
    float cx[2] = { 0.0f, 0.15f }, cy[2] = { 0.0f, 0.0f };
    put_contacts(&f, "R1", &p1, cx, cy, 2, l1, 1, 1000);
    seeker_fusion_tick(&f, 1000, &t);
    CHECK(t.n == 2 && t.tracks[0].id != t.tracks[1].id);
    CHECK(!strstr(t.tracks[0].label, "R1-1+R1-2") && !strstr(t.tracks[1].label, "R1-1+R1-2"));
    put_contacts(&f, "R2", &p2, cx, cy, 1, l2, 1, 1010);
    seeker_fusion_tick(&f, 1010, &t);
    CHECK(t.n == 2 && t.tracks[0].id != t.tracks[1].id);
    int fused = 0;
    for (int i = 0; i < t.n; i++)
        fused += !strcmp(t.tracks[i].nodes, "R1+R2");
    CHECK(fused == 1);

    /* Crossing paths: A walks +x, B walks -x 0.4 m behind, 1 m/s, 15 Hz, for 3 s. Each phone
     * lists contacts left to right (so their labels swap as the two pass), and R1 sees only one
     * of them while they overlap within 3 deg. Ids must stay with the people. */
    seeker_fusion_init(&f, &cfg);
    seeker_fusion_set_pose(&f, "R1", &p1, "cli", 1);
    seeker_fusion_set_pose(&f, "R2", &p2, "cli", 1);
    int id_a = -1, id_b = -1, swaps = 0, both = 0, frames = 0;
    for (int i = 0; i <= 45; i++) {
        int64_t now = 5000 + 66 * i;
        float s = -1.5f + 3.0f * (float)i / 45.0f;
        float pa[2] = { s, 0.0f }, pb[2] = { -s, 0.4f };
        const seeker_pose *ps[2] = { &p1, &p2 };
        for (int nd = 0; nd < 2; nd++) {
            float ba, bb, ra, rb;
            aim(ps[nd], pa[0], pa[1], &ba, &ra);
            aim(ps[nd], pb[0], pb[1], &bb, &rb);
            int a_first = ba <= bb;
            float qx[2] = { a_first ? pa[0] : pb[0], a_first ? pb[0] : pa[0] };
            float qy[2] = { a_first ? pa[1] : pb[1], a_first ? pb[1] : pa[1] };
            int n = 2;
            if (nd == 0 && fabsf(ba - bb) < 3.0f) {
                n = 1; /* occluded: R1 reports the nearer one only */
                int a_near = ra < rb;
                qx[0] = a_near ? pa[0] : pb[0];
                qy[0] = a_near ? pa[1] : pb[1];
            }
            put_contacts(&f, nd ? "R2" : "R1", ps[nd], qx, qy, n, nd ? l2 : l1, 1, now);
        }
        seeker_fusion_tick(&f, now, &t);
        int ka = track_near(&t, pa[0], pa[1], 0.35f), kb = track_near(&t, pb[0], pb[1], 0.35f);
        frames++;
        if (ka >= 0 && kb >= 0 && ka != kb)
            both++;
        if (ka >= 0) {
            if (id_a < 0)
                id_a = t.tracks[ka].id;
            else if (t.tracks[ka].id != id_a)
                swaps++;
        }
        if (kb >= 0) {
            if (id_b < 0)
                id_b = t.tracks[kb].id;
            else if (t.tracks[kb].id != id_b)
                swaps++;
        }
    }
    printf("crossing: ids %d and %d, %d id changes, both tracked in %d of %d frames\n", id_a, id_b,
           swaps, both, frames);
    CHECK(id_a > 0 && id_b > 0 && id_a != id_b && swaps == 0 && both >= frames - 6);

    /* Coasting: a person missing for 0.6 s comes back under a new phone label, near where the
     * track was going: same id. Missing longer than drop_ms (1.5 s): a new id. */
    seeker_fusion_init(&f, &cfg);
    float x1[1] = { 0.0f }, y1[1] = { 3.0f };
    const char *la[1] = { "R1-1" }, *lb[1] = { "R1-9" }, *lc[1] = { "R1-4" };
    seeker_pose o1 = { 0, 0, 0 };
    seeker_fusion_set_pose(&f, "R1", &o1, "cli", 1);
    for (int i = 0; i < 10; i++) {
        put_contacts(&f, "R1", &o1, x1, y1, 1, la, 1, 1000 + 66 * i);
        seeker_fusion_tick(&f, 1000 + 66 * i, &t);
    }
    int id0 = t.tracks[0].id;
    put_contacts(&f, "R1", &o1, x1, y1, 0, la, 1, 1700); /* node alive, nobody in view */
    seeker_fusion_tick(&f, 1700, &t);
    CHECK(t.n == 0);
    y1[0] = 3.1f;
    put_contacts(&f, "R1", &o1, x1, y1, 1, lb, 1, 2200);
    seeker_fusion_tick(&f, 2200, &t);
    CHECK(t.n == 1 && t.tracks[0].id == id0);
    put_contacts(&f, "R1", &o1, x1, y1, 0, la, 1, 2300);
    seeker_fusion_tick(&f, 2300, &t);
    put_contacts(&f, "R1", &o1, x1, y1, 0, la, 1, 3500);
    seeker_fusion_tick(&f, 3500, &t);
    put_contacts(&f, "R1", &o1, x1, y1, 1, lc, 1, 4000);
    seeker_fusion_tick(&f, 4000, &t);
    CHECK(t.n == 1 && t.tracks[0].id != id0);
}

static void test_tracks_roundtrip(void)
{
    static seeker_tracks t, back;
    memset(&t, 0, sizeof t);
    snprintf(t.src, sizeof t.src, "hud_fusion");
    snprintf(t.heading_src, sizeof t.heading_src, "none");
    t.n = SEEKER_MAX_TRACKS;
    for (int i = 0; i < t.n; i++) {
        seeker_track *k = &t.tracks[i];
        k->id = i + 1;
        snprintf(k->label, sizeof k->label, "R1-%d+R2-%d", i, i);
        snprintf(k->cls, sizeof k->cls, "person");
        snprintf(k->src, sizeof k->src, "remote");
        snprintf(k->colour, sizeof k->colour, "cyan");
        snprintf(k->node_id, sizeof k->node_id, "R1");
        snprintf(k->nodes, sizeof k->nodes, "R1+R2");
        snprintf(k->range_src, sizeof k->range_src, "remote");
        k->has_range = k->has_bearing = k->has_room = k->has_node_box = 1;
        k->range_m = 12.34f;
        k->bearing_deg = -123.4f;
        k->room_x_m = -5.5f;
        k->room_y_m = 7.25f;
        k->node_box[0] = 100; k->node_box[1] = 50; k->node_box[2] = 40; k->node_box[3] = 160;
    }
    t.n_nodes = SEEKER_MAX_NODES;
    for (int i = 0; i < t.n_nodes; i++) {
        t.nodes[i].node_id[0] = 'R';
        t.nodes[i].node_id[1] = (char)('1' + i);
        t.nodes[i].node_id[2] = '\0';
        snprintf(t.nodes[i].state, sizeof t.nodes[i].state, "lost");
    }
    char buf[SEEKER_MAX_DATAGRAM];
    int len = seeker_encode_tracks(&t, buf, sizeof buf);
    printf("worst-case tracks datagram: %d bytes (limit %d)\n", len, SEEKER_MAX_DATAGRAM);
    CHECK(len > 0 && len < SEEKER_MAX_DATAGRAM);
    char err[64];
    CHECK(seeker_decode_tracks(buf, len, &back, err, sizeof err) == 0);
    CHECK(back.n == t.n && back.n_nodes == t.n_nodes);
    CHECK(NEAR(back.tracks[3].room_y_m, 7.25, 1e-3) && back.tracks[3].has_node_box);
    CHECK(!strcmp(back.nodes[7].state, "lost"));
}

/* SEEKER_SOURCE_DIR (set by the build) lets test_core run from build/ as well as from the
 * source folder; a binary copied to another machine falls back to the cwd. */
static const char *config_path(void)
{
#ifdef SEEKER_SOURCE_DIR
    if (access(SEEKER_SOURCE_DIR "/config/seeker.json", R_OK) == 0)
        return SEEKER_SOURCE_DIR "/config/seeker.json";
#endif
    return "config/seeker.json";
}

static void test_config(void)
{
    static seeker_config c;
    CHECK(seeker_config_load(&c, config_path()) == 0);
    CHECK(c.n_nodes == 2 && c.drop_ms == 1500 && !strcmp(c.background, "radar"));
    CHECK(c.n_children == 2 && !strcmp(c.children[0].name, "hud_fusion"));
    CHECK(c.children[1].prio == 50 && !strcmp(c.children[1].policy, "fifo"));
    static seeker_config d;
    seeker_config_defaults(&d);
    CHECK(c.range_smooth == d.range_smooth && NEAR(c.range_smooth_alpha, d.range_smooth_alpha, 1e-6) &&
          NEAR(c.range_smooth_beta, d.range_smooth_beta, 1e-6) &&
          NEAR(c.tri_range_sigma_k, d.tri_range_sigma_k, 1e-6) &&
          NEAR(c.tri_min_cross_deg, d.tri_min_cross_deg, 1e-6) &&
          NEAR(c.track_gate_max_m, d.track_gate_max_m, 1e-6)); /* seeker.json spells out defaults */
}

/* ---- EagleEye wearer view ---- */

static const char *src_file(const char *rel)
{
#ifdef SEEKER_SOURCE_DIR
    static char buf[512];
    snprintf(buf, sizeof buf, "%s/%s", SEEKER_SOURCE_DIR, rel);
    if (access(buf, R_OK) == 0)
        return buf;
#endif
    return rel;
}

static void test_projection(void)
{
    wv_camera cam = { 90.0f, 1000, 500, 1.7f, 0.0f, 0.0f }; /* f = 500 px */
    wv_pose o = { 0, 0, 0 };
    wv_proj p;
    CHECK(NEAR(wv_focal_px(&cam), 500, 1e-3));

    /* Centre: straight ahead on the boresight. */
    CHECK(wv_project_point(&o, &cam, 0, 4, &p) == 1);
    CHECK(p.in_front && NEAR(p.u, 500, 1e-3) && NEAR(p.bearing_deg, 0, 1e-4));
    CHECK(NEAR(p.range_m, 4, 1e-5) && NEAR(p.z_m, 4, 1e-5));
    CHECK(NEAR(p.box_w, 62.5, 1e-3) && NEAR(p.box_x, 468.75, 1e-3));
    CHECK(NEAR(p.box_y, 250 + 500 * (1.7 - 1.75) / 4, 1e-3)); /* head row */
    CHECK(NEAR(p.box_y + p.box_h, 250 + 500 * 1.7 / 4, 1e-3)); /* feet row */
    CHECK(wv_project_point(&o, &cam, 0, 8, &p) && NEAR(p.box_w, 31.25, 1e-3)); /* half at 2x */

    /* Edges: 45 deg is exactly the image edge, 50 deg is outside; the left edge mirrors it. */
    CHECK(wv_project_point(&o, &cam, 4, 4, &p) == 1 && NEAR(p.u, 1000, 1e-2));
    CHECK(NEAR(p.bearing_deg, 45, 1e-3));
    CHECK(wv_project_point(&o, &cam, -4, 4, &p) == 1 && NEAR(p.u, 0, 1e-2));
    float s50 = sinf(50 * 0.0174533f), c50 = cosf(50 * 0.0174533f);
    CHECK(wv_project_point(&o, &cam, 4 * s50, 4 * c50, &p) == 0 && p.in_front && !p.in_fov);
    CHECK(p.u > 1000);

    /* Same bearing as phone.html / IF-1: atan((u - cx) / f). */
    float s20 = sinf(20 * 0.0174533f), c20 = cosf(20 * 0.0174533f);
    wv_project_point(&o, &cam, 3 * s20, 3 * c20, &p);
    CHECK(NEAR(atanf((p.u - 500) / 500) * 57.29578f, 20, 1e-3));
    CHECK(NEAR(wv_bearing_to_u(&cam, 20), p.u, 1e-2) && NEAR(wv_bearing_to_u(&cam, 0), 500, 1e-3));

    /* Behind the wearer: never projected (no mirrored ghost in front). */
    CHECK(wv_project_point(&o, &cam, 0, -3, &p) == 0 && !p.in_front && !p.in_fov);
    CHECK(NEAR(fabsf(p.bearing_deg), 180, 1e-3) && p.z_m < 0);
    CHECK(wv_project_point(&o, &cam, 0, 0.1f, &p) == 0 && !p.in_front); /* inside WV_NEAR_M */
    CHECK(wv_project_point(&o, &cam, 0.3f, -2, &p) == 0 && p.bearing_deg > 90);

    /* Edge arrows for targets outside the FOV. */
    wv_edge e = wv_edge_for(50);
    CHECK(e.side == 1 && !e.behind && NEAR(e.turn_deg, 50, 1e-4));
    e = wv_edge_for(-120);
    CHECK(e.side == -1 && e.behind && NEAR(e.turn_deg, 120, 1e-4));
    e = wv_edge_for(180);
    CHECK(e.side == 1 && e.behind);
    e = wv_edge_for(-90);
    CHECK(e.side == -1 && !e.behind);
    CHECK(wv_bearing_in_fov(&cam, 45) && !wv_bearing_in_fov(&cam, 46) && !wv_bearing_in_fov(&cam, -170));
    CHECK(wv_bearing_in_fov(&cam, 360 + 10)); /* wrapped */
    CHECK(NEAR(wv_wrap180(540), 180, 1e-4) && NEAR(wv_wrap180(-190), 170, 1e-4));

    /* Wearer pose: at (1,2) facing +x (yaw 90), (5,2) is dead ahead and (1,6) is 90 deg left. */
    wv_pose w = { 1, 2, 90 };
    CHECK(wv_project_point(&w, &cam, 5, 2, &p) == 1 && NEAR(p.u, 500, 1e-2) && NEAR(p.range_m, 4, 1e-4));
    wv_project_point(&w, &cam, 1, 6, &p);
    CHECK(!p.in_front && NEAR(p.bearing_deg, -90, 1e-3) && wv_edge_for(p.bearing_deg).side == -1);

    /* Unset camera fields fall back to 66 deg, 1280 px wide. */
    wv_camera unset = { 0, 0, 0, 0, 0, 0 };
    CHECK(NEAR(wv_focal_px(&unset), 640 / tanf(33 * 0.0174533f), 1e-2));

    /* Viewport: 4:3 phone frames are pillarboxed on 16:9, 16:9 fills it. */
    wv_viewport vp = wv_viewport_fit(640, 480, 1920, 1080);
    CHECK(vp.w == 1440 && vp.h == 1080 && vp.x == 240 && vp.y == 0 && NEAR(vp.s, 2.25, 1e-5));
    vp = wv_viewport_fit(1280, 720, 1920, 1080);
    CHECK(vp.w == 1920 && vp.h == 1080 && vp.x == 0 && NEAR(vp.s, 1.5, 1e-5));
}

static seeker_track *add_track(seeker_tracks *t, const char *nodes, const char *node_id,
                               int has_room, float x, float y)
{
    seeker_track *k = &t->tracks[t->n++];
    memset(k, 0, sizeof *k);
    k->id = t->n;
    snprintf(k->label, sizeof k->label, "%s-%d", node_id, t->n);
    snprintf(k->nodes, sizeof k->nodes, "%s", nodes);
    snprintf(k->node_id, sizeof k->node_id, "%s", node_id);
    snprintf(k->colour, sizeof k->colour, "cyan");
    k->has_room = has_room;
    k->room_x_m = x;
    k->room_y_m = y;
    return k;
}

static void add_node(seeker_tracks *t, const char *id, const char *state, float x, float y, float yaw)
{
    seeker_node_status *n = &t->nodes[t->n_nodes++];
    memset(n, 0, sizeof *n);
    snprintf(n->node_id, sizeof n->node_id, "%s", id);
    snprintf(n->state, sizeof n->state, "%s", state);
    n->x_m = x;
    n->y_m = y;
    n->yaw_deg = yaw;
    n->hfov_deg = 66;
}

static void test_occlusion(void)
{
    CHECK(wv_seen_by("R1+W", "R1", "W") && wv_seen_by("W", "W", "W"));
    CHECK(wv_seen_by("R1+R2+W", "R1", "W") && wv_seen_by("", "W", "W"));
    CHECK(!wv_seen_by("R1", "R1", "W") && !wv_seen_by("W2+R1", "R1", "W"));
    CHECK(!wv_seen_by("RW", "RW", "W") && !wv_seen_by(NULL, "R1", "W") && !wv_seen_by("W", "W", ""));

    static wv_config w;
    wv_config_defaults(&w);
    w.cam.hfov_deg = 66;
    wv_pose o = { 0, 0, 0 };
    static seeker_tracks t;
    memset(&t, 0, sizeof t);
    add_node(&t, "W", "ok", 0, 0, 0);
    add_node(&t, "R1", "ok", -3, 5.5f, 104);
    add_track(&t, "W+R1", "W", 1, 0, 3);        /* 0: seen by the helmet phone */
    add_track(&t, "R1", "R1", 1, 1, 5);         /* 1: behind the wall, in the FOV: x-ray */
    add_track(&t, "R1", "R1", 1, 0.1f, 3.2f);   /* 2: same person as 0, fusion did not merge */
    add_track(&t, "R1", "R1", 1, 0.1f, 6.5f);   /* 3: same bearing as 0 but 3.5 m further: x-ray */
    add_track(&t, "R2", "R2", 1, 0, -4);        /* 4: behind the wearer */
    seeker_track *k5 = add_track(&t, "W", "W", 0, 0, 0); /* 5: helmet bearing only, -15 deg */
    k5->node_bearing_deg = -15;
    float b = -15.5f * 0.0174533f;
    add_track(&t, "R2", "R2", 1, 4 * sinf(b), 4 * cosf(b)); /* 6: same person as 5, with range */
    seeker_track *k7 = add_track(&t, "R2", "R2", 0, 0, 0);  /* 7: R2 bearing only, unplaced */
    k7->node_bearing_deg = 5;

    wv_item it[SEEKER_MAX_TRACKS];
    int n = wv_classify(&t, &w, &o, it, SEEKER_MAX_TRACKS);
    CHECK(n == 8);
    CHECK(it[0].direct && it[0].in_fov && it[0].dup_of < 0 && NEAR(it[0].range_m, 3, 1e-4));
    CHECK(!it[1].direct && it[1].dup_of < 0 && it[1].in_fov && it[1].has_proj); /* x-ray */
    CHECK(!it[2].direct && it[2].dup_of == 0);
    CHECK(!it[3].direct && it[3].dup_of < 0 && it[3].in_fov);
    CHECK(!it[4].direct && !it[4].in_fov && it[4].has_bearing && !it[4].p.in_front);
    CHECK(it[5].direct && it[5].in_fov && NEAR(it[5].bearing_deg, -15, 1e-4));
    CHECK(it[6].dup_of == 5 && it[5].has_range && NEAR(it[5].range_m, 4, 1e-3)); /* lent range */
    CHECK(it[5].has_proj && it[5].p.in_fov);
    CHECK(!it[7].has_bearing && !it[7].direct);

    /* A tighter association gate keeps 2 as its own x-ray track. */
    w.assoc_range_m = 0.1f;
    n = wv_classify(&t, &w, &o, it, SEEKER_MAX_TRACKS);
    CHECK(it[2].dup_of < 0 && !it[2].direct);
    w.assoc_range_m = 1.5f;

    /* A different wearer id: nothing is seen directly. */
    snprintf(w.node, sizeof w.node, "R9");
    n = wv_classify(&t, &w, &o, it, SEEKER_MAX_TRACKS);
    int any_direct = 0;
    for (int i = 0; i < n; i++)
        any_direct |= it[i].direct;
    CHECK(!any_direct);
    snprintf(w.node, sizeof w.node, "W");

    /* Pose source. */
    wv_pose p;
    int live;
    char st[8];
    w.pose = (wv_pose){ 1, 1, 10 };
    t.nodes[0].yaw_deg = 30;
    wv_resolve_pose(&w, &t, &p, &live, st, sizeof st);
    CHECK(live && NEAR(p.yaw_deg, 30, 1e-4) && NEAR(p.x_m, 0, 1e-4) && !strcmp(st, "ok"));
    w.pose_src = WV_POSE_STATIC;
    wv_resolve_pose(&w, &t, &p, &live, st, sizeof st);
    CHECK(!live && NEAR(p.yaw_deg, 10, 1e-4) && NEAR(p.x_m, 1, 1e-4));
    w.pose_src = WV_POSE_AUTO;
    snprintf(t.nodes[0].state, sizeof t.nodes[0].state, "wait");
    wv_resolve_pose(&w, &t, &p, &live, st, sizeof st);
    CHECK(!live && NEAR(p.yaw_deg, 10, 1e-4));
    w.pose_src = WV_POSE_FUSION;
    wv_resolve_pose(&w, &t, &p, &live, st, sizeof st);
    CHECK(live && NEAR(p.yaw_deg, 30, 1e-4));
    wv_resolve_pose(&w, NULL, &p, &live, st, sizeof st);
    CHECK(!live && st[0] == 0 && NEAR(p.x_m, 1, 1e-4));

    /* Pose strings, hud_fusion --pose style. */
    char node[SEEKER_NODE_LEN] = "keep";
    CHECK(wv_parse_pose("1.5,-2,90", node, sizeof node, &p) == 0 && NEAR(p.y_m, -2, 1e-5));
    CHECK(!strcmp(node, "keep"));
    CHECK(wv_parse_pose("W=0.5,1,-45", node, sizeof node, &p) == 0 && !strcmp(node, "W"));
    CHECK(NEAR(p.yaw_deg, -45, 1e-5));
    CHECK(wv_parse_pose("=1,2,3", node, sizeof node, &p) < 0);
    CHECK(wv_parse_pose("1,2", node, sizeof node, &p) < 0);
    CHECK(wv_parse_pose("1,2,3,4", node, sizeof node, &p) < 0);
    CHECK(wv_parse_pose("R1=x,2,3", node, sizeof node, &p) < 0 && !strcmp(node, "W"));
    float wall[4];
    CHECK(wv_parse_wall("-4,2.5,4,2.5", wall) == 0 && NEAR(wall[1], 2.5, 1e-5));
    CHECK(wv_parse_wall("1,2,3", wall) < 0);
    wv_pose_src ps;
    CHECK(wv_parse_pose_src("fusion", &ps) == 0 && ps == WV_POSE_FUSION);
    CHECK(wv_parse_pose_src("sideways", &ps) < 0);

    /* The demo config: W from remote.nodes, display.wearer on top. */
    static seeker_config cfg;
    CHECK(seeker_config_load(&cfg, src_file("config/seeker.eagleeye.json")) == 0);
    wv_config_load(&w, &cfg, src_file("config/seeker.eagleeye.json"), NULL);
    CHECK(!strcmp(w.node, "W") && w.n_walls == 1 && NEAR(w.walls[0][1], 2.5, 1e-5));
    CHECK(NEAR(w.cam.hfov_deg, 66, 1e-5) && w.cam.image_w == 1280 && w.pose_src == WV_POSE_AUTO);
    wv_config_load(&w, &cfg, src_file("config/seeker.eagleeye.json"), "R1");
    CHECK(!strcmp(w.node, "R1") && NEAR(w.pose.yaw_deg, 0, 1e-5)); /* display.wearer.pose wins */
    wv_config_load(&w, &cfg, "/nonexistent.json", "R2");
    CHECK(!strcmp(w.node, "R2") && NEAR(w.pose.yaw_deg, 315, 1e-5) && w.n_walls == 0);
    /* No display.wearer: the wearer is the video's view_node (R1 in config/seeker.json). */
    static seeker_config plain;
    seeker_config_load(&plain, config_path());
    wv_config_load(&w, &plain, config_path(), NULL);
    CHECK(!strcmp(w.node, plain.view_node) && !strcmp(w.node, "R1"));
}

static int count_colour(const canvas *c, uint32_t col, int x0, int x1, int y0, int y1)
{
    int n = 0;
    for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++)
            n += c->px[y * c->stride + x] == col;
    return n;
}

static void test_wearer_render_mirror(void)
{
    /* Sink mirror: pixel (1, 0) lands at (w - 2, 0) with mirror h, row 0 lands on row h-1 with v. */
    uint32_t src_px[8 * 2] = { 0 }, dst[8 * 2];
    canvas sc = { src_px, 8, 2, 8 };
    src_px[1] = C_RED;
    seeker_sink s;
    memset(&s, 0, sizeof s);
    s.w = 8;
    s.h = 2;
    s.mirror_h = 1;
    seeker_sink_blit(&s, &sc, dst, 8);
    CHECK(dst[6] == C_RED && dst[1] == 0);
    s.mirror_h = 0;
    s.mirror_v = 1;
    seeker_sink_blit(&s, &sc, dst, 8);
    CHECK(dst[8 + 1] == C_RED && dst[1] == 0);

    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    v.cfg.inset = 0;
    const int W = 640, H = 360;
    canvas c = { calloc((size_t)W * H, 4), W, H, W };
    uint32_t *out = calloc((size_t)W * H, 4);
    CHECK(c.px && out);
    if (!c.px || !out)
        return;
    static seeker_tracks t;
    memset(&t, 0, sizeof t);
    add_node(&t, "W", "ok", 0, 0, 0);
    add_node(&t, "R1", "ok", -3, 5.5f, 104);
    add_track(&t, "W+R2", "W", 1, 1.2f, 3.0f); /* seen, 21.8 deg right, 3.2 m: amber */

    wv_render(&c, &cfg, &v, &t, 0, NULL);
    int body = H - 70; /* above the legend */
    int amber_l = count_colour(&c, C_AMBER, 0, W / 2, 0, body);
    int amber_r = count_colour(&c, C_AMBER, W / 2, W, 0, body);
    CHECK(amber_r > 100 && amber_l == 0 && v.last.n_direct == 1 && v.last.n_xray == 0);

    /* Visor mirror: the seen box flips to the left half, so its reflection reads right again. */
    memset(&s, 0, sizeof s);
    s.w = W;
    s.h = H;
    s.mirror_h = 1;
    seeker_sink_blit(&s, &c, out, W);
    canvas m = { out, W, H, W };
    CHECK(count_colour(&m, C_AMBER, 0, W / 2, 0, body) == amber_r);
    CHECK(count_colour(&m, C_AMBER, W / 2, W, 0, body) == 0);

    /* Behind the wall on the left: dashed cyan x-ray box; behind the wearer: edge arrow. */
    memset(&t, 0, sizeof t);
    add_node(&t, "W", "ok", 0, 0, 0);
    add_track(&t, "R1", "R1", 1, -1.0f, 4.5f);
    memset(v.colours, 0, sizeof v.colours); /* another scene: not the seen person held */
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    CHECK(count_colour(&c, C_CYAN, 0, W / 2, 0, body) > 50);
    CHECK(count_colour(&c, C_CYAN, W / 2, W, 0, body) == 0);
    CHECK(v.last.n_xray == 1 && v.last.n_direct == 0 && v.last.n_off == 0);

    memset(&t, 0, sizeof t);
    add_node(&t, "W", "ok", 0, 0, 0);
    add_track(&t, "R2", "R2", 1, -1.0f, -3.0f); /* 162 deg left */
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    CHECK(v.last.n_off == 1 && v.last.n_xray == 0);
    CHECK(count_colour(&c, C_CYAN, 0, 60, H / 4, body) > 10); /* chevron on the left edge */
    CHECK(count_colour(&c, C_CYAN, W / 2, W, 0, body) == 0);

    /* Live yaw: the wearer turns 30 deg right, so a target dead ahead in the room moves left. */
    memset(&t, 0, sizeof t);
    add_node(&t, "W", "ok", 0, 0, 30);
    add_track(&t, "W", "W", 1, 0, 6);
    v.cfg.yaw_alpha = 1.0f;
    memset(v.colours, 0, sizeof v.colours); /* track id 1 is another person now */
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    CHECK(v.last.pose_live && NEAR(v.last.pose.yaw_deg, 30, 1e-3) && v.last.n_direct == 1);
    CHECK(count_colour(&c, C_GREEN, 0, W / 2, 0, body) > 50); /* 6 m: green, left half */
    CHECK(count_colour(&c, C_GREEN, W / 2, W, 0, body) == 0);

    /* No tracks yet and fusion stale: draws, counts nothing. */
    wv_render(&c, &cfg, &v, NULL, 1, NULL);
    CHECK(v.last.n_direct + v.last.n_xray + v.last.n_off == 0 && !v.last.pose_live);

    /* Video under the symbology: video_rx hands over a canvas-sized frame with the picture at rect. */
    uint32_t *vid = malloc((size_t)W * H * 4);
    if (vid) {
        for (int i = 0; i < W * H; i++)
            vid[i] = RGB(0, 0, 200);
        video_frame vf;
        memset(&vf, 0, sizeof vf);
        vf.px = vid;
        vf.w = vf.stride = W;
        vf.h = H;
        vf.rect[2] = W;
        vf.rect[3] = H;
        vf.src_w = 1280;
        vf.src_h = 720;
        snprintf(vf.node_id, sizeof vf.node_id, "W");
        wv_render(&c, &cfg, &v, &t, 0, &vf);
        CHECK(c.px[(H - 2) * W + 2] == RGB(0, 0, 200));
        v.cfg.visor = 1; /* visor ignores video: black stays transparent on the combiner */
        wv_render(&c, &cfg, &v, &t, 0, &vf);
        CHECK(c.px[(H - 2) * W + 2] == C_BLACK);
        v.cfg.visor = 0;
        snprintf(vf.node_id, sizeof vf.node_id, "R1"); /* not the helmet: projection would lie */
        wv_render(&c, &cfg, &v, &t, 0, &vf);
        CHECK(c.px[(H - 2) * W + 2] == C_BLACK);

        /* 4:3 phone frame pillarboxed to x 80..560: targets map into that rect, not the canvas. */
        snprintf(vf.node_id, sizeof vf.node_id, "W");
        vf.rect[0] = 80;
        vf.rect[2] = 480;
        vf.src_w = 640;
        vf.src_h = 480;
        for (int i = 0; i < W * H; i++)
            vid[i] = C_BLACK;
        memset(&t, 0, sizeof t);
        add_node(&t, "W", "ok", 0, 0, 0);
        float b25 = 25 * 0.0174533f;
        add_track(&t, "W", "W", 1, 6 * sinf(b25), 6 * cosf(b25)); /* 6 m, 25 deg right: green */
        memset(v.colours, 0, sizeof v.colours);
        wv_render(&c, &cfg, &v, &t, 0, &vf);
        /* u = 320 + (320 / tan 33) tan 25 = 549.8 px of 640, so x = 80 + 0.75 u = 492. */
        CHECK(count_colour(&c, C_GREEN, 460, 525, 0, body) > 50);
        CHECK(count_colour(&c, C_GREEN, 560, W, 0, body) == 0);
        free(vid);
    }
    free(c.px);
    free(out);
}

/* ---- IF-5.6 video ---- */

typedef struct {
    uint8_t *b;
    size_t n, cap;
} bytes;

static void bytes_put(bytes *o, const void *p, size_t n)
{
    if (o->n + n > o->cap) {
        size_t cap = (o->n + n) * 2;
        uint8_t *nb = realloc(o->b, cap);
        if (!nb)
            abort();
        o->b = nb;
        o->cap = cap;
    }
    memcpy(o->b + o->n, p, n);
    o->n += n;
}

static void jpg_sink(void *ctx, void *data, int size)
{
    bytes_put(ctx, data, (size_t)size);
}

static const uint8_t QUAD[4][3] = { { 255, 0, 0 }, { 0, 255, 0 }, { 0, 0, 255 }, { 255, 255, 255 } };

/* Red top left, green top right, blue bottom left, white bottom right. stb_image_write
 * subsamples chroma 4:2:0 (like phone JPEGs) at quality <= 90 and keeps 4:4:4 above. */
static bytes make_jpeg(int w, int h, int quality)
{
    uint8_t *rgb = malloc((size_t)w * h * 3);
    for (int y = 0; y < h; y++)
        for (int x = 0; x < w; x++)
            memcpy(rgb + ((size_t)y * w + x) * 3, QUAD[(y >= h / 2) * 2 + (x >= w / 2)], 3);
    bytes o = { 0 };
    stbi_write_jpg_to_func(jpg_sink, &o, w, h, 3, rgb, quality);
    free(rgb);
    return o;
}

static int near_rgb(const uint8_t *p, const uint8_t *want, int tol)
{
    for (int i = 0; i < 3; i++)
        if (abs((int)p[i] - (int)want[i]) > tol)
            return 0;
    return 1;
}

static int near_px(uint32_t px, const uint8_t *want, int tol)
{
    uint8_t p[3] = { (uint8_t)(px >> 16), (uint8_t)(px >> 8), (uint8_t)px };
    return near_rgb(p, want, tol);
}

static void put_frame(bytes *o, const char *node, uint8_t v, uint32_t seq, const void *payload,
                      uint32_t len)
{
    skv2_header h;
    memset(&h, 0, sizeof h);
    h.v = v;
    snprintf(h.node_id, sizeof h.node_id, "%s", node);
    h.ts_ms = 1000u + seq;
    h.width = 64;
    h.height = 48;
    h.seq = seq;
    h.payload_len = len;
    uint8_t hb[SKV2_HEADER_LEN];
    skv2_header_encode(&h, hb);
    bytes_put(o, hb, sizeof hb);
    bytes_put(o, payload, len);
}

/* INTERFACES.md IF-5.6 example header, byte for byte. */
static const uint8_t IF56_EXAMPLE[SKV2_HEADER_LEN] = {
    0x53, 0x4B, 0x56, 0x32, 0x02, 0x00, 0x00, 0x00, 0x52, 0x31, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x49, 0xB0, 0x21,
    0x05, 0x00, 0x02, 0xD0, 0x00, 0x00, 0x01, 0xA4, 0x00, 0x01, 0x4A, 0x20,
};

static void test_skv2_header(void)
{
    skv2_header h;
    CHECK(skv2_header_decode(IF56_EXAMPLE, &h) == SKV2_OK);
    CHECK(h.v == 2 && h.flags == 0 && h.reserved == 0 && !strcmp(h.node_id, "R1"));
    CHECK(h.ts_ms == 4829217 && h.width == 1280 && h.height == 720);
    CHECK(h.seq == 420 && h.payload_len == 84512);
    uint8_t back[SKV2_HEADER_LEN];
    skv2_header_encode(&h, back);
    CHECK(memcmp(back, IF56_EXAMPLE, sizeof back) == 0);

    skv2_header g, g2;
    memset(&g, 0, sizeof g);
    g.v = 2;
    snprintf(g.node_id, sizeof g.node_id, "ABCDEFGH");
    g.ts_ms = 0x0123456789ABCDEFull;
    g.width = 640;
    g.height = 480;
    g.seq = 0xFFFFFFFFu;
    g.payload_len = SKV2_MAX_PAYLOAD;
    skv2_header_encode(&g, back);
    CHECK(skv2_header_decode(back, &g2) == SKV2_OK);
    CHECK(!strcmp(g2.node_id, "ABCDEFGH") && g2.ts_ms == g.ts_ms && g2.seq == g.seq);
    CHECK(g2.width == 640 && g2.height == 480 && g2.payload_len == SKV2_MAX_PAYLOAD);

    uint8_t b[SKV2_HEADER_LEN];
    memcpy(b, IF56_EXAMPLE, sizeof b);
    b[3] = '1';
    CHECK(skv2_header_decode(b, &h) == SKV2_ERR_MAGIC);
    memcpy(b, IF56_EXAMPLE, sizeof b);
    b[4] = 3;
    CHECK(skv2_header_decode(b, &h) == SKV2_ERR_VERSION && h.v == 3);
    memcpy(b, IF56_EXAMPLE, sizeof b);
    memset(b + 32, 0, 4);
    CHECK(skv2_header_decode(b, &h) == SKV2_ERR_LEN);
    b[33] = 0x20; /* 0x00200001 = 2 MB + 1 */
    b[35] = 0x01;
    CHECK(skv2_header_decode(b, &h) == SKV2_ERR_LEN);
}

/* Payload bytes stay above 0x80 so no payload can contain the ASCII magic by accident. */
static void fill_payload(uint8_t *p, size_t n, int salt)
{
    for (size_t i = 0; i < n; i++)
        p[i] = (uint8_t)(0x80 + (i * 7 + (size_t)salt) % 120);
}

static void test_skv2_stream(void)
{
    static uint8_t p1[100], p2[70000], pbad[50], p3[10];
    fill_payload(p1, sizeof p1, 1);
    fill_payload(p2, sizeof p2, 2);
    fill_payload(pbad, sizeof pbad, 3);
    fill_payload(p3, sizeof p3, 4);
    bytes s = { 0 };
    const char *junk = "GET / HTTP/1.1\r\nSKV SK"; /* ends in a false start right before SKV2 */
    bytes_put(&s, junk, strlen(junk));
    put_frame(&s, "R1", 2, 1, p1, sizeof p1);
    put_frame(&s, "R1", 2, 2, p2, sizeof p2);
    put_frame(&s, "R1", 3, 99, pbad, sizeof pbad); /* unknown v: warned, skipped */
    skv2_header z;
    memset(&z, 0, sizeof z);
    z.v = 2;
    uint8_t zb[SKV2_HEADER_LEN];
    skv2_header_encode(&z, zb); /* payload_len 0: skipped */
    bytes_put(&s, zb, sizeof zb);
    put_frame(&s, "R2", 2, 3, p3, sizeof p3);

    const uint8_t *want[4] = { NULL, p1, p2, p3 };
    size_t want_len[4] = { 0, sizeof p1, sizeof p2, sizeof p3 };
    skv2_header h;
    const uint8_t *jp;
    size_t len;

    /* Byte by byte, taking after every byte: every good frame comes out once, in order. */
    skv2_parser p;
    skv2_parser_init(&p);
    uint32_t got[8];
    int ng = 0, ok = 1;
    for (size_t i = 0; i < s.n; i++) {
        skv2_parser_feed(&p, s.b + i, 1);
        if (skv2_parser_take(&p, &h, &jp, &len) && ng < 8) {
            got[ng++] = h.seq;
            if (h.seq > 3 || len != want_len[h.seq] || memcmp(jp, want[h.seq], len) != 0)
                ok = 0;
        }
    }
    CHECK(ok && ng == 3 && got[0] == 1 && got[1] == 2 && got[2] == 3);
    CHECK(p.frames == 3 && p.dropped == 0 && p.bad_version == 1 && p.last_bad_v == 3);
    /* junk, then the bad-v frame; the zero-length header turns up while still scanning after
     * it, so it is the same resync episode */
    CHECK(p.bad_len == 1 && p.resyncs == 2 && p.bytes == s.n);
    CHECK(!skv2_parser_take(&p, &h, &jp, &len));
    skv2_parser_free(&p);

    /* All at once: only the newest survives; the two older ones count as dropped. */
    skv2_parser_init(&p);
    CHECK(skv2_parser_feed(&p, s.b, s.n) == 3);
    CHECK(skv2_parser_take(&p, &h, &jp, &len) == 1);
    CHECK(h.seq == 3 && !strcmp(h.node_id, "R2") && len == sizeof p3 && !memcmp(jp, p3, len));
    CHECK(p.dropped == 2 && !skv2_parser_take(&p, &h, &jp, &len));
    skv2_parser_free(&p);

    /* Random chunk sizes: whatever is taken is in order and intact, and the last one is seq 3. */
    skv2_parser_init(&p);
    unsigned rng = 12345;
    size_t off = 0;
    int taken = 0;
    uint32_t last = 0;
    ok = 1;
    while (off < s.n) {
        rng = rng * 1103515245u + 12345u;
        size_t n = 1 + (rng >> 8) % 5000;
        if (n > s.n - off)
            n = s.n - off;
        skv2_parser_feed(&p, s.b + off, n);
        off += n;
        if (skv2_parser_take(&p, &h, &jp, &len)) {
            if (h.seq <= last || h.seq > 3 || len != want_len[h.seq] ||
                memcmp(jp, want[h.seq], len) != 0)
                ok = 0;
            last = h.seq;
            taken++;
        }
    }
    CHECK(ok && last == 3 && p.frames == 3 && p.dropped == (uint64_t)(3 - taken));
    skv2_parser_free(&p);

    /* A new connection mid-frame: reset forgets the half frame. */
    bytes t = { 0 };
    put_frame(&t, "R1", 2, 1, p2, sizeof p2);
    size_t half = t.n / 2;
    put_frame(&t, "R1", 2, 2, p1, sizeof p1);
    skv2_parser_init(&p);
    skv2_parser_feed(&p, t.b, half);
    skv2_parser_reset(&p);
    skv2_parser_feed(&p, t.b + (t.n - SKV2_HEADER_LEN - sizeof p1), SKV2_HEADER_LEN + sizeof p1);
    CHECK(skv2_parser_take(&p, &h, &jp, &len) == 1 && h.seq == 2 && len == sizeof p1);
    CHECK(p.resyncs == 0);
    skv2_parser_free(&p);
    free(t.b);
    free(s.b);
}

static void test_jpeg_decode(void)
{
    const int qualities[2] = { 80, 95 }; /* 4:2:0 and 4:4:4 */
    for (int qi = 0; qi < 2; qi++) {
        bytes j = make_jpeg(64, 48, qualities[qi]);
        CHECK(j.n > 100 && j.b[0] == 0xFF && j.b[1] == 0xD8);
        int w = 0, h = 0;
        uint8_t *rgb = video_jpeg_decode(j.b, j.n, &w, &h);
        CHECK(rgb && w == 64 && h == 48);
        if (rgb) {
            const int at[4][2] = { { 16, 12 }, { 48, 12 }, { 16, 36 }, { 48, 36 } };
            for (int q = 0; q < 4; q++)
                CHECK(near_rgb(rgb + ((size_t)at[q][1] * w + at[q][0]) * 3, QUAD[q], 40));
            video_jpeg_free(rgb);
        }
        CHECK(video_jpeg_decode(j.b, 20, &w, &h) == NULL); /* cut before the frame header */
        free(j.b);
    }
    int w, h;
    const uint8_t png[8] = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };
    CHECK(video_jpeg_decode(png, sizeof png, &w, &h) == NULL);
    CHECK(video_jpeg_decode((const uint8_t *)"not a jpeg", 10, &w, &h) == NULL);
    CHECK(video_jpeg_decode(NULL, 0, &w, &h) == NULL);
}

static void test_video_scale(void)
{
    int r[4];
    video_fit_rect(640, 480, 1920, 1080, r); /* phone.html 4:3 on the 1080p HUD: pillarbox */
    CHECK(r[0] == 240 && r[1] == 0 && r[2] == 1440 && r[3] == 1080);
    video_fit_rect(1280, 720, 1920, 1080, r);
    CHECK(r[0] == 0 && r[1] == 0 && r[2] == 1920 && r[3] == 1080);
    video_fit_rect(1080, 1920, 1920, 1080, r); /* portrait phone */
    CHECK(r[0] == 656 && r[1] == 0 && r[2] == 607 && r[3] == 1080);
    video_fit_rect(1920, 480, 1920, 1080, r); /* letterbox */
    CHECK(r[0] == 0 && r[1] == 300 && r[2] == 1920 && r[3] == 480);
    video_fit_rect(0, 0, 100, 50, r);
    CHECK(r[0] == 0 && r[1] == 0 && r[2] == 100 && r[3] == 50);

    const uint8_t src[2 * 2 * 3] = { 255, 0, 0, 0, 0, 255, 0, 255, 0, 255, 255, 255 };
    uint32_t dst[4 * 10];
    for (int i = 0; i < 40; i++)
        dst[i] = 0xAAAAAAAAu;
    video_fit_rect(2, 2, 8, 4, r);
    CHECK(r[0] == 2 && r[1] == 0 && r[2] == 4 && r[3] == 4);
    video_blit_rgb(src, 2, 2, dst, 8, 4, 10, r);
    CHECK(dst[0] == 0 && dst[1] == 0 && dst[6] == 0 && dst[7] == 0 && dst[37] == 0);
    CHECK(dst[2] == 0xFF0000u && dst[3] == 0xFF0000u && dst[4] == 0x0000FFu && dst[5] == 0x0000FFu);
    CHECK(dst[32] == 0x00FF00u && dst[35] == 0xFFFFFFu && dst[10 + 2] == 0xFF0000u);
    CHECK(dst[8] == 0xAAAAAAAAu && dst[39] == 0xAAAAAAAAu); /* stride padding untouched */
}

static void test_video_state(void)
{
    video_stats st;
    memset(&st, 0, sizeof st);
    video_frame f;
    memset(&f, 0, sizeof f);
    static uint32_t one;
    CHECK(video_rx_state(&f, NULL, 1000) == VIDEO_OFF);
    CHECK(video_rx_state(&f, &st, 1000) == VIDEO_NONE);
    st.connected = 1;
    CHECK(video_rx_state(&f, &st, 1000) == VIDEO_LOST); /* connected, nothing decoded yet */
    f.px = &one;
    f.t_rx_ms = 1000;
    CHECK(video_rx_state(&f, &st, 1000 + VIDEO_STALE_MS) == VIDEO_OK);
    CHECK(video_rx_state(&f, &st, 1000 + VIDEO_STALE_MS + 1) == VIDEO_STALE); /* kept, dimmed */
    CHECK(video_rx_state(&f, &st, 1000 + VIDEO_HOLD_MS) == VIDEO_STALE);
    CHECK(video_rx_state(&f, &st, 1000 + VIDEO_HOLD_MS + 1) == VIDEO_LOST);
    CHECK(VIDEO_SHOWN(VIDEO_STALE) && VIDEO_SHOWN(VIDEO_OK) && !VIDEO_SHOWN(VIDEO_LOST));
    CHECK(!strcmp(video_state_name(VIDEO_STALE), "stale"));
    st.conn = 1; /* a new relay: the previous relay's fresh frame is not its video */
    CHECK(video_rx_state(&f, &st, 1000) == VIDEO_LOST);
    f.conn = 1;
    CHECK(video_rx_state(&f, &st, 1000) == VIDEO_OK);
    st.connected = 0;
    CHECK(video_rx_state(&f, &st, 1000) == VIDEO_NONE); /* relay gone: NO VIDEO at once */
}

static int connect_to(int family, int port)
{
    int fd = socket(family, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;
    int rc;
    if (family == AF_INET) {
        struct sockaddr_in a;
        memset(&a, 0, sizeof a);
        a.sin_family = AF_INET;
        a.sin_port = htons((unsigned short)port);
        inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
        rc = connect(fd, (struct sockaddr *)&a, sizeof a);
    } else {
        struct sockaddr_in6 a;
        memset(&a, 0, sizeof a);
        a.sin6_family = AF_INET6;
        a.sin6_port = htons((unsigned short)port);
        a.sin6_addr = in6addr_loopback;
        rc = connect(fd, (struct sockaddr *)&a, sizeof a);
    }
    if (rc < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static int send_all(int fd, const uint8_t *b, size_t n)
{
    while (n > 0) {
        ssize_t k = send(fd, b, n, 0);
        if (k <= 0)
            return -1;
        b += k;
        n -= (size_t)k;
    }
    return 0;
}

static const video_frame *wait_frame(video_rx *rx, uint32_t seq, video_stats *st)
{
    int64_t end = seeker_now_ms() + 3000;
    while (seeker_now_ms() < end) {
        const video_frame *f = video_rx_latest(rx, st);
        if (f->px && f->seq == seq)
            return f;
        seeker_sleep_ms(5);
    }
    return NULL;
}

/* The receiver thread end to end over loopback TCP: IPv4, then IPv6 replacing it. */
static void test_video_rx_tcp(void)
{
    signal(SIGPIPE, SIG_IGN);
    video_rx *rx = video_rx_start(0, 64, 48, 0);
    CHECK(rx != NULL);
    if (!rx)
        return;
    int port = video_rx_port(rx);
    CHECK(port > 0);
    bytes j = make_jpeg(32, 24, 90);
    bytes s = { 0 };
    bytes_put(&s, "xx", 2); /* garbage first: the receiver must resync */
    put_frame(&s, "R1", 2, 7, j.b, (uint32_t)j.n);
    int fd = connect_to(AF_INET, port);
    CHECK(fd >= 0 && send_all(fd, s.b, s.n) == 0);
    video_stats st;
    const video_frame *f = wait_frame(rx, 7, &st);
    CHECK(f != NULL);
    if (f) {
        CHECK(!strcmp(f->node_id, "R1") && st.connected && st.frames >= 1 && st.resyncs <= 1);
        CHECK(f->src_w == 32 && f->src_h == 24 && f->w == 64 && f->h == 48);
        CHECK(f->rect[0] == 0 && f->rect[1] == 0 && f->rect[2] == 64 && f->rect[3] == 48);
        CHECK(near_px(f->px[12 * f->stride + 16], QUAD[0], 40));
        CHECK(near_px(f->px[36 * f->stride + 48], QUAD[3], 40));
        CHECK(video_rx_state(f, &st, seeker_now_ms()) == VIDEO_OK);
    }

    int fd6 = st.ipv6 ? connect_to(AF_INET6, port) : -1;
    if (fd6 >= 0) {
        bytes s6 = { 0 };
        put_frame(&s6, "R2", 2, 8, j.b, (uint32_t)j.n);
        CHECK(send_all(fd6, s6.b, s6.n) == 0);
        f = wait_frame(rx, 8, &st);
        CHECK(f && !strcmp(f->node_id, "R2"));
        close(fd6);
        free(s6.b);
    } else {
        printf("note: no IPv6 listener or no ::1 here, IPv6 receive check skipped\n");
    }
    close(fd);
    int64_t end = seeker_now_ms() + 2000;
    do {
        video_rx_latest(rx, &st);
        if (!st.connected)
            break;
        seeker_sleep_ms(10);
    } while (seeker_now_ms() < end);
    CHECK(!st.connected);
    video_rx_stop(rx);
    free(s.b);
    free(j.b);
}

/* Camera to HUD latency (latency.h): the phone's epoch-ms ts_ms is mapped onto the Pi clock by
 * the fastest frame in the window; a Pi-clock ts_ms (relay stamp) is used as is; the optional
 * SKV2_FLAG_AGE capture-to-send ms adds in; seq gaps count as skipped; the receiver carries
 * ts_ms, age and the arrival time to the frame, and video_rx_wait wakes on a new frame. */
static void test_latency(void)
{
    static lat_meter m;
    lat_summary s;
    lat_init(&m, 0);
    CHECK(lat_tick(&m, 1000, &s) == 0); /* starts the first second */
    const uint64_t phone0 = 1763780000000ull; /* phone Date.now() at Pi t = 0 */
    /* 30 frames in a second: 20 ms of network, plus 0..58 ms of queueing; 25 ms on the Pi. */
    for (int i = 0; i < 30; i++) {
        int64_t sent = 1000 + 33 * i, q = (i % 30) * 2;
        lat_sample ls = { phone0 + (uint64_t)sent, -1, sent + 20 + q, sent + 20 + q + 10,
                          sent + 20 + q + 25, 6.0f, 3.0f + (float)(i % 3), 2.0f, 4.0f, 1.5f, 8.0f };
        lat_frame(&m, (uint32_t)(i < 29 ? i : 31), &ls); /* 29, 30 skipped */
    }
    CHECK(lat_tick(&m, 2000, &s) == 1);
    CHECK(s.n == 30 && NEAR(s.fps, 30, 0.01) && !s.same_clock && s.skipped == 2);
    CHECK(NEAR(s.net_p50, 28, 0.5) && NEAR(s.net_p95, 56, 0.5)); /* above the fastest frame */
    CHECK(NEAR(s.pi_p50, 25, 0.01) && NEAR(s.decode_p50, 10, 0.01) && NEAR(s.wait_p50, 15, 0.01));
    CHECK(NEAR(s.total_p50, 53, 0.5) && s.age_p50 < 0);
    char txt[64], line[512];
    lat_text(&m, txt, sizeof txt);
    CHECK(!strcmp(txt, "LAT 53MS P95 81MS FPS 30"));
    s.rx_fps = 31.0f;
    lat_log_line(&s, 0, line, sizeof line);
    CHECK(strstr(line, "total p50 53 p95 81 ms") && strstr(line, "phone n/a") &&
          strstr(line, "above the best frame"));
    CHECK(strstr(line, "stages p50 ms: decode 6.0 scale 4.0 copy 2.0 overlay 4.0 blit 1.5 post 8.0") &&
          strstr(line, "received 31.0 fps"));
    printf("latency line: %s\n", line);

    /* The phone's clock is set back 1 h: the window starts over instead of reading +1 h. */
    lat_sample jump = { phone0 - 3600000ull + 2100, -1, 2120, 2130, 2145, 0, 0, 0, 0, 0, 0 };
    lat_frame(&m, 40, &jump);
    CHECK(lat_tick(&m, 3000, &s) == 1 && s.n == 1 && NEAR(s.net_p50, 0, 0.01) &&
          NEAR(s.total_p50, 25, 0.01));
    CHECK(lat_tick(&m, 4000, &s) == 1 && s.n == 0);
    lat_text(&m, txt, sizeof txt);
    CHECK(!strcmp(txt, "LAT --"));
    lat_log_line(&s, 0, line, sizeof line);
    CHECK(strstr(line, "no wearer frame") != NULL);

    /* A relay stamp (Pi monotonic): absolute; capture age and the base add in. */
    lat_init(&m, 40);
    lat_tick(&m, 10000, NULL);
    lat_sample rel = { 10000, 35, 10012, 10020, 10030, 0, 0, 0, 0, 0, 0 };
    lat_frame(&m, 1, &rel);
    CHECK(lat_tick(&m, 11000, &s) == 1 && s.same_clock && NEAR(s.net_p50, 12, 0.01) &&
          NEAR(s.age_p50, 35, 0.01) && NEAR(s.total_p50, 35 + 12 + 18 + 40, 0.01));

    /* Through the receiver: ts_ms, the age flag and arrival come out on the frame. */
    signal(SIGPIPE, SIG_IGN);
    video_rx *rx = video_rx_start(0, 64, 48, 0);
    CHECK(rx != NULL);
    if (!rx)
        return;
    CHECK(video_rx_wait(rx, 20) == 0); /* nothing yet: times out */
    bytes j = make_jpeg(32, 24, 90);
    skv2_header h;
    memset(&h, 0, sizeof h);
    h.v = 2;
    h.flags = SKV2_FLAG_AGE;
    h.reserved = 42;
    snprintf(h.node_id, sizeof h.node_id, "R1");
    h.ts_ms = phone0 + 5;
    h.width = 32;
    h.height = 24;
    h.seq = 9;
    h.payload_len = (uint32_t)j.n;
    uint8_t hb[SKV2_HEADER_LEN];
    skv2_header_encode(&h, hb);
    int64_t t_send = seeker_now_ms();
    int fd = connect_to(AF_INET, video_rx_port(rx));
    CHECK(fd >= 0 && send_all(fd, hb, sizeof hb) == 0 && send_all(fd, j.b, j.n) == 0);
    CHECK(video_rx_wait(rx, 3000) == 1);
    video_stats st;
    const video_frame *f = video_rx_latest(rx, &st);
    CHECK(f->px && f->seq == 9 && f->ts_ms == phone0 + 5 && f->age_ms == 42);
    CHECK(f->t_arrive_ms >= t_send && f->t_arrive_ms <= f->t_rx_ms);
    CHECK(video_rx_wait(rx, 10) == 0); /* handed out: no newer frame */
    close(fd);
    video_rx_stop(rx);
    free(j.b);
}

/* Polls the receiver until video_rx_why says want (or ms pass); returns the last answer. */
static int wait_why(video_rx *rx, int want, int ms, video_stats *st)
{
    int64_t end = seeker_now_ms() + ms;
    int why;
    do {
        const video_frame *f = video_rx_latest(rx, st);
        why = video_rx_why(f, st, seeker_now_ms());
        if (why == want)
            break;
        seeker_sleep_ms(10);
    } while (seeker_now_ms() < end);
    return why;
}

/* The HUD says why there is no picture: no bridge, bridge but no frames, frames that stopped,
 * frames that do not decode, bytes that are not SKV2. */
static void test_video_rx_why(void)
{
    signal(SIGPIPE, SIG_IGN);
    CHECK(video_rx_why(NULL, NULL, 0) == VIDEO_WHY_OFF);
    CHECK(!strcmp(video_why_text(VIDEO_WHY_NO_FRAMES), "NO VIDEO: PHONE NOT SENDING"));
    CHECK(!strcmp(video_why_name(VIDEO_WHY_NO_LINK), "no-link") && video_why_text(VIDEO_WHY_OK)[0] == 0);
    video_rx *rx = video_rx_start(0, 64, 48, 0);
    CHECK(rx != NULL);
    if (!rx)
        return;
    int port = video_rx_port(rx);
    video_stats st;
    CHECK(wait_why(rx, VIDEO_WHY_NO_LINK, 200, &st) == VIDEO_WHY_NO_LINK);

    int fd = connect_to(AF_INET, port);
    CHECK(fd >= 0);
    CHECK(wait_why(rx, VIDEO_WHY_NO_FRAMES, 2000, &st) == VIDEO_WHY_NO_FRAMES);
    CHECK(st.connected && st.conn_ms > 0 && st.conn_bytes == 0);

    bytes j = make_jpeg(32, 24, 90), s = { 0 };
    put_frame(&s, "R1", 2, 1, j.b, (uint32_t)j.n);
    CHECK(send_all(fd, s.b, s.n) == 0);
    CHECK(wait_why(rx, VIDEO_WHY_OK, 2000, &st) == VIDEO_WHY_OK);
    CHECK(st.conn_frames == 1 && st.conn_bytes == s.n);
    CHECK(wait_why(rx, VIDEO_WHY_STALLED, VIDEO_HOLD_MS + 1000, &st) == VIDEO_WHY_STALLED);

    bytes bad = { 0 };
    put_frame(&bad, "R1", 2, 2, (const uint8_t *)"not a jpeg", 10);
    CHECK(send_all(fd, bad.b, bad.n) == 0);
    CHECK(wait_why(rx, VIDEO_WHY_DECODE, 2000, &st) == VIDEO_WHY_DECODE);
    CHECK(st.conn_decode_errors == 1);

    int fd2 = connect_to(AF_INET, port); /* a new bridge replaces the old one */
    CHECK(fd2 >= 0 && wait_why(rx, VIDEO_WHY_NO_FRAMES, 2000, &st) == VIDEO_WHY_NO_FRAMES);
    CHECK(send_all(fd2, (const uint8_t *)"GET / HTTP/1.1\r\n\r\n", 18) == 0);
    CHECK(wait_why(rx, VIDEO_WHY_BAD_DATA, VIDEO_BAD_DATA_MS + 1500, &st) == VIDEO_WHY_BAD_DATA);
    close(fd2);
    close(fd);
    CHECK(wait_why(rx, VIDEO_WHY_NO_LINK, 2000, &st) == VIDEO_WHY_NO_LINK && st.conn_ms == 0);
    video_rx_stop(rx);
    free(s.b);
    free(bad.b);
    free(j.b);
}

/* Whole file into a malloc'd buffer (NUL terminated); *n is the byte count. */
static char *slurp(const char *path, size_t *n)
{
    FILE *f = fopen(path, "rb");
    if (!f)
        return NULL;
    char *b = malloc(1 << 16);
    *n = b ? fread(b, 1, (1 << 16) - 1, f) : 0;
    fclose(f);
    if (b)
        b[*n] = '\0';
    return b;
}

/* The PPM header note is one "# " line after "P6", whatever bytes a relay put in node_id. */
static void test_ppm_note(void)
{
    uint32_t px[3 * 2] = { RGB(255, 0, 0), RGB(0, 255, 0), RGB(0, 0, 255), 0, 0, RGB(1, 2, 3) };
    canvas c = { px, 3, 2, 3 };
    char path[64];
    snprintf(path, sizeof path, "%s/seeker_test_%ld.ppm", access("/tmp", W_OK) == 0 ? "/tmp" : ".",
             (long)getpid());
    CHECK(seeker_sink_write_ppm_note(path, &c, "seeker hud frame=1 video=ok node=R\n1\x80") == 0);
    size_t n = 0;
    char *b = slurp(path, &n);
    static const char head[] = "P6\n# seeker hud frame=1 video=ok node=R?1?\n3 2\n255\n";
    const size_t hl = sizeof head - 1;
    CHECK(b && n == hl + 3 * 2 * 3 && !memcmp(b, head, hl));
    if (b && n == hl + 18) {
        const uint8_t *d = (const uint8_t *)b + hl;
        CHECK(d[0] == 255 && d[1] == 0 && d[4] == 255 && d[8] == 255 && d[15] == 1 && d[17] == 3);
    }
    free(b);
    CHECK(seeker_sink_write_ppm(path, &c) == 0); /* no note: plain header, as before */
    b = slurp(path, &n);
    CHECK(b && n == 11 + 18 && !memcmp(b, "P6\n3 2\n255\n", 11));
    free(b);
    unlink(path);

    video_frame f;
    memset(&f, 0, sizeof f);
    char note[160];
    hud_frame_note(note, sizeof note, 7, VIDEO_NONE, 0, &f);
    CHECK(!strcmp(note, "seeker hud frame=7 video=none drawn=0 node=- seq=0"));
    f.px = px;
    snprintf(f.node_id, sizeof f.node_id, "R1");
    f.seq = 4000000000u;
    hud_frame_note(note, sizeof note, 8, VIDEO_OK, 1, &f);
    CHECK(!strcmp(note, "seeker hud frame=8 video=ok drawn=1 node=R1 seq=4000000000"));
    CHECK(!strcmp(video_state_name(VIDEO_OFF), "off") && !strcmp(video_state_name(VIDEO_LOST), "lost"));

    /* The ppm sink writes every ppm_every-th frame and any frame whose state (video, drawn,
     * node) changed, so a state that lasts one or two frames is never missed; a new seq alone
     * is not a change. */
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    cfg.width = 3;
    cfg.height = 2;
    seeker_sink s;
    memset(&s, 0, sizeof s);
    snprintf(s.ppm_path, sizeof s.ppm_path, "%s", path);
    s.ppm_every = 1000;
    CHECK(seeker_sink_open(&s, "ppm", &cfg) == 0);
    static const struct {
        const char *note;
        int written;
    } seq[] = {
        { "seeker hud frame=0 video=none drawn=0 node=- seq=0", 1 },  /* first frame */
        { "seeker hud frame=1 video=none drawn=0 node=- seq=0", 0 },
        { "seeker hud frame=2 video=ok drawn=1 node=R2 seq=1", 1 },   /* the one frame */
        { "seeker hud frame=3 video=ok drawn=1 node=R2 seq=1", 0 },
        { "seeker hud frame=4 video=ok drawn=1 node=R2 seq=2", 0 },   /* new seq only */
        { "seeker hud frame=5 video=lost drawn=0 node=R2 seq=2", 1 }, /* VIDEO LOST */
        { "seeker hud frame=6 video=ok drawn=1 node=R1 seq=3", 1 },   /* other node */
    };
    int ok = 1;
    for (size_t i = 0; i < sizeof seq / sizeof seq[0]; i++) {
        unlink(path);
        snprintf(s.ppm_note, sizeof s.ppm_note, "%s", seq[i].note);
        s.present(&s, &c);
        int written = access(path, R_OK) == 0;
        if (written != seq[i].written) {
            fprintf(stderr, "ppm sink: frame %zu written %d, want %d\n", i, written, seq[i].written);
            ok = 0;
        }
    }
    CHECK(ok);
    unlink(path);
}

/* hud_render's return value is what the snapshot note and the "frame drawn" log line report. */
static void test_hud_render_drawn(void)
{
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    enum { W = 128, H = 72 };
    static uint32_t px[W * H], vid[W * H];
    canvas cv = { px, W, H, W };
    for (int i = 0; i < W * H; i++)
        vid[i] = RGB(255, 0, 0);
    video_frame f;
    memset(&f, 0, sizeof f);
    f.px = vid;
    f.w = f.stride = f.src_w = W;
    f.h = f.src_h = H;
    f.rect[2] = W;
    f.rect[3] = H;
    snprintf(f.node_id, sizeof f.node_id, "R1");
    hud_state st = { .background = "wearer", .view_node = "R1", .video = &f,
                     .video_state = VIDEO_OK, .tracks_age_ms = -1 };
    CHECK(hud_render(&cv, &cfg, &st) == 1 && px[20 * W + 8] == RGB(255, 0, 0));
    st.video_state = VIDEO_LOST;
    CHECK(hud_render(&cv, &cfg, &st) == 0 && px[20 * W + 8] == C_BLACK);
    st.video_state = VIDEO_OK;
    st.background = "radar";
    CHECK(hud_render(&cv, &cfg, &st) == 0);

    /* EagleEye: only the wearer node's own video is drawn. */
    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    st.background = "wearer";
    st.wearer = &v;
    CHECK(hud_render(&cv, &cfg, &st) == 0 && v.last.video == -1); /* R1 is not the wearer */
    snprintf(f.node_id, sizeof f.node_id, "%s", v.cfg.node);
    CHECK(hud_render(&cv, &cfg, &st) == 1 && v.last.video == 1 && v.last.rendered);
    /* Radar fallback (no fresh video): the wearer view is not drawn, its stats say so. */
    v.cfg.radar_fallback = 1;
    st.video_state = VIDEO_LOST;
    CHECK(hud_render(&cv, &cfg, &st) == 0 && !v.last.rendered);
    v.cfg.radar_fallback = 0;
    CHECK(hud_render(&cv, &cfg, &st) == 0 && v.last.rendered);
}

/* The Pi failure of scripts/video_loopback_test.sh, over real loopback TCP: a relay slow to send
 * its first frame, a snapshot while it streams, then the relay closes. The picture is drawn only
 * while the relay is up, so a snapshot taken after the sender exits (what the script used to do)
 * is the radar fallback, black at the sampled corner. */
static void test_video_snapshot_timing(void)
{
    signal(SIGPIPE, SIG_IGN);
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    enum { W = 128, H = 72 };
    static uint32_t px[W * H];
    canvas cv = { px, W, H, W };
    video_rx *rx = video_rx_start(0, W, H, 0);
    CHECK(rx != NULL);
    if (!rx)
        return;
    int fd = connect_to(AF_INET, video_rx_port(rx));
    CHECK(fd >= 0);
    video_stats st;
    const video_frame *f = NULL;
    int64_t end = seeker_now_ms() + 2000;
    do { /* connected, first frame not sent yet */
        f = video_rx_latest(rx, &st);
        if (st.connected)
            break;
        seeker_sleep_ms(5);
    } while (seeker_now_ms() < end);
    seeker_sleep_ms(300);
    f = video_rx_latest(rx, &st);
    hud_state hs = { .background = "wearer", .view_node = "R1", .video = f, .tracks_age_ms = -1 };
    hs.video_state = video_rx_state(f, &st, seeker_now_ms());
    CHECK(st.connected && hs.video_state == VIDEO_LOST);
    char note[160];
    int drawn = hud_render(&cv, &cfg, &hs);
    hud_frame_note(note, sizeof note, 1, hs.video_state, drawn, f);
    CHECK(drawn == 0 && !strcmp(note, "seeker hud frame=1 video=lost drawn=0 node=- seq=0"));

    bytes j = make_jpeg(64, 48, 95), s = { 0 }; /* 4:3 on 16:9: pillarboxed to x 16..112 */
    put_frame(&s, "R1", 2, 5, j.b, (uint32_t)j.n);
    CHECK(fd >= 0 && send_all(fd, s.b, s.n) == 0);
    f = wait_frame(rx, 5, &st);
    CHECK(f != NULL);
    if (f) {
        hs.video = f;
        hs.video_state = video_rx_state(f, &st, seeker_now_ms());
        drawn = hud_render(&cv, &cfg, &hs);
        hud_frame_note(note, sizeof note, 2, hs.video_state, drawn, f);
        CHECK(drawn == 1 && !strcmp(note, "seeker hud frame=2 video=ok drawn=1 node=R1 seq=5"));
        CHECK(f->rect[0] == 16 && f->rect[2] == 96 && f->rect[3] == 72);
        CHECK(near_px(px[20 * W + 24], QUAD[0], 60)); /* red corner, as the script samples */
        CHECK(px[20 * W + 8] == C_BLACK); /* pillarbox bar */
    }

    if (fd >= 0)
        close(fd);
    end = seeker_now_ms() + 2000;
    do {
        f = video_rx_latest(rx, &st);
        if (!st.connected)
            break;
        seeker_sleep_ms(5);
    } while (seeker_now_ms() < end);
    hs.video = f;
    hs.video_state = video_rx_state(f, &st, seeker_now_ms());
    drawn = hud_render(&cv, &cfg, &hs);
    hud_frame_note(note, sizeof note, 3, hs.video_state, drawn, f);
    CHECK(!st.connected && drawn == 0);
    CHECK(!strcmp(note, "seeker hud frame=3 video=none drawn=0 node=R1 seq=5"));
    CHECK(px[20 * W + 24] == C_BLACK); /* the 0,0,0 the Pi reported */

    /* A new relay right away: R1's last frame is still under VIDEO_STALE_MS old but belongs to
     * the closed connection, so it is VIDEO LOST (not drawn) until the new relay's frame. */
    fd = connect_to(AF_INET, video_rx_port(rx));
    CHECK(fd >= 0);
    end = seeker_now_ms() + 2000;
    do {
        f = video_rx_latest(rx, &st);
        if (st.connected)
            break;
        seeker_sleep_ms(5);
    } while (seeker_now_ms() < end);
    CHECK(st.connected && f->px && f->seq == 5);
    CHECK(seeker_now_ms() - f->t_rx_ms <= VIDEO_STALE_MS);
    hs.video = f;
    hs.video_state = video_rx_state(f, &st, seeker_now_ms());
    CHECK(hs.video_state == VIDEO_LOST && hud_render(&cv, &cfg, &hs) == 0);
    if (fd >= 0)
        close(fd);
    video_rx_stop(rx);
    free(s.b);
    free(j.b);
}


/* The Pi flicker (Sat Nov 21 live run): helmet video over Wi-Fi arrives in bursts, with gaps of
 * 0.5 to 2 s while the phone runs pose and tag detection. The wearer view with the radar fallback
 * (run-live's --video-fallback radar) must stay on the picture through those gaps (dimmed, VIDEO
 * STALE) and fall back to the radar only after VIDEO_HOLD_MS. */
static void test_video_bursty(void)
{
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    enum { W = 128, H = 72 };
    static uint32_t px[W * H], vid[W * H];
    canvas cv = { px, W, H, W };
    for (int i = 0; i < W * H; i++)
        vid[i] = RGB(250, 0, 0);
    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    v.cfg.radar_fallback = 1;
    video_frame f;
    memset(&f, 0, sizeof f);
    f.px = vid;
    f.w = f.stride = f.src_w = W;
    f.h = f.src_h = H;
    f.rect[2] = W;
    f.rect[3] = H;
    snprintf(f.node_id, sizeof f.node_id, "%s", v.cfg.node);
    video_stats st;
    memset(&st, 0, sizeof st);
    st.connected = 1;
    hud_state hs = { .background = "wearer", .view_node = v.cfg.node, .video = &f,
                     .tracks_age_ms = -1, .wearer = &v };

    /* Frame gaps as seen on the Pi: bursts, then holes up to 1.95 s. */
    static const int gaps[] = { 70, 60, 1300, 80, 900, 1700, 120, 600, 1950, 66, 66, 1450 };
    int64_t t = 1000;
    f.t_rx_ms = t;
    int radar_flips = 0, old_rule_flips = 0, was_radar = 0, old_was_off = 0, not_dimmed_stale = 0;
    for (size_t g = 0; g < sizeof gaps / sizeof gaps[0]; g++) {
        for (int64_t now = t; now < t + gaps[g]; now += 10) {
            hs.video_state = video_rx_state(&f, &st, now);
            int drawn = hud_render(&cv, &cfg, &hs);
            int radar = !v.last.rendered;
            radar_flips += radar && !was_radar;
            was_radar = radar;
            int old_off = now - f.t_rx_ms > VIDEO_STALE_MS; /* the old 500 ms cut-off */
            old_rule_flips += old_off && !old_was_off;
            old_was_off = old_off;
            if (hs.video_state == VIDEO_STALE && (!drawn || px[36 * W + 4] != RGB(125, 0, 0)))
                not_dimmed_stale++;
        }
        t += gaps[g];
        f.t_rx_ms = t; /* next frame arrives */
        f.seq++;
    }
    CHECK(radar_flips == 0);    /* never left the picture */
    CHECK(old_rule_flips >= 5); /* what the 500 ms rule did: flicker */
    CHECK(not_dimmed_stale == 0);

    /* Fresh: full brightness. Stale: dimmed, tagged, still drawn; note says so. */
    hs.video_state = video_rx_state(&f, &st, t + 100);
    CHECK(hs.video_state == VIDEO_OK && hud_render(&cv, &cfg, &hs) == 1 &&
          px[36 * W + 4] == RGB(250, 0, 0));
    hs.video_state = video_rx_state(&f, &st, t + 1200);
    CHECK(hs.video_state == VIDEO_STALE && hud_render(&cv, &cfg, &hs) == 1 && v.last.rendered &&
          px[36 * W + 4] == RGB(125, 0, 0));
    char note[160];
    hud_frame_note(note, sizeof note, 9, hs.video_state, 1, &f);
    CHECK(strstr(note, " video=stale drawn=1 ") != NULL);
    /* A real outage (no frame for over VIDEO_HOLD_MS): radar once, back on the next frame. */
    hs.video_state = video_rx_state(&f, &st, t + VIDEO_HOLD_MS + 1);
    CHECK(hs.video_state == VIDEO_LOST && hud_render(&cv, &cfg, &hs) == 0 && !v.last.rendered);
    f.t_rx_ms = t + 2600;
    hs.video_state = video_rx_state(&f, &st, t + 2610);
    CHECK(hs.video_state == VIDEO_OK && hud_render(&cv, &cfg, &hs) == 1 && v.last.rendered);
    /* Plain --background wearer (no EagleEye) keeps the stale picture too. */
    hs.wearer = NULL;
    hs.video_state = VIDEO_STALE;
    CHECK(hud_render(&cv, &cfg, &hs) == 1 && px[36 * W + 4] == RGB(125, 0, 0));
}

/* The wearer view pins the receiver to the wearer node: another phone's frames (R2 also opens
 * its page with video=1) are skipped before decoding, so they never replace or refresh R1's
 * picture, and with only R2 sending the HUD says so. */
static void test_video_pinned_node(void)
{
    signal(SIGPIPE, SIG_IGN);
    video_rx *rx = video_rx_start(0, 64, 48, 0);
    CHECK(rx != NULL);
    if (!rx)
        return;
    video_rx_only_node(rx, "R1");
    int fd = connect_to(AF_INET, video_rx_port(rx));
    CHECK(fd >= 0);
    bytes j = make_jpeg(32, 24, 90), a = { 0 }, b = { 0 }, c = { 0 };
    put_frame(&a, "R2", 2, 3, j.b, (uint32_t)j.n);
    CHECK(fd >= 0 && send_all(fd, a.b, a.n) == 0);
    video_stats st;
    CHECK(wait_why(rx, VIDEO_WHY_OTHER_NODE, 2000, &st) == VIDEO_WHY_OTHER_NODE);
    CHECK(st.other_frames == 1 && st.conn_other_frames == 1 && st.conn_frames == 0);
    CHECK(!strcmp(video_why_name(VIDEO_WHY_OTHER_NODE), "other-node"));
    const video_frame *f = video_rx_latest(rx, &st);
    CHECK(!f->px && video_rx_state(f, &st, seeker_now_ms()) == VIDEO_LOST);

    put_frame(&b, "R1", 2, 4, j.b, (uint32_t)j.n);
    CHECK(send_all(fd, b.b, b.n) == 0);
    f = wait_frame(rx, 4, &st);
    CHECK(f && !strcmp(f->node_id, "R1"));
    int64_t t_r1 = f ? f->t_rx_ms : 0;
    seeker_sleep_ms(50);
    put_frame(&c, "R2", 2, 5, j.b, (uint32_t)j.n);
    CHECK(send_all(fd, c.b, c.n) == 0);
    int64_t end = seeker_now_ms() + 2000;
    do {
        f = video_rx_latest(rx, &st);
        if (st.other_frames >= 2)
            break;
        seeker_sleep_ms(5);
    } while (seeker_now_ms() < end);
    CHECK(st.other_frames == 2 && f->seq == 4 && !strcmp(f->node_id, "R1") && f->t_rx_ms == t_r1);
    CHECK(video_rx_why(f, &st, seeker_now_ms()) == VIDEO_WHY_OK);
    if (fd >= 0)
        close(fd);
    video_rx_stop(rx);
    free(a.b);
    free(b.b);
    free(c.b);
    free(j.b);
}

static void test_behind_wall_and_live_pose(void)
{
    char err[128];
    /* Ask 4a: partial pose (x_m,y_m numeric, yaw_deg null) updates xy without waiting for yaw. */
    const char *js_part = "{\"type\":\"remote_contacts\",\"v\":2,\"node_id\":\"R1\",\"t_mono_ms\":1000,"
                          "\"pose\":{\"x_m\":0,\"y_m\":0,\"yaw_deg\":null},"
                          "\"contacts\":[{\"id\":\"p1\",\"cls\":\"person\",\"bearing_deg\":5,"
                          "\"range_m\":2.5,\"box\":[420,120,80,240]}]}";
    seeker_remote_contacts m;
    CHECK(seeker_decode_remote_contacts(js_part, (int)strlen(js_part), &m, err, sizeof err) == 0);
    CHECK(!m.has_pose && m.has_xy && !m.has_yaw && NEAR(m.x_m, 0, 1e-4) && NEAR(m.y_m, 0, 1e-4));

    static seeker_config cfg;
    CHECK(seeker_config_load(&cfg, src_file("config/seeker.eagleeye.json")) == 0);
    static seeker_fusion fus;
    seeker_fusion_init(&fus, &cfg);
    /* Before messages: R1 had config (-3.0, 5.5, 104). --init-pose sets (0,0,0) unpinned. */
    seeker_pose r1_init = { 0, 0, 0 };
    seeker_pose r2_pin = { 0, 2, 180 };
    CHECK(seeker_fusion_set_pose(&fus, "R1", &r1_init, "init", 0) >= 0);
    CHECK(seeker_fusion_set_pose(&fus, "R2", &r2_pin, "cli", 1) >= 0);

    seeker_fusion_ingest(&fus, &m, 1000);
    static seeker_tracks tr;
    seeker_fusion_tick(&fus, 1010, &tr);
    int r1i = -1, r2i = -1;
    for (int i = 0; i < tr.n_nodes; i++) {
        if (!strcmp(tr.nodes[i].node_id, "R1")) r1i = i;
        if (!strcmp(tr.nodes[i].node_id, "R2")) r2i = i;
    }
    CHECK(r1i >= 0 && r2i >= 0);
    CHECK(NEAR(tr.nodes[r1i].x_m, 0, 1e-4) && NEAR(tr.nodes[r1i].y_m, 0, 1e-4));
    CHECK(!strcmp(tr.nodes[r1i].pose_src, "partial"));

    /* Ask 3: R2 pin beats message pose. */
    const char *js_r2_msg = "{\"type\":\"remote_contacts\",\"v\":2,\"node_id\":\"R2\",\"t_mono_ms\":1000,"
                            "\"pose\":{\"x_m\":9,\"y_m\":9,\"yaw_deg\":45},"
                            "\"contacts\":[]}";
    seeker_remote_contacts m2;
    CHECK(seeker_decode_remote_contacts(js_r2_msg, (int)strlen(js_r2_msg), &m2, err, sizeof err) == 0);
    CHECK(m2.has_pose);
    seeker_fusion_ingest(&fus, &m2, 1015);
    seeker_fusion_tick(&fus, 1020, &tr);
    CHECK(NEAR(tr.nodes[r2i].x_m, 0, 1e-4) && NEAR(tr.nodes[r2i].y_m, 2, 1e-4) &&
          NEAR(tr.nodes[r2i].yaw_deg, 180, 1e-4));

    /* pose_src survives tracks JSON roundtrip; WV_POSE_AUTO keeps *live=0 until "message". */
    char tbuf[4096];
    int tn = seeker_encode_tracks(&tr, tbuf, sizeof tbuf);
    CHECK(tn > 0);
    static seeker_tracks tr2;
    CHECK(seeker_decode_tracks(tbuf, tn, &tr2, err, sizeof err) == 0);
    CHECK(!strcmp(tr2.nodes[r1i].pose_src, "partial"));

    wv_config wc;
    wv_config_defaults(&wc);
    snprintf(wc.node, sizeof wc.node, "R1");
    wc.pose_src = WV_POSE_AUTO;
    wv_pose rp;
    int live = 9;
    wv_resolve_pose(&wc, &tr2, &rp, &live, NULL, 0);
    CHECK(!live && NEAR(rp.x_m, 0, 1e-4) && NEAR(rp.y_m, 0, 1e-4));

    /* Full R1 message pose with yaw turns POSE LIVE on. */
    const char *js_full = "{\"type\":\"remote_contacts\",\"v\":2,\"node_id\":\"R1\",\"t_mono_ms\":1050,"
                          "\"pose\":{\"x_m\":0,\"y_m\":0,\"yaw_deg\":12},"
                          "\"contacts\":[]}";
    CHECK(seeker_decode_remote_contacts(js_full, (int)strlen(js_full), &m, err, sizeof err) == 0);
    CHECK(m.has_pose);
    seeker_fusion_ingest(&fus, &m, 1050);
    seeker_fusion_tick(&fus, 1060, &tr);
    wv_resolve_pose(&wc, &tr, &rp, &live, NULL, 0);
    CHECK(live && NEAR(rp.yaw_deg, 12, 1e-4) && !strcmp(tr.nodes[r1i].pose_src, "message"));

    /* Ask 4b: SEEN node_box uses video frame size (640x480), not config 1280x720. */
    static wv_view vv;
    memset(&vv, 0, sizeof vv);
    wv_config_defaults(&vv.cfg);
    snprintf(vv.cfg.node, sizeof vv.cfg.node, "R1");
    vv.cfg.inset = 0;
    vv.cfg.cam.image_w = 1280;
    vv.cfg.cam.image_h = 720;
    const int W = 640, H = 480;
    canvas c = { calloc((size_t)W * H, 4), W, H, W };
    uint32_t *vid = calloc((size_t)W * H, 4);
    CHECK(c.px && vid);
    if (c.px && vid) {
        video_frame vf;
        memset(&vf, 0, sizeof vf);
        vf.px = vid;
        vf.w = vf.stride = W;
        vf.h = H;
        vf.rect[2] = W;
        vf.rect[3] = H;
        vf.src_w = 640;
        vf.src_h = 480;
        snprintf(vf.node_id, sizeof vf.node_id, "R1");
        static seeker_tracks vt;
        memset(&vt, 0, sizeof vt);
        add_node(&vt, "R1", "ok", 0, 0, 0);
        seeker_track *tk = add_track(&vt, "R1", "R1", 1, 1.0f, 3.0f);
        tk->has_node_box = 1;
        tk->node_box[0] = 420;
        tk->node_box[1] = 120;
        tk->node_box[2] = 80;
        tk->node_box[3] = 240;
        wv_render(&c, &cfg, &vv, &vt, 0, &vf);
        int body = H - 70;
        CHECK(count_colour(&c, C_AMBER, W / 2, W, 0, body) > 50);
        CHECK(count_colour(&c, C_AMBER, 0, W / 2, 0, body) == 0);

        /* Ask 1: wall segment keeps behind-wall R2 sighting XRAY even on same bearing as R1 SEEN. */
        vv.cfg.n_walls = 1;
        vv.cfg.walls[0][0] = -4;
        vv.cfg.walls[0][1] = 2.5f;
        vv.cfg.walls[0][2] = 4;
        vv.cfg.walls[0][3] = 2.5f;
        memset(&vt, 0, sizeof vt);
        add_node(&vt, "R1", "ok", 0, 0, 0);
        add_node(&vt, "R2", "ok", 0, 2, 180);
        add_track(&vt, "R1", "R1", 1, 0.05f, 1.5f); /* front of wall: SEEN */
        add_track(&vt, "R2", "R2", 1, 0.10f, 4.0f); /* behind wall: XRAY, not dup */
        wv_item it[SEEKER_MAX_TRACKS];
        wv_pose p0 = { 0, 0, 0 };
        int nc = wv_classify(&vt, &vv.cfg, &p0, it, SEEKER_MAX_TRACKS);
        CHECK(nc == 2 && !it[0].behind_wall && it[1].behind_wall && it[1].dup_of < 0);
        wv_render(&c, &cfg, &vv, &vt, 0, NULL);
        CHECK(vv.last.n_direct == 1 && vv.last.n_xray == 1 && vv.last.n_off == 0);

        /* Without walls, R2 person inside R1 FOV with R1 blind still classifies XRAY. */
        vv.cfg.n_walls = 0;
        memset(&vt, 0, sizeof vt);
        add_node(&vt, "R1", "ok", 0, 0, 0);
        add_node(&vt, "R2", "ok", 0, 2, 180);
        add_track(&vt, "R2", "R2", 1, 0.1f, 4.0f);
        memset(vv.colours, 0, sizeof vv.colours); /* another scene: not the seen person held */
        wv_render(&c, &cfg, &vv, &vt, 0, NULL);
        CHECK(vv.last.n_xray == 1 && vv.last.n_off == 0);

        /* Ask 2: single-node contact with range_m null keeps a track and draws ray + XRAY. */
        seeker_fusion_init(&fus, &cfg);
        seeker_pose r2_side = { 2.0f, 4.0f, 270.0f };
        CHECK(seeker_fusion_set_pose(&fus, "R1", &r1_init, "init", 0) >= 0);
        CHECK(seeker_fusion_set_pose(&fus, "R2", &r2_side, "cli", 1) >= 0);
        const char *js_null_rng = "{\"type\":\"remote_contacts\",\"v\":2,\"node_id\":\"R2\",\"t_mono_ms\":2000,"
                                  "\"contacts\":[{\"id\":\"p1\",\"cls\":\"person\","
                                  "\"bearing_deg\":0,\"range_m\":null}]}";
        CHECK(seeker_decode_remote_contacts(js_null_rng, (int)strlen(js_null_rng), &m2, err, sizeof err) == 0);
        CHECK(m2.n == 1 && !m2.c[0].has_range);
        seeker_fusion_ingest(&fus, &m2, 2000);
        seeker_fusion_tick(&fus, 2010, &vt);
        CHECK(vt.n == 1 && !vt.tracks[0].has_room && !vt.tracks[0].has_range &&
              !strcmp(vt.tracks[0].node_id, "R2"));
        vv.cfg.n_walls = 1;
        nc = wv_classify(&vt, &vv.cfg, &p0, it, SEEKER_MAX_TRACKS);
        CHECK(nc == 1 && it[0].has_bearing && it[0].in_fov && it[0].has_ray_span && !it[0].has_range);
        wv_render(&c, &cfg, &vv, &vt, 0, NULL);
        CHECK(vv.last.n_xray == 1 && vv.last.n_off == 0 && vv.last.n_unplaced == 0);
        CHECK(count_colour(&c, C_CYAN, 0, W, 0, body) > 80);

        /* Range-less contact from R2 pointing outside R1 FOV draws as OFF chevron. */
        seeker_pose r2_away = { 4.0f, 0.0f, 90.0f };
        CHECK(seeker_fusion_set_pose(&fus, "R2", &r2_away, "cli", 1) >= 0);
        seeker_fusion_tick(&fus, 2020, &vt);
        wv_render(&c, &cfg, &vv, &vt, 0, NULL);
        CHECK(vv.last.n_off == 1 && vv.last.n_xray == 0 && vv.last.n_unplaced == 0);
    }
    free(c.px);
    free(vid);
}

static void r1_pose_msg(seeker_fusion *f, const char *pose_json, int64_t t)
{
    char js[256], err[96];
    snprintf(js, sizeof js,
             "{\"type\":\"remote_contacts\",\"v\":2,\"node_id\":\"R1\",\"t_mono_ms\":%lld,"
             "\"pose\":%s,\"contacts\":[]}",
             (long long)t, pose_json);
    seeker_remote_contacts m;
    CHECK(seeker_decode_remote_contacts(js, (int)strlen(js), &m, err, sizeof err) == 0);
    seeker_fusion_ingest(f, &m, t);
}

static int r1_at(const seeker_fusion *f, float x, float y, float yaw, const char *src)
{
    for (int i = 0; i < f->n_nodes; i++)
        if (!strcmp(f->nodes[i].id, "R1"))
            return NEAR(f->nodes[i].x_m, x, 1e-4) && NEAR(f->nodes[i].y_m, y, 1e-4) &&
                   NEAR(f->nodes[i].yaw_deg, yaw, 1e-4) && !strcmp(f->nodes[i].pose_src, src);
    return 0;
}

/* R1's live ArUco pose (phone.html ?tag=1): pose null after a live pose holds the last pose
 * ("held", HUD "POSE HELD ... (NO TAG)"), never 0,0,0; one-frame jumps (planar POSIT mirror
 * flips) are ignored, a real move lands after SEEKER_POSE_JUMP_N messages in a row. */
static void test_tag_pose_hold_and_gate(void)
{
    static seeker_config cfg;
    CHECK(seeker_config_load(&cfg, src_file("config/seeker.eagleeye.json")) == 0);
    static seeker_fusion fus;
    seeker_fusion_init(&fus, &cfg);
    fus.pose_smooth = 0; /* this test is about the gate and holds: each live pose taken as is */
    CHECK(fus.pose_gate == 1);
    seeker_pose r1_init = { 0, 0, 0 };
    CHECK(seeker_fusion_set_pose(&fus, "R1", &r1_init, "init", 0) >= 0);
    wv_config wc;
    wv_config_defaults(&wc);
    snprintf(wc.node, sizeof wc.node, "R1");
    wc.pose_src = WV_POSE_AUTO;
    static seeker_tracks tr, tr2;
    wv_pose rp;
    int live = 9;
    int64_t t = 1000;

    /* No tag yet: starting pose, "POSE START". */
    r1_pose_msg(&fus, "null", t);
    CHECK(r1_at(&fus, 0, 0, 0, "init"));
    seeker_fusion_tick(&fus, t + 5, &tr);
    wv_resolve_pose(&wc, &tr, &rp, &live, NULL, 0);
    CHECK(!live && wv_pose_mode_of(&wc, &tr, live) == WV_MODE_START);

    /* First tag fix is taken at once, however far from the start pose. */
    r1_pose_msg(&fus, "{\"x_m\":0.6,\"y_m\":-2.0,\"yaw_deg\":-15}", t += 66);
    CHECK(r1_at(&fus, 0.6f, -2.0f, -15, "message"));
    seeker_fusion_tick(&fus, t + 5, &tr);
    wv_resolve_pose(&wc, &tr, &rp, &live, NULL, 0);
    CHECK(live && wv_pose_mode_of(&wc, &tr, live) == WV_MODE_LIVE);

    /* Tag drops (phone sends pose null after its 1 s hold): last pose kept, marked held. */
    for (int i = 0; i < 20; i++)
        r1_pose_msg(&fus, "null", t += 66);
    CHECK(r1_at(&fus, 0.6f, -2.0f, -15, "held"));
    seeker_fusion_tick(&fus, t + 5, &tr);
    char tbuf[4096], err[96];
    int tn = seeker_encode_tracks(&tr, tbuf, sizeof tbuf);
    CHECK(tn > 0 && seeker_decode_tracks(tbuf, tn, &tr2, err, sizeof err) == 0);
    wv_resolve_pose(&wc, &tr2, &rp, &live, NULL, 0);
    CHECK(!live && wv_pose_mode_of(&wc, &tr2, live) == WV_MODE_HELD);
    CHECK(NEAR(rp.x_m, 0.6, 1e-4) && NEAR(rp.y_m, -2.0, 1e-4) && NEAR(rp.yaw_deg, -15, 1e-4));
    CHECK(!strcmp(wv_pose_mode_name(WV_MODE_HELD), "held"));
    {
        static wv_view vv;
        memset(&vv, 0, sizeof vv);
        vv.cfg = wc;
        const int W = 320, H = 240;
        canvas c = { calloc((size_t)W * H, 4), W, H, W };
        CHECK(c.px != NULL);
        if (c.px)
            wv_render(&c, &cfg, &vv, &tr2, 0, NULL);
        free(c.px);
        CHECK(vv.last.pose_mode == WV_MODE_HELD && NEAR(vv.last.pose.x_m, 0.6, 1e-4));
    }
    /* A static wearer view never claims HELD. */
    wv_config ws = wc;
    ws.pose_src = WV_POSE_STATIC;
    CHECK(wv_pose_mode_of(&ws, &tr2, 0) == WV_MODE_FIXED);

    /* Tag back after a hold: no jump gate, live again once two poses agree, but eased in (no
     * jump on the HUD): the pose starts where it was held and reaches the tag pose within a
     * second of ticks. */
    r1_pose_msg(&fus, "{\"x_m\":1.5,\"y_m\":-1.0,\"yaw_deg\":40}", t += 66);
    CHECK(r1_at(&fus, 0.6f, -2.0f, -15, "held")); /* one pose back: waits for a second */
    r1_pose_msg(&fus, "{\"x_m\":1.5,\"y_m\":-1.0,\"yaw_deg\":40}", t += 66);
    CHECK(r1_at(&fus, 0.6f, -2.0f, -15, "message"));
    seeker_fusion_tick(&fus, t + 100, &tr);
    for (int i = 0; i < fus.n_nodes; i++)
        if (!strcmp(fus.nodes[i].id, "R1"))
            CHECK(fus.nodes[i].x_m > 0.65f && fus.nodes[i].x_m < 1.5f); /* on its way */
    seeker_fusion_tick(&fus, t + 1000, &tr);
    CHECK(r1_at(&fus, 1.5f, -1.0f, 40, "message"));
    t += 1000;

    /* One-frame mirror flip (x and yaw change sign): ignored. A small move is taken. */
    r1_pose_msg(&fus, "{\"x_m\":-1.5,\"y_m\":-1.0,\"yaw_deg\":-40}", t += 66);
    CHECK(r1_at(&fus, 1.5f, -1.0f, 40, "message"));
    r1_pose_msg(&fus, "{\"x_m\":1.52,\"y_m\":-1.0,\"yaw_deg\":41}", t += 66);
    CHECK(r1_at(&fus, 1.52f, -1.0f, 41, "message"));
    /* Yaw alone past the gate counts too. */
    r1_pose_msg(&fus, "{\"x_m\":1.52,\"y_m\":-1.0,\"yaw_deg\":-20}", t += 66);
    CHECK(r1_at(&fus, 1.52f, -1.0f, 41, "message"));
    /* SEEKER_POSE_JUMP_N jumps in a row are a real move. */
    for (int i = 0; i < SEEKER_POSE_JUMP_N - 1; i++) {
        r1_pose_msg(&fus, "{\"x_m\":-1.5,\"y_m\":-1.0,\"yaw_deg\":-40}", t += 66);
        CHECK(r1_at(&fus, 1.52f, -1.0f, 41, "message"));
    }
    r1_pose_msg(&fus, "{\"x_m\":-1.5,\"y_m\":-1.0,\"yaw_deg\":-40}", t += 66);
    CHECK(r1_at(&fus, -1.5f, -1.0f, -40, "message"));

    /* hud_fusion --no-pose-gate: every full pose is taken. */
    fus.pose_gate = 0;
    r1_pose_msg(&fus, "{\"x_m\":2.5,\"y_m\":-1.0,\"yaw_deg\":0}", t += 66);
    CHECK(r1_at(&fus, 2.5f, -1.0f, 0, "message"));

    /* run-live's config: only R1 and R2, R1 is the wearer and video node, 640x480, no walls. */
    static seeker_config lc;
    CHECK(seeker_config_load(&lc, src_file("config/seeker.live.json")) == 0);
    CHECK(lc.n_nodes == 2 && !strcmp(lc.view_node, "R1"));
    wv_config lw;
    wv_config_load(&lw, &lc, src_file("config/seeker.live.json"), NULL);
    CHECK(!strcmp(lw.node, "R1") && lw.n_walls == 0 && lw.cam.image_w == 640 &&
          lw.cam.image_h == 480 && lw.radar_fallback && NEAR(lw.pose.y_m, -2.0, 1e-5));
}

/* Tag 7 in Felix's room (docs/LIVE-RUN.md, R2 on tag 7): run-live's default TAG7_POSE. */
static const seeker_pose TAG7 = { -0.463f, 0.06f, 90.0f };

static void test_tag_frame_transform(void)
{
    seeker_pose room, back;
    /* 2 m in front of tag 7, facing it: left part of the room, facing +x (toward the corner). */
    seeker_pose front = { 0, -2, 0 };
    seeker_pose_from_tag(&TAG7, &front, &room);
    CHECK(NEAR(room.x_m, -2.463, 1e-4) && NEAR(room.y_m, 0.06, 1e-4) &&
          NEAR(room.yaw_deg, 90, 1e-4));
    /* Stepping toward tag 7 (its y up) moves toward +x, toward tag 0's side of the room. */
    seeker_pose closer = { 0, -1, 0 };
    seeker_pose_from_tag(&TAG7, &closer, &room);
    CHECK(NEAR(room.x_m, -1.463, 1e-4) && NEAR(room.y_m, 0.06, 1e-4));
    /* Tag 7's +x (your right as you face it) is room -y: out toward the front of the room. */
    seeker_pose right = { 1, 0, 0 };
    seeker_pose_from_tag(&TAG7, &right, &room);
    CHECK(NEAR(room.x_m, -0.463, 1e-4) && NEAR(room.y_m, -0.94, 1e-4));
    /* Turning right in tag 7's frame turns right in the room; yaw wraps to (-180, 180]. */
    seeker_pose turned = { 0, -2, 120 };
    seeker_pose_from_tag(&TAG7, &turned, &room);
    CHECK(NEAR(room.yaw_deg, -150, 1e-4));
    /* Tag 0 at the origin is the identity (R1's frame). */
    seeker_pose tag0 = { 0, 0, 0 }, r1 = { 0.6f, -2.0f, -15 };
    seeker_pose_from_tag(&tag0, &r1, &room);
    CHECK(NEAR(room.x_m, 0.6, 1e-6) && NEAR(room.y_m, -2.0, 1e-6) && NEAR(room.yaw_deg, -15, 1e-6));

    /* Round trips over the room in front of both tags, every heading. */
    int ok = 1;
    for (float x = -3.0f; x <= 1.01f; x += 0.5f)
        for (float y = -3.0f; y <= 1.01f; y += 0.5f)
            for (float yaw = -180.0f; yaw <= 180.0f; yaw += 30.0f) {
                seeker_pose p = { x, y, yaw }, l;
                seeker_pose_to_tag(&TAG7, &p, &l);
                seeker_pose_from_tag(&TAG7, &l, &back);
                ok &= NEAR(back.x_m, x, 1e-4) && NEAR(back.y_m, y, 1e-4) &&
                      NEAR(seeker_wrap180(back.yaw_deg - yaw), 0, 1e-3);
                seeker_pose_from_tag(&TAG7, &p, &room);
                seeker_pose_to_tag(&TAG7, &room, &back);
                ok &= NEAR(back.x_m, x, 1e-4) && NEAR(back.y_m, y, 1e-4) &&
                      NEAR(seeker_wrap180(back.yaw_deg - yaw), 0, 1e-3);
                /* A contact placed from the tag-frame pose, then moved to the room, lands where
                 * the room pose places it. */
                float lx, ly, rx, ry;
                seeker_pose_to_world(&l, 15, 2.5f, &lx, &ly);
                seeker_pose pt = { lx, ly, 0 }, ptr;
                seeker_pose_from_tag(&TAG7, &pt, &ptr);
                seeker_pose_to_world(&p, 15, 2.5f, &rx, &ry);
                ok &= NEAR(ptr.x_m, rx, 1e-4) && NEAR(ptr.y_m, ry, 1e-4);
            }
    CHECK(ok);
}

static int node_at(const seeker_fusion *f, const char *id, float x, float y, float yaw,
                   const char *src)
{
    for (int i = 0; i < f->n_nodes; i++)
        if (!strcmp(f->nodes[i].id, id))
            return NEAR(f->nodes[i].x_m, x, 1e-3) && NEAR(f->nodes[i].y_m, y, 1e-3) &&
                   NEAR(seeker_wrap180(f->nodes[i].yaw_deg - yaw), 0, 1e-2) &&
                   !strcmp(f->nodes[i].pose_src, src);
    return 0;
}

/* run-live R2_MODE=tag: hud_fusion --tag-frame R2=TAG7. R2 is not pinned; its tag-7-frame
 * message pose lands at its true room pose, and both phones put one person at one room spot. */
static void test_tag_frame_fusion(void)
{
    static seeker_config cfg;
    CHECK(seeker_config_load(&cfg, src_file("config/seeker.live.json")) == 0);
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    seeker_pose r1_start = { 0, -2, 0 };
    CHECK(seeker_fusion_set_pose(&f, "R1", &r1_start, "init", 0) >= 0);
    int i2 = seeker_fusion_set_tag_frame(&f, "R2", &TAG7);
    CHECK(i2 >= 0 && f.nodes[i2].has_tag_frame && !f.nodes[i2].pose_pinned);
    CHECK(node_at(&f, "R2", -2.463f, 0.06f, 90, "init")); /* starts 2 m in front of tag 7 */

    /* R1 sees tag 0 (room frame as is); R2 sees tag 7 and sends its pose in tag 7's frame. */
    const seeker_pose r1 = { 0.2f, -2.5f, -15 }, r2 = { -2.2f, -0.6f, 90 };
    const float px = -0.4f, py = -1.2f;
    seeker_pose r2_local;
    seeker_pose_to_tag(&TAG7, &r2, &r2_local);
    CHECK(NEAR(r2_local.x_m, 0.66, 1e-4) && NEAR(r2_local.y_m, -1.737, 1e-4) &&
          NEAR(r2_local.yaw_deg, 0, 1e-4));
    seeker_remote_contacts m1 = phone_msg("R1", &r1, px, py, 0);
    seeker_remote_contacts m2 = phone_msg("R2", &r2, px, py, 0);
    m1.has_pose = m2.has_pose = 1;
    m1.x_m = r1.x_m, m1.y_m = r1.y_m, m1.yaw_deg = r1.yaw_deg;
    m2.x_m = r2_local.x_m, m2.y_m = r2_local.y_m, m2.yaw_deg = r2_local.yaw_deg;
    static seeker_tracks t;
    int64_t now = 1000;
    seeker_fusion_ingest(&f, &m1, now);
    seeker_fusion_ingest(&f, &m2, now + 5);
    CHECK(node_at(&f, "R1", r1.x_m, r1.y_m, r1.yaw_deg, "message"));
    CHECK(node_at(&f, "R2", r2.x_m, r2.y_m, r2.yaw_deg, "message"));
    seeker_fusion_tick(&f, now + 10, &t);
    CHECK(t.n == 1 && !strcmp(t.tracks[0].nodes, "R1+R2") && t.tracks[0].has_room);
    CHECK(NEAR(t.tracks[0].room_x_m, px, 1e-2) && NEAR(t.tracks[0].room_y_m, py, 1e-2));
    CHECK(t.n_nodes == 2 && !strcmp(t.nodes[1].node_id, "R2") &&
          NEAR(t.nodes[1].x_m, r2.x_m, 1e-3) && !strcmp(t.nodes[1].pose_src, "message"));

    /* A one-frame mirror flip in tag 7's frame is ignored like R1's. */
    m2.x_m = -r2_local.x_m;
    seeker_fusion_ingest(&f, &m2, now += 66);
    CHECK(node_at(&f, "R2", r2.x_m, r2.y_m, r2.yaw_deg, "message"));
    m2.x_m = r2_local.x_m;

    /* R2 loses tag 7 (pose null): last room pose held, never 0,0,0 or the start pose; R1 stays
     * live. Tracks say "held" (HUD status "R2 ..HZ HOLD"). */
    seeker_remote_contacts m2n = phone_msg("R2", &r2, px, py, 0);
    for (int k = 0; k < 20; k++) {
        seeker_fusion_ingest(&f, &m1, now += 66);
        seeker_fusion_ingest(&f, &m2n, now + 3);
    }
    CHECK(node_at(&f, "R2", r2.x_m, r2.y_m, r2.yaw_deg, "held"));
    CHECK(node_at(&f, "R1", r1.x_m, r1.y_m, r1.yaw_deg, "message"));
    seeker_fusion_tick(&f, now + 10, &t);
    CHECK(!strcmp(t.nodes[1].pose_src, "held") && NEAR(t.nodes[1].y_m, r2.y_m, 1e-3));
    CHECK(t.n == 1 && NEAR(t.tracks[0].room_x_m, px, 1e-2) && NEAR(t.tracks[0].room_y_m, py, 1e-2));

    /* Tag 7 back after R2 walked 1 m toward it: in the room frame, eased in over the ticks. */
    seeker_pose r2b = { r2.x_m + 1.0f, r2.y_m, r2.yaw_deg }, r2b_local;
    seeker_pose_to_tag(&TAG7, &r2b, &r2b_local);
    m2.x_m = r2b_local.x_m, m2.y_m = r2b_local.y_m, m2.yaw_deg = r2b_local.yaw_deg;
    seeker_fusion_ingest(&f, &m2, now += 66);
    seeker_fusion_ingest(&f, &m2, now += 66); /* two poses that agree after the loss */
    seeker_fusion_tick(&f, now + 1000, &t);
    CHECK(node_at(&f, "R2", r2b.x_m, r2b.y_m, r2b.yaw_deg, "message"));

    /* No tag fix ever: R2 stays on its start pose ("init", HUD "R2 ..HZ START"). */
    static seeker_fusion g;
    seeker_fusion_init(&g, &cfg);
    seeker_fusion_set_tag_frame(&g, "R2", &TAG7);
    for (int k = 0; k < 5; k++)
        seeker_fusion_ingest(&g, &m2n, 100 + 66 * k);
    CHECK(node_at(&g, "R2", -2.463f, 0.06f, 90, "init"));
    /* A later --init-pose replaces the start, still unpinned. */
    seeker_pose st = { -2.0f, -1.0f, 80 };
    seeker_fusion_set_pose(&g, "R2", &st, "init", 0);
    CHECK(node_at(&g, "R2", -2.0f, -1.0f, 80, "init") && g.nodes[1].has_tag_frame &&
          !g.nodes[1].pose_pinned);
    /* remote.pose_source "config" still lets a tag-frame node move (it has no fixed pose). */
    cfg.pose_from_config = 1;
    seeker_fusion_init(&g, &cfg);
    seeker_fusion_set_tag_frame(&g, "R2", &TAG7);
    seeker_fusion_ingest(&g, &m2, 100);
    CHECK(node_at(&g, "R2", r2b.x_m, r2b.y_m, r2b.yaw_deg, "message"));
    seeker_fusion_ingest(&g, &m1, 101); /* R1 has no tag frame: config pose kept */
    CHECK(node_at(&g, "R1", 0, -2, 0, "config"));
}

/* R2's pose message the way phone.html sends it (?tag=1), through the real decoder. */
static void r2_pose_msg(seeker_fusion *f, const char *pose_json, int64_t t)
{
    char js[256], err[96];
    snprintf(js, sizeof js,
             "{\"type\":\"remote_contacts\",\"v\":2,\"node_id\":\"R2\",\"t_mono_ms\":%lld,"
             "\"pose\":%s,\"contacts\":[]}",
             (long long)t, pose_json);
    seeker_remote_contacts m;
    CHECK(seeker_decode_remote_contacts(js, (int)strlen(js), &m, err, sizeof err) == 0);
    seeker_fusion_ingest(f, &m, t);
}

/* pose.tag_id names the tag a pose was measured from; only a whole number >= 0 counts. */
static void test_decode_tag_id(void)
{
    const char *cases[][2] = {
        { "{\"x_m\":0.66,\"y_m\":-1.737,\"yaw_deg\":0,\"tag_id\":7}", "7" },
        { "{\"x_m\":0.6,\"y_m\":-2,\"yaw_deg\":-15,\"tag_id\":0}", "0" },
        { "{\"x_m\":0.6,\"y_m\":-2,\"yaw_deg\":-15}", "" },
        { "{\"x_m\":0.6,\"y_m\":-2,\"yaw_deg\":-15,\"tag_id\":null}", "" },
        { "{\"x_m\":0.6,\"y_m\":-2,\"yaw_deg\":-15,\"tag_id\":1.5}", "" },
        { "{\"x_m\":0.6,\"y_m\":-2,\"yaw_deg\":-15,\"tag_id\":-1}", "" },
        { "{\"x_m\":0.6,\"y_m\":-2,\"yaw_deg\":-15,\"tag_id\":\"7\"}", "" },
    };
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        char js[256], err[96];
        snprintf(js, sizeof js,
                 "{\"type\":\"remote_contacts\",\"v\":2,\"node_id\":\"R2\",\"t_mono_ms\":1,"
                 "\"pose\":%s,\"contacts\":[]}", cases[i][0]);
        seeker_remote_contacts m;
        int ok = seeker_decode_remote_contacts(js, (int)strlen(js), &m, err, sizeof err) == 0 &&
                 m.has_pose;
        if (cases[i][1][0])
            ok = ok && m.has_tag_id && m.tag_id == atoi(cases[i][1]);
        else
            ok = ok && !m.has_tag_id;
        CHECK(ok);
    }
}

/* run-live R2_MODE=tag: R2 locates itself from tag 0 when it sees it (pose already in the room
 * frame) and falls back to tag 7 (hud_fusion --tag-pose 7=TAG7). The phone names the tag it used
 * in pose.tag_id; a pose without it is tag 0's (Daniel's current page with ?tagid=0,7 sees only
 * tag 0). Tag switches go through the jump gate and are never stuck. */
static void test_multi_tag(void)
{
    static seeker_config cfg;
    CHECK(seeker_config_load(&cfg, src_file("config/seeker.live.json")) == 0);
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    f.pose_smooth = 0; /* tag frames: each live pose taken as is */
    CHECK(seeker_fusion_set_tag(&f, 7, &TAG7) == 0);
    CHECK(seeker_fusion_set_tag(&f, 0, &TAG7) == -1); /* tag 0 is the room origin */
    seeker_pose start = { -2.463f, 0.06f, 90 };
    CHECK(seeker_fusion_set_pose(&f, "R2", &start, "init", 0) == 1);
    CHECK(!f.nodes[1].pose_pinned && !f.nodes[1].has_tag_frame);
    char js[160];
    int64_t t = 1000;

    /* R2 at room -2.2,-0.6, facing +x, sees tag 0 at an angle: used as is. */
    const seeker_pose r2 = { -2.2f, -0.6f, 90 };
    r2_pose_msg(&f, "{\"x_m\":-2.2,\"y_m\":-0.6,\"yaw_deg\":90,\"tag_id\":0}", t);
    CHECK(node_at(&f, "R2", -2.2f, -0.6f, 90, "message") && f.nodes[1].last_tag == 0);
    /* No tag_id (today's phone.html): room frame as is, no transform. */
    r2_pose_msg(&f, "{\"x_m\":-2.2,\"y_m\":-0.6,\"yaw_deg\":90}", t += 66);
    CHECK(node_at(&f, "R2", -2.2f, -0.6f, 90, "message") && f.nodes[1].last_tag == -1);

    /* Tag 0 out of view, tag 7 in view: falls back to tag 7 at once (same room pose). */
    seeker_pose l7;
    seeker_pose_to_tag(&TAG7, &r2, &l7);
    snprintf(js, sizeof js, "{\"x_m\":%.4f,\"y_m\":%.4f,\"yaw_deg\":%.2f,\"tag_id\":7}", l7.x_m,
             l7.y_m, l7.yaw_deg);
    r2_pose_msg(&f, js, t += 66);
    CHECK(node_at(&f, "R2", r2.x_m, r2.y_m, r2.yaw_deg, "message") && f.nodes[1].last_tag == 7);
    /* R2 walks 0.3 m toward tag 7 on tag 7 alone: follows. */
    seeker_pose r2w = { -1.9f, -0.6f, 90 }, l7w;
    seeker_pose_to_tag(&TAG7, &r2w, &l7w);
    snprintf(js, sizeof js, "{\"x_m\":%.4f,\"y_m\":%.4f,\"yaw_deg\":%.2f,\"tag_id\":7}", l7w.x_m,
             l7w.y_m, l7w.yaw_deg);
    r2_pose_msg(&f, js, t += 66);
    CHECK(node_at(&f, "R2", -1.9f, -0.6f, 90, "message"));
    /* Tag 0 back (preferred by the phone): taken at once. */
    r2_pose_msg(&f, "{\"x_m\":-1.9,\"y_m\":-0.6,\"yaw_deg\":90,\"tag_id\":0}", t += 66);
    CHECK(node_at(&f, "R2", -1.9f, -0.6f, 90, "message") && f.nodes[1].last_tag == 0);

    /* TAG7_POSE measured 0.9 m off: tag 7 disagrees with tag 0 by more than the gate. A
     * flicker (tag 7 for one frame, then tag 0 again) never moves R2 ... */
    static const seeker_pose tag7_off = { -0.463f, 0.96f, 90 };
    CHECK(seeker_fusion_set_tag(&f, 7, &tag7_off) == 0 && f.n_tags == 1);
    for (int k = 0; k < 6; k++) {
        r2_pose_msg(&f, k % 2 ? "{\"x_m\":-1.9,\"y_m\":-0.6,\"yaw_deg\":90,\"tag_id\":0}" : js,
                    t += 66);
        CHECK(node_at(&f, "R2", -1.9f, -0.6f, 90, "message"));
    }
    /* ... but staying on tag 7 is a real switch: taken on the SEEKER_POSE_JUMP_N-th frame. */
    for (int k = 1; k < SEEKER_POSE_JUMP_N; k++) {
        r2_pose_msg(&f, js, t += 66);
        CHECK(node_at(&f, "R2", -1.9f, -0.6f, 90, "message"));
    }
    r2_pose_msg(&f, js, t += 66);
    CHECK(node_at(&f, "R2", -1.9f, 0.3f, 90, "message") && f.nodes[1].last_tag == 7);
    /* And back to tag 0 the same way, not stuck on tag 7. */
    for (int k = 1; k < SEEKER_POSE_JUMP_N; k++)
        r2_pose_msg(&f, "{\"x_m\":-1.9,\"y_m\":-0.6,\"yaw_deg\":90,\"tag_id\":0}", t += 66);
    CHECK(node_at(&f, "R2", -1.9f, 0.3f, 90, "message"));
    r2_pose_msg(&f, "{\"x_m\":-1.9,\"y_m\":-0.6,\"yaw_deg\":90,\"tag_id\":0}", t += 66);
    CHECK(node_at(&f, "R2", -1.9f, -0.6f, 90, "message") && f.nodes[1].last_tag == 0);

    /* No tag in view (pose null): held, never 0,0,0. A tag with no room pose counts as null. */
    r2_pose_msg(&f, "null", t += 66);
    CHECK(node_at(&f, "R2", -1.9f, -0.6f, 90, "held"));
    r2_pose_msg(&f, "{\"x_m\":0,\"y_m\":0,\"yaw_deg\":0,\"tag_id\":5}", t += 66);
    CHECK(node_at(&f, "R2", -1.9f, -0.6f, 90, "held") && f.nodes[1].warned_tag == 5);
    r2_pose_msg(&f, "{\"x_m\":-1.0,\"y_m\":-1.5,\"yaw_deg\":60,\"tag_id\":0}", t += 66);
    {
        static seeker_tracks settle; /* back after a hold: eased in within a second */
        seeker_fusion_tick(&f, t += 1000, &settle);
    }
    CHECK(node_at(&f, "R2", -1.0f, -1.5f, 60, "message"));

    /* R1 (tag 0 only) and R2 (on tag 7) put one person at one room spot. */
    CHECK(seeker_fusion_set_tag(&f, 7, &TAG7) == 0);
    const seeker_pose r1 = { 0.2f, -2.5f, -15 };
    const float px = -0.4f, py = -1.2f;
    seeker_remote_contacts m1 = phone_msg("R1", &r1, px, py, 0);
    seeker_remote_contacts m2 = phone_msg("R2", &r2, px, py, 0);
    m1.has_pose = m2.has_pose = 1;
    m1.x_m = r1.x_m, m1.y_m = r1.y_m, m1.yaw_deg = r1.yaw_deg;
    m1.has_tag_id = 1, m1.tag_id = 0;
    m2.x_m = l7.x_m, m2.y_m = l7.y_m, m2.yaw_deg = l7.yaw_deg;
    m2.has_tag_id = 1, m2.tag_id = 7;
    static seeker_tracks tr;
    for (int k = 0; k < SEEKER_POSE_JUMP_N; k++) { /* R2 moved 1.2 m since: a gated jump */
        seeker_fusion_ingest(&f, &m1, t += 66);
        seeker_fusion_ingest(&f, &m2, t + 3);
    }
    CHECK(node_at(&f, "R2", r2.x_m, r2.y_m, r2.yaw_deg, "message"));
    seeker_fusion_tick(&f, t + 10, &tr);
    CHECK(tr.n == 1 && !strcmp(tr.tracks[0].nodes, "R1+R2") && tr.tracks[0].has_room);
    CHECK(NEAR(tr.tracks[0].room_x_m, px, 1e-2) && NEAR(tr.tracks[0].room_y_m, py, 1e-2));

    /* --tag-frame (one unnamed tag) still applies to untagged poses only; a tag_id wins. */
    static seeker_fusion g;
    seeker_fusion_init(&g, &cfg);
    seeker_fusion_set_tag_frame(&g, "R2", &TAG7);
    seeker_fusion_set_tag(&g, 7, &TAG7);
    snprintf(js, sizeof js, "{\"x_m\":%.4f,\"y_m\":%.4f,\"yaw_deg\":%.2f}", l7.x_m, l7.y_m,
             l7.yaw_deg);
    r2_pose_msg(&g, js, 100);
    CHECK(node_at(&g, "R2", r2.x_m, r2.y_m, r2.yaw_deg, "message"));
    r2_pose_msg(&g, "{\"x_m\":-2.0,\"y_m\":-0.5,\"yaw_deg\":85,\"tag_id\":0}", 166);
    CHECK(node_at(&g, "R2", -2.0f, -0.5f, 85, "message"));
}

/* ---- ghost skeleton (pts): decode, budget, person-local, smoothing, fusion, render ---- */

/* A standing person in person-height units from the mid-hip, x right, y down (COCO-17 order). */
static const float SKEL_T[SEEKER_N_JOINTS][2] = {
    { 0.00f, -0.43f }, { -0.02f, -0.45f }, { 0.02f, -0.45f }, { -0.05f, -0.44f },
    { 0.05f, -0.44f }, { -0.12f, -0.29f }, { 0.12f, -0.29f }, { -0.15f, -0.12f },
    { 0.15f, -0.12f }, { -0.16f, 0.03f },  { 0.16f, 0.03f },  { -0.08f, 0.00f },
    { 0.08f, 0.00f },  { -0.08f, 0.25f },  { 0.08f, 0.25f },  { -0.08f, 0.48f },
    { 0.08f, 0.48f },
};

/* Image-normalized landmarks of SKEL_T for a phone frame w x h (hfov 66) with the mid-hip at
 * pixel (ax, ay) and the person range_m away. */
static void skel_image(seeker_contact *c, int w, int h, float range_m, float ax, float ay,
                       float noise)
{
    float f = (w * 0.5f) / tanf(33.0f * 0.017453293f), hpx = f * 1.7f / range_m;
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        c->pts[j][0] = (ax + (SKEL_T[j][0] + noise * (float)gauss()) * hpx) / (float)w;
        c->pts[j][1] = (ay + (SKEL_T[j][1] + noise * (float)gauss()) * hpx) / (float)h;
        c->pts[j][2] = 0.9f;
    }
    c->has_pts = 1;
}

static float skel_err(const float (*p)[3])
{
    float e = 0;
    for (int j = 0; j < SEEKER_N_JOINTS; j++)
        e = fmaxf(e, hypotf(p[j][0] - SKEL_T[j][0], p[j][1] - SKEL_T[j][1]));
    return e;
}

static void test_pts_decode(void)
{
    char js[8192], pts[1024];
    int n = 0;
    for (int j = 0; j < SEEKER_N_JOINTS; j++)
        n += snprintf(pts + n, sizeof pts - (size_t)n, "%s%d,%d,%d", j ? "," : "", 400 + j,
                      -20 + 70 * j, j == 3 ? 150 : 90);
    snprintf(js, sizeof js,
             "{\"type\":\"remote_contacts\",\"v\":2,\"node_id\":\"R2\",\"ts_ms\":5,"
             "\"contacts\":[{\"id\":\"R2-p1\",\"bearing_deg\":-12.5,\"range_m\":2.4,"
             "\"box\":[100,40,80,300],\"pts\":[%s]},"
             "{\"id\":\"R2-p2\",\"bearing_deg\":3,\"range_m\":null,\"pts\":[1,2,3]},"
             "{\"id\":\"R2-p3\",\"bearing_deg\":4,\"pts\":null}]}",
             pts);
    static seeker_remote_contacts m;
    char err[64];
    CHECK(seeker_decode_remote_contacts(js, (int)strlen(js), &m, err, sizeof err) == 0 && m.n == 3);
    CHECK(m.c[0].has_pts && NEAR(m.c[0].pts[0][0], 0.400, 1e-6) && NEAR(m.c[0].pts[0][1], -0.020, 1e-6));
    CHECK(NEAR(m.c[0].pts[16][0], 0.416, 1e-6) && NEAR(m.c[0].pts[16][1], 1.100, 1e-6)); /* not clamped */
    CHECK(NEAR(m.c[0].pts[0][2], 0.9, 1e-6) && NEAR(m.c[0].pts[3][2], 1.0, 1e-6)); /* vis clamped */
    CHECK(!m.c[1].has_pts && !m.c[2].has_pts);

    /* 16 contacts with skeletons, the phone's worst case: under 8 KB and decoded whole. */
    n = snprintf(js, sizeof js, "{\"type\":\"remote_contacts\",\"v\":2,\"node_id\":\"R2\","
                                "\"ts_ms\":5,\"width\":640,\"height\":480,\"hfov_deg\":66,"
                                "\"pose\":null,\"contacts\":[");
    for (int i = 0; i < SEEKER_MAX_CONTACTS; i++) {
        n += snprintf(js + n, sizeof js - (size_t)n,
                      "%s{\"id\":\"R2-p%d\",\"bearing_deg\":-12.3,\"conf\":0.93,"
                      "\"box\":[123,45,67,289],\"range_m\":2.45,\"range_src\":\"full\",\"pts\":[",
                      i ? "," : "", i);
        for (int j = 0; j < SEEKER_N_JOINTS; j++)
            n += snprintf(js + n, sizeof js - (size_t)n, "%s%d,%d,%d", j ? "," : "", 512 + j,
                          634 - j, 97);
        n += snprintf(js + n, sizeof js - (size_t)n, "]}");
    }
    n += snprintf(js + n, sizeof js - (size_t)n, "]}");
    printf("16 contacts with pts: %d bytes (limit %d)\n", n, SEEKER_MAX_DATAGRAM);
    CHECK(n < SEEKER_MAX_DATAGRAM);
    CHECK(seeker_decode_remote_contacts(js, n, &m, err, sizeof err) == 0 && m.n == 16);
    int all = 1;
    for (int i = 0; i < m.n; i++)
        all &= m.c[i].has_pts && NEAR(m.c[i].pts[5][0], 0.517, 1e-6);
    CHECK(all);

    /* Token budget: a datagram dense with small numbers (here an unknown field a future page
     * might add) still decodes; the old 2048-token cap dropped the whole datagram. */
    n = snprintf(js, sizeof js, "{\"type\":\"remote_contacts\",\"v\":2,\"node_id\":\"R2\","
                                "\"contacts\":[{\"bearing_deg\":1}],\"future\":[");
    for (int i = 0; i < 3000; i++)
        n += snprintf(js + n, sizeof js - (size_t)n, "%s0", i ? "," : "");
    n += snprintf(js + n, sizeof js - (size_t)n, "]}");
    CHECK(n < SEEKER_MAX_DATAGRAM);
    CHECK(seeker_decode_remote_contacts(js, n, &m, err, sizeof err) == 0 && m.n == 1);
}

static void test_tracks_pts(void)
{
    static seeker_tracks t, back;
    memset(&t, 0, sizeof t);
    snprintf(t.src, sizeof t.src, "hud_fusion");
    t.n = SEEKER_MAX_TRACKS;
    for (int i = 0; i < t.n; i++) { /* the worst case of test_tracks_roundtrip, all with pts */
        seeker_track *k = &t.tracks[i];
        k->id = i + 1;
        snprintf(k->label, sizeof k->label, "R1-%d+R2-%d", i, i);
        snprintf(k->cls, sizeof k->cls, "person");
        snprintf(k->src, sizeof k->src, "remote");
        snprintf(k->colour, sizeof k->colour, "cyan");
        snprintf(k->node_id, sizeof k->node_id, "R2");
        snprintf(k->nodes, sizeof k->nodes, "%s", i % 2 ? "R2" : "R1+R2"); /* odd: single node */
        snprintf(k->range_src, sizeof k->range_src, "remote");
        k->has_range = k->has_bearing = k->has_room = k->has_node_box = k->has_pts = 1;
        k->range_m = 12.34f;
        k->bearing_deg = -123.4f;
        k->room_x_m = -5.5f;
        k->room_y_m = 7.25f;
        k->node_box[0] = 100; k->node_box[1] = 50; k->node_box[2] = 40; k->node_box[3] = 160;
        for (int j = 0; j < SEEKER_N_JOINTS; j++) {
            k->pts[j][0] = -SKEL_T[j][0] - 0.5f;
            k->pts[j][1] = SKEL_T[j][1] - 0.4f;
            k->pts[j][2] = 0.87f;
        }
    }
    t.n_nodes = SEEKER_MAX_NODES;
    for (int i = 0; i < t.n_nodes; i++) {
        snprintf(t.nodes[i].node_id, sizeof t.nodes[i].node_id, "R%d", (i + 1) % 100);
        snprintf(t.nodes[i].state, sizeof t.nodes[i].state, "lost");
        snprintf(t.nodes[i].pose_src, sizeof t.nodes[i].pose_src, "message");
    }
    char buf[SEEKER_MAX_DATAGRAM], err[64];
    int len = seeker_encode_tracks(&t, buf, sizeof buf);
    printf("worst-case tracks datagram with pts: %d bytes (limit %d)\n", len, SEEKER_MAX_DATAGRAM);
    CHECK(len > 0 && len <= SEEKER_MAX_DATAGRAM);
    CHECK(seeker_decode_tracks(buf, len, &back, err, sizeof err) == 0 && back.n == t.n &&
          back.n_nodes == t.n_nodes);
    int carriers = 0, multi = 0;
    for (int i = 0; i < back.n; i++) {
        carriers += back.tracks[i].has_pts;
        multi += back.tracks[i].has_pts && strchr(back.tracks[i].nodes, '+');
    }
    printf("  %d tracks carry pts (cap %d)\n", carriers, SEEKER_PTS_MAX_CARRIERS);
    CHECK(carriers >= 3 && carriers <= SEEKER_PTS_MAX_CARRIERS && multi == 0);
    CHECK(back.tracks[1].has_pts && NEAR(back.tracks[1].pts[5][0], 0.12 - 0.5, 1.5e-3) &&
          NEAR(back.tracks[1].pts[16][1], 0.08, 1.5e-3) && NEAR(back.tracks[1].pts[0][2], 0.87, 6e-3));
    /* A buffer that holds the tracks but not their pts sheds every pts, never a track. */
    t.n = 2;
    static seeker_tracks bare;
    bare = t;
    bare.tracks[0].has_pts = bare.tracks[1].has_pts = 0;
    int need = seeker_encode_tracks(&bare, buf, sizeof buf);
    int small = seeker_encode_tracks(&t, buf, need + 100);
    CHECK(need > 0 && small == need && seeker_decode_tracks(buf, small, &back, err, sizeof err) == 0 &&
          back.n == 2 && !back.tracks[0].has_pts && !back.tracks[1].has_pts);
    CHECK(seeker_encode_tracks(&t, buf, need - 1) == -1);
}

static void test_skel_local(void)
{
    seeker_contact c;
    memset(&c, 0, sizeof c);
    float out[SEEKER_N_JOINTS][3];
    skel_image(&c, 640, 480, 3.0f, 400, 250, 0);
    CHECK(seeker_skel_local(&c, 640, 480, 66, 3.0f, out) && skel_err(out) < 1e-4);
    /* The same person seen at another range and place: same person-local skeleton. */
    skel_image(&c, 640, 480, 1.5f, 200, 300, 0);
    CHECK(seeker_skel_local(&c, 640, 480, 66, 1.5f, out) && skel_err(out) < 1e-4);
    /* Hips hidden: anchored on the shoulders, still at the mid-hip. */
    c.pts[11][2] = c.pts[12][2] = 0.1f;
    CHECK(seeker_skel_local(&c, 640, 480, 66, 1.5f, out) && skel_err(out) < 1e-4);
    /* No hips, no shoulders: the box centre, else nothing. */
    c.pts[5][2] = c.pts[6][2] = 0.1f;
    CHECK(!seeker_skel_local(&c, 640, 480, 66, 1.5f, out));
    c.has_box = 1;
    c.box[0] = 150; c.box[1] = 100; c.box[2] = 100; c.box[3] = 400;
    CHECK(seeker_skel_local(&c, 640, 480, 66, 1.5f, out));
    CHECK(!seeker_skel_local(&c, 640, 480, 66, 0.05f, out) && !seeker_skel_local(&c, 0, 0, 66, 2, out));
    c.has_pts = 0;
    CHECK(!seeker_skel_local(&c, 640, 480, 66, 2, out));
}

/* One raw frame of SKEL_T with noise, joint 9 (left wrist) at wx. */
static void skel_raw(float raw[SEEKER_N_JOINTS][3], float noise, float wx)
{
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        raw[j][0] = SKEL_T[j][0] + noise * (float)gauss();
        raw[j][1] = SKEL_T[j][1] + noise * (float)gauss();
        raw[j][2] = 0.9f;
    }
    raw[9][0] = wx;
}

static void test_skel_smooth(void)
{
    static seeker_skel s;
    float raw[SEEKER_N_JOINTS][3], out[SEEKER_N_JOINTS][3];
    const float (*rp)[3] = (const float (*)[3])raw;
    /* Standing still with landmark jitter (sigma 0.02 person heights, a few px): at 15 Hz the
     * smoothed joints wander far less than the raw ones. */
    memset(&s, 0, sizeof s);
    double e_raw = 0, e_out = 0;
    int ne = 0;
    int64_t t = 1000;
    for (int i = 0; i < 60; i++, t += 66) {
        skel_raw(raw, 0.02f, SKEL_T[9][0] + 0.02f * (float)gauss());
        CHECK(seeker_skel_step(&s, "R2", i + 1, t, rp, 1, out) == 1);
        if (i >= 15)
            for (int j = 0; j < SEEKER_N_JOINTS; j++) {
                e_raw += pow(raw[j][0] - SKEL_T[j][0], 2) + pow(raw[j][1] - SKEL_T[j][1], 2);
                e_out += pow(out[j][0] - SKEL_T[j][0], 2) + pow(out[j][1] - SKEL_T[j][1], 2);
                ne++;
            }
    }
    e_raw = sqrt(e_raw / ne);
    e_out = sqrt(e_out / ne);
    printf("skeleton jitter: raw %.4f, smoothed %.4f person heights rms (x%.2f)\n", e_raw, e_out,
           e_out / e_raw);
    CHECK(e_out < 0.6 * e_raw);

    /* A waving wrist (1.2 person heights per second, no noise): little lag, no trail. */
    float wx = SKEL_T[9][0];
    for (int i = 0; i < 12; i++, t += 66) {
        wx += 1.2f * 0.066f;
        skel_raw(raw, 0, wx);
        seeker_skel_step(&s, "R2", 100 + i, t, rp, 1, out);
    }
    printf("skeleton lag on a 1.2 /s wrist: %.3f person heights\n", wx - out[9][0]);
    CHECK(wx - out[9][0] < 0.12f && out[9][0] <= wx + 1e-4f);
    /* A jump past SEEKER_SKEL_SNAP (another person's landmarks): restarts there, no slide. */
    skel_raw(raw, 0, wx - 0.9f);
    seeker_skel_step(&s, "R2", 200, t += 66, rp, 1, out);
    CHECK(NEAR(out[9][0], wx - 0.9f, 1e-5));

    /* The same datagram on the next ticks: same output, no filter step. */
    float keep = out[0][1];
    skel_raw(raw, 0.3f, 0);
    seeker_skel_step(&s, "R2", 200, t, rp, 1, out);
    CHECK(out[0][1] == keep);

    /* Visibility: a joint under 0.3 is held 300 ms, then hidden; never drawn if never seen. */
    memset(&s, 0, sizeof s);
    t = 5000;
    skel_raw(raw, 0, SKEL_T[9][0]);
    raw[15][2] = 0.1f; /* left ankle behind a desk from the start */
    seeker_skel_step(&s, "R2", 1, t, rp, 1, out);
    CHECK(out[15][2] == 0 && out[16][2] >= 0.3f);
    raw[16][2] = 0.1f; /* right ankle drops out */
    seeker_skel_step(&s, "R2", 2, t += 200, rp, 1, out);
    CHECK(out[16][2] >= 0.3f && NEAR(out[16][1], SKEL_T[16][1], 1e-5)); /* held */
    seeker_skel_step(&s, "R2", 3, t += 200, rp, 1, out);
    CHECK(out[16][2] == 0 && out[0][2] >= 0.3f); /* gone after 400 ms; the rest stays */

    /* Datagrams without a skeleton: the last one is held 500 ms, then none. */
    CHECK(seeker_skel_step(&s, "R2", 4, t += 300, NULL, 1, out) == 1);
    CHECK(seeker_skel_step(&s, "R2", 5, t += 300, NULL, 1, out) == 0);
    /* Another phone takes over as primary: its view restarts every joint. */
    skel_raw(raw, 0, 0.4f);
    CHECK(seeker_skel_step(&s, "R1", 1, t += 66, rp, 1, out) == 1 && NEAR(out[9][0], 0.4, 1e-6));
    /* --no-skel-smooth passes joints straight through. */
    skel_raw(raw, 0.05f, 0.1f);
    seeker_skel_step(&s, "R1", 2, t += 66, rp, 0, out);
    CHECK(NEAR(out[3][0], raw[3][0], 1e-6) && NEAR(out[9][0], 0.1, 1e-6));
}

/* R2 alone sees a person (the wall case) with a skeleton: the track carries it in person units
 * through hud_fusion and the 47102 datagram, smoothed. */
static void test_skel_fusion(void)
{
    static seeker_config cfg;
    CHECK(seeker_config_load(&cfg, src_file("config/seeker.live.json")) == 0);
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    const seeker_pose r2 = { -2.2f, -0.6f, 90 };
    CHECK(seeker_fusion_set_pose(&f, "R2", &r2, "cli", 1) >= 0);
    const float px = -0.4f, py = -1.2f;
    static seeker_tracks t, back;
    char buf[SEEKER_MAX_DATAGRAM], err[64];
    double worst_raw = 0, worst_out = 0;
    int64_t now = 1000;
    for (int i = 0; i < 30; i++, now += 66) {
        seeker_remote_contacts m = phone_msg("R2", &r2, px, py, 1);
        m.width = 640;
        m.height = 480;
        m.hfov_deg = 66;
        float f_px = 320.0f / tanf(33.0f * 0.017453293f);
        float u = 320.0f + f_px * tanf(m.c[0].bearing_deg * 0.017453293f);
        skel_image(&m.c[0], 640, 480, m.c[0].range_m, u, 240, i ? 0.02f : 0);
        float raw[SEEKER_N_JOINTS][3];
        seeker_skel_local(&m.c[0], 640, 480, 66, m.c[0].range_m, raw);
        seeker_fusion_ingest(&f, &m, now);
        seeker_fusion_tick(&f, now + 5, &t);
        CHECK(t.n == 1 && t.tracks[0].has_pts);
        if (i == 0)
            CHECK(skel_err((const float (*)[3])t.tracks[0].pts) < 1e-3);
        if (i >= 10) {
            worst_raw = fmax(worst_raw, skel_err((const float (*)[3])raw));
            worst_out = fmax(worst_out, skel_err((const float (*)[3])t.tracks[0].pts));
        }
    }
    printf("fused skeleton, worst joint error: raw %.3f, sent %.3f person heights\n", worst_raw,
           worst_out);
    CHECK(worst_out < worst_raw);
    int len = seeker_encode_tracks(&t, buf, sizeof buf);
    CHECK(len > 0 && seeker_decode_tracks(buf, len, &back, err, sizeof err) == 0);
    CHECK(back.tracks[0].has_pts && NEAR(back.tracks[0].pts[6][0], t.tracks[0].pts[6][0], 1e-3));
    /* The phone stops sending pts (?skel=0): the ghost goes after the hold. */
    for (int i = 0; i < 12; i++, now += 66) {
        seeker_remote_contacts m = phone_msg("R2", &r2, px, py, 1);
        seeker_fusion_ingest(&f, &m, now);
        seeker_fusion_tick(&f, now + 5, &t);
    }
    CHECK(t.n == 1 && !t.tracks[0].has_pts);
}

/* The wearer view draws an x-ray person with pts as a ghost skeleton (thin hull, bones), and
 * keeps the plain dashed box and diamond without. */
static void test_skel_render(void)
{
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    v.cfg.inset = 0;
    const int W = 640, H = 360;
    canvas c = { calloc((size_t)W * H, 4), W, H, W };
    CHECK(c.px != NULL);
    if (!c.px)
        return;
    static seeker_tracks t;
    memset(&t, 0, sizeof t);
    add_node(&t, "W", "ok", 0, 0, 0);
    add_node(&t, "R2", "ok", -2, 1, 90);
    seeker_track *k = add_track(&t, "R2", "R2", 1, 0.0f, 3.0f); /* straight ahead, x-ray */
    int body = H - 70;
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    int plain = count_colour(&c, C_CYAN, 0, W, 0, body);
    CHECK(v.last.n_xray == 1 && v.last.n_skel == 0 && plain > 50);
    k->has_pts = 1;
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        k->pts[j][0] = SKEL_T[j][0];
        k->pts[j][1] = SKEL_T[j][1];
        k->pts[j][2] = 0.9f;
    }
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    CHECK(v.last.n_xray == 1 && v.last.n_skel == 1);
    /* Bones in the middle of the screen: the torso column between the shoulders, around the
     * centre line, is cyan; the plain marker had only its diamond there. */
    int torso = count_colour(&c, C_CYAN, W / 2 - 40, W / 2 + 40, H / 4, body);
    printf("ghost skeleton: %d cyan px in the torso band (plain marker %d px on the whole view)\n",
           torso, plain);
    CHECK(torso > 40);
    /* Every joint hidden: back to the plain marker. */
    for (int j = 0; j < SEEKER_N_JOINTS; j++)
        k->pts[j][2] = 0;
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    CHECK(v.last.n_skel == 0 && v.last.n_xray == 1);
    /* Seen people never get a ghost, pts or not. */
    snprintf(k->nodes, sizeof k->nodes, "W+R2");
    k->pts[0][2] = 0.9f;
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    CHECK(v.last.n_direct == 1 && v.last.n_skel == 0);
    free(c.px);
}

/* ---- plan theme primitives (hud_display --theme plan) ---- */

static void test_plan_primitives(void)
{
    /* Library default stays classic, so every other test sees the old pixels. */
    CHECK(draw_theme() == DRAW_THEME_CLASSIC);
    CHECK(draw_theme_parse("plan") == DRAW_THEME_PLAN && draw_theme_parse("classic") == 0 &&
          draw_theme_parse("glass") == -1 && draw_theme_parse(NULL) == -1);
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    CHECK(!strcmp(cfg.theme, "plan")); /* hud_display's default */
    CHECK(draw_mix(C_BLACK, C_WHITE, 128) == RGB(128, 128, 128));
    CHECK(draw_mix(RGB(10, 20, 30), C_RED, 0) == RGB(10, 20, 30));
    CHECK(draw_mix(RGB(10, 20, 30), C_RED, 255) == C_RED);

    /* The font set: real lowercase, the mono is fixed pitch, the sans proportional, heavier
     * weights put down more ink. Loose bounds: the family is swappable (tools/gen_hud_fonts.py). */
    CHECK(draw_ui_width(18, DRAW_FONT_MONO_REGULAR, "iiii") == draw_ui_width(18, DRAW_FONT_MONO_REGULAR, "WWWW"));
    CHECK(draw_ui_width(18, DRAW_FONT_SANS, "Behind wall") > 50 && draw_ui_width(18, DRAW_FONT_SANS, "") == 0);
    CHECK(draw_ui_width(18, DRAW_FONT_SANS, "iiii") < draw_ui_width(18, DRAW_FONT_SANS, "WWWW"));
    CHECK(draw_ui_width(18, DRAW_FONT_MONO, "iiii") == draw_ui_width(18, DRAW_FONT_MONO, "WWWW"));
    CHECK(draw_ui_width(18, DRAW_FONT_MONO, "4.7\xc2\xb0") == draw_ui_width(18, DRAW_FONT_MONO, "4.70"));
    int cap = draw_ui_cap(20, DRAW_FONT_SANS);
    CHECK(cap >= 10 && cap <= 17);
    enum { W = 120, H = 60 };
    static uint32_t px[W * H];
    canvas c = { px, W, H, W };
    long ink[3];
    static const int faces[3] = { DRAW_FONT_SANS, DRAW_FONT_SANS_MEDIUM, DRAW_FONT_SANS_SEMIBOLD };
    for (int f = 0; f < 3; f++) {
        draw_clear(&c, C_BLACK);
        draw_ui_text(&c, 4, 40, 24, faces[f], C_WHITE, 255, "Hall");
        ink[f] = 0;
        for (int i = 0; i < W * H; i++)
            ink[f] += px[i] & 255;
    }
    CHECK(ink[0] > 0 && ink[1] > ink[0] && ink[2] > ink[1]);
    /* Cut-corner plate: filled inside, the cut corner stays clear, the edge is drawn. */
    draw_clear(&c, C_BLACK);
    draw_poly_style ps = { 0 };
    ps.fill = C_RED;
    ps.fill_a = 255;
    ps.edge = C_WHITE;
    ps.edge_a = 255;
    ps.edge_w = 2;
    draw_cut_rect(&c, 10, 10, 60, 30, 12, &ps);
    CHECK(px[25 * W + 40] == C_RED);                       /* inside */
    CHECK(px[12 * W + 67] == C_BLACK);                     /* in the cut corner */
    CHECK(px[10 * W + 30] == C_WHITE || px[11 * W + 30] == C_WHITE); /* top edge */
    /* Hatch only inside the shape. */
    draw_clear(&c, C_BLACK);
    draw_poly_style hs = { 0 };
    hs.hatch = C_WHITE;
    hs.hatch_a = 255;
    hs.hatch_gap = 6;
    hs.hatch_w = 2;
    draw_cut_rect(&c, 20, 20, 40, 20, 0, &hs);
    int lit = 0, outside = 0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            int in = x >= 20 && x < 60 && y >= 20 && y < 40;
            if (px[y * W + x] != C_BLACK) {
                if (in)
                    lit++;
                else
                    outside++;
            }
        }
    CHECK(lit > 100 && lit < 40 * 20 && outside < 40);
    /* Dashed circle: on and off around the ring. */
    draw_clear(&c, C_BLACK);
    draw_circle_aa(&c, 60, 30, 20, 2, 8, 6, C_WHITE, 255);
    int on = 0;
    for (int a = 0; a < 360; a += 3) {
        float rad = (float)a * 0.0174533f;
        on += px[(int)(30 + 20 * sinf(rad)) * W + (int)(60 + 20 * cosf(rad))] != C_BLACK;
    }
    CHECK(on > 40 && on < 100);

    /* Caps on a horizontal 6 px line from x 20 to 60 at y 30. */
    draw_clear(&c, C_BLACK);
    draw_line_aa(&c, 20, 30, 60, 30, 6, DRAW_CAP_BUTT, C_WHITE, 255);
    CHECK(px[30 * W + 40] == C_WHITE && px[30 * W + 18] == C_BLACK && px[30 * W + 62] == C_BLACK);
    CHECK(px[25 * W + 40] == C_BLACK && px[34 * W + 40] == C_BLACK); /* 3 px each side, no more */
    draw_clear(&c, C_BLACK);
    draw_line_aa(&c, 20, 30, 60, 30, 6, DRAW_CAP_SQUARE, C_WHITE, 255);
    CHECK(px[28 * W + 18] == C_WHITE && px[28 * W + 61] == C_WHITE); /* corners past the ends */
    draw_clear(&c, C_BLACK);
    draw_line_aa(&c, 20, 30, 60, 30, 6, DRAW_CAP_ROUND, C_WHITE, 255);
    CHECK(px[30 * W + 18] == C_WHITE && px[27 * W + 17] != C_WHITE); /* reaches out, corner cut */
    /* Dashes: 6 on, 6 off from x 10, butt caps: on at 12, off at 19, on again at 25. */
    draw_clear(&c, C_BLACK);
    draw_dashed_aa(&c, 10, 30, 110, 30, 2, 6, 6, DRAW_CAP_BUTT, C_WHITE, 255);
    CHECK(px[30 * W + 12] == C_WHITE && px[30 * W + 19] == C_BLACK && px[30 * W + 25] == C_WHITE);
    /* Alpha: a half-strength line over black is mid grey. */
    draw_clear(&c, C_BLACK);
    draw_line_aa(&c, 20, 30, 60, 30, 6, DRAW_CAP_BUTT, C_WHITE, 128);
    CHECK(px[30 * W + 40] == RGB(128, 128, 128));
}

static int count_rgb(const canvas *c, uint32_t col, int x0, int x1, int y0, int y1)
{
    int n = 0;
    for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++)
            n += c->px[y * c->stride + x] == col;
    return n;
}

/* Pixels within `tol` (RGB distance) of a colour: anti-aliased 1 px lines rarely hit it exactly. */
static int count_near_rgb(const canvas *c, uint32_t col, int tol, int x0, int x1, int y0, int y1)
{
    int n = 0;
    for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++) {
            uint32_t p = c->px[y * c->stride + x];
            int dr = (int)((p >> 16) & 255) - (int)((col >> 16) & 255);
            int dg = (int)((p >> 8) & 255) - (int)((col >> 8) & 255);
            int db = (int)(p & 255) - (int)(col & 255);
            n += dr * dr + dg * dg + db * db <= tol * tol;
        }
    return n;
}

/* Pixels with the hidden-line hue (pale yellow over a dark frame), at any anti-aliased strength. */
/* Pixels in the plan theme's behind-a-wall green (P_HIDDEN, also blended over video). */
static int count_hidden(const canvas *c, int x0, int x1, int y0, int y1)
{
    int n = 0;
    for (int y = y0; y < y1; y++)
        for (int x = x0; x < x1; x++) {
            uint32_t p = c->px[y * c->stride + x];
            int r = (int)((p >> 16) & 255), g = (int)((p >> 8) & 255), b = (int)(p & 255);
            n += g >= 80 && g > r + 40 && g > b + 15;
        }
    return n;
}

/* The plan theme draws the same people as classic, in its own marks and words. */
static void test_plan_render(void)
{
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    const int W = 640, H = 360;
    canvas c = { calloc((size_t)W * H, 4), W, H, W };
    CHECK(c.px != NULL);
    if (!c.px)
        return;
    static seeker_tracks t;
    memset(&t, 0, sizeof t);
    add_node(&t, "W", "ok", 0, 0, 0);
    add_node(&t, "R2", "ok", -2, 1, 90);
    seeker_track *k = add_track(&t, "R2", "R2", 1, 0.0f, 3.0f); /* straight ahead, behind a wall */
    k->has_pts = 1;
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        k->pts[j][0] = SKEL_T[j][0];
        k->pts[j][1] = SKEL_T[j][1];
        k->pts[j][2] = 0.9f;
    }
    draw_set_theme(DRAW_THEME_PLAN);
    anim_person_set_enabled(0); /* the settled plan look; arrivals are tested in test_anim_person */
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    CHECK(v.last.n_xray == 1 && v.last.n_skel == 1);
    int hidden = count_rgb(&c, P_HIDDEN, W / 2 - 60, W / 2 + 60, 0, H);
    printf("plan ghost skeleton: %d hidden-line px near the centre\n", hidden);
    CHECK(hidden > 60);
    CHECK(count_rgb(&c, C_CYAN, 0, W, 0, H) == 0 && count_rgb(&c, C_HUD, 0, W, 0, H) == 0);
    /* No pts: the hull and a dashed cross, still hidden colour, no skeleton counted. */
    k->has_pts = 0;
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    CHECK(v.last.n_skel == 0 && v.last.n_xray == 1 && count_hidden(&c, W / 2 - 30, W / 2 + 30, 0, H) > 10);
    /* Seen and close (1 m): red; far (6 m): no red at all. */
    snprintf(k->nodes, sizeof k->nodes, "W+R2");
    k->room_y_m = 1.0f;
    v.clock_ms = 1000;
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    int red = count_near_rgb(&c, P_ALERT, 70, 0, W, 0, H);
    printf("plan close person: %d red px\n", red);
    CHECK(v.last.n_direct == 1 && red > 50);
    memset(v.colours, 0, sizeof v.colours);
    k->room_y_m = 6.0f;
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    CHECK(v.last.n_direct == 1 && count_near_rgb(&c, P_ALERT, 70, 0, W, 0, H) == 0);
    /* Down is red even far away (the rescue case); crouching is not. */
    k->posture = SEEKER_POSTURE_CROUCH;
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    CHECK(v.last.n_posture[SEEKER_POSTURE_CROUCH] == 1 && count_near_rgb(&c, P_ALERT, 70, 0, W, 0, H) == 0);
    k->posture = SEEKER_POSTURE_DOWN;
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    CHECK(v.last.n_posture[SEEKER_POSTURE_DOWN] == 1 && count_near_rgb(&c, P_ALERT, 70, 0, W, 0, H) > 30);
    /* Down behind a wall: the plate goes red, the ghost stays in the hidden colour. */
    snprintf(k->nodes, sizeof k->nodes, "R2");
    k->room_y_m = 3.0f;
    k->has_pts = 1;
    memset(v.colours, 0, sizeof v.colours); /* another scene: not the seen person held */
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    printf("plan down behind wall: %d red px, %d hidden-line px\n", count_near_rgb(&c, P_ALERT, 70, 0, W, 0, H),
           count_hidden(&c, 0, W, 0, H));
    CHECK(v.last.n_xray == 1 && v.last.n_skel == 1 && count_near_rgb(&c, P_ALERT, 70, 0, W, 0, H) > 30 &&
          count_hidden(&c, 0, W, 0, H) > 60);
    k->posture = SEEKER_POSTURE_UNKNOWN;
    /* The title block grows a Delay cell for the latency readout. */
    CHECK(plan_title_block(NULL, 1.0f, "R1 live", 0, &t, "W", "Fixed", "140 ms") >
          plan_title_block(NULL, 1.0f, "R1 live", 0, &t, "W", "Fixed", NULL));

    /* Map fallback: the wearer triangle is at the wearer's pose, so a person 3 m straight ahead of
     * the wearer (not of the room origin) lands right above it on the screen's centre line. */
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    snprintf(v.cfg.node, sizeof v.cfg.node, "R1");
    v.cfg.pose.x_m = 2.0f;
    v.cfg.pose.y_m = -2.0f;
    v.cfg.pose_src = WV_POSE_STATIC;
    v.cfg.radar_fallback = 1;
    memset(&t, 0, sizeof t);
    add_node(&t, "R1", "ok", 2.0f, -2.0f, 0);
    add_track(&t, "R1", "R1", 1, 2.0f, 1.0f);
    hud_state hs;
    memset(&hs, 0, sizeof hs);
    hs.t = &t;
    hs.tracks_age_ms = 10;
    hs.background = "wearer";
    hs.view_node = "R1";
    hs.wearer = &v;
    hs.video_state = VIDEO_LOST;
    CHECK(hud_render(&c, &cfg, &hs) == 0);
    float u = plan_u(&c);
    int m24 = (int)lroundf(24 * u), h = H - 2 * m24 - plan_title_h(u) - (int)lroundf(16 * u);
    float ppm = ((float)h * 0.72f - 40 * u) / 6.0f, ay = (float)m24 + (float)(int)lroundf((float)h * 0.72f);
    int px0 = W / 2 - 1, py0 = (int)lroundf(ay - 3.0f * ppm);
    CHECK(count_rgb(&c, P_SEEN, px0 - 3, px0 + 3, py0 - 3, py0 + 3) > 4);
    draw_set_theme(DRAW_THEME_CLASSIC);
    anim_person_set_enabled(1);
    free(c.px);
}

/* ---- plan theme alert animations (display/anim_alert.h) ---- */

static int canvas_eq(const canvas *a, const canvas *b)
{
    return memcmp(a->px, b->px, (size_t)a->w * (size_t)a->h * 4) == 0;
}

static int canvas_diff(const canvas *a, const canvas *b)
{
    int n = 0;
    for (int i = 0; i < a->w * a->h; i++)
        n += a->px[i] != b->px[i];
    return n;
}

static void test_anim_alert(void)
{
    anim_alert_set_enabled(1);
    anim_alert_reset();
    /* No frame time: everything is the plain plan look. */
    anim_fx f = anim_alert_track(5, 1, 1);
    CHECK(!f.on && anim_alert_tint(&f, P_SEEN) == P_SEEN && anim_alert_alpha(&f, 255) == 255);

    /* Entering: the alert is on and red on the entering frame; the hatch starts at 20% and is
     * full by 200 ms. */
    anim_alert_frame(1000);
    f = anim_alert_track(5, 0, 0);
    CHECK(f.on && !f.alert && f.leave == 0 && f.ring < 0 && f.pulse == 1);
    anim_alert_frame(1033);
    f = anim_alert_track(5, 1, 0);
    CHECK(f.on && f.alert && NEAR(f.hatch, ANIM_HATCH_START, 1e-6) && f.ring < 0 && f.pulse == 1);
    CHECK(anim_alert_tint(&f, P_ALERT) == P_ALERT); /* no blend in: full alert colour at once */
    f = anim_alert_track(5, 1, 0);                  /* asking again this frame changes nothing */
    CHECK(NEAR(f.hatch, ANIM_HATCH_START, 1e-6));
    anim_alert_frame(1133);
    f = anim_alert_track(5, 1, 0);
    CHECK(f.hatch > ANIM_HATCH_START && f.hatch < 1);
    anim_alert_frame(1233);
    f = anim_alert_track(5, 1, 0);
    CHECK(f.hatch == 1);
    /* Leaving: fades from the alert colour back to normal over 200 ms. */
    anim_alert_frame(1500);
    f = anim_alert_track(5, 0, 0);
    CHECK(!f.alert && NEAR(f.leave, 1, 1e-6) && anim_alert_tint(&f, P_SEEN) == P_ALERT);
    anim_alert_frame(1600);
    f = anim_alert_track(5, 0, 0);
    uint32_t mid = anim_alert_tint(&f, P_SEEN);
    CHECK(NEAR(f.leave, 0.5, 1e-3) && mid != P_SEEN && mid != P_ALERT);
    anim_alert_frame(1700);
    f = anim_alert_track(5, 0, 0);
    CHECK(f.leave == 0 && anim_alert_tint(&f, P_SEEN) == P_SEEN);
    /* Back into the alert while the hatch was still fading: it stays full (no restart). */
    anim_alert_frame(1800);
    anim_alert_track(5, 1, 0);
    anim_alert_frame(1850);
    anim_alert_track(5, 0, 0);
    anim_alert_frame(1900);
    f = anim_alert_track(5, 1, 0);
    CHECK(f.alert && f.hatch == 1);

    /* Person down: ring ping for 500 ms from the entering frame, then the breathing pulse,
     * period 1.2 s, never below 70%. */
    anim_alert_frame(2000);
    f = anim_alert_track(6, 1, 1);
    CHECK(f.alert && f.down && f.ring == 0 && f.pulse == 1);
    anim_alert_frame(2250);
    f = anim_alert_track(6, 1, 1);
    CHECK(NEAR(f.ring, 0.5, 1e-3) && f.pulse == 1);
    anim_alert_frame(2500);
    f = anim_alert_track(6, 1, 1);
    CHECK(f.ring < 0 && NEAR(f.pulse, 1, 1e-6));
    anim_alert_frame(3100);
    f = anim_alert_track(6, 1, 1);
    CHECK(NEAR(f.pulse, ANIM_PULSE_MIN, 1e-4)); /* half a period in: the low point */
    anim_alert_frame(3700);
    f = anim_alert_track(6, 1, 1);
    CHECK(NEAR(f.pulse, 1, 1e-4)); /* a full period: back to 100% */
    float lo = 2, hi = -1;
    int alo = 256;
    for (int64_t t = 3701; t < 12000; t += 7) {
        anim_alert_frame(t);
        f = anim_alert_track(6, 1, 1);
        lo = fminf(lo, f.pulse), hi = fmaxf(hi, f.pulse);
        int a = anim_alert_alpha(&f, 255);
        if (a < alo)
            alo = a;
    }
    printf("anim down pulse: %.3f to %.3f, lowest frame alpha %d of 255\n", lo, hi, alo);
    CHECK(lo >= ANIM_PULSE_MIN - 1e-5f && lo < ANIM_PULSE_MIN + 0.01f && hi <= 1.0f && hi > 0.99f);
    CHECK(alo >= 178); /* 70% of 255 */
    /* Down without the alert (stale) is not down. */
    f = anim_alert_track(9, 0, 1);
    CHECK(!f.down && f.ring < 0 && f.pulse == 1);

    /* Removal and id reuse: unseen for ANIM_FORGET_MS, the id starts fresh. */
    anim_alert_frame(20000);
    anim_alert_track(7, 1, 0);
    anim_alert_frame(20000 + ANIM_FORGET_MS + 1);
    f = anim_alert_track(7, 1, 0);
    CHECK(f.alert && NEAR(f.hatch, ANIM_HATCH_START, 1e-6));
    /* No id: no state. A full table: the extras get the plain look, nothing breaks. */
    f = anim_alert_track(0, 1, 1);
    CHECK(!f.on && f.alert);
    for (int id = 100; id < 100 + ANIM_ALERT_SLOTS + 4; id++)
        f = anim_alert_track(id, 1, 0);
    CHECK(!f.on);
    /* Time going back (hud_shot jumping) starts over; off is the plain look. */
    anim_alert_frame(500);
    f = anim_alert_track(7, 1, 0);
    CHECK(f.on && NEAR(f.hatch, ANIM_HATCH_START, 1e-6));
    anim_alert_set_enabled(0);
    f = anim_alert_track(7, 1, 1);
    CHECK(!f.on && !anim_alert_enabled());
    anim_alert_set_enabled(1);

    /* Title block cells and warning plates on a canvas. */
    draw_set_theme(DRAW_THEME_PLAN);
    const int W = 640, H = 360;
    canvas a = { calloc((size_t)W * H, 4), W, H, W }, b = { calloc((size_t)W * H, 4), W, H, W };
    CHECK(a.px && b.px);
    if (!a.px || !b.px) {
        free(a.px);
        free(b.px);
        return;
    }
    anim_alert_reset();
    anim_alert_frame(1000);
    anim_alert_cell(&a, 0, 20, 20, 15, P_INK, "R1 live 1", 10, 10, 200, 40);
    /* Digits only ("1.2 s late" to "1.3 s late", the clock): swapped at once, crisp. */
    anim_alert_frame(1033);
    draw_clear(&a, 0);
    anim_alert_cell(&a, 0, 20, 20, 15, P_INK, "R1 live 2", 10, 10, 200, 40);
    anim_alert_frame(1066);
    draw_clear(&a, 0);
    draw_clear(&b, 0);
    anim_alert_cell(&a, 0, 20, 20, 15, P_INK, "R1 live 3", 10, 10, 200, 40);
    draw_ui_text_top(&b, 20, 20, 15, DRAW_FONT_MONO, P_INK, 255, "R1 live 3");
    CHECK(canvas_eq(&a, &b));
    /* New words: crossfade, the new value already drawn (60% or more) on the changing frame,
     * plain text once 150 ms have passed. */
    anim_alert_frame(1100);
    draw_clear(&a, 0);
    anim_alert_cell(&a, 0, 20, 20, 15, P_ALERT, "Video 1.2 s late", 10, 10, 200, 40);
    draw_clear(&b, 0);
    draw_ui_text_top(&b, 20, 20, 15, DRAW_FONT_MONO, P_ALERT, 153, "Video 1.2 s late");
    int ink_a = count_near_rgb(&a, P_ALERT, 120, 0, W, 0, H), ink_b = count_near_rgb(&b, P_ALERT, 120, 0, W, 0, H);
    CHECK(!canvas_eq(&a, &b) && ink_a > 20 && ink_a >= ink_b * 8 / 10);
    anim_alert_frame(1100 + ANIM_CELL_MS);
    draw_clear(&a, 0);
    draw_clear(&b, 0);
    anim_alert_cell(&a, 0, 20, 20, 15, P_ALERT, "Video 1.3 s late", 10, 10, 200, 40);
    draw_ui_text_top(&b, 20, 20, 15, DRAW_FONT_MONO, P_ALERT, 255, "Video 1.3 s late");
    CHECK(canvas_eq(&a, &b));
    /* The late hatch sweeps in and fades out. */
    float sw, k;
    anim_alert_frame(2000);
    CHECK(anim_alert_late(1, &sw, &k) && NEAR(sw, ANIM_HATCH_START, 1e-6) && k == 1);
    anim_alert_frame(2000 + ANIM_HATCH_MS);
    CHECK(anim_alert_late(1, &sw, &k) && sw == 1 && k == 1);
    anim_alert_frame(2300);
    CHECK(anim_alert_late(0, &sw, &k) && k == 1);
    anim_alert_frame(2400);
    CHECK(anim_alert_late(0, &sw, &k) && NEAR(k, 0.5, 1e-3));
    anim_alert_frame(2300 + ANIM_LEAVE_MS);
    CHECK(!anim_alert_late(0, &sw, &k));

    /* Warning plate: drawn (60%, 8 px up) on its first frame, the plain plate after 150 ms,
     * sliding out for 150 ms once it is gone. */
    plan_line wl[2] = { { "R1 position held", DRAW_FONT_SANS_SEMIBOLD, 19, P_INK, 255 },
                        { "phone lost its marker", DRAW_FONT_MONO, 15, P_INK, P_INK2_A } };
    draw_clear(&b, 0);
    plan_plate(&b, 0.5f, 12, 20, wl, 2, P_ALERT, 0);
    canvas z = { calloc((size_t)W * H, 4), W, H, W };
    anim_alert_frame(3000);
    draw_clear(&a, 0);
    anim_alert_warn(&a, 0.5f, 12, 20, wl, 2, P_ALERT);
    anim_alert_warn_flush(&a, 0.5f);
    CHECK(z.px && canvas_diff(&a, &z) > 500 && !canvas_eq(&a, &b));
    anim_alert_frame(3000 + ANIM_PLATE_MS);
    draw_clear(&a, 0);
    anim_alert_warn(&a, 0.5f, 12, 20, wl, 2, P_ALERT);
    anim_alert_warn_flush(&a, 0.5f);
    CHECK(canvas_eq(&a, &b));
    anim_alert_frame(3200);
    draw_clear(&a, 0);
    anim_alert_warn_flush(&a, 0.5f); /* gone this frame: still drawn, starting to leave */
    CHECK(canvas_diff(&a, &z) > 500);
    anim_alert_frame(3200 + ANIM_PLATE_MS);
    draw_clear(&a, 0);
    anim_alert_warn_flush(&a, 0.5f);
    CHECK(canvas_eq(&a, &z));
    free(z.px);

    /* Through hud_render: someone steps close. Red on the very frame the state turns true; long
     * after, the frame equals the plain plan theme exactly; animations off == plan at any time. */
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    v.cfg.visor = 1;
    v.cfg.radar_fallback = 0;
    v.box_ease_off = 1;
    static seeker_tracks t;
    memset(&t, 0, sizeof t);
    add_node(&t, "W", "ok", 0, 0, 0);
    seeker_track *p = add_track(&t, "W", "W", 1, 0.2f, 6.0f);
    hud_state hs;
    memset(&hs, 0, sizeof hs);
    hs.t = &t;
    hs.tracks_age_ms = 10;
    hs.background = "wearer";
    hs.view_node = "W";
    hs.wearer = &v;
    hs.video_state = VIDEO_LOST;
    anim_alert_reset();
    anim_person_set_enabled(0); /* this tests the alert module alone: no appear trace, */
    anim_map_set_enabled(0);    /* no map scan line */
    hs.now_ms = v.clock_ms = 50000;
    hud_render(&a, &cfg, &hs);
    CHECK(count_near_rgb(&a, P_ALERT, 70, 0, W, 0, H) == 0);
    p->room_y_m = 1.0f;
    memset(v.colours, 0, sizeof v.colours); /* the colour hold is fusion's business, not ours */
    hs.now_ms = v.clock_ms = 50033;
    hud_render(&a, &cfg, &hs);
    int red = count_near_rgb(&a, P_ALERT, 70, 0, W, 0, H);
    printf("anim entering frame: %d alert px\n", red);
    CHECK(v.last.n_direct == 1 && red > 50);
    /* The same moment with animations off differs only by decoration (the hatch sweep). */
    anim_alert_set_enabled(0);
    hud_render(&b, &cfg, &hs);
    anim_alert_set_enabled(1);
    CHECK(!canvas_eq(&a, &b) && count_near_rgb(&b, P_ALERT, 70, 0, W, 0, H) > 50);
    /* Steady (a second in): pixel-identical to the plain plan look (animations off, and no
     * frame time). The title block's clock can tick between renders, so try twice. */
    int same = 0;
    for (int tries = 0; tries < 2 && !same; tries++) {
        hs.now_ms = v.clock_ms = 51033;
        hud_render(&a, &cfg, &hs);
        anim_alert_set_enabled(0);
        hud_render(&b, &cfg, &hs);
        anim_alert_set_enabled(1);
        same = canvas_eq(&a, &b);
    }
    CHECK(same);
    /* Off == plan at any moment, including the transition frames. */
    p->room_y_m = 6.0f;
    same = 0;
    for (int tries = 0; tries < 2 && !same; tries++) {
        anim_alert_set_enabled(0);
        hs.now_ms = v.clock_ms = 52000;
        hud_render(&a, &cfg, &hs);
        hs.now_ms = 0;
        anim_alert_set_enabled(1);
        hud_render(&b, &cfg, &hs);
        same = canvas_eq(&a, &b);
    }
    CHECK(same);
    /* Leaving: 100 ms after the alert ended the colour is still on its way back. */
    anim_alert_reset();
    p->room_y_m = 1.0f;
    hs.now_ms = v.clock_ms = 60000;
    hud_render(&a, &cfg, &hs);
    memset(v.colours, 0, sizeof v.colours); /* drop the colour hysteresis: far at once */
    p->room_y_m = 6.0f;
    hs.now_ms = v.clock_ms = 60100;
    hud_render(&a, &cfg, &hs);
    hs.now_ms = v.clock_ms = 60200;
    hud_render(&a, &cfg, &hs);
    int fading = count_near_rgb(&a, P_SEEN, 40, 0, W, 0, H);
    hs.now_ms = v.clock_ms = 60400;
    hud_render(&a, &cfg, &hs);
    int settled = count_near_rgb(&a, P_SEEN, 40, 0, W, 0, H);
    printf("anim leaving: %d seen-colour px mid fade, %d settled\n", fading, settled);
    CHECK(settled > 30 && fading < settled);

    /* Cleanup: no animation state or frame time leaks into later tests. */
    anim_alert_reset();
    anim_alert_set_enabled(1);
    draw_set_theme(DRAW_THEME_CLASSIC);
    anim_person_set_enabled(1);
    anim_map_set_enabled(1);
    free(a.px);
    free(b.px);
}

/* ---- person animations (display/anim_person.h) ---- */

/* One frame in which only track `id` is drawn in view. */
static ap_fx ap_step(int64_t tm, int id, int direct)
{
    static wv_item it;
    static seeker_track k;
    int b[4] = { 10, 10, 40, 90 };
    anim_person_begin(tm, 1.0f);
    ap_fx f = anim_person_track(id, direct);
    k.id = id;
    it.direct = direct;
    anim_person_store(id, &it, &k, b);
    anim_person_end();
    return f;
}

/* Track `id` drawn every 33 ms from t0 up to and including t1. */
static void ap_run_to(int64_t t0, int64_t t1, int id, int direct)
{
    for (int64_t tm = t0; tm < t1; tm += 33)
        ap_step(tm, id, direct);
    ap_step(t1, id, direct);
}

/* The echo of a lost track this frame (its alpha), or -1 when none is drawn. */
static float ap_echo_a(void)
{
    static wv_item it;
    static seeker_track k;
    int b[4];
    ap_fx f;
    for (int i = 0; i < AP_SLOTS; i++)
        if (anim_person_echo(i, &it, &k, b, &f))
            return f.mark_a;
    return -1.0f;
}

static void ap_scene(seeker_tracks *t, int seen_on, int hid_on)
{
    memset(t, 0, sizeof *t);
    add_node(t, "W", "ok", 0, 0, 0);
    add_node(t, "R2", "ok", -2, 1, 90);
    if (seen_on) {
        seeker_track *k = add_track(t, "W", "W", 1, -0.9f, 3.0f);
        k->id = 1;
    }
    if (hid_on) {
        seeker_track *k = add_track(t, "R2", "R2", 1, 0.9f, 3.0f);
        k->id = 2;
        k->has_pts = 1;
        for (int j = 0; j < SEEKER_N_JOINTS; j++) {
            k->pts[j][0] = SKEL_T[j][0];
            k->pts[j][1] = SKEL_T[j][1];
            k->pts[j][2] = 0.9f;
        }
    }
}

/* Pixels that differ between a and b above the title block row (it prints the wall clock), inside
 * the rectangle (x0, y0)-(x1, y1). */
static int ap_diff(const canvas *a, const uint32_t *b, int x0, int y0, int x1, int y1)
{
    int ymax = a->h - (int)lroundf(24 * plan_u(a)) - plan_title_h(plan_u(a)) - 2, n = 0;
    for (int y = y0 < 0 ? 0 : y0; y < y1 && y < ymax; y++)
        for (int x = x0 < 0 ? 0 : x0; x < x1 && x < a->w; x++)
            n += a->px[y * a->stride + x] != b[y * a->w + x];
    return n;
}

static void test_anim_person(void)
{
    const int64_t T = 10000;
    ap_fx f;
    anim_person_set_enabled(1);

    /* Seen: corners trace (180 ms), then the leader (120 ms), then the plate (120 ms). */
    anim_person_reset();
    f = ap_step(T, 1, 1);
    CHECK(f.active && f.trace < 0.5f && f.leader == 0.0f && f.plate == 0.0f);
    f = ap_step(T + 90, 1, 1);
    CHECK(f.trace > 0.0f && f.trace < 1.0f);
    f = ap_step(T + 170, 1, 1);
    CHECK(f.leader == 0.0f);
    f = ap_step(T + 180, 1, 1);
    CHECK(f.trace == 1.0f);
    f = ap_step(T + 300, 1, 1);
    CHECK(f.leader == 1.0f && f.plate == 0.0f);
    f = ap_step(T + 360, 1, 1);
    CHECK(f.plate > 0.0f && f.plate < 1.0f);
    f = ap_step(T + 420, 1, 1);
    CHECK(f.plate == 1.0f && !f.active);
    CHECK(!anim_person_get(1).active && !anim_person_get(99).active);

    /* Behind a wall: 60% at once, full at 150 ms; hatch sweep 220 ms; dashes march. */
    anim_person_reset();
    f = ap_step(T, 2, 0);
    CHECK(f.active && f.mark_a >= 0.6f && f.mark_a < 1.0f);
    f = ap_step(T + 100, 2, 0);
    CHECK(f.hatch > 0.0f && f.hatch < 1.0f);
    f = ap_step(T + 150, 2, 0);
    CHECK(f.mark_a == 1.0f);
    f = ap_step(T + 220, 2, 0);
    CHECK(f.hatch == 1.0f);
    float m0 = f.march;
    f = ap_step(T + 420, 2, 0);
    CHECK(f.plate == 1.0f && f.leader == 1.0f && fabsf(f.march - m0 - 0.012f * 200) < 0.01f);

    /* Lost: an echo at 50% falling to 0 over 220 ms, no plate; then nothing. */
    const int64_t L = T + 500;
    anim_person_reset();
    ap_run_to(T, L, 5, 1);
    anim_person_begin(L + 100, 1.0f);
    anim_person_end();
    float ea = ap_echo_a();
    printf("anim person: echo alpha %.3f at 100 ms after the track went\n", ea);
    CHECK(ea > 0.2f && ea < 0.3f && !anim_person_get(5).active);
    anim_person_begin(L + 220, 1.0f);
    anim_person_end();
    CHECK(ap_echo_a() < 0);
    anim_person_begin(L + 600, 1.0f);
    anim_person_end();
    CHECK(ap_echo_a() < 0);

    /* Back during the echo: no replay at all. */
    anim_person_reset();
    ap_run_to(T, L, 5, 1);
    anim_person_begin(L + 100, 1.0f);
    anim_person_end();
    f = ap_step(L + 133, 5, 1);
    CHECK(f.plate == 1.0f && f.trace == 1.0f && ap_echo_a() < 0);
    /* Back after a second: the plate is up at once, the frame replays its trace. */
    anim_person_reset();
    ap_run_to(T, L, 5, 1);
    anim_person_begin(L + 300, 1.0f);
    anim_person_end();
    f = ap_step(L + 1000, 5, 1);
    CHECK(f.plate == 1.0f && f.trace < 1.0f);
    /* Back after AP_FORGET_MS (or fusion reusing the id): a new arrival. */
    anim_person_reset();
    ap_run_to(T, L, 5, 1);
    anim_person_begin(L + 300, 1.0f);
    anim_person_end();
    f = ap_step(L + AP_FORGET_MS + 500, 5, 1);
    CHECK(f.plate == 0.0f && f.trace < 0.5f);
    /* Seen to behind a wall: new marks intro, same plate. */
    anim_person_reset();
    ap_run_to(T, L, 7, 1);
    f = ap_step(L + 33, 7, 0);
    CHECK(f.hatch < 1.0f && f.mark_a < 1.0f && f.plate == 1.0f);

    /* Capacity: 40 people in one frame (more than fusion ever sends): AP_SLOTS animated, the rest
     * drawn plainly. The next frame, SEEKER_MAX_TRACKS new people: the 40 are not drawn any more, so
     * their echoes give way and every new arrival animates. */
    anim_person_reset();
    int on1 = 0, on2 = 0;
    anim_person_begin(T, 1.0f);
    for (int id = 100; id < 140; id++)
        on1 += anim_person_track(id, 1).active;
    anim_person_end();
    anim_person_begin(T + 33, 1.0f);
    anim_person_end();
    anim_person_begin(T + 66, 1.0f);
    for (int id = 200; id < 200 + SEEKER_MAX_TRACKS; id++)
        on2 += anim_person_track(id, id & 1).active;
    anim_person_end();
    printf("anim person: %d of 40 animated, then %d of %d (table %d)\n", on1, on2, SEEKER_MAX_TRACKS, AP_SLOTS);
    CHECK(on1 == AP_SLOTS && on2 == SEEKER_MAX_TRACKS);

    /* Off: plain drawing, no echoes. */
    anim_person_set_enabled(0);
    f = ap_step(T, 1, 1);
    CHECK(!f.active && f.plate == 1.0f && f.trace == 1.0f);
    anim_person_begin(T + 100, 1.0f);
    anim_person_end();
    CHECK(ap_echo_a() < 0);
    anim_person_set_enabled(1);

    /* Rendering. 331500 ms is a whole number of march periods, so at that clock a settled scene
     * draws exactly the plain plan marks. */
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    static wv_view v;
    static seeker_tracks t;
    const int W = 640, H = 360;
    const int64_t S = 331500;
    canvas c = { calloc((size_t)W * H, 4), W, H, W };
    uint32_t *ref = calloc((size_t)W * H, 4), *ref_none = calloc((size_t)W * H, 4);
    CHECK(c.px && ref && ref_none);
    if (!c.px || !ref || !ref_none) {
        free(c.px);
        free(ref);
        free(ref_none);
        return;
    }
    draw_set_theme(DRAW_THEME_PLAN);
    /* References with the animations off: both people, and nobody. */
    anim_person_set_enabled(0);
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    ap_scene(&t, 0, 0);
    v.clock_ms = S;
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    memcpy(ref_none, c.px, (size_t)W * H * 4);
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    ap_scene(&t, 1, 1);
    v.clock_ms = S - 1000;
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    memcpy(ref, c.px, (size_t)W * H * 4);
    wv_stats rs = v.last;
    CHECK(rs.n_direct == 1 && rs.n_xray == 1 && rs.n_drawn == 2);

    /* First frame with the animations on: same boxes in the same places, decorated differently. */
    anim_person_set_enabled(1);
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    v.clock_ms = S - 1000;
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    int same = v.last.n_drawn == rs.n_drawn;
    for (int i = 0; same && i < rs.n_drawn; i++)
        same = v.last.drawn[i].id == rs.drawn[i].id && v.last.drawn[i].x == rs.drawn[i].x &&
               v.last.drawn[i].y == rs.drawn[i].y && v.last.drawn[i].w == rs.drawn[i].w &&
               v.last.drawn[i].h == rs.drawn[i].h;
    int d0 = ap_diff(&c, ref, 0, 0, W, H);
    CHECK(same && d0 > 0);
    /* The seen frame's corners are drawn at full strength on the first frame, the middle of its top
     * edge only faintly (the corners trace towards it). */
    int sx = -1;
    for (int i = 0; i < rs.n_drawn; i++)
        if (rs.drawn[i].direct)
            sx = i;
    CHECK(sx >= 0);
    if (sx >= 0) {
        int bx = rs.drawn[sx].x, by = rs.drawn[sx].y, bw = rs.drawn[sx].w, hi = 0, mid = 0;
        for (int y = by - 2; y <= by + 2; y++) {
            for (int x = bx + 2; x < bx + 6; x++)
                hi += (int)((c.px[y * W + x] >> 8) & 255) > 100;
            for (int x = bx + bw / 2 - 2; x <= bx + bw / 2 + 2; x++)
                mid += (int)((c.px[y * W + x] >> 8) & 255) > 100;
        }
        CHECK(hi >= 4 && mid == 0);
    }
    /* Settled: pixel for pixel the plain plan drawing. */
    for (int64_t tm = S - 967; tm < S; tm += 33) {
        v.clock_ms = tm;
        wv_render(&c, &cfg, &v, &t, 0, NULL);
    }
    v.clock_ms = S;
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    int d1 = ap_diff(&c, ref, 0, 0, W, H);
    printf("anim person: %d px differ on the first frame, %d once settled\n", d0, d1);
    CHECK(d1 == 0);

    /* Lost (the seen person alone): an echo inside its last box at 50 ms, gone at 230 ms. */
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    ap_scene(&t, 1, 0);
    for (int64_t tm = S - 1000; tm <= S; tm += 20) {
        v.clock_ms = tm;
        wv_render(&c, &cfg, &v, &t, 0, NULL);
    }
    int bx = v.last.drawn[0].x, by = v.last.drawn[0].y, bw = v.last.drawn[0].w, bh = v.last.drawn[0].h;
    ap_scene(&t, 0, 0);
    v.clock_ms = S + 50;
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    int e50 = ap_diff(&c, ref_none, bx - 4, by - 4, bx + bw + 4, by + bh + 4);
    v.clock_ms = S + 230;
    wv_render(&c, &cfg, &v, &t, 0, NULL);
    int e230 = ap_diff(&c, ref_none, 0, 0, W, H);
    printf("anim person: echo %d px at 50 ms, %d px at 230 ms\n", e50, e230);
    CHECK(e50 > 100 && e230 == 0);
    draw_set_theme(DRAW_THEME_CLASSIC);
    free(c.px);
    free(ref);
    free(ref_none);
}

/* ---- the three plan animation modules on one clock and one switch (display/anim.h) ---- */

/* A seen person and one behind a wall arriving at `arrive`, the wearer fixed; hud_render at `now`. */
static void anim_all_frame(canvas *c, wv_view *v, seeker_tracks *t, int64_t now, int64_t arrive)
{
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    memset(t, 0, sizeof *t);
    add_node(t, "W", "ok", 0, 0, 0);
    add_node(t, "R2", "ok", -2, 1, 90);
    if (now >= arrive) {
        add_track(t, "W", "W", 1, -0.6f, 4.0f)->id = 1;
        seeker_track *k = add_track(t, "R2", "R2", 1, 0.6f, 4.0f);
        k->id = 2;
        k->has_pts = 1;
        for (int j = 0; j < SEEKER_N_JOINTS; j++) {
            k->pts[j][0] = SKEL_T[j][0];
            k->pts[j][1] = SKEL_T[j][1];
            k->pts[j][2] = 0.9f;
        }
    }
    hud_state hs;
    memset(&hs, 0, sizeof hs);
    hs.t = t;
    hs.tracks_age_ms = 10;
    hs.background = "wearer";
    hs.view_node = "W";
    hs.wearer = v;
    hs.video_state = VIDEO_LOST;
    hs.now_ms = now;
    hud_render(c, &cfg, &hs);
}

static void anim_all_view(wv_view *v)
{
    memset(v, 0, sizeof *v);
    wv_config_defaults(&v->cfg);
    v->cfg.visor = 1;
    v->cfg.radar_fallback = 0;
    v->cfg.n_walls = 1;
    v->cfg.walls[0][0] = 0.3f;
    v->cfg.walls[0][1] = 2.0f;
    v->cfg.walls[0][2] = 2.5f;
    v->cfg.walls[0][3] = 2.0f;
}

static void test_anim_switch(void)
{
    enum { W = 640, H = 360 };
    canvas a = { calloc((size_t)W * H, 4), W, H, W }, b = { calloc((size_t)W * H, 4), W, H, W };
    CHECK(a.px && b.px);
    if (!a.px || !b.px)
        return;
    static wv_view v;
    static seeker_tracks t;
    draw_set_theme(DRAW_THEME_PLAN);
    const int64_t T = 70000;
    /* One clock: hud_render hands now_ms to the wearer view (the person animations). */
    anim_set_enabled(1);
    anim_alert_reset();
    anim_person_reset();
    anim_map_reset();
    anim_all_view(&v);
    anim_all_frame(&a, &v, &t, T - 500, T);
    CHECK(v.clock_ms == T - 500);
    anim_all_frame(&a, &v, &t, T + 60, T); /* both people arriving, the map pinging */
    CHECK(v.clock_ms == T + 60 && v.last.n_direct == 1 && v.last.n_xray == 1);
    /* Off: every moment draws the same still plan, equal to a fresh view with no clock at all.
     * (The title block's clock can tick between renders: try twice.) */
    int same = 0, animated = 0;
    for (int tries = 0; tries < 2 && !same; tries++) {
        anim_set_enabled(0);
        anim_all_view(&v);
        anim_all_frame(&b, &v, &t, 0, 0); /* reference: no clock */
        int ok = 1;
        static const int64_t at[] = { T + 60, T + 200, T + 1500, T + 4000 };
        for (int i = 0; i < 4; i++) {
            anim_all_view(&v);
            anim_all_frame(&a, &v, &t, T - 500, T);
            anim_all_frame(&a, &v, &t, at[i], T);
            ok = ok && canvas_eq(&a, &b);
        }
        same = ok;
    }
    CHECK(same);
    /* On: the arrival frame is decorated (trace, sweep, ping), so it differs from the still. */
    anim_set_enabled(1);
    anim_alert_reset();
    anim_person_reset();
    anim_map_reset();
    anim_all_view(&v);
    anim_all_frame(&a, &v, &t, T - 500, T);
    anim_all_frame(&a, &v, &t, T + 60, T);
    animated = canvas_diff(&a, &b);
    printf("anim switch: arrival frame differs from the still plan in %d px\n", animated);
    CHECK(animated > 200);
    draw_set_theme(DRAW_THEME_CLASSIC);
    free(a.px);
    free(b.px);
}

/* ---- the handoff crossfade and the person animations on one person ---- */

/* One person 5 m out, x-ray (R2 sees them) or seen (the wearer's phone W sees them too). */
static void ah_scene(seeker_tracks *t, int seen)
{
    memset(t, 0, sizeof *t);
    add_node(t, "W", "ok", 0, 0, 0);
    add_node(t, "R2", "ok", -2, 1, 90);
    seeker_track *k = add_track(t, seen ? "W,R2" : "R2", seen ? "W" : "R2", 1, -0.6f, 5.0f);
    k->id = 1;
    k->has_pts = 1;
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        k->pts[j][0] = SKEL_T[j][0];
        k->pts[j][1] = SKEL_T[j][1];
        k->pts[j][2] = 0.9f;
    }
}

/* x-ray from s - 1500 ms, seen from s - 100 ms (the flip), every 20 ms up to s. */
static void ah_run(canvas *c, wv_view *v, int anim, int handoff, int64_t s)
{
    static seeker_config cfg;
    static seeker_tracks t;
    seeker_config_defaults(&cfg);
    anim_set_enabled(anim);
    anim_alert_reset();
    anim_map_reset();
    memset(v, 0, sizeof *v);
    wv_config_defaults(&v->cfg);
    v->cfg.handoff = handoff;
    for (int64_t tm = s - 1500; tm <= s; tm += 20) {
        ah_scene(&t, tm >= s - 100);
        v->clock_ms = tm;
        wv_render(c, &cfg, v, &t, 0, NULL);
    }
}

static int ah_diff(const canvas *a, const canvas *b, int x0, int y0, int x1, int y1)
{
    int n = 0;
    for (int y = y0 < 0 ? 0 : y0; y < y1 && y < a->h; y++)
        for (int x = x0 < 0 ? 0 : x0; x < x1 && x < a->w; x++)
            n += a->px[y * a->stride + x] != b->px[y * b->stride + x];
    return n;
}

static void test_anim_handoff(void)
{
    /* The fx: a look change mid-intro keeps its hatch sweep and marching, loses its own fades. */
    const int64_t T = 20000;
    anim_person_set_enabled(1);
    anim_person_begin(T, 1.0f);
    anim_person_track(5, 1);
    anim_person_end();
    anim_person_begin(T + 2000, 1.0f);
    anim_person_track(5, 1);
    anim_person_end();
    anim_person_begin(T + 2040, 1.0f);
    ap_fx f = anim_person_track(5, 0), g = ap_crossfading(f);
    anim_person_end();
    CHECK(f.active && f.mark_a < 1.0f && f.hatch < 1.0f && f.march != 0.0f);
    CHECK(g.active && g.mark_a == 1.0f && g.trace == 1.0f && g.hatch == f.hatch && g.march == f.march);
    ap_fx o = ap_old_look(0), os = ap_old_look(1);
    CHECK(o.march == f.march && o.mark_a == 1.0f && o.hatch == 1.0f && !os.active);
    anim_person_set_enabled(0);
    CHECK(!ap_old_look(0).active && !ap_crossfading(anim_person_track(5, 0)).active);

    /* Rendered: 331500 ms is a whole number of march periods (test_anim_person), and 100 ms after
     * the flip the crossfade is at 0.4 while the corner trace would still be open. */
    enum { W = 640, H = 360 };
    const int64_t S = 331500;
    canvas a = { calloc((size_t)W * H, 4), W, H, W }, b = { calloc((size_t)W * H, 4), W, H, W };
    CHECK(a.px && b.px);
    if (!a.px || !b.px) {
        free(a.px);
        free(b.px);
        return;
    }
    static wv_view v;
    draw_set_theme(DRAW_THEME_PLAN);
    ah_run(&b, &v, 0, 1, S);
    wv_stats off = v.last;
    ah_run(&a, &v, 1, 1, S);
    wv_stats on = v.last;
    CHECK(off.n_drawn == 1 && on.n_drawn == 1 && off.n_fading == 1 && on.n_fading == 1);
    CHECK(on.drawn[0].direct && on.drawn[0].x == off.drawn[0].x && on.drawn[0].w == off.drawn[0].w);
    int bx = on.drawn[0].x, by = on.drawn[0].y, bw = on.drawn[0].w, bh = on.drawn[0].h;
    int d_xfade = ah_diff(&a, &b, bx - 6, by - 6, bx + bw + 6, by + bh + 6);
    /* --handoff off: no crossfade, so the person animations' own look change plays. */
    ah_run(&b, &v, 0, 0, S);
    ah_run(&a, &v, 1, 0, S);
    CHECK(v.last.n_fading == 0 && v.last.n_drawn == 1);
    int d_plain = ah_diff(&a, &b, bx - 6, by - 6, bx + bw + 6, by + bh + 6);
    printf("anim handoff: mid-crossfade box differs anim on vs off in %d px (handoff on), %d px (handoff off)\n",
           d_xfade, d_plain);
    CHECK(d_xfade == 0 && d_plain > 50);
    anim_set_enabled(1);
    draw_set_theme(DRAW_THEME_CLASSIC);
    free(a.px);
    free(b.px);
}

/* ---- plan map animations (display/anim_map.h) ---- */

static wv_config g_am_wc;
static wv_pose g_am_pose;

/* The full-screen plan of `marks` at time `t` (0: no clock), wall straight ahead, no phones. */
static void am_draw(canvas *c, int64_t t, const plan_mark *marks, int n, int walls)
{
    anim_map_frame(t);
    draw_clear(c, P_GROUND);
    g_am_wc.n_walls = walls;
    plan_map(c, plan_u(c), 0, 0, c->w, c->h, 40.0f, 0.8f, 0, &g_am_pose, "R1", 66.0f, &g_am_wc, NULL, marks, n);
}

/* Pixels that differ between a and b inside the box. */
static int am_diff(const canvas *a, const canvas *b, int x0, int x1, int y0, int y1)
{
    int n = 0;
    for (int y = y0 < 0 ? 0 : y0; y < y1 && y < a->h; y++)
        for (int x = x0 < 0 ? 0 : x0; x < x1 && x < a->w; x++)
            n += a->px[y * a->stride + x] != b->px[y * b->stride + x];
    return n;
}

static plan_mark am_mark(int id, int kind, float x, float y, int down)
{
    plan_mark m;
    memset(&m, 0, sizeof m);
    m.id = id;
    m.kind = kind;
    m.x = x;
    m.y = y;
    m.down = down;
    m.close = down;
    return m;
}

static void test_anim_map(void)
{
    const int W = 640, H = 360;
    canvas a = { calloc((size_t)W * H, 4), W, H, W }, b = { calloc((size_t)W * H, 4), W, H, W };
    CHECK(a.px && b.px);
    if (!a.px || !b.px)
        return;
    memset(&g_am_wc, 0, sizeof g_am_wc);
    wv_config_defaults(&g_am_wc);
    g_am_wc.walls[0][0] = -1.0f; /* 2 m wall 2 m ahead */
    g_am_wc.walls[0][1] = 2.0f;
    g_am_wc.walls[0][2] = 1.0f;
    g_am_wc.walls[0][3] = 2.0f;
    memset(&g_am_pose, 0, sizeof g_am_pose);
    const int64_t T = 50000;
    /* Screen of a room point: wearer at (W/2 - 1, 0.8 H), 40 px per m, heading up (plan.c). */
#define AM_SX(x) (W / 2 + (int)lroundf((x) * 40.0f))
#define AM_SY(y) ((int)lroundf(H * 0.8f) - (int)lroundf((y) * 40.0f))
    CHECK(anim_map_enabled());

    /* Disabled, and enabled with no clock, both draw exactly the unanimated plan, mid-animation. */
    plan_mark mk[3] = { am_mark(1, PLAN_SEEN, -0.5f, 3.0f, 0), am_mark(2, PLAN_HIDDEN, 0.6f, 3.2f, 0),
                        am_mark(3, PLAN_SEEN, -4.0f, 1.0f, 1) };
    anim_map_reset();
    am_draw(&b, 0, mk, 3, 1); /* reference: no clock */
    anim_map_set_enabled(0);
    for (int64_t dt = 0; dt <= 3000; dt += 150) {
        am_draw(&a, T + dt, mk, 3, 1);
        CHECK(am_diff(&a, &b, 0, W, 0, H) == 0);
    }
    anim_map_set_enabled(1);
    anim_map_reset();
    am_draw(&a, 0, mk, 3, 1);
    CHECK(am_diff(&a, &b, 0, W, 0, H) == 0);

    /* First frame: every marker is there at full strength, the wall is not drawn yet. */
    anim_map_reset();
    am_draw(&a, T, mk, 3, 1);
    int sx = AM_SX(-0.5f), sy = AM_SY(3.0f);
    CHECK(a.px[sy * W + sx] == P_SEEN && b.px[sy * W + sx] == P_SEEN); /* seen squares: the palette's cyan */
    int hid_a = count_near_rgb(&a, P_HIDDEN, 90, AM_SX(0.6f) - 4, AM_SX(0.6f) + 5, AM_SY(3.2f) - 4, AM_SY(3.2f) + 5);
    int hid_b = count_near_rgb(&b, P_HIDDEN, 90, AM_SX(0.6f) - 4, AM_SX(0.6f) + 5, AM_SY(3.2f) - 4, AM_SY(3.2f) + 5);
    printf("anim map: first frame hidden square %d px (unanimated %d)\n", hid_a, hid_b);
    CHECK(hid_a > 6 && hid_a * 2 > hid_b);
    int wall_now = am_diff(&a, &b, AM_SX(-0.8f), AM_SX(0.8f), AM_SY(2.0f) - 2, AM_SY(2.0f) + 3);
    printf("anim map: first frame, %d wall px still to draw\n", wall_now);
    CHECK(wall_now > 40);
    /* Draw-in done by 300 ms: the wall equals the settled plan. */
    am_draw(&a, T + 320, mk, 3, 1);
    CHECK(am_diff(&a, &b, AM_SX(-0.8f), AM_SX(0.8f), AM_SY(2.0f) - 2, AM_SY(2.0f) + 3) == 0);

    /* Ping: one person off to the side (outside the cone's scan), no walls. A ring around them
     * while 0..450 ms old, nothing after; the square itself is drawn every frame. */
    plan_mark p1 = am_mark(7, PLAN_SEEN, -4.0f, 1.0f, 0);
    int px = AM_SX(-4.0f), py = AM_SY(1.0f), R = 30;
    anim_map_reset();
    am_draw(&b, 0, &p1, 1, 0);
    am_draw(&a, T - 1000, NULL, 0, 0); /* the map already up, empty */
    am_draw(&a, T, &p1, 1, 0);
    CHECK(a.px[py * W + px] == P_SEEN);
    am_draw(&a, T + 200, &p1, 1, 0);
    int ring = am_diff(&a, &b, px - R, px + R, py - R, py + R);
    am_draw(&a, T + 460, &p1, 1, 0);
    int after = am_diff(&a, &b, px - R, px + R, py - R, py + R);
    printf("anim map: ping %d px at 200 ms, %d at 460 ms\n", ring, after);
    CHECK(ring > 20 && after == 0);
    /* Still on the map a second later: no new ping. */
    am_draw(&a, T + 1200, &p1, 1, 0);
    am_draw(&a, T + 1300, &p1, 1, 0);
    CHECK(am_diff(&a, &b, px - R, px + R, py - R, py + R) == 0);
    /* Gone over a second, then the id comes back (or is reused): it pings again. */
    for (int64_t t = T + 1400; t <= T + 2600; t += 100)
        am_draw(&a, t, NULL, 0, 0);
    am_draw(&a, T + 2700, &p1, 1, 0);
    am_draw(&a, T + 2900, &p1, 1, 0);
    CHECK(am_diff(&a, &b, px - R, px + R, py - R, py + R) > 20);
    /* A one-frame dropout does not. */
    am_draw(&a, T + 3500, &p1, 1, 0);
    am_draw(&a, T + 3600, NULL, 0, 0);
    am_draw(&a, T + 3700, &p1, 1, 0);
    CHECK(am_diff(&a, &b, px - R, px + R, py - R, py + R) == 0);
    /* Untracked (id 0) and the table full of other ids: no crash, a new id still pings. */
    static plan_mark many[100];
    for (int i = 0; i < 100; i++)
        many[i] = am_mark(1000 + i, PLAN_SEEN, 3.0f, 1.0f, 0);
    am_draw(&a, T + 3800, many, 100, 0);
    plan_mark p0 = am_mark(0, PLAN_SEEN, -4.0f, 1.0f, 0), p9 = am_mark(9, PLAN_SEEN, -4.0f, 1.0f, 0);
    am_draw(&a, T + 3900, &p0, 1, 0);
    am_draw(&a, T + 4000, &p0, 1, 0);
    CHECK(am_diff(&a, &b, px - R, px + R, py - R, py + R) == 0);
    am_draw(&a, T + 4100, &p9, 1, 0);
    am_draw(&a, T + 4300, &p9, 1, 0);
    CHECK(am_diff(&a, &b, px - R, px + R, py - R, py + R) > 20);

    /* Down: a ring every 1.4 s from going down, quiet in between (frames every 100 ms). */
    plan_mark d = am_mark(7, PLAN_SEEN, -4.0f, 1.0f, 1);
    anim_map_reset();
    am_draw(&b, 0, &d, 1, 0);
    p1.id = 7;
    for (int64_t t = T; t < T + 1000; t += 100)
        am_draw(&a, t, &p1, 1, 0); /* standing first */
    int pulses = 0, quiets = 0;
    for (int64_t t = T + 1000; t < T + 1000 + 4 * 1400; t += 100) {
        am_draw(&a, t, &d, 1, 0);
        int ph = (int)((t - T - 1000) % 1400), n = am_diff(&a, &b, px - 40, px + 40, py - 40, py + 40);
        if (ph == 300) {
            pulses += n > 20;
            if (t == T + 1300)
                printf("anim map: down ring %d px at +300 ms\n", n);
        } else if (ph >= 1000)
            quiets += n == 0;
    }
    CHECK(pulses == 4 && quiets == 4 * 4);
    /* Up again: no more rings. */
    for (int64_t t = T + 6600; t <= T + 8000; t += 100)
        am_draw(&a, t, &p1, 1, 0);
    memcpy(b.px, a.px, (size_t)W * H * 4);
    am_draw(&a, T + 8100, &p1, 1, 0);
    am_draw(&a, T + 8500, &p1, 1, 0);
    CHECK(am_diff(&a, &b, px - 40, px + 40, py - 40, py + 40) == 0);

    /* Hidden dashes march: the square changes over time, stays the same at the same time. */
    plan_mark h = am_mark(8, PLAN_HIDDEN, -4.0f, 1.0f, 0);
    anim_map_reset();
    am_draw(&a, T, &h, 1, 0);
    am_draw(&a, T + 1000, &h, 1, 0);
    am_draw(&b, T + 1200, &h, 1, 0);
    CHECK(am_diff(&a, &b, px - 8, px + 8, py - 8, py + 8) > 0);
    am_draw(&a, T + 1200, &h, 1, 0);
    CHECK(am_diff(&a, &b, 0, W, 0, H) == 0); /* same time, same pixels */

    /* Scan: a faint line inside the cone at some moments, none in the still part of the period. */
    anim_map_reset();
    am_draw(&b, 0, NULL, 0, 0);
    am_draw(&a, T, NULL, 0, 0);
    int scan_on = 0;
    for (int64_t t = T + 400; t < T + 2000; t += 100) {
        am_draw(&a, t, NULL, 0, 0);
        scan_on += am_diff(&a, &b, 0, W, 0, H) > 30;
    }
    am_draw(&a, T + 300 + 3000 - 500, NULL, 0, 0);
    CHECK(scan_on >= 12 && am_diff(&a, &b, 0, W, 0, H) == 0);

    /* Through hud_render: the full-screen plan's first frame has the person square already. */
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    snprintf(v.cfg.node, sizeof v.cfg.node, "R1");
    v.cfg.pose_src = WV_POSE_STATIC;
    v.cfg.radar_fallback = 1;
    static seeker_tracks t;
    memset(&t, 0, sizeof t);
    add_node(&t, "R1", "ok", 0, 0, 0);
    seeker_track *k = add_track(&t, "R1", "R1", 1, -4.0f, 1.0f); /* off to the side: no scan there */
    k->id = 5;
    k->posture = SEEKER_POSTURE_DOWN;
    hud_state hs;
    memset(&hs, 0, sizeof hs);
    hs.t = &t;
    hs.tracks_age_ms = 10;
    hs.background = "wearer";
    hs.view_node = "R1";
    hs.wearer = &v;
    hs.video_state = VIDEO_LOST;
    draw_set_theme(DRAW_THEME_PLAN);
    anim_map_reset();
    hs.now_ms = T; /* hud_render hands the frame time to the map animations */
    hud_render(&a, &cfg, &hs);
    float u = plan_u(&a);
    int m24 = (int)lroundf(24 * u), hh = H - 2 * m24 - plan_title_h(u) - (int)lroundf(16 * u);
    float ppm = ((float)hh * 0.72f - 40 * u) / 6.0f, fay = (float)m24 + (float)(int)lroundf((float)hh * 0.72f);
    int qx = W / 2 + (int)lroundf(-4.0f * ppm), qy = (int)lroundf(fay - 1.0f * ppm);
    CHECK(count_rgb(&a, P_RED, qx - 3, qx + 3, qy - 3, qy + 3) > 4); /* down: red, first frame */
    for (int64_t t = T + 100; t <= T + 1200; t += 100) {
        if (t == T + 1200)
            memcpy(b.px, a.px, (size_t)W * H * 4); /* T + 1100: between rings */
        hs.now_ms = t;
        hud_render(&a, &cfg, &hs);
    }
    CHECK(am_diff(&a, &b, qx - 30, qx + 30, qy - 30, qy + 30) == 0);
    for (int64_t t = T + 1300; t <= T + 1700; t += 100) {
        hs.now_ms = t;
        hud_render(&a, &cfg, &hs);
    }
    int hr = am_diff(&a, &b, qx - 30, qx + 30, qy - 30, qy + 30); /* T + 1700: second ring */
    printf("anim map: hud_render down ring %d px\n", hr);
    CHECK(hr > 20);
    draw_set_theme(DRAW_THEME_CLASSIC);
#undef AM_SX
#undef AM_SY
    anim_map_reset();
    free(a.px);
    free(b.px);
}

/* ---- threat colour hysteresis (common/seeker_colour.h) ---- */

/* The colour before this fix: red under near_m or while an EMA of tick-to-tick range change is
 * over closing_mps (fusion's old closing_mps, 30 Hz ticks over ~15 Hz range steps). */
typedef struct {
    int has;
    float r, closing;
    int64_t t;
} old_colour;

static int old_level(old_colour *o, float r, int64_t now, const seeker_config *cfg)
{
    if (o->has && now > o->t) {
        float dt = (float)(now - o->t) / 1000.0f;
        if (dt > 0.02f) {
            o->closing = 0.7f * o->closing + 0.3f * (o->r - r) / dt;
            o->r = r;
            o->t = now;
        }
    } else {
        o->has = 1;
        o->r = r;
        o->t = now;
    }
    return r < cfg->near_m || o->closing > cfg->closing_mps ? SEEKER_RED
           : r < cfg->mid_m                                 ? SEEKER_AMBER
                                                             : SEEKER_GREEN;
}

/* Feeds range(t) at 30 Hz ticks, the value changing at 15 Hz like a phone's, for ms. Counts
 * level changes and red ticks, and the shortest time a level was held. */
typedef struct {
    int changes, red, shortest_ms, last;
    int64_t since;
} colour_run;

static void run_colour(seeker_colour *s, colour_run *cr, float (*range)(int64_t, float), int64_t t0,
                       int ms, const seeker_config *cfg, old_colour *old, colour_run *ocr)
{
    float r = 0;
    for (int64_t now = t0; now < t0 + ms; now += 33) {
        if ((now - t0) % 66 < 33)
            r = range(now - t0, (float)gauss());
        int lv = seeker_colour_update(s, r, now, cfg);
        colour_run *c2[2] = { cr, ocr };
        int lvs[2] = { lv, old ? old_level(old, r, now, cfg) : lv };
        for (int q = 0; q < (old ? 2 : 1); q++) {
            colour_run *c = c2[q];
            if (c->since && lvs[q] != c->last) {
                c->changes++;
                if (now - c->since < c->shortest_ms)
                    c->shortest_ms = (int)(now - c->since);
            }
            if (!c->since || lvs[q] != c->last) {
                c->last = lvs[q];
                c->since = now;
            }
            c->red += lvs[q] == SEEKER_RED;
        }
    }
}

static float r_still_3m(int64_t t, float g) { (void)t; return 3.0f + 0.15f * g; }
static float r_at_2m(int64_t t, float g) { (void)t; return 2.05f + 0.1f * g; }
static float r_walk_in(int64_t t, float g) /* 6 m, walking in at 1 m/s from t = 1 s, stops at 3 m */
{
    float r = t < 1000 ? 6.0f : t < 4000 ? 6.0f - (float)(t - 1000) / 1000.0f : 3.0f;
    return r + 0.1f * g;
}
static float r_slow_in(int64_t t, float g) { return 4.5f - 0.4f * (float)t / 1000.0f + 0.1f * g; }

static void test_colour_hysteresis(void)
{
    static seeker_config cfg;
    seeker_config_defaults(&cfg); /* near 2.0 m, mid 5.0 m, closing 0.5 m/s */
    seeker_colour s;
    colour_run cr, ocr;
    old_colour old;

    /* Standing still 3 m away with a noisy mono range (sigma 0.15 m): steady amber. */
    memset(&s, 0, sizeof s);
    memset(&cr, 0, sizeof cr);
    memset(&ocr, 0, sizeof ocr);
    memset(&old, 0, sizeof old);
    cr.shortest_ms = ocr.shortest_ms = 1 << 30;
    run_colour(&s, &cr, r_still_3m, 1000, 20000, &cfg, &old, &ocr);
    printf("colour, still at 3 m, noisy range, 20 s: %d changes, %d red ticks (old rule: %d "
           "changes, %d red ticks)\n", cr.changes, cr.red, ocr.changes, ocr.red);
    CHECK(cr.changes == 0 && cr.red == 0 && s.level == SEEKER_AMBER);
    CHECK(ocr.changes > 10); /* the flicker Felix saw */

    /* Right at the 2 m line (sigma 0.1 m): few changes, none shorter than the hold. */
    memset(&s, 0, sizeof s);
    memset(&cr, 0, sizeof cr);
    cr.shortest_ms = 1 << 30;
    run_colour(&s, &cr, r_at_2m, 1000, 10000, &cfg, NULL, NULL);
    printf("colour, at the 2 m line, 10 s: %d changes, shortest %d ms\n", cr.changes,
           cr.changes ? cr.shortest_ms : 0);
    CHECK(cr.changes <= 3 && (cr.changes == 0 || cr.shortest_ms >= SEEKER_COLOUR_HOLD_MS));

    /* A real fast approach (1 m/s from 6 m) still turns red, and amber again once they stop. */
    memset(&s, 0, sizeof s);
    int64_t red_at = 0, amber_after = 0;
    float r = 0;
    for (int64_t t = 0; t < 7000; t += 33) {
        if (t % 66 < 33)
            r = r_walk_in(t, (float)gauss());
        int lv = seeker_colour_update(&s, r, 1000 + t, &cfg);
        if (lv == SEEKER_RED && !red_at)
            red_at = t;
        if (t > 4000 && lv == SEEKER_AMBER && !amber_after)
            amber_after = t;
    }
    printf("colour, walking in at 1 m/s from t=1 s: red at %lld ms, amber again %lld ms after "
           "stopping\n", (long long)red_at, (long long)(amber_after - 4000));
    CHECK(red_at > 1000 && red_at < 2200);
    CHECK(amber_after > 4000 && amber_after < 5600);

    /* A slow approach (0.4 m/s) stays amber. */
    memset(&s, 0, sizeof s);
    memset(&cr, 0, sizeof cr);
    cr.shortest_ms = 1 << 30;
    run_colour(&s, &cr, r_slow_in, 1000, 3500, &cfg, NULL, NULL);
    CHECK(cr.red == 0);

    /* Distance hysteresis: red under 2.0 m, kept to 2.3 m; and every colour holds 400 ms. */
    memset(&s, 0, sizeof s);
    int64_t t = 1000;
    CHECK(seeker_colour_update(&s, 1.9f, t, &cfg) == SEEKER_RED);
    CHECK(seeker_colour_update(&s, 2.2f, t += 500, &cfg) == SEEKER_RED);
    CHECK(seeker_colour_update(&s, 2.4f, t += 33, &cfg) == SEEKER_AMBER);
    CHECK(seeker_colour_update(&s, 1.9f, t += 100, &cfg) == SEEKER_AMBER); /* held */
    CHECK(seeker_colour_update(&s, 1.9f, t += 300, &cfg) == SEEKER_RED);
    CHECK(seeker_colour_update(&s, 5.2f, t += 900, &cfg) == SEEKER_AMBER); /* 5.0 + 0.3 */
    CHECK(seeker_colour_update(&s, 5.4f, t += 33, &cfg) == SEEKER_AMBER); /* held */
    CHECK(seeker_colour_update(&s, 5.4f, t += 400, &cfg) == SEEKER_GREEN);
    CHECK(seeker_colour_update(&s, 4.9f, t += 33, &cfg) == SEEKER_GREEN); /* held */
    CHECK(seeker_colour_update(&s, 4.9f, t += 400, &cfg) == SEEKER_AMBER);
    CHECK(!strcmp(seeker_colour_name(SEEKER_RED), "red") && !strcmp(seeker_colour_name(SEEKER_AMBER), "amber"));
    /* A gap longer than SEEKER_COLOUR_RESET_MS starts over (a returning track, no stale hold). */
    CHECK(seeker_colour_update(&s, 6.0f, t += 2000, &cfg) == SEEKER_GREEN);

    /* Through hud_fusion: a single-phone person standing 3 m from the origin with a noisy size
     * range never turns the tracks colour (radar fallback, plain wearer view) red. */
    static seeker_config lc;
    CHECK(seeker_config_load(&lc, src_file("config/seeker.live.json")) == 0);
    static seeker_fusion f;
    seeker_fusion_init(&f, &lc);
    const seeker_pose r2 = { -3.0f, 0.0f, 90 };
    seeker_fusion_set_pose(&f, "R2", &r2, "cli", 1);
    static seeker_tracks tr;
    int red = 0, changes = 0;
    char last[8] = "";
    float max_closing = 0;
    for (int i = 0; i < 300; i++) {
        /* 3 m in front of R1 (the live config's wearer_node, at 0,-2). */
        seeker_remote_contacts m = phone_msg("R2", &r2, 0.0f, 1.0f, 1);
        m.c[0].range_m += 0.25f * (float)gauss();
        seeker_fusion_ingest(&f, &m, 1000 + 66 * i);
        for (int k = 0; k < 2; k++) {
            seeker_fusion_tick(&f, 1000 + 66 * i + 5 + 33 * k, &tr);
            if (tr.n != 1)
                continue;
            red += !strcmp(tr.tracks[0].colour, "red");
            if (i > 15) {
                changes += last[0] && strcmp(last, tr.tracks[0].colour) != 0;
                max_closing = fmaxf(max_closing, fabsf(tr.tracks[0].closing_mps));
            }
            snprintf(last, sizeof last, "%s", tr.tracks[0].colour);
        }
    }
    printf("fusion colour, still at 3 m, 20 s: %d red ticks, %d changes, |closing| up to %.2f m/s\n",
           red, changes, max_closing);
    CHECK(red == 0 && changes == 0 && !strcmp(last, "amber") && max_closing < 0.5f);

    /* Wearer view (range from the wearer): a seen person at 3 m with a jittery position. */
    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    v.cfg.inset = 0;
    const int W = 320, H = 180;
    canvas c = { calloc((size_t)W * H, 4), W, H, W };
    CHECK(c.px != NULL);
    if (!c.px)
        return;
    int red_frames = 0, amber_frames = 0;
    for (int i = 0; i < 200; i++) {
        static seeker_tracks wt;
        memset(&wt, 0, sizeof wt);
        add_node(&wt, "W", "ok", 0, 0, 0);
        seeker_track *k = add_track(&wt, "W", "W", 1, 0.2f, 3.0f + 0.15f * (float)gauss());
        k->closing_mps = 2.0f * (float)gauss(); /* fusion's value is not used for the colour */
        v.clock_ms = 1000 + 33 * i;
        wv_render(&c, &cfg, &v, &wt, 0, NULL);
        red_frames += count_colour(&c, C_RED, 0, W, 0, H - 40) > 0;
        amber_frames += count_colour(&c, C_AMBER, 0, W, 0, H - 40) > 0;
    }
    printf("wearer view, seen at 3 m, 200 frames: %d red, %d amber\n", red_frames, amber_frames);
    CHECK(red_frames == 0 && amber_frames == 200);
    free(c.px);
}

/* ---- posture (common/seeker_posture.h) ---- */

/* Front-view squat and a person lying on their side across the view (person-local units). */
static const float SKEL_CROUCH[SEEKER_N_JOINTS][2] = {
    { 0.00f, -0.36f }, { -0.02f, -0.38f }, { 0.02f, -0.38f }, { -0.05f, -0.37f },
    { 0.05f, -0.37f }, { -0.12f, -0.22f }, { 0.12f, -0.22f }, { -0.16f, -0.08f },
    { 0.16f, -0.08f }, { -0.12f, 0.02f },  { 0.12f, 0.02f },  { -0.09f, 0.00f },
    { 0.09f, 0.00f },  { -0.18f, 0.03f },  { 0.18f, 0.03f },  { -0.12f, 0.24f },
    { 0.12f, 0.24f },
};
static const float SKEL_DOWN[SEEKER_N_JOINTS][2] = {
    { -0.48f, -0.02f }, { -0.47f, -0.04f }, { -0.47f, -0.03f }, { -0.44f, -0.03f },
    { -0.44f, -0.01f }, { -0.30f, -0.02f }, { -0.30f, 0.02f },  { -0.18f, 0.04f },
    { -0.18f, 0.06f },  { -0.06f, 0.05f },  { -0.05f, 0.07f },  { 0.00f, -0.02f },
    { 0.00f, 0.02f },   { 0.25f, -0.01f },  { 0.25f, 0.02f },   { 0.47f, 0.00f },
    { 0.48f, 0.02f },
};
/* Side view on a staircase: the left foot on a step, the right foot two steps (0.21 person
 * heights) higher and forward, its knee bent about 120 deg. The feet are off the floor, which
 * the classifier never looks at. */
static const float SKEL_STAIRS[SEEKER_N_JOINTS][2] = {
    { 0.06f, -0.43f }, { 0.05f, -0.45f }, { 0.07f, -0.45f }, { 0.02f, -0.44f },
    { 0.03f, -0.44f }, { -0.01f, -0.29f }, { 0.01f, -0.29f }, { 0.02f, -0.12f },
    { 0.04f, -0.12f }, { 0.04f, 0.03f },  { 0.06f, 0.03f },  { -0.01f, 0.00f },
    { 0.01f, 0.00f },  { -0.02f, 0.25f }, { 0.17f, 0.12f },  { -0.03f, 0.48f },
    { 0.15f, 0.27f },
};
/* Sitting on a chair facing the phone: thighs point at the camera (short in the picture), shins
 * vertical, so the 2D knee angle looks almost straight; and the same seen from the side. */
static const float SKEL_SIT_FRONT[SEEKER_N_JOINTS][2] = {
    { 0.00f, -0.43f }, { -0.02f, -0.45f }, { 0.02f, -0.45f }, { -0.05f, -0.44f },
    { 0.05f, -0.44f }, { -0.12f, -0.29f }, { 0.12f, -0.29f }, { -0.15f, -0.12f },
    { 0.15f, -0.12f }, { -0.12f, 0.04f },  { 0.12f, 0.04f },  { -0.08f, 0.00f },
    { 0.08f, 0.00f },  { -0.10f, 0.07f },  { 0.10f, 0.07f },  { -0.10f, 0.33f },
    { 0.10f, 0.33f },
};
static const float SKEL_SIT_SIDE[SEEKER_N_JOINTS][2] = {
    { 0.06f, -0.43f }, { 0.05f, -0.45f }, { 0.07f, -0.45f }, { 0.02f, -0.44f },
    { 0.03f, -0.44f }, { -0.01f, -0.29f }, { 0.01f, -0.29f }, { 0.04f, -0.12f },
    { 0.05f, -0.12f }, { 0.12f, -0.04f },  { 0.13f, -0.04f }, { -0.01f, 0.00f },
    { 0.01f, 0.00f },  { 0.23f, 0.01f },   { 0.25f, 0.02f },  { 0.22f, 0.26f },
    { 0.24f, 0.27f },
};
/* Kneeling on the left knee, side view: right thigh forward, left thigh down, left shin on the
 * floor behind. */
static const float SKEL_KNEEL[SEEKER_N_JOINTS][2] = {
    { 0.06f, -0.43f }, { 0.05f, -0.45f }, { 0.07f, -0.45f }, { 0.02f, -0.44f },
    { 0.03f, -0.44f }, { -0.01f, -0.29f }, { 0.01f, -0.29f }, { 0.03f, -0.12f },
    { 0.04f, -0.12f }, { 0.08f, 0.02f },   { 0.09f, 0.02f },  { -0.01f, 0.00f },
    { 0.01f, 0.00f },  { -0.02f, 0.25f },  { 0.24f, 0.02f },  { -0.26f, 0.27f },
    { 0.22f, 0.27f },
};
/* Bending over on straight legs (picking something up), side view: torso 70 deg from vertical. */
static const float SKEL_BEND[SEEKER_N_JOINTS][2] = {
    { 0.40f, -0.08f }, { 0.39f, -0.10f }, { 0.41f, -0.10f }, { 0.36f, -0.11f },
    { 0.37f, -0.11f }, { 0.26f, -0.10f }, { 0.28f, -0.10f }, { 0.30f, 0.02f },
    { 0.31f, 0.02f },  { 0.33f, 0.15f },   { 0.34f, 0.15f },  { -0.01f, 0.00f },
    { 0.01f, 0.00f },  { -0.01f, 0.25f },  { 0.01f, 0.25f },  { -0.01f, 0.48f },
    { 0.01f, 0.48f },
};

/* SKEL_STAIRS with the legs swapped (the next stair, the left foot up). */
static void stairs_mirror(float out[SEEKER_N_JOINTS][2])
{
    memcpy(out, SKEL_STAIRS, sizeof SKEL_STAIRS);
    for (int q = 0; q < 2; q++) {
        out[13][q] = SKEL_STAIRS[14][q], out[14][q] = SKEL_STAIRS[13][q];
        out[15][q] = SKEL_STAIRS[16][q], out[16][q] = SKEL_STAIRS[15][q];
    }
}

static int classify_once(const float (*tmpl)[2], const int *hide, int nhide, int cur,
                         seeker_posture_feat *f);

/* tmpl plus noise, as person-local raw joints; joints in hide[] get visibility 0.1. */
static void posture_raw(float raw[SEEKER_N_JOINTS][3], const float (*tmpl)[2], float noise,
                        const int *hide, int nhide)
{
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        raw[j][0] = tmpl[j][0] + noise * (float)gauss();
        raw[j][1] = tmpl[j][1] + noise * (float)gauss();
        raw[j][2] = 0.9f;
    }
    for (int i = 0; i < nhide; i++)
        raw[hide[i]][2] = 0.1f;
}

/* Phone messages at 15 Hz, fusion ticks at 30 Hz: skeleton smoothing then posture, as in
 * hud_fusion. Returns the label after ms; counts label changes; first_ms gets the time of the
 * first change to want (-1 if never). */
static int posture_run(seeker_skel *sk, seeker_posture *ps, int64_t *t, long *seq, int ms,
                       const float (*tmpl)[2], float noise, const int *hide, int nhide,
                       int want, int64_t *first_ms, int *changes)
{
    float raw[SEEKER_N_JOINTS][3], out[SEEKER_N_JOINTS][3];
    int64_t start = *t;
    if (first_ms)
        *first_ms = -1;
    int label = ps->label;
    for (; *t < start + ms; *t += 33) {
        if ((*t - start) % 66 < 33) {
            posture_raw(raw, tmpl, noise, hide, nhide);
            ++*seq;
        }
        int has = seeker_skel_step(sk, "R2", *seq, *t, (const float (*)[3])raw, 1, out);
        int l = seeker_posture_update(ps, has ? (const float (*)[3])out : NULL, *t);
        if (l != label) {
            if (changes)
                ++*changes;
            if (first_ms && *first_ms < 0 && l == want)
                *first_ms = *t - start;
            label = l;
        }
    }
    return label;
}

/* One noise-free frame of tmpl (hide[] at visibility 0.1), classified from cur. */
static int classify_once(const float (*tmpl)[2], const int *hide, int nhide, int cur,
                         seeker_posture_feat *f)
{
    float raw[SEEKER_N_JOINTS][3];
    seeker_posture_feat tmp;
    if (!f)
        f = &tmp;
    posture_raw(raw, tmpl, 0, hide, nhide);
    seeker_posture_features((const float (*)[3])raw, f);
    return seeker_posture_classify(f, cur);
}

static void test_posture(void)
{
    seeker_posture_feat f;
    float raw[SEEKER_N_JOINTS][3];
    static const int legs[] = { 13, 14, 15, 16 }, below_sh[] = { 11, 12, 13, 14, 15, 16 };

    /* One frame each, from UNKNOWN. */
    posture_raw(raw, SKEL_T, 0, NULL, 0);
    CHECK(seeker_posture_features((const float (*)[3])raw, &f) && f.torso_deg < 5 &&
          f.thigh_drop > 0.8f && f.knee_deg > 170);
    CHECK(seeker_posture_classify(&f, SEEKER_POSTURE_UNKNOWN) == SEEKER_POSTURE_STAND);
    posture_raw(raw, SKEL_CROUCH, 0, NULL, 0);
    CHECK(seeker_posture_features((const float (*)[3])raw, &f) && f.thigh_drop < 0.2f &&
          f.knee_deg < 100);
    CHECK(seeker_posture_classify(&f, SEEKER_POSTURE_UNKNOWN) == SEEKER_POSTURE_CROUCH);
    posture_raw(raw, SKEL_DOWN, 0, NULL, 0);
    CHECK(seeker_posture_features((const float (*)[3])raw, &f) && f.torso_deg > 80);
    CHECK(seeker_posture_classify(&f, SEEKER_POSTURE_UNKNOWN) == SEEKER_POSTURE_DOWN);
    /* Partial bodies: an upright upper body is UNKNOWN (standing or crouching behind the wall:
     * no guess); a horizontal one is DOWN; no hips at all is UNKNOWN. */
    posture_raw(raw, SKEL_T, 0, legs, 4);
    CHECK(seeker_posture_features((const float (*)[3])raw, &f) && !f.has_knees);
    CHECK(seeker_posture_classify(&f, SEEKER_POSTURE_UNKNOWN) == SEEKER_POSTURE_UNKNOWN);
    CHECK(seeker_posture_classify(&f, SEEKER_POSTURE_STAND) == SEEKER_POSTURE_UNKNOWN);
    posture_raw(raw, SKEL_DOWN, 0, legs, 4);
    seeker_posture_features((const float (*)[3])raw, &f);
    CHECK(seeker_posture_classify(&f, SEEKER_POSTURE_UNKNOWN) == SEEKER_POSTURE_DOWN);
    posture_raw(raw, SKEL_T, 0, below_sh, 6);
    CHECK(!seeker_posture_features((const float (*)[3])raw, &f) &&
          seeker_posture_classify(&f, SEEKER_POSTURE_STAND) == SEEKER_POSTURE_UNKNOWN);
    /* Knees but no ankles (behind a desk, a stair, the frame edge): no full leg, UNKNOWN, never
     * a crouch, whatever the current label. */
    static const int ankles[] = { 15, 16 }, low_leg[] = { 13, 15 }, low_ankle[] = { 15 };
    posture_raw(raw, SKEL_T, 0, ankles, 2);
    seeker_posture_features((const float (*)[3])raw, &f);
    CHECK(f.has_knees && !f.has_knee_angle && f.n_legs == 0);
    CHECK(seeker_posture_classify(&f, SEEKER_POSTURE_UNKNOWN) == SEEKER_POSTURE_UNKNOWN);
    CHECK(seeker_posture_classify(&f, SEEKER_POSTURE_STAND) == SEEKER_POSTURE_UNKNOWN);
    CHECK(seeker_posture_classify(&f, SEEKER_POSTURE_CROUCH) == SEEKER_POSTURE_UNKNOWN);
    CHECK(classify_once(SKEL_CROUCH, ankles, 2, SEEKER_POSTURE_UNKNOWN, NULL) ==
          SEEKER_POSTURE_UNKNOWN);
    /* MediaPipe's invented hidden legs: under the leg visibility (0.4 < 0.5) they do not count;
     * an ankle folded onto the knee does not count either. */
    posture_raw(raw, SKEL_CROUCH, 0, NULL, 0);
    for (int j = 13; j <= 16; j++)
        raw[j][2] = 0.4f;
    seeker_posture_features((const float (*)[3])raw, &f);
    CHECK(f.n_legs == 0 && seeker_posture_classify(&f, SEEKER_POSTURE_STAND) == SEEKER_POSTURE_UNKNOWN);
    float fold[SEEKER_N_JOINTS][2];
    memcpy(fold, SKEL_T, sizeof fold);
    fold[15][0] = -0.05f, fold[16][0] = 0.05f, fold[15][1] = fold[16][1] = 0.27f;
    CHECK(classify_once((const float (*)[2])fold, NULL, 0, SEEKER_POSTURE_UNKNOWN, &f) ==
              SEEKER_POSTURE_UNKNOWN &&
          f.n_legs == 0);

    /* Standing on stairs: one foot two steps up, its knee bent (about 120 deg). The straight
     * lower leg carries the body: STAND. The old rules (mean of the visible knees) called the
     * same body CROUCH as soon as the step hid the lower leg. */
    CHECK(classify_once(SKEL_STAIRS, NULL, 0, SEEKER_POSTURE_UNKNOWN, &f) == SEEKER_POSTURE_STAND);
    printf("posture stairs: L knee %.0f deg thigh %.2f, R knee %.0f deg thigh %.2f\n",
           f.leg[0].knee_deg, f.leg[0].thigh_drop, f.leg[1].knee_deg, f.leg[1].thigh_drop);
    CHECK(f.n_legs == 2 && f.leg[1].knee_deg < 125 && f.leg[1].thigh_drop < 0.45f);
    for (int cur = SEEKER_POSTURE_UNKNOWN; cur <= SEEKER_POSTURE_STAND; cur++) {
        /* lower leg or only its ankle hidden by the step: one folded leg is no evidence */
        CHECK(classify_once(SKEL_STAIRS, low_leg, 2, cur, NULL) == cur);
        CHECK(classify_once(SKEL_STAIRS, low_ankle, 1, cur, NULL) == cur);
    }
    /* Sitting (both views) and kneeling: both legs folded, CROUCH. Facing the phone the 2D knee
     * angle looks nearly straight; the short, level thigh (thigh drop) gives it away. */
    CHECK(classify_once(SKEL_SIT_FRONT, NULL, 0, SEEKER_POSTURE_STAND, &f) == SEEKER_POSTURE_CROUCH);
    printf("posture sitting, facing: knee %.0f deg, thigh drop %.2f, thigh length %.2f torso\n",
           f.knee_deg, f.thigh_drop, f.leg[0].thigh_ratio);
    CHECK(f.knee_deg > 150 && f.thigh_drop < 0.3f && f.leg[0].thigh_ratio < 0.4f);
    CHECK(classify_once(SKEL_SIT_SIDE, NULL, 0, SEEKER_POSTURE_STAND, &f) == SEEKER_POSTURE_CROUCH &&
          f.knee_deg < 100);
    CHECK(classify_once(SKEL_KNEEL, NULL, 0, SEEKER_POSTURE_STAND, &f) == SEEKER_POSTURE_CROUCH);
    /* Sitting with the shins hidden (under a table): UNKNOWN, not a guess. */
    CHECK(classify_once(SKEL_SIT_FRONT, ankles, 2, SEEKER_POSTURE_UNKNOWN, NULL) ==
          SEEKER_POSTURE_UNKNOWN);
    /* A single deeply folded leg: kept as is by default, CROUCH with one_leg on. */
    static const int left_leg[] = { 13, 15 };
    CHECK(classify_once(SKEL_CROUCH, left_leg, 2, SEEKER_POSTURE_STAND, NULL) == SEEKER_POSTURE_STAND);
    seeker_posture_tune tune, def;
    seeker_posture_tune_defaults(&def);
    tune = def;
    tune.one_leg = 1;
    seeker_posture_set_tune(&tune);
    CHECK(classify_once(SKEL_CROUCH, left_leg, 2, SEEKER_POSTURE_STAND, NULL) == SEEKER_POSTURE_CROUCH);
    CHECK(classify_once(SKEL_STAIRS, low_leg, 2, SEEKER_POSTURE_STAND, NULL) == SEEKER_POSTURE_STAND);
    seeker_posture_set_tune(&def);
    /* Bending over on straight legs: the torso is flat but the legs carry the body, not DOWN. */
    CHECK(classify_once(SKEL_BEND, NULL, 0, SEEKER_POSTURE_UNKNOWN, &f) == SEEKER_POSTURE_UNKNOWN &&
          f.torso_deg > 65);
    CHECK(classify_once(SKEL_BEND, NULL, 0, SEEKER_POSTURE_STAND, NULL) == SEEKER_POSTURE_STAND);
    /* Joints on or past the frame edge lose their visibility (MediaPipe still places them). */
    {
        float lp[SEEKER_N_JOINTS][3], img[SEEKER_N_JOINTS][3];
        for (int j = 0; j < SEEKER_N_JOINTS; j++) {
            lp[j][0] = lp[j][1] = 0, lp[j][2] = 0.9f;
            img[j][0] = 0.5f, img[j][1] = 0.4f + 0.03f * (float)j, img[j][2] = 0.9f;
        }
        img[3][0] = -0.02f, img[15][1] = 1.02f, img[16][1] = 0.996f;
        seeker_posture_mask_offframe(lp, (const float (*)[3])img);
        CHECK(lp[3][2] == 0 && lp[15][2] == 0 && lp[16][2] == 0 && lp[14][2] > 0 && lp[0][2] > 0);
    }
    /* Env knobs. */
    setenv("SEEKER_POSTURE_LEG_VIS", "0.6", 1);
    setenv("SEEKER_POSTURE_VOTES", "5/8", 1);
    setenv("SEEKER_POSTURE_CROUCH_KNEE", "lots", 1);
    {
        char bad[96];
        tune = def;
        CHECK(seeker_posture_tune_env(&tune, bad, sizeof bad) == 2 && tune.leg_vis == 0.6f &&
              tune.vote_n == 5 && tune.vote_m == 8 && tune.crouch_knee == def.crouch_knee &&
              strstr(bad, "SEEKER_POSTURE_CROUCH_KNEE"));
    }
    unsetenv("SEEKER_POSTURE_LEG_VIS");
    unsetenv("SEEKER_POSTURE_VOTES");
    unsetenv("SEEKER_POSTURE_CROUCH_KNEE");

    /* Dead band: knees a little bent (knee about 140 deg) keeps STAND, but does not enter it. */
    float bent[SEEKER_N_JOINTS][2];
    memcpy(bent, SKEL_T, sizeof bent);
    bent[13][0] = -0.16f, bent[14][0] = 0.16f, bent[13][1] = bent[14][1] = 0.22f;
    posture_raw(raw, (const float (*)[2])bent, 0, NULL, 0);
    seeker_posture_features((const float (*)[3])raw, &f);
    printf("posture dead band: knee %.0f deg, thigh drop %.2f\n", f.knee_deg, f.thigh_drop);
    CHECK(seeker_posture_classify(&f, SEEKER_POSTURE_STAND) == SEEKER_POSTURE_STAND);
    CHECK(seeker_posture_classify(&f, SEEKER_POSTURE_UNKNOWN) == SEEKER_POSTURE_UNKNOWN);
    CHECK(!strcmp(seeker_posture_word(SEEKER_POSTURE_DOWN), "DOWN") &&
          seeker_posture_word(SEEKER_POSTURE_UNKNOWN)[0] == 0 &&
          seeker_posture_parse("crouch") == SEEKER_POSTURE_CROUCH && seeker_posture_parse("x") < 0);

    /* Through skeleton smoothing at phone rate: a noisy standing person (sigma 0.03 person
     * heights on every joint) settles on STAND and never changes for 10 s. */
    static seeker_skel sk;
    static seeker_posture ps;
    memset(&sk, 0, sizeof sk);
    memset(&ps, 0, sizeof ps);
    int64_t t = 1000, first;
    long seq = 0;
    int changes = 0;
    CHECK(posture_run(&sk, &ps, &t, &seq, 1500, SKEL_T, 0.03f, NULL, 0, SEEKER_POSTURE_STAND,
                      &first, NULL) == SEEKER_POSTURE_STAND);
    printf("posture: first STAND label %lld ms after the person appears\n", (long long)first);
    CHECK(posture_run(&sk, &ps, &t, &seq, 10000, SKEL_T, 0.03f, NULL, 0, -1, NULL, &changes) ==
          SEEKER_POSTURE_STAND);
    CHECK(changes == 0);
    /* Stand -> crouch within 0.7 s, and back. */
    int64_t to_crouch, to_stand, to_down;
    CHECK(posture_run(&sk, &ps, &t, &seq, 2000, SKEL_CROUCH, 0.03f, NULL, 0, SEEKER_POSTURE_CROUCH,
                      &to_crouch, NULL) == SEEKER_POSTURE_CROUCH);
    CHECK(posture_run(&sk, &ps, &t, &seq, 2000, SKEL_T, 0.03f, NULL, 0, SEEKER_POSTURE_STAND,
                      &to_stand, NULL) == SEEKER_POSTURE_STAND);
    CHECK(posture_run(&sk, &ps, &t, &seq, 2000, SKEL_DOWN, 0.03f, NULL, 0, SEEKER_POSTURE_DOWN,
                      &to_down, NULL) == SEEKER_POSTURE_DOWN);
    printf("posture transitions: stand->crouch %lld ms, crouch->stand %lld ms, stand->down %lld ms\n",
           (long long)to_crouch, (long long)to_stand, (long long)to_down);
    CHECK(to_crouch > 0 && to_crouch <= 700 && to_stand > 0 && to_stand <= 700 && to_down > 0 &&
          to_down <= 900);
    /* Down, legs then hidden behind the wall: still DOWN (torso flat). Standing with legs
     * hidden: UNKNOWN after about 1 s (12 of 15 votes), never a guessed CROUCH. */
    CHECK(posture_run(&sk, &ps, &t, &seq, 1500, SKEL_DOWN, 0.03f, legs, 4, -1, NULL, NULL) ==
          SEEKER_POSTURE_DOWN);
    posture_run(&sk, &ps, &t, &seq, 2000, SKEL_T, 0.03f, NULL, 0, -1, NULL, NULL);
    int64_t to_unknown;
    changes = 0;
    CHECK(posture_run(&sk, &ps, &t, &seq, 2000, SKEL_T, 0.03f, legs, 4, SEEKER_POSTURE_UNKNOWN,
                      &to_unknown, &changes) == SEEKER_POSTURE_UNKNOWN);
    printf("posture: legs hidden, UNKNOWN after %lld ms\n", (long long)to_unknown);
    CHECK(changes == 1 && to_unknown >= 700 && to_unknown <= 1300);
    /* Ankles hidden behind a step while standing: UNKNOWN, never CROUCH. */
    posture_run(&sk, &ps, &t, &seq, 2000, SKEL_T, 0.03f, NULL, 0, -1, NULL, NULL);
    changes = 0;
    CHECK(posture_run(&sk, &ps, &t, &seq, 3000, SKEL_T, 0.03f, ankles, 2, SEEKER_POSTURE_CROUCH,
                      &first, &changes) == SEEKER_POSTURE_UNKNOWN &&
          first < 0 && changes == 1);

    /* Climbing the stairs for 12 s (a step every 0.6 s, alternate legs up, noise), and every
     * other step the lower leg hidden by the stair: STAND, never CROUCH. */
    float stairs_l[SEEKER_N_JOINTS][2];
    stairs_mirror(stairs_l);
    posture_run(&sk, &ps, &t, &seq, 2000, SKEL_T, 0.03f, NULL, 0, -1, NULL, NULL);
    int crouched = 0, stair_changes = 0;
    static const int low_right[] = { 14, 16 };
    for (int step = 0; step < 20; step++) {
        /* 0: right foot up; 1: the same, lower (left) leg hidden; 2, 3: left foot up */
        const int *hide = step % 4 == 1 ? low_leg : step % 4 == 3 ? low_right : NULL;
        int nh = hide ? 2 : 0;
        const float (*tm)[2] = step % 4 < 2 ? SKEL_STAIRS : (const float (*)[2])stairs_l;
        changes = 0;
        int l = posture_run(&sk, &ps, &t, &seq, 600, tm, 0.03f, hide, nh, SEEKER_POSTURE_CROUCH,
                            &first, &changes);
        crouched += first >= 0 || l == SEEKER_POSTURE_CROUCH;
        stair_changes += changes;
    }
    printf("posture: 12 s climbing stairs: %d crouch labels, %d changes, ends %s\n", crouched,
           stair_changes, seeker_posture_name(ps.label));
    CHECK(crouched == 0 && stair_changes == 0 && ps.label == SEEKER_POSTURE_STAND);
    /* N of M: a 200 ms crouch-shaped glitch (3 phone frames) while standing never flips. */
    changes = 0;
    posture_run(&sk, &ps, &t, &seq, 200, SKEL_CROUCH, 0.03f, NULL, 0, -1, NULL, &changes);
    posture_run(&sk, &ps, &t, &seq, 2000, SKEL_T, 0.03f, NULL, 0, -1, NULL, &changes);
    CHECK(changes == 0 && ps.label == SEEKER_POSTURE_STAND);
    /* Sitting down (facing the phone) and getting up: CROUCH and back, one change each. */
    int64_t to_sit;
    changes = 0;
    CHECK(posture_run(&sk, &ps, &t, &seq, 2000, SKEL_SIT_FRONT, 0.03f, NULL, 0,
                      SEEKER_POSTURE_CROUCH, &to_sit, &changes) == SEEKER_POSTURE_CROUCH);
    CHECK(posture_run(&sk, &ps, &t, &seq, 2000, SKEL_T, 0.03f, NULL, 0, -1, NULL, &changes) ==
          SEEKER_POSTURE_STAND);
    printf("posture: sitting down, CROUCH after %lld ms\n", (long long)to_sit);
    CHECK(changes == 2 && to_sit > 0 && to_sit <= 800);

    /* Wire: posture rides the tracks datagram, only when known. */
    static seeker_tracks tr, back;
    memset(&tr, 0, sizeof tr);
    tr.n = 2;
    tr.tracks[0].posture = SEEKER_POSTURE_DOWN;
    tr.tracks[1].posture = SEEKER_POSTURE_UNKNOWN;
    char buf[SEEKER_MAX_DATAGRAM], err[64];
    int len = seeker_encode_tracks(&tr, buf, sizeof buf);
    CHECK(len > 0 && strstr(buf, "\"posture\":\"down\"") && !strstr(buf, "unknown"));
    CHECK(seeker_decode_tracks(buf, len, &back, err, sizeof err) == 0 &&
          back.tracks[0].posture == SEEKER_POSTURE_DOWN &&
          back.tracks[1].posture == SEEKER_POSTURE_UNKNOWN);

    /* Through hud_fusion: an R2-only person who crouches is labelled on the track. */
    static seeker_config cfg;
    CHECK(seeker_config_load(&cfg, src_file("config/seeker.live.json")) == 0);
    static seeker_fusion fu;
    seeker_fusion_init(&fu, &cfg);
    const seeker_pose r2 = { -2.2f, -0.6f, 90 };
    seeker_fusion_set_pose(&fu, "R2", &r2, "cli", 1);
    static seeker_tracks ft;
    int64_t now = 1000, crouch_at = -1;
    for (int i = 0; i < 60; i++, now += 66) {
        seeker_remote_contacts m = phone_msg("R2", &r2, -0.4f, -1.2f, 1);
        m.width = 640;
        m.height = 480;
        m.hfov_deg = 66;
        posture_raw(raw, i < 30 ? SKEL_T : SKEL_CROUCH, 0.02f, NULL, 0);
        float fpx = 320.0f / tanf(33.0f * 0.017453293f), hpx = fpx * 1.7f / m.c[0].range_m;
        float u = 320.0f + fpx * tanf(m.c[0].bearing_deg * 0.017453293f);
        for (int j = 0; j < SEEKER_N_JOINTS; j++) {
            m.c[0].pts[j][0] = (u + raw[j][0] * hpx) / 640.0f;
            m.c[0].pts[j][1] = (240.0f + raw[j][1] * hpx) / 480.0f;
            m.c[0].pts[j][2] = 0.9f;
        }
        m.c[0].has_pts = 1;
        seeker_fusion_ingest(&fu, &m, now);
        seeker_fusion_tick(&fu, now + 5, &ft);
        seeker_fusion_tick(&fu, now + 38, &ft);
        if (i == 29)
            CHECK(ft.n == 1 && ft.tracks[0].posture == SEEKER_POSTURE_STAND);
        if (i >= 30 && crouch_at < 0 && ft.tracks[0].posture == SEEKER_POSTURE_CROUCH)
            crouch_at = now + 38 - (1000 + 30 * 66);
    }
    printf("fusion posture: stand -> crouch on the track after %lld ms\n", (long long)crouch_at);
    CHECK(crouch_at > 0 && crouch_at <= 700);

    /* Standing close to R2 with the legs below the frame: MediaPipe still places knees and
     * ankles (here folded under the hips, visibility 0.9) past the bottom edge. They are masked:
     * the person reads UNKNOWN, never CROUCH. */
    static seeker_fusion fo;
    seeker_fusion_init(&fo, &cfg);
    seeker_fusion_set_pose(&fo, "R2", &r2, "cli", 1);
    int off_crouch = 0, off_last = -1;
    for (int i = 0; i < 45; i++, now += 66) {
        seeker_remote_contacts m = phone_msg("R2", &r2, -0.4f, -1.2f, 1);
        m.width = 640;
        m.height = 480;
        m.hfov_deg = 66;
        posture_raw(raw, i < 15 ? SKEL_T : SKEL_CROUCH, 0.01f, NULL, 0);
        float fpx = 320.0f / tanf(33.0f * 0.017453293f), hpx = fpx * 1.7f / m.c[0].range_m;
        float u = 320.0f + fpx * tanf(m.c[0].bearing_deg * 0.017453293f);
        float hip_y = i < 15 ? 240.0f : 0.97f * 480.0f;
        for (int j = 0; j < SEEKER_N_JOINTS; j++) {
            m.c[0].pts[j][0] = (u + raw[j][0] * hpx) / 640.0f;
            m.c[0].pts[j][1] = (hip_y + raw[j][1] * hpx) / 480.0f;
            m.c[0].pts[j][2] = 0.9f;
        }
        m.c[0].has_pts = 1;
        seeker_fusion_ingest(&fo, &m, now);
        seeker_fusion_tick(&fo, now + 5, &ft);
        seeker_fusion_tick(&fo, now + 38, &ft);
        if (ft.n == 1) {
            off_crouch += i >= 15 && ft.tracks[0].posture == SEEKER_POSTURE_CROUCH;
            off_last = ft.tracks[0].posture;
        }
    }
    printf("fusion posture: legs below the frame: %d crouch ticks, ends %s\n", off_crouch,
           seeker_posture_name(off_last));
    CHECK(off_crouch == 0 && off_last == SEEKER_POSTURE_UNKNOWN);

    /* HUD: the wearer view tags the x-ray person (and counts postures for the log line). */
    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    v.cfg.inset = 0;
    canvas c = { calloc(640 * 360, 4), 640, 360, 640 };
    CHECK(c.px != NULL);
    if (!c.px)
        return;
    static seeker_tracks wt;
    memset(&wt, 0, sizeof wt);
    add_node(&wt, "W", "ok", 0, 0, 0);
    seeker_track *k = add_track(&wt, "R2", "R2", 1, 0.0f, 3.0f);
    k->posture = SEEKER_POSTURE_DOWN;
    seeker_track *k2 = add_track(&wt, "W", "W", 1, 1.0f, 4.0f);
    k2->posture = SEEKER_POSTURE_CROUCH;
    wv_render(&c, &cfg, &v, &wt, 0, NULL);
    CHECK(v.last.n_xray == 1 && v.last.n_direct == 1 &&
          v.last.n_posture[SEEKER_POSTURE_DOWN] == 1 && v.last.n_posture[SEEKER_POSTURE_CROUCH] == 1);
    free(c.px);
}

/* Calibration: Pi-side FOV (hud_fusion --hfov), wearer-relative ranges (fusion.wearer_node),
 * and the wearer view keeping a frozen picture through a short VIDEO LOST. */
static void test_calibration_and_freeze(void)
{
    float land, port;
    CHECK(seeker_hfov_preset("iphone17pro", &land, &port) == 0 && NEAR(land, 71.6, 0.1) &&
          NEAR(port, 56.8, 0.1));
    CHECK(seeker_hfov_preset("iphone16", &land, &port) == 0 && NEAR(land, 67.3, 0.1) &&
          NEAR(port, 53.1, 0.1));
    CHECK(seeker_hfov_preset("iphone-ultrawide", &land, &port) == 0 && NEAR(land, 106.2, 0.1));
    CHECK(seeker_hfov_preset("pixel", &land, &port) != 0);

    static seeker_config cfg;
    CHECK(seeker_config_load(&cfg, src_file("config/seeker.live.json")) == 0);
    CHECK(!strcmp(cfg.wearer_node, "R1") && cfg.predict_ms == 100);
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    CHECK(seeker_fusion_set_hfov(&f, "R2", "iphone16") >= 0);
    CHECK(seeker_fusion_set_hfov(&f, "R2", "500") < 0 && seeker_fusion_set_hfov(&f, "R2", "x") < 0);
    const seeker_pose r1 = { 0.5f, -2.5f, 10 }, r2 = { -2.4f, 0.5f, 80 };
    seeker_fusion_set_pose(&f, "R1", &r1, "cli", 1);
    seeker_fusion_set_pose(&f, "R2", &r2, "cli", 1);
    /* R2's phone computed its bearing with its default 66 deg; the real FOV is 67.3. */
    seeker_remote_contacts m = phone_msg("R2", &r2, 0.4f, 1.2f, 1);
    float real_b = m.c[0].bearing_deg;
    float sent_b = atanf(tanf(real_b * 0.0174533f) * tanf(33.0f * 0.0174533f) /
                         tanf(67.3f / 2 * 0.0174533f)) * 57.29578f;
    m.c[0].bearing_deg = sent_b;
    /* ... and its box-height range with that focal length: too long by tan(67.3/2)/tan(33). */
    float true_r = m.c[0].range_m;
    m.c[0].range_m = true_r * tanf(67.3f / 2 * 0.0174533f) / tanf(33.0f * 0.0174533f);
    m.hfov_deg = 66;
    m.width = 640;
    m.height = 480;
    int i2 = seeker_fusion_ingest(&f, &m, 1000);
    CHECK(i2 >= 0 && NEAR(f.nodes[i2].last.c[0].bearing_deg, real_b, 0.05) &&
          NEAR(f.nodes[i2].hfov_deg, 67.3, 0.1) && NEAR(f.nodes[i2].last.c[0].range_m, true_r, 0.01));
    /* Ranges and bearings from R1 (wearer_node), not from tag 0. */
    static seeker_tracks t;
    seeker_fusion_tick(&f, 1005, &t);
    CHECK(t.n == 1 && !strcmp(t.wearer_node, "R1") && NEAR(t.wearer_yaw_deg, 10, 1e-4));
    float want = hypotf(0.4f - 0.5f, 1.2f + 2.5f);
    CHECK(NEAR(t.tracks[0].range_m, want, 0.05));
    CHECK(NEAR(t.tracks[0].bearing_deg, atan2f(-0.1f, 3.7f) * 57.29578f - 10, 0.5));
    char buf[SEEKER_MAX_DATAGRAM], err[64];
    static seeker_tracks back;
    int len = seeker_encode_tracks(&t, buf, sizeof buf);
    CHECK(len > 0 && seeker_decode_tracks(buf, len, &back, err, sizeof err) == 0 &&
          !strcmp(back.wearer_node, "R1"));

    /* Freeze: LOST with a recent frame of this relay keeps the wearer view (quarter bright). */
    static seeker_config dc;
    seeker_config_defaults(&dc);
    enum { W = 128, H = 72 };
    static uint32_t px[W * H], vid[W * H];
    canvas c = { px, W, H, W };
    for (int i = 0; i < W * H; i++)
        vid[i] = RGB(200, 0, 0);
    video_frame vf;
    memset(&vf, 0, sizeof vf);
    vf.px = vid;
    vf.w = vf.stride = vf.src_w = W;
    vf.h = vf.src_h = H;
    vf.rect[2] = W;
    vf.rect[3] = H;
    vf.t_rx_ms = 1000;
    static wv_view v;
    memset(&v, 0, sizeof v);
    wv_config_defaults(&v.cfg);
    v.cfg.radar_fallback = 1;
    v.cfg.inset = 0;
    snprintf(vf.node_id, sizeof vf.node_id, "%s", v.cfg.node);
    video_stats st;
    memset(&st, 0, sizeof st);
    st.connected = 1;
    hud_state hs = { .background = "wearer", .view_node = v.cfg.node, .video = &vf,
                     .tracks_age_ms = -1, .wearer = &v };
    int64_t now = 1000 + VIDEO_HOLD_MS + 500;
    hs.video_state = video_rx_state(&vf, &st, now);
    hs.video_frozen = video_rx_frozen(&vf, &st, now, VIDEO_FREEZE_MS);
    CHECK(hs.video_state == VIDEO_LOST && hs.video_frozen);
    CHECK(hud_render(&c, &dc, &hs) == 1 && v.last.rendered && px[36 * W + 4] == RGB(50, 0, 0));
    now = 1000 + VIDEO_FREEZE_MS + 1;
    hs.video_frozen = video_rx_frozen(&vf, &st, now, VIDEO_FREEZE_MS);
    CHECK(!hs.video_frozen && hud_render(&c, &dc, &hs) == 0 && !v.last.rendered); /* radar */
    st.conn = 1; /* a new relay: the old picture is not frozen, radar at once */
    CHECK(!video_rx_frozen(&vf, &st, 1000 + VIDEO_HOLD_MS + 1, VIDEO_FREEZE_MS));
}

/* Felix's run: R1 lost tag 0, and the HUD stayed choppy after the tag came back. Replay: R1
 * live 5 s (tag pose noise, 20% of frames the flat tag's mirror solution), tag lost 3 s (pose
 * null) while the wearer turns 30 deg, then back; the first pose back is an outlier (the tag at
 * the picture's edge). The outlier is never taken, the pose is back on the second agreeing
 * frame and on the new heading within a second, and afterwards it is as steady as before. */
static void test_tag_reacquire(void)
{
    static seeker_config cfg;
    CHECK(seeker_config_load(&cfg, src_file("config/seeker.live.json")) == 0);
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    seeker_pose r1 = { 0.0f, -2.0f, 0.0f };
    CHECK(seeker_fusion_set_pose(&f, "R1", &r1, "init", 0) >= 0);
    static seeker_tracks tr;
    double step_sum[2] = { 0, 0 }, step_max[2] = { 0, 0 }, prev = 0;
    int steps[2] = { 0, 0 }, took_outlier = 0, back_ms = -1, msgs_back = 0;
    float err_late = 0;
    for (int64_t t = 0; t < 14000; t += 33) {
        if (t % 99 == 0) {
            int lost = t >= 5000 && t < 8000;
            float yaw = t >= 8000 ? 50.0f : 20.0f, x = 0.7f, y = -1.85f;
            char js[128];
            if (lost) {
                snprintf(js, sizeof js, "null");
            } else if (t >= 8000 && msgs_back++ == 0) {
                snprintf(js, sizeof js, "{\"x_m\":2.2,\"y_m\":-0.5,\"yaw_deg\":-60}"); /* edge outlier */
            } else {
                float px = x + 0.04f * (float)gauss(), py = y + 0.04f * (float)gauss();
                float pyaw = yaw + 3.0f * (float)gauss();
                if (urand() < 0.2) { /* the mirror solution */
                    float b = atan2f(-px, -py) * 57.29578f;
                    px = -px;
                    pyaw = pyaw - 2.0f * b;
                }
                snprintf(js, sizeof js, "{\"x_m\":%.3f,\"y_m\":%.3f,\"yaw_deg\":%.2f}", px, py, pyaw);
            }
            r1_pose_msg(&f, js, t);
        }
        seeker_fusion_tick(&f, t, &tr);
        const seeker_fusion_node *n = &f.nodes[0];
        took_outlier |= hypotf(n->x_m - 2.2f, n->y_m + 0.5f) < 0.6f;
        if (back_ms < 0 && t >= 8000 && !strcmp(n->pose_src, "message"))
            back_ms = (int)(t - 8000);
        int ph = t >= 2000 && t < 5000 ? 0 : t >= 10000 ? 1 : -1;
        double d = fabs(seeker_wrap180(n->yaw_deg - (float)prev));
        if (ph >= 0) {
            step_sum[ph] += d;
            steps[ph]++;
            if (d > step_max[ph])
                step_max[ph] = d;
        }
        if (t >= 9000) {
            float e = hypotf(n->x_m - 0.7f, n->y_m + 1.85f) + fabsf(seeker_wrap180(n->yaw_deg - 50.0f)) / 30.0f;
            if (e > err_late)
                err_late = e;
        }
        prev = n->yaw_deg;
    }
    printf("tag reacquire: back %d ms after the tag, %ld reacquisitions, yaw step mean %.2f -> %.2f "
           "max %.2f -> %.2f deg (before the loss -> after), worst late error %.2f\n",
           back_ms, f.nodes[0].reacqs, step_sum[0] / steps[0], step_sum[1] / steps[1], step_max[0],
           step_max[1], err_late);
    CHECK(!took_outlier && back_ms >= 0 && back_ms <= 300 && f.nodes[0].reacqs == 1);
    CHECK(step_sum[1] / steps[1] < 1.5 * step_sum[0] / steps[0] + 0.1 && step_max[1] < 3.0);
    CHECK(err_late < 0.35f);
}

/* Felix's run: the x-ray (R2-only) person drawn far too low in R1's view. R1 is handheld at chest
 * height (1.30 m), not 1.7 m, and tilted up a little; the projection assumed a level camera at
 * 1.7 m. Truth here: 1.30 m, 6 deg up. Old assumptions, R1_CAM_H=1.30 alone, and with the tilt
 * learned from a person R1 sees itself: the x-ray box's middle against the truth, in px. */
static void test_xray_height(void)
{
    static seeker_config cfg;
    seeker_config_defaults(&cfg);
    const int W = 640, H = 480;
    wv_camera truth = { 67.3f, W, H, 1.30f, -4.0f, 0.0f };
    const wv_pose r1 = { 0.0f, -2.0f, 0.0f };
    wv_proj pd, px; /* the direct person and the x-ray person through the true camera */
    CHECK(wv_project_point(&r1, &truth, 0.6f, 1.0f, &pd) && wv_project_point(&r1, &truth, -0.8f, 2.0f, &px));
    float truth_mid = px.box_y + 0.5f * px.box_h;
    static seeker_tracks t;
    memset(&t, 0, sizeof t);
    add_node(&t, "R1", "ok", 0.0f, -2.0f, 0.0f);
    add_node(&t, "R2", "ok", -1.59f, -1.4f, 49.0f);
    seeker_track *kd = add_track(&t, "R1", "R1", 1, 0.6f, 1.0f);
    kd->has_node_box = 1;
    kd->node_box[0] = pd.box_x, kd->node_box[1] = pd.box_y, kd->node_box[2] = pd.box_w;
    kd->node_box[3] = pd.box_h;
    add_track(&t, "R2", "R2", 1, -0.8f, 2.0f);
    static uint32_t px_buf[640 * 480], canvas_px[640 * 480];
    canvas c = { canvas_px, W, H, W };
    video_frame f;
    memset(&f, 0, sizeof f);
    f.px = px_buf;
    f.w = f.stride = W;
    f.h = H;
    f.src_w = W;
    f.src_h = H;
    f.rect[2] = W;
    f.rect[3] = H;
    snprintf(f.node_id, sizeof f.node_id, "R1");
    float err[3];
    for (int mode = 0; mode < 3; mode++) {
        static wv_view v;
        memset(&v, 0, sizeof v);
        wv_config_defaults(&v.cfg);
        snprintf(v.cfg.node, sizeof v.cfg.node, "R1");
        v.cfg.pose = r1;
        v.cfg.pose_src = WV_POSE_STATIC;
        v.cfg.cam.hfov_deg = 67.3f;
        v.cfg.hfov_fixed = 1;
        v.cfg.cam.cam_h_m = mode == 0 ? 1.7f : 1.30f;
        v.cfg.pitch_auto = mode == 2;
        v.box_ease_off = 1;
        for (int k = 0; k < 60; k++) {
            v.clock_ms = 1000 + 33 * k;
            wv_render(&c, &cfg, &v, &t, 0, &f);
        }
        float mid = -1;
        for (int i = 0; i < v.last.n_drawn; i++)
            if (!v.last.drawn[i].direct)
                mid = (float)v.last.drawn[i].y + 0.5f * (float)v.last.drawn[i].h;
        err[mode] = mid - truth_mid;
        if (mode == 2)
            printf("x-ray height: learned tilt %+.1f deg from %d sightings (direct %d xray %d)\n",
                   v.tilt_fix, v.tilt_n, v.last.n_direct, v.last.n_xray);
    }
    printf("x-ray height, middle of the box vs truth (px, + is too low): old %.0f, R1_CAM_H=1.30 "
           "%.0f, + learned tilt %.0f\n", err[0], err[1], err[2]);
    CHECK(err[0] > 60.0f);                 /* the bug: far too low */
    CHECK(fabsf(err[1]) < fabsf(err[0]));  /* the height alone helps */
    CHECK(fabsf(err[2]) < 8.0f);           /* the learned tilt puts it on the person */

    /* Pitch on the wire: pose.pitch_deg (or top level) reaches the node status. */
    seeker_remote_contacts m;
    char err_s[96];
    const char *js = "{\"type\":\"remote_contacts\",\"v\":2,\"node_id\":\"R1\",\"t_mono_ms\":1,"
                     "\"pose\":{\"x_m\":0,\"y_m\":-2,\"yaw_deg\":0,\"pitch_deg\":6.5},\"contacts\":[]}";
    CHECK(seeker_decode_remote_contacts(js, (int)strlen(js), &m, err_s, sizeof err_s) == 0 &&
          m.has_pitch && NEAR(m.pitch_deg, 6.5, 1e-4));
    /* Ground-plane range: feet 0.6 m below a 1.27 m level camera's line at 3 m. */
    float f640 = 320.0f / tanf(35.8f * 0.0174533f);
    float v_feet = 240.0f + f640 * 1.27f / 3.0f;
    CHECK(NEAR(seeker_ground_range(v_feet, 640, 480, 71.6f, 1.27f, 0.0f), 3.0f, 0.01));
    CHECK(seeker_ground_range(200.0f, 640, 480, 71.6f, 1.27f, 0.0f) == 0); /* above the horizon */
}

/* Felix's Pi run (R2_MODE=tag, LAT=1): R1's tag pose flipped between two clusters 1.4 m and
 * 40 deg apart, the two solutions of a flat 186 mm tag seen about 20 deg off its axis at 2 m.
 * The old gate took the other cluster after 3 frames in a row, so the wearer (and the whole
 * plan around it) jumped back and forth. Replayed with the logged poses: R1 stays on its
 * solution through mixed flips and runs of 3 or 4, and switches only when the other one wins
 * 8 of 10 frames. */
static void test_tag_flip_replay(void)
{
    static const float A[][3] = { { 0.62f, -1.86f, -31.4f }, { 0.69f, -1.82f, -33.6f },
                                  { 0.78f, -1.78f, -36.2f }, { 0.70f, -1.81f, -34.3f },
                                  { 0.66f, -1.85f, -33.0f } };
    static const float B[][3] = { { -0.59f, -1.88f, 10.6f }, { -0.61f, -1.87f, 11.2f } };
    /* The model: A's mirror lands on B (and back) within the flip tolerance. */
    for (int i = 0; i < 5; i++) {
        seeker_pose a = { A[i][0], A[i][1], A[i][2] }, m;
        float b = atan2f(-a.x_m, -a.y_m) * 57.29578f;
        m.x_m = -a.x_m, m.y_m = a.y_m, m.yaw_deg = seeker_wrap180(a.yaw_deg - 2.0f * b);
        CHECK(hypotf(m.x_m - B[0][0], m.y_m - B[0][1]) < SEEKER_POSE_FLIP_M &&
              fabsf(seeker_wrap180(m.yaw_deg - B[0][2])) < SEEKER_POSE_FLIP_DEG);
    }
    static seeker_config cfg;
    CHECK(seeker_config_load(&cfg, src_file("config/seeker.live.json")) == 0);
    static seeker_fusion f;
    /* A sequence like the log: mostly A, B in ones, twos, threes and fours. */
    static const char seq[] = "AAAAABAABBAAABBBAAAABBBBAABAAAAABABBBAAAAA";
    seeker_pose r1 = { 0.0f, -2.0f, 0.0f };
    float worst_d = 0, worst_yaw = 0;
    int64_t t = 1000;
    for (int before = 1; before >= 0; before--) { /* before: the jump gate alone (main) */
        seeker_fusion_init(&f, &cfg);
        f.pose_flip = !before;
        CHECK(seeker_fusion_set_pose(&f, "R1", &r1, "init", 0) >= 0);
        t = 1000;
        worst_d = worst_yaw = 0;
        for (int k = 0; seq[k]; k++) {
            const float *p = seq[k] == 'A' ? A[k % 5] : B[k % 2];
            char js[96];
            snprintf(js, sizeof js, "{\"x_m\":%.2f,\"y_m\":%.2f,\"yaw_deg\":%.1f}", p[0], p[1], p[2]);
            r1_pose_msg(&f, js, t += 100);
            const seeker_fusion_node *n = &f.nodes[0];
            float d = hypotf(n->x_m - 0.69f, n->y_m + 1.82f), dy = fabsf(seeker_wrap180(n->yaw_deg + 33.6f));
            if (k > 0 && d > worst_d)
                worst_d = d;
            if (k > 0 && dy > worst_yaw)
                worst_yaw = dy;
        }
        CHECK(!strcmp(f.nodes[0].id, "R1"));
        printf("tag flip replay, %s: %ld flips kept on the first solution, R1 at most %.2f m / %.1f deg "
               "from it\n", before ? "before (gate only)" : "after", f.nodes[0].flips, worst_d, worst_yaw);
        if (before)
            CHECK(worst_d > 1.0f); /* the old gate took the other solution */
    }
    CHECK(f.nodes[0].flips >= 15 && worst_d < 0.25f && worst_yaw < 8.0f);
    /* The other solution for good (the phone settles on it): R1 follows within a second. */
    for (int k = 0; k < 12; k++) {
        char js[96];
        snprintf(js, sizeof js, "{\"x_m\":%.2f,\"y_m\":%.2f,\"yaw_deg\":%.1f}", B[k % 2][0],
                 B[k % 2][1], B[k % 2][2]);
        r1_pose_msg(&f, js, t += 100);
    }
    const seeker_fusion_node *n = &f.nodes[0];
    printf("tag flip replay: after 12 frames of the other solution R1 is at %.2f,%.2f,%.1f\n", n->x_m,
           n->y_m, n->yaw_deg);
    CHECK(hypotf(n->x_m + 0.60f, n->y_m + 1.875f) < 0.15f && fabsf(seeker_wrap180(n->yaw_deg - 10.9f)) < 5.0f);
}

/* Felix's two-phone run: R2 pinned to the wearer's left looked "on the wrong side and moving".
 * A pinned R2 never moves in the room; on the HUD's wearer-centred, heading-up plan it moves only
 * with R1's tag pose. R1's tag pose here is noisy (6 cm, 4 deg per frame, 10 Hz): R2 must stay
 * on the left every frame, and the smoothing must shrink its shake. */
static void test_r2_pinned_left(void)
{
    static seeker_config cfg;
    CHECK(seeker_config_load(&cfg, src_file("config/seeker.live.json")) == 0);
    float sd_right[2] = { 0, 0 }, sd_ahead[2] = { 0, 0 }, worst_right[2] = { -99, -99 };
    for (int mode = 0; mode < 2; mode++) {
        static seeker_fusion f;
        seeker_fusion_init(&f, &cfg);
        f.pose_smooth = mode;
        seeker_pose r1 = { 0.0f, -2.0f, 0.0f }, r2 = { -1.5f, -2.0f, 90.0f };
        CHECK(seeker_fusion_set_pose(&f, "R1", &r1, "init", 0) >= 0);
        CHECK(seeker_fusion_set_pose(&f, "R2", &r2, "cli", 1) >= 0);
        float right0, ahead0;
        CHECK(seeker_fusion_relative(&f, "R1", "R2", &right0, &ahead0) == 0);
        CHECK(NEAR(right0, -1.5f, 1e-4) && NEAR(ahead0, 0.0f, 1e-4)); /* hud_fusion's start line */
        static wv_view v;
        memset(&v, 0, sizeof v);
        wv_config_defaults(&v.cfg);
        snprintf(v.cfg.node, sizeof v.cfg.node, "R1");
        v.cfg.pose_src = WV_POSE_AUTO;
        v.cfg.pose.x_m = 0.0f;
        v.cfg.pose.y_m = -2.0f;
        enum { W = 320, H = 180 };
        static uint32_t px[W * H];
        canvas c = { px, W, H, W };
        draw_set_theme(DRAW_THEME_PLAN);
        static seeker_tracks tr;
        double sr = 0, sr2 = 0, sa = 0, sa2 = 0;
        int n = 0, left = 1, pinned = 1;
        for (int64_t t = 0; t < 10000; t += 33) {
            if (t % 99 == 0) {
                char js[128];
                snprintf(js, sizeof js, "{\"x_m\":%.3f,\"y_m\":%.3f,\"yaw_deg\":%.2f,\"tag_id\":0}",
                         0.06 * gauss(), -2.0 + 0.06 * gauss(), 4.0 * gauss());
                r1_pose_msg(&f, js, t);
            }
            seeker_fusion_tick(&f, t, &tr);
            v.clock_ms = t;
            wv_render(&c, &cfg, &v, &tr, 0, NULL);
            if (t < 1000)
                continue;
            const seeker_node_status *s2 = NULL;
            for (int i = 0; i < tr.n_nodes; i++)
                if (!strcmp(tr.nodes[i].node_id, "R2"))
                    s2 = &tr.nodes[i];
            if (!s2)
                continue;
            pinned &= NEAR(s2->x_m, -1.5f, 1e-4) && NEAR(s2->y_m, -2.0f, 1e-4);
            float right, ahead; /* where the plan draws R2, as wv_plan calls plan_map */
            plan_rel(&v.last.pose, v.last.pose.yaw_deg, s2->x_m, s2->y_m, &right, &ahead);
            left &= right < -1.0f;
            if (right > worst_right[mode])
                worst_right[mode] = right;
            sr += right, sr2 += (double)right * right, sa += ahead, sa2 += (double)ahead * ahead;
            n++;
        }
        CHECK(n > 200 && pinned && left);
        sd_right[mode] = (float)sqrt(sr2 / n - (sr / n) * (sr / n));
        sd_ahead[mode] = (float)sqrt(sa2 / n - (sa / n) * (sa / n));
    }
    draw_set_theme(DRAW_THEME_CLASSIC);
    printf("R2 pinned 1.5 m left of a noisy R1 (6 cm, 4 deg): on the plan, shake right %.3f -> %.3f m, "
           "ahead %.3f -> %.3f m (raw -> smoothed), never right of %.2f m\n",
           sd_right[0], sd_right[1], sd_ahead[0], sd_ahead[1], worst_right[1]);
    CHECK(sd_right[1] < 0.6f * sd_right[0] && sd_ahead[1] < 0.85f * sd_ahead[0]);
    CHECK(worst_right[1] < -1.3f);
}

/* Moving wearer without a tag: held, compass yaw (heading_deg), home after pose_home_ms, and a
 * tag back eased in (fusion step_pose). */
static void test_moving_wearer_hold(void)
{
    static seeker_config cfg;
    CHECK(seeker_config_load(&cfg, src_file("config/seeker.live.json")) == 0);
    CHECK(cfg.pose_home_ms == 10000);
    static seeker_fusion f;
    seeker_fusion_init(&f, &cfg);
    const seeker_pose home = { 0, -2.5f, 0 };
    int i1 = seeker_fusion_set_pose(&f, "R1", &home, "init", 0);
    CHECK(i1 >= 0 && f.nodes[i1].has_home);
    static seeker_tracks t;
    char js[200], err[64];
    int64_t now = 1000;
#define R1MSG(pose, head)                                                                          \
    do {                                                                                           \
        snprintf(js, sizeof js,                                                                    \
                 "{\"type\":\"remote_contacts\",\"v\":2,\"node_id\":\"R1\",\"pose\":%s,"           \
                 "\"heading_deg\":%.1f,\"contacts\":[]}", pose, (double)(head));                  \
        seeker_remote_contacts m;                                                                  \
        CHECK(seeker_decode_remote_contacts(js, (int)strlen(js), &m, err, sizeof err) == 0 &&      \
              m.has_heading);                                                                      \
        seeker_fusion_ingest(&f, &m, now);                                                         \
        seeker_fusion_tick(&f, now + 5, &t);                                                       \
        now += 66;                                                                                 \
    } while (0)
    for (int i = 0; i < 10; i++) /* tag in view, compass reads 30 deg less than the room yaw */
        R1MSG("{\"x_m\":0.8,\"y_m\":-2.0,\"yaw_deg\":20}", -10.0f);
    seeker_fusion_node *n = &f.nodes[i1];
    CHECK(NEAR(n->x_m, 0.8, 1e-4) && n->has_head_off && NEAR(n->head_off, 30, 0.01));
    /* Tag lost while the wearer turns right to 50 deg: position held, yaw follows the compass. */
    for (int i = 0; i < 15; i++)
        R1MSG("null", 20.0f);
    CHECK(!strcmp(n->pose_src, "heading") && NEAR(n->x_m, 0.8, 1e-4) && NEAR(n->yaw_deg, 50, 0.5));
    /* 10 s without a tag: eases home (x,y), yaw stays on the compass. */
    for (int i = 0; i < 160; i++)
        R1MSG("null", 20.0f);
    CHECK(!strcmp(n->pose_src, "home") && NEAR(n->x_m, 0, 0.01) && NEAR(n->y_m, -2.5, 0.01) &&
          NEAR(n->yaw_deg, 50, 0.5));
    /* Tag back: no jump; within a second at the tag pose, live (two poses that agree). */
    R1MSG("{\"x_m\":1.0,\"y_m\":-2.2,\"yaw_deg\":45}", 15.0f);
    R1MSG("{\"x_m\":1.0,\"y_m\":-2.2,\"yaw_deg\":45}", 15.0f);
    CHECK(!strcmp(n->pose_src, "message") && n->x_m < 0.8f);
    for (int i = 0; i < 15; i++)
        R1MSG("{\"x_m\":1.0,\"y_m\":-2.2,\"yaw_deg\":45}", 15.0f);
    CHECK(NEAR(n->x_m, 1.0, 0.01) && NEAR(n->y_m, -2.2, 0.01) && NEAR(n->yaw_deg, 45, 0.5));
    /* The HUD calls all three no-tag states held (amber). */
    wv_config wc;
    wv_config_defaults(&wc);
    snprintf(wc.node, sizeof wc.node, "R1");
    snprintf(t.nodes[0].pose_src, sizeof t.nodes[0].pose_src, "home");
    snprintf(t.nodes[0].node_id, sizeof t.nodes[0].node_id, "R1");
    t.n_nodes = 1;
    CHECK(wv_pose_mode_of(&wc, &t, 0) == WV_MODE_HELD);
    snprintf(t.nodes[0].pose_src, sizeof t.nodes[0].pose_src, "heading");
    CHECK(wv_pose_mode_of(&wc, &t, 0) == WV_MODE_HELD);
#undef R1MSG
}

int main(void)
{
    test_json();
    test_decode();
    test_decode_v2();
    test_decode_phone_smoothing();
    test_smoothing();
    test_geometry();
    test_fusion();
    test_pose();
    test_triangulate();
    test_fusion_poses();
    test_tri_weighting();
    test_range_smoothing();
    test_close_people();
    test_tracks_roundtrip();
    test_config();
    test_skv2_header();
    test_skv2_stream();
    test_jpeg_decode();
    test_video_scale();
    test_video_state();
    test_video_rx_tcp();
    test_video_rx_why();
    test_latency();
    test_ppm_note();
    test_hud_render_drawn();
    test_video_snapshot_timing();
    test_video_bursty();
    test_video_pinned_node();
    test_pts_decode();
    test_tracks_pts();
    test_skel_local();
    test_skel_smooth();
    test_skel_fusion();
    test_skel_render();
    test_plan_primitives();
    test_plan_render();
    test_anim_alert();
    test_anim_person();
    test_anim_map();
    test_anim_switch();
    test_anim_handoff();
    test_colour_hysteresis();
    test_posture();
    test_calibration_and_freeze();
    test_moving_wearer_hold();
    test_r2_pinned_left();
    test_tag_flip_replay();
    test_xray_height();
    test_tag_reacquire();
    test_projection();
    test_occlusion();
    test_wearer_render_mirror();
    test_behind_wall_and_live_pose();
    test_tag_pose_hold_and_gate();
    test_tag_frame_transform();
    test_tag_frame_fusion();
    test_decode_tag_id();
    test_multi_tag();
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}
