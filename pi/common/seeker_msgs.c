#include "seeker_msgs.h"
#include "seeker_json.h"
#include "seeker_posture.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* Any 8192-byte datagram fits: a token takes at least two bytes ("0,"). 16 contacts with pts are
 * about 1300. Static, not on the stack (64 KB): each process decodes on one thread only. */
#define MAX_TOKS (SEEKER_MAX_DATAGRAM / 2)

static int fail(char *err, int errsz, const char *why)
{
    if (err && errsz > 0)
        snprintf(err, (size_t)errsz, "%s", why);
    return -1;
}

static int parse_box(const char *s, const sj_tok *t, int arr, float box[4])
{
    if (arr < 0 || t[arr].type != SJ_ARR || t[arr].size != 4)
        return 0;
    float sum = 0;
    for (int k = 0; k < 4; k++) {
        int it = sj_index(t, arr, k);
        if (it < 0 || t[it].type != SJ_NUM)
            return 0;
        box[k] = (float)sj_num(s, t, it, 0);
        sum += box[k] < 0 ? -box[k] : box[k];
    }
    return sum > 0 && box[2] > 0 && box[3] > 0;
}

/* pts: 51 numbers, x and y divided by xy_div, visibility by v_div (clamped to 0..1). */
static int parse_pts(const char *s, const sj_tok *t, int arr, float pts[SEEKER_N_JOINTS][3],
                     float xy_div, float v_div)
{
    if (arr < 0 || t[arr].type != SJ_ARR || t[arr].size != SEEKER_N_JOINTS * 3)
        return 0;
    int it = arr + 1;
    for (int j = 0; j < SEEKER_N_JOINTS; j++)
        for (int q = 0; q < 3; q++, it = t[it].next) {
            if (it <= 0 || t[it].type != SJ_NUM)
                return 0;
            float v = (float)sj_num(s, t, it, 0);
            if (q < 2) {
                pts[j][q] = v / xy_div;
            } else {
                v /= v_div;
                pts[j][q] = v < 0 ? 0 : v > 1 ? 1 : v;
            }
        }
    return 1;
}

int seeker_decode_remote_contacts(const char *buf, int len, seeker_remote_contacts *out,
                                  char *err, int errsz)
{
    static sj_tok t[MAX_TOKS];
    memset(out, 0, sizeof *out);
    if (sj_parse(buf, len, t, MAX_TOKS) < 0 || t[0].type != SJ_OBJ)
        return fail(err, errsz, "not a JSON object");
    if (!sj_str_eq(buf, t, sj_get(buf, t, 0, "type"), "remote_contacts"))
        return fail(err, errsz, "type is not remote_contacts");
    int v = sj_get(buf, t, 0, "v");
    double ver = sj_num(buf, t, v, 1);
    if (ver != 1 && ver != 2) {
        char why[48];
        snprintf(why, sizeof why, "unsupported v %g (want 1 or 2)", ver);
        return fail(err, errsz, why);
    }
    out->v = (int)ver;
    if (sj_str(buf, t, sj_get(buf, t, 0, "node_id"), out->node_id, sizeof out->node_id) != 0 ||
        out->node_id[0] == '\0')
        return fail(err, errsz, "missing or bad node_id");
    int ts = sj_get(buf, t, 0, "t_mono_ms");
    if (sj_is_null(t, ts))
        ts = sj_get(buf, t, 0, "ts_ms"); /* v2 */
    out->t_mono_ms = (int64_t)sj_num(buf, t, ts, 0);

    /* Accept full or partial pose (e.g. x_m,y_m from ?pose= while yaw_deg is still null). */
    int pose = sj_get(buf, t, 0, "pose");
    if (pose >= 0 && t[pose].type == SJ_OBJ) {
        int px = sj_get(buf, t, pose, "x_m"), py = sj_get(buf, t, pose, "y_m");
        int pyaw = sj_get(buf, t, pose, "yaw_deg");
        if (px >= 0 && t[px].type == SJ_NUM && py >= 0 && t[py].type == SJ_NUM) {
            out->has_xy = 1;
            out->x_m = (float)sj_num(buf, t, px, 0);
            out->y_m = (float)sj_num(buf, t, py, 0);
        }
        if (pyaw >= 0 && t[pyaw].type == SJ_NUM) {
            out->has_yaw = 1;
            out->yaw_deg = (float)sj_num(buf, t, pyaw, 0);
        }
        out->has_pose = out->has_xy && out->has_yaw;
        int ptag = sj_get(buf, t, pose, "tag_id");
        if (ptag >= 0 && t[ptag].type == SJ_NUM) {
            double id = sj_num(buf, t, ptag, -1);
            if (id >= 0 && id <= 1000000 && id == (double)(int)id) {
                out->has_tag_id = 1;
                out->tag_id = (int)id;
            }
        }
    }
    int pp = pose >= 0 && t[pose].type == SJ_OBJ ? sj_get(buf, t, pose, "pitch_deg") : -1;
    if (pp < 0)
        pp = sj_get(buf, t, 0, "pitch_deg");
    if (pp >= 0 && t[pp].type == SJ_NUM) {
        float pd = (float)sj_num(buf, t, pp, 0);
        if (pd >= -90.0f && pd <= 90.0f) {
            out->has_pitch = 1;
            out->pitch_deg = pd;
        }
    }
    int hd = sj_get(buf, t, 0, "heading_deg");
    if (hd >= 0 && t[hd].type == SJ_NUM) {
        out->has_heading = 1;
        out->heading_deg = (float)sj_num(buf, t, hd, 0);
    }
    /* v2 frame info; 0 when missing or null. */
    out->width = (int)sj_num(buf, t, sj_get(buf, t, 0, "width"), 0);
    out->height = (int)sj_num(buf, t, sj_get(buf, t, 0, "height"), 0);
    out->hfov_deg = (float)sj_num(buf, t, sj_get(buf, t, 0, "hfov_deg"), 0);

    int arr = sj_get(buf, t, 0, "contacts");
    if (arr >= 0 && t[arr].type == SJ_NULL)
        return 0;
    if (arr < 0 || t[arr].type != SJ_ARR)
        return fail(err, errsz, "missing or bad contacts (want a list)");
    for (int i = 0; i < t[arr].size && out->n < SEEKER_MAX_CONTACTS; i++) {
        int c = sj_index(t, arr, i);
        if (c < 0 || t[c].type != SJ_OBJ)
            continue;
        int b = sj_get(buf, t, c, "bearing_deg");
        if (b < 0 || t[b].type != SJ_NUM)
            continue; /* a contact without a bearing cannot be placed */
        seeker_contact *k = &out->c[out->n];
        k->bearing_deg = (float)sj_num(buf, t, b, 0);
        if (sj_str(buf, t, sj_get(buf, t, c, "id"), k->id, sizeof k->id) != 0 || !k->id[0]) {
            char node[SEEKER_NODE_LEN];
            memcpy(node, out->node_id, sizeof node);
            snprintf(k->id, sizeof k->id, "%s-%d", node, out->n + 1);
        }
        if (sj_str(buf, t, sj_get(buf, t, c, "cls"), k->cls, sizeof k->cls) != 0 || !k->cls[0])
            snprintf(k->cls, sizeof k->cls, "person");
        k->conf = (float)sj_num(buf, t, sj_get(buf, t, c, "conf"), 0);
        k->has_box = parse_box(buf, t, sj_get(buf, t, c, "box"), k->box);
        int r = sj_get(buf, t, c, "range_m");
        if (!sj_is_null(t, r) && t[r].type == SJ_NUM) {
            k->range_m = (float)sj_num(buf, t, r, 0);
            k->has_range = k->range_m > 0;
        }
        k->has_pts = parse_pts(buf, t, sj_get(buf, t, c, "pts"), k->pts, 1000.0f, 100.0f);
        out->n++;
    }
    return 0;
}

/* ---- tracks encode ---- */

typedef struct {
    char *b;
    int sz, n, err;
} jw;

static void jw_put(jw *w, const char *fmt, ...) __attribute__((format(printf, 2, 3)));
static void jw_put(jw *w, const char *fmt, ...)
{
    if (w->err)
        return;
    va_list ap;
    va_start(ap, fmt);
    int k = vsnprintf(w->b + w->n, (size_t)(w->sz - w->n), fmt, ap);
    va_end(ap);
    if (k < 0 || k >= w->sz - w->n)
        w->err = 1;
    else
        w->n += k;
}

static void jw_str(jw *w, const char *key, const char *val)
{
    jw_put(w, "\"%s\":\"", key);
    for (const char *p = val; *p && !w->err; p++) {
        unsigned char c = (unsigned char)*p;
        if (c == '"' || c == '\\')
            jw_put(w, "\\%c", c);
        else if (c < 0x20)
            jw_put(w, "\\u%04x", c);
        else
            jw_put(w, "%c", c);
    }
    jw_put(w, "\"");
}

static void jw_numn(jw *w, const char *key, int has, float v)
{
    if (has)
        jw_put(w, "\"%s\":%.2f", key, v);
    else
        jw_put(w, "\"%s\":null", key);
}

static void jw_box(jw *w, const char *key, int has, const float b[4])
{
    if (has)
        jw_put(w, "\"%s\":[%.0f,%.0f,%.0f,%.0f]", key, b[0], b[1], b[2], b[3]);
    else
        jw_put(w, "\"%s\":null", key);
}

static void encode_track(jw *w, const seeker_track *k, int with_pts)
{
    jw_put(w, "{\"id\":%d,", k->id);
    jw_str(w, "label", k->label); jw_put(w, ",");
    jw_str(w, "cls", k->cls); jw_put(w, ",");
    jw_str(w, "src", k->src); jw_put(w, ",");
    jw_put(w, "\"visible\":%s,\"box\":null,", k->visible ? "true" : "false");
    jw_numn(w, "range_m", k->has_range, k->range_m); jw_put(w, ",");
    if (k->has_range)
        jw_str(w, "range_src", k->range_src);
    else
        jw_put(w, "\"range_src\":null");
    jw_put(w, ",\"range_conf\":%.2f,", k->range_conf);
    jw_numn(w, "bearing_deg", k->has_bearing, k->bearing_deg); jw_put(w, ",");
    jw_put(w, "\"elev_deg\":%.1f,\"closing_mps\":%.2f,", k->elev_deg, k->closing_mps);
    jw_str(w, "colour", k->colour);
    jw_put(w, ",\"age_ms\":%d,", k->age_ms);
    jw_str(w, "node_id", k->node_id); jw_put(w, ",");
    jw_str(w, "nodes", k->nodes);
    jw_put(w, ",\"node_bearing_deg\":%.1f,", k->node_bearing_deg);
    jw_numn(w, "room_x_m", k->has_room, k->room_x_m); jw_put(w, ",");
    jw_numn(w, "room_y_m", k->has_room, k->room_y_m); jw_put(w, ",");
    jw_box(w, "node_box", k->has_node_box, k->node_box);
    if (k->posture > SEEKER_POSTURE_UNKNOWN && k->posture <= SEEKER_POSTURE_DOWN)
        jw_put(w, ",\"posture\":\"%s\"", seeker_posture_name(k->posture));
    if (with_pts && k->has_pts) {
        jw_put(w, ",\"pts\":[");
        for (int j = 0; j < SEEKER_N_JOINTS; j++)
            jw_put(w, "%s%.0f,%.0f,%.0f", j ? "," : "", k->pts[j][0] * 1000.0f,
                   k->pts[j][1] * 1000.0f, k->pts[j][2] * 100.0f);
        jw_put(w, "]");
    }
    jw_put(w, "}");
}

static int encode_tracks_with(const seeker_tracks *m, const int *with_pts, char *buf, int bufsz)
{
    jw w = { buf, bufsz, 0, 0 };
    jw_put(&w, "{\"type\":\"tracks\",\"v\":1,");
    jw_str(&w, "src", m->src);
    jw_put(&w, ",\"t_mono_ms\":%lld,\"basis_frame_seq\":%lld,", (long long)m->t_mono_ms,
           (long long)m->basis_frame_seq);
    jw_numn(&w, "heading_deg", m->has_heading, m->heading_deg); jw_put(&w, ",");
    jw_str(&w, "heading_src", m->heading_src);
    jw_put(&w, ",\"wearer_yaw_deg\":%.1f,", m->wearer_yaw_deg);
    if (m->wearer_node[0]) {
        jw_str(&w, "wearer_node", m->wearer_node);
        jw_put(&w, ",");
    }
    jw_put(&w, "\"tracks\":[");
    for (int i = 0; i < m->n; i++) {
        if (i)
            jw_put(&w, ",");
        encode_track(&w, &m->tracks[i], with_pts[i]);
    }
    jw_put(&w, "],\"nodes\":[");
    for (int i = 0; i < m->n_nodes; i++) {
        const seeker_node_status *s = &m->nodes[i];
        if (i)
            jw_put(&w, ",");
        jw_put(&w, "{");
        jw_str(&w, "node_id", s->node_id); jw_put(&w, ",");
        jw_str(&w, "state", s->state);
        jw_put(&w, ",\"age_ms\":%d,\"x_m\":%.2f,\"y_m\":%.2f,\"yaw_deg\":%.1f,\"hfov_deg\":%.1f,"
                   "\"n_contacts\":%d,\"rate_hz\":%.1f",
               s->age_ms, s->x_m, s->y_m, s->yaw_deg, s->hfov_deg, s->n_contacts, s->rate_hz);
        if (s->pose_src[0]) {
            jw_put(&w, ",");
            jw_str(&w, "pose_src", s->pose_src);
        }
        if (s->has_pitch)
            jw_put(&w, ",\"pitch_deg\":%.1f", s->pitch_deg);
        jw_put(&w, "}");
    }
    jw_put(&w, "]}");
    return w.err ? -1 : w.n;
}

/* Skeletons are optional: at most SEEKER_PTS_MAX_CARRIERS tracks carry pts, single-node tracks
 * first (a person both phones see is SEEN by the wearer, no ghost needed), and pts are shed one
 * carrier at a time, never a track, until the datagram fits. */
int seeker_encode_tracks(const seeker_tracks *m, char *buf, int bufsz)
{
    int order[SEEKER_MAX_TRACKS], no = 0;
    for (int pass = 0; pass < 2; pass++)
        for (int i = 0; i < m->n && i < SEEKER_MAX_TRACKS; i++)
            if (m->tracks[i].has_pts && (strchr(m->tracks[i].nodes, '+') != NULL) == pass)
                order[no++] = i;
    if (no > SEEKER_PTS_MAX_CARRIERS)
        no = SEEKER_PTS_MAX_CARRIERS;
    for (int nc = no;; nc--) {
        int with_pts[SEEKER_MAX_TRACKS] = { 0 };
        for (int c = 0; c < nc; c++)
            with_pts[order[c]] = 1;
        int n = encode_tracks_with(m, with_pts, buf, bufsz);
        if (n >= 0 || nc == 0)
            return n;
    }
}

/* ---- tracks decode (hud_display side) ---- */

static float numf(const char *s, const sj_tok *t, int obj, const char *key, float def)
{
    return (float)sj_num(s, t, sj_get(s, t, obj, key), def);
}

static int numn(const char *s, const sj_tok *t, int obj, const char *key, float *out)
{
    int i = sj_get(s, t, obj, key);
    if (sj_is_null(t, i) || t[i].type != SJ_NUM)
        return 0;
    *out = (float)sj_num(s, t, i, 0);
    return 1;
}

static void strf(const char *s, const sj_tok *t, int obj, const char *key, char *out, int sz)
{
    sj_str(s, t, sj_get(s, t, obj, key), out, sz);
}

int seeker_decode_tracks(const char *buf, int len, seeker_tracks *out, char *err, int errsz)
{
    static sj_tok t[MAX_TOKS];
    memset(out, 0, sizeof *out);
    if (sj_parse(buf, len, t, MAX_TOKS) < 0 || t[0].type != SJ_OBJ)
        return fail(err, errsz, "not a JSON object");
    if (!sj_str_eq(buf, t, sj_get(buf, t, 0, "type"), "tracks"))
        return fail(err, errsz, "type is not tracks");
    strf(buf, t, 0, "src", out->src, sizeof out->src);
    out->t_mono_ms = (int64_t)sj_num(buf, t, sj_get(buf, t, 0, "t_mono_ms"), 0);
    out->basis_frame_seq = (int64_t)sj_num(buf, t, sj_get(buf, t, 0, "basis_frame_seq"), 0);
    out->has_heading = numn(buf, t, 0, "heading_deg", &out->heading_deg);
    strf(buf, t, 0, "heading_src", out->heading_src, sizeof out->heading_src);
    out->wearer_yaw_deg = numf(buf, t, 0, "wearer_yaw_deg", 0);
    strf(buf, t, 0, "wearer_node", out->wearer_node, sizeof out->wearer_node);

    int arr = sj_get(buf, t, 0, "tracks");
    if (arr < 0 || t[arr].type != SJ_ARR)
        return fail(err, errsz, "missing tracks list");
    for (int i = 0; i < t[arr].size && out->n < SEEKER_MAX_TRACKS; i++) {
        int o = sj_index(t, arr, i);
        if (o < 0 || t[o].type != SJ_OBJ)
            continue;
        seeker_track *k = &out->tracks[out->n++];
        k->id = (int)numf(buf, t, o, "id", 0);
        strf(buf, t, o, "label", k->label, sizeof k->label);
        strf(buf, t, o, "cls", k->cls, sizeof k->cls);
        strf(buf, t, o, "src", k->src, sizeof k->src);
        k->visible = sj_bool(buf, t, sj_get(buf, t, o, "visible"), 0);
        k->has_range = numn(buf, t, o, "range_m", &k->range_m);
        strf(buf, t, o, "range_src", k->range_src, sizeof k->range_src);
        k->range_conf = numf(buf, t, o, "range_conf", 0);
        k->has_bearing = numn(buf, t, o, "bearing_deg", &k->bearing_deg);
        k->elev_deg = numf(buf, t, o, "elev_deg", 0);
        k->closing_mps = numf(buf, t, o, "closing_mps", 0);
        strf(buf, t, o, "colour", k->colour, sizeof k->colour);
        k->age_ms = (int)numf(buf, t, o, "age_ms", 0);
        strf(buf, t, o, "node_id", k->node_id, sizeof k->node_id);
        strf(buf, t, o, "nodes", k->nodes, sizeof k->nodes);
        k->node_bearing_deg = numf(buf, t, o, "node_bearing_deg", 0);
        k->has_room = numn(buf, t, o, "room_x_m", &k->room_x_m) &&
                      numn(buf, t, o, "room_y_m", &k->room_y_m);
        k->has_node_box = parse_box(buf, t, sj_get(buf, t, o, "node_box"), k->node_box);
        k->has_pts = parse_pts(buf, t, sj_get(buf, t, o, "pts"), k->pts, 1000.0f, 100.0f);
        char post[12] = "";
        strf(buf, t, o, "posture", post, sizeof post);
        k->posture = post[0] && seeker_posture_parse(post) > 0 ? seeker_posture_parse(post)
                                                               : SEEKER_POSTURE_UNKNOWN;
    }
    int nodes = sj_get(buf, t, 0, "nodes");
    for (int i = 0; nodes >= 0 && t[nodes].type == SJ_ARR && i < t[nodes].size &&
                    out->n_nodes < SEEKER_MAX_NODES;
         i++) {
        int o = sj_index(t, nodes, i);
        if (o < 0 || t[o].type != SJ_OBJ)
            continue;
        seeker_node_status *s = &out->nodes[out->n_nodes++];
        strf(buf, t, o, "node_id", s->node_id, sizeof s->node_id);
        strf(buf, t, o, "state", s->state, sizeof s->state);
        s->age_ms = (int)numf(buf, t, o, "age_ms", -1);
        s->x_m = numf(buf, t, o, "x_m", 0);
        s->y_m = numf(buf, t, o, "y_m", 0);
        s->yaw_deg = numf(buf, t, o, "yaw_deg", 0);
        s->hfov_deg = numf(buf, t, o, "hfov_deg", 60);
        s->n_contacts = (int)numf(buf, t, o, "n_contacts", 0);
        s->rate_hz = numf(buf, t, o, "rate_hz", 0);
        strf(buf, t, o, "pose_src", s->pose_src, sizeof s->pose_src);
        s->has_pitch = numn(buf, t, o, "pitch_deg", &s->pitch_deg);
    }
    return 0;
}
