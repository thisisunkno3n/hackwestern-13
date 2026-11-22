#include "wearer_view.h"
#include "anim.h"
#include "anim_alert.h"
#include "plan.h"
#include "anim_person.h"
#include "seeker_json.h"
#include "seeker_time.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEG2RAD 0.017453292519943295f
#define CFG_MAX_BYTES (64 * 1024)
#define CFG_MAX_TOKS 8192

/* ---- config ---- */

void wv_config_defaults(wv_config *w)
{
    memset(w, 0, sizeof *w);
    snprintf(w->node, sizeof w->node, "W");
    w->cam.hfov_deg = 66.0f;
    w->cam.image_w = 1280;
    w->cam.image_h = 720;
    w->cam.cam_h_m = 1.7f;
    w->pitch_auto = 1;
    w->pose_src = WV_POSE_AUTO;
    w->assoc_deg = 6.0f;
    w->assoc_range_m = 1.5f;
    w->yaw_alpha = 0.35f;
    w->inset = 1;
    w->handoff = 1;
}

int wv_parse_pose(const char *s, char *node, int nodesz, wv_pose *out)
{
    if (!s)
        return -1;
    char buf[96], id[SEEKER_NODE_LEN];
    int has_id = strchr(s, '=') != NULL;
    if (snprintf(buf, sizeof buf, "%s%s", has_id ? "" : "W=", s) >= (int)sizeof buf)
        return -1;
    seeker_pose p;
    if (seeker_pose_parse(buf, id, &p, NULL, 0) != 0)
        return -1;
    if (has_id && node && nodesz > 0)
        snprintf(node, (size_t)nodesz, "%s", id);
    *out = p;
    return 0;
}

int wv_parse_wall(const char *s, float out[4])
{
    char extra;
    if (!s || sscanf(s, "%f,%f,%f,%f%c", &out[0], &out[1], &out[2], &out[3], &extra) != 4)
        return -1;
    return 0;
}

int wv_parse_pose_src(const char *s, wv_pose_src *out)
{
    if (!s)
        return -1;
    if (!strcmp(s, "auto"))
        *out = WV_POSE_AUTO;
    else if (!strcmp(s, "static") || !strcmp(s, "config"))
        *out = WV_POSE_STATIC;
    else if (!strcmp(s, "fusion") || !strcmp(s, "live"))
        *out = WV_POSE_FUSION;
    else
        return -1;
    return 0;
}

static void jf(const char *s, const sj_tok *t, int o, const char *k, float *v)
{
    *v = (float)sj_num(s, t, sj_get(s, t, o, k), *v);
}

static void ji(const char *s, const sj_tok *t, int o, const char *k, int *v)
{
    *v = (int)sj_num(s, t, sj_get(s, t, o, k), *v);
}

static void file_overlay(wv_config *w, const char *s, const sj_tok *t, int o)
{
    int p = sj_get(s, t, o, "pose");
    if (p >= 0 && t[p].type == SJ_ARR && t[p].size == 3) {
        w->pose.x_m = (float)sj_num(s, t, sj_index(t, p, 0), w->pose.x_m);
        w->pose.y_m = (float)sj_num(s, t, sj_index(t, p, 1), w->pose.y_m);
        w->pose.yaw_deg = (float)sj_num(s, t, sj_index(t, p, 2), w->pose.yaw_deg);
    } else if (p >= 0 && t[p].type == SJ_OBJ) {
        jf(s, t, p, "x_m", &w->pose.x_m);
        jf(s, t, p, "y_m", &w->pose.y_m);
        jf(s, t, p, "yaw_deg", &w->pose.yaw_deg);
    }
    char src[16] = "";
    int i = sj_get(s, t, o, "pose_source");
    if (i >= 0 && t[i].type == SJ_STR && sj_str(s, t, i, src, sizeof src) == 0 &&
        wv_parse_pose_src(src, &w->pose_src) != 0)
        fprintf(stderr, "[hud_display] display.wearer.pose_source %s unknown, using auto\n", src);
    jf(s, t, o, "hfov_deg", &w->cam.hfov_deg);
    ji(s, t, o, "image_w", &w->cam.image_w);
    ji(s, t, o, "image_h", &w->cam.image_h);
    jf(s, t, o, "cam_h_m", &w->cam.cam_h_m);
    jf(s, t, o, "pitch_deg", &w->cam.pitch_deg);
    jf(s, t, o, "person_h_m", &w->cam.person_h_m);
    jf(s, t, o, "xray_up_deg", &w->xray_up_deg);
    w->pitch_auto = sj_bool(s, t, sj_get(s, t, o, "pitch_auto"), w->pitch_auto);
    jf(s, t, o, "assoc_deg", &w->assoc_deg);
    jf(s, t, o, "assoc_range_m", &w->assoc_range_m);
    jf(s, t, o, "yaw_alpha", &w->yaw_alpha);
    w->visor = sj_bool(s, t, sj_get(s, t, o, "visor"), w->visor);
    char fb[16] = "";
    i = sj_get(s, t, o, "video_fallback");
    if (i >= 0 && t[i].type == SJ_STR && sj_str(s, t, i, fb, sizeof fb) == 0)
        w->radar_fallback = !strcmp(fb, "radar");
    w->inset = sj_bool(s, t, sj_get(s, t, o, "inset"), w->inset);
    int walls = sj_get(s, t, o, "walls");
    for (int k = 0; walls >= 0 && t[walls].type == SJ_ARR && k < t[walls].size &&
                    w->n_walls < WV_MAX_WALLS;
         k++) {
        int a = sj_index(t, walls, k);
        if (a < 0 || t[a].type != SJ_ARR || t[a].size != 4)
            continue;
        for (int j = 0; j < 4; j++)
            w->walls[w->n_walls][j] = (float)sj_num(s, t, sj_index(t, a, j), 0);
        w->n_walls++;
    }
}

void wv_config_load(wv_config *w, const seeker_config *cfg, const char *path, const char *node)
{
    wv_config_defaults(w);
    if (cfg && cfg->view_node[0]) /* the video source is the wearer's eyes unless told otherwise */
        snprintf(w->node, sizeof w->node, "%s", cfg->view_node);
    char *buf = NULL;
    sj_tok *t = NULL;
    int wobj = -1;
    FILE *f = path ? fopen(path, "rb") : NULL;
    if (f) {
        buf = malloc(CFG_MAX_BYTES);
        t = malloc(sizeof(sj_tok) * CFG_MAX_TOKS);
        int n = buf ? (int)fread(buf, 1, CFG_MAX_BYTES, f) : 0;
        if (t && n > 0 && n < CFG_MAX_BYTES && sj_parse(buf, n, t, CFG_MAX_TOKS) > 0 &&
            t[0].type == SJ_OBJ) {
            int d = sj_get(buf, t, 0, "display");
            wobj = d >= 0 ? sj_get(buf, t, d, "wearer") : -1;
            if (wobj >= 0 && t[wobj].type != SJ_OBJ)
                wobj = -1;
        }
        fclose(f);
    }
    if (wobj >= 0) {
        w->from_config = 1;
        int i = sj_get(buf, t, wobj, "node");
        if (i >= 0 && t[i].type == SJ_STR)
            sj_str(buf, t, i, w->node, sizeof w->node);
    }
    if (node && node[0])
        snprintf(w->node, sizeof w->node, "%s", node);
    const seeker_node_cfg *nc = cfg ? seeker_config_node(cfg, w->node) : NULL;
    if (nc) {
        w->pose.x_m = nc->x_m;
        w->pose.y_m = nc->y_m;
        w->pose.yaw_deg = nc->yaw_deg;
        if (nc->hfov_deg > 0)
            w->cam.hfov_deg = nc->hfov_deg;
        if (nc->image_w > 0 && nc->image_h > 0) {
            w->cam.image_w = nc->image_w;
            w->cam.image_h = nc->image_h;
        }
    }
    if (wobj >= 0)
        file_overlay(w, buf, t, wobj);
    free(buf);
    free(t);
}

/* ---- classification ---- */

int wv_seen_by(const char *nodes, const char *primary, const char *wearer)
{
    if (!wearer || !wearer[0])
        return 0;
    if (primary && !strcmp(primary, wearer))
        return 1;
    size_t wl = strlen(wearer);
    for (const char *p = nodes; p && *p;) {
        const char *e = strchr(p, '+');
        size_t n = e ? (size_t)(e - p) : strlen(p);
        if (n == wl && !strncmp(p, wearer, wl))
            return 1;
        if (!e)
            break;
        p = e + 1;
    }
    return 0;
}

static const seeker_node_status *find_node(const seeker_tracks *t, const char *id)
{
    for (int i = 0; t && i < t->n_nodes; i++)
        if (!strcmp(t->nodes[i].node_id, id))
            return &t->nodes[i];
    return NULL;
}

static float orient2d(float ax, float ay, float bx, float by, float cx, float cy)
{
    return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

static int seg_crosses(float ax, float ay, float bx, float by, const float w[4])
{
    float o1 = orient2d(ax, ay, bx, by, w[0], w[1]);
    float o2 = orient2d(ax, ay, bx, by, w[2], w[3]);
    float o3 = orient2d(w[0], w[1], w[2], w[3], ax, ay);
    float o4 = orient2d(w[0], w[1], w[2], w[3], bx, by);
    return (o1 * o2 < 0.0f) && (o3 * o4 < 0.0f);
}

static int behind_any_wall(const wv_config *w, const wv_pose *pose, float px, float py)
{
    for (int i = 0; i < w->n_walls; i++)
        if (seg_crosses(pose->x_m, pose->y_m, px, py, w->walls[i]))
            return 1;
    return 0;
}

void wv_resolve_pose(const wv_config *w, const seeker_tracks *t, wv_pose *out, int *live,
                     char *state, int statesz)
{
    *out = w->pose;
    *live = 0;
    const seeker_node_status *n = find_node(t, w->node);
    if (state && statesz > 0)
        snprintf(state, (size_t)statesz, "%s", n ? n->state : "");
    if (!n || w->pose_src == WV_POSE_STATIC)
        return;
    if (w->pose_src == WV_POSE_AUTO && !strcmp(n->state, "wait"))
        return;
    out->x_m = n->x_m;
    out->y_m = n->y_m;
    out->yaw_deg = n->yaw_deg;
    if (w->pose_src == WV_POSE_FUSION || !n->pose_src[0] || !strcmp(n->pose_src, "message"))
        *live = 1;
}

wv_pose_mode wv_pose_mode_of(const wv_config *w, const seeker_tracks *t, int live)
{
    const seeker_node_status *n = find_node(t, w->node);
    if (!n || w->pose_src == WV_POSE_STATIC)
        return WV_MODE_FIXED;
    if (!strcmp(n->pose_src, "held") || !strcmp(n->pose_src, "heading") ||
        !strcmp(n->pose_src, "home"))
        return WV_MODE_HELD;
    if (live)
        return WV_MODE_LIVE;
    return !strcmp(n->pose_src, "init") ? WV_MODE_START : WV_MODE_FIXED;
}

const char *wv_pose_mode_name(wv_pose_mode m)
{
    return m == WV_MODE_LIVE ? "live" : m == WV_MODE_HELD ? "held" : m == WV_MODE_START ? "start"
                                                                                         : "fixed";
}

int wv_classify(const seeker_tracks *t, const wv_config *w, const wv_pose *pose, wv_item *items,
                int max)
{
    int n = 0;
    for (int i = 0; t && i < t->n && n < max; i++) {
        const seeker_track *k = &t->tracks[i];
        wv_item *it = &items[n++];
        memset(it, 0, sizeof *it);
        it->idx = i;
        it->dup_of = -1;
        it->fade = 1.0f;
        it->direct = wv_seen_by(k->nodes, k->node_id, w->node);
        if (k->has_room) {
            wv_project_point(pose, &w->cam, k->room_x_m, k->room_y_m, &it->p);
            it->has_proj = it->has_bearing = it->has_range = 1;
            it->bearing_deg = it->p.bearing_deg;
            it->range_m = it->p.range_m;
            it->in_fov = it->p.in_fov;
            it->behind_wall = behind_any_wall(w, pose, k->room_x_m, k->room_y_m);
        } else if (!strcmp(k->node_id, w->node)) {
            it->has_bearing = 1; /* the helmet phone's own bearing, no range */
            it->bearing_deg = wv_wrap180(k->node_bearing_deg);
            it->in_fov = wv_bearing_in_fov(&w->cam, it->bearing_deg);
        } else {
            /* Single-node contact with range_m null from another phone: place its bearing ray. */
            const seeker_node_status *sn = find_node(t, k->node_id);
            if (sn) {
                float theta = (sn->yaw_deg + k->node_bearing_deg) * DEG2RAD;
                float ux = sinf(theta), uy = cosf(theta);
                int in_n = 0, bw_n = 0;
                float in_u[32], in_v[32], in_b[32];
                int in_bw[32];
                float best_front_d = 999.0f, best_front_b = 0.0f;
                int have_front = 0, best_front_bw = 0;
                for (int step = 0; step < 31; step++) {
                    float d = 0.5f + 0.25f * (float)step;
                    float px = sn->x_m + d * ux, py = sn->y_m + d * uy;
                    wv_proj sp;
                    wv_project_point(pose, &w->cam, px, py, &sp);
                    int bw = behind_any_wall(w, pose, px, py);
                    if (sp.in_fov && in_n < 32) {
                        float vv = sp.box_y + sp.box_h * 0.5f;
                        if (vv < 0.0f) vv = 0.0f;
                        if (vv >= (float)w->cam.image_h) vv = (float)(w->cam.image_h - 1);
                        in_u[in_n] = sp.u;
                        in_v[in_n] = vv;
                        in_b[in_n] = sp.bearing_deg;
                        in_bw[in_n] = bw;
                        if (bw) bw_n++;
                        in_n++;
                    }
                    if (sp.in_front) {
                        float ad = fabsf(sp.bearing_deg);
                        if (!have_front || ad < best_front_d) {
                            have_front = 1;
                            best_front_d = ad;
                            best_front_b = sp.bearing_deg;
                            best_front_bw = bw;
                        }
                    } else if (!have_front && step == 8) {
                        best_front_b = sp.bearing_deg;
                        best_front_bw = bw;
                    }
                }
                if (in_n > 0) {
                    int pick[32], pn = 0;
                    for (int j = 0; j < in_n; j++)
                        if (bw_n == 0 || in_bw[j])
                            pick[pn++] = j;
                    int first = pick[0], mid = pick[pn / 2], last = pick[pn - 1];
                    it->has_bearing = 1;
                    it->in_fov = 1;
                    it->bearing_deg = in_b[mid];
                    it->behind_wall = bw_n > 0;
                    it->has_ray_span = 1;
                    it->ray_u0 = in_u[first];
                    it->ray_v0 = in_v[first];
                    it->ray_u1 = in_u[last];
                    it->ray_v1 = in_v[last];
                } else {
                    it->has_bearing = 1;
                    it->in_fov = 0;
                    it->bearing_deg = best_front_b;
                    it->behind_wall = best_front_bw;
                }
            }
        }
    }
    /* Fusion merges nodes only when both give a range; a helmet sighting without range and the
     * same person from R1 arrive as two tracks. Keep the helmet one and lend it R1's range. A
     * second person straight behind the first (range far apart), or across a wall, stays x-ray. */
    for (int a = 0; a < n; a++) {
        wv_item *o = &items[a];
        if (o->direct || !o->has_bearing)
            continue;
        int best = -1;
        float best_d = w->assoc_deg;
        for (int b = 0; b < n; b++) {
            const wv_item *d = &items[b];
            if (!d->direct || !d->has_bearing)
                continue;
            if (o->behind_wall && !d->behind_wall)
                continue;
            float db = fabsf(wv_wrap180(o->bearing_deg - d->bearing_deg));
            if (db > best_d)
                continue;
            /* Mono ranges are +-10 to 30%: with the handoff on, the gap may grow with the range
             * (35% past assoc_range_m / 0.35, 4.3 m at the default 1.5 m). */
            float tol = w->assoc_range_m;
            if (w->handoff && o->has_range && d->has_range)
                tol *= fmaxf(1.0f, 0.35f * fmaxf(o->range_m, d->range_m) / 1.5f);
            if (o->has_range && d->has_range && fabsf(o->range_m - d->range_m) > tol)
                continue;
            best = b;
            best_d = db;
        }
        if (best < 0)
            continue;
        o->dup_of = best;
        wv_item *d = &items[best];
        if (!d->has_range && o->has_range) {
            float a2 = (pose->yaw_deg + d->bearing_deg) * DEG2RAD;
            d->has_range = 1;
            d->range_m = o->range_m;
            wv_project_point(pose, &w->cam, pose->x_m + o->range_m * sinf(a2),
                             pose->y_m + o->range_m * cosf(a2), &d->p);
            d->has_proj = 1;
            d->in_fov = d->p.in_fov;
        }
    }
    return n;
}

/* ---- drawing ---- */

typedef struct {
    canvas *c;
    const seeker_config *cfg;
    const wv_config *w;
    wv_camera cam;  /* image size follows the video frame when there is one */
    wv_viewport vp;
    int ts, lw, stale;
    int n_lab;
    int lab[64][4]; /* placed label rectangles, so labels of nearby people do not overprint */
    int n_skel;     /* x-ray people drawn as ghost skeletons */
    int n_posture[4]; /* people in view by SEEKER_POSTURE_* */
    int n_drawn;
    int drawn[SEEKER_MAX_TRACKS][7]; /* id, direct, x, y, w, h, level */
    wv_view *view;  /* per-track display state (colour, eased box) */
    int plan;       /* draw_theme() == DRAW_THEME_PLAN */
    float u;        /* plan: frame height / 720 */
    float fade_a;   /* plan marks: alpha factor (crossfade) */
    int n_fading;
    anim_fx fx;     /* plan: this person's alert animation (anim_alert.h), zero = none */
    int cur_slot;   /* plan: wv_view.colours slot of the person being labelled, -1 none */
    ap_fx pfx;      /* plan: the person being drawn's animation (anim_person.h), 0 = none */
} rc;

static int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static int to_sx(const rc *r, float u)
{
    return (int)lroundf(r->vp.x0 + u * (float)r->vp.w / (float)r->cam.image_w);
}

static int to_sy(const rc *r, float v)
{
    return (int)lroundf(r->vp.y0 + v * (float)r->vp.h / (float)r->cam.image_h);
}

static uint32_t direct_colour(const rc *r, const wv_item *it, const seeker_track *k)
{
    if (r->stale || k->age_ms > r->cfg->stale_ms)
        return C_GREY;
    if (!it->has_range || it->level < 0)
        return C_HUD;
    return it->level == SEEKER_RED ? C_RED : it->level == SEEKER_AMBER ? C_AMBER : C_GREEN;
}

/* Threat level of each seen person with a range: distance from the wearer and its trend, with
 * hysteresis and a minimum hold per colour, so a noisy range does not flicker red and amber. */
static void assign_levels(wv_view *v, const seeker_config *cfg, const seeker_tracks *t,
                          wv_item *items, int n, int64_t now)
{
    for (int i = 0; i < WV_COLOUR_SLOTS; i++)
        if (v->colours[i].id && now - v->colours[i].seen_ms > SEEKER_COLOUR_RESET_MS)
            v->colours[i].id = 0;
    for (int i = 0; i < n; i++) {
        wv_item *it = &items[i];
        it->level = -1;
        it->slot = -1;
        if (it->dup_of >= 0)
            continue;
        int id = t->tracks[it->idx].id;
        wv_colour_slot *slot = NULL, *spare = NULL;
        for (int j = 0; id > 0 && j < WV_COLOUR_SLOTS && !slot; j++) {
            if (v->colours[j].id == id)
                slot = &v->colours[j];
            else if (!v->colours[j].id && !spare)
                spare = &v->colours[j];
        }
        if (!slot && spare) {
            memset(spare, 0, sizeof *spare);
            spare->id = id;
            slot = spare;
        }
        if (slot) {
            slot->seen_ms = now;
            it->slot = (int)(slot - v->colours);
        }
        if (slot && v->cfg.handoff) {
            int raw = it->direct;
            if (raw)
                slot->direct_ms = now;
            int want = raw || (slot->shown_direct && now - slot->direct_ms <= WV_SEEN_HOLD_MS);
            if (slot->shown && want != slot->shown_direct)
                slot->xfade_ms = now;
            slot->shown = 1;
            slot->shown_direct = want;
            it->direct = want;
            it->held = want && !raw;
            v->last.n_held += it->held;
            if (slot->xfade_ms && now - slot->xfade_ms < WV_XFADE_MS)
                it->fade = (float)(now - slot->xfade_ms) / (float)WV_XFADE_MS;
        }
        if (!it->direct || !it->has_range)
            continue;
        if (slot) {
            it->level = seeker_colour_update(&slot->c, it->range_m, now, cfg);
            it->closing = slot->c.fast;
        } else { /* no track id: plain thresholds */
            it->level = it->range_m < cfg->near_m ? SEEKER_RED
                        : it->range_m < cfg->mid_m ? SEEKER_AMBER
                                                   : SEEKER_GREEN;
        }
    }
}

static uint32_t xray_colour(const rc *r, const seeker_track *k)
{
    return r->stale || k->age_ms > r->cfg->stale_ms ? C_GREY : C_CYAN;
}

static void dashed_rect(canvas *c, int x, int y, int w, int h, int thick, int dash, uint32_t col)
{
    draw_dashed(c, x, y, x + w, y, thick, dash, col);
    draw_dashed(c, x + w, y, x + w, y + h, thick, dash, col);
    draw_dashed(c, x + w, y + h, x, y + h, thick, dash, col);
    draw_dashed(c, x, y + h, x, y, thick, dash, col);
}

static void range_str(char *out, size_t n, const wv_item *it)
{
    if (it->has_range)
        snprintf(out, n, "%.1fM", it->range_m);
    else
        snprintf(out, n, "--.-M");
}

/* Screen box for an in-FOV item: the helmet phone's own detection box when it has one, else a
 * person-sized box from the projection, else a tall marker at the bearing. */
/* pitch_auto: for each person the wearer's own phone sees whole (box clear of the top and bottom
 * edges), the angle between the middle of its box and the middle of the projected one is what
 * the assumed camera height and tilt get wrong; the correction eases toward it. */
static void learn_tilt(wv_view *v, const wv_camera *cam, const seeker_tracks *t, const wv_item *items,
                       int n)
{
    float f = wv_focal_px(cam), H = (float)cam->image_h;
    if (H <= 0 || f <= 0)
        return;
    for (int i = 0; i < n; i++) {
        const wv_item *it = &items[i];
        const seeker_track *k = &t->tracks[it->idx];
        if (!it->direct || !it->has_proj || !k->has_node_box || strcmp(k->node_id, v->cfg.node) ||
            it->dup_of >= 0)
            continue;
        float top = k->node_box[1], bot = k->node_box[1] + k->node_box[3];
        if (top < 0.02f * H || bot > 0.98f * H || k->node_box[3] < 0.15f * H || it->p.box_h <= 0 ||
            it->range_m < 0.8f || it->range_m > 8.0f)
            continue;
        float v_obs = 0.5f * (top + bot), v_pred = it->p.box_y + 0.5f * it->p.box_h;
        float d = (atanf((v_obs - H * 0.5f) / f) - atanf((v_pred - H * 0.5f) / f)) * 57.29578f;
        if (fabsf(d) > 25.0f)
            continue; /* not the same person (or a wild box) */
        float fix = v->tilt_fix + 0.08f * d;
        v->tilt_fix = fix > 20.0f ? 20.0f : fix < -20.0f ? -20.0f : fix;
        v->tilt_n++;
    }
}

static void item_box(const rc *r, const wv_item *it, const seeker_track *k, int *x, int *y, int *w,
                     int *h)
{
    const wv_camera *cc = &r->cam;
    if (it->direct && k->has_node_box && !strcmp(k->node_id, r->w->node) && cc->image_w > 0 &&
        cc->image_h > 0 && k->node_box[2] > 0 && k->node_box[3] > 0 &&
        k->node_box[0] + k->node_box[2] <= cc->image_w * 1.05f &&
        k->node_box[1] + k->node_box[3] <= cc->image_h * 1.05f) {
        float sx = (float)r->vp.w / (float)cc->image_w, sy = (float)r->vp.h / (float)cc->image_h;
        *x = r->vp.x + (int)lroundf(k->node_box[0] * sx);
        *y = r->vp.y + (int)lroundf(k->node_box[1] * sy);
        *w = (int)lroundf(k->node_box[2] * sx);
        *h = (int)lroundf(k->node_box[3] * sy);
    } else if (it->has_proj) {
        *x = to_sx(r, it->p.box_x);
        *y = to_sy(r, it->p.box_y);
        *w = to_sx(r, it->p.box_x + it->p.box_w) - *x;
        *h = to_sy(r, it->p.box_y + it->p.box_h) - *y;
    } else {
        int cx = to_sx(r, wv_bearing_to_u(&r->cam, it->bearing_deg));
        *h = r->vp.h * 2 / 5;
        *w = *h / 4;
        *x = cx - *w / 2;
        *y = r->vp.y + r->vp.h / 2 - *h / 3;
    }
    int minw = 14 * r->ts, minh = 28 * r->ts;
    if (*w < minw) {
        *x -= (minw - *w) / 2;
        *w = minw;
    }
    if (*h < minh) {
        *y -= (minh - *h) / 2;
        *h = minh;
    }
    /* Keep huge (very close) boxes on screen so the raster never walks far off canvas. */
    int x1 = clampi(*x + *w, r->vp.x - 4, r->vp.x + r->vp.w + 4);
    int y1 = clampi(*y + *h, r->vp.y - 4, r->vp.y + r->vp.h + 4);
    *x = clampi(*x, r->vp.x - 4, r->vp.x + r->vp.w + 4);
    *y = clampi(*y, r->vp.y - 4, r->vp.y + r->vp.h + 4);
    *w = x1 - *x;
    *h = y1 - *y;
}

static int overlaps(const int *a, int x, int y, int w, int h)
{
    return x < a[0] + a[2] && a[0] < x + w && y < a[1] + a[3] && a[1] < y + h;
}

/* Finds a free row for a w x h label at x near y: moves up past whatever it would overprint,
 * then down if that runs off the top. Records the result. */
static int place_label(rc *r, int x, int y, int w, int h)
{
    int top = 4, bot = r->c->h - h - 4;
    for (int dir = -1; dir <= 1; dir += 2) {
        int yy = clampi(y, top, bot);
        for (int tries = 0; tries < 12 && yy >= top && yy <= bot; tries++) {
            int hit = -1;
            for (int i = 0; i < r->n_lab && hit < 0; i++)
                if (overlaps(r->lab[i], x - 3, yy - 3, w + 6, h + 6))
                    hit = i;
            if (hit < 0) {
                y = yy;
                goto found;
            }
            yy = dir < 0 ? r->lab[hit][1] - h - 4 : r->lab[hit][1] + r->lab[hit][3] + 4;
        }
    }
    y = clampi(y, top, bot);
found:
    if (r->n_lab < 64) {
        int *L = r->lab[r->n_lab++];
        L[0] = x - 3;
        L[1] = y - 3;
        L[2] = w + 6;
        L[3] = h + 6;
    }
    return y;
}

static void text_at(rc *r, int x, int y, int scale, uint32_t col, const char *s)
{
    int tw = draw_text_width(scale, s);
    y = place_label(r, x, y, tw, 7 * scale);
    draw_fill(r->c, x - 3, y - 3, tw + 6, 7 * scale + 6, C_BLACK);
    draw_text(r->c, x, y, scale, col, s);
}

static void label_at(rc *r, int cx, int y, int scale, uint32_t col, const char *s)
{
    int tw = draw_text_width(scale, s);
    text_at(r, clampi(cx - tw / 2, 4, r->c->w - tw - 4), y, scale, col, s);
}

/* Label band: below the heading tape, above the legend and hud_render's status line. */
static int tape_y(const rc *r)
{
    return r->vp.y + 10 * r->ts + 8;
}

static int readout_y(const rc *r)
{
    return tape_y(r) + 8 * r->ts + 16;
}

static int label_top(const rc *r)
{
    return readout_y(r) + 7 * r->ts + 10;
}

static int label_bottom(const rc *r)
{
    int st = r->c->h >= 600 ? 2 : 1;
    return r->c->h - 12 * st - 14 * r->ts - 16;
}

/* COCO-17 bones (seeker_msgs.h joint order). */
static const unsigned char WV_BONES[][2] = {
    { 5, 6 },  { 5, 7 },   { 7, 9 },   { 6, 8 },   { 8, 10 }, /* shoulders, arms */
    { 5, 11 }, { 6, 12 },  { 11, 12 },                       /* torso */
    { 11, 13 }, { 13, 15 }, { 12, 14 }, { 14, 16 },          /* legs */
    { 0, 5 },  { 0, 6 },   { 0, 1 },   { 0, 2 },   { 1, 3 }, { 2, 4 } /* head, neck */
};

/* Ghost skeleton for an x-ray person: the track's joints are in person-height units from the
 * mid-hip (fusion/seeker_skel.h), scaled by the projected person height, with the hip about 0.9 m
 * above the floor standing (0.45 m crouching, 0.15 m down). Returns the number of bones drawn
 * (0: the caller keeps the plain marker). */
static int draw_skeleton(rc *r, const wv_item *it, const seeker_track *k, uint32_t col)
{
    if (!k->has_pts || !it->has_proj || it->p.box_h <= 0)
        return 0;
    const wv_proj *p = &it->p;
    int sx[SEEKER_N_JOINTS], sy[SEEKER_N_JOINTS], ok[SEEKER_N_JOINTS];
    float hip_row = k->posture == SEEKER_POSTURE_DOWN     ? 0.91f
                    : k->posture == SEEKER_POSTURE_CROUCH ? 0.74f
                                                          : 0.49f;
    float hip_v = p->box_y + hip_row * p->box_h;
    int low = -1000000;
    for (int j = 0; j < SEEKER_N_JOINTS; j++) {
        sx[j] = to_sx(r, p->u + k->pts[j][0] * p->box_h);
        sy[j] = to_sy(r, hip_v + k->pts[j][1] * p->box_h);
        ok[j] = k->pts[j][2] >= 0.3f;
        if (ok[j] && sy[j] > low)
            low = sy[j];
    }
    /* A person down on the floor a few metres away is below the camera's view (1.7 m high,
     * about 25 deg half-FOV): lift the ghost to the bottom edge so it still shows. */
    int floor_y = label_bottom(r) - 4; /* above the legend and the status line */
    if (k->posture == SEEKER_POSTURE_DOWN && low > floor_y)
        for (int j = 0; j < SEEKER_N_JOINTS; j++)
            sy[j] -= low - floor_y;
    int nb = 0;
    for (size_t b = 0; b < sizeof WV_BONES / sizeof WV_BONES[0]; b++) {
        int a = WV_BONES[b][0], c = WV_BONES[b][1];
        if (ok[a] && ok[c]) {
            draw_dashed(r->c, sx[a], sy[a], sx[c], sy[c], r->lw, 6, col);
            nb++;
        }
    }
    if (nb == 0)
        return 0;
    for (int j = 0; j < SEEKER_N_JOINTS; j++)
        if (ok[j])
            draw_diamond(r->c, sx[j], sy[j], 2 + r->ts, 1, col);
    return nb;
}

/* WV_BOX_EASE: the drawn box glides toward this frame's box; a big jump (another detection, a
 * track that moved far) snaps so it never slides across the screen. */
static void ease_box(rc *r, const wv_item *it, int *x, int *y, int *w, int *h)
{
    if (it->slot < 0 || !r->view || r->view->box_ease_off)
        return;
    wv_colour_slot *s = &r->view->colours[it->slot];
    float jump = hypotf((float)*x + *w * 0.5f - (s->bx + s->bw * 0.5f),
                        (float)*y + *h * 0.5f - (s->by + s->bh * 0.5f));
    if (!s->box_valid || jump > WV_BOX_SNAP * (float)(*h > 1 ? *h : 1)) {
        s->bx = (float)*x, s->by = (float)*y, s->bw = (float)*w, s->bh = (float)*h;
        s->box_valid = 1;
    } else {
        s->bx += WV_BOX_EASE * ((float)*x - s->bx);
        s->by += WV_BOX_EASE * ((float)*y - s->by);
        s->bw += WV_BOX_EASE * ((float)*w - s->bw);
        s->bh += WV_BOX_EASE * ((float)*h - s->bh);
    }
    *x = (int)lroundf(s->bx), *y = (int)lroundf(s->by);
    *w = (int)lroundf(s->bw), *h = (int)lroundf(s->bh);
}

static void draw_in_view(rc *r, const wv_item *it, const seeker_track *k)
{
    int x, y, w, h;
    item_box(r, it, k, &x, &y, &w, &h);
    ease_box(r, it, &x, &y, &w, &h);
    char rs[16], via[40];
    range_str(rs, sizeof rs, it);
    int cx = x + w / 2, gap = 6 + 2 * r->ts;
    int tag_h = 7 * r->ts, rng_h = 7 * (r->ts + 1);
    uint32_t col = it->direct ? direct_colour(r, it, k) : xray_colour(r, k);
    if (r->n_drawn < SEEKER_MAX_TRACKS) {
        int *d = &r->drawn[r->n_drawn++][0];
        d[0] = k->id, d[1] = it->direct, d[2] = x, d[3] = y, d[4] = w, d[5] = h, d[6] = it->level;
    }
    if (it->direct) {
        draw_rect(r->c, x, y, w, h, r->lw + 1, col);
        draw_brackets(r->c, x - 6, y - 6, w + 12, h + 12, r->lw + 1, col);
    } else {
        if (it->has_ray_span)
            draw_dashed(r->c, to_sx(r, it->ray_u0), to_sy(r, it->ray_v0),
                        to_sx(r, it->ray_u1), to_sy(r, it->ray_v1), r->lw, 10, col);
        if (draw_skeleton(r, it, k, col) > 0) {
            dashed_rect(r->c, x, y, w, h, 1, 14, col); /* faint hull around the ghost */
            r->n_skel++;
        } else {
            dashed_rect(r->c, x, y, w, h, r->lw + 1, 10, col);
            draw_diamond(r->c, cx, y + h / 2, 6 + 3 * r->ts, r->lw, col);
        }
    }
    /* Tag above the box, or just inside its top when a close person fills the frame. */
    int tag_y = y - gap - 6 - tag_h;
    if (tag_y < label_top(r))
        tag_y = (y > label_top(r) ? y : label_top(r)) + gap;
    char tag[24];
    const char *pw = seeker_posture_word(k->posture);
    snprintf(tag, sizeof tag, "%s%s%s", it->direct ? "SEEN" : "XRAY", pw[0] ? " " : "", pw);
    label_at(r, cx, tag_y, r->ts, col, tag);
    r->n_posture[k->posture >= 0 && k->posture <= SEEKER_POSTURE_DOWN ? k->posture : 0]++;
    /* Range (and the x-ray source) below the box, pulled up inside it at the bottom edge. */
    int below_h = rng_h + (it->direct ? 0 : gap + tag_h);
    int ry = y + h + gap + 6;
    if (ry + below_h > label_bottom(r))
        ry = label_bottom(r) - below_h;
    if (ry < tag_y + tag_h + gap)
        ry = tag_y + tag_h + gap;
    label_at(r, cx, ry, r->ts + 1, col, rs);
    if (!it->direct) {
        snprintf(via, sizeof via, "VIA %s", k->nodes[0] ? k->nodes : k->node_id);
        label_at(r, cx, ry + rng_h + gap, r->ts, col, via);
    }
}

static void draw_edge(rc *r, const wv_item *it, const seeker_track *k, int slot)
{
    wv_edge e = wv_edge_for(it->bearing_deg);
    uint32_t col = it->direct ? direct_colour(r, it, k) : xray_colour(r, k);
    int s = 12 + 6 * r->ts, row = 3 * s + 8 * r->ts;
    int off = (slot % 2 ? 1 : -1) * ((slot + 1) / 2) * row;
    int ylo = label_top(r) + s, yhi = r->vp.y + r->vp.h - s - 4;
    if (r->w->inset && e.side > 0) { /* stay above the radar inset in the bottom-right corner */
        int st = r->c->h >= 600 ? 2 : 1;
        yhi = r->c->h - 2 * (r->c->h / 6) - 30 - 12 * st - s - 8 * r->ts;
    }
    int cy = clampi(r->vp.y + r->vp.h / 2 + off, ylo, yhi > ylo ? yhi : ylo);
    int tip = e.side > 0 ? r->vp.x + r->vp.w - 10 : r->vp.x + 10;
    int in = -e.side; /* arms point back into the screen */
    int th = r->lw + 1;
    for (int n = 0; n < (e.behind ? 2 : 1); n++) {
        int tx = tip + in * n * (s * 2 / 3);
        if (it->direct) {
            draw_line(r->c, tx, cy, tx + in * s, cy - s, th, col);
            draw_line(r->c, tx, cy, tx + in * s, cy + s, th, col);
        } else {
            draw_dashed(r->c, tx, cy, tx + in * s, cy - s, th, 5, col);
            draw_dashed(r->c, tx, cy, tx + in * s, cy + s, th, 5, col);
        }
    }
    char rs[16], lab[48];
    range_str(rs, sizeof rs, it);
    const char *pw = seeker_posture_word(k->posture);
    snprintf(lab, sizeof lab, "%s %.0f%s%s%s", rs, e.turn_deg, e.behind ? " BEHIND" : "",
             pw[0] ? " " : "", pw);
    const char *txt = lab;
    int tw = draw_text_width(r->ts, txt);
    int lx = e.side > 0 ? tip - s * 2 - 8 - tw : tip + s * 2 + 8;
    text_at(r, lx, cy - 4 * r->ts, r->ts, col, txt);
}

static void readout_text(char *hd, size_t n, const wv_pose *pose, wv_pose_mode mode)
{
    snprintf(hd, n, "%03.0f %s", fmodf(fmodf(pose->yaw_deg, 360.0f) + 360.0f, 360.0f),
             mode == WV_MODE_LIVE ? "LIVE" : mode == WV_MODE_HELD ? "HOLD"
                                         : mode == WV_MODE_START ? "WAIT" : "FIX");
}

static void draw_tape(const rc *r, const wv_pose *pose, wv_pose_mode mode)
{
    canvas *c = r->c;
    float half = r->cam.hfov_deg * 0.5f;
    int y0 = tape_y(r);
    int xl = to_sx(r, 0), xr = to_sx(r, (float)r->cam.image_w);
    draw_line(c, xl + 8, y0, xr - 8, y0, 1, C_HUD);
    /* Ticks at absolute headings (every 5 deg), placed through the pinhole like the targets. */
    for (int a = (int)ceilf((pose->yaw_deg - half) / 5.0f) * 5; a <= pose->yaw_deg + half; a += 5) {
        int px = to_sx(r, wv_bearing_to_u(&r->cam, (float)a - pose->yaw_deg));
        int hdg = (a % 360 + 360) % 360;
        int major = hdg % 10 == 0;
        draw_line(c, px, y0, px, y0 + (major ? 10 : 5) * r->ts / 2 + 2, 1, C_HUD);
        if (hdg % 30 == 0) {
            char lab[8];
            snprintf(lab, sizeof lab, "%03d", hdg);
            draw_text(c, px - draw_text_width(r->ts, lab) / 2, y0 - 9 * r->ts, r->ts, C_HUD, lab);
        }
    }
    int cx = r->vp.x + r->vp.w / 2;
    draw_line(c, cx, y0 + 8 * r->ts, cx - 7, y0 + 8 * r->ts + 10, r->lw, C_WHITE);
    draw_line(c, cx, y0 + 8 * r->ts, cx + 7, y0 + 8 * r->ts + 10, r->lw, C_WHITE);
    char hd[24];
    readout_text(hd, sizeof hd, pose, mode);
    int tw = draw_text_width(r->ts, hd), ry = readout_y(r);
    draw_fill(c, cx - tw / 2 - 3, ry - 3, tw + 6, 7 * r->ts + 6, C_BLACK);
    draw_text(c, cx - tw / 2, ry, r->ts, C_WHITE, hd);
}

static void inset_map(const wv_pose *p, float k, int R, int cx, int cy, float x, float y, int *sx,
                      int *sy)
{
    float a = p->yaw_deg * DEG2RAD, dx = x - p->x_m, dy = y - p->y_m;
    float fz = dx * sinf(a) + dy * cosf(a), fx = dx * cosf(a) - dy * sinf(a);
    float px = fx * k, py = fz * k, m = hypotf(px, py);
    if (m > (float)R) {
        px *= (float)R / m;
        py *= (float)R / m;
    }
    *sx = cx + (int)lroundf(px);
    *sy = cy - (int)lroundf(py);
}

/* Radar inset square: centre and radius. */
static void inset_geo(const canvas *c, int *cx, int *cy, int *R)
{
    int st = c->h >= 600 ? 2 : 1; /* hud_render's status line scale */
    *R = c->h / 6;
    *cx = c->w - *R - 24;
    *cy = c->h - *R - 24 - 12 * st;
}

static void draw_inset(const rc *r, const seeker_tracks *t, const wv_item *items, int n,
                       const wv_pose *pose)
{
    canvas *c = r->c;
    int R, cx, cy;
    inset_geo(c, &cx, &cy, &R);
    float radar_m = r->cfg->radar_m > 0.5f ? r->cfg->radar_m : 8.0f, k = (float)R / radar_m;
    int ts = r->ts > 1 ? r->ts - 1 : 1;
    draw_fill(c, cx - R - 6, cy - R - 6, 2 * R + 12, 2 * R + 12, C_BLACK);
    draw_rect(c, cx - R - 6, cy - R - 6, 2 * R + 12, 2 * R + 12, 1, C_DIM);
    for (float m = 2.0f; m < radar_m + 0.01f; m += 2.0f)
        draw_circle(c, cx, cy, (int)lroundf(m * k), 1, C_DIM);
    float half = r->cam.hfov_deg * 0.5f * DEG2RAD;
    draw_line(c, cx, cy, cx + (int)lroundf(R * sinf(half)), cy - (int)lroundf(R * cosf(half)), 1,
              C_HUD);
    draw_line(c, cx, cy, cx - (int)lroundf(R * sinf(half)), cy - (int)lroundf(R * cosf(half)), 1,
              C_HUD);
    draw_line(c, cx, cy - 7, cx - 5, cy + 5, r->lw, C_WHITE);
    draw_line(c, cx, cy - 7, cx + 5, cy + 5, r->lw, C_WHITE);
    int nx, ny;
    inset_map(pose, k, R, cx, cy, pose->x_m, pose->y_m + radar_m * 2, &nx, &ny);
    draw_text(c, nx - 2 * ts, ny - 3 * ts, ts, C_DIM, "N");

    for (int i = 0; i < r->w->n_walls; i++) {
        int x0, y0, x1, y1;
        const float *wl = r->w->walls[i];
        inset_map(pose, k, R, cx, cy, wl[0], wl[1], &x0, &y0);
        inset_map(pose, k, R, cx, cy, wl[2], wl[3], &x1, &y1);
        draw_line(c, x0, y0, x1, y1, r->lw + 1, C_WHITE);
    }
    for (int i = 0; t && i < t->n_nodes; i++) {
        const seeker_node_status *s = &t->nodes[i];
        if (!strcmp(s->node_id, r->w->node))
            continue;
        uint32_t col = !strcmp(s->state, "ok") ? C_HUD : !strcmp(s->state, "lost") ? C_AMBER : C_GREY;
        inset_map(pose, k, R, cx, cy, s->x_m, s->y_m, &nx, &ny);
        draw_fill(c, nx - 4, ny - 4, 8, 8, col);
        draw_text(c, nx + 6, ny - 6, ts, col, s->node_id);
    }
    for (int i = 0; t && i < n; i++) {
        const wv_item *it = &items[i];
        const seeker_track *tk = &t->tracks[it->idx];
        if (it->dup_of >= 0)
            continue;
        if (!it->has_range && !it->direct) {
            /* Range-less sighting from another phone: dashed ray from that phone. */
            const seeker_node_status *s = find_node(t, tk->node_id);
            if (!s)
                continue;
            float a = (s->yaw_deg + tk->node_bearing_deg) * DEG2RAD;
            int x0, y0, x1, y1;
            inset_map(pose, k, R, cx, cy, s->x_m, s->y_m, &x0, &y0);
            inset_map(pose, k, R, cx, cy, s->x_m + radar_m * sinf(a), s->y_m + radar_m * cosf(a),
                      &x1, &y1);
            draw_dashed(c, x0, y0, x1, y1, 1, 4, xray_colour(r, tk));
            continue;
        }
        int px, py;
        if (it->has_range) {
            float a = (pose->yaw_deg + it->bearing_deg) * DEG2RAD;
            inset_map(pose, k, R, cx, cy, pose->x_m + it->range_m * sinf(a),
                      pose->y_m + it->range_m * cosf(a), &px, &py);
        } else {
            float a = it->bearing_deg * DEG2RAD;
            draw_dashed(c, cx, cy, cx + (int)lroundf(R * sinf(a)), cy - (int)lroundf(R * cosf(a)),
                        1, 4, direct_colour(r, it, tk));
            continue;
        }
        if (it->direct) {
            uint32_t col = direct_colour(r, it, tk);
            draw_fill(c, px - 5, py - 5, 11, 11, col);
        } else {
            draw_diamond(c, px, py, 7, r->lw, xray_colour(r, tk));
        }
    }
}

/* video_rx already scaled the picture into a canvas-sized buffer at f->rect. */
static void draw_video(canvas *c, const video_frame *f)
{
    if (f->w == c->w && f->h == c->h && f->stride == c->stride) {
        memcpy(c->px, f->px, (size_t)c->stride * (size_t)c->h * 4);
        return;
    }
    int w = f->w < c->w ? f->w : c->w, h = f->h < c->h ? f->h : c->h;
    for (int y = 0; y < h; y++)
        memcpy(c->px + (size_t)y * c->stride, f->px + (size_t)y * f->stride, (size_t)w * 4);
}

static void draw_legend(const rc *r)
{
    int st = r->c->h >= 600 ? 2 : 1;
    int y = r->c->h - 12 * st - 14 * r->ts - 8, x = 12, s = 7 * r->ts;
    draw_fill(r->c, x - 4, y - 4, draw_text_width(r->ts, "SEEN  XRAY  OFF") + 3 * s + 40,
              s + 8, C_BLACK);
    draw_rect(r->c, x, y, s, s, r->lw, C_GREEN);
    x = draw_text(r->c, x + s + 6, y, r->ts, C_GREEN, "SEEN") + 14;
    dashed_rect(r->c, x, y, s, s, r->lw, 3, C_CYAN);
    x = draw_text(r->c, x + s + 6, y, r->ts, C_CYAN, "XRAY") + 14;
    draw_line(r->c, x + s, y, x, y + s / 2, r->lw, C_HUD);
    draw_line(r->c, x, y + s / 2, x + s, y + s, r->lw, C_HUD);
    draw_text(r->c, x + s + 6, y, r->ts, C_HUD, "OFF");
}

/* ---- plan theme (draw_theme() == DRAW_THEME_PLAN): a drawing over the camera ---- */

static int P(const rc *r, float v) /* design px (at 720 tall) to frame px */
{
    return (int)lroundf(v * r->u);
}

static int p_stale(const rc *r, const seeker_track *k)
{
    return r->stale || k->age_ms > r->cfg->stale_ms;
}

/* "R2", or "R1 and R2" for a "R1+R2" node list. */
static void p_nodes(char *out, size_t n, const seeker_track *k)
{
    const char *src = k->nodes[0] ? k->nodes : k->node_id;
    size_t o = 0;
    for (; *src && o + 6 < n; src++) {
        if (*src == '+')
            o += (size_t)snprintf(out + o, n - o, " and ");
        else
            out[o++] = *src;
    }
    out[o < n ? o : n - 1] = '\0';
}

/* Is this rectangle clear of every placed label and figure (with the same 3 px margin)? */
static int p_free(const rc *r, int x, int y, int w, int h)
{
    for (int i = 0; i < r->n_lab; i++)
        if (overlaps(r->lab[i], x - 3, y - 3, w + 6, h + 6))
            return 0;
    return 1;
}

/* Leader and plate for a box: to its right (dx px from the right edge, plate top dy px above the
 * box top), else to its left, else centred above it; the first spot that is on screen and clear
 * of other plates and figures wins. If none is, the right-hand spot is nudged up or down. The
 * leader starts lead_dy px down the box's side (from the top centre for the plate above). */
static void p_tag(rc *r, int bx, int by, int bw, float lead_dy, float dx, float dy,
                  const plan_line *l, int n, uint32_t col, int red_hatch)
{
    int pw, ph, edge = P(r, 8), W = r->c->w, H = r->c->h;
    plan_plate_size(r->u, l, n, &pw, &ph);
    int above = by - P(r, 16) - ph;
    int cand[5][2] = { { bx + bw + P(r, dx), by - P(r, dy) },
                       { bx - P(r, dx) - pw, by - P(r, dy) },
                       { bx + bw / 2 - pw / 2, above },
                       { bx, above },
                       { bx + bw - pw, above } };
    int pick = -1;
    for (int i = 0; i < 5 && pick < 0; i++) {
        int cx = cand[i][0], cy = cand[i][1];
        if (cx >= edge && cx + pw <= W - edge && cy >= edge && cy + ph <= H - edge && p_free(r, cx, cy, pw, ph))
            pick = i;
    }
    int px, py;
    if (pick >= 0) {
        px = cand[pick][0];
        py = place_label(r, px, cand[pick][1], pw, ph); /* records it; already clear */
    } else {
        pick = cand[0][0] + pw <= W - edge ? 0 : 1;
        px = clampi(cand[pick][0], edge, W - pw - edge);
        py = place_label(r, px, clampi(cand[pick][1], edge, H - ph - edge), pw, ph);
    }
    /* A plate that has to move (the layout changed) glides to its new spot (animations on). */
    if (r->cur_slot >= 0 && r->view && anim_enabled()) {
        wv_colour_slot *sl = &r->view->colours[r->cur_slot];
        if (sl->plate_valid && (fabsf(sl->plate_x - (float)px) > 1.0f || fabsf(sl->plate_y - (float)py) > 1.0f)) {
            sl->plate_x += P_PLATE_GLIDE * ((float)px - sl->plate_x);
            sl->plate_y += P_PLATE_GLIDE * ((float)py - sl->plate_y);
            px = (int)lroundf(sl->plate_x);
            py = (int)lroundf(sl->plate_y);
        } else {
            sl->plate_x = (float)px;
            sl->plate_y = (float)py;
        }
        sl->plate_valid = 1;
    }
    if (pick >= 2) { /* above: a straight leader up from the box top */
        float lx = (float)clampi(px + pw / 2, bx + P(r, 6), bx + bw - P(r, 6));
        ap_leader(r->c, r->u, lx, (float)by, lx, (float)(py + ph), col, &r->pfx);
    } else {
        int right = pick == 0;
        float lx = (float)(right ? bx + bw : bx);
        float ly = (float)clampi(by + P(r, lead_dy), 0, H - 1);
        ap_leader(r->c, r->u, lx, ly, (float)(right ? px : px + pw), (float)(py + P(r, 14)), col, &r->pfx);
    }
    /* Fading in (anim_person): slides in from the person's side, right plate from the left, left
     * from the right, above from below. After that, the alert sweep (anim_alert) owns the plate. */
    if (r->pfx.active && r->pfx.plate < 1.0f)
        ap_plate(r->c, r->u, px, py, l, n, col, red_hatch, pick == 0 ? -1 : pick == 1 ? 1 : 0, pick >= 2, &r->pfx);
    else
        anim_alert_plate(r->c, r->u, px, py, l, n, col, red_hatch, &r->fx);
}

/* The box a person is drawn in this frame: item_box, eased like classic (WV_BOX_EASE), and cut
 * off above the bottom edge. Called once per person per frame: the easing keeps state. raw gets
 * the eased box before the cut (the stats record that one, like classic). */
static void p_box(rc *r, const wv_item *it, const seeker_track *k, int b[4], int raw[4])
{
    int x, y, w, h;
    item_box(r, it, k, &x, &y, &w, &h);
    ease_box(r, it, &x, &y, &w, &h);
    raw[0] = x, raw[1] = y, raw[2] = w, raw[3] = h;
    int bottom = r->c->h - P(r, 8);
    if (y + h > bottom)
        h = bottom - y;
    if (h < 8)
        h = 8;
    b[0] = x, b[1] = y, b[2] = w, b[3] = h;
}

static int p_down(const seeker_track *k)
{
    return k->posture == SEEKER_POSTURE_DOWN;
}

/* A seen person close by (the classic RED rule, with its hysteresis). */
static int p_close(const rc *r, const wv_item *it, const seeker_track *k)
{
    return it->direct && !p_stale(r, k) && it->has_range && it->level == SEEKER_RED;
}

/* Red in the plan theme: a seen person close by, or anyone down (the rescue case). */
static int p_alarm(const rc *r, const wv_item *it, const seeker_track *k)
{
    return !p_stale(r, k) && (p_down(k) || p_close(r, it, k));
}

/* Plain posture words after the distance: standing and unknown say nothing, down is its own title. */
static const char *p_posture(const seeker_track *k)
{
    return k->posture == SEEKER_POSTURE_CROUCH ? ", crouching" : "";
}

/* Lowest drawing row for a ghost lifted into view: above the title block row. */
static int p_floor(const rc *r)
{
    return r->c->h - P(r, 24) - plan_title_h(r->u) - P(r, 8);
}

/* COCO-17 bones as drawn in the plan theme (no face bones: the head circle stands for them). */
static const unsigned char P_BONES[][2] = { { 5, 6 },   { 5, 7 },   { 7, 9 },   { 6, 8 },   { 8, 10 },
                                            { 5, 11 },  { 6, 12 },  { 11, 12 }, { 11, 13 }, { 13, 15 },
                                            { 12, 14 }, { 14, 16 }, { 0, 5 },   { 0, 6 } };

/* Solid rectangle plus centre ticks (seen), or hatched dashed hull plus a dashed skeleton or a
 * dashed cross (behind a wall). For someone down behind a wall the hull follows the lying ghost
 * (b is updated, so the plate and the label room follow it too). */
static void p_marks(rc *r, const wv_item *it, const seeker_track *k, int b[4])
{
    int x = b[0], y = b[1], w = b[2], h = b[3];
    int st = p_stale(r, k);
    if (it->direct) {
        uint32_t col = anim_alert_tint(&r->fx, p_alarm(r, it, k) ? P_ALERT : P_SEEN);
        int a = (int)(anim_alert_alpha(&r->fx, st ? 115 : 255) * r->fade_a);
        if (r->pfx.active && (r->pfx.echo || r->pfx.trace < 1.0f)) { /* corner trace, or an echo */
            ap_seen_frame(r->c, r->u, x, y, w, h, col, a, &r->pfx);
            return;
        }
        draw_poly_style fs = { 0 };
        fs.edge = col;
        fs.edge_a = a;
        fs.edge_w = 2.0f * r->u;
        float my = y + h * 0.5f, t8 = 8 * r->u;
        plan_halo_line(r->c, r->u, x - t8, my, x + t8, my, 2 * r->u, DRAW_CAP_BUTT, a);
        plan_halo_line(r->c, r->u, x + w - t8, my, x + w + t8, my, 2 * r->u, DRAW_CAP_BUTT, a);
        plan_halo_style(&fs, r->u, a);
        draw_cut_rect(r->c, (float)x, (float)y, (float)w, (float)h, 0, &fs);
        draw_line_aa(r->c, x - t8, my, x + t8, my, 2 * r->u, DRAW_CAP_BUTT, col, a);
        draw_line_aa(r->c, x + w - t8, my, x + w + t8, my, 2 * r->u, DRAW_CAP_BUTT, col, a);
        anim_alert_ring(r->c, r->u, &r->fx, (float)x, (float)y, (float)w, (float)h, r->fade_a);
        return;
    }
    int a = (int)(ap_alpha(&r->pfx, st ? 120 : 255) * r->fade_a);
    if (it->has_ray_span)
        plan_dash(r->c, r->u, (float)to_sx(r, it->ray_u0), (float)to_sy(r, it->ray_v0), (float)to_sx(r, it->ray_u1),
                  (float)to_sy(r, it->ray_v1), 2 * r->u, 10 * r->u, 7 * r->u, P_HIDDEN, a * 7 / 10);
    /* Skeleton joints first (classic's placement: hip row by posture, a person down lifted to the
     * bottom of the view), so the hull can follow a lying ghost. */
    float sx[SEEKER_N_JOINTS], sy[SEEKER_N_JOINTS], tall = 0;
    int ok[SEEKER_N_JOINTS], have = 0;
    if (k->has_pts && it->has_proj && it->p.box_h > 0) {
        const wv_proj *p = &it->p;
        float hip_row = p_down(k) ? 0.91f : k->posture == SEEKER_POSTURE_CROUCH ? 0.74f : 0.49f;
        float hip_v = p->box_y + hip_row * p->box_h, low = -1e9f;
        float kx = (float)r->vp.w / (float)r->cam.image_w, ky = (float)r->vp.h / (float)r->cam.image_h;
        for (int j = 0; j < SEEKER_N_JOINTS; j++) {
            sx[j] = r->vp.x0 + (p->u + k->pts[j][0] * p->box_h) * kx;
            sy[j] = r->vp.y0 + (hip_v + k->pts[j][1] * p->box_h) * ky;
            ok[j] = k->pts[j][2] >= 0.3f;
            if (ok[j] && sy[j] > low)
                low = sy[j];
            have |= ok[j];
        }
        float floor_y = (float)p_floor(r);
        if (p_down(k) && low > floor_y)
            for (int j = 0; j < SEEKER_N_JOINTS; j++)
                sy[j] -= low - floor_y;
        tall = p->box_h * ky;
    }
    if (have && p_down(k)) { /* hull around the lying ghost, not the standing box */
        float x0 = 1e9f, y0 = 1e9f, x1 = -1e9f, y1 = -1e9f, m = tall * 0.07f;
        for (int j = 0; j < SEEKER_N_JOINTS; j++)
            if (ok[j]) {
                x0 = fminf(x0, sx[j]), y0 = fminf(y0, sy[j]);
                x1 = fmaxf(x1, sx[j]), y1 = fmaxf(y1, sy[j]);
            }
        x = (int)lroundf(x0 - m), y = (int)lroundf(y0 - m * 1.6f);
        w = (int)lroundf(x1 - x0 + 2 * m), h = (int)lroundf(y1 - y0 + m * 2.6f);
        b[0] = x, b[1] = y, b[2] = w, b[3] = h;
    }
    draw_poly_style hs = { 0 };
    hs.hatch = P_HIDDEN;
    hs.hatch_a = (int)(ap_alpha(&r->pfx, 18) * r->fade_a); /* 7% */
    hs.hatch_gap = 12 * r->u;
    hs.hatch_w = 2 * r->u;
    if (hs.hatch_a >= 10 && !r->pfx.echo) /* a 4% hatch or an echo's is not worth the fill */
        ap_hatch_rect(r->c, (float)x, (float)y, (float)w, (float)h, &hs, &r->pfx);
    plan_dash_rect(r->c, r->u, (float)x, (float)y, (float)w, (float)h, 1.5f * r->u, 10 * r->u, 7 * r->u, P_HIDDEN,
                   a * 55 / 100);
    anim_alert_hull(r->c, r->u, &r->fx, (float)x, (float)y, (float)w, (float)h, r->fade_a);
    int nb = 0;
    /* Two passes: every halo first, then the colour, so one bone's halo never cuts another bone. */
    for (int pass = 0; have && pass < 2; pass++) {
        nb = 0;
        for (size_t bi = 0; bi < sizeof P_BONES / sizeof P_BONES[0]; bi++) {
            int ja = P_BONES[bi][0], jb = P_BONES[bi][1];
            if (ok[ja] && ok[jb]) {
                if (pass)
                    draw_dashed_aa(r->c, sx[ja], sy[ja], sx[jb], sy[jb], 3 * r->u, 11 * r->u, 7 * r->u, DRAW_CAP_BUTT,
                                   P_HIDDEN, a);
                else
                    plan_halo_dash(r->c, r->u, sx[ja], sy[ja], sx[jb], sy[jb], 3 * r->u, 11 * r->u, 7 * r->u, a);
                nb++;
            }
        }
        if (nb > 0) {
            float hr = tall * 0.055f, hy = sy[0] - tall * 0.015f, hw = 3 * r->u + (pass ? 0 : P_HALO_W * r->u);
            if (ok[0])
                draw_circle_aa(r->c, sx[0], hy, hr, hw, 11 * r->u, 7 * r->u, pass ? P_HIDDEN : P_HALO,
                               pass ? a : P_HALO_A * a / 255);
            float q = (float)P(r, 8);
            for (int j = 5; j < SEEKER_N_JOINTS; j++)
                if (ok[j] && pass)
                    plan_square(r->c, r->u, (float)lroundf(sx[j]), (float)lroundf(sy[j]), q, P_HIDDEN, a);
            if (pass)
                r->n_skel++;
        }
    }
    if (nb == 0) { /* no skeleton: a dashed cross at the centre of the hull */
        float cx = x + w * 0.5f, cy = y + h * 0.5f, arm = fminf((float)w, (float)h) * 0.22f;
        plan_halo_dash(r->c, r->u, cx - arm, cy, cx + arm, cy, 2 * r->u, 8 * r->u, 5 * r->u, a);
        plan_halo_dash(r->c, r->u, cx, cy - arm, cx, cy + arm, 2 * r->u, 8 * r->u, 5 * r->u, a);
        draw_dashed_aa(r->c, cx - arm, cy, cx + arm, cy, 2 * r->u, 8 * r->u, 5 * r->u, DRAW_CAP_BUTT, P_HIDDEN, a);
        draw_dashed_aa(r->c, cx, cy - arm, cx, cy + arm, 2 * r->u, 8 * r->u, 5 * r->u, DRAW_CAP_BUTT, P_HIDDEN, a);
    }
}

/* Plate words for one person: title and value (distance from the wearer, posture, source). */
static void p_words(const rc *r, const wv_item *it, const seeker_track *k, char *title, size_t tn,
                    char *val, size_t vn, const char *extra)
{
    char rs[16] = "";
    if (it->has_range)
        snprintf(rs, sizeof rs, "%.1f m", it->range_m);
    if (it->direct) {
        snprintf(title, tn, "%s", p_down(k) ? "Person down" : "Person");
        const char *near = p_close(r, it, k) && !p_down(k) ? (it->closing ? ", coming closer" : ", close") : "";
        snprintf(val, vn, "%s%s%s%s", rs[0] ? rs : "no range", p_posture(k), near, extra ? extra : "");
        return;
    }
    char by[32];
    p_nodes(by, sizeof by, k);
    const char *where = it->behind_wall ? "Behind wall" : "Out of sight";
    snprintf(title, tn, "%s%s", where, p_down(k) ? ", person down" : "");
    if (rs[0])
        snprintf(val, vn, "%s%s, seen by %s%s", rs, p_posture(k), by, extra ? extra : "");
    else
        snprintf(val, vn, "%sseen by %s%s", k->posture == SEEKER_POSTURE_CROUCH ? "crouching, " : "", by,
                 extra ? extra : "");
}

static void p_label(rc *r, const wv_item *it, const seeker_track *k, const int b[4])
{
    char title[48], val[72];
    p_words(r, it, k, title, sizeof title, val, sizeof val, NULL);
    int alarm = p_alarm(r, it, k);
    if (it->direct) {
        const anim_fx *f = &r->fx;
        plan_line l[2] = { { title, DRAW_FONT_SANS_SEMIBOLD, 19, anim_alert_tint(f, alarm && p_down(k) ? P_ALERT : P_INK), 255 },
                           { val, DRAW_FONT_MONO, 15, anim_alert_tint(f, alarm ? P_ALERT : P_INK), alarm ? 255 : P_INK2_A } };
        p_tag(r, b[0], b[1], b[2], 18, 46, 30, l, 2, anim_alert_tint(f, alarm ? P_ALERT : P_SEEN), alarm);
        return;
    }
    plan_line l[2] = { { title, DRAW_FONT_SANS_SEMIBOLD, 19, anim_alert_tint(&r->fx, alarm ? P_ALERT : P_HIDDEN), 255 },
                       { val, DRAW_FONT_MONO, 15, P_INK, P_INK2_A } };
    p_tag(r, b[0], b[1], b[2], 20, 40, 34, l, 2, anim_alert_tint(&r->fx, alarm ? P_ALERT : P_HIDDEN), alarm);
}

static void p_inset_rect(const rc *r, int *x, int *y, int *w, int *h)
{
    *w = P(r, 260);
    *h = P(r, 196);
    *x = r->c->w - P(r, 24) - *w;
    *y = r->c->h - P(r, 24) - *h;
}

static void p_edge(rc *r, const wv_item *it, const seeker_track *k, int slot)
{
    wv_edge e = wv_edge_for(it->bearing_deg);
    int alarm = p_alarm(r, it, k), a = p_stale(r, k) ? 120 : 255;
    uint32_t col = anim_alert_tint(&r->fx, alarm ? P_ALERT : it->direct ? P_SEEN : P_HIDDEN);
    float s = 16 * r->u;
    int row = P(r, 76), off = (slot % 2 ? 1 : -1) * ((slot + 1) / 2) * row;
    int ylo = P(r, 24) + (int)s, yhi = r->c->h - P(r, 24) - plan_title_h(r->u) - (int)s;
    if (r->w->inset && e.side > 0) {
        int ix, iy, iw, ih;
        p_inset_rect(r, &ix, &iy, &iw, &ih);
        yhi = iy - (int)s;
    }
    int cy = clampi(r->vp.y + r->vp.h / 2 + off, ylo, yhi > ylo ? yhi : ylo);
    float tip = e.side > 0 ? (float)(r->vp.x + r->vp.w) - 14 * r->u : (float)r->vp.x + 14 * r->u;
    float in = (float)-e.side;
    for (int m = 0; m < (e.behind ? 2 : 1); m++) {
        float tx = tip + in * m * s * 0.75f;
        if (it->direct) {
            plan_halo_line(r->c, r->u, tx, (float)cy, tx + in * s, cy - s, 2 * r->u, DRAW_CAP_SQUARE, a);
            plan_halo_line(r->c, r->u, tx, (float)cy, tx + in * s, cy + s, 2 * r->u, DRAW_CAP_SQUARE, a);
            draw_line_aa(r->c, tx, (float)cy, tx + in * s, cy - s, 2 * r->u, DRAW_CAP_SQUARE, col, a);
            draw_line_aa(r->c, tx, (float)cy, tx + in * s, cy + s, 2 * r->u, DRAW_CAP_SQUARE, col, a);
        } else {
            plan_halo_dash(r->c, r->u, tx, (float)cy, tx + in * s, cy - s, 2 * r->u, 5 * r->u, 4 * r->u, a);
            plan_halo_dash(r->c, r->u, tx, (float)cy, tx + in * s, cy + s, 2 * r->u, 5 * r->u, 4 * r->u, a);
            draw_dashed_aa(r->c, tx, (float)cy, tx + in * s, cy - s, 2 * r->u, 5 * r->u, 4 * r->u, DRAW_CAP_BUTT, col, a);
            draw_dashed_aa(r->c, tx, (float)cy, tx + in * s, cy + s, 2 * r->u, 5 * r->u, 4 * r->u, DRAW_CAP_BUTT, col, a);
        }
    }
    char turn[40], title[48], val[96];
    snprintf(turn, sizeof turn, ", %.0f\xc2\xb0 %s%s", e.turn_deg, e.side > 0 ? "right" : "left",
             e.behind ? ", behind you" : "");
    p_words(r, it, k, title, sizeof title, val, sizeof val, turn);
    plan_line l[2] = { { title, DRAW_FONT_SANS_SEMIBOLD, 19, alarm ? P_ALERT : it->direct ? P_INK : P_HIDDEN, 255 },
                       { val, DRAW_FONT_MONO, 15, alarm && it->direct ? P_ALERT : P_INK,
                         alarm && it->direct ? 255 : P_INK2_A } };
    int pw, ph;
    plan_plate_size(r->u, l, 2, &pw, &ph);
    int px = e.side > 0 ? (int)(tip - s * 2) - pw : (int)(tip + s * 2);
    int py = place_label(r, px, cy - ph / 2, pw, ph);
    anim_alert_plate(r->c, r->u, px, py, l, 2, col, alarm, &r->fx);
}

/* Position cell and warning words for the wearer's pose (fusion's pose_src for the wearer). */
static const char *p_position(const wv_stats *s, const char *src)
{
    if (s->pose_mode == WV_MODE_HELD)
        return !strcmp(src, "heading") ? "Compass" : !strcmp(src, "home") ? "Easing home" : "Held";
    static const char *const word[] = { "Fixed", "Live", "Held", "Starting" };
    return word[s->pose_mode];
}

static void wv_plan(rc *r, wv_view *v, const seeker_tracks *t, const wv_item *items, int n,
                    const wv_pose *pose, wv_stats *s, int use_video, int other_node,
                    const video_frame *video)
{
    const wv_config *w = r->w;
    canvas *c = r->c;
    const seeker_node_status *self = t ? find_node(t, w->node) : NULL;
    const char *src = self ? self->pose_src : "";
    /* What needs saying first (top left), then what the frame is. */
    char wt[64] = "", wv_[64] = "";
    uint32_t wedge = P_ALERT; /* every warning plate has the alert edge */
    int wred = 0;             /* and the worst also an alert title */
    if (!t) {
        snprintf(wt, sizeof wt, "Waiting for fusion");
        wred = 1;
    } else if (!s->wearer_state[0]) {
        snprintf(wt, sizeof wt, "%s not in fusion", w->node);
        snprintf(wv_, sizeof wv_, "check the phone link");
    } else if (strcmp(s->wearer_state, "ok") != 0) {
        snprintf(wt, sizeof wt, "%s %s", w->node, !strcmp(s->wearer_state, "lost") ? "lost" : "waiting");
        snprintf(wv_, sizeof wv_, "showing people behind walls only");
        wred = 1;
    } else if (s->pose_mode == WV_MODE_HELD) {
        if (!strcmp(src, "heading")) {
            snprintf(wt, sizeof wt, "%s on compass", w->node);
            snprintf(wv_, sizeof wv_, "no marker, heading from the compass");
        } else if (!strcmp(src, "home")) {
            snprintf(wt, sizeof wt, "%s easing home", w->node);
            snprintf(wv_, sizeof wv_, "no marker, back to its home spot");
        } else {
            snprintf(wt, sizeof wt, "%s position held", w->node);
            snprintf(wv_, sizeof wv_, "phone lost its marker");
        }
    } else if (other_node && video) {
        snprintf(wt, sizeof wt, "Video from %s, not %s", video->node_id, w->node);
    }
    plan_line wl[2] = { { wt, DRAW_FONT_SANS_SEMIBOLD, 19, wred ? P_ALERT : P_INK, 255 },
                        { wv_, DRAW_FONT_MONO, 15, P_INK, P_INK2_A } };
    int wn = wv_[0] ? 2 : 1, ww = 0, wh = 0;
    if (wt[0]) {
        plan_plate_size(r->u, wl, wn, &ww, &wh);
        place_label(r, P(r, 24), P(r, 24), ww, wh);
    }
    char cam[48];
    int late = use_video && v->video_stale;
    if (late) {
        long age = video ? (long)(seeker_now_ms() - video->t_rx_ms) : 0;
        snprintf(cam, sizeof cam, "Video %ld.%ld s late", age / 1000, age % 1000 / 100);
    } else if (use_video)
        snprintf(cam, sizeof cam, "%s live", w->node);
    else
        snprintf(cam, sizeof cam, "%s", other_node ? "Wrong camera" : w->visor ? "Visor" : "No video");
    const char *pos = p_position(s, src);
    int tbw = plan_title_block(NULL, r->u, cam, late, t, w->node, pos, v->delay_text), tbh = plan_title_h(r->u);
    place_label(r, P(r, 24), c->h - P(r, 24) - tbh, tbw, tbh);
    int ix, iy, iw, ih;
    p_inset_rect(r, &ix, &iy, &iw, &ih);
    if (w->inset)
        place_label(r, ix, iy, iw, ih);

    /* Each person's box once per frame (easing), in classic's order: behind walls, then seen. */
    static int box[SEEKER_MAX_TRACKS][4];
    for (int pass = 0; pass < 2; pass++)
        for (int i = 0; i < n && i < SEEKER_MAX_TRACKS; i++) {
            const wv_item *it = &items[i];
            const seeker_track *k = &t->tracks[it->idx];
            if (it->dup_of >= 0 || !it->has_bearing || !it->in_fov) {
                if (pass == 0)
                    anim_person_keep(k->id); /* still tracked, just not drawn here: no echo */
                continue;
            }
            if (it->direct != pass)
                continue;
            int raw[4];
            p_box(r, it, k, box[i], raw);
            if (r->n_drawn < SEEKER_MAX_TRACKS) {
                int *d = &r->drawn[r->n_drawn++][0];
                d[0] = k->id, d[1] = it->direct, d[2] = raw[0], d[3] = raw[1], d[4] = raw[2], d[5] = raw[3],
                d[6] = it->level;
            }
            r->n_posture[k->posture >= 0 && k->posture <= SEEKER_POSTURE_DOWN ? k->posture : 0]++;
            /* Marks: people behind walls first, then seen people over them. A change of look
             * dissolves: the old look fades out in the same box while the new one fades in. The
             * crossfade alone sets both looks' opacity; the person animations only style them,
             * and the alert decoration goes with the new look only. */
            r->fx = anim_alert_track(k->id, p_alarm(r, it, k), p_down(k));
            r->pfx = anim_person_track(k->id, it->direct);
            if (it->fade < 1.0f) {
                r->pfx = ap_crossfading(r->pfx);
                anim_fx nfx = r->fx;
                ap_fx npfx = r->pfx;
                wv_item old = *it;
                old.direct = !it->direct;
                int ob[4] = { box[i][0], box[i][1], box[i][2], box[i][3] };
                memset(&r->fx, 0, sizeof r->fx);
                memset(&r->pfx, 0, sizeof r->pfx);
                r->fade_a = 1.0f - it->fade;
                draw_set_dash_phase(old.direct ? 0.0f : anim_person_march());
                p_marks(r, &old, k, ob);
                r->fx = nfx;
                r->pfx = npfx;
                r->fade_a = it->fade;
                r->n_fading++;
            }
            draw_set_dash_phase(r->pfx.march);
            p_marks(r, it, k, box[i]);
            r->fade_a = 1.0f;
            draw_set_dash_phase(0);
            memset(&r->fx, 0, sizeof r->fx);
            memset(&r->pfx, 0, sizeof r->pfx);
            anim_person_store(k->id, it, k, box[i]);
            /* Labels come after all marks and keep off every figure (plates move up or down). */
            /* A standing ghost's head ring rises above its hull; a lying one's is inside it. */
            int head = it->direct || p_down(k) ? 0 : box[i][3] / 8;
            if (r->n_lab < 64) {
                int *L = r->lab[r->n_lab++];
                L[0] = box[i][0] - 4;
                L[1] = box[i][1] - 4 - head;
                L[2] = box[i][2] + 8;
                L[3] = box[i][3] + 8 + head;
            }
        }
    /* Echoes of people no longer drawn in view: their last marks, fading, no plate. */
    anim_person_end();
    for (int e = 0; e < AP_SLOTS; e++) {
        static wv_item ei;
        static seeker_track ek;
        int eb[4], ns = r->n_skel;
        memset(&r->fx, 0, sizeof r->fx); /* an echo has no alert state */
        if (!anim_person_echo(e, &ei, &ek, eb, &r->pfx))
            continue;
        draw_set_dash_phase(r->pfx.march);
        p_marks(r, &ei, &ek, eb);
        draw_set_dash_phase(0);
        r->n_skel = ns;
    }
    memset(&r->fx, 0, sizeof r->fx);
    memset(&r->pfx, 0, sizeof r->pfx);
    int slot_l = 0, slot_r = 0;
    for (int pass = 0; pass < 3; pass++)
        for (int i = 0; i < n; i++) {
            const wv_item *it = &items[i];
            const seeker_track *k = &t->tracks[it->idx];
            if (it->dup_of >= 0 || !it->has_bearing) {
                if (pass == 0 && it->dup_of >= 0)
                    s->n_dup++;
                else if (pass == 0)
                    s->n_unplaced++;
                continue;
            }
            r->fx = anim_alert_track(k->id, p_alarm(r, it, k), p_down(k));
            if (it->in_fov && pass == it->direct) {
                if (i < SEEKER_MAX_TRACKS) {
                    r->pfx = anim_person_get(k->id);
                    r->cur_slot = it->slot;
                    p_label(r, it, k, box[i]);
                    r->cur_slot = -1;
                    memset(&r->pfx, 0, sizeof r->pfx);
                }
                if (it->direct)
                    s->n_direct++;
                else
                    s->n_xray++;
            } else if (!it->in_fov && pass == 2) {
                int right = wv_edge_for(it->bearing_deg).side > 0;
                p_edge(r, it, k, right ? slot_r++ : slot_l++);
                s->n_off++;
            }
        }

    if (w->inset) {
        static plan_mark marks[SEEKER_MAX_TRACKS];
        int nm = 0;
        for (int i = 0; t && i < n && nm < SEEKER_MAX_TRACKS; i++) {
            const wv_item *it = &items[i];
            const seeker_track *k = &t->tracks[it->idx];
            if (it->dup_of >= 0)
                continue;
            plan_mark *m = &marks[nm];
            memset(m, 0, sizeof *m);
            m->id = k->id;
            m->down = !p_stale(r, k) && p_down(k);
            if (k->has_room) {
                m->kind = it->direct ? PLAN_SEEN : PLAN_HIDDEN;
                m->close = p_alarm(r, it, k);
                m->x = k->room_x_m;
                m->y = k->room_y_m;
            } else if (it->direct) {
                m->kind = PLAN_RAY;
                m->x = pose->x_m;
                m->y = pose->y_m;
                m->ray_deg = pose->yaw_deg + it->bearing_deg;
            } else {
                const seeker_node_status *sn = find_node(t, k->node_id);
                if (!sn)
                    continue;
                m->kind = PLAN_RAY;
                m->x = sn->x_m;
                m->y = sn->y_m;
                m->ray_deg = sn->yaw_deg + k->node_bearing_deg;
            }
            nm++;
        }
        plan_map(c, r->u, ix, iy, iw, ih, 26 * r->u, (float)(ih - P(r, 22)) / (float)ih, 1, pose, w->node,
                 r->cam.hfov_deg, w, t, marks, nm);
    }
    plan_title_block(c, r->u, cam, late, t, w->node, pos, v->delay_text);
    if (wt[0])
        anim_alert_warn(c, r->u, P(r, 24), P(r, 24), wl, wn, wedge);
    s->n_skel = r->n_skel;
    s->n_fading = r->n_fading;
    memcpy(s->n_posture, r->n_posture, sizeof s->n_posture);
    s->n_drawn = r->n_drawn;
    for (int i = 0; i < r->n_drawn; i++) {
        s->drawn[i].id = r->drawn[i][0];
        s->drawn[i].direct = r->drawn[i][1];
        s->drawn[i].x = r->drawn[i][2];
        s->drawn[i].y = r->drawn[i][3];
        s->drawn[i].w = r->drawn[i][4];
        s->drawn[i].h = r->drawn[i][5];
        s->drawn[i].level = r->drawn[i][6];
    }
}

void wv_render(canvas *c, const seeker_config *cfg, wv_view *v, const seeker_tracks *t, int stale,
               const video_frame *video)
{
    const wv_config *w = &v->cfg;
    rc r;
    memset(&r, 0, sizeof r);
    r.c = c;
    r.cfg = cfg;
    r.w = w;
    r.view = v;
    r.cam = w->cam;
    const seeker_node_status *self = t ? find_node(t, w->node) : NULL;
    if (!w->hfov_fixed && self && self->hfov_deg > 10.0f && self->hfov_deg < 170.0f)
        r.cam.hfov_deg = self->hfov_deg; /* the FOV the phone computes its bearings with */
    r.ts = c->h >= 1000 ? 3 : c->h >= 600 ? 2 : 1;
    r.lw = cfg->line_px > 0 ? cfg->line_px : 2;
    r.stale = stale;
    r.plan = draw_theme() == DRAW_THEME_PLAN;
    r.cur_slot = -1;
    r.u = plan_u(c);
    r.fade_a = 1.0f;
    int have_video = video && video->px && video->src_w > 0 && video->src_h > 0 &&
                     video->rect[2] > 0 && video->rect[3] > 0;
    int other_node = have_video && video->node_id[0] && strcmp(video->node_id, w->node) != 0;
    int use_video = have_video && !other_node && !w->visor;
    if (use_video) {
        r.cam.image_w = video->src_w;
        r.cam.image_h = video->src_h;
    }
    if (r.cam.image_w <= 0 || r.cam.image_h <= 0) {
        r.cam.image_w = 1280;
        r.cam.image_h = 720;
    }
    r.vp = wv_viewport_fit(r.cam.image_w, r.cam.image_h, c->w, c->h);
    int64_t vu0 = seeker_now_us();
    /* A canvas-sized frame (the receiver letterboxes into one) overwrites every pixel. */
    if (!use_video || video->w != c->w || video->h != c->h || video->stride != c->stride)
        draw_clear(c, C_BLACK);
    if (use_video) {
        draw_video(c, video);
        for (int d = 0; d < v->video_stale && d < 2; d++) /* stale: half, frozen: a quarter */
            draw_dim(c, video->rect[0], video->rect[1], video->rect[2], video->rect[3]);
        r.vp.x = video->rect[0];
        r.vp.y = video->rect[1];
        r.vp.w = video->rect[2];
        r.vp.h = video->rect[3];
        r.vp.x0 = (float)r.vp.x;
        r.vp.y0 = (float)r.vp.y;
        r.vp.s = (float)r.vp.w / (float)r.cam.image_w;
    }

    wv_stats *s = &v->last;
    memset(s, 0, sizeof *s);
    s->video_ms = (float)(seeker_now_us() - vu0) / 1000.0f;
    wv_pose pose;
    wv_resolve_pose(w, t, &pose, &s->pose_live, s->wearer_state, sizeof s->wearer_state);
    if (s->pose_live) {
        float a = w->yaw_alpha > 0.0f && w->yaw_alpha <= 1.0f ? w->yaw_alpha : 1.0f;
        if (!v->have_yaw || stale)
            v->yaw = pose.yaw_deg;
        else
            v->yaw = wv_wrap180(v->yaw + a * wv_wrap180(pose.yaw_deg - v->yaw));
        v->have_yaw = 1;
        pose.yaw_deg = v->yaw;
    } else {
        v->have_yaw = 0;
    }
    s->rendered = 1;
    s->pose = pose;
    s->pose_mode = wv_pose_mode_of(w, t, s->pose_live);
    s->video = use_video ? 1 : other_node ? -1 : 0;

    /* Camera tilt for the x-ray rows: the phone's own pitch when it sends one, else configured;
     * less the nudge; plus what R1's own sightings taught (pitch_auto). */
    int phone_pitch = self && self->has_pitch;
    r.cam.pitch_deg = (phone_pitch ? self->pitch_deg : w->cam.pitch_deg) - w->xray_up_deg +
                      (w->pitch_auto ? v->tilt_fix : 0.0f);
    s->pitch_deg = r.cam.pitch_deg;
    s->pitch_src = phone_pitch;
    wv_config wc = *w;
    wc.cam = r.cam;
    static wv_item items[SEEKER_MAX_TRACKS];
    int n = wv_classify(t, &wc, &pose, items, SEEKER_MAX_TRACKS);
    if (w->pitch_auto && use_video)
        learn_tilt(v, &r.cam, t, items, n);
    int64_t now = v->clock_ms ? v->clock_ms : seeker_now_ms(); /* the frame's one timestamp */
    assign_levels(v, cfg, t, items, n, now);
    if (r.plan) {
        anim_person_begin(now, r.u);
        wv_plan(&r, v, t, items, n, &pose, s, use_video, other_node, video);
        return;
    }

    int cx = r.vp.x + r.vp.w / 2, cy = r.vp.y + r.vp.h / 2;
    {
        char hd[24];
        readout_text(hd, sizeof hd, &pose, s->pose_mode);
        int tw = draw_text_width(r.ts, hd);
        place_label(&r, cx - tw / 2, readout_y(&r), tw, 7 * r.ts); /* drawn last, kept clear */
    }
    draw_line(c, cx - 16, cy, cx - 6, cy, r.lw, C_HUD);
    draw_line(c, cx + 6, cy, cx + 16, cy, r.lw, C_HUD);
    draw_line(c, cx, cy - 16, cx, cy - 6, r.lw, C_HUD);
    draw_line(c, cx, cy + 6, cx, cy + 16, r.lw, C_HUD);

    /* Corner text: composed first so labels keep clear of it, drawn last on black. */
    char lines[5][64];
    uint32_t lcol[5];
    int nl = 0, y0 = r.vp.y + 34 * r.ts + 20;
    snprintf(lines[nl], sizeof lines[0], "EAGLEEYE %s", w->node);
    lcol[nl++] = C_WHITE;
    static const char *const pose_word[] = { "POSE FIXED", "POSE LIVE", "POSE HELD", "POSE START" };
    const char *src = self ? self->pose_src : "";
    snprintf(lines[nl], sizeof lines[0], "%s X%.1f Y%.1f%s",
             s->pose_mode == WV_MODE_HELD && !strcmp(src, "home") ? "POSE HOME"
                                                                 : pose_word[s->pose_mode],
             pose.x_m, pose.y_m,
             s->pose_mode == WV_MODE_HELD && !strcmp(src, "heading") ? " (NO TAG, COMPASS)"
             : s->pose_mode == WV_MODE_HELD                         ? " (NO TAG)"
             : s->pose_mode == WV_MODE_START                        ? " (NO TAG YET)"
                                                                    : "");
    lcol[nl++] = s->pose_mode == WV_MODE_HELD || s->pose_mode == WV_MODE_START ? C_AMBER : C_HUD;
    if (use_video)
        snprintf(lines[nl], sizeof lines[0], "VIDEO %dX%d HFOV %.0f", video->src_w, video->src_h,
                 r.cam.hfov_deg);
    else if (other_node)
        snprintf(lines[nl], sizeof lines[0], "VIDEO FROM %s, NOT %s", video->node_id, w->node);
    else
        snprintf(lines[nl], sizeof lines[0], "%s HFOV %.0f", w->visor ? "VISOR" : "NO VIDEO",
                 r.cam.hfov_deg);
    lcol[nl++] = C_HUD;
    if (!t)
        snprintf(lines[nl], sizeof lines[0], "WAITING FOR FUSION");
    else if (!s->wearer_state[0])
        snprintf(lines[nl], sizeof lines[0], "%s NOT IN FUSION", w->node);
    else if (strcmp(s->wearer_state, "ok") != 0)
        snprintf(lines[nl], sizeof lines[0], "%s %s: XRAY ONLY", w->node,
                 !strcmp(s->wearer_state, "lost") ? "LOST" : "WAIT");
    else
        lines[nl][0] = '\0';
    lcol[nl++] = C_AMBER;
    for (int i = 0; i < nl; i++)
        if (lines[i][0])
            place_label(&r, 12, y0 + i * 10 * r.ts, draw_text_width(r.ts, lines[i]), 7 * r.ts);
    int cw = draw_text_width(r.ts, "SEEN 16  XRAY 16  OFF 16");
    place_label(&r, c->w - cw - 12, y0, cw, 7 * r.ts);
    if (w->inset) {
        int icx, icy, R;
        inset_geo(c, &icx, &icy, &R);
        place_label(&r, icx - R - 6, icy - R - 6, 2 * R + 12, 2 * R + 12);
    }

    /* X-ray boxes, then seen boxes on top, then edge arrows over both. */
    int slot_l = 0, slot_r = 0;
    for (int pass = 0; pass < 3; pass++) {
        for (int i = 0; i < n; i++) {
            const wv_item *it = &items[i];
            const seeker_track *k = &t->tracks[it->idx];
            if (it->dup_of >= 0 || !it->has_bearing) {
                if (pass == 0 && it->dup_of >= 0)
                    s->n_dup++;
                else if (pass == 0)
                    s->n_unplaced++;
                continue;
            }
            if (it->in_fov && pass == it->direct) {
                draw_in_view(&r, it, k);
                if (it->direct)
                    s->n_direct++;
                else
                    s->n_xray++;
            } else if (!it->in_fov && pass == 2) {
                int right = wv_edge_for(it->bearing_deg).side > 0;
                draw_edge(&r, it, k, right ? slot_r++ : slot_l++);
                s->n_off++;
            }
        }
    }

    draw_tape(&r, &pose, s->pose_mode);
    for (int i = 0; i < nl; i++) {
        if (!lines[i][0])
            continue;
        int yy = y0 + i * 10 * r.ts, tw = draw_text_width(r.ts, lines[i]);
        draw_fill(c, 9, yy - 3, tw + 6, 7 * r.ts + 6, C_BLACK);
        draw_text(c, 12, yy, r.ts, lcol[i], lines[i]);
    }
    char line[64];
    snprintf(line, sizeof line, "SEEN %d  XRAY %d  OFF %d", s->n_direct, s->n_xray, s->n_off);
    int tw = draw_text_width(r.ts, line);
    draw_fill(c, c->w - tw - 15, y0 - 3, tw + 6, 7 * r.ts + 6, C_BLACK);
    draw_text(c, c->w - tw - 12, y0, r.ts, C_WHITE, line);
    draw_legend(&r);
    if (w->inset)
        draw_inset(&r, t, items, n, &pose);
    s->n_skel = r.n_skel;
    memcpy(s->n_posture, r.n_posture, sizeof s->n_posture);
    s->n_drawn = r.n_drawn;
    for (int i = 0; i < r.n_drawn; i++) {
        s->drawn[i].id = r.drawn[i][0];
        s->drawn[i].direct = r.drawn[i][1];
        s->drawn[i].x = r.drawn[i][2];
        s->drawn[i].y = r.drawn[i][3];
        s->drawn[i].w = r.drawn[i][4];
        s->drawn[i].h = r.drawn[i][5];
        s->drawn[i].level = r.drawn[i][6];
    }
}
