#include "hud_render.h"
#include "plan.h"
#include "anim.h"
#include "seeker_time.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f
#define FUSION_STALE_MS 500

typedef struct {
    canvas *c;
    const seeker_config *cfg;
    const hud_state *st;
    int ts;    /* text scale */
    int lw;    /* line width */
    int stale; /* fusion silent: draw everything grey */
} ctx;

static float wrap180(float d)
{
    while (d > 180.0f)
        d -= 360.0f;
    while (d <= -180.0f)
        d += 360.0f;
    return d;
}

static const seeker_node_status *node_status(const seeker_tracks *t, const char *id)
{
    for (int i = 0; t && i < t->n_nodes; i++)
        if (!strcmp(t->nodes[i].node_id, id))
            return &t->nodes[i];
    return NULL;
}

static uint32_t track_colour(const ctx *x, const seeker_track *k)
{
    return x->stale ? C_GREY : draw_colour_named(k->colour);
}

static void range_text(char *out, int n, const seeker_track *k, int has, float r)
{
    const char *pw = seeker_posture_word(k->posture);
    if (has)
        snprintf(out, (size_t)n, "%s %s%.1fM%s%s", k->label, k->range_conf < 0.5f ? "~" : "", r,
                 pw[0] ? " " : "", pw);
    else
        snprintf(out, (size_t)n, "%s --.-M%s%s", k->label, pw[0] ? " " : "", pw);
}

/* ---- radar map (full screen or inset) ---- */

typedef struct {
    int cx, cy, R;
    float k; /* px per metre */
    float yaw;
    float ox, oy; /* room point at the centre: the wearer node, else the origin */
} radar_geo;

static void radar_map(const radar_geo *g, float x, float y, int *sx, int *sy, int clamp)
{
    x -= g->ox;
    y -= g->oy;
    float r = hypotf(x, y);
    float b = atan2f(x, y) * RAD2DEG - g->yaw;
    float pr = r * g->k;
    if (clamp && pr > g->R)
        pr = (float)g->R;
    *sx = g->cx + (int)lroundf(pr * sinf(b * DEG2RAD));
    *sy = g->cy - (int)lroundf(pr * cosf(b * DEG2RAD));
}

static void draw_radar(ctx *x, int cx, int cy, int R, int full)
{
    canvas *c = x->c;
    const seeker_tracks *t = x->st->t;
    float radar_m = x->cfg->radar_m > 0.5f ? x->cfg->radar_m : 8.0f;
    radar_geo g = { cx, cy, R, (float)R / radar_m, t ? t->wearer_yaw_deg : 0, 0, 0 };
    const seeker_node_status *wearer = t && t->wearer_node[0] ? node_status(t, t->wearer_node) : NULL;
    if (wearer) { /* fusion.wearer_node: YOU is that phone, not the room origin */
        g.ox = wearer->x_m;
        g.oy = wearer->y_m;
    }
    int ts = full ? x->ts : (x->ts > 1 ? x->ts - 1 : 1);

    for (float m = 2.0f; m < radar_m + 0.01f; m += 2.0f) {
        int r = (int)lroundf(m * g.k);
        draw_circle(c, cx, cy, r, 1, m + 2.0f > radar_m ? C_HUD : C_DIM);
        if (full) {
            char lab[8];
            snprintf(lab, sizeof lab, "%.0fM", m);
            draw_text(c, cx + 4, cy - r + 4, ts, C_DIM, lab);
        }
    }
    draw_line(c, cx - R, cy, cx + R, cy, 1, C_DIM);
    draw_line(c, cx, cy - R, cx, cy + R, 1, C_DIM);
    int s = full ? 10 : 6;
    draw_line(c, cx, cy - s, cx - s * 2 / 3, cy + s * 2 / 3, x->lw, C_HUD);
    draw_line(c, cx, cy - s, cx + s * 2 / 3, cy + s * 2 / 3, x->lw, C_HUD);
    draw_line(c, cx - s * 2 / 3, cy + s * 2 / 3, cx + s * 2 / 3, cy + s * 2 / 3, x->lw, C_HUD);
    if (full)
        draw_text(c, cx + s + 4, cy + 4, ts, C_HUD, "YOU");

    for (int i = 0; t && i < t->n_nodes; i++) {
        const seeker_node_status *n = &t->nodes[i];
        int nx, ny;
        radar_map(&g, n->x_m, n->y_m, &nx, &ny, 1);
        uint32_t col = !strcmp(n->state, "ok") ? C_HUD : !strcmp(n->state, "lost") ? C_AMBER
                                                                                   : C_GREY;
        float cone_m = radar_m * 0.6f;
        for (int side = -1; side <= 1; side += 2) {
            float a = (n->yaw_deg + side * n->hfov_deg * 0.5f) * DEG2RAD;
            int ex, ey;
            radar_map(&g, n->x_m + cone_m * sinf(a), n->y_m + cone_m * cosf(a), &ex, &ey, 1);
            draw_dashed(c, nx, ny, ex, ey, 1, 4, col == C_HUD ? C_DIM : col);
        }
        draw_fill(c, nx - s / 2, ny - s / 2, s, s, col);
        char lab[24];
        snprintf(lab, sizeof lab, "%s%s", n->node_id,
                 !strcmp(n->state, "lost") ? " LOST" : !strcmp(n->state, "wait") ? " --" : "");
        draw_text(c, nx + s, ny - s, ts, col, lab);
    }

    for (int i = 0; t && i < t->n; i++) {
        const seeker_track *k = &t->tracks[i];
        uint32_t col = track_colour(x, k);
        char lab[48];
        if (k->has_room) {
            int px, py;
            radar_map(&g, k->room_x_m, k->room_y_m, &px, &py, 1);
            draw_diamond(c, px, py, full ? 9 : 5, x->lw, col);
            if (full) {
                range_text(lab, sizeof lab, k, k->has_range, k->range_m);
                draw_text(c, px + 12, py - 4, ts, col, lab);
            }
            continue;
        }
        const seeker_node_status *n = node_status(t, k->node_id);
        if (!n)
            continue;
        /* No range: a dashed bearing ray from the phone, cut at the radar edge. */
        float a = (n->yaw_deg + k->node_bearing_deg) * DEG2RAD, len = 0;
        while (len < radar_m * 2.0f &&
               hypotf(n->x_m + (len + 0.25f) * sinf(a) - g.ox,
                      n->y_m + (len + 0.25f) * cosf(a) - g.oy) <= radar_m)
            len += 0.25f;
        int nx, ny, ex, ey;
        radar_map(&g, n->x_m, n->y_m, &nx, &ny, 1);
        radar_map(&g, n->x_m + len * sinf(a), n->y_m + len * cosf(a), &ex, &ey, 1);
        draw_dashed(c, nx, ny, ex, ey, x->lw, 8, col);
        if (full) {
            range_text(lab, sizeof lab, k, 0, 0);
            draw_text(c, ex + 6, ey - 4, ts, col, lab);
        }
    }
}

/* ---- heading tape (IF-7) ---- */

static void draw_tape(ctx *x, const seeker_track *marks, int n_marks, const float *mark_bearing)
{
    canvas *c = x->c;
    const seeker_tracks *t = x->st->t;
    int has_heading = t && t->has_heading;
    float centre = has_heading ? t->heading_deg : 0.0f;
    int tw = c->w / 2, cx = c->w / 2, y0 = 12 * x->ts;
    float ppd = (float)tw / 60.0f;
    draw_line(c, cx - tw / 2, y0, cx + tw / 2, y0, 1, C_HUD);
    for (int d = (int)floorf((centre - 30) / 5) * 5; d <= centre + 30; d += 5) {
        int px = cx + (int)lroundf((d - centre) * ppd);
        if (px < cx - tw / 2 || px > cx + tw / 2)
            continue;
        int major = d % 10 == 0;
        draw_line(c, px, y0, px, y0 + (major ? 12 : 6), x->lw > 1 ? x->lw - 1 : 1, C_HUD);
        if (major) {
            int v = ((d % 360) + 360) % 360;
            char lab[8];
            if (has_heading && v % 90 == 0)
                snprintf(lab, sizeof lab, "%c", "NESW"[v / 90]);
            else
                snprintf(lab, sizeof lab, "%03d", v);
            draw_text(c, px - draw_text_width(x->ts, lab) / 2, y0 - 9 * x->ts, x->ts, C_HUD, lab);
        }
    }
    draw_line(c, cx, y0 + 14, cx - 7, y0 + 24, x->lw, C_HUD);
    draw_line(c, cx, y0 + 14, cx + 7, y0 + 24, x->lw, C_HUD);
    if (!has_heading)
        draw_text(c, cx + tw / 2 + 30 * x->ts, y0 - 4 * x->ts, x->ts, C_HUD, "REL");
    for (int i = 0; i < n_marks; i++) {
        float d = wrap180(mark_bearing[i] - (has_heading ? centre : 0));
        if (fabsf(d) > 30)
            continue;
        draw_diamond(c, cx + (int)lroundf(d * ppd), y0 + 32, 5, 1, track_colour(x, &marks[i]));
    }
}

/* ---- banners and status line ---- */

static void banner(ctx *x, int *y, int bs, const char *s)
{
    int w = draw_text_width(bs, s), bx = (x->c->w - w) / 2;
    draw_fill(x->c, bx - 8, *y - 4, w + 16, 7 * bs + 8, C_BLACK);
    draw_text(x->c, bx, *y, bs, C_AMBER, s);
    *y += 10 * bs;
}

static void draw_banners(ctx *x)
{
    const seeker_tracks *t = x->st->t;
    int y = 70 * x->ts / 2 + 20, bs = x->ts + 1;
    if (!strcmp(x->st->background, "wearer")) {
        static const char *const plain[] = { "VIDEO OFF", "NO VIDEO", "VIDEO LOST" };
        /* EagleEye runs on black without video (visor), so only a dropped or broken stream from
         * a connected bridge is news there. */
        int eagle = x->st->wearer != NULL && !x->st->wearer->cfg.radar_fallback;
        int why = x->st->video_why;
        if (!VIDEO_SHOWN(x->st->video_state) &&
            (!eagle || (x->st->video_state == VIDEO_LOST && !x->st->wearer->cfg.visor)))
            banner(x, &y, bs, why != VIDEO_WHY_OK ? video_why_text(why) : plain[x->st->video_state]);
        if (x->st->video_state == VIDEO_STALE && x->st->video) {
            /* Small tag, not a banner: the last picture stays up (dimmed) through Wi-Fi gaps. */
            char tag[48];
            long age = (long)(seeker_now_ms() - x->st->video->t_rx_ms);
            snprintf(tag, sizeof tag, "VIDEO STALE %ld.%ld S", age / 1000, age % 1000 / 100);
            int tw = draw_text_width(x->ts, tag), tx = x->c->w - tw - 16, ty = 10;
            draw_fill(x->c, tx - 6, ty - 4, tw + 12, 7 * x->ts + 8, C_BLACK);
            draw_text(x->c, tx, ty, x->ts, C_AMBER, tag);
        }
    }
    if (x->stale)
        banner(x, &y, bs, "FUSION LOST");
    for (int i = 0; t && i < t->n_nodes; i++) {
        if (strcmp(t->nodes[i].state, "lost") != 0)
            continue;
        char s[32];
        snprintf(s, sizeof s, "NODE %s LOST", t->nodes[i].node_id);
        banner(x, &y, bs, s);
    }
}

static void draw_light(ctx *x, int *px, int y, const char *label, const char *proc)
{
    seeker_health_table *h = x->st->health;
    uint32_t col = C_GREY;
    for (int i = 0; h && i < SEEKER_HEALTH_SLOTS; i++) {
        if (strcmp(h->slot[i].name, proc) != 0)
            continue;
        int fresh = seeker_now_ms() - h->slot[i].last_beat_ms < 500;
        col = (fresh && h->slot[i].state == SH_OK) ? C_GREEN : C_RED;
    }
    draw_fill(x->c, *px, y, 8 * x->ts / 2 + 4, 8 * x->ts / 2 + 4, col);
    *px += 8 * x->ts / 2 + 8;
    *px = draw_text(x->c, *px, y, x->ts, col, label) + 6 * x->ts;
}

static void draw_status(ctx *x)
{
    canvas *c = x->c;
    const hud_state *st = x->st;
    int y = c->h - 12 * x->ts;
    char s[160], fus[24], vid[24];
    if (st->t && !x->stale)
        snprintf(fus, sizeof fus, "%dMS", st->tracks_age_ms);
    else
        snprintf(fus, sizeof fus, "--");
    if (st->video_state == VIDEO_OK)
        snprintf(vid, sizeof vid, "  VID %.0fFPS", st->video_fps);
    else if (st->video_state == VIDEO_STALE)
        snprintf(vid, sizeof vid, "  VID STALE");
    else if (st->video_state == VIDEO_OFF)
        vid[0] = '\0';
    else if (st->video_why == VIDEO_WHY_OK)
        snprintf(vid, sizeof vid, "  VID --");
    else {
        char up[16];
        snprintf(up, sizeof up, "%s", video_why_name(st->video_why));
        for (char *p = up; *p; p++)
            *p = (char)(*p >= 'a' && *p <= 'z' ? *p - 'a' + 'A' : *p);
        snprintf(vid, sizeof vid, "  VID %s", up);
    }
    if (VIDEO_SHOWN(st->video_state) && !strcmp(st->background, "wearer"))
        draw_fill(c, 0, y - 3 * x->ts, c->w, c->h - y + 3 * x->ts, C_BLACK);
    snprintf(s, sizeof s, "FUS->HUD %s%s  TRK %d  %.0fFPS  %s  PRIO %d", fus, vid,
             st->t ? st->t->n : 0, st->fps, !strcmp(st->background, "wearer") ? "WEARER" : "RADAR",
             st->hud_prio);
    int px = draw_text(c, 10, y, x->ts, C_HUD, s) + 12 * x->ts;
    draw_light(x, &px, y, "FUS", "hud_fusion");
    draw_light(x, &px, y, "HUD", "hud_display");
    draw_light(x, &px, y, "SUP", "hud_supervisor");
    for (int i = 0; st->t && i < st->t->n_nodes; i++) {
        const seeker_node_status *n = &st->t->nodes[i];
        char ns[32];
        int held = !strcmp(n->pose_src, "held") || !strcmp(n->pose_src, "heading") ||
                   !strcmp(n->pose_src, "home"); /* no tag since a live pose (fusion step_pose) */
        int start = !strcmp(n->pose_src, "init"); /* unpinned, no tag fix yet: starting pose */
        if (!strcmp(n->state, "ok"))
            snprintf(ns, sizeof ns, "%s %.0fHZ%s", n->node_id, n->rate_hz,
                     held ? " HOLD" : start ? " START" : "");
        else
            snprintf(ns, sizeof ns, "%s %s", n->node_id, !strcmp(n->state, "lost") ? "LOST" : "--");
        uint32_t col = !strcmp(n->state, "ok") ? (held || start ? C_AMBER : C_HUD)
                       : !strcmp(n->state, "lost") ? C_AMBER
                                                   : C_GREY;
        px = draw_text(c, px, y, x->ts, col, ns) + 6 * x->ts;
    }
}

/* ---- wearer view: IF-5.6 video with symbology in the view node's frame ---- */

static void draw_video(canvas *c, const video_frame *f)
{
    if (f->w == c->w && f->h == c->h && f->stride == c->stride) {
        memcpy(c->px, f->px, (size_t)c->stride * (size_t)c->h * 4);
        return;
    }
    draw_clear(c, C_BLACK);
    int w = f->w < c->w ? f->w : c->w, h = f->h < c->h ? f->h : c->h;
    for (int y = 0; y < h; y++)
        memcpy(c->px + (size_t)y * c->stride, f->px + (size_t)y * f->stride, (size_t)w * 4);
}

static void draw_wearer(ctx *x, const video_frame *f)
{
    canvas *c = x->c;
    const seeker_tracks *t = x->st->t;
    /* Symbology follows the node whose picture is on screen; the relay picks it (view_node). */
    const char *view = f->node_id[0] ? f->node_id : x->st->view_node;
    const seeker_node_status *v = node_status(t, view);
    const seeker_node_cfg *vc = seeker_config_node(x->cfg, view);
    float hfov = v && v->hfov_deg > 1 ? v->hfov_deg : vc && vc->hfov_deg > 1 ? vc->hfov_deg : 66.0f;
    int rx = f->rect[0], ry = f->rect[1], rw = f->rect[2], rh = f->rect[3];
    int cx = rx + rw / 2, cy = ry + rh / 2;
    /* IF-1 canonical bearing: x = cx + f * tan(bearing), f = (w / 2) / tan(hfov / 2). */
    float focal = (float)rw * 0.5f / tanf(hfov * 0.5f * DEG2RAD);

    draw_video(c, f);
    if (x->st->video_state == VIDEO_STALE)
        draw_dim(c, rx, ry, rw, rh);
    char cam[32];
    snprintf(cam, sizeof cam, "CAM %s", view);
    draw_text(c, rx + 10, ry + 70 * x->ts / 2 + 20, x->ts, C_HUD, cam);

    draw_line(c, cx - 14, cy, cx + 14, cy, x->lw, C_HUD);
    draw_line(c, cx, cy - 14, cx, cy + 14, x->lw, C_HUD);

    float marks_b[SEEKER_MAX_TRACKS];
    seeker_track marks[SEEKER_MAX_TRACKS];
    int nm = 0;
    for (int i = 0; t && i < t->n; i++) {
        const seeker_track *k = &t->tracks[i];
        uint32_t col = track_colour(x, k);
        char lab[48];
        int from_view = !strcmp(k->node_id, view);
        float b, r = 0;
        int has_r = 0;
        if (k->has_room && v) {
            b = wrap180(atan2f(k->room_x_m - v->x_m, k->room_y_m - v->y_m) * RAD2DEG - v->yaw_deg);
            r = hypotf(k->room_x_m - v->x_m, k->room_y_m - v->y_m);
            has_r = 1;
        } else if (from_view) {
            b = k->node_bearing_deg;
        } else {
            continue;
        }
        marks[nm] = *k;
        marks_b[nm++] = b;
        range_text(lab, sizeof lab, k, has_r, r);
        /* node_box is in the view node's pixels, the same frame as the JPEG on screen (phone.html
         * detects on the frames it streams). */
        if (from_view && k->has_node_box && f->src_w > 0 && f->src_h > 0) {
            float sx = (float)rw / f->src_w, sy = (float)rh / f->src_h;
            int bx = rx + (int)(k->node_box[0] * sx), by = ry + (int)(k->node_box[1] * sy);
            int bw = (int)(k->node_box[2] * sx), bh = (int)(k->node_box[3] * sy);
            draw_brackets(c, bx, by, bw, bh, x->lw + 1, col);
            draw_text(c, bx, by + bh + 6, x->ts, col, lab);
        } else if (fabsf(b) < hfov * 0.5f) {
            int px = cx + (int)lroundf(focal * tanf(b * DEG2RAD));
            draw_diamond(c, px, cy, 12, x->lw, col);
            draw_text(c, px - 6 * x->ts, cy - 12 - 10 * x->ts, x->ts, col, "R");
            draw_text(c, px + 16, cy + 6, x->ts, col, lab);
        } else {
            int right = b > 0, ex = right ? rx + rw - 24 : rx + 24;
            int d = right ? -1 : 1;
            draw_line(c, ex, cy, ex + d * 16, cy - 16, x->lw + 1, col);
            draw_line(c, ex, cy, ex + d * 16, cy + 16, x->lw + 1, col);
            int tw = draw_text_width(x->ts, lab);
            draw_text(c, right ? ex - tw - 24 : ex + 24, cy + 22, x->ts, col, lab);
        }
    }
    draw_tape(x, marks, nm, marks_b);

    int R = c->h / 6, icx = c->w - R - 24, icy = c->h - R - 24 - 12 * x->ts;
    draw_fill(c, icx - R - 6, icy - R - 6, 2 * R + 12, 2 * R + 12, C_BLACK);
    draw_rect(c, icx - R - 6, icy - R - 6, 2 * R + 12, 2 * R + 12, 1, C_DIM);
    draw_radar(x, icx, icy, R, 0);
}

/* ---- plan theme (draw_theme() == DRAW_THEME_PLAN) ---- */

/* Full-screen plan when there is no camera picture: the same drawing as the corner inset, larger,
 * with each person's distance from the wearer. */
static void plan_full(ctx *x, float u, const char *camera)
{
    canvas *c = x->c;
    const hud_state *st = x->st;
    const seeker_tracks *t = st->t;
    draw_clear(c, P_GROUND);
    /* Centre: like classic's radar, fusion's wearer node (tracks.wearer_node) with fusion's
     * wearer yaw; else the EagleEye wearer's pose; else the room origin. */
    wv_pose pose = { 0, 0, t ? t->wearer_yaw_deg : 0 };
    const char *node = NULL;
    float hfov = 66.0f;
    const wv_config *wc = NULL;
    int live = 0;
    wv_pose_mode mode = WV_MODE_FIXED;
    char state[8];
    const seeker_node_status *self = NULL;
    if (st->wearer) {
        wc = &st->wearer->cfg;
        node = wc->node;
        hfov = wc->cam.hfov_deg;
        wv_resolve_pose(wc, t, &pose, &live, state, sizeof state);
        mode = wv_pose_mode_of(wc, t, live);
        self = node_status(t, node);
        if (!wc->hfov_fixed && self && self->hfov_deg > 10.0f && self->hfov_deg < 170.0f)
            hfov = self->hfov_deg;
    }
    const seeker_node_status *wn = t && t->wearer_node[0] ? node_status(t, t->wearer_node) : NULL;
    if (wn) {
        pose.x_m = wn->x_m;
        pose.y_m = wn->y_m;
        pose.yaw_deg = t->wearer_yaw_deg;
        if (!node)
            node = t->wearer_node;
    }
    static plan_mark marks[SEEKER_MAX_TRACKS];
    int nm = 0;
    for (int i = 0; t && i < t->n && nm < SEEKER_MAX_TRACKS; i++) {
        const seeker_track *k = &t->tracks[i];
        plan_mark *m = &marks[nm];
        memset(m, 0, sizeof *m);
        m->id = k->id;
        int seen = node ? wv_seen_by(k->nodes, k->node_id, node) : 1;
        if (k->has_room) {
            float d = hypotf(k->room_x_m - pose.x_m, k->room_y_m - pose.y_m); /* from the wearer */
            int down = k->posture == SEEKER_POSTURE_DOWN;
            m->kind = seen ? PLAN_SEEN : PLAN_HIDDEN;
            m->close = !x->stale && (down || (seen && d < x->cfg->near_m));
            m->down = !x->stale && down;
            m->x = k->room_x_m;
            m->y = k->room_y_m;
            snprintf(m->label, sizeof m->label, "%.1f m%s", d,
                     down ? ", down" : k->posture == SEEKER_POSTURE_CROUCH ? ", crouching" : "");
        } else {
            const seeker_node_status *n = node_status(t, k->node_id);
            if (!n)
                continue;
            m->kind = PLAN_RAY;
            m->x = n->x_m;
            m->y = n->y_m;
            m->ray_deg = n->yaw_deg + k->node_bearing_deg;
        }
        nm++;
    }
    int m24 = (int)lroundf(24 * u), tb = plan_title_h(u);
    int x0 = m24, y0 = m24, w = c->w - 2 * m24, h = c->h - 2 * m24 - tb - (int)lroundf(16 * u);
    float ppm = ((float)h * 0.72f - 40 * u) / 6.0f; /* about 6 m ahead fits */
    plan_map(c, u, x0, y0, w, h, ppm, 0.72f, 0, &pose, node, hfov, wc, t, marks, nm);
    draw_ui_text_top(c, x0, y0, (int)lroundf(19 * u), DRAW_FONT_SANS_SEMIBOLD, P_INK, 255, "Plan");
    static const char *const pos_word[] = { "Fixed", "Live", "Held", "Starting" };
    const char *src = self ? self->pose_src : "";
    const char *pos = mode == WV_MODE_HELD && !strcmp(src, "heading") ? "Compass"
                      : mode == WV_MODE_HELD && !strcmp(src, "home") ? "Easing home"
                                                                      : pos_word[mode];
    plan_title_block(c, u, camera, 0, t, node, pos, st->lat_text);
}

/* Centre-top plates: why the picture is missing, fusion and phones lost. */
static void plan_banners(ctx *x, float u, int wv_radar, int eagle_view)
{
    const hud_state *st = x->st;
    const seeker_tracks *t = st->t;
    int y = (int)lroundf(24 * u), gap = (int)lroundf(8 * u);
    if (!strcmp(st->background, "wearer")) {
        int eagle = st->wearer != NULL && !st->wearer->cfg.radar_fallback;
        int frozen = st->wearer && st->video_state == VIDEO_LOST && st->video_frozen;
        if (!VIDEO_SHOWN(st->video_state) &&
            (!eagle || (st->video_state == VIDEO_LOST && !st->wearer->cfg.visor)))
            y += plan_banner(x->c, u, y,
                             frozen     ? "Video lost, last picture shown"
                             : wv_radar ? "Camera paused, showing plan"
                                        : "No camera picture",
                             plan_video_why(st->video_why), P_ALERT, P_INK) + gap;
    }
    if (x->stale)
        y += plan_banner(x->c, u, y, "Fusion lost", "no tracks for over half a second", P_ALERT, P_ALERT) + gap;
    for (int i = 0; t && i < t->n_nodes; i++) {
        if (strcmp(t->nodes[i].state, "lost") != 0)
            continue;
        if (eagle_view && st->wearer && !strcmp(t->nodes[i].node_id, st->wearer->cfg.node))
            continue; /* the wearer view says it itself */
        char s[32];
        snprintf(s, sizeof s, "%s lost", t->nodes[i].node_id);
        y += plan_banner(x->c, u, y, s, "", P_ALERT, P_ALERT) + gap;
    }
}

int hud_render(canvas *c, const seeker_config *cfg, const hud_state *st)
{
    ctx x = { c, cfg, st, c->h >= 600 ? 2 : 1, cfg->line_px > 0 ? cfg->line_px : 2, 0 };
    x.stale = !st->t || st->tracks_age_ms > FUSION_STALE_MS;
    int drawn = 0;
    /* The one frame time every plan animation runs on (hud_state.now_ms, 0 = none). */
    anim_alert_frame(st->now_ms);
    anim_map_frame(st->now_ms);

    int shown = VIDEO_SHOWN(st->video_state);
    /* EagleEye only: a brief stall keeps the last picture (dark) instead of flipping to radar. */
    int frozen = st->wearer && !shown && st->video_state == VIDEO_LOST && st->video_frozen;
    int wv_radar = st->wearer && st->wearer->cfg.radar_fallback && !st->wearer->cfg.visor &&
                   !shown && !frozen;
    if (st->wearer) {
        st->wearer->last.rendered = 0; /* wv_render sets it */
        st->wearer->video_stale = frozen ? 2 : st->video_state == VIDEO_STALE;
    }
    if (draw_theme() == DRAW_THEME_PLAN) {
        float u = plan_u(c);
        int eagle_view = !strcmp(st->background, "wearer") && st->wearer && !wv_radar;
        if (eagle_view) {
            st->wearer->delay_text = st->lat_text; /* the title block's Delay cell */
            if (st->now_ms > 0)
                st->wearer->clock_ms = st->now_ms; /* the person animations' frame time, too */
            wv_render(c, cfg, st->wearer, st->t, x.stale, shown || frozen ? st->video : NULL);
            st->wearer->delay_text = NULL;
            drawn = st->wearer->last.video > 0;
        } else if (!strcmp(st->background, "wearer") && shown && st->video) {
            draw_wearer(&x, st->video); /* no-config fallback view keeps the classic marks */
            plan_title_block(c, u, "Live", st->video_state == VIDEO_STALE, st->t, st->view_node, "Fixed",
                             st->lat_text);
            drawn = 1;
        } else {
            plan_full(&x, u, !strcmp(st->background, "wearer") ? "Camera paused" : "Plan view");
        }
        plan_banners(&x, u, wv_radar, eagle_view);
        anim_alert_warn_flush(c, u); /* warning plates sliding out */
        return drawn;
    }
    if (!strcmp(st->background, "wearer") && st->wearer && !wv_radar) {
        wv_render(c, cfg, st->wearer, st->t, x.stale, shown || frozen ? st->video : NULL);
        drawn = st->wearer->last.video > 0;
    } else if (!strcmp(st->background, "wearer") && shown && st->video) {
        draw_wearer(&x, st->video);
        drawn = 1;
    } else {
        draw_clear(c, C_BLACK);
        int top = 70 * x.ts / 2 + 10, bottom = 16 * x.ts;
        int R = (c->h - top - bottom) / 2 - 10;
        draw_radar(&x, c->w / 2, top + 10 + R, R, 1);
        float b[SEEKER_MAX_TRACKS];
        int nm = 0;
        seeker_track marks[SEEKER_MAX_TRACKS];
        for (int i = 0; st->t && i < st->t->n; i++)
            if (st->t->tracks[i].has_bearing) {
                marks[nm] = st->t->tracks[i];
                b[nm++] = st->t->tracks[i].bearing_deg;
            }
        draw_tape(&x, marks, nm, b);
    }
    draw_banners(&x);
    draw_status(&x);
    if (st->lat_text && st->lat_text[0]) { /* right end of the status line */
        int tw = draw_text_width(x.ts, st->lat_text);
        int lx = c->w - tw - 10, ly = c->h - 12 * x.ts;
        draw_fill(c, lx - 8, ly - 3 * x.ts, tw + 18, 12 * x.ts, C_BLACK);
        draw_text(c, lx, ly, x.ts, C_HUD, st->lat_text);
    }
    return drawn;
}

void hud_frame_note(char *out, size_t n, long frame, int video_state, int drawn,
                    const video_frame *f)
{
    int have = f && f->px;
    snprintf(out, n, "seeker hud frame=%ld video=%s drawn=%d node=%s seq=%lu", frame,
             video_state_name(video_state), drawn, have && f->node_id[0] ? f->node_id : "-",
             have ? (unsigned long)f->seq : 0ul);
}
