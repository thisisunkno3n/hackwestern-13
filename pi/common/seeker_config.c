#include "seeker_config.h"
#include "seeker_json.h"
#include "seeker_log.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CFG_MAX_BYTES (64 * 1024)
#define CFG_MAX_TOKS 8192

void seeker_config_defaults(seeker_config *c)
{
    memset(c, 0, sizeof *c);
    c->n_nodes = 2;
    c->nodes[0] = (seeker_node_cfg){ "R1", -2.0f, 3.0f, 135.0f, 66.0f, 640, 480 };
    c->nodes[1] = (seeker_node_cfg){ "R2", 2.0f, 3.0f, 225.0f, 66.0f, 640, 480 };
    c->drop_ms = 1500;
    c->assoc_m = 0.8f;
    c->near_m = 2.0f;
    c->mid_m = 5.0f;
    c->closing_mps = 0.5f;
    c->stale_ms = 500;
    snprintf(c->sink, sizeof c->sink, "screen");
    snprintf(c->background, sizeof c->background, "radar");
    snprintf(c->theme, sizeof c->theme, "plan");
    c->anim = 1;
    snprintf(c->view_node, sizeof c->view_node, "R1");
    snprintf(c->mirror, sizeof c->mirror, "none");
    c->line_px = 2;
    c->radar_m = 8.0f;
    c->width = 1280;
    c->height = 720;
    c->fps = 30;
    c->fusion_rate_hz = 30.0f;
    c->smooth = 1;
    c->smooth_alpha = 0.5f;
    c->smooth_beta = 0.15f;
    c->smooth_gate_m = 1.5f;
    c->smooth_max_dt_ms = 250;
    c->smooth_reset_ms = 500;
    c->predict_ms = 100;
    c->pose_home_ms = 10000;
    c->range_smooth = 1;
    c->range_smooth_alpha = 0.2f;
    c->range_smooth_beta = 0.015f;
    c->range_smooth_gate_frac = 0.65f;
    c->tri_bearing_sigma_deg = 3.0f; /* keep in step with seeker_tri_defaults() */
    c->tri_range_sigma_frac = 0.20f;
    c->tri_range_sigma_k = 0.05f;
    c->tri_min_cross_deg = 15.0f;
    c->tri_min_cross_bearing_only_deg = 5.0f;
    c->tri_track_prior_sigma_m = 0.5f;
    c->tri_track_prior_cap = 9.0f;
    c->track_sigma_m = 0.3f;
    c->track_gate_sigmas = 3.0f;
    c->track_gate_max_m = 3.0f;
    snprintf(c->tracks_host, sizeof c->tracks_host, "127.0.0.1");
    c->beat_ms = 100;
    c->dead_ms = 500;
    c->backoff_ms = 500;
    c->supervisor_prio = 45;
    snprintf(c->supervisor_policy, sizeof c->supervisor_policy, "fifo");
    c->hud_low = 10;
}

static void get_f(const char *s, const sj_tok *t, int obj, const char *k, float *v)
{
    *v = (float)sj_num(s, t, sj_get(s, t, obj, k), *v);
}

static void get_i(const char *s, const sj_tok *t, int obj, const char *k, int *v)
{
    *v = (int)sj_num(s, t, sj_get(s, t, obj, k), *v);
}

static void get_b(const char *s, const sj_tok *t, int obj, const char *k, int *v)
{
    *v = sj_bool(s, t, sj_get(s, t, obj, k), *v);
}

static void get_s(const char *s, const sj_tok *t, int obj, const char *k, char *v, int sz)
{
    int i = sj_get(s, t, obj, k);
    if (i >= 0 && t[i].type == SJ_STR)
        sj_str(s, t, i, v, sz);
}

static void get_prio(const char *s, const sj_tok *t, int prio, const char *name, int *p,
                     char *policy, int policysz)
{
    int o = sj_get(s, t, prio, name);
    if (o < 0 || t[o].type != SJ_OBJ)
        return;
    get_i(s, t, o, "prio", p);
    get_s(s, t, o, "policy", policy, policysz);
}

static void overlay(seeker_config *c, const char *s, const sj_tok *t)
{
    int remote = sj_get(s, t, 0, "remote");
    if (remote >= 0) {
        get_i(s, t, remote, "drop_ms", &c->drop_ms);
        get_f(s, t, remote, "assoc_m", &c->assoc_m);
        char ps[16] = "";
        get_s(s, t, remote, "pose_source", ps, sizeof ps);
        if (ps[0])
            c->pose_from_config = strcmp(ps, "config") == 0;
        int arr = sj_get(s, t, remote, "nodes");
        if (arr >= 0 && t[arr].type == SJ_ARR) {
            c->n_nodes = 0;
            for (int i = 0; i < t[arr].size && c->n_nodes < SEEKER_MAX_NODES; i++) {
                int o = sj_index(t, arr, i);
                seeker_node_cfg *n = &c->nodes[c->n_nodes];
                *n = (seeker_node_cfg){ "", 0, 0, 0, 66.0f, 640, 480 };
                get_s(s, t, o, "id", n->id, sizeof n->id);
                if (!n->id[0])
                    continue;
                get_f(s, t, o, "x_m", &n->x_m);
                get_f(s, t, o, "y_m", &n->y_m);
                get_f(s, t, o, "yaw_deg", &n->yaw_deg);
                get_f(s, t, o, "hfov_deg", &n->hfov_deg);
                get_i(s, t, o, "image_w", &n->image_w);
                get_i(s, t, o, "image_h", &n->image_h);
                c->n_nodes++;
            }
        }
    }

    int col = sj_get(s, t, 0, "colours");
    if (col >= 0) {
        get_f(s, t, col, "near_m", &c->near_m);
        get_f(s, t, col, "mid_m", &c->mid_m);
        get_f(s, t, col, "closing_mps", &c->closing_mps);
        get_i(s, t, col, "stale_ms", &c->stale_ms);
        get_b(s, t, col, "remote_by_range", &c->remote_by_range);
    }

    int d = sj_get(s, t, 0, "display");
    if (d >= 0) {
        get_s(s, t, d, "sink", c->sink, sizeof c->sink);
        get_s(s, t, d, "background", c->background, sizeof c->background);
        get_s(s, t, d, "theme", c->theme, sizeof c->theme);
        get_b(s, t, d, "anim", &c->anim); /* true / false, or "on" / "off" */
        {
            char a[8] = "";
            get_s(s, t, d, "anim", a, sizeof a);
            if (!strcmp(a, "off"))
                c->anim = 0;
            else if (!strcmp(a, "on"))
                c->anim = 1;
        }
        get_s(s, t, d, "view_node", c->view_node, sizeof c->view_node);
        get_s(s, t, d, "mirror", c->mirror, sizeof c->mirror);
        get_i(s, t, d, "line_px", &c->line_px);
        get_f(s, t, d, "radar_m", &c->radar_m);
        get_i(s, t, d, "fps", &c->fps);
        get_b(s, t, d, "swap_rb", &c->swap_rb);
        get_i(s, t, d, "display_index", &c->display_index);
        int sz = sj_get(s, t, d, "size");
        if (sz >= 0 && t[sz].type == SJ_ARR && t[sz].size == 2) {
            c->width = (int)sj_num(s, t, sj_index(t, sz, 0), c->width);
            c->height = (int)sj_num(s, t, sj_index(t, sz, 1), c->height);
        }
    }

    int f = sj_get(s, t, 0, "fusion");
    if (f >= 0) {
        get_f(s, t, f, "rate_hz", &c->fusion_rate_hz);
        get_s(s, t, f, "tracks_host", c->tracks_host, sizeof c->tracks_host);
        get_f(s, t, f, "wearer_yaw_deg", &c->wearer_yaw_deg);
        get_s(s, t, f, "wearer_node", c->wearer_node, sizeof c->wearer_node);
        get_i(s, t, f, "predict_ms", &c->predict_ms);
        get_i(s, t, f, "pose_home_ms", &c->pose_home_ms);
        get_b(s, t, f, "smooth", &c->smooth);
        get_f(s, t, f, "smooth_alpha", &c->smooth_alpha);
        get_f(s, t, f, "smooth_beta", &c->smooth_beta);
        get_f(s, t, f, "smooth_gate_m", &c->smooth_gate_m);
        get_i(s, t, f, "smooth_max_dt_ms", &c->smooth_max_dt_ms);
        get_i(s, t, f, "smooth_reset_ms", &c->smooth_reset_ms);
        get_b(s, t, f, "range_smooth", &c->range_smooth);
        get_f(s, t, f, "range_smooth_alpha", &c->range_smooth_alpha);
        get_f(s, t, f, "range_smooth_beta", &c->range_smooth_beta);
        get_f(s, t, f, "range_smooth_gate_frac", &c->range_smooth_gate_frac);
        get_f(s, t, f, "tri_bearing_sigma_deg", &c->tri_bearing_sigma_deg);
        get_f(s, t, f, "tri_range_sigma_frac", &c->tri_range_sigma_frac);
        get_f(s, t, f, "tri_range_sigma_k", &c->tri_range_sigma_k);
        get_f(s, t, f, "tri_min_cross_deg", &c->tri_min_cross_deg);
        get_f(s, t, f, "tri_min_cross_bearing_only_deg", &c->tri_min_cross_bearing_only_deg);
        get_f(s, t, f, "tri_track_prior_sigma_m", &c->tri_track_prior_sigma_m);
        get_f(s, t, f, "tri_track_prior_cap", &c->tri_track_prior_cap);
        get_f(s, t, f, "track_sigma_m", &c->track_sigma_m);
        get_f(s, t, f, "track_gate_sigmas", &c->track_gate_sigmas);
        get_f(s, t, f, "track_gate_max_m", &c->track_gate_max_m);
    }

    int h = sj_get(s, t, 0, "health");
    if (h >= 0) {
        get_i(s, t, h, "beat_ms", &c->beat_ms);
        get_i(s, t, h, "dead_ms", &c->dead_ms);
        get_i(s, t, h, "backoff_ms", &c->backoff_ms);
    }

    int prio = sj_get(s, t, 0, "prio");
    if (prio >= 0) {
        get_prio(s, t, prio, "hud_supervisor", &c->supervisor_prio, c->supervisor_policy,
                 sizeof c->supervisor_policy);
        get_i(s, t, prio, "hud_low", &c->hud_low);
    }

    int sup = sj_get(s, t, 0, "supervisor");
    int kids = sup >= 0 ? sj_get(s, t, sup, "children") : -1;
    if (kids >= 0 && t[kids].type == SJ_ARR) {
        c->n_children = 0;
        for (int i = 0; i < t[kids].size && c->n_children < SEEKER_MAX_CHILDREN; i++) {
            int o = sj_index(t, kids, i);
            seeker_child_cfg *k = &c->children[c->n_children];
            memset(k, 0, sizeof *k);
            snprintf(k->policy, sizeof k->policy, "other");
            get_s(s, t, o, "name", k->name, sizeof k->name);
            int av = sj_get(s, t, o, "argv");
            if (!k->name[0] || av < 0 || t[av].type != SJ_ARR || t[av].size < 1)
                continue;
            for (int a = 0; a < t[av].size && k->argc < SEEKER_MAX_ARGS - 1; a++)
                sj_str(s, t, sj_index(t, av, a), k->argv[k->argc++], sizeof k->argv[0]);
            if (prio >= 0)
                get_prio(s, t, prio, k->name, &k->prio, k->policy, sizeof k->policy);
            c->n_children++;
        }
    }
}

int seeker_config_load(seeker_config *c, const char *path)
{
    seeker_config_defaults(c);
    FILE *f = fopen(path, "rb");
    if (!f) {
        seeker_log("config %s not readable, using defaults", path);
        return -1;
    }
    char *buf = malloc(CFG_MAX_BYTES);
    sj_tok *t = malloc(sizeof(sj_tok) * CFG_MAX_TOKS);
    int rc = -1;
    if (buf && t) {
        int n = (int)fread(buf, 1, CFG_MAX_BYTES, f);
        if (n > 0 && n < CFG_MAX_BYTES && sj_parse(buf, n, t, CFG_MAX_TOKS) > 0 &&
            t[0].type == SJ_OBJ) {
            overlay(c, buf, t);
            rc = 0;
        } else {
            seeker_log("config %s is not valid JSON (or over 64 KB), using defaults", path);
        }
    }
    free(buf);
    free(t);
    fclose(f);
    return rc;
}

const seeker_node_cfg *seeker_config_node(const seeker_config *c, const char *id)
{
    for (int i = 0; i < c->n_nodes; i++)
        if (strcmp(c->nodes[i].id, id) == 0)
            return &c->nodes[i];
    return NULL;
}
