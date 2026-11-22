#ifndef SEEKER_CONFIG_H
#define SEEKER_CONFIG_H

/*
 * IF-8 config/seeker.json, only the keys the Pi lane reads after the pivot. Every key has a
 * default here, so a missing or partial file still runs. Keys marked PROPOSED are not in IF-8 yet.
 */

#include "seeker_msgs.h"

#define SEEKER_MAX_CHILDREN 8
#define SEEKER_MAX_ARGS 12

typedef struct {
    char id[SEEKER_NODE_LEN];
    float x_m, y_m, yaw_deg;
    float hfov_deg;     /* PROPOSED: for the radar view cone */
    int image_w, image_h; /* PROPOSED: node pixel size, to scale `box` (IF-5.4 does not carry it) */
} seeker_node_cfg;

typedef struct {
    char name[32];
    char argv[SEEKER_MAX_ARGS][128];
    int argc;
    int prio;           /* from the prio section, 0 = leave as inherited */
    char policy[8];     /* "fifo", "rr", "other" */
} seeker_child_cfg;

typedef struct {
    /* remote */
    int n_nodes;
    seeker_node_cfg nodes[SEEKER_MAX_NODES];
    int drop_ms;        /* IF-5.4: 1500 */
    float assoc_m;      /* PROPOSED: merge R1 and R2 contacts closer than this in the room */
    int pose_from_config; /* PROPOSED: 1 = ignore the message pose, use remote.nodes instead */

    /* colours (IF-7) */
    float near_m, mid_m, closing_mps;
    int stale_ms;
    int remote_by_range; /* PROPOSED: colour remote contacts by range band instead of cyan */

    /* display */
    char sink[16];        /* "screen", "ppm", "none" */
    char background[16];  /* PROPOSED: "radar" (default) or "wearer" */
    char view_node[SEEKER_NODE_LEN]; /* PROPOSED: phone used as the wearer view */
    char mirror[8];        /* "none", "h", "v" */
    int line_px;
    float radar_m;        /* PROPOSED: radius of the radar map */
    int width, height;    /* used when the sink cannot report a size */
    int fps;
    int swap_rb;          /* PROPOSED: flip R and B if Screen's byte order differs on the Pi */
    int display_index;    /* PROPOSED: index into SCREEN_PROPERTY_DISPLAYS (0 is assumed HDMI0) */
    char theme[12];       /* "plan" (default) or "classic" (hud_display --theme) */

    /* fusion (PROPOSED section) */
    float fusion_rate_hz;
    char tracks_host[64];
    float wearer_yaw_deg;
    char wearer_node[SEEKER_NODE_LEN]; /* fusion.wearer_node: track range, bearing and colour
                                        * from this node's pose; "" = the room origin facing
                                        * wearer_yaw_deg (the old taped-origin wearer) */
    int predict_ms; /* fusion.predict_ms: tracks moved on by their velocity to each tick, at
                     * most this far past their newest datagram (0 = off) */
    int pose_home_ms; /* fusion.pose_home_ms: a moving phone with a home (--init-pose) and no
                       * live pose for this long eases back home (0 = never) */
    /* Alpha-beta smoothing of fused track positions (fusion.smooth*, hud_fusion --smooth-*). */
    int smooth;
    float smooth_alpha, smooth_beta; /* position and velocity gains, 0..1 */
    float smooth_gate_m;             /* a jump farther than this from the prediction snaps */
    int smooth_max_dt_ms;            /* dt between measurements is clamped to this */
    int smooth_reset_ms;             /* a track absent this long restarts from its next fix */
    /* Range filter for single-node (mono range) tracks, applied instead of the room filter. Gains
     * are per datagram (tuned at 15 Hz); off when smooth is off. */
    int range_smooth;
    float range_smooth_alpha, range_smooth_beta;
    float range_smooth_gate_frac; /* snap gate: max(smooth_gate_m, frac * predicted range) */
    /* Triangulation weights (fusion.tri_*, seeker_triangulate.h). Mono range sigma is
     * sqrt((frac * r)^2 + (k * r^2)^2). */
    float tri_bearing_sigma_deg;
    float tri_range_sigma_frac;
    float tri_range_sigma_k;
    float tri_min_cross_deg;              /* below this crossing angle, use the size ranges */
    float tri_min_cross_bearing_only_deg; /* same, when no ray has a range */
    float tri_track_prior_sigma_m; /* grouping prefers pairings near predicted tracks */
    float tri_track_prior_cap;     /* chi-square cap of that prior term */
    /* Track identity: fixes are assigned to live tracks (matched within drop_ms) by predicted
     * position. Gate: max(smooth_gate_m, min(track_gate_sigmas * s, track_gate_max_m)), where
     * s^2 = track_sigma_m^2 + fix sigma^2. */
    float track_sigma_m;
    float track_gate_sigmas;
    float track_gate_max_m;

    /* health (IF-6) */
    int beat_ms, dead_ms, backoff_ms;

    /* supervisor (PROPOSED section) and prio (IF-3) */
    int n_children;
    seeker_child_cfg children[SEEKER_MAX_CHILDREN];
    int supervisor_prio;
    char supervisor_policy[8];
    int hud_low;
} seeker_config;

void seeker_config_defaults(seeker_config *c);
/* Loads defaults, then overlays the file. Returns 0, or -1 if the file is unreadable or invalid
 * (defaults stay in place and a reason is logged). */
int seeker_config_load(seeker_config *c, const char *path);
const seeker_node_cfg *seeker_config_node(const seeker_config *c, const char *id);

#endif
