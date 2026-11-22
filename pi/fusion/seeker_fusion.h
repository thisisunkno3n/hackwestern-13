#ifndef SEEKER_FUSION_H
#define SEEKER_FUSION_H

/*
 * hud_fusion core, kept free of sockets so tests can drive it with fake clocks.
 * Tracks each node_id separately and marks a node LOST after cfg->drop_ms (1500) without a
 * datagram, by receive time (IF-5.4). Each contact becomes a ray from its node's pose; rays from
 * different nodes that meet are triangulated into one track (seeker_triangulate.h), and the fix is
 * placed in the room frame (IF-1) and then in the wearer's frame. A lone contact without a range
 * keeps only its node bearing.
 *
 * Track identity: every tick, fixes are assigned to live tracks (output within the last drop_ms)
 * by predicted position (alpha-beta velocity, gated, minimum total cost, one fix per track), so
 * two people who pass close by keep their ids even when the phones' contact labels swap. A track
 * with no fix this tick coasts (kept, not sent) for drop_ms. A fix no live track takes reuses the
 * track that last had its label (so a node back from LOST keeps its ids), else gets a new id.
 * Two contacts of one node are never fused into one track.
 *
 * Skeleton: a track whose primary contact carries pts (phone.html, PR #7) gets them in
 * person-height units from the mid-hip, scaled by the smoothed distance from that phone, and
 * smoothed per joint in its track memory (seeker_skel.h). Only placed tracks (with a room fix)
 * carry one; the encoder caps how many go out per datagram.
 *
 * Smoothing (fusion.smooth): a triangulated or parallel-fallback fix runs through a room-frame
 * alpha-beta filter; a single-node mono range point runs through a range-only filter instead
 * (fusion.range_smooth), so the noisy size range is smoothed along the ray while the bearing,
 * which is already accurate, is not delayed.
 *
 * Node pose precedence: a pinned pose (hud_fusion --pose / --poses) always wins; otherwise a
 * message pose replaces the remote.nodes pose unless remote.pose_source is "config". A message
 * with pose null keeps the last pose (pose_src "held" after a full message pose).
 *
 * Tag frames. Tag 0 is the room origin. A message pose may name the tag it was measured from
 * (pose.tag_id, phone.html ?tag=1&tagid=0,7): tag 0 is the room frame as is, another tag is
 * moved into the room frame with its room pose (seeker_fusion_set_tag, hud_fusion --tag-pose),
 * and a tag with no room pose counts as pose null (the node holds). A pose with no tag_id is in
 * the room frame, unless the node has a tag frame (seeker_fusion_set_tag_frame, hud_fusion
 * --tag-frame: the phone sees only one tag, e.g. ?tagid=7, and does not name it). Either way the
 * room-frame pose then goes through the jump gate and the hold, so a switch between tags whose
 * room poses disagree by more than the gate is taken after SEEKER_POSE_JUMP_N consistent frames.
 * A node with a tag frame is never pinned, and takes message poses even with remote.pose_source
 * "config".
 */

#include "seeker_colour.h"
#include "seeker_config.h"
#include "seeker_msgs.h"
#include "seeker_pose.h"
#include "seeker_posture.h"
#include "seeker_skel.h"
#include "seeker_triangulate.h"

typedef enum { NODE_WAIT = 0, NODE_OK, NODE_LOST } seeker_node_state;

/* Crouch height model. phone.html ranges from the eye (or nose) to mid-ankle height over
 * SEEKER_STAND_EYE_ANKLE_FRAC of the standing height when both show. Crouched (both legs folded,
 * seeker_posture_classify on the contact's own landmarks) that height is
 * SEEKER_CROUCH_EYE_ANKLE_FRAC of the standing height (about 0.48 in a full squat to 0.68 with a
 * knee down), so the range reads 1.3 to 1.9x long: far enough past the other phone's fix that
 * triangulation splits the person in two. Such a range is scaled by the ratio of the two. A
 * contact ranged from the torso (no eyes or ankles at visibility 0.5) needs nothing: the torso
 * does not fold. */
#define SEEKER_STAND_EYE_ANKLE_FRAC 0.90f /* phone.html EYE_ANKLE_FRAC */
#define SEEKER_CROUCH_EYE_ANKLE_FRAC 0.58f
/* Body range (crouch_range_k): shares of a standing height, and the limits of the factor. */
#define SEEKER_BODY_LEG_SEG_FRAC 0.25f  /* hip to knee, knee to ankle */
#define SEEKER_BODY_TORSO_FRAC 0.30f    /* mid-shoulder to mid-hip (phone.html TORSO_FRAC) */
#define SEEKER_BODY_SHOULDER_FRAC 0.24f /* shoulder to shoulder */
#define SEEKER_BODY_STAND_K 0.88f       /* a standing person reads about this */
#define SEEKER_BODY_K_KEEP 0.78f        /* at or over this: the range as sent */
#define SEEKER_BODY_K_FULL 0.62f        /* at or under this: scaled in full */
#define SEEKER_BODY_K_MIN 0.35f
#define SEEKER_BODY_FEET_TOL 0.045f     /* ankles within this share of the picture height: same spot */
#define SEEKER_BODY_SAME 0.85f          /* height under this share of the tallest there: folded */
#define SEEKER_BODY_TRUST_MS 3000       /* how long a reference, or a range from a body length, stands */
#define SEEKER_BODY_REFS 8
typedef struct {
    char id[SEEKER_ID_LEN]; /* the phone's contact id */
    int64_t ms;             /* last message with it (0: free) */
    int has_feet;           /* ankle_y and used are a reference */
    float ankle_y, used;    /* ankle row (share of the height) and the tallest height in px there */
    int64_t trust_ms;       /* last range from a body length, */
    float trust_range;      /* and that range, corrected */
} seeker_body_ref;

typedef struct {
    char id[SEEKER_NODE_LEN];
    seeker_node_state state;
    int64_t last_rx_ms;
    int64_t first_rx_ms;
    long msgs;
    float rate_hz; /* smoothed receive rate */
    float x_m, y_m, yaw_deg, hfov_deg;
    int image_w, image_h;
    seeker_remote_contacts last;
    int pose_pinned;  /* 1: message poses are ignored for this node */
    char pose_src[8]; /* "config", "init", "file", "cli", "message", "partial", "default", or
                       * "held": the node sent a full pose, then pose null (tag out of view);
                       * the last full pose is kept */
    int warned_zero_pose;
    int pose_jumps;   /* consecutive live poses rejected by the jump gate */
    seeker_pose jump_pose; /* the last of them */
    int oe_valid;               /* live-pose One Euro state (pose_smooth) */
    float oe_dx, oe_dy, oe_dyaw; /* filtered speed, m/s and deg/s */
    int64_t oe_ms;
    int has_pitch;         /* the phone sent its camera tilt (pitch_deg), at pitch_ms */
    float pitch_deg;
    int64_t pitch_ms;
    float range_k;         /* multiplies the phone's range_m (hud_fusion --range-scale, FOV rescale) */
    float cam_h, cam_pitch; /* camera height (m, 0: unknown) and tilt (deg, up +) for the
                             * ground-plane range check (hud_fusion --cam-h, --cam-pitch) */
    int64_t ground_log_ms;
    unsigned flip_hist;    /* tag flips: bit i set when live pose i back was the mirror solution */
    int reacq_have;        /* back after a loss: first pose seen, waiting for one that agrees */
    seeker_pose reacq;     /* that pose (room), and when */
    int64_t reacq_ms;
    long reacqs;           /* poses taken back after a loss (logs, tests) */
    long flips;            /* mirror solutions replaced by their mirror (logs) */
    int has_tag_frame;     /* 1: untagged message poses are in tag_frame's tag frame */
    seeker_pose tag_frame; /* room pose of the tag this node's phone locates itself from */
    int last_tag;          /* pose.tag_id of the last pose taken, -1 untagged */
    /* Moving wearer without a tag in view (seeker_fusion_tick): */
    int has_home;          /* home: the --init-pose (WEARER_POSE), eased to after pose_home_ms */
    seeker_pose home;
    int64_t held_ms;       /* when the last live pose was lost */
    int has_head_off;      /* room yaw minus compass heading, learned while tag poses arrive */
    float head_off;
    int blending;          /* easing the pose toward target (back from a hold, or home) */
    int blend_yaw;         /* the blend also moves the yaw (not when the compass drives it) */
    seeker_pose target;
    int64_t blend_ms, blend_start_ms;
    float hfov_set;        /* hud_fusion --hfov: the phone's real horizontal FOV (deg), 0 = trust
                            * the phone; with hfov_preset, landscape and portrait values */
    float hfov_set_portrait;
    int warned_tag;        /* unknown tag id last logged, -1 none */
    int64_t crouch_log_ms; /* last "crouch range" log line */
    long crouch_ranges;    /* contacts ranged with the crouch height (logs, tests) */
    seeker_body_ref body[SEEKER_BODY_REFS]; /* body range references, by contact id */
    /* Occlusion guard (SEEKER_OCCL_*): */
    unsigned char part[SEEKER_MAX_CONTACTS]; /* last message's contacts: 1 shows part of a body,
                                              * 2 whole but held partial, 4 near frag_bearing */
    int64_t frag_ms;       /* when this node last saw part of someone, at frag_bearing */
    float frag_bearing;
    int frag_whole;        /* whole sightings near frag_bearing since then */
    long frag_held;        /* whole sightings taken as partial (logs, tests) */
} seeker_fusion_node;

#define SEEKER_MAX_TAGS 16

typedef struct {
    int id;
    seeker_pose pose; /* the tag's room pose (seeker_pose_from_tag) */
} seeker_fusion_tag;

/* Jump gate on live message poses: a full pose more than SEEKER_POSE_JUMP_M or
 * SEEKER_POSE_JUMP_DEG away from the current live pose is ignored until SEEKER_POSE_JUMP_N of them
 * arrive in a row, each within the same limits of the one before. One-frame flips of the phone's marker pose (planar POSIT picks the mirror
 * solution) then never move the wearer; a real move still lands within 3 messages (~200 ms). */
#define SEEKER_POSE_JUMP_M 0.75f
#define SEEKER_POSE_JUMP_DEG 30.0f
#define SEEKER_POSE_JUMP_N 3
/* A flat tag seen from off its axis has two camera poses that fit the image almost equally well
 * (planar pose ambiguity; js-aruco2 POSIT keeps the "best" one, which flips with noise). On the
 * floor the second one is the first mirrored across the tag's normal, at the same range, with
 * the tag at the same place in the picture: in the tag's frame (x, y, yaw) -> (-x, y, yaw - 2b),
 * b the bearing from the camera to the tag. A live pose that jumped but whose mirror lands
 * within SEEKER_POSE_FLIP_M / SEEKER_POSE_FLIP_DEG of the node is taken as that mirror (the same
 * place, solved the other way). Only when SEEKER_POSE_FLIP_SWITCH of the last
 * SEEKER_POSE_FLIP_WINDOW live poses were mirrors does the node switch to the other solution. */
/* Back from a loss of SEEKER_POSE_REACQ_MS or more (pose null, tag out of view): the filters
 * and flip history start over, and the pose is taken once two in a row agree (directly or as
 * mirrors, within SEEKER_POSE_FLIP_M / SEEKER_POSE_FLIP_DEG), or after SEEKER_POSE_REACQ_WAIT_MS
 * anyway: the first frames with the tag at the picture's edge are the worst. */
#define SEEKER_POSE_REACQ_MS 1000
#define SEEKER_POSE_REACQ_WAIT_MS 600
#define SEEKER_POSE_FLIP_M 0.45f
#define SEEKER_POSE_FLIP_DEG 15.0f
#define SEEKER_POSE_FLIP_WINDOW 10
#define SEEKER_POSE_FLIP_SWITCH 8
/* Live (tag) poses are noisy: a few cm and a few degrees per frame from ArUco at 2 m, and the
 * HUD's wearer-centred map turns that into a shaking room (a phone 4 m away moves 0.2 m per 3
 * deg). One Euro filter on x, y and yaw: a low cutoff standing still, opening with the filtered
 * speed (per m/s, per deg/s) so walking and turning follow. */
#define SEEKER_POSE_MIN_CUTOFF_HZ 0.4f
#define SEEKER_POSE_BETA_M 0.6f     /* Hz per m/s */
#define SEEKER_POSE_BETA_DEG 0.012f /* Hz per deg/s */
#define SEEKER_POSE_D_CUTOFF_HZ 1.0f

/* Constant-velocity alpha-beta filter on a room position (m, m/s). */
typedef struct {
    int valid;
    float x, y, vx, vy;
} seeker_ab;

/* Constant-velocity alpha-beta filter on one node's range to a contact (m, m/s). */
typedef struct {
    int valid;
    float r, v;
} seeker_rf;

typedef struct {
    char label[SEEKER_ID_LEN];
    int id;
    seeker_colour colour; /* range trend, approach speed and threat colour (seeker_colour.h) */
    int64_t seen_ms;
    seeker_ab ab;
    long ab_seq;       /* sum of the source nodes' message counts at the last filter update */
    int64_t ab_msg_ms; /* primary node's message clock (t_mono_ms or ts_ms) at that update */
    int64_t ab_rx_ms;  /* Pi receive time of that update */
    int ab_node;       /* primary node index + 1 at that update (another phone's clock differs) */
    seeker_rf rf;      /* range filter, while the track is a single-node mono range point */
    int pv_valid;      /* glide velocity (fusion.predict_ms): from successive filtered */
    float pvx, pvy;    /* positions, so it covers sideways motion of a one-phone track too */
    float pv_x, pv_y;  /* filtered position at the last update */
    long dbg_seq;      /* message count at the last --debug track line */
    int has_pos;       /* px, py hold its last room position (sent at pos_ms) */
    float px, py;
    int64_t pos_ms;
    int matched;       /* taken by a cluster this tick */
    seeker_skel skel;  /* smoothed ghost skeleton of the primary contact (seeker_skel.h) */
    seeker_posture posture; /* STAND / CROUCH / DOWN from that skeleton (seeker_posture.h) */
    /* Handoff (fusion.handoff, SEEKER_HANDOFF_*): */
    unsigned node_mask; /* nodes (bit = node index) that saw the whole person at the last update */
    float cxx, cxy, cyy; /* position covariance of the last fix (m^2): long along a mono ray */
    int rf_node;        /* node index + 1 the range filter's range is measured from, 0 none */
    float rk[SEEKER_MAX_NODES];      /* per node: fused distance / that node's mono range */
    int64_t rk_ms[SEEKER_MAX_NODES]; /* when it was last learned (0 never) */
    float ho_x, ho_y;   /* handoff blend: offset from the new fix, decaying from ho_ms */
    int64_t ho_ms;
    long handoffs;      /* node set changes this track lived through (logs, tests) */
} seeker_fusion_mem;

/* Handoff between phones (an R2-only x-ray person R1 starts to see, or the other way round).
 * Contacts that show only part of a body (no eyes+ankles and no full torso at visibility 0.5,
 * and a box that does not look like a whole person) are partial: their size range is that of
 * the visible part, so triangulation uses them by bearing (seeker_ray.partial), they are left
 * out of the track's node list (R1 seeing a hand round a corner keeps the person XRAY, placed
 * on R1's bearing), and the wearer's own partial sightings never start a track: they only
 * continue a live one whose predicted position is on their bearing. Track identity gates on the
 * fix's and the track's own covariance (a mono range is +-20 to 30% along its ray), 1 sigma
 * wider when the node set changes, and a shared phone contact id is the same person within
 * track_gate_max_m. When the node set changes the filters restart at the new fix and the sent
 * position glides there from the old one (SEEKER_HANDOFF_TAU_MS) instead of kicking the
 * velocity. Each node's mono range is rescaled by what fused fixes taught (rk, from messages of
 * both phones under SEEKER_HANDOFF_FRESH_MS old, fading back to 1), so a person R1 stops seeing
 * stays near where the fused fix put them instead of jumping by the other phone's range error. */
#define SEEKER_HANDOFF_TAU_MS 90.0f   /* position blend time constant */
#define SEEKER_HANDOFF_MS 350         /* blend length */
#define SEEKER_HANDOFF_MIN_M 0.10f    /* smaller jumps are filtered as usual */
#define SEEKER_HANDOFF_RK_TAU_MS 4000.0f /* a learned range ratio fades back to 1 */
#define SEEKER_HANDOFF_FRESH_MS 200   /* ratios are learned only from messages this new */
#define SEEKER_HANDOFF_BEARING_DEG 20.0f /* bearing-only continuation: at most this far off */

/* Occlusion guard (fusion.occl_guard, on with the handoff; hud_fusion --no-occl-guard). A phone
 * that sees someone cut by a wall's edge still gets every joint from the pose model: the hidden
 * ones guessed, pushed on into the wall and fanned out, at visibility 0.3 to 0.8, and now and then
 * a confident guess of the whole body (head and ankles over 0.5) that reads as a whole person.
 *   - A whole contact within SEEKER_OCCL_BEARING_DEG of where its node saw part of someone in the
 *     last SEEKER_OCCL_HOLD_MS is held partial until SEEKER_OCCL_WHOLE_N whole messages in a row:
 *     one guessed frame does not make the wearer the primary (no SEEN flash, no crossfade, no
 *     skeleton from that view). Triangulation still takes it as whole (its range with its
 *     bearing), as before: by bearing alone its guessed hip would slide the fix along the other
 *     phone's ray.
 *   - With every view of the person partial, the skeleton comes from the one with the most joints
 *     at SEEKER_OCCL_FRAG_VIS inside its frame (not the wearer's just because it is the wearer's).
 *   - From a partial view, or one near where its node saw a part in the last SEEKER_OCCL_HOLD_MS,
 *     joints under SEEKER_OCCL_FRAG_VIS are not taken, nor are joints placed on or past the
 *     frame's edge from any view: they are held, then hidden (seeker_skel.h).
 *   - The wearer's lone partial sighting continues a track on its bearing at the distance of the
 *     track's last fix, not one moved on by its velocity: no walking on into the wall. */
#define SEEKER_OCCL_HOLD_MS 1000
#define SEEKER_OCCL_BEARING_DEG 8.0f
#define SEEKER_OCCL_WHOLE_N 3
#define SEEKER_OCCL_FRAG_VIS 0.5f

#define SEEKER_FUSION_MEM 32

typedef struct {
    const seeker_config *cfg;
    int n_nodes;
    seeker_fusion_node nodes[SEEKER_MAX_NODES];
    seeker_fusion_mem mem[SEEKER_FUSION_MEM];
    int next_id;
    int64_t tick;
    int debug; /* log filter snaps and one line per track per new datagram */
    int pose_gate; /* 1 (init default): jump gate on live message poses */
    int skel_smooth; /* 1 (init default): One Euro on skeleton joints; 0 passes them raw */
    int pose_blend;  /* 1 (init default): a pose back from a hold eases in; 0 jumps (old) */
    int pose_smooth; /* 1 (init default): live message poses are smoothed (One Euro) */
    int pose_flip;   /* 1 (init default): tag pose flips are caught (SEEKER_POSE_FLIP_*) */
    int range_hfov;  /* 1 (init default): box-height ranges rescaled with --hfov's real FOV */
    float person_h;  /* people's height in m for box-height range (0: phone.html's 1.7) */
    int ground_range; /* 1: a node with cam_h also ranges from the feet row (averaged in) */
    int handoff;     /* 1 (init default): partial contacts, covariance gate and the handoff blend
                      * (SEEKER_HANDOFF_*); 0 is the old identity code (hud_fusion --no-handoff) */
    int crouch_range; /* 1 (init default): eye-to-ankle ranges of a crouched contact use the
                       * crouch height (SEEKER_CROUCH_*); 0: as the phone sent them
                       * (hud_fusion --no-crouch-range) */
    int occl_guard;  /* 1 (init default), with handoff: a person cut by a wall or the frame edge
                      * (SEEKER_OCCL_*); 0 is the old code (hud_fusion --no-occl-guard) */
    int n_tags;    /* room poses of tags other than 0 (hud_fusion --tag-pose) */
    seeker_fusion_tag tags[SEEKER_MAX_TAGS];
    seeker_tri_opts tri; /* triangulation tuning; init copies assoc_m and the fusion.tri_* keys */
} seeker_fusion;

void seeker_fusion_init(seeker_fusion *f, const seeker_config *cfg);
/* Sets a node's pose (adding the node if it is not configured). pin = 1 keeps it over message
 * poses. src names the origin for logs ("cli", "file"). Returns the node index or -1 if full. */
int seeker_fusion_set_pose(seeker_fusion *f, const char *id, const seeker_pose *p, const char *src,
                           int pin);
/* Message poses from node id are in the frame of a tag at room pose tag (adding the node if it is
 * not configured). Unpins the node and starts it 2 m in front of that tag, facing it (pose_src
 * "init"; a later seeker_fusion_set_pose with pin 0 replaces the start). Returns the node index or
 * -1 if full. */
int seeker_fusion_set_tag_frame(seeker_fusion *f, const char *id, const seeker_pose *tag);
/* Room pose of ArUco tag tag_id, for message poses that name it in pose.tag_id. Returns 0, or -1
 * if tag_id is not > 0 (tag 0 is the room origin) or the table is full. */
int seeker_fusion_set_tag(seeker_fusion *f, int tag_id, const seeker_pose *room);

/* The phone's real horizontal FOV, from a number of degrees or a preset name (see
 * seeker_hfov_preset). Every bearing the node sends was computed on the phone with the FOV it
 * reports (hfov_deg, phone.html ?hfov=, default 66): it is rescaled to the real one,
 * tan(b') = tan(b) tan(real / 2) / tan(sent / 2), and the node's FOV (view cone, wearer
 * projection) becomes the real one. Returns the node index, or -1 for an unknown preset or a
 * full table. */
int seeker_fusion_set_hfov(seeker_fusion *f, const char *id, const char *spec);
/* Where node `id` sits as seen from node `from` (its pose now): metres to its right (negative:
 * left) and ahead (negative: behind). Returns 0, or -1 if either node is unknown. */
/* Multiplies node id's box-height ranges by k (0.2..5). Returns 0, or -1. */
int seeker_fusion_set_range_scale(seeker_fusion *f, const char *id, float k);
/* Camera height (m) and tilt (deg, up positive) of node id, for the ground-plane range: distance
 * from how far below the horizon the feet are. Returns 0, or -1. */
int seeker_fusion_set_cam(seeker_fusion *f, const char *id, float cam_h, float pitch_deg);
/* Ground-plane range of a person whose feet are at image row v_feet (px) in a w x h frame with
 * the given horizontal FOV, from a camera cam_h above the floor tilted pitch_deg up; 0 when the
 * feet are not below the horizon. */
float seeker_ground_range(float v_feet, int w, int h, float hfov_deg, float cam_h, float pitch_deg);
int seeker_fusion_relative(const seeker_fusion *f, const char *from, const char *id, float *right_m,
                           float *ahead_m);
/* Horizontal FOV estimates for a 4:3 640x480 Safari stream, from Apple's 35 mm-equivalent focal
 * lengths over the full sensor width (diagonal 43.27 mm, 4:3 frame 34.6 x 26.0 mm):
 * "iphone17pro" (Fusion Main 24 mm) 71.6 landscape / 56.8 portrait, "iphone16" (Fusion Main
 * 26 mm) 67.3 / 53.1, "iphone-ultrawide" (13 mm) 106.2 / 89.9. Estimates: a cropped video mode
 * narrows them; check with a person at a taped bearing (docs/LIVE-RUN.md). Returns 0, or -1. */
int seeker_hfov_preset(const char *name, float *landscape, float *portrait);
/* Feeds one decoded datagram received at now_ms. Returns the node index, or -1 if the node table
 * is full. Logs "NODE Rn resumed" when a LOST node comes back. */
int seeker_fusion_ingest(seeker_fusion *f, const seeker_remote_contacts *m, int64_t now_ms);
/* Applies the LOST rule and builds the outgoing tracks message. */
void seeker_fusion_tick(seeker_fusion *f, int64_t now_ms, seeker_tracks *out);

/* One filter step with measurement (zx, zy) taken dt_s after the previous one. dt is clamped to
 * [5 ms, cfg->smooth_max_dt_ms]. An invalid state, or a measurement farther than
 * cfg->smooth_gate_m from the prediction, snaps to the measurement with zero velocity: returns 1
 * then, else 0. */
int seeker_ab_update(seeker_ab *s, float zx, float zy, float dt_s, const seeker_config *cfg);
/* Same for a single-node track's range: gains cfg->range_smooth_alpha and range_smooth_beta, and
 * a snap gate of max(cfg->smooth_gate_m, cfg->range_smooth_gate_frac * predicted range). */
int seeker_range_update(seeker_rf *s, float z, float dt_s, const seeker_config *cfg);

/* Geometry helpers (exposed for tests). Angles in degrees, clockwise from +y. */
void seeker_node_to_room(float nx, float ny, float nyaw, float bearing, float range, float *x,
                         float *y);
float seeker_wrap180(float deg);

#endif
