#ifndef SEEKER_WEARER_VIEW_H
#define SEEKER_WEARER_VIEW_H

/*
 * EagleEye wearer view (hud_display --view wearer): people through walls from the wearer's point
 * of view. A phone taped to the helmet (node `W` by default) faces forward; fixed phones (R1, R2)
 * see the rest of the building. hud_fusion places everyone in the room frame; this view projects
 * each fused track into the helmet phone's image with a pinhole model (projection.h):
 *
 *   seen directly   the track's `nodes` include the wearer node: solid box in range colour.
 *   x-ray           seen only by other nodes (behind a wall, or just not detected by the helmet
 *                   phone): dashed cyan outline, "XRAY" tag, source node.
 *   off screen      outside the helmet phone's FOV: chevron on the left or right edge pointing
 *                   the shorter way round, with range and turn angle ("BEHIND" past 90 deg).
 *
 * Drawn over the helmet phone's IF-5.6 video when a fresh frame is there, else on black (visor /
 * Pepper's ghost: black is transparent on the combiner; mirror with --mirror h in the sink).
 * A heading-up radar inset sits in the bottom-right corner.
 */

#include "draw.h"
#include "projection.h"
#include "seeker_colour.h"
#include "seeker_config.h"
#include "seeker_posture.h"
#include "seeker_msgs.h"
#include "video_rx.h"

typedef enum { WV_POSE_AUTO = 0, WV_POSE_STATIC, WV_POSE_FUSION } wv_pose_src;

/* What the wearer pose on screen is: LIVE (fusion's message pose), HELD (the helmet phone sent
 * pose null after a live pose, fusion keeps the last one: "POSE HELD ... (NO TAG)"), START (the
 * unpinned starting pose, no live pose yet: "POSE START ... (NO TAG YET)"), FIXED (anything else). */
typedef enum { WV_MODE_FIXED = 0, WV_MODE_LIVE, WV_MODE_HELD, WV_MODE_START } wv_pose_mode;

#define WV_MAX_WALLS 8

typedef struct {
    char node[SEEKER_NODE_LEN]; /* wearer (helmet phone) node id */
    wv_pose pose;               /* static pose in the room frame */
    wv_camera cam;
    wv_pose_src pose_src; /* auto: fusion's pose for the wearer node once it is heard, else static */
    float assoc_deg;      /* other-node track within this bearing of a wearer-camera track ... */
    float assoc_range_m;  /* ... and this range (when both have one) is the same person */
    float yaw_alpha;      /* weight of each new live yaw sample, 1 = no smoothing */
    int visor;            /* draw on black even when video frames arrive */
    int inset;            /* radar inset on */
    int radar_fallback;   /* no fresh video (and not visor): full radar instead of black (IF-7) */
    int from_config;      /* the config file has a display.wearer object (turns EagleEye on) */
    int hfov_fixed;       /* --wearer-hfov given: keep cam.hfov_deg; else use the FOV the wearer
                           * phone reports (fusion node status), so the phone link's &hfov= is
                           * the one setting for bearings and the projection alike */
    int n_walls;
    float walls[WV_MAX_WALLS][4]; /* x1, y1, x2, y2 (m), drawn on the inset only */
    /* Height of the x-ray people in the picture: cam.cam_h_m (camera above the floor),
     * cam.pitch_deg (tilt up; the wearer phone's own pitch_deg replaces it when it sends one),
     * cam.person_h_m, xray_up_deg (a nudge: positive draws them higher), and pitch_auto: the tilt
     * corrected from people R1 itself sees whole (its own boxes against the projection). */
    float xray_up_deg;
    int pitch_auto;
    int handoff; /* 1 (default): SEEN hold (WV_SEEN_HOLD_MS), crossfade (WV_XFADE_MS) and a
                  * range-relative duplicate check; hud_display --handoff off */
} wv_config;

typedef struct {
    int idx;          /* index into seeker_tracks.tracks */
    int direct;       /* seen by the wearer camera */
    int dup_of;       /* -1, or the direct item this other-node track duplicates (not drawn) */
    int has_bearing;  /* placed in the wearer's frame */
    float bearing_deg;
    int has_range;
    float range_m;
    int has_proj;     /* p is valid (needs a range) */
    wv_proj p;
    int in_fov;
    int behind_wall;  /* 1 when line from wearer to track/ray crosses a configured wall */
    int has_ray_span; /* 1 when a range-less remote ray projects across the wearer's FOV */
    float ray_u0, ray_v0, ray_u1, ray_v1; /* image px of the ray segment inside FOV */
    int level;        /* seen person's SEEKER_RED / AMBER / GREEN (wv_render), -1 without range */
    int slot;         /* index into wv_view.colours for this track (wv_render), -1 none */
    int closing;      /* seen person approaching fast (the colour state's hysteresis), plan theme */
    int held;         /* drawn as seen though the wearer's phone missed them this frame */
    float fade;       /* 0..1: how far the crossfade from the other look is (1: done) */
} wv_item;

#define WV_COLOUR_SLOTS 32

typedef struct {
    int id;           /* fusion track id, 0 = free */
    int64_t seen_ms;
    seeker_colour c;  /* range from the wearer, with hysteresis (seeker_colour.h) */
    int box_valid;    /* eased screen box (WV_BOX_EASE), snapped on a jump */
    float bx, by, bw, bh;
    int shown;        /* drawn before: a change of look crossfades */
    int shown_direct; /* drawn as seen (with the WV_SEEN_HOLD_MS hold) */
    int64_t direct_ms, xfade_ms; /* last frame the wearer's phone saw them; start of the crossfade */
    int plate_valid;  /* plan theme, animations on: where this person's plate was drawn, so a plate */
    float plate_x, plate_y; /* that has to move (layout changed) glides there instead of jumping */
} wv_colour_slot;

/* Handoff between x-ray and seen (wv_config.handoff). The wearer's phone misses a person now and
 * then (a frame under its confidence cut, a hand at a wall edge): a person seen stays seen for
 * WV_SEEN_HOLD_MS, so the look does not flip at the phone's frame rate. A change of look
 * (x-ray ghost to seen box or back) crossfades over WV_XFADE_MS in the plan theme, both drawn in
 * the same eased box. */
#define WV_SEEN_HOLD_MS 400
#define WV_XFADE_MS 250

/* Drawn boxes move this fraction of the way to the new box each frame (30 fps: about 50 ms of
 * lag), so detection-box noise and 15 Hz steps do not shake the overlay; a jump larger than
 * WV_BOX_SNAP of the box height snaps. */
#define WV_BOX_EASE 0.5f
#define WV_BOX_SNAP 0.6f

typedef struct {
    wv_pose pose;    /* pose used for the last frame */
    int pose_live;   /* 1 if it came from fusion's wearer node */
    wv_pose_mode pose_mode;
    int rendered;    /* 0: the last frame was the radar fallback, the fields above are stale */
    char wearer_state[8]; /* "ok", "lost", "wait", or "" when fusion does not list the node */
    int n_direct, n_xray, n_off, n_dup, n_unplaced;
    int n_skel;      /* x-ray people drawn as ghost skeletons (track pts) */
    int n_posture[4]; /* people in view by SEEKER_POSTURE_* (unknown, stand, crouch, down) */
    int n_drawn;      /* boxes drawn in view this frame (tests and the smoothness sim) */
    int n_held;       /* drawn as seen through WV_SEEN_HOLD_MS */
    int n_fading;     /* crossfading between x-ray and seen */
    struct {
        int id, direct, x, y, w, h, level;
    } drawn[SEEKER_MAX_TRACKS];
    float video_ms;   /* copying the picture into the canvas (and dimming it), this frame */
    float pitch_deg;  /* the camera tilt the x-ray projection used this frame (all terms) */
    int pitch_src;    /* 0 configured (--wearer-pitch), 1 the phone's pitch_deg */
    int video;       /* 1 drawn over video, 0 black, -1 frame from another node (not drawn) */
} wv_stats;

typedef struct {
    wv_config cfg;
    int have_yaw;     /* smoothing state for a live yaw */
    float yaw;
    wv_stats last;
    int video_stale;  /* set by the caller per frame: 1 the video passed in is stale (dimmed to
                       * half), 2 it is a frozen picture through VIDEO LOST (a quarter) */
    wv_colour_slot colours[WV_COLOUR_SLOTS]; /* per seen person, keyed by track id */
    int64_t clock_ms; /* the frame's one timestamp (ms) for the handoff hold and crossfade and the
                       * person animations: hud_render sets it from hud_state.now_ms in the plan
                       * theme, tests and tools set it; 0 = seeker_now_ms() at render */
    int box_ease_off; /* 1: draw boxes where they are this frame (no WV_BOX_EASE) */
    const char *delay_text; /* plan theme: camera to HUD delay for the title block's Delay cell
                             * (hud_state.lat_text, SEEKER_LAT=1), NULL for no cell */
    float tilt_fix;   /* pitch_auto: learned tilt correction, deg */
    int tilt_n;       /* sightings it learned from */
} wv_view;

void wv_config_defaults(wv_config *w);

/* Wearer settings: defaults, then remote.nodes[node] (pose, hfov_deg, image_w, image_h) from
 * `cfg`, then the optional `display.wearer` object of the config file at `path`:
 *   {"node":"W","pose":[x,y,yaw],"pose_source":"auto","hfov_deg":66,"image_w":1280,
 *    "image_h":720,"cam_h_m":1.7,"assoc_deg":6,"visor":false,"inset":true,
 *    "walls":[[-4,2.5,4,2.5]]}
 * Node: `node` (CLI --wearer-node) if not NULL, else display.wearer.node, else display.view_node
 * (the video source), else "W". */
void wv_config_load(wv_config *w, const seeker_config *cfg, const char *path, const char *node);

/* "ID=x,y,yaw" (seeker_pose_parse, the hud_fusion --pose format) or a bare "x,y,yaw". Writes the
 * id to node when present. Returns 0, or -1 if malformed. */
int wv_parse_pose(const char *s, char *node, int nodesz, wv_pose *out);
int wv_parse_wall(const char *s, float out[4]);
int wv_parse_pose_src(const char *s, wv_pose_src *out);

/* Is `wearer` one of the '+'-separated ids in `nodes`, or the primary node? */
int wv_seen_by(const char *nodes, const char *primary, const char *wearer);

/* Pose for this frame, from fusion's node list or the static config (see pose_src). */
void wv_resolve_pose(const wv_config *w, const seeker_tracks *t, wv_pose *out, int *live,
                     char *state, int statesz);
/* Mode of that pose from fusion's pose_src for the wearer node; `live` is wv_resolve_pose's. */
wv_pose_mode wv_pose_mode_of(const wv_config *w, const seeker_tracks *t, int live);
const char *wv_pose_mode_name(wv_pose_mode m); /* "fixed", "live", "held", "start" */

/* Places every track in the wearer's frame and decides seen-directly / x-ray / duplicate.
 * Returns the item count (one per track, in track order). */
int wv_classify(const seeker_tracks *t, const wv_config *w, const wv_pose *pose, wv_item *items,
                int max);

/* Draws the full wearer view (background, symbology, inset). `video` is the newest fresh IF-5.6
 * frame (video_rx, already canvas-sized and letterboxed), or NULL; frames from a node other than
 * the wearer are not drawn (the projection would not match). `stale` greys everything. */
void wv_render(canvas *c, const seeker_config *cfg, wv_view *v, const seeker_tracks *t, int stale,
               const video_frame *video);

#endif
