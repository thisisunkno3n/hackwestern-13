#ifndef SEEKER_MSGS_H
#define SEEKER_MSGS_H

/*
 * IF-5 message structs with JSON decode and encode. Field names follow INTERFACES.md exactly.
 * Fields marked PROPOSED are not in INTERFACES.md yet: they need Felix and Daniel to agree and an
 * INTERFACES.md PR before this leaves drafts/ (receivers ignore unknown fields, IF-5).
 */

#include <stdint.h>

#define SEEKER_MAX_DATAGRAM 8192 /* IF-5: max 8 KB per datagram */
#define SEEKER_PORT_TRACKS 47102
#define SEEKER_PORT_REMOTE 47103

#define SEEKER_ID_LEN 24
#define SEEKER_NODE_LEN 8
#define SEEKER_MAX_CONTACTS 16
#define SEEKER_MAX_TRACKS 16
#define SEEKER_MAX_NODES 8
/* PROPOSED (Daniel's PR #7, progress/DANIEL-skeleton-xray-spec.md): COCO-17 skeleton per contact,
 * wire "pts": [x0,y0,v0, ...] 51 ints, x,y image-normalized * 1000, visibility * 100. Joints:
 * 0 nose 1 L-eye 2 R-eye 3 L-ear 4 R-ear 5 L-sh 6 R-sh 7 L-el 8 R-el 9 L-wr 10 R-wr 11 L-hip
 * 12 R-hip 13 L-knee 14 R-knee 15 L-ankle 16 R-ankle. */
#define SEEKER_N_JOINTS 17
/* Tracks that may carry pts in one tracks datagram (single-node ones first); the encoder drops
 * pts, never a track, when the datagram would not fit. */
#define SEEKER_PTS_MAX_CARRIERS 6

/* IF-5.4 remote_contacts, one contact in the node's own frame. */
typedef struct {
    char id[SEEKER_ID_LEN];
    char cls[12];
    float conf;
    int has_box; /* 0 when box is missing, null or all zero (ws_ingest defaults it to [0,0,0,0]) */
    float box[4]; /* [x, y, w, h] in node pixels */
    float bearing_deg; /* clockwise from the node's boresight */
    int has_range; /* 0 when range_m is missing, null or not positive */
    float range_m;
    int has_pts;   /* PROPOSED: 1 when pts held all 17 joints */
    float pts[SEEKER_N_JOINTS][3]; /* x, y image-normalized (0..1, may run past the frame),
                                    * visibility 0..1 */
} seeker_contact;

typedef struct {
    char node_id[SEEKER_NODE_LEN];
    int v; /* 1 or 2 */
    int64_t t_mono_ms; /* t_mono_ms, else v2 ts_ms; informational only, never compared (IF-1) */
    int has_pose; /* 1 when x_m, y_m and yaw_deg are all present */
    int has_xy;   /* 1 when x_m and y_m are numbers (even if yaw_deg is null) */
    int has_yaw;  /* 1 when yaw_deg is a number */
    float x_m, y_m, yaw_deg;
    int has_tag_id; /* 1 when pose.tag_id is a whole number >= 0: the ArUco tag whose frame the
                     * pose is in (phone.html ?tag=1 with several tagid). Absent: room frame */
    int tag_id;
    int has_heading;   /* PROPOSED: top-level heading_deg, the phone's raw compass heading
                        * (deg clockwise from north, not zeroed), sent with or without a pose */
    float heading_deg;
    int has_pitch;     /* PROPOSED: pose.pitch_deg or top-level pitch_deg, the camera's tilt
                        * (up positive, -90..90), e.g. from the tag pose or DeviceOrientation */
    float pitch_deg;
    int width, height; /* v2 node frame size in px; 0 when missing or null */
    float hfov_deg;    /* v2; 0 when missing or null */
    int n;
    seeker_contact c[SEEKER_MAX_CONTACTS];
} seeker_remote_contacts;

/* Decodes one datagram (buf need not be NUL terminated). Returns 0 or -1 with a reason in err. */
int seeker_decode_remote_contacts(const char *buf, int len, seeker_remote_contacts *out,
                                  char *err, int errsz);

/* IF-5.2 tracks, one track. */
typedef struct {
    int id;
    char label[SEEKER_ID_LEN]; /* e.g. "R1-1", "R1+R2" */
    char cls[12];
    char src[8]; /* "remote" (no local camera after the pivot) */
    int visible;
    int has_range;
    float range_m;
    char range_src[8];
    float range_conf;
    int has_bearing; /* wearer bearing needs a range; 0 means null */
    float bearing_deg;
    float elev_deg;
    float closing_mps;
    char colour[8];
    int age_ms;
    /* PROPOSED extensions for remote-only contacts and the radar map: */
    char node_id[SEEKER_NODE_LEN]; /* primary node */
    char nodes[SEEKER_ID_LEN];     /* "R1" or "R1+R2" when both nodes see it */
    float node_bearing_deg;        /* bearing in the primary node's frame */
    int has_room;
    float room_x_m, room_y_m;      /* IF-1 room frame */
    int has_node_box;
    float node_box[4];             /* primary node's pixels */
    int posture;                   /* PROPOSED: SEEKER_POSTURE_* (seeker_posture.h), wire
                                    * "posture":"stand|crouch|down", absent = unknown */
    int has_pts;                   /* smoothed skeleton of the primary contact (seeker_skel.h) */
    float pts[SEEKER_N_JOINTS][3]; /* person-height units from the mid-hip (x right, y down),
                                    * visibility 0..1 (0: do not draw the joint) */
} seeker_track;

/* PROPOSED: per-node status so hud_display can draw NODE Rn LOST without its own timers. */
typedef struct {
    char node_id[SEEKER_NODE_LEN];
    char state[8]; /* "ok", "lost", "wait" (configured, never heard) */
    int age_ms;    /* since the last datagram by receive time; -1 if never */
    float x_m, y_m, yaw_deg, hfov_deg;
    int n_contacts;
    float rate_hz;
    char pose_src[12]; /* optional: "config", "init", "file", "cli", "message", "partial",
                        * "default"; only "message" (full x,y,yaw from the phone) is a live pose */
    int has_pitch;     /* optional "pitch_deg": the phone's camera tilt, fresh (< 2 s) */
    float pitch_deg;
} seeker_node_status;

typedef struct {
    char src[16];
    int64_t t_mono_ms;
    int64_t basis_frame_seq; /* fusion tick counter (no local frames after the pivot) */
    int has_heading;
    float heading_deg;
    char heading_src[12];
    float wearer_yaw_deg;
    char wearer_node[SEEKER_NODE_LEN]; /* PROPOSED: ranges and bearings are from this node's
                                        * pose (its entry in nodes); "" = the room origin */
    int n;
    seeker_track tracks[SEEKER_MAX_TRACKS];
    int n_nodes;                                /* PROPOSED */
    seeker_node_status nodes[SEEKER_MAX_NODES]; /* PROPOSED */
} seeker_tracks;

/* Returns the encoded length, or -1 if it would not fit in bufsz. */
int seeker_encode_tracks(const seeker_tracks *m, char *buf, int bufsz);
int seeker_decode_tracks(const char *buf, int len, seeker_tracks *out, char *err, int errsz);

#endif
