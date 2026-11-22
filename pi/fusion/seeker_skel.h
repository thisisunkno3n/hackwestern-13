#ifndef SEEKER_SKEL_H
#define SEEKER_SKEL_H

/*
 * Ghost skeleton for a person the wearer does not see (XRAY), from the COCO-17 landmarks a phone
 * sends as pts (progress/DANIEL-skeleton-xray-spec.md). Two steps, both in hud_fusion:
 *
 * 1. seeker_skel_local: image-normalized landmarks to person-local coordinates, in person-height
 *    units (a 1.7 m person is 1.0 tall) from the mid-hip, x right, y down. One uniform scale from
 *    the range (not the detection box), so a partial body keeps its proportions. The display
 *    scales them back by the projected person height, so range scaling comes for free.
 *
 * 2. seeker_skel_step: smoothing per track (fusion's track memory) and per joint. MediaPipe's raw
 *    landmarks jitter by a few pixels every frame; the phone smooths only bearing and range. Each
 *    joint coordinate runs through a One Euro filter (heavy smoothing when still, little lag when
 *    the joint moves fast). A joint below SEEKER_SKEL_VIS_MIN keeps its last position for
 *    SEEKER_SKEL_JOINT_HOLD_MS, then is hidden; a datagram without a skeleton keeps the last one
 *    for SEEKER_SKEL_HOLD_MS. A joint that jumps more than SEEKER_SKEL_SNAP (another person, a
 *    flipped detection) or comes back after its hold restarts at the new place instead of
 *    sliding there, so the ghost never trails across the screen. A new primary node (another
 *    phone's view of the person) restarts every joint.
 */

#include "seeker_msgs.h"

#include <stdint.h>

#define SEEKER_SKEL_VIS_MIN 0.3f        /* phone.html's own landmark visibility cut */
#define SEEKER_SKEL_JOINT_HOLD_MS 300   /* a joint that drops out stays this long */
#define SEEKER_SKEL_HOLD_MS 500         /* the whole skeleton, when datagrams stop carrying it */
#define SEEKER_SKEL_SNAP 0.5f           /* person heights: a bigger jump restarts the joint */
#define SEEKER_SKEL_MIN_CUTOFF_HZ 1.0f  /* One Euro: cutoff when the joint is still */
#define SEEKER_SKEL_BETA 1.5f           /* cutoff rise per person height per second of speed */
#define SEEKER_SKEL_D_CUTOFF_HZ 1.0f    /* cutoff for the speed estimate */
#define SEEKER_SKEL_PERSON_M 1.7f       /* phone.html HEIGHT_M */

typedef struct {
    int valid;
    char node[SEEKER_NODE_LEN]; /* primary node whose view the joints are in */
    long seq;                   /* its message count at the last update */
    int64_t t_ms;               /* receive time of that message */
    int64_t pts_ms;             /* the last message that carried a skeleton */
    float x[SEEKER_N_JOINTS], y[SEEKER_N_JOINTS];   /* filtered, person-height units */
    float dx[SEEKER_N_JOINTS], dy[SEEKER_N_JOINTS]; /* filtered speed, units per second */
    float vis[SEEKER_N_JOINTS];
    int64_t joint_ms[SEEKER_N_JOINTS]; /* last message the joint was visible in (0: never) */
} seeker_skel;

/* Contact landmarks to person-local joints (x, y person-height units, visibility 0..1). w, h:
 * the phone's frame in px; hfov_deg its horizontal FOV; range_m its distance to the person (the
 * smoothed track distance, so the scale does not jitter with the raw size range). Anchor: the
 * mid-hip, else the mid-shoulder 0.29 person heights up, else the box centre. Returns 1, or 0
 * when there is no skeleton, no frame size, no range or no anchor. */
int seeker_skel_local(const seeker_contact *c, int w, int h, float hfov_deg, float range_m,
                      float out[SEEKER_N_JOINTS][3]);

/* One fusion tick for one track. node, seq, rx_ms: the primary node, its message count and the
 * receive time of its newest message; raw: that message's person-local skeleton, or NULL when it
 * had none for this person. Only a new seq updates the filters; ticks in between repeat the last
 * output. smooth 0 passes joints through unfiltered (hud_fusion --no-skel-smooth), holds and
 * visibility rules still apply. Writes the skeleton to send into out (visibility 0: do not draw
 * that joint) and returns 1, or 0 when there is none to send. */
int seeker_skel_step(seeker_skel *s, const char *node, long seq, int64_t rx_ms,
                     const float (*raw)[3], int smooth, float out[SEEKER_N_JOINTS][3]);

#endif
