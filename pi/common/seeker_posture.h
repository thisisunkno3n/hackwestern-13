#ifndef SEEKER_POSTURE_H
#define SEEKER_POSTURE_H

/*
 * Posture of one person (STAND, CROUCH, DOWN or UNKNOWN) from the smoothed ghost skeleton
 * (seeker_skel.h: COCO-17 joints in person-height units from the mid-hip, x right, y down,
 * visibility 0 for joints not to use). Only scale-free joint geometry is used, never the box,
 * the skeleton's size or where the feet are in the picture: for a single-phone person the range
 * comes from the landmarks' pixel height, and a person on a staircase has their feet above the
 * floor, so neither says how tall the person stands.
 *
 * Per body:
 *   torso_deg   angle of mid-shoulder to mid-hip from vertical (0 upright, 90 horizontal)
 *   flat        horizontal over vertical extent of shoulders, hips and the visible legs
 * Per leg, only for a full leg (hip, knee and ankle each at visibility leg_vis or more, inside
 * the phone's frame, and a shin at least 0.3 torso lengths long; MediaPipe invents hidden legs
 * with low visibility and often folds them onto the knee):
 *   knee_deg    hip-knee-ankle angle (180 straight)
 *   thigh_drop  how far the knee sits below the hip, over the torso length (about 0.85 for a
 *               vertical thigh; near 0 for a thigh that is horizontal or points at the camera,
 *               as when sitting, squatting or kneeling)
 *   thigh_ratio hip-knee length over the torso length (logged; short = thigh toward the camera)
 *   leg_h       how far the ankle sits below the hip, over the body height in the same units:
 *               the largest of thigh or shin / 0.245, torso / 0.29 and shoulder line / 0.25
 *               (whatever the view shortens least). About 0.47 standing, 0.36 in an ordinary
 *               crouch (hips 0.6 m up), 0.15 sitting on the heels. Only with both shoulders and
 *               both hips visible (else 9: never low).
 * A leg is LOW when leg_h is under crouch_leg and thigh_drop under SEEKER_POSTURE_LOW_THIGH. A
 * leg SUPPORTS the body when it is straight and vertical (knee over stand_knee and thigh_drop
 * over stand_thigh) and not low, and is FOLDED when its knee is under crouch_knee, thigh_drop is
 * under crouch_thigh, or it is low. Low is the test that works facing the phone from chest
 * height: the knees point at the camera, so the 2D knee angle stays near 160 deg, and the
 * forward lean shortens the torso, so thigh_drop stays over crouch_thigh until the hips are
 * almost on the floor.
 *
 * Rules, with dead bands that keep the current label (hysteresis):
 *   DOWN    torso over 65 deg from vertical (stays DOWN above 50), or legs visible and the body
 *           at least 1.6x wider than tall (stays above 1.3), and no supporting leg (someone
 *           bending over on straight legs is not down). Works on an upper body alone.
 *   STAND   at least one supporting leg and the torso within 35 deg (stays within 45). One
 *           leg is enough: on a staircase the other foot is a step or two up and its knee bent.
 *   CROUCH  both legs full and both folded (squatting, kneeling, sitting). One folded leg with
 *           the other one hidden is the upper leg on a stair as often as a crouch: the label is
 *           kept as it is (one_leg 1: a single deeply folded leg, knee under 100 deg and
 *           thigh_drop under 0.3, is enough). Side on, the far ankle is often behind the near
 *           leg while the far hip and knee show: then one folded leg plus a far thigh that is
 *           folded too (its knee under 0.7 of the full leg's longer segment below the hip; a
 *           standing leg's is 1) is CROUCH.
 *   UNKNOWN no torso, or no full leg (ankles hidden or out of frame: behind a wall, a desk or a
 *           stair the legs are often hidden; no guess).
 *
 * Debounce: one vote per new skeleton (at most every SEEKER_POSTURE_VOTE_MIN_MS, and every
 * SEEKER_POSTURE_VOTE_REPEAT_MS while the skeleton is held or missing). A new label needs
 * vote_n of the last vote_m votes (6 of 9, about 0.4 s at the phone's 15 Hz), UNKNOWN needs
 * SEEKER_POSTURE_UNKNOWN_N of the last SEEKER_POSTURE_UNKNOWN_M (about 1 s), and a label stays
 * at least SEEKER_POSTURE_HOLD_MS.
 *
 * Live tuning (hud_fusion reads these at start, see seeker_posture_tune_env):
 *   SEEKER_POSTURE_LEG_VIS      0.5   knee/ankle visibility for a full leg
 *   SEEKER_POSTURE_CROUCH_KNEE  120   folded: knee under this (deg) ...
 *   SEEKER_POSTURE_CROUCH_THIGH 0.45  ... or thigh_drop under this
 *   SEEKER_POSTURE_CROUCH_LEG   0.41  ... or leg_h under this with the knee near the hip (0: off)
 *   SEEKER_POSTURE_STAND_KNEE   150   supporting: knee over this (deg) ...
 *   SEEKER_POSTURE_STAND_THIGH  0.6   ... and thigh_drop over this
 *   SEEKER_POSTURE_ONE_LEG      0     1: one deeply folded leg is enough for CROUCH
 *   SEEKER_POSTURE_VOTES        6/9   votes for a new label, of the last frames
 */

#include <stdint.h>

#define SEEKER_POSTURE_VIS_MIN 0.3f      /* shoulders and hips */
#define SEEKER_POSTURE_LOW_THIGH 0.75f /* thigh_drop under this with a short leg: folded */
#define SEEKER_POSTURE_HOLD_MS 500
#define SEEKER_POSTURE_RESET_MS 1000
#define SEEKER_POSTURE_VOTES_MAX 16
#define SEEKER_POSTURE_UNKNOWN_N 12
#define SEEKER_POSTURE_UNKNOWN_M 15
#define SEEKER_POSTURE_VOTE_MIN_MS 40
#define SEEKER_POSTURE_VOTE_REPEAT_MS 100

enum { SEEKER_POSTURE_UNKNOWN, SEEKER_POSTURE_STAND, SEEKER_POSTURE_CROUCH, SEEKER_POSTURE_DOWN };

typedef struct {
    float leg_vis;
    float crouch_knee, crouch_thigh, crouch_leg;
    float stand_knee, stand_thigh;
    int one_leg;
    int vote_n, vote_m;
} seeker_posture_tune;

typedef struct {
    int ok; /* a full leg: hip, knee, ankle visible and in frame, real shin */
    float knee_deg, thigh_drop, thigh_ratio;
    float seg, leg_h; /* the longer of thigh and shin; hip-to-ankle height over it */
    int half;         /* not a full leg, but its hip and knee show and the other leg is full */
    float thigh_h;    /* then: the knee's drop below the hip over the full leg's seg */
} seeker_posture_leg;

typedef struct {
    int has_torso, has_knees, has_knee_angle, has_flat;
    float torso_deg, flat;
    float thigh_drop, knee_deg; /* of the straightest full leg (logs and tests) */
    int n_legs;                 /* full legs */
    seeker_posture_leg leg[2];  /* left, right */
} seeker_posture_feat;

typedef struct {
    int valid;
    int label;            /* SEEKER_POSTURE_* */
    int64_t label_ms;     /* when it last changed */
    int cand;             /* what the newest vote says */
    int64_t t_ms;         /* last update */
    int64_t vote_ms;      /* last vote */
    int votes[SEEKER_POSTURE_VOTES_MAX];
    int nvotes, head;     /* ring of the newest votes */
    float sig;            /* signature of the last voted skeleton (a held one repeats it) */
    seeker_posture_feat feat;
} seeker_posture;

void seeker_posture_tune_defaults(seeker_posture_tune *t);
/* Overrides from the SEEKER_POSTURE_* environment. Returns how many were applied; names of
 * values that did not parse go to bad (comma separated, may be NULL). */
int seeker_posture_tune_env(seeker_posture_tune *t, char *bad, int badsz);
/* The tuning every classify/update uses (defaults until set). */
void seeker_posture_set_tune(const seeker_posture_tune *t);
const seeker_posture_tune *seeker_posture_get_tune(void);
/* One line for the start log. */
void seeker_posture_tune_describe(const seeker_posture_tune *t, char *buf, int n);

/* Features of one skeleton (17 x {x, y, visibility}). Returns 1 if the torso is visible. */
int seeker_posture_features(const float (*pts)[3], seeker_posture_feat *f);
/* The label the features point to, given the current label (for the dead bands). */
int seeker_posture_classify(const seeker_posture_feat *f, int current);
/* Zero the visibility of joints of a person-local skeleton whose image landmark (img, the
 * phone's 0..1 normalized pts) lies on or outside the frame edge: MediaPipe still places them. */
void seeker_posture_mask_offframe(float (*pts)[3], const float (*img)[3]);
/* One update at now_ms with the person's skeleton, or NULL when there is none. Returns the label
 * to show. */
int seeker_posture_update(seeker_posture *s, const float (*pts)[3], int64_t now_ms);
/* "12 deg torso, legs: L knee 172 deg thigh 0.86, R knee 95 deg thigh 0.30" for the logs. */
void seeker_posture_describe(const seeker_posture_feat *f, char *buf, int n);
/* "unknown", "stand", "crouch", "down" (the tracks posture field) and back (-1 if not one). */
const char *seeker_posture_name(int label);
int seeker_posture_parse(const char *name);
/* HUD word: "STAND", "CROUCH", "DOWN", "" for unknown. */
const char *seeker_posture_word(int label);

#endif
