#ifndef SEEKER_PROJECTION_H
#define SEEKER_PROJECTION_H

/*
 * EagleEye pinhole projection: room-frame points (IF-1: +y forward, +x right, yaw clockwise from
 * +y, metres and degrees) into the wearer phone's image, then onto the HUD canvas. Pure math, no
 * drawing, so test_core can check it.
 *
 * Camera model: a phone taped to the helmet, boresight along the wearer pose yaw, tilted up by
 * pitch_deg (0: level; roll 0). f = (image_w / 2) / tan(hfov / 2) (IF-1 canonical), square pixels, principal point at
 * the image centre. u = cx + f * x_cam / z_cam is the same atan bearing phone.html reports.
 */

#include "seeker_pose.h"

typedef seeker_pose wv_pose; /* hud_fusion's pose type and frame (fusion/seeker_pose.h) */

typedef struct {
    float hfov_deg;
    int image_w, image_h; /* the helmet phone's frame in px (detections and video) */
    float cam_h_m;        /* camera height above the floor, for the person box rows */
    float pitch_deg;      /* camera tilt, up positive (0: level) */
    float person_h_m;     /* a person's height for the box top (0: WV_PERSON_H_M) */
} wv_camera;

#define WV_NEAR_M 0.2f       /* closer than this in front of the lens: not projected */
#define WV_PERSON_H_M 1.75f  /* box top (unless cam.person_h_m); the box bottom is the floor */
#define WV_PERSON_W_M 0.5f

typedef struct {
    float bearing_deg; /* wearer-relative, clockwise positive, (-180, 180] */
    float range_m;     /* horizontal distance from the wearer */
    float z_m;         /* depth along the boresight (negative: behind the wearer) */
    int in_front;      /* z_m >= WV_NEAR_M; u and the box are only valid when set */
    int in_fov;        /* in_front and 0 <= u <= image_w */
    float u;           /* image column of the person's centre */
    float box_x, box_y, box_w, box_h; /* person-sized box in image px (may exceed the image) */
} wv_proj;

float wv_focal_px(const wv_camera *cam);
float wv_wrap180(float deg);

/* Projects room point (px, py). Returns in_fov. */
int wv_project_point(const wv_pose *pose, const wv_camera *cam, float px, float py, wv_proj *out);

/* Image column for a wearer-relative bearing (only meaningful for |bearing| < 90). */
float wv_bearing_to_u(const wv_camera *cam, float bearing_deg);

/* Is a wearer-relative bearing inside the horizontal FOV? */
int wv_bearing_in_fov(const wv_camera *cam, float bearing_deg);

/* Edge arrow for a target outside the FOV: which screen edge, and whether it is behind. */
typedef struct {
    int side;       /* -1 left edge, +1 right edge (turn that way to face it) */
    int behind;     /* |bearing| > 90 */
    float turn_deg; /* |bearing| */
} wv_edge;

wv_edge wv_edge_for(float bearing_deg);

/* Image px to canvas px: the image is fitted into the canvas keeping its aspect, centred. */
typedef struct {
    float x0, y0, s; /* canvas = x0 + u * s, y0 + v * s */
    int x, y, w, h;  /* the fitted image rectangle on the canvas */
} wv_viewport;

wv_viewport wv_viewport_fit(int image_w, int image_h, int canvas_w, int canvas_h);

#endif
