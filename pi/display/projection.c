#include "projection.h"

#include <math.h>

#define DEG2RAD 0.017453292519943295f
#define RAD2DEG 57.29577951308232f

float wv_wrap180(float d)
{
    if (!isfinite(d))
        return 0.0f;
    d = fmodf(d, 360.0f);
    if (d > 180.0f)
        d -= 360.0f;
    else if (d <= -180.0f)
        d += 360.0f;
    return d;
}

static float hfov_of(const wv_camera *cam)
{
    float h = cam->hfov_deg;
    return h > 5.0f && h < 175.0f ? h : 66.0f;
}

static int width_of(const wv_camera *cam)
{
    return cam->image_w > 0 ? cam->image_w : 1280;
}

static int height_of(const wv_camera *cam)
{
    return cam->image_h > 0 ? cam->image_h : width_of(cam) * 9 / 16;
}

float wv_focal_px(const wv_camera *cam)
{
    return ((float)width_of(cam) / 2.0f) / tanf(hfov_of(cam) * 0.5f * DEG2RAD);
}

float wv_bearing_to_u(const wv_camera *cam, float bearing_deg)
{
    return (float)width_of(cam) / 2.0f + wv_focal_px(cam) * tanf(bearing_deg * DEG2RAD);
}

int wv_bearing_in_fov(const wv_camera *cam, float bearing_deg)
{
    return fabsf(wv_wrap180(bearing_deg)) <= hfov_of(cam) * 0.5f;
}

int wv_project_point(const wv_pose *pose, const wv_camera *cam, float px, float py, wv_proj *out)
{
    float a = pose->yaw_deg * DEG2RAD;
    float dx = px - pose->x_m, dy = py - pose->y_m;
    /* Boresight (sin a, cos a); the camera's right is (cos a, -sin a). */
    float z = dx * sinf(a) + dy * cosf(a);
    float x = dx * cosf(a) - dy * sinf(a);
    float W = (float)width_of(cam), H = (float)height_of(cam), f = wv_focal_px(cam);

    out->z_m = z;
    out->range_m = hypotf(dx, dy);
    out->bearing_deg = out->range_m > 1e-4f ? wv_wrap180(atan2f(x, z) * RAD2DEG) : 0.0f;
    out->in_front = z >= WV_NEAR_M;
    out->in_fov = 0;
    out->u = out->box_x = out->box_y = out->box_w = out->box_h = 0;
    if (!out->in_front)
        return 0;
    out->u = W / 2.0f + f * x / z;
    out->in_fov = out->u >= 0.0f && out->u <= W;
    float h_cam = cam->cam_h_m > 0.1f ? cam->cam_h_m : 1.7f;
    float h_person = cam->person_h_m > 0.5f ? cam->person_h_m : WV_PERSON_H_M;
    /* Image row of a point at height h above the floor: its elevation from the lens, less the
     * camera's tilt (tilted up, everything sits lower in the picture). */
    float pitch = cam->pitch_deg * DEG2RAD;
    float v_top = H / 2.0f - f * tanf(atanf((h_person - h_cam) / z) - pitch);
    float v_bot = H / 2.0f - f * tanf(atanf(-h_cam / z) - pitch);
    out->box_w = f * WV_PERSON_W_M / z;
    out->box_x = out->u - out->box_w / 2.0f;
    out->box_y = v_top;
    out->box_h = v_bot - v_top;
    return out->in_fov;
}

wv_edge wv_edge_for(float bearing_deg)
{
    float b = wv_wrap180(bearing_deg);
    wv_edge e = { b >= 0.0f ? 1 : -1, fabsf(b) > 90.0f, fabsf(b) };
    return e;
}

wv_viewport wv_viewport_fit(int image_w, int image_h, int canvas_w, int canvas_h)
{
    wv_viewport v;
    if (image_w <= 0 || image_h <= 0) {
        image_w = canvas_w;
        image_h = canvas_h;
    }
    float sx = (float)canvas_w / (float)image_w, sy = (float)canvas_h / (float)image_h;
    v.s = sx < sy ? sx : sy;
    v.w = (int)lroundf((float)image_w * v.s);
    v.h = (int)lroundf((float)image_h * v.s);
    v.x = (canvas_w - v.w) / 2;
    v.y = (canvas_h - v.h) / 2;
    v.x0 = (float)v.x;
    v.y0 = (float)v.y;
    return v;
}
