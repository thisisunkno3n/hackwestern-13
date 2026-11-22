#include "video_decode.h"

#include <string.h>

/* stb_image is third-party code: keep -Wall -Wextra for our files without editing the header. */
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wall"
#pragma clang diagnostic ignored "-Wextra"
#elif defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wall"
#pragma GCC diagnostic ignored "-Wextra"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wunused-function"
#endif
#define STB_IMAGE_IMPLEMENTATION
#define STBI_ONLY_JPEG
#define STBI_NO_STDIO
#define STBI_NO_LINEAR
#define STBI_NO_HDR
#define STBI_NO_THREAD_LOCALS /* only the receiver thread decodes */
#define STBI_MAX_DIMENSIONS VIDEO_MAX_DIM
#include "../third_party/stb/stb_image.h"
#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

uint8_t *video_jpeg_decode(const uint8_t *jpg, size_t len, int *w, int *h)
{
    if (!jpg || len < 4 || len > 0x7fffffff || jpg[0] != 0xFF || jpg[1] != 0xD8)
        return NULL;
    int n = 0;
    return stbi_load_from_memory(jpg, (int)len, w, h, &n, 3);
}

void video_jpeg_free(uint8_t *rgb)
{
    stbi_image_free(rgb);
}

void video_fit_rect(int sw, int sh, int dw, int dh, int rect[4])
{
    if (sw <= 0 || sh <= 0) {
        rect[0] = rect[1] = 0;
        rect[2] = dw;
        rect[3] = dh;
        return;
    }
    int w = dw, h = (int)((long long)dw * sh / sw);
    if (h > dh) {
        h = dh;
        w = (int)((long long)dh * sw / sh);
    }
    rect[0] = (dw - w) / 2;
    rect[1] = (dh - h) / 2;
    rect[2] = w;
    rect[3] = h;
}

void video_blit_rgb(const uint8_t *rgb, int sw, int sh, uint32_t *dst, int dw, int dh, int stride,
                    const int rect[4])
{
    int rx = rect[0], ry = rect[1], rw = rect[2], rh = rect[3];
    if (rw <= 0 || rh <= 0 || sw <= 0 || sh <= 0) {
        for (int y = 0; y < dh; y++)
            memset(dst + (size_t)y * stride, 0, (size_t)dw * 4);
        return;
    }
    uint32_t xstep = (uint32_t)(((uint64_t)sw << 16) / (uint64_t)rw);
    int prev_sy = -1;
    for (int y = 0; y < dh; y++) {
        uint32_t *d = dst + (size_t)y * stride;
        if (y < ry || y >= ry + rh) {
            memset(d, 0, (size_t)dw * 4);
            continue;
        }
        int sy = (int)((long long)(y - ry) * sh / rh);
        if (sy == prev_sy) { /* upscaling repeats source rows: copy the row just made */
            memcpy(d, d - stride, (size_t)dw * 4);
            continue;
        }
        prev_sy = sy;
        const uint8_t *s = rgb + (size_t)sy * sw * 3;
        if (rx > 0)
            memset(d, 0, (size_t)rx * 4);
        uint32_t acc = xstep / 2;
        for (int x = 0; x < rw; x++, acc += xstep) {
            const uint8_t *p = s + (acc >> 16) * 3;
            d[rx + x] = ((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2];
        }
        int right = dw - rx - rw;
        if (right > 0)
            memset(d + rx + rw, 0, (size_t)right * 4);
    }
}
