#include "latency.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SAME_CLOCK_MS 600000 /* |arrive - ts| under 10 min: ts is Pi monotonic, not epoch ms */
#define CLOCK_JUMP_MS 10000  /* net above this: the phone's clock was set, start over */

void lat_init(lat_meter *m, float base_ms)
{
    memset(m, 0, sizeof *m);
    m->base_ms = base_ms;
    for (int i = 0; i < LAT_WINDOW_S; i++)
        m->bucket_sec[i] = -1;
}

static void clock_reset(lat_meter *m)
{
    for (int i = 0; i < LAT_WINDOW_S; i++)
        m->bucket_sec[i] = -1;
}

static int64_t window_min(const lat_meter *m, int64_t sec, int64_t d)
{
    int64_t best = d;
    for (int i = 0; i < LAT_WINDOW_S; i++)
        if (m->bucket_sec[i] > sec - LAT_WINDOW_S && m->bucket_min[i] < best)
            best = m->bucket_min[i];
    return best;
}

static void window_add(lat_meter *m, int64_t sec, int64_t d)
{
    int slot = (int)(sec % LAT_WINDOW_S);
    if (m->bucket_sec[slot] != sec) {
        m->bucket_sec[slot] = sec;
        m->bucket_min[slot] = d;
    } else if (d < m->bucket_min[slot]) {
        m->bucket_min[slot] = d;
    }
}

void lat_frame(lat_meter *m, uint32_t seq, const lat_sample *s)
{
    if (m->have_seq && seq > m->last_seq && seq - m->last_seq < 1000)
        m->skipped += (int)(seq - m->last_seq - 1);
    m->last_seq = seq;
    m->have_seq = 1;
    if (m->n >= LAT_MAX_SAMPLES)
        return;
    int64_t d = s->arrive_ms - (int64_t)s->ts_ms;
    int same = d > -SAME_CLOCK_MS && d < SAME_CLOCK_MS;
    if (!m->have_clock || same != m->same_clock) {
        clock_reset(m);
        m->same_clock = same;
        m->have_clock = 1;
    }
    int64_t sec = s->arrive_ms / 1000;
    float net;
    if (same) {
        net = (float)d;
    } else {
        int64_t off = window_min(m, sec, d);
        if (d - off > CLOCK_JUMP_MS) {
            clock_reset(m);
            off = d;
        }
        net = (float)(d - off);
    }
    window_add(m, sec, d);
    if (net < 0)
        net = 0;
    float pi = (float)(s->present_ms - s->arrive_ms);
    float age = s->age_ms >= 0 ? (float)s->age_ms : 0.0f;
    int i = m->n++;
    m->net[i] = net;
    m->pi[i] = pi;
    m->decode[i] = (float)(s->publish_ms - s->arrive_ms);
    m->wait[i] = (float)(s->present_ms - s->publish_ms);
    m->total[i] = age + net + pi + m->base_ms;
    if (s->age_ms >= 0)
        m->age[m->n_age++] = age;
    const float st[LAT_N_STAGES] = { s->dec_ms, s->scale_ms, s->copy_ms, s->overlay_ms, s->blit_ms,
                                     s->post_ms };
    for (int k = 0; k < LAT_N_STAGES; k++)
        m->stage[k][i] = st[k];
}

static int cmp_f(const void *a, const void *b)
{
    float x = *(const float *)a, y = *(const float *)b;
    return x < y ? -1 : x > y;
}

static void pct(const float *v, int n, float *p50, float *p95)
{
    float tmp[LAT_MAX_SAMPLES];
    if (n <= 0) {
        *p50 = -1;
        if (p95)
            *p95 = -1;
        return;
    }
    memcpy(tmp, v, (size_t)n * sizeof *tmp);
    qsort(tmp, (size_t)n, sizeof *tmp, cmp_f);
    *p50 = tmp[(n - 1) / 2];
    if (p95)
        *p95 = tmp[(int)(0.95f * (float)(n - 1) + 0.5f)];
}

int lat_tick(lat_meter *m, int64_t now_ms, lat_summary *out)
{
    if (!m->sec_start) {
        m->sec_start = now_ms;
        return 0;
    }
    if (now_ms - m->sec_start < 1000)
        return 0;
    lat_summary s;
    memset(&s, 0, sizeof s);
    s.n = m->n;
    s.fps = (float)m->n * 1000.0f / (float)(now_ms - m->sec_start);
    pct(m->total, m->n, &s.total_p50, &s.total_p95);
    pct(m->net, m->n, &s.net_p50, &s.net_p95);
    pct(m->pi, m->n, &s.pi_p50, &s.pi_p95);
    pct(m->decode, m->n, &s.decode_p50, NULL);
    pct(m->wait, m->n, &s.wait_p50, NULL);
    pct(m->age, m->n_age, &s.age_p50, NULL);
    for (int k = 0; k < LAT_N_STAGES; k++)
        pct(m->stage[k], m->n, &s.stage_p50[k], NULL);
    s.same_clock = m->same_clock;
    s.skipped = m->skipped;
    m->n = m->n_age = m->skipped = 0;
    m->sec_start = now_ms;
    m->last = s;
    m->have_last = 1;
    if (out)
        *out = s;
    return 1;
}

void lat_text(const lat_meter *m, char *out, size_t n)
{
    if (!m->have_last || m->last.n == 0)
        snprintf(out, n, "LAT --");
    else
        snprintf(out, n, "LAT %.0fMS P95 %.0fMS FPS %.0f", m->last.total_p50, m->last.total_p95,
                 m->last.fps);
}

void lat_log_line(const lat_summary *s, float base_ms, char *out, size_t n)
{
    if (s->n == 0) {
        snprintf(out, n, "latency: no wearer frame drawn in the last second");
        return;
    }
    char age[32];
    if (s->age_p50 >= 0)
        snprintf(age, sizeof age, "%.0f", s->age_p50);
    else
        snprintf(age, sizeof age, "n/a");
    snprintf(out, n,
             "latency: %.1f fps drawn, %d skipped | total p50 %.0f p95 %.0f ms = phone %s + net "
             "p50 %.0f p95 %.0f (%s) + pi p50 %.0f p95 %.0f (decode %.0f, wait+draw %.0f) + "
             "base %.0f",
             s->fps, s->skipped, s->total_p50, s->total_p95, age, s->net_p50, s->net_p95,
             s->same_clock ? "relay stamp, absolute" : "above the best frame in 30 s",
             s->pi_p50, s->pi_p95, s->decode_p50, s->wait_p50, base_ms);
    size_t len = strlen(out);
    if (len + 1 < n)
        snprintf(out + len, n - len,
                 " | stages p50 ms: decode %.1f scale %.1f copy %.1f overlay %.1f blit %.1f post "
                 "%.1f | received %.1f fps",
                 s->stage_p50[LAT_DEC], s->stage_p50[LAT_SCALE], s->stage_p50[LAT_COPY],
                 s->stage_p50[LAT_OVERLAY], s->stage_p50[LAT_BLIT], s->stage_p50[LAT_POST],
                 s->rx_fps);
}
