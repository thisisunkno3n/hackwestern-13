#include "seeker_pose.h"

#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define DEG2RAD 0.017453292519943295f

static int fail(char *err, int errsz, const char *why)
{
    if (err && errsz > 0)
        snprintf(err, (size_t)errsz, "%s", why);
    return -1;
}

static const char *skip_ws(const char *s)
{
    while (*s == ' ' || *s == '\t')
        s++;
    return s;
}

/* One finite number, then optional spaces, then `end` (',' or '\0'). */
static const char *parse_num(const char *s, char end, float *out)
{
    char *e;
    errno = 0;
    double v = strtod(skip_ws(s), &e);
    if (e == skip_ws(s) || errno == ERANGE || !isfinite(v))
        return NULL;
    e = (char *)skip_ws(e);
    if (*e != end)
        return NULL;
    *out = (float)v;
    return end ? e + 1 : e;
}

int seeker_pose_parse(const char *s, char id[SEEKER_NODE_LEN], seeker_pose *p, char *err,
                      int errsz)
{
    s = skip_ws(s);
    int n = 0;
    while (s[n] && (isalnum((unsigned char)s[n]) || s[n] == '_' || s[n] == '-'))
        n++;
    if (n == 0 || n >= SEEKER_NODE_LEN)
        return fail(err, errsz, "want ID=x_m,y_m,yaw_deg with a 1 to 7 character node id");
    const char *q = skip_ws(s + n);
    if (*q != '=')
        return fail(err, errsz, "want ID=x_m,y_m,yaw_deg (missing '=')");
    seeker_pose v;
    q = parse_num(q + 1, ',', &v.x_m);
    if (q)
        q = parse_num(q, ',', &v.y_m);
    if (q)
        q = parse_num(q, '\0', &v.yaw_deg);
    if (!q)
        return fail(err, errsz, "want three numbers x_m,y_m,yaw_deg after '='");
    memcpy(id, s, (size_t)n);
    id[n] = '\0';
    *p = v;
    return 0;
}

int seeker_pose_table_set(seeker_pose_table *t, const char *id, const seeker_pose *p,
                          const char *src)
{
    seeker_pose_entry *e = (seeker_pose_entry *)seeker_pose_table_get(t, id);
    if (!e) {
        if (t->n >= SEEKER_MAX_NODES)
            return -1;
        e = &t->e[t->n++];
        snprintf(e->id, sizeof e->id, "%s", id);
    }
    e->pose = *p;
    snprintf(e->src, sizeof e->src, "%s", src);
    return 0;
}

const seeker_pose_entry *seeker_pose_table_get(const seeker_pose_table *t, const char *id)
{
    for (int i = 0; i < t->n; i++)
        if (strcmp(t->e[i].id, id) == 0)
            return &t->e[i];
    return NULL;
}

int seeker_pose_load_file(seeker_pose_table *t, const char *path, char *err, int errsz)
{
    FILE *f = fopen(path, "r");
    if (!f) {
        if (err && errsz > 0)
            snprintf(err, (size_t)errsz, "%s: cannot open", path);
        return -1;
    }
    char line[256];
    int lineno = 0, count = 0;
    while (fgets(line, sizeof line, f)) {
        lineno++;
        char *hash = strchr(line, '#');
        if (hash)
            *hash = '\0';
        size_t len = strlen(line);
        while (len > 0 && isspace((unsigned char)line[len - 1]))
            line[--len] = '\0';
        if (*skip_ws(line) == '\0')
            continue;
        char id[SEEKER_NODE_LEN], why[96];
        seeker_pose p;
        if (seeker_pose_parse(line, id, &p, why, sizeof why) != 0 ||
            seeker_pose_table_set(t, id, &p, "file") != 0) {
            if (err && errsz > 0)
                snprintf(err, (size_t)errsz, "%s:%d: %s", path, lineno,
                         t->n >= SEEKER_MAX_NODES ? "too many nodes" : why);
            fclose(f);
            return -1;
        }
        count++;
    }
    fclose(f);
    return count;
}

float seeker_pose_world_bearing(const seeker_pose *p, float bearing_deg)
{
    float d = fmodf(p->yaw_deg + bearing_deg, 360.0f);
    if (d > 180.0f)
        d -= 360.0f;
    else if (d <= -180.0f)
        d += 360.0f;
    return d;
}

void seeker_pose_to_world(const seeker_pose *p, float bearing_deg, float range_m, float *x,
                          float *y)
{
    float a = (p->yaw_deg + bearing_deg) * DEG2RAD;
    *x = p->x_m + range_m * sinf(a);
    *y = p->y_m + range_m * cosf(a);
}

static float wrap180(float d)
{
    d = fmodf(d, 360.0f);
    if (d > 180.0f)
        d -= 360.0f;
    else if (d <= -180.0f)
        d += 360.0f;
    return d;
}

void seeker_pose_from_tag(const seeker_pose *tag, const seeker_pose *local, seeker_pose *room)
{
    float a = tag->yaw_deg * DEG2RAD, c = cosf(a), s = sinf(a);
    seeker_pose r = { tag->x_m + local->x_m * c + local->y_m * s,
                      tag->y_m - local->x_m * s + local->y_m * c,
                      wrap180(tag->yaw_deg + local->yaw_deg) };
    *room = r;
}

void seeker_pose_to_tag(const seeker_pose *tag, const seeker_pose *room, seeker_pose *local)
{
    float a = tag->yaw_deg * DEG2RAD, c = cosf(a), s = sinf(a);
    float dx = room->x_m - tag->x_m, dy = room->y_m - tag->y_m;
    seeker_pose l = { dx * c - dy * s, dx * s + dy * c, wrap180(room->yaw_deg - tag->yaw_deg) };
    *local = l;
}
