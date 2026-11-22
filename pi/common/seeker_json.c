#include "seeker_json.h"

#include <stdlib.h>
#include <string.h>

#define SJ_MAX_DEPTH 16

typedef struct {
    const char *s;
    int len, pos;
    sj_tok *t;
    int max, n;
} sj_p;

static void skip_ws(sj_p *p)
{
    while (p->pos < p->len) {
        char c = p->s[p->pos];
        if (c != ' ' && c != '\t' && c != '\n' && c != '\r')
            break;
        p->pos++;
    }
}

static int new_tok(sj_p *p, sj_type type, int start)
{
    if (p->n >= p->max)
        return -1;
    sj_tok *k = &p->t[p->n];
    k->type = type;
    k->start = start;
    k->end = start;
    k->size = 0;
    k->next = p->n + 1;
    return p->n++;
}

static int parse_value(sj_p *p, int depth);

static int parse_string(sj_p *p)
{
    int i = new_tok(p, SJ_STR, ++p->pos); /* skip opening quote */
    if (i < 0)
        return -1;
    while (p->pos < p->len) {
        char c = p->s[p->pos];
        if (c == '\\') {
            p->pos += 2;
            continue;
        }
        if (c == '"') {
            p->t[i].end = p->pos++;
            return i;
        }
        if ((unsigned char)c < 0x20)
            return -1;
        p->pos++;
    }
    return -1;
}

static int parse_literal(sj_p *p, const char *lit, sj_type type)
{
    int n = (int)strlen(lit);
    if (p->len - p->pos < n || memcmp(p->s + p->pos, lit, (size_t)n) != 0)
        return -1;
    int i = new_tok(p, type, p->pos);
    if (i < 0)
        return -1;
    p->pos += n;
    p->t[i].end = p->pos;
    return i;
}

static int parse_number(sj_p *p)
{
    int i = new_tok(p, SJ_NUM, p->pos);
    if (i < 0)
        return -1;
    int digits = 0;
    while (p->pos < p->len) {
        char c = p->s[p->pos];
        if ((c >= '0' && c <= '9'))
            digits++;
        else if (!(c == '-' || c == '+' || c == '.' || c == 'e' || c == 'E'))
            break;
        p->pos++;
    }
    p->t[i].end = p->pos;
    return digits ? i : -1;
}

static int parse_container(sj_p *p, int depth, int is_obj)
{
    if (depth > SJ_MAX_DEPTH)
        return -1;
    int i = new_tok(p, is_obj ? SJ_OBJ : SJ_ARR, p->pos++);
    if (i < 0)
        return -1;
    char close = is_obj ? '}' : ']';
    skip_ws(p);
    if (p->pos < p->len && p->s[p->pos] == close) {
        p->pos++;
    } else {
        for (;;) {
            skip_ws(p);
            if (is_obj) {
                if (p->pos >= p->len || p->s[p->pos] != '"' || parse_string(p) < 0)
                    return -1;
                skip_ws(p);
                if (p->pos >= p->len || p->s[p->pos] != ':')
                    return -1;
                p->pos++;
            }
            if (parse_value(p, depth + 1) < 0)
                return -1;
            p->t[i].size++;
            skip_ws(p);
            if (p->pos >= p->len)
                return -1;
            char c = p->s[p->pos++];
            if (c == close)
                break;
            if (c != ',')
                return -1;
        }
    }
    p->t[i].end = p->pos;
    p->t[i].next = p->n;
    return i;
}

static int parse_value(sj_p *p, int depth)
{
    skip_ws(p);
    if (p->pos >= p->len)
        return -1;
    char c = p->s[p->pos];
    switch (c) {
    case '{': return parse_container(p, depth, 1);
    case '[': return parse_container(p, depth, 0);
    case '"': return parse_string(p);
    case 't': return parse_literal(p, "true", SJ_BOOL);
    case 'f': return parse_literal(p, "false", SJ_BOOL);
    case 'n': return parse_literal(p, "null", SJ_NULL);
    default:
        if (c == '-' || (c >= '0' && c <= '9'))
            return parse_number(p);
        return -1;
    }
}

int sj_parse(const char *s, int len, sj_tok *t, int max)
{
    sj_p p = { s, len, 0, t, max, 0 };
    if (parse_value(&p, 0) < 0)
        return -1;
    skip_ws(&p);
    return p.pos == p.len ? p.n : -1;
}

int sj_get(const char *s, const sj_tok *t, int obj, const char *key)
{
    if (obj < 0 || t[obj].type != SJ_OBJ)
        return -1;
    size_t klen = strlen(key);
    int k = obj + 1;
    for (int n = 0; n < t[obj].size; n++) {
        int v = t[k].next;
        if ((size_t)(t[k].end - t[k].start) == klen && memcmp(s + t[k].start, key, klen) == 0)
            return v;
        k = t[v].next;
    }
    return -1;
}

int sj_index(const sj_tok *t, int arr, int i)
{
    if (arr < 0 || t[arr].type != SJ_ARR || i < 0 || i >= t[arr].size)
        return -1;
    int k = arr + 1;
    while (i-- > 0)
        k = t[k].next;
    return k;
}

int sj_is_null(const sj_tok *t, int i)
{
    return i < 0 || t[i].type == SJ_NULL;
}

double sj_num(const char *s, const sj_tok *t, int i, double def)
{
    if (i < 0 || t[i].type != SJ_NUM)
        return def;
    char buf[48];
    int n = t[i].end - t[i].start;
    if (n <= 0 || n >= (int)sizeof buf)
        return def;
    memcpy(buf, s + t[i].start, (size_t)n);
    buf[n] = '\0';
    char *endp;
    double v = strtod(buf, &endp);
    return endp == buf ? def : v;
}

int sj_bool(const char *s, const sj_tok *t, int i, int def)
{
    if (i < 0 || t[i].type != SJ_BOOL)
        return def;
    return s[t[i].start] == 't';
}

int sj_str(const char *s, const sj_tok *t, int i, char *out, int outsz)
{
    if (outsz <= 0)
        return -1;
    out[0] = '\0';
    if (i < 0 || t[i].type != SJ_STR)
        return -1;
    int o = 0;
    for (int k = t[i].start; k < t[i].end && o < outsz - 1; k++) {
        char c = s[k];
        if (c == '\\' && k + 1 < t[i].end) {
            char e = s[++k];
            switch (e) {
            case 'n': c = '\n'; break;
            case 't': c = '\t'; break;
            case 'r': c = '\r'; break;
            case 'b': c = '\b'; break;
            case 'f': c = '\f'; break;
            case 'u': c = '?'; k += 4; break;
            default: c = e; break;
            }
        }
        out[o++] = c;
    }
    out[o] = '\0';
    return 0;
}

int sj_str_eq(const char *s, const sj_tok *t, int i, const char *lit)
{
    if (i < 0 || t[i].type != SJ_STR)
        return 0;
    size_t n = strlen(lit);
    return (size_t)(t[i].end - t[i].start) == n && memcmp(s + t[i].start, lit, n) == 0;
}
