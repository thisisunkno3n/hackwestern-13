#ifndef SEEKER_JSON_H
#define SEEKER_JSON_H

/*
 * Tiny hand-rolled JSON reader (no packages needed on the Pi). It tokenizes into a caller-owned
 * array, no malloc. Strict enough for our datagrams and config; not a validating parser (it accepts
 * a few non-standard inputs, such as leading zeros in numbers).
 */

typedef enum { SJ_NULL, SJ_BOOL, SJ_NUM, SJ_STR, SJ_ARR, SJ_OBJ } sj_type;

typedef struct {
    sj_type type;
    int start, end; /* byte range in the text; strings exclude the quotes */
    int size;       /* arrays: item count; objects: key count */
    int next;       /* token index just past this token's subtree */
} sj_tok;

/* Returns the token count, or -1 on a syntax error or when `max` tokens are not enough. */
int sj_parse(const char *s, int len, sj_tok *t, int max);

/* Object lookup: returns the value token index for `key`, or -1. */
int sj_get(const char *s, const sj_tok *t, int obj, const char *key);
/* Array item i: returns the token index, or -1. */
int sj_index(const sj_tok *t, int arr, int i);

int sj_is_null(const sj_tok *t, int i); /* also true for i < 0 (missing) */
double sj_num(const char *s, const sj_tok *t, int i, double def);
int sj_bool(const char *s, const sj_tok *t, int i, int def);
/* Copies a string value with basic escapes decoded (\uXXXX becomes '?'). Returns 0 if ok. */
int sj_str(const char *s, const sj_tok *t, int i, char *out, int outsz);
int sj_str_eq(const char *s, const sj_tok *t, int i, const char *lit);

#endif
