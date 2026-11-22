#ifndef SEEKER_LOG_H
#define SEEKER_LOG_H

/*
 * IF-1 logging: human logs to stderr prefixed "[process]".
 * Deliberately not slog2: that would be QNX-only code outside the three allowed backends. The
 * supervisor redirects each child's stderr to logs/<name>.log, so `sloginfo` will NOT show these.
 */

void seeker_log_init(const char *process_name);
const char *seeker_log_name(void);
void seeker_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* Appends one JSON line to logs/events.jsonl (IF-9). `json_fields` is the inside of an object,
 * e.g. "\"event\":\"node_lost\",\"node_id\":\"R1\"". Best effort; failures are ignored. */
void seeker_event(const char *json_fields);

#endif
