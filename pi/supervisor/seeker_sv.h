#ifndef SEEKER_SV_H
#define SEEKER_SV_H

/*
 * Pure helpers for seeker_supervisor (no processes, no clocks), so tests/test_supervisor.c can
 * check them on any host.
 */

#include <stddef.h>
#include <stdint.h>

/*
 * Restart backoff: the first restart waits min_ms, each further failure doubles the wait up to
 * max_ms. A child that ran for at least stable_ms before exiting counts as healthy, so its next
 * restart waits min_ms again.
 */
typedef struct {
    int min_ms, max_ms, stable_ms;
    int failures; /* consecutive short runs, including the current one */
} sv_backoff;

void sv_backoff_init(sv_backoff *b, int min_ms, int max_ms, int stable_ms);
/* Call once per exit (or failed spawn, with uptime_ms 0). Returns the delay before the restart. */
int sv_backoff_next(sv_backoff *b, int64_t uptime_ms);

/*
 * Priority spec in the QNX `on -p` / `slay -P` form: "<prio>[f|r|o]", e.g. "50f", "30r", "10".
 * f = SCHED_FIFO, r = SCHED_RR, o = SCHED_OTHER, no letter = SCHED_RR. "0" or "" means
 * "inherit" and returns prio 0. Returns 0 on success, -1 on a malformed spec.
 */
int sv_parse_prio(const char *spec, int *prio, int *policy);
/* "50f", "30r", "10o"; "inherit" for prio 0. */
const char *sv_prio_str(int prio, int policy, char *buf, size_t n);

/* waitpid() status as text: "exited with status 3", "killed by SIGSEGV (11)". */
const char *sv_describe_status(int status, char *buf, size_t n);
/* "SIGTERM" etc. for the usual signals, else "signal N". */
const char *sv_signal_name(int sig, char *buf, size_t n);

/* TCP/UDP port "1".."65535" (digits only). Returns the port or -1. */
int sv_parse_port(const char *s);
/* Node id such as "R1": 1..max_len characters of [A-Za-z0-9_-]. Returns 1 if valid. */
int sv_valid_node_id(const char *s, int max_len);

#endif
