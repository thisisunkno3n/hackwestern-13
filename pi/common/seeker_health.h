#ifndef SEEKER_HEALTH_H
#define SEEKER_HEALTH_H

/*
 * IF-6 health table in POSIX shared memory `/seeker_health`. One slot per process name.
 * The supervisor creates and owns the table; children attach and beat. Without a supervisor a
 * child creates the table itself so it still runs standalone (stub-friendly, IF-1).
 */

#include <stdint.h>
#include <sys/types.h>

#define SEEKER_HEALTH_NAME "/seeker_health"
#define SEEKER_HEALTH_SLOTS 8

typedef enum { SH_EMPTY = 0, SH_STARTING, SH_OK, SH_DEGRADED, SH_DEAD } seeker_state;

typedef struct {
    char name[32];
    volatile int32_t pid;
    volatile int64_t last_beat_ms;
    volatile int32_t loop_ms;
    volatile int32_t state;
    volatile int32_t restarts;
} seeker_health_slot;

typedef struct {
    uint32_t magic, version;
    seeker_health_slot slot[SEEKER_HEALTH_SLOTS];
} seeker_health_table;

seeker_health_table *seeker_health_open(int create_fresh);
/* Finds or claims the slot for `name`. Returns the index or -1 if the table is full. */
int seeker_health_slot_for(seeker_health_table *h, const char *name);
void seeker_health_beat(seeker_health_table *h, int slot, int loop_ms, seeker_state st);
const char *seeker_state_name(int st);

#endif
