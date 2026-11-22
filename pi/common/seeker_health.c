#include "seeker_health.h"
#include "seeker_time.h"

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#define HEALTH_MAGIC 0x534b4831u /* "SKH1" */

seeker_health_table *seeker_health_open(int create_fresh)
{
    if (create_fresh)
        shm_unlink(SEEKER_HEALTH_NAME);
    int fd = shm_open(SEEKER_HEALTH_NAME, O_RDWR | O_CREAT, 0666);
    if (fd < 0)
        return NULL;
    /* Ignoring failure: some systems refuse to resize an existing object of the right size. */
    (void)!ftruncate(fd, sizeof(seeker_health_table));
    void *p = mmap(NULL, sizeof(seeker_health_table), PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED)
        return NULL;
    seeker_health_table *h = p;
    if (h->magic != HEALTH_MAGIC || h->version != 1) {
        memset(h, 0, sizeof *h);
        h->magic = HEALTH_MAGIC;
        h->version = 1;
    }
    return h;
}

int seeker_health_slot_for(seeker_health_table *h, const char *name)
{
    if (!h)
        return -1;
    int free_slot = -1;
    for (int i = 0; i < SEEKER_HEALTH_SLOTS; i++) {
        if (h->slot[i].name[0] == '\0') {
            if (free_slot < 0)
                free_slot = i;
        } else if (strncmp(h->slot[i].name, name, sizeof h->slot[i].name) == 0) {
            return i;
        }
    }
    if (free_slot >= 0) {
        /* Racy if two processes claim at once; the supervisor pre-claims slots to avoid that. */
        snprintf(h->slot[free_slot].name, sizeof h->slot[free_slot].name, "%s", name);
        h->slot[free_slot].state = SH_STARTING;
    }
    return free_slot;
}

void seeker_health_beat(seeker_health_table *h, int slot, int loop_ms, seeker_state st)
{
    if (!h || slot < 0)
        return;
    seeker_health_slot *s = &h->slot[slot];
    s->pid = (int32_t)getpid();
    s->loop_ms = loop_ms;
    s->state = st;
    s->last_beat_ms = seeker_now_ms();
}

const char *seeker_state_name(int st)
{
    switch (st) {
    case SH_STARTING: return "starting";
    case SH_OK: return "ok";
    case SH_DEGRADED: return "degraded";
    case SH_DEAD: return "dead";
    default: return "empty";
    }
}
