#ifndef SEEKER_POSE_H
#define SEEKER_POSE_H

/*
 * Per-node (phone) poses in the IF-1 room frame: +y forward, +x right, yaw_deg clockwise from +y.
 * One text format everywhere, the same as phone.html's ?pose=x_m,y_m,yaw_deg with the node id in
 * front:
 *
 *   R1=-2.0,3.0,145
 *
 * hud_fusion takes it from --pose (repeatable) and from --poses FILE (one entry per line, blank
 * lines and '#' comments ignored).
 */

#include "seeker_msgs.h"

typedef struct {
    float x_m, y_m, yaw_deg;
} seeker_pose;

typedef struct {
    char id[SEEKER_NODE_LEN];
    seeker_pose pose;
    char src[8]; /* "cli" or "file" */
} seeker_pose_entry;

typedef struct {
    int n;
    seeker_pose_entry e[SEEKER_MAX_NODES];
} seeker_pose_table;

/* Parses "R1=x_m,y_m,yaw_deg" (spaces allowed around each part). Returns 0, or -1 with a reason in
 * err. The id must be 1 to SEEKER_NODE_LEN-1 characters of [A-Za-z0-9_-]. */
int seeker_pose_parse(const char *s, char id[SEEKER_NODE_LEN], seeker_pose *p, char *err,
                      int errsz);
/* Adds or replaces the entry for id. Returns 0, or -1 if the table is full. */
int seeker_pose_table_set(seeker_pose_table *t, const char *id, const seeker_pose *p,
                          const char *src);
const seeker_pose_entry *seeker_pose_table_get(const seeker_pose_table *t, const char *id);
/* Reads a pose file into t (src "file"). Returns the number of entries, or -1 with the file name,
 * line number and reason in err. */
int seeker_pose_load_file(seeker_pose_table *t, const char *path, char *err, int errsz);

/* Bearing in the node's frame (clockwise from boresight) to a room-frame bearing (clockwise from
 * +y), wrapped to (-180, 180]. */
float seeker_pose_world_bearing(const seeker_pose *p, float bearing_deg);
/* Contact at bearing_deg and range_m from the node to room x, y. */
void seeker_pose_to_world(const seeker_pose *p, float bearing_deg, float range_m, float *x,
                          float *y);

/* Tag frames. phone.html ?tag=1&tagid=N reports the camera pose in tag N's own frame: origin at the
 * tag centre, +y the way you face looking straight at the tag (into its wall), +x your right, yaw
 * clockwise from +y. `tag` is that tag's pose in the room frame (its centre, and the room yaw of
 * its +y axis), so tag 0 at the room origin is 0,0,0. x,y and yaw transform independently, so a
 * partial pose (x,y only or yaw only) can go through too. Yaw out is wrapped to (-180, 180]. */
void seeker_pose_from_tag(const seeker_pose *tag, const seeker_pose *local, seeker_pose *room);
/* The inverse: a room pose in tag's frame. */
void seeker_pose_to_tag(const seeker_pose *tag, const seeker_pose *room, seeker_pose *local);

#endif
