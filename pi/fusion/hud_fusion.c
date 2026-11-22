/*
 * hud_fusion: remote_contacts (UDP 47103, IF-5.4) in, tracks (UDP 47102 to hud_display, IF-5.2)
 * out at fusion.rate_hz (30). Beats in /seeker_health (IF-6).
 *
 *   hud_fusion [--config config/seeker.json] [--debug] [--run-for <s>] [--no-health]
 *              [--no-smooth] [--smooth-alpha a] [--smooth-beta b] [--smooth-gate m]
 *              [--no-range-smooth] [--no-pose-gate] [--no-pose-smooth] [--no-pose-flip] [--no-handoff]
 *              [--no-crouch-range] [--no-occl-guard]
 *              [--pose ID=x_m,y_m,yaw_deg]... [--init-pose ID=x_m,y_m,yaw_deg]... [--poses FILE] [--pose-source message|config]
 *              [--tag-pose TAG=x_m,y_m,yaw_deg]... [--tag-frame ID=tag_x_m,tag_y_m,tag_yaw_deg]...
 *
 * --debug prints one line per received datagram to stderr (lanes task 5), one line per track per
 * new datagram (method, raw fix against the sent position, crossing angle and fix sigma, or the
 * raw and smoothed size range for a single-phone track), one line when a track's smoothing filter
 * snaps to a jump past the gate, and each fused (multi-phone) track at most once a second.
 * Smoothing flags override fusion.smooth, smooth_alpha, smooth_beta and smooth_gate_m.
 * --no-range-smooth turns off only the single-phone range filter (fusion.range_smooth); those
 * tracks then go through the room filter like fused ones.
 * --pose pins one node's room-frame pose (IF-1: +y forward, +x right, yaw clockwise from +y), the
 * same x_m,y_m,yaw_deg format as phone.html's ?pose=. Repeat it per node. A pinned pose wins over
 * the pose in remote_contacts messages and over remote.nodes. --pose beats --poses for one node.
 * --init-pose sets a node's starting pose (overriding remote.nodes) without pinning it, so
 * live message poses still replace it. --pose and --poses beat --init-pose.
 * --poses FILE reads the same ID=x_m,y_m,yaw_deg entries, one per line ('#' starts a comment).
 * --pose-source overrides remote.pose_source for unpinned nodes: "message" (a message pose replaces
 * the remote.nodes pose) or "config" (message poses are ignored).
 * --person-h, --range-scale ID=k, --no-range-hfov: box-height range corrections (people's
 * height vs phone.html's 1.7 m, a per-phone factor, and the --hfov FOV rescale of the range,
 * on by default; off for a phone whose range was calibrated on its page). --cam-h ID=m and
 * --cam-pitch ID=deg: that phone's camera height and tilt, for a ground-plane range from the
 * feet row, logged next to the box-height range ("range check") and averaged in with
 * --ground-range.
 * --no-pose-flip turns off the tag flip catch (SEEKER_POSE_FLIP_*: a live pose that is the
 * mirror solution of a flat tag is taken as its mirror; the jump gate alone decides).
 * --no-handoff turns off the phone-to-phone handoff (seeker_fusion.h SEEKER_HANDOFF_*: partial
 * contacts by bearing, covariance track gate, glide on a node set change, learned range ratios)
 * and runs the old track identity code.
 * --no-crouch-range takes a crouched person's eye-to-ankle range as the phone sent it, instead
 * of with the crouch height (seeker_fusion.h SEEKER_CROUCH_EYE_ANKLE_FRAC).
 * --no-occl-guard turns off the occlusion guard (seeker_fusion.h SEEKER_OCCL_*: a whole sighting
 * near where the same phone just saw part of someone held partial, skeleton joints off the frame or
 * guessed in a partial view not taken, confirmed joint jumps, no glide from where an unseen person
 * would have walked on). The guard needs the handoff.
 * --no-pose-smooth takes every live message pose as is (no One Euro, SEEKER_POSE_*_CUTOFF).
 * --no-pose-gate turns off the jump gate on live message poses (seeker_fusion.h): every full
 * message pose is then taken at once, one-frame tag flips included.
 * --no-skel-smooth sends ghost skeletons (phone pts) unfiltered, for comparing on the HUD; the
 * visibility cut and the short holds stay (fusion/seeker_skel.h).
 * --wearer-node overrides fusion.wearer_node: track range, bearing and colour are measured from
 * that node's pose (the radar centres on it), not from the room origin.
 * --hfov ID=deg|preset gives a phone's real horizontal FOV (presets iphone17pro, iphone16,
 * iphone-ultrawide pick landscape or portrait from the frame size); its bearings, computed on
 * the phone with the FOV it reports (?hfov=, default 66), are rescaled. No phone link change.
 * --predict-ms overrides fusion.predict_ms (100): each tick moves a track on by its velocity, at
 * most this far past its newest datagram, so people glide at the HUD's 30 fps instead of
 * stepping at the phones' 15 Hz. 0 turns it off.
 * --tag-pose gives the room pose of ArUco tag TAG (> 0; tag 0 is the room origin): its centre
 * x_m,y_m and the room yaw of the way you face looking straight at it. A message pose that names
 * the tag it was measured from (pose.tag_id, phone.html ?tag=1&tagid=0,7) is moved into the room
 * frame with it; tag_id 0 is the room frame as is; a tag with no --tag-pose counts as pose null.
 * Repeat it per tag. A switch between tags goes through the jump gate (taken after 3 consistent
 * frames when the two tags disagree by more than 0.75 m or 30 deg).
 * --tag-frame is for a phone that sees one tag and does not name it (?tagid=7 on a page without
 * pose.tag_id): that node's untagged message poses are in the frame of a tag at the given room
 * pose. The node is not pinned and starts 2 m in front of that tag, facing it, until its first
 * tag fix (--init-pose replaces that start). Pinning the same node with --pose or --poses is an
 * error. A pose that names its tag_id uses --tag-pose instead.
 */

#include "seeker_config.h"
#include "seeker_fusion.h"
#include "seeker_health.h"
#include "seeker_log.h"
#include "seeker_msgs.h"
#include "seeker_pose.h"
#include "seeker_time.h"
#include "seeker_udp.h"

#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static void debug_line(const seeker_remote_contacts *m, const struct sockaddr_storage *from)
{
    char ip[SEEKER_UDP_ADDRSTRLEN];
    seeker_udp_ntop(from, ip, sizeof ip);
    if (m->n == 0) {
        seeker_log("rx %s from %s  0 contacts", m->node_id, ip);
        return;
    }
    const seeker_contact *c = &m->c[0];
    char rng[16];
    if (c->has_range)
        snprintf(rng, sizeof rng, "%.1f m", c->range_m);
    else
        snprintf(rng, sizeof rng, "n/a");
    seeker_log("rx %s from %s  %d contact%s  %s bearing %.1f  range %s  box %s", m->node_id, ip,
               m->n, m->n == 1 ? "" : "s", c->id, c->bearing_deg, rng, c->has_box ? "yes" : "no");
}

static void usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [--config f] [--debug] [--run-for s] [--no-health]\n"
            "          [--no-smooth] [--smooth-alpha 0..1] [--smooth-beta 0..1] [--smooth-gate m]\n"
            "          [--no-range-smooth] [--no-pose-gate] [--no-pose-smooth] [--no-pose-flip] [--no-handoff]\n"
            "          [--no-crouch-range] [--no-occl-guard]\n"
            "          [--no-skel-smooth] [--person-h m] [--range-scale ID=k] [--no-range-hfov]\n"
            "          [--cam-h ID=m] [--cam-pitch ID=deg] [--ground-range]\n"
            "          [--wearer-node ID] [--predict-ms N] [--hfov ID=deg|preset]...\n"
            "          [--pose ID=x_m,y_m,yaw_deg]... [--init-pose ID=x_m,y_m,yaw_deg]... [--poses FILE] [--pose-source message|config]\n"
            "          [--tag-pose TAG=x_m,y_m,yaw_deg]... [--tag-frame ID=tag_x_m,tag_y_m,tag_yaw_deg]...\n",
            argv0);
}

static void debug_fused(const seeker_tracks *t)
{
    for (int i = 0; i < t->n; i++) {
        const seeker_track *k = &t->tracks[i];
        if (!strchr(k->nodes, '+'))
            continue;
        if (k->has_room)
            seeker_log("fused track %d %s room %.2f,%.2f range %.2f m bearing %.1f conf %.2f", k->id,
                       k->nodes, k->room_x_m, k->room_y_m, k->range_m, k->bearing_deg,
                       k->range_conf);
        else
            seeker_log("fused track %d %s (no fix)", k->id, k->nodes);
    }
}

int main(int argc, char **argv)
{
    const char *cfg_path = "config/seeker.json";
    const char *pose_file = NULL, *pose_source = NULL;
    int debug = 0, use_health = 1;
    double run_for = 0;
    int no_smooth = 0, no_range_smooth = 0, no_pose_gate = 0, no_skel_smooth = 0, predict_ms = -1;
    int no_pose_smooth = 0, no_pose_flip = 0, no_handoff = 0, no_crouch_range = 0, no_occl_guard = 0;
    const char *wearer_node = NULL;
    const char *hfov_args[SEEKER_MAX_NODES];
    int n_hfov = 0;
    const char *node_args[3 * SEEKER_MAX_NODES][2]; /* --range-scale, --cam-h, --cam-pitch */
    int n_node_args = 0, no_range_hfov = 0, ground_range = 0;
    float person_h = 0;
    double alpha = -1, beta = -1, gate = -1;
    static seeker_pose_table init_poses, cli_poses, tag_frames, tag_poses;
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--config") && i + 1 < argc)
            cfg_path = argv[++i];
        else if (!strcmp(argv[i], "--tag-pose") && i + 1 < argc) {
            char id[SEEKER_NODE_LEN], err[96];
            seeker_pose p;
            int bad = seeker_pose_parse(argv[++i], id, &p, err, sizeof err) != 0;
            if (!bad && (strspn(id, "0123456789") != strlen(id) || atoi(id) <= 0)) {
                snprintf(err, sizeof err, "want TAG=x_m,y_m,yaw_deg with TAG a tag id > 0 "
                                          "(tag 0 is the room origin)");
                bad = 1;
            }
            if (bad || seeker_pose_table_set(&tag_poses, id, &p, "tag") != 0) {
                fprintf(stderr, "--tag-pose %s: %s\n", argv[i], bad ? err : "too many tags");
                return 2;
            }
        }
        else if (!strcmp(argv[i], "--tag-frame") && i + 1 < argc) {
            char id[SEEKER_NODE_LEN], err[96];
            seeker_pose p;
            if (seeker_pose_parse(argv[++i], id, &p, err, sizeof err) != 0 ||
                seeker_pose_table_set(&tag_frames, id, &p, "tag") != 0) {
                fprintf(stderr, "--tag-frame %s: %s\n", argv[i],
                        tag_frames.n >= SEEKER_MAX_NODES ? "too many nodes" : err);
                return 2;
            }
        }
        else if (!strcmp(argv[i], "--pose") && i + 1 < argc) {
            char id[SEEKER_NODE_LEN], err[96];
            seeker_pose p;
            if (seeker_pose_parse(argv[++i], id, &p, err, sizeof err) != 0 ||
                seeker_pose_table_set(&cli_poses, id, &p, "cli") != 0) {
                fprintf(stderr, "--pose %s: %s\n", argv[i],
                        cli_poses.n >= SEEKER_MAX_NODES ? "too many nodes" : err);
                return 2;
            }
        } else if (!strcmp(argv[i], "--init-pose") && i + 1 < argc) {
            char id[SEEKER_NODE_LEN], err[96];
            seeker_pose p;
            if (seeker_pose_parse(argv[++i], id, &p, err, sizeof err) != 0 ||
                seeker_pose_table_set(&init_poses, id, &p, "init") != 0) {
                fprintf(stderr, "--init-pose %s: %s\n", argv[i],
                        init_poses.n >= SEEKER_MAX_NODES ? "too many nodes" : err);
                return 2;
            }
        } else if (!strcmp(argv[i], "--poses") && i + 1 < argc)
            pose_file = argv[++i];
        else if (!strcmp(argv[i], "--pose-source") && i + 1 < argc &&
                 (!strcmp(argv[i + 1], "message") || !strcmp(argv[i + 1], "config")))
            pose_source = argv[++i];
        else if (!strcmp(argv[i], "--debug"))
            debug = 1;
        else if (!strcmp(argv[i], "--run-for") && i + 1 < argc)
            run_for = atof(argv[++i]);
        else if (!strcmp(argv[i], "--no-health"))
            use_health = 0;
        else if (!strcmp(argv[i], "--no-smooth"))
            no_smooth = 1;
        else if (!strcmp(argv[i], "--no-range-smooth"))
            no_range_smooth = 1;
        else if (!strcmp(argv[i], "--no-pose-gate"))
            no_pose_gate = 1;
        else if (!strcmp(argv[i], "--no-pose-smooth"))
            no_pose_smooth = 1;
        else if (!strcmp(argv[i], "--no-pose-flip"))
            no_pose_flip = 1;
        else if (!strcmp(argv[i], "--no-handoff"))
            no_handoff = 1;
        else if (!strcmp(argv[i], "--no-crouch-range"))
            no_crouch_range = 1;
        else if (!strcmp(argv[i], "--no-occl-guard"))
            no_occl_guard = 1;
        else if (!strcmp(argv[i], "--no-skel-smooth"))
            no_skel_smooth = 1;
        else if (!strcmp(argv[i], "--wearer-node") && i + 1 < argc)
            wearer_node = argv[++i];
        else if (!strcmp(argv[i], "--hfov") && i + 1 < argc && n_hfov < SEEKER_MAX_NODES)
            hfov_args[n_hfov++] = argv[++i];
        else if ((!strcmp(argv[i], "--range-scale") || !strcmp(argv[i], "--cam-h") ||
                  !strcmp(argv[i], "--cam-pitch")) &&
                 i + 1 < argc && n_node_args < 3 * SEEKER_MAX_NODES) {
            node_args[n_node_args][0] = argv[i];
            node_args[n_node_args++][1] = argv[++i];
        }
        else if (!strcmp(argv[i], "--person-h") && i + 1 < argc)
            person_h = (float)atof(argv[++i]);
        else if (!strcmp(argv[i], "--no-range-hfov"))
            no_range_hfov = 1;
        else if (!strcmp(argv[i], "--ground-range"))
            ground_range = 1;
        else if (!strcmp(argv[i], "--predict-ms") && i + 1 < argc)
            predict_ms = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--smooth-alpha") && i + 1 < argc)
            alpha = atof(argv[++i]);
        else if (!strcmp(argv[i], "--smooth-beta") && i + 1 < argc)
            beta = atof(argv[++i]);
        else if (!strcmp(argv[i], "--smooth-gate") && i + 1 < argc)
            gate = atof(argv[++i]);
        else {
            usage(argv[0]);
            return 2;
        }
    }
    seeker_log_init("hud_fusion");
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    static seeker_config cfg;
    seeker_config_load(&cfg, cfg_path);
    if (no_smooth)
        cfg.smooth = 0;
    if (no_range_smooth)
        cfg.range_smooth = 0;
    if (alpha >= 0)
        cfg.smooth_alpha = (float)alpha;
    if (beta >= 0)
        cfg.smooth_beta = (float)beta;
    if (gate >= 0)
        cfg.smooth_gate_m = (float)gate;
    if (cfg.smooth_alpha <= 0 || cfg.smooth_alpha > 1 || cfg.smooth_beta < 0 ||
        cfg.smooth_beta > 1) {
        seeker_log("smooth_alpha must be in (0, 1] and smooth_beta in [0, 1]");
        return 2;
    }
    if (cfg.range_smooth_alpha <= 0 || cfg.range_smooth_alpha > 1 || cfg.range_smooth_beta < 0 ||
        cfg.range_smooth_beta > 1) {
        seeker_log("range_smooth_alpha must be in (0, 1] and range_smooth_beta in [0, 1]");
        return 2;
    }
    if (pose_source)
        cfg.pose_from_config = !strcmp(pose_source, "config");
    if (wearer_node)
        snprintf(cfg.wearer_node, sizeof cfg.wearer_node, "%s", wearer_node);
    if (predict_ms >= 0)
        cfg.predict_ms = predict_ms;
    static seeker_fusion fus;
    seeker_fusion_init(&fus, &cfg);
    fus.debug = debug;
    fus.pose_gate = !no_pose_gate;
    fus.pose_smooth = !no_pose_smooth;
    fus.pose_flip = !no_pose_flip;
    fus.handoff = !no_handoff;
    fus.crouch_range = !no_crouch_range;
    fus.occl_guard = !no_occl_guard && !no_handoff;
    fus.skel_smooth = !no_skel_smooth;
    for (int i = 0; i < n_hfov; i++) {
        char id[SEEKER_NODE_LEN];
        const char *eq = strchr(hfov_args[i], '=');
        size_t len = eq ? (size_t)(eq - hfov_args[i]) : 0;
        if (!eq || len == 0 || len >= sizeof id) {
            fprintf(stderr, "--hfov %s: want ID=degrees or ID=preset\n", hfov_args[i]);
            return 2;
        }
        memcpy(id, hfov_args[i], len);
        id[len] = '\0';
        int k = seeker_fusion_set_hfov(&fus, id, eq + 1);
        if (k < 0) {
            fprintf(stderr, "--hfov %s: want degrees (10..170) or a preset: iphone17pro, "
                            "iphone16, iphone-ultrawide\n", hfov_args[i]);
            return 2;
        }
        seeker_log("node %s real horizontal FOV %.1f deg landscape, %.1f portrait (%s): its "
                   "bearings%s are rescaled from the FOV the phone reports", id,
                   fus.nodes[k].hfov_set, fus.nodes[k].hfov_set_portrait, eq + 1,
                   no_range_hfov ? "" : " and box-height ranges");
    }
    fus.range_hfov = !no_range_hfov;
    fus.ground_range = ground_range;
    if (person_h > 0) {
        if (person_h < 1.0f || person_h > 2.3f) {
            fprintf(stderr, "--person-h %.2f: want metres, 1.0..2.3\n", person_h);
            return 2;
        }
        fus.person_h = person_h;
        seeker_log("people's height %.2f m: box-height ranges x %.3f (phone.html assumes %.2f m)",
                   person_h, person_h / SEEKER_SKEL_PERSON_M, SEEKER_SKEL_PERSON_M);
    }
    for (int i = 0; i < n_node_args; i++) {
        const char *flag = node_args[i][0], *a = node_args[i][1], *eq = strchr(a, '=');
        char id[SEEKER_NODE_LEN];
        size_t len = eq ? (size_t)(eq - a) : 0;
        char *end = NULL;
        float v = eq ? strtof(eq + 1, &end) : 0;
        if (!eq || len == 0 || len >= sizeof id || !end || end == eq + 1 || *end) {
            fprintf(stderr, "%s %s: want ID=number\n", flag, a);
            return 2;
        }
        memcpy(id, a, len);
        id[len] = '\0';
        int rc;
        if (!strcmp(flag, "--range-scale")) {
            rc = seeker_fusion_set_range_scale(&fus, id, v);
            if (rc == 0)
                seeker_log("node %s box-height ranges x %.3f (--range-scale)", id, v);
        } else {
            float h = 0, p = 0;
            for (int k = 0; k < fus.n_nodes; k++)
                if (!strcmp(fus.nodes[k].id, id))
                    h = fus.nodes[k].cam_h, p = fus.nodes[k].cam_pitch;
            if (!strcmp(flag, "--cam-h"))
                h = v;
            else
                p = v;
            rc = seeker_fusion_set_cam(&fus, id, h > 0.1f ? h : 1.5f, p);
            if (rc == 0)
                seeker_log("node %s camera %.2f m above the floor, tilted %.1f deg: ground-plane range "
                           "%s", id, h > 0.1f ? h : 1.5f, p,
                           ground_range ? "averaged into its ranges" : "logged as a check only");
        }
        if (rc != 0) {
            fprintf(stderr, "%s %s: out of range\n", flag, a);
            return 2;
        }
    }

    static seeker_pose_table file_poses;
    if (pose_file) {
        char err[160];
        int n = seeker_pose_load_file(&file_poses, pose_file, err, sizeof err);
        if (n < 0) {
            seeker_log("--poses %s", err);
            return 2;
        }
        seeker_log("%d pose%s from %s", n, n == 1 ? "" : "s", pose_file);
    }
    for (int i = 0; i < tag_poses.n; i++) {
        const seeker_pose_entry *e = &tag_poses.e[i];
        if (seeker_fusion_set_tag(&fus, atoi(e->id), &e->pose) != 0) {
            seeker_log("tag table full (%d), cannot add tag %s", SEEKER_MAX_TAGS, e->id);
            return 2;
        }
        seeker_log("tag %s at room %.3f,%.3f,%.1f: poses a phone sends from it (pose.tag_id %s) "
                   "are moved into the room frame", e->id, e->pose.x_m, e->pose.y_m,
                   e->pose.yaw_deg, e->id);
    }
    for (int i = 0; i < tag_frames.n; i++) {
        const seeker_pose_entry *e = &tag_frames.e[i];
        if (seeker_pose_table_get(&cli_poses, e->id) || seeker_pose_table_get(&file_poses, e->id)) {
            seeker_log("--tag-frame %s and a pin (--pose/--poses) for %s: a tag-frame node moves, "
                       "drop its pin", e->id, e->id);
            return 2;
        }
        if (seeker_fusion_set_tag_frame(&fus, e->id, &e->pose) < 0) {
            seeker_log("node table full (%d), cannot add %s", SEEKER_MAX_NODES, e->id);
            return 2;
        }
    }
    for (int i = 0; i < init_poses.n; i++) {
        const seeker_pose_entry *e = &init_poses.e[i];
        if (seeker_fusion_set_pose(&fus, e->id, &e->pose, e->src, 0) < 0) {
            seeker_log("node table full (%d), cannot add %s", SEEKER_MAX_NODES, e->id);
            return 2;
        }
    }
    const seeker_pose_table *tables[2] = { &file_poses, &cli_poses }; /* cli last, so it wins */
    for (int t = 0; t < 2; t++)
        for (int i = 0; i < tables[t]->n; i++) {
            const seeker_pose_entry *e = &tables[t]->e[i];
            if (seeker_fusion_set_pose(&fus, e->id, &e->pose, e->src, 1) < 0) {
                seeker_log("node table full (%d), cannot add %s", SEEKER_MAX_NODES, e->id);
                return 2;
            }
        }
    for (int i = 0; i < fus.n_nodes; i++) {
        const seeker_fusion_node *n = &fus.nodes[i];
        if (n->has_tag_frame) {
            seeker_log("node %s pose %.2f,%.2f,%.1f (%s, not pinned; message poses without tag_id "
                       "are in the frame of a tag at room %.3f,%.3f,%.1f)", n->id, n->x_m, n->y_m, n->yaw_deg,
                       n->pose_src, n->tag_frame.x_m, n->tag_frame.y_m, n->tag_frame.yaw_deg);
            continue;
        }
        seeker_log("node %s pose %.2f,%.2f,%.1f (%s%s)", n->id, n->x_m, n->y_m, n->yaw_deg,
                   n->pose_src,
                   n->pose_pinned ? ", pinned" : cfg.pose_from_config ? ", messages ignored"
                                                                      : ", messages may replace");
    }
    /* Plain words for the setup check: where each other phone is from the wearer's pose. */
    for (int i = 0; i < fus.n_nodes; i++) {
        const seeker_fusion_node *n = &fus.nodes[i];
        float right, ahead;
        if (!cfg.wearer_node[0] || !strcmp(n->id, cfg.wearer_node) ||
            seeker_fusion_relative(&fus, cfg.wearer_node, n->id, &right, &ahead) != 0)
            continue;
        if (fabsf(right) < 0.05f)
            seeker_log("node %s is straight %s of %s's starting pose, %.2f m away%s (not to the "
                       "side: check its x)", n->id, ahead < 0 ? "behind" : "ahead",
                       cfg.wearer_node, fabsf(ahead), n->pose_pinned ? ", pinned" : "");
        else
            seeker_log("node %s is %.2f m to the %s and %.2f m %s of %s's starting pose%s", n->id,
                       fabsf(right), right < 0 ? "LEFT" : "RIGHT", fabsf(ahead),
                       ahead < 0 ? "behind" : "ahead", cfg.wearer_node,
                       n->pose_pinned ? " (pinned)" : "");
    }

    seeker_udp_rx rx;
    if (seeker_udp_listen(&rx, SEEKER_PORT_REMOTE) != 0) {
        seeker_log("cannot bind udp/%d", SEEKER_PORT_REMOTE);
        return 1;
    }
    int tx = socket(AF_INET, SOCK_DGRAM, 0);
    struct sockaddr_in dst;
    if (tx < 0 || seeker_udp_addr(cfg.tracks_host, SEEKER_PORT_TRACKS, &dst) != 0) {
        seeker_log("cannot set up tracks output to %s:%d", cfg.tracks_host, SEEKER_PORT_TRACKS);
        return 1;
    }

    seeker_health_table *health = use_health ? seeker_health_open(0) : NULL;
    int slot = seeker_health_slot_for(health, "hud_fusion");
    if (use_health && !health)
        seeker_log("no /seeker_health (shm_open failed); running without heartbeats");

    seeker_log("listening on udp/%d (%s), tracks -> %s:%d at %.0f Hz, drop after %d ms",
               SEEKER_PORT_REMOTE, rx.n == 2 ? "IPv4 + IPv6" : "IPv4 only", cfg.tracks_host,
               SEEKER_PORT_TRACKS, cfg.fusion_rate_hz, cfg.drop_ms);
    if (fus.pose_gate && fus.pose_flip)
        seeker_log("live poses (tags): a jump that is the tag's mirror solution (within %.2f m, "
                   "%.0f deg) is taken as its mirror; switching only after %d of %d frames "
                   "(--no-pose-flip: off)", SEEKER_POSE_FLIP_M, SEEKER_POSE_FLIP_DEG,
                   SEEKER_POSE_FLIP_SWITCH, SEEKER_POSE_FLIP_WINDOW);
    if (fus.pose_smooth)
        seeker_log("live poses (tags): One Euro, min cutoff %.1f Hz, beta %.2f Hz per m/s and %.3f "
                   "per deg/s (--no-pose-smooth: raw)", SEEKER_POSE_MIN_CUTOFF_HZ,
                   SEEKER_POSE_BETA_M, SEEKER_POSE_BETA_DEG);
    if (cfg.smooth)
        seeker_log("smoothing tracks: alpha %.2f beta %.2f gate %.1f m", cfg.smooth_alpha,
                   cfg.smooth_beta, cfg.smooth_gate_m);
    else
        seeker_log("smoothing off");
    if (cfg.smooth && cfg.range_smooth)
        seeker_log("single-phone range filter: alpha %.2f beta %.3f gate max(%.1f m, %.2f x range)",
                   cfg.range_smooth_alpha, cfg.range_smooth_beta, cfg.smooth_gate_m,
                   cfg.range_smooth_gate_frac);
    seeker_log("triangulation: bearing sigma %.1f deg, size range sigma sqrt((%.2f r)^2 + "
               "(%.3f r^2)^2), size range below %.0f deg crossing (bearing-only %.0f deg)",
               fus.tri.bearing_sigma_deg, fus.tri.range_sigma_frac, fus.tri.range_sigma_k,
               fus.tri.min_cross_deg, fus.tri.min_cross_bearing_only_deg);
    seeker_log("track identity: gate max(%.1f m, min(%.1f sigma, %.1f m)), coast %d ms",
               cfg.smooth_gate_m, cfg.track_gate_sigmas, cfg.track_gate_max_m, cfg.drop_ms);
    if (fus.handoff)
        seeker_log("handoff: partial contacts by bearing (x%.0f sigma), covariance gate %.0f sigma (+1 on "
                   "a node set change), glide %d ms, learned range ratios fade over %.0f s",
                   fus.tri.partial_bearing_k, cfg.track_gate_sigmas, SEEKER_HANDOFF_MS,
                   SEEKER_HANDOFF_RK_TAU_MS / 1000.0f);
    else
        seeker_log("handoff: off (--no-handoff), old track identity");
    if (fus.crouch_range)
        seeker_log("body range: a range from a length shorter than the person (crouched, bent, legs "
                   "hidden) is scaled by that length over the person's own torso, shoulders, thigh "
                   "or shin (--no-crouch-range: as sent)");
    else
        seeker_log("crouch range: off (--no-crouch-range), ranges as the phones sent them");
    if (fus.occl_guard)
        seeker_log("occlusion guard: a whole sighting within %.0f deg of a part seen in the last %d ms held "
                   "partial for %d messages, joints off the frame or under %.1f in a partial view not "
                   "taken, joint jumps confirmed, handoff glide from at most %d ms ahead",
                   SEEKER_OCCL_BEARING_DEG, SEEKER_OCCL_HOLD_MS, SEEKER_OCCL_WHOLE_N - 1, SEEKER_OCCL_FRAG_VIS,
                   cfg.predict_ms);
    else
        seeker_log("occlusion guard: off%s", no_handoff ? " (needs the handoff)" : " (--no-occl-guard)");
    if (fus.skel_smooth)
        seeker_log("skeletons (pts): One Euro per joint (min cutoff %.1f Hz, beta %.1f), joints "
                   "under visibility %.2f held %d ms, whole skeleton held %d ms, up to %d per "
                   "datagram",
                   SEEKER_SKEL_MIN_CUTOFF_HZ, SEEKER_SKEL_BETA, SEEKER_SKEL_VIS_MIN,
                   SEEKER_SKEL_JOINT_HOLD_MS, SEEKER_SKEL_HOLD_MS, SEEKER_PTS_MAX_CARRIERS);
    else
        seeker_log("skeletons (pts): unfiltered (--no-skel-smooth)");
    {
        seeker_posture_tune pt;
        char bad[160], line[400];
        seeker_posture_tune_defaults(&pt);
        int n_env = seeker_posture_tune_env(&pt, bad, sizeof bad);
        seeker_posture_set_tune(&pt);
        seeker_posture_tune_describe(&pt, line, sizeof line);
        seeker_log("posture: %s%s", line, n_env ? " (SEEKER_POSTURE_* env)" : "");
        if (bad[0])
            seeker_log("posture: ignored env %s (not a number in range; see common/seeker_posture.h)",
                       bad);
    }

    const int64_t period = (int64_t)(1000.0 / (cfg.fusion_rate_hz > 0 ? cfg.fusion_rate_hz : 30));
    const int64_t t_start = seeker_now_ms();
    int64_t next_tick = t_start;
    long n_rx = 0, n_bad = 0;
    int64_t last_fused_log = 0;
    static char buf[SEEKER_MAX_DATAGRAM + 1];
    static char out[SEEKER_MAX_DATAGRAM];
    static seeker_remote_contacts msg;
    static seeker_tracks tracks;

    while (!g_stop) {
        int64_t now = seeker_now_ms();
        if (run_for > 0 && now - t_start >= (int64_t)(run_for * 1000))
            break;
        int wait = (int)(next_tick - now);
        struct sockaddr_storage from;
        memset(&from, 0, sizeof from);
        int n = seeker_udp_recv(&rx, buf, sizeof buf, wait > 0 ? wait : 0, &from);
        int64_t t_rx = seeker_now_ms();
        if (n > 0) {
            char err[64];
            if (seeker_decode_remote_contacts(buf, n, &msg, err, sizeof err) == 0) {
                seeker_fusion_ingest(&fus, &msg, t_rx);
                n_rx++;
                if (debug)
                    debug_line(&msg, &from);
            } else if (++n_bad <= 5 || n_bad % 100 == 0) {
                seeker_log("dropped datagram (%d bytes): %s [%ld bad so far]", n, err, n_bad);
            }
        }
        if (t_rx >= next_tick) {
            int64_t t0 = seeker_now_ms();
            seeker_fusion_tick(&fus, t_rx, &tracks);
            if (debug && t_rx - last_fused_log >= 1000) {
                last_fused_log = t_rx;
                debug_fused(&tracks);
            }
            int len = seeker_encode_tracks(&tracks, out, sizeof out);
            if (len > 0)
                sendto(tx, out, (size_t)len, 0, (struct sockaddr *)&dst, sizeof dst);
            else
                seeker_log("tracks message over %d bytes, not sent", SEEKER_MAX_DATAGRAM);
            seeker_health_beat(health, slot, (int)(seeker_now_ms() - t0), SH_OK);
            next_tick += period;
            if (next_tick < t_rx)
                next_tick = t_rx + period; /* fell behind: skip, never queue */
        }
    }
    seeker_log("exit: %ld datagrams, %ld rejected", n_rx, n_bad);
    seeker_udp_close(&rx);
    close(tx);
    return 0;
}
