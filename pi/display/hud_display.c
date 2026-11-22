/*
 * hud_display: tracks (UDP 47102, IF-5.2 plus the PROPOSED nodes list) in, HUD out.
 * Never blocks on fusion: each frame takes the newest datagram, if any, and draws. If fusion goes
 * quiet the last tracks are drawn grey under a FUSION LOST banner (IF-3, IF-7 stale rules).
 * IF-5.6 video arrives on TCP 47106 in a receiver thread (video_rx.c) that decodes and scales off
 * this loop; each frame here only try-locks for the newest picture.
 *
 *   hud_display [--config f] [--sink screen|ppm|none] [--background radar|wearer]
 *               [--view-node R1] [--mirror none|h|v] [--ppm path] [--ppm-every n]
 *               [--run-for s] [--no-health] [--video-port n] [--video-prio n] [--no-video]
 *               [--frame-sync on|off] [--screen-buffer WxH] [--vsync on|off]
 *               EagleEye wearer view (docs/EAGLEEYE.md):
 *               [--view radar|wearer] [--wearer-node W] [--wearer-pose [W=]x,y,yaw]
 *               [--wearer-pose-src auto|static|fusion] [--wearer-hfov deg] [--wearer-image WxH]
 *               [--wearer-cam-h m] [--wearer-pitch deg] [--xray-up deg] [--person-h m]
 *               [--pitch-auto on|off] [--wall x1,y1,x2,y2|none]... [--no-walls] [--visor] [--no-inset] [--mirror]
 *               [--video-fallback black|radar] [--handoff on|off]
 *               [--pose ID=x,y,yaw]... [--poses FILE]   (hud_fusion's format; the wearer
 *               node's entry pins its pose)
 *
 * Latency (latency.h): every wearer frame drawn is timed from its SKV2 ts_ms to the screen post;
 * hud_display.log gets a "latency:" line every 10 s, every second with SEEKER_LAT=1, which also
 * puts "LAT 120MS P95 180MS FPS 28" top right on the HUD. SEEKER_LAT_BASE_MS adds the fixed
 * part measured once with the stopwatch photo (docs/LIVE-RUN.md "Latency").
 * --frame-sync on (default): a new video frame is drawn as soon as it is decoded, not on the
 * next 1/fps tick (up to 33 ms sooner); off: the fixed 1/fps loop only.
 * --screen-buffer 1920x1080: the screen sink draws at that size and QNX Screen scales it to the
 * display (a 4K monitor otherwise means 4x the pixels to draw, scale video into and copy).
 */

#include "anim_map.h"
#include "hud_render.h"
#include "latency.h"
#include "seeker_config.h"
#include "seeker_health.h"
#include "seeker_log.h"
#include "seeker_msgs.h"
#include "seeker_pose.h"
#include "seeker_time.h"
#include "seeker_udp.h"
#include "sink.h"
#include "video_proto.h"
#include "video_rx.h"

#include <math.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static volatile sig_atomic_t g_stop;

static void on_signal(int sig)
{
    (void)sig;
    g_stop = 1;
}

static void log_eagle(const wv_view *v)
{
    const wv_stats *s = &v->last;
    if (!s->rendered) {
        seeker_log("eagleeye: %s view not drawn: radar fallback (no fresh video)", v->cfg.node);
        return;
    }
    seeker_log("eagleeye: %s %s pose %s %.2f,%.2f,%.1f  seen %d  xray %d  off %d  dup %d  "
               "unplaced %d  video %s  skel %d  stand %d crouch %d down %d",
               v->cfg.node, s->wearer_state[0] ? s->wearer_state : "absent",
               wv_pose_mode_name(s->pose_mode), s->pose.x_m, s->pose.y_m, s->pose.yaw_deg,
               s->n_direct, s->n_xray, s->n_off, s->n_dup, s->n_unplaced,
               s->video > 0 ? "on" : s->video < 0 ? "other-node" : "off", s->n_skel,
               s->n_posture[SEEKER_POSTURE_STAND], s->n_posture[SEEKER_POSTURE_CROUCH],
               s->n_posture[SEEKER_POSTURE_DOWN]);
}

static int usage(const char *argv0)
{
    fprintf(stderr,
            "usage: %s [--config f] [--sink screen|ppm|none] [--view|--background radar|wearer]\n"
            "       [--view-node Rn] [--mirror [none|h|v]] [--ppm path] [--ppm-every n]\n"
            "       [--run-for s] [--no-health] [--video-port n] [--video-prio n] [--no-video]\n"
            "       [--frame-sync on|off] [--screen-buffer WxH] [--vsync on|off]\n"
            "       [--theme plan|classic]\n"
            "  wearer view: [--wearer-node W] [--wearer-pose [W=]x,y,yaw]\n"
            "       [--wearer-pose-src auto|static|fusion] [--wearer-hfov deg]\n"
            "       [--wearer-image WxH] [--wearer-cam-h m] [--wearer-pitch deg] [--xray-up deg]\n"
            "       [--person-h m] [--pitch-auto on|off] [--wall x1,y1,x2,y2|none]... [--no-walls]\n"
            "       [--visor] [--no-inset] [--video-fallback black|radar] [--handoff on|off]\n"
            "       [--pose ID=x,y,yaw]... [--poses FILE]\n",
            argv0);
    return 2;
}

int main(int argc, char **argv)
{
    const char *cfg_path = "config/seeker.json";
    const char *sink_kind = NULL, *bg = NULL, *view = NULL, *mirror = NULL, *ppm = NULL;
    const char *theme = NULL;
    int ppm_every = 0, use_health = 1, use_video = 1, video_port = SEEKER_PORT_VIDEO;
    int video_prio = -1, frame_sync = 1, buf_w = 0, buf_h = 0, vsync = 1;
    double run_for = 0;
    const char *w_node = NULL, *w_pose = NULL, *w_src = NULL, *w_image = NULL, *w_fallback = NULL;
    float w_hfov = 0, w_cam_h = 0, w_pitch = 0, w_xray_up = 0, w_person_h = 0;
    int w_pitch_set = 0, w_up_set = 0, w_pitch_auto = -1, w_handoff = -1;
    int w_visor = 0, w_no_inset = 0, n_walls = 0, clear_walls = 0;
    const char *view_mode = NULL;
    float walls[WV_MAX_WALLS][4];
    static seeker_pose_table poses; /* --pose / --poses, the hud_fusion format */
    seeker_pose_table cli_poses = { 0 };
    const char *poses_file = NULL;
    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];
        const char *v = i + 1 < argc ? argv[i + 1] : NULL;
        if (!strcmp(a, "--config") && v) cfg_path = argv[++i];
        else if (!strcmp(a, "--sink") && v) sink_kind = argv[++i];
        else if (!strcmp(a, "--background") && v) bg = argv[++i];
        else if (!strcmp(a, "--theme") && v) theme = argv[++i];
        else if (!strcmp(a, "--view") && v) bg = view_mode = argv[++i];
        else if (!strcmp(a, "--view-node") && v) view = argv[++i];
        else if (!strcmp(a, "--mirror")) /* bare --mirror: horizontal, for the visor */
            mirror = v && strncmp(v, "--", 2) ? argv[++i] : "h";
        else if (!strcmp(a, "--ppm") && v) ppm = argv[++i];
        else if (!strcmp(a, "--ppm-every") && v) ppm_every = atoi(argv[++i]);
        else if (!strcmp(a, "--run-for") && v) run_for = atof(argv[++i]);
        else if (!strcmp(a, "--no-health")) use_health = 0;
        else if (!strcmp(a, "--wearer-node") && v) w_node = argv[++i];
        else if (!strcmp(a, "--wearer-pose") && v) w_pose = argv[++i];
        else if (!strcmp(a, "--poses") && v) poses_file = argv[++i];
        else if (!strcmp(a, "--pose") && v) {
            char id[SEEKER_NODE_LEN], err[96] = "too many poses";
            seeker_pose p;
            if (seeker_pose_parse(argv[++i], id, &p, err, sizeof err) != 0 ||
                seeker_pose_table_set(&cli_poses, id, &p, "cli") != 0) {
                fprintf(stderr, "--pose %s: %s\n", argv[i], err);
                return 2;
            }
        }
        else if (!strcmp(a, "--video-port") && v) video_port = atoi(argv[++i]);
        else if (!strcmp(a, "--video-prio") && v) video_prio = atoi(argv[++i]);
        else if (!strcmp(a, "--no-video")) use_video = 0;
        else if (!strcmp(a, "--screen-buffer") && v) {
            if (sscanf(argv[++i], "%dx%d", &buf_w, &buf_h) != 2 || buf_w < 320 || buf_h < 240 ||
                buf_w > 7680 || buf_h > 4320) {
                fprintf(stderr, "--screen-buffer wants WxH, e.g. 1920x1080, got %s\n", argv[i]);
                return 2;
            }
        }
        else if (!strcmp(a, "--frame-sync") && v) {
            const char *fs = argv[++i];
            if (strcmp(fs, "on") && strcmp(fs, "off")) {
                fprintf(stderr, "--frame-sync wants on or off, got %s\n", fs);
                return 2;
            }
            frame_sync = !strcmp(fs, "on");
        }
        else if (!strcmp(a, "--vsync") && v) {
            const char *vs = argv[++i];
            if (strcmp(vs, "on") && strcmp(vs, "off")) {
                fprintf(stderr, "--vsync wants on or off, got %s\n", vs);
                return 2;
            }
            vsync = !strcmp(vs, "on");
        }
        else if (!strcmp(a, "--wearer-pose-src") && v) w_src = argv[++i];
        else if (!strcmp(a, "--wearer-hfov") && v) w_hfov = (float)atof(argv[++i]);
        else if (!strcmp(a, "--wearer-image") && v) w_image = argv[++i];
        else if (!strcmp(a, "--wearer-cam-h") && v) w_cam_h = (float)atof(argv[++i]);
        else if (!strcmp(a, "--wearer-pitch") && v) w_pitch = (float)atof(argv[++i]), w_pitch_set = 1;
        else if (!strcmp(a, "--xray-up") && v) w_xray_up = (float)atof(argv[++i]), w_up_set = 1;
        else if (!strcmp(a, "--person-h") && v) w_person_h = (float)atof(argv[++i]);
        else if (!strcmp(a, "--pitch-auto") && v) {
            const char *pa = argv[++i];
            if (strcmp(pa, "on") && strcmp(pa, "off")) {
                fprintf(stderr, "--pitch-auto wants on or off, got %s\n", pa);
                return 2;
            }
            w_pitch_auto = !strcmp(pa, "on");
        }
        else if (!strcmp(a, "--handoff") && v) {
            const char *ho = argv[++i];
            if (strcmp(ho, "on") && strcmp(ho, "off")) {
                fprintf(stderr, "--handoff wants on or off, got %s\n", ho);
                return 2;
            }
            w_handoff = !strcmp(ho, "on");
        }
        else if (!strcmp(a, "--no-walls"))
            clear_walls = 1;
        else if (!strcmp(a, "--wall") && v) {
            const char *wv = argv[++i];
            if (!strcmp(wv, "none")) {
                clear_walls = 1;
                n_walls = 0;
            } else if (n_walls < WV_MAX_WALLS) {
                if (wv_parse_wall(wv, walls[n_walls]) != 0) {
                    fprintf(stderr, "--wall wants x1,y1,x2,y2 in metres (or none), got %s\n", wv);
                    return 2;
                }
                clear_walls = 1;
                n_walls++;
            }
        }
        else if (!strcmp(a, "--visor")) w_visor = 1;
        else if (!strcmp(a, "--video-fallback") && v) w_fallback = argv[++i];
        else if (!strcmp(a, "--no-inset")) w_no_inset = 1;
        else return usage(argv[0]);
    }
    seeker_log_init("hud_display");
    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    static seeker_config cfg;
    seeker_config_load(&cfg, cfg_path);
    if (sink_kind) snprintf(cfg.sink, sizeof cfg.sink, "%s", sink_kind);
    if (bg) snprintf(cfg.background, sizeof cfg.background, "%s", bg);
    if (theme) snprintf(cfg.theme, sizeof cfg.theme, "%s", theme);
    if (draw_theme_parse(cfg.theme) < 0) {
        seeker_log("unknown theme %s (plan or classic), using plan", cfg.theme);
        snprintf(cfg.theme, sizeof cfg.theme, "plan");
    }
    draw_set_theme(draw_theme_parse(cfg.theme));
    if (view) snprintf(cfg.view_node, sizeof cfg.view_node, "%s", view);
    if (mirror) snprintf(cfg.mirror, sizeof cfg.mirror, "%s", mirror);
    if (strcmp(cfg.mirror, "none") && strcmp(cfg.mirror, "h") && strcmp(cfg.mirror, "v")) {
        seeker_log("unknown mirror %s (none, h or v), using none", cfg.mirror);
        snprintf(cfg.mirror, sizeof cfg.mirror, "none");
    }
    if (strcmp(cfg.background, "radar") && strcmp(cfg.background, "wearer")) {
        seeker_log("unknown background %s, using radar", cfg.background);
        snprintf(cfg.background, sizeof cfg.background, "radar");
    }

    /* EagleEye wearer node: --wearer-node, the id in --wearer-pose, --view-node, display.wearer.node,
     * display.view_node, else "W". */
    static wv_view wearer;
    char pose_node[SEEKER_NODE_LEN] = "";
    wv_pose cli_pose;
    if (w_pose && wv_parse_pose(w_pose, pose_node, sizeof pose_node, &cli_pose) != 0) {
        fprintf(stderr, "--wearer-pose wants x,y,yaw or ID=x,y,yaw, got %s\n", w_pose);
        return 2;
    }
    if (poses_file) {
        char err[160];
        if (seeker_pose_load_file(&poses, poses_file, err, sizeof err) < 0) {
            fprintf(stderr, "--poses: %s\n", err);
            return 2;
        }
    }
    for (int i = 0; i < cli_poses.n; i++) /* --pose wins over --poses, as in hud_fusion */
        seeker_pose_table_set(&poses, cli_poses.e[i].id, &cli_poses.e[i].pose, "cli");
    const char *wn = w_node ? w_node : pose_node[0] ? pose_node : view;
    wv_config_load(&wearer.cfg, &cfg, cfg_path, wn);
    /* A pinned pose (as in hud_fusion) ignores the phone's message pose. */
    const seeker_pose_entry *pinned = seeker_pose_table_get(&poses, wearer.cfg.node);
    if (pinned) {
        wearer.cfg.pose = pinned->pose;
        wearer.cfg.pose_src = WV_POSE_STATIC;
    }
    if (w_pose) {
        wearer.cfg.pose = cli_pose;
        wearer.cfg.pose_src = WV_POSE_STATIC;
    }
    if (w_src && wv_parse_pose_src(w_src, &wearer.cfg.pose_src) != 0) {
        fprintf(stderr, "--wearer-pose-src wants auto, static or fusion, got %s\n", w_src);
        return 2;
    }
    if (w_hfov > 0) {
        wearer.cfg.cam.hfov_deg = w_hfov;
        wearer.cfg.hfov_fixed = 1;
    }
    if (w_cam_h > 0) wearer.cfg.cam.cam_h_m = w_cam_h;
    if (w_pitch_set) wearer.cfg.cam.pitch_deg = w_pitch;
    if (w_up_set) wearer.cfg.xray_up_deg = w_xray_up;
    if (w_person_h > 0) wearer.cfg.cam.person_h_m = w_person_h;
    if (w_pitch_auto >= 0) wearer.cfg.pitch_auto = w_pitch_auto;
    if (w_handoff >= 0) wearer.cfg.handoff = w_handoff;
    if (w_image && sscanf(w_image, "%dx%d", &wearer.cfg.cam.image_w, &wearer.cfg.cam.image_h) != 2) {
        fprintf(stderr, "--wearer-image wants WxH, got %s\n", w_image);
        return 2;
    }
    if (w_visor) wearer.cfg.visor = 1;
    if (w_fallback) {
        if (strcmp(w_fallback, "radar") && strcmp(w_fallback, "black")) {
            fprintf(stderr, "--video-fallback wants radar or black, got %s\n", w_fallback);
            return 2;
        }
        wearer.cfg.radar_fallback = !strcmp(w_fallback, "radar");
    }
    if (w_no_inset) wearer.cfg.inset = 0;
    if (clear_walls)
        wearer.cfg.n_walls = 0;
    for (int i = 0; i < n_walls && wearer.cfg.n_walls < WV_MAX_WALLS; i++)
        memcpy(wearer.cfg.walls[wearer.cfg.n_walls++], walls[i], sizeof walls[i]);
    /* EagleEye when asked for (--view wearer, a display.wearer block, or any wearer flag); plain
     * --background wearer keeps the IF-7 phone video view. */
    const int eagle_flags = w_node || w_pose || w_src || w_hfov > 0 || w_image || w_cam_h > 0 ||
                            n_walls || clear_walls || w_visor || w_no_inset || w_fallback ||
                            cli_poses.n || poses_file;
    const int eagle = !strcmp(cfg.background, "wearer") &&
                      (view_mode || wearer.cfg.from_config || eagle_flags);
    if (eagle) /* the video comes from the helmet phone */
        snprintf(cfg.view_node, sizeof cfg.view_node, "%s", wearer.cfg.node);

    seeker_sink sink;
    memset(&sink, 0, sizeof sink);
    if (ppm) snprintf(sink.ppm_path, sizeof sink.ppm_path, "%s", ppm);
    sink.ppm_every = ppm_every;
    sink.buf_w = buf_w;
    sink.buf_h = buf_h;
    sink.no_vsync = !vsync;
    if (seeker_sink_open(&sink, cfg.sink, &cfg) != 0) {
        seeker_log("sink %s failed to open", cfg.sink);
        return 1;
    }
    canvas cv = { calloc((size_t)sink.w * (size_t)sink.h, 4), sink.w, sink.h, sink.w };
    if (!cv.px)
        return 1;
    if (draw_theme() == DRAW_THEME_PLAN)
        draw_ui_init(sink.h); /* bake the HUD font set at its sizes before the first frame */

    seeker_udp_rx rx;
    if (seeker_udp_listen(&rx, SEEKER_PORT_TRACKS) != 0) {
        seeker_log("cannot bind udp/%d", SEEKER_PORT_TRACKS);
        return 1;
    }
    seeker_health_table *health = use_health ? seeker_health_open(0) : NULL;
    int slot = seeker_health_slot_for(health, "hud_display");

    struct sched_param sp;
    int policy = 0, prio = 0;
    if (pthread_getschedparam(pthread_self(), &policy, &sp) == 0)
        prio = sp.sched_priority;
    seeker_log("sink %s %dx%d, background %s%s%s, mirror %s, priority %d, theme %s", cfg.sink,
               sink.w, sink.h, cfg.background, strcmp(cfg.background, "wearer") ? "" : " view ",
               strcmp(cfg.background, "wearer") ? "" : cfg.view_node, cfg.mirror, prio, cfg.theme);
    if (eagle) {
        const wv_config *w = &wearer.cfg;
        seeker_log("eagleeye: wearer %s pose %.2f,%.2f,%.1f (%s), hfov %.1f, image %dx%d, "
                   "cam %.2f m, %d wall(s), no video: %s%s%s",
                   w->node, w->pose.x_m, w->pose.y_m, w->pose.yaw_deg,
                   w->pose_src == WV_POSE_STATIC ? "static" : w->pose_src == WV_POSE_FUSION
                                                                 ? "fusion" : "auto",
                   w->cam.hfov_deg, w->cam.image_w, w->cam.image_h, w->cam.cam_h_m, w->n_walls,
                   w->radar_fallback && !w->visor ? "radar" : "black",
                   w->visor ? ", visor" : "", w->inset ? "" : ", no inset");
        seeker_log("eagleeye: x-ray height: camera %.2f m above the floor, tilt %.1f deg (the phone's "
                   "pitch_deg replaces it when sent), people %.2f m tall, nudge %+.1f deg up, "
                   "learn the tilt from %s's own sightings: %s",
                   w->cam.cam_h_m, w->cam.pitch_deg,
                   w->cam.person_h_m > 0.5f ? w->cam.person_h_m : WV_PERSON_H_M, w->xray_up_deg,
                   w->node, w->pitch_auto ? "on" : "off");
        if (w->handoff)
            seeker_log("eagleeye: handoff: a person %s saw stays seen %d ms, x-ray to seen crossfades "
                       "over %d ms (plan theme)", w->node, WV_SEEN_HOLD_MS, WV_XFADE_MS);
        else
            seeker_log("eagleeye: handoff: off (--handoff off)");
    }

    /* Decode below the draw loop (IF-3: 50 fifo) but above fusion (30) and hud_stress (10), so
     * video keeps up under the CPU-load demo without ever delaying a HUD frame. */
    if (video_prio < 0)
        video_prio = (policy == SCHED_FIFO || policy == SCHED_RR) && prio > 10 ? prio - 10 : 0;
    video_rx *vrx = NULL;
    if (use_video) {
        vrx = video_rx_start(video_port, cv.w, cv.h, video_prio);
        if (!vrx)
            seeker_log("video disabled: tcp/%d unavailable; HUD runs without it", video_port);
        else if (eagle) {
            /* The projection is in the wearer camera's frame: another phone's frames must never
             * replace (or refresh) the wearer's picture. */
            video_rx_only_node(vrx, wearer.cfg.node);
            seeker_log("video: wearer view pinned to %s's video; picture kept %d ms after the "
                       "last frame (dimmed after %d ms), then %s",
                       wearer.cfg.node, VIDEO_HOLD_MS, VIDEO_STALE_MS,
                       wearer.cfg.radar_fallback && !wearer.cfg.visor ? "radar" : "black");
        }
    }

    static char buf[SEEKER_MAX_DATAGRAM + 1];
    static seeker_tracks tracks, incoming;
    int have = 0;
    int64_t t_rx = 0;
    const int64_t period = 1000 / (cfg.fps > 0 ? cfg.fps : 30);
    const int64_t t_start = seeker_now_ms();
    int64_t next = t_start, fps_t = t_start;
    long frames = 0, fps_frames = 0;
    float fps = 0;
    int eagle_key = -1;
    int64_t eagle_log_ms = 0;
    int video_drawn = -1, video_why = -1;
    int64_t video_why_ms = 0;
    const char *lat_env = getenv("SEEKER_LAT"), *lat_base = getenv("SEEKER_LAT_BASE_MS");
    const int lat_show = lat_env && !strcmp(lat_env, "1");
    static lat_meter lat;
    lat_init(&lat, lat_base ? (float)atof(lat_base) : 0.0f);
    char lat_txt[64] = "";
    uint32_t lat_seq = 0, lat_conn = 0;
    int lat_have = 0, lat_prev_n = 0;
    int64_t lat_log_ms = 0, vid_draw_ms = 0, tilt_log_ms = 0;
    float tilt_logged = 0;
    seeker_log("latency: per-frame camera->HUD timing on, log every %s, HUD readout %s "
               "(SEEKER_LAT=1), base %.0f ms (SEEKER_LAT_BASE_MS), frame sync %s",
               lat_show ? "second" : "10 s", lat_show ? "on" : "off", lat.base_ms,
               frame_sync ? "on" : "off");

    while (!g_stop) {
        int64_t now = seeker_now_ms();
        if (run_for > 0 && now - t_start >= (int64_t)(run_for * 1000))
            break;
        for (int guard = 0; guard < 64; guard++) { /* newest wins, never queue */
            int n = seeker_udp_recv(&rx, buf, sizeof buf, 0, NULL);
            if (n <= 0)
                break;
            char err[64];
            if (seeker_decode_tracks(buf, n, &incoming, err, sizeof err) == 0) {
                tracks = incoming;
                have = 1;
                t_rx = seeker_now_ms();
            }
        }
        int64_t t0 = seeker_now_ms();
        video_stats vs;
        const video_frame *vf = vrx ? video_rx_latest(vrx, &vs) : NULL;
        hud_state st = { have ? &tracks : NULL, have ? (int)(t0 - t_rx) : -1, health, fps, prio,
                         cfg.background, cfg.view_node, vf,
                         video_rx_state(vf, vrx ? &vs : NULL, t0), vrx ? vs.fps : 0,
                         eagle ? &wearer : NULL, video_rx_why(vf, vrx ? &vs : NULL, t0),
                         video_rx_frozen(vf, vrx ? &vs : NULL, t0, VIDEO_FREEZE_MS),
                         lat_show ? lat_txt : NULL };
        if (st.video_why != video_why && t0 - video_why_ms >= 1000) {
            if (st.video_why == VIDEO_WHY_OK)
                seeker_log("video: ok, %s picture on the HUD", vf->node_id);
            else
                seeker_log("video: %s, HUD says \"%s\"", video_why_name(st.video_why),
                           video_why_text(st.video_why));
            video_why = st.video_why;
            video_why_ms = t0;
        }
        int64_t ru0 = seeker_now_us();
        anim_map_frame(t0); /* the plan map animations' clock for this frame */
        int drawn = hud_render(&cv, &cfg, &st);
        float render_ms = (float)(seeker_now_us() - ru0) / 1000.0f;
        if (drawn != video_drawn) { /* the line scripts and operators wait for */
            if (drawn)
                seeker_log("video: frame drawn on the HUD: %s seq %u %dx%d (hud frame %ld)",
                           vf->node_id, vf->seq, vf->src_w, vf->src_h, frames);
            else if (video_drawn > 0)
                seeker_log("video: picture no longer drawn (video %s, hud frame %ld)",
                           video_state_name(st.video_state), frames);
            video_drawn = drawn;
        }
        hud_frame_note(sink.ppm_note, sizeof sink.ppm_note, frames, st.video_state, drawn, vf);
        if (eagle) { /* one line when the seen / x-ray / off-screen counts change, max 4 per s */
            const wv_stats *ws = &wearer.last;
            int key = ws->n_direct * 10000 + ws->n_xray * 100 + ws->n_off + ((int)ws->pose_mode << 20) +
                      ((ws->video + 1) << 22) + (ws->rendered << 24) + (ws->n_skel << 25);
            key = key * 31 + ws->n_posture[1] * 289 + ws->n_posture[2] * 17 + ws->n_posture[3];
            if (key != eagle_key && t0 - eagle_log_ms >= 250) {
                log_eagle(&wearer);
                eagle_key = key;
                eagle_log_ms = t0;
            }
        }
        if (sink.present(&sink, &cv) != 0 && frames % 300 == 0)
            seeker_log("present failed");
        if (eagle && wearer.cfg.pitch_auto && fabsf(wearer.tilt_fix - tilt_logged) >= 1.0f &&
            t0 - tilt_log_ms >= 2000) {
            seeker_log("eagleeye: x-ray height learned from %s's own sightings: tilt %+.1f deg "
                       "(%d sightings), x-ray rows now use %.1f deg%s", wearer.cfg.node,
                       wearer.tilt_fix, wearer.tilt_n, wearer.last.pitch_deg,
                       wearer.last.pitch_src ? " incl. the phone's pitch" : "");
            tilt_logged = wearer.tilt_fix;
            tilt_log_ms = t0;
        }
        int64_t t_post = seeker_now_ms();
        if (drawn && vf && vf->px && (!lat_have || vf->seq != lat_seq || vf->conn != lat_conn)) {
            float copy_ms = eagle && wearer.last.rendered ? wearer.last.video_ms : 0.0f;
            lat_sample ls = { vf->ts_ms,     vf->age_ms,         vf->t_arrive_ms,
                              vf->t_rx_ms,   t_post,             vf->dec_ms,
                              vf->scale_ms,  copy_ms,            render_ms - copy_ms,
                              sink.blit_ms,  sink.post_ms };
            lat_frame(&lat, vf->seq, &ls);
            vid_draw_ms = t_post;
            lat_seq = vf->seq;
            lat_conn = vf->conn;
            lat_have = 1;
        }
        lat_summary lsum;
        if (lat_tick(&lat, t_post, &lsum)) {
            lat_text(&lat, lat_txt, sizeof lat_txt);
            if (draw_theme() == DRAW_THEME_PLAN) { /* the title block's Delay cell: a value only */
                if (lat.have_last && lat.last.n > 0)
                    snprintf(lat_txt, sizeof lat_txt, "%.0f ms", lat.last.total_p50);
                else
                    snprintf(lat_txt, sizeof lat_txt, "no frames");
            }
            if (lsum.n > 0 ? lat_show || t_post - lat_log_ms >= 10000 : lat_prev_n > 0) {
                char line[512];
                lsum.rx_fps = vrx ? vs.rx_fps : 0.0f;
                lat_log_line(&lsum, lat.base_ms, line, sizeof line);
                seeker_log("%s", line);
                lat_log_ms = t_post;
            }
            lat_prev_n = lsum.n;
        }
        frames++;
        fps_frames++;
        seeker_health_beat(health, slot, (int)(seeker_now_ms() - t0), SH_OK);
        if (now - fps_t >= 1000) {
            fps = (float)fps_frames * 1000.0f / (float)(now - fps_t);
            fps_frames = 0;
            fps_t = now;
            if ((frames / cfg.fps) % 5 == 0) {
                seeker_log("%.1f fps, draw+present %lld ms, tracks %s", fps,
                           (long long)(seeker_now_ms() - t0), have ? "yes" : "none yet");
                if (eagle)
                    log_eagle(&wearer);
            }
        }
        next += period;
        /* Video flowing: frames drive the loop and the tick only fills a gap of 1.5 periods,
         * so a 24 fps phone gives 24 draws a second, not 24 plus 30. */
        if (frame_sync && vrx && vid_draw_ms && t_post - vid_draw_ms < 500 &&
            next < vid_draw_ms + period * 3 / 2)
            next = vid_draw_ms + period * 3 / 2;
        int64_t wait = next - seeker_now_ms();
        if (wait > 0 && vrx && frame_sync) {
            if (video_rx_wait(vrx, (int)wait))
                next = seeker_now_ms(); /* draw the new frame now; the tick restarts from it */
        } else if (wait > 0) {
            seeker_sleep_ms(wait);
        } else {
            next = seeker_now_ms();
        }
    }
    seeker_log("exit after %ld frames", frames);
    video_rx_stop(vrx);
    sink.close(&sink);
    free(cv.px);
    return 0;
}
