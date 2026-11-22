cmake_minimum_required(VERSION 3.13)
project(seeker_pi_lane C)

# SEEKER_TARGET (IF-1): laptop | linux | qnx. qnx links QNX Screen and libsocket.
# Defaults to qnx when CMake runs on QNX (or with the QNX toolchain file), so a plain
# `cmake ..` on the Pi needs no flags.
if(CMAKE_SYSTEM_NAME STREQUAL "QNX")
  set(_seeker_default_target "qnx")
else()
  set(_seeker_default_target "laptop")
endif()
set(SEEKER_TARGET "${_seeker_default_target}" CACHE STRING "laptop, linux or qnx")
# Host only: build hud_display's screen sink against host-stub/ (NOT QNX) to compile-check it.
option(SEEKER_SCREEN_STUB "Build the screen sink against the host stub (not QNX)" OFF)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()
add_compile_options(-Wall -Wextra)
# POSIX 2008 everywhere; QNX also needs _QNX_SOURCE for some libc extensions to stay visible
# under a strict -std=c11:
#   https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.prog/topic/devel_ConformingToStandards.html
add_compile_definitions(_POSIX_C_SOURCE=200809L)
if(SEEKER_TARGET STREQUAL "qnx")
  add_compile_definitions(_QNX_SOURCE)
  # Native QNX gcc predefines __QNX__ (800 on SDP 8.0). Do not define it here.
  # https://www.qnx.com/developers/docs/8.0/com.qnx.doc.neutrino.lib_ref/topic/manifests.html
endif()

set(SYS_LIBS m)
if(SEEKER_TARGET STREQUAL "qnx")
  # BSD sockets live in libsocket on QNX 8.0 (io-sock). Link the shared library
  # libsocket.so; there is no io-sock libsocket.a.
  # https://www.qnx.com/developers/docs/8.0/com.qnx.doc.qnxsdp.migration/topic/migrate_app.html
  list(APPEND SYS_LIBS socket)
else()
  find_package(Threads REQUIRED)
  list(APPEND SYS_LIBS Threads::Threads)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    list(APPEND SYS_LIBS rt)     # shm_open on older glibc
  endif()
endif()

add_library(seeker_common STATIC
  common/seeker_time.c common/seeker_log.c common/seeker_json.c common/seeker_msgs.c
  common/seeker_config.c common/seeker_udp.c common/seeker_health.c common/seeker_colour.c
  common/seeker_posture.c)
target_include_directories(seeker_common PUBLIC common)
target_link_libraries(seeker_common PUBLIC ${SYS_LIBS})

add_library(seeker_fusion_core STATIC
  fusion/seeker_fusion.c fusion/seeker_pose.c fusion/seeker_triangulate.c fusion/seeker_assign.c
  fusion/seeker_skel.c)
target_include_directories(seeker_fusion_core PUBLIC fusion)
target_link_libraries(seeker_fusion_core PUBLIC seeker_common)

add_executable(hud_fusion fusion/hud_fusion.c)
target_link_libraries(hud_fusion seeker_fusion_core)

add_executable(udp_recv tools/udp_recv.c)
target_link_libraries(udp_recv seeker_common)

add_executable(hud_supervisor supervisor/hud_supervisor.c)
target_link_libraries(hud_supervisor seeker_common)

# IF-5.6 video on TCP 47106: SKV2 framing, JPEG decode (vendored third_party/stb, no system
# libraries) and the receiver thread.
add_library(seeker_video STATIC display/video_proto.c display/video_decode.c display/video_rx.c
  display/latency.c)
target_include_directories(seeker_video PUBLIC display)
target_link_libraries(seeker_video PUBLIC seeker_common)

add_library(seeker_sv STATIC supervisor/seeker_sv.c)
target_include_directories(seeker_sv PUBLIC supervisor)
add_executable(seeker_supervisor supervisor/seeker_supervisor.c)
target_link_libraries(seeker_supervisor seeker_sv seeker_fusion_core) # seeker_pose for --pose

# Plan theme: drafting primitives, Atkinson Hyperlegible compiled in (third_party/atkinson, made by
# tools/gen_hud_fonts.py) and drawn with stb_truetype.
add_library(seeker_draw STATIC display/draw.c display/draw_font.c display/plan.c display/anim_alert.c display/anim_person.c display/anim_map.c
  third_party/atkinson/hud_fonts.c)
target_include_directories(seeker_draw PUBLIC display fusion)
target_link_libraries(seeker_draw PUBLIC seeker_common)

set(HUD_SRC display/hud_display.c display/hud_render.c display/sink.c
  display/projection.c display/wearer_view.c fusion/seeker_pose.c)
if(SEEKER_TARGET STREQUAL "qnx")
  add_executable(hud_display ${HUD_SRC} display/sink_screen.c)
  # Prefer the official SDP <screen/screen.h>. This Pi image ships libscreen
  # but not the header. Use third_party/qnx-screen only when the real header
  # is absent. Never put host-stub/ on a qnx include path.
  set(_seeker_screen_h "")
  if(DEFINED ENV{QNX_TARGET} AND EXISTS "$ENV{QNX_TARGET}/usr/include/screen/screen.h")
    set(_seeker_screen_h "$ENV{QNX_TARGET}/usr/include/screen/screen.h")
  elseif(EXISTS "/usr/include/screen/screen.h")
    set(_seeker_screen_h "/usr/include/screen/screen.h")
  elseif(EXISTS "/system/include/screen/screen.h")
    set(_seeker_screen_h "/system/include/screen/screen.h")
  endif()
  if(_seeker_screen_h STREQUAL "")
    message(WARNING "No official <screen/screen.h> on this sysroot; using third_party/qnx-screen (constants cited from QNX 8.0 docs). A successful compile is not a QNX test pass; nothing counts as tested until it runs on the QNX Pi.")
    target_include_directories(hud_display PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/third_party/qnx-screen)
  else()
    message(STATUS "Using official Screen header: ${_seeker_screen_h}")
  endif()
  target_link_libraries(hud_display seeker_draw seeker_video seeker_common screen)
elseif(SEEKER_SCREEN_STUB)
  add_executable(hud_display ${HUD_SRC} display/sink_screen.c host-stub/screen_stub.c)
  target_include_directories(hud_display PRIVATE host-stub)
  target_link_libraries(hud_display seeker_draw seeker_video seeker_common)
else()
  add_executable(hud_display ${HUD_SRC} display/sink_noscreen.c)
  target_link_libraries(hud_display seeker_draw seeker_video seeker_common)
endif()
target_include_directories(hud_display PRIVATE display fusion)

enable_testing()
add_executable(test_core tests/test_core.c display/projection.c display/wearer_view.c
  display/sink.c display/sink_noscreen.c display/hud_render.c)
target_include_directories(test_core PRIVATE display)
target_link_libraries(test_core seeker_fusion_core seeker_video seeker_draw)

# Host-only: renders wearer-view frames of a made-up scene to PNG for both themes
# (tools/hud_shot.c). Not built for QNX.
if(NOT SEEKER_TARGET STREQUAL "qnx")
  add_executable(hud_shot tools/hud_shot.c display/projection.c display/wearer_view.c
    display/hud_render.c fusion/seeker_pose.c)
  target_include_directories(hud_shot PRIVATE display fusion)
  target_link_libraries(hud_shot seeker_draw seeker_video seeker_common)
  # Plan map animation sequences and timing (tools/hud_anim_map_shot.c, tools/anim_map_media.sh).
  add_executable(hud_anim_map_shot tools/hud_anim_map_shot.c display/projection.c display/wearer_view.c
    display/hud_render.c fusion/seeker_pose.c)
  target_include_directories(hud_anim_map_shot PRIVATE display fusion)
  target_link_libraries(hud_anim_map_shot seeker_draw seeker_video seeker_common)
endif()
target_compile_definitions(test_core PRIVATE SEEKER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_test(NAME core COMMAND test_core WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
# Smoothness simulation (tests/sim_smooth.c): synthetic phones through fusion, the tracks
# datagram and the wearer view on a simulated clock; before/after metrics.
add_executable(sim_smooth tests/sim_smooth.c display/projection.c display/wearer_view.c
  display/sink.c display/sink_noscreen.c display/hud_render.c)
target_include_directories(sim_smooth PRIVATE display)
target_link_libraries(sim_smooth seeker_fusion_core seeker_video seeker_draw)
target_compile_definitions(sim_smooth PRIVATE SEEKER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_test(NAME smoothness COMMAND sim_smooth WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
# Handoff replay (tests/test_handoff.c): x-ray person R1 then sees (step-out, hand-out), before
# and after the handoff changes.
add_executable(test_handoff tests/test_handoff.c display/projection.c display/wearer_view.c)
target_include_directories(test_handoff PRIVATE display)
target_link_libraries(test_handoff seeker_fusion_core seeker_video seeker_draw)
target_compile_definitions(test_handoff PRIVATE SEEKER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
add_test(NAME handoff COMMAND test_handoff WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
add_executable(test_supervisor tests/test_supervisor.c)
target_link_libraries(test_supervisor seeker_sv)
add_test(NAME supervisor_unit COMMAND test_supervisor)
# Spawns real children and binds UDP 47103/47102: stop any running demo before `ctest`
# (sh scripts/stop-live.sh). On QNX a held port makes the test's hud_fusion crash-loop.
add_test(NAME supervisor_restart
         COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_supervisor.sh $<TARGET_FILE_DIR:seeker_supervisor>
         WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
# IF-5.6 sender -> hud_display end to end (needs python3). Own ports and log folders, so they run
# beside a demo on 47106 and in parallel with each other. video_loopback_slow is the Pi timing
# that broke the old sleep-based snapshots: a late first frame, a sender that lingers after
# closing, and a slower PPM cadence.
foreach(_vt video_loopback video_loopback_slow)
  add_test(NAME ${_vt} COMMAND sh ${CMAKE_CURRENT_SOURCE_DIR}/scripts/video_loopback_test.sh
           WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
endforeach()
# scripts/seeker_bridge.py (wraps Daniel's ws_ingest.py for live video): framing, video link
# reconnect, video_ctrl. Needs python3 only; skipped where there is none.
find_program(SEEKER_PYTHON3 python3)
if(SEEKER_PYTHON3)
  add_test(NAME seeker_bridge
           COMMAND ${SEEKER_PYTHON3} ${CMAKE_CURRENT_SOURCE_DIR}/tests/test_seeker_bridge.py)
endif()
set_tests_properties(video_loopback PROPERTIES ENVIRONMENT
  "BUILD=$<TARGET_FILE_DIR:hud_display>;VIDEO_PORT=47116;VIDEO_LOG=logs/video-ctest")
set_tests_properties(video_loopback_slow PROPERTIES ENVIRONMENT
  "BUILD=$<TARGET_FILE_DIR:hud_display>;VIDEO_PORT=47126;VIDEO_LOG=logs/video-slow;FIRST_FRAME_DELAY=1;SENDER_EXIT_DELAY=0.5;PPM_EVERY=10")
