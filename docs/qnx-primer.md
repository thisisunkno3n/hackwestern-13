# QNX primer for Seeker (first time on QNX)

For Felix: about 10 minutes of reading. QNX is a POSIX real-time OS with a microkernel, so your C
and C++ (pthreads, sockets, `open`/`read`/`ioctl`-style calls, OpenCV) mostly just works. The
differences are in how drivers and builds work. Commands here are illustrative setup steps, not
project code.

## Mental model in 6 points

1. **Microkernel.** Drivers (camera, I2C, serial, USB, network) are ordinary user-space processes
   called *resource managers* that own paths like `/dev/i2c1` or `/dev/ser2`. If a driver crashes,
   the kernel and your app survive. That's the reliability story behind the Seeker HUD, where
   capture, inference, fusion and the HUD are separate processes.
2. **POSIX.** pthreads, `clock_gettime(CLOCK_MONOTONIC)`, BSD sockets and `select`/`poll` all
   behave as on Linux. Device control uses `devctl()` (QNX's typed version of ioctl), e.g.
   `DCMD_I2C_SENDRECV` for I2C.
3. **Priorities are real.** Threads have priorities (higher wins) and policies (FIFO, round robin).
   A high-priority thread preempts lower ones immediately. The Seeker HUD process sits at the
   top, inference at the bottom (IF-3). `pidin` shows every thread's priority and state; use it
   in the demo.
4. **Message passing** (channels, `MsgSend`/`MsgReceive`) is QNX's native IPC. Seeker shares frames
   through POSIX shared memory and small messages over UDP on localhost for portability, with QNX
   pulses as a stretch (IF-4, IF-5).
5. **Tools you'll type:** `pidin` (processes and threads), `slay <name>` (kill by name),
   `sloginfo` (system log), `hogs` (CPU use per process), `use <binary>` (usage help), `on -p <prio> <cmd>` (run at a priority),
   `ls /dev`. Package manager on the QSTI image: `apk`.
6. **The Pi image** is the QNX 8.0 Quick Start Target Image (QSTI), loaned preloaded. It includes
   the QNX Developer Desktop, so it can also build code on the Pi itself.

## Before 9pm (allowed prep, not project code)

1. Create a myQNX account and get the **free non-commercial QNX SDP 8.0 license**:
   https://www.qnx.com/getqnx
2. Install the **QNX Software Center**, then install **QNX SDP 8.0** on the laptop (Linux or
   Windows host; it's a multi-GB download, so do it on good Wi-Fi before the event). Optional: the
   QNX Toolkit for VS Code (examples at https://github.com/qnx/QNX-Toolkit-for-VS-Code-Examples).
3. Check the install: source the environment script (`source ~/qnx800/qnxsdp-env.sh`), then run
   `qcc -V` and check that it lists `gcc_ntoaarch64le`.
4. Clone public repos (allowed, they're open source): `git clone https://github.com/qnx/QNX-AI-workshop`,
   `git clone https://github.com/qnx/Raspberry-Pi-Hardware-Component-Samples`,
   `git clone https://github.com/qnx-ports/build-files`. Read the workshop README and slides PDF.
5. Skim https://elahav.gitlab.io/qnx-rpi-book/ for processes, priorities and message passing.

## Building for the Pi (aarch64)

Target triple: **aarch64le** (QNX calls it `ntoaarch64le`). Two ways, both fine; use both.

| Path | When | How |
| --- | --- | --- |
| **Cross-compile on the laptop** | Dependency-free C: fusion, supervisor, unit tests. Also the camera, HUD and inference processes once a sysroot exists. | After sourcing `qnxsdp-env.sh`: `qcc -Vgcc_ntoaarch64le` (C) or `q++ -Vgcc_ntoaarch64le` (C++). With CMake, use a QNX toolchain file like `qnx.nto.toolchain.cmake` from qnx-ports/build-files (it sets `CMAKE_SYSTEM_NAME QNX` and the compilers). |
| **Build on the Pi** | `hud_capture`, `hud_display` (camapi, Screen) and `hud_infer` (TFLite C API) when there's no laptop sysroot | On the Pi: install deps with `apk` (the workshop's `configure.sh` lists the TFLite and sensor packages: `tflite-runtime-dev tflite-runtime qnx-sf-base-dev`), then build with CMake there. |

Cross-compiling the camera, HUD and inference processes needs camapi, Screen and TFLite headers and libraries for aarch64 on the
laptop. Either copy them from the Pi after `apk add` into a local sysroot folder (fastest), or build
them from qnx-ports (slow; TFLite recommends 32 GB RAM for a parallel build). Decide on Saturday
morning based on how fast the Pi builds.

## The QNX AI workshop as reference (not a fork)

The Pi code is **C on the `camera_example3_viewfinder` example source, drawing with QNX Screen**
(Felix, 12:37am Sat). https://github.com/qnx/QNX-AI-workshop is a **reference only**, not forked.
What it shows:
- `rpi_cam.cpp`: camera through the QNX Sensor Framework (`camera_open`, viewfinder callback, NV12).
  Pi 5 units: CAM/DISP0 = 3, CAM/DISP1 = 4, USB = 5. Seeker uses a Logitech C270 on **unit 5**
  (the default USB camera in the image's `sensor_rpi5.conf`, which also has an MJPEG entry).
- `handtracking.cpp`: TFLite interpreter setup with XNNPACK and threads, running MediaPipe's
  `palm_detection_lite` and `hand_landmark_lite`. A reference for `hud_infer` (written in C with the TFLite C API).
- `configure.sh`: the exact `apk` packages. `CMakeLists.txt`: a working link line (tensorflow-lite,
  camapi, plus a flatbuffers version shim).
- Its ROS 2 "visitors" feature already shows detections from other nodes on the network. That's
  the reference for the remote-node twist, but our remote contacts go over plain UDP (IF-5.4) so
  the laptop and the Linux Cubie need no ROS 2.

Running TFLite (the qnx-ports TensorFlow Lite runtime) on QNX OS on the Pi is what satisfies "uses
at least one open-source AI module from QNX" for Best Use of QNX. Screenshot the unmodified workshop
running on the loaner Pi for the Devpost.

## Getting onto the Pi

1. Power: the official 27 W USB-C supply (5 V, 5 A). Weak supplies cause random resets.
2. Network: the kit's default Wi-Fi is a hotspot with SSID `QNX` and password `everywhere` (public
   kit defaults), so a phone hotspot with that name works headless. Ethernet from a laptop is the
   fallback. Hosts: `qnxpi-felix.local` (helmet) and `qnxpi.local` (spare only); see
   [tasks/ws0-bootstrap.md](workstreams/ws0-bootstrap.md#headless-pi-setup).
3. Log in: `ssh qnxuser@<host>` with password `qnxuser` (kit default; QSTI docs:
   https://gitlab.com/qnx/quick-start-images/raspberry-pi-qnx-8.0-quick-start-image).
4. Deploy: `scp build-qnx/hud_* config/seeker.json qnxuser@<host>:~/hackwestern-13/`. Seeker's
   `scripts/qnx/deploy.sh` (WS1) wraps this.
5. Run with root if setting priorities above the user limit needs it (`su` or `sudo` per the
   image), then check `pidin` for priorities.

## First-hour checklist on the board (Fri about 11pm, Pi in hand)

`uname -a` (version), check RAM (`pidin info`, expect 8 GB), run `camera_example3_viewfinder -u 5`
(the MJPEG entry if the default fails), run the unmodified workshop with `camera_unit:=5`, check which HDMI port the display uses,
and `on -p 50 sleep 1 & pidin` to see that priorities can be set. Answers go into the hack repo's `docs/bringup-notes.md`.
