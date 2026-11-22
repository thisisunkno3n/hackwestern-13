# Seeker HUD architecture

Contracts (names, fields, ports, priorities) are in [INTERFACES.md](../INTERFACES.md); this file
explains how the pieces fit and why.

## Hardware

```
   helmet (hard hat or bike helmet)
   +-------------------------------------------------+
   |  Logitech C270 USB webcam (boresight, CamAPI unit 5)  [+ 2nd USB webcam for stereo, 6 to 10 cm]
   |  small HDMI screen, face down, in a 3D-printed brim (visor light source; Felix's monitor on HDMI0 for dev)
   |  acrylic sheet at 45 degrees in front of one eye (Pepper's ghost combiner)
   +-------------------------------------------------+
          | USB + micro-HDMI
   helmet Pi: Pi 5 8 GB on QNX 8.0 (qnxpi-felix.local), belt or back of helmet, USB-C power bank
          | Wi-Fi (phone hotspot `QNX`)
   laptop: MJPEG view of the HUD for the audience, latency chart
   remote cameras: phones with an IP camera app (R1 [+ R2]) -> Mac relay (OpenCV + detection) -> JSON to the Pi
   spare: Daniel's Pi 5 on QNX 8.0 (qnxpi.local), only if the helmet Pi dies
   optional: iPhone + AirPods (head motion) sending UDP to the Pi
```

## Processes (on the Pi)

```
 CamAPI --> hud_capture (40) --frames--> /seeker_frames (shm ring, newest wins)
                                             |                    |
                                             v                    v
                                  hud_infer (11)            hud_display (50, highest)
                                      | detections UDP          ^   draws newest frame
                                      v                         |   + newest tracks
 remote node --UDP--> hud_fusion (30) --tracks UDP (30 Hz)------+   presents (QNX Screen)
 iPhone head pose --UDP-->  ^                                   |   MJPEG to laptop (9)
                                                                v
 hud_supervisor (45): spawns, sets priorities, heartbeats in /seeker_health, restarts
 hud_stress (10): demo load only
```

The key decision is **decoupling**. `hud_display` wakes on every new camera frame and draws with
whatever tracks are newest; it never waits for inference. Inference runs at the lowest priority
and can slow down or die without touching the camera-to-display path. Fusion coasts tracks at 30 Hz
(constant-velocity prediction in image space) so brackets stay glued between detections.

## Latency budget (Pi 5, 1280x720, targets to measure)

| Stage | Target |
| --- | --- |
| Sensor to CamAPI callback | Measure (rolling shutter + ISP); report it rather than guess |
| Callback to ring publish (copy, NV12 to BGR) | 5 ms or less |
| Ring to `hud_display` wake | 1 ms or less |
| Draw symbology (OpenCV on the frame, or on black in visor mode) | 5 ms or less |
| Present (QNX Screen) | Measure |
| **Capture to present** | **Under 40 ms, and flat (p99 within 10 ms of p50) under `hud_stress`** |
| Detection age (capture of the inferred frame to present) | Under 150 ms; shown live as `DET` |

The demo chart plots `cap_to_present_ms` for two runs under the same `hud_stress` load: HUD at
priority 50 (flat) and HUD at 10 (spiky). Same code, different priority: that's the "real need for
QNX" slide.

## Detection

Two TFLite models on the CPU with XNNPACK, through the qnx-ports TensorFlow Lite runtime (verified
by the QNX AI workshop on a QNX Pi 5): MediaPipe `palm_detection_lite` plus `hand_landmark_lite` for
hands (workshop code), and a small person detector (EfficientDet-Lite0 or SSD MobileNet v2 COCO,
person class only). ONNX Runtime from qnx-ports with a 320 px YOLO-nano is the alternative backend
if TFLite person detection is too slow or too weak.

## Range

- **Focal length f (C270):** the camera is a Logitech C270 (fixed focus, about 55 degrees
  horizontal FOV, an estimate). At 1280x720, f = 640 / tan(27.5 degrees), about 1230 px, so the
  starting value is **f = 1200 px**. Calibrate it once on the Pi (WS3 step): a checkerboard with
  OpenCV `calibrateCamera`, or a known-size object of real height H at a taped distance Z, giving
  f = h_px * Z / H. Recalibrate if the capture size changes (640x480 may be a different crop).
  Both range formulas below use this f.
- **Monocular (default):** Z = f * H_real / h_pixels, with H = 1.70 m for a standing person and h_px the
  box height. Confidence drops when the box touches the image edge or the person isn't upright
  (aspect ratio check). Good to about 10 to 20 percent indoors, which is fine for colour bands.
- **Stereo (if a second webcam turns up):** a second USB webcam (ideally another C270, or recalibrate
  each), 6 to 10 cm baseline, rigidly mounted. Two USB cameras at once on CamAPI and their lack of
  frame sync are unverified risks. Detect on the left image only, then search the box region along the
  same rows in the right image (normalized cross-correlation) for disparity d, and Z = f * B / d (f in px, B in m, d in px).
  Cheap, because there's no full depth map. Fusion prefers stereo when the match score is good.
- Example at f = 1200 px: a 1.70 m person 240 px tall is 8.5 m away; 680 px tall is 3.0 m.

## Remote node (the twist)

**One QNX Pi, one remote eye.** All the build runs on the helmet Pi (`qnxpi-felix`): camera,
detection, HUD. The remote cameras are **phones running an IP camera app** elsewhere in the room, streaming over
the hotspot. A Mac relay (not a camera) reads each stream with a Python + OpenCV script (YOLOv8n or
MediaPipe person detection; N nodes, one `node_id` per phone, each drawn with its own label) that sends **only detections** (box, label,
bearing) as JSON over the hotspot to the Pi, which draws them as remote markers: the "see behind a
wall" demo. If a relay or phone stops sending, the HUD shows `NODE R1 LOST` and never stalls. Daniel's Pi
(`qnxpi.local`) is a spare only. The node knows its own pose in the room frame (taped
position and yaw, IF-1) and sends each person's bearing and range. `hud_fusion` converts each
contact to room coordinates, then to the wearer's frame using the wearer's yaw (head pose if
available, else the calibration direction). Contacts not visible to the wearer are drawn **cyan**:
a diamond on the horizon if within the HUD's field of view (behind a wall), an edge chevron if
outside it, and always on the radar inset. Honest scope: the wearer stands on the taped mark; there
is no wearer position tracking.

## Head tracking (optional, WS8)

An iPhone app reads AirPods head motion (Core Motion headphone motion) and the phone's compass
heading, and sends `head_pose` over UDP at 20 Hz or more. With it, remote markers and the heading
tape are world-locked as the wearer turns. Without it, the tape is relative (`REL`) and the radar
inset still works.

## Optics (Pepper's ghost)

A bright small HDMI display faces down from the brim; a clear acrylic sheet at 45 degrees reflects
it into one eye while the real world shows through. Black pixels are transparent, so visor mode
draws on black. The reflection flips the image, so `display.mirror` corrects it. Two known problems:
the virtual image is only a few centimetres away (hard to focus), and acrylic gives a faint double
image. Mitigations: lengthen the display-to-combiner path, add an optional Fresnel or reading-glass
lens, use thick lines and large text, and dim the room for the demo. Fallback: video mode on a
monitor or the laptop MJPEG view.

## AI split

All perception and every decision run on the device: TFLite detection, plain-C fusion, range and
colour rules. No cloud call is on the HUD path, and there is no LLM on the Pi. Optional (Gemini
prize, off the real-time path): `tools/after_action.py` sends the event and track log to Gemini
after a run and writes a plain-language after-action report.

## Portability and the Cubie fallback

Everything except the `qnx:` capture source is POSIX (IF-1 Portability). Laptop on Friday night:
`webcam:0` or `video:` clips, `imshow` sink, sims for remote and head pose. **If no QNX Pi 5 is
available (two are in hand, so this is only if both die),** the full stack runs on the **Radxa Cubie (A7Z) borrowed from Western
Aero** on Linux with the same C270 as a UVC webcam. As far as we
know QNX has no BSP for it, so that doesn't count for Best Use of QNX.
With a spare QNX Pi, the Cubie is only
needed if both die.
