# Seeker Devpost outline

Working name: **Seeker**. Tagline: "A fighter-style HUD helmet on QNX that marks people you can't
see." Hashtags and categories: Best Use of QNX, PCBWay Best Hardware Prototype, [MLH] Best Use of
Gemini API (only if the after-action report shipped).

## Inspiration
Firefighters and search-and-rescue teams lose track of people in smoke and rubble. Fighter pilots
get helmet displays (F-35) and soldiers are getting them (Anduril EagleEye). Felix wanted his own,
built on a real-time OS.

## What it does
- See-through HUD (Pepper's ghost visor) with target brackets, range, a heading tape and contact colours.
- On-device person and hand detection on a Raspberry Pi 5 running QNX.
- Phone cameras elsewhere in the room stream to a laptop relay that detects people and sends only detections to
  the helmet, so the HUD marks people behind walls (cyan).
- Never freezes: separate processes, the HUD at top priority, live restart of the AI.

## How we built it
- QNX 8.0 on a Pi 5 (8 GB); remote phone cameras (IP camera app) relayed by a Mac running Python + OpenCV (YOLOv8n or MediaPipe); a Logitech C270 USB webcam through the QNX Sensor Framework (CamAPI, unit 5).
- Processes: `hud_capture`, `hud_infer`, `hud_fusion`, `hud_display`, `hud_supervisor`; frames in
  POSIX shared memory, messages over UDP.
- TensorFlow Lite from qnx-ports (MediaPipe hand models, a COCO person detector) [ONNX Runtime from qnx-ports].
- Range: Z = f * H / h from person pixel height [stereo Z = f * B / d].
- Visor: 3D-printed brim, small HDMI display, acrylic combiner at 45 degrees.
- [AirPods head tracking via an iPhone app for world-locked markers.]
- Built by Felix and Daniel with many parallel coding agents working from interface contracts,
  both SSH'd into one Pi.

## Real-time results
[Latency chart: camera-to-display p50 / p99 with all cores stressed, HUD at priority 50 vs 10.]
[Restart time of `hud_infer` after a kill.] [Inference ms per model on the Pi.]

## Challenges
[Camera grabber on CamAPI. Pepper's ghost focus and brightness. Keeping the HUD decoupled from inference.]

## Accomplishments
[Fill from the gates that passed.]

## What we learned
[Priorities in practice, shared memory, optics.]

## What's next
Thermal camera for smoke, a custom helmet PCB (PCBWay), helmet-to-helmet mesh, wearer position
tracking for full world-locked markers.

## Media
Backup video (one take through the visor), latency chart, `pidin` screenshot, architecture
diagram, photo of the visor, screenshot of the unmodified QNX AI workshop on the loaner Pi.

## Built with
qnx, raspberry-pi, c, qnx-screen, tensorflow-lite, mediapipe, opencv, python, openscad, [swift, onnxruntime, gemini]
