# Seeker HUD parts list

**Nothing is being bought.** Everything is brought, borrowed from Western Aero, or sourced at the
event. **Felix has had the QNX Pi 5 since about 10:55pm Fri.** The hardware desk is open Sat
10:00am to 10:30pm and Sun 8:30am to 2:00pm (ask there for a C920 and a second webcam).

## Compute and cameras

| Item | Source | Notes | Fallback |
| --- | --- | --- | --- |
| Raspberry Pi 5 **8 GB** with QNX 8.0, **helmet** | **In hand** (Felix, Fri 10:55pm), `qnxpi-felix.local` | Check the RAM (`pidin info`) | Daniel's Pi |
| Raspberry Pi 5 **8 GB** with QNX 8.0, **spare** | **In hand** (Daniel, Fri 11:21pm), `qnxpi.local` | Spare only; swap in if the build Pi dies | Cubie |
| **Radxa Cubie (A7Z)** | Borrow from Western Aero | Linux only (as far as we know no QNX BSP), so it doesn't count for Best Use of QNX. Only needed if both QNX Pis die; then the main board, with the C270 as a UVC webcam | Laptop as the remote node |
| **Logitech C270 USB webcam** (wearer camera) | In hand | UVC, 720p 30 fps, fixed focus, about 55 degrees HFOV (estimate), f about 1200 px at 1280x720 before calibration. CamAPI USB unit 5 (default entry, or the MJPEG entry in `sensor_rpi5.conf`). **Not on QNX's tested list** (the C920 is) | **C920** from the hardware desk |
| Second USB webcam (stereo, optional) | Ask the QNX table / desk | Rigid 6 to 10 cm baseline; two USB cameras at once on CamAPI is unverified | Monocular range |
| Remote node cameras | Phones with an IP camera app (`R1`, `R2` optional) | Stream over the hotspot to a Mac relay (OpenCV, YOLOv8n or MediaPipe), JSON to the Pi (WS6) | A laptop webcam as the stream |
| Phone hotspot | Felix's phone | SSID `QNX`, password `everywhere` (kit default Wi-Fi); only one hotspot with that name | Ethernet from a laptop |
| Ethernet cable + USB Ethernet adapter | Bring | SSH and the MJPEG view, independent of venue Wi-Fi | Venue Wi-Fi |
| USB-C power bank, PD 5 V 3 A or more (27 W PSU at the desk) | Bring | Wearable power; under-powered Pis brown out | Tethered PSU for the demo |

## Visor (HUD optics)

| Item | Source | Notes | Fallback |
| --- | --- | --- | --- |
| **Small bright HDMI display** (3.5 to 5 inch, 800x480 or better) | Ask the QNX table / desk / Western Aero | HDMI needs no driver; avoid SPI displays (need drivers on QNX). Brightness matters more than resolution | HUD on a monitor or the laptop MJPEG view |
| micro-HDMI to HDMI cable (short or flat) | Bring | Pi 5 is micro-HDMI | |
| **Clear acrylic sheet** 2 to 3 mm, about 10 x 8 cm | Desk / makerspace scrap / a picture-frame cover | Pepper's ghost combiner at 45 degrees. Teleprompter beam-splitter film is better if anyone has it | Picture-frame glass (careful with edges) |
| **3D printer access** + PLA | Ask the desk Friday night | Brim mount, combiner slot, display shroud (WS7, each part under 3 h) | Cardboard + hot glue template (WS7) |
| Hard hat or bike helmet | Bring / borrow | Firefighter / SAR pitch | Headband + strap |
| Fresnel lens or reading-glass lens (+2 to +3) | Optional | Pushes the virtual image further out so it's easier to focus | Longer display-to-combiner path |
| Velcro, zip ties, hot glue, black tape | Desk | Light shroud and cable management | |

## Optional head tracking

| Item | Source | Notes |
| --- | --- | --- |
| iPhone + AirPods that support head tracking (AirPods Pro, Max, or 3rd gen and later) | Felix | WS8 |
| Mac with Xcode | Felix | Needed to install the HeadBridge app on the iPhone; WS8 is cut without it |

## Demo

Laptop (QNX SDP installed), a monitor for the audience view if available, phone + small tripod for
the backup video (including a shot through the combiner), tape for the floor mark and the node pose.
