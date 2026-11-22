# QNX setup notes (headless Pi, hostnames, VNC, gotchas)

Everything verified on the hackathon QNX 8.0 Raspberry Pi 5 image, Fri Nov 20 to Sat Nov 21 (ET).
For the background on QNX itself (SDP, cross-compiling, the AI workshop as reference) see
[qnx-primer.md](qnx-primer.md).

## Machines

| Machine | Host | Role |
| --- | --- | --- |
| Helmet Pi (Felix), QNX 8.0 Pi 5 8 GB | `qnxpi-felix.local` | The **only** build and demo Pi: helmet camera, detection, HUD |
| Spare Pi (Daniel) | `qnxpi.local` (kit default) | Unused unless the helmet Pi dies |
| Phones `R1` (+ `R2` optional) | on the hotspot | IP camera app streaming video |
| Mac relays | on the hotspot | `remote/hud_remote.py --stream <phone URL>`: OpenCV + person detector, JSON to the Pi |

## Public kit defaults (not secrets)

| Setting | Value |
| --- | --- |
| Wi-Fi | Phone hotspot SSID `QNX`, password `everywhere` (the Pi joins it on boot) |
| SSH | `qnxuser` / `qnxuser` |
| VNC | Server runs by default, password `qnxuser` |

Run only **one** hotspot named `QNX` (Felix's iPhone). If `.local` names don't resolve, read the
Pi's IP from the hotspot client list; Ethernet from a laptop is the last resort.

## Headless setup

There is no keyboard or mouse. Both teammates `ssh qnxuser@qnxpi-felix.local` from their own
laptops at the same time; deploys go one person at a time (say "deploying" in chat first).

Display: **Felix's monitor on HDMI0** for development. Later a small screen is the light source of
the Pepper's ghost visor.

Quick check:

```bash
ssh qnxuser@qnxpi-felix.local 'hostname; uname -a; pidin info | head -3'
```

Expect `qnxpi-felix`, QNX 8.0 and about 8 GB of memory.

## Camera (verified 12:18am Sat)

The Logitech C270 is the default USB camera, CamAPI **unit 5** (CSI units 3 and 4 are unused):

```bash
camera_example3_viewfinder -u 5
```

It shows the C270 image on the HDMI0 monitor. If the default entry is slow, try the MJPEG entry in
`sensor_rpi5.conf`. The C270 isn't on QNX's tested list (the C920 is); fallback is a C920 from the
hardware desk.

## VNC over an SSH tunnel

On an iPhone hotspot the Mac **can't reach the Pi's IPv4 address** directly, so
`vnc://qnxpi-felix.local` does not work. Tunnel it:

```bash
ssh -L 5901:localhost:5900 qnxuser@qnxpi-felix.local
```

Then on the Mac: Finder, Cmd+K, `vnc://localhost:5901`, password `qnxuser`.

**VNC tearing is not the camera.** VNC looks choppy; judge FPS on the HDMI monitor only. The same
reachability problem can hit the WS6 Mac relay; send to the Pi's IP from the hotspot list or use an
SSH tunnel.

## Gotchas

- **Hostname lives in `/boot/network`** as `HOSTNAME=`, applied at boot when that file is newer
  than the system setting. This image has **no `/boot/qnx_config.txt`**, and editing
  `/boot/user-data` (cloud-config `hostname:`) does nothing
  ([QNX docs: boot](https://www.qnx.com/developers/docs/qnxeverywhere/com.qnx.doc.target_images/topic/cti/boot.html)).
  What Felix ran (done, confirmed 11:36pm Fri):

  ```bash
  sudo sed -i 's/^HOSTNAME=qnxpi$/HOSTNAME=qnxpi-felix/' /boot/network
  sudo shutdown
  ```

- **`shutdown` reboots by default.** There is no `-r` flag (`shutdown -r` is an illegal option).
- Priorities: `on -p 50 <cmd>` runs at a priority; `pidin` shows each thread's priority. Setting
  priorities above the user limit may need `sudo`.
- Useful tools: `pidin`, `slay <name>`, `sloginfo`, `hogs`, `use <binary>`, package manager `apk`.
- Power: use the official 27 W USB-C supply; weak supplies cause random resets.

## QNX pitch reminders

Separate processes per stage, a supervisor watchdog that restarts `hud_infer` live while the HUD
keeps drawing, and a latency chart under `hud_stress` (HUD at priority 50 flat vs 10 spiky). Details:
[architecture.md](architecture.md) and [INTERFACES.md](../INTERFACES.md) IF-3, IF-6, IF-9.
