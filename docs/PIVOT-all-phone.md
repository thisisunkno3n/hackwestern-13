# PROJECT PIVOT: all-phone vision into the RTOS

**Decision by Daniel (co-lead), 2026-11-21. Project-wide. Supersedes the C270 helmet-camera design.**
Felix: this rewrites parts of WS1/WS2/WS4; please confirm the HUD question below; everything else
reuses the existing interfaces.

## What changes
- **No webcams anywhere.** The C270 and `hud_capture`'s CamAPI local-capture path are **out**.
- **Two phones are the cameras AND the vision.** Each phone runs person detection in the browser
  (MediaPipe / TF.js), and **streams detections** (not video) into the **RTOS (QNX Pi)**.
- **The QNX Pi fuses the two detection streams and drives the display.** Fusion (`hud_fusion`, C) and
  the display stay on the Pi; that's still the RTOS story: deterministic real-time fusion + draw.

## New data flow
```
Phone R1 (browser: getUserMedia -> MediaPipe person detect) --\
                                                               >--UDP/WS detections--> QNX Pi
Phone R2 (browser: getUserMedia -> MediaPipe person detect) --/        hud_fusion (C, RTOS)
                                                                         -> hud_display (HUD)
```
- Transport + message: **reuse IF-5.4 `remote_contacts`** almost unchanged: every phone is a node
  (`R1`, `R2`) sending `{id, cls, conf, box, bearing_deg, range_m}` with its taped `pose`. No new
  contract; no "local vs remote" split anymore (there is no local camera).
- Low bandwidth (detections, not video): good for the hotspot and for the RTOS.

## Impact per workstream
| WS | Before | After |
| --- | --- | --- |
| WS1 capture | C270 via CamAPI to frame ring | **Dropped** (no local camera). CamAPI spike kept as reference only. |
| WS2 detection | TFLite/ONNX on the Pi | **Moves to the phones** (in-browser MediaPipe/TF.js). |
| WS3 fusion (Felix) | fuse local + remote | **Stays**: now fuses two phone nodes. Mostly unchanged. |
| WS4 HUD | see-through wearer view + overlays | **See HUD question below.** |
| WS6 (Daniel) | Mac relay reads phone IP-cam | **Phone sensor web app + Pi ingest.** No Mac relay needed. |
| WS7 visor | Pepper's-ghost combiner over wearer cam | Changes / optional (no wearer camera). |

## The one open question for Felix: what does the HUD draw over?
There's no wearer camera now, so the HUD background changes. Two choices:
- **(A) Tactical radar / god's-eye map (recommended).** The Pi draws a top-down room map with the two
  phones, their view cones, and fused contacts (cyan = seen only by a node). Detections-only, fully
  RTOS-light, matches the "EagleEye" framing. Least new work.
- **(B) One phone is the wearer's view.** Its video is streamed to the Pi and shown with overlays
  (true see-through HUD). Heavier: video into the Pi + decode; reintroduces the latency risk the
  brief warns about. Keep only if the wearer-POV look is essential.

**Recommendation: (A)** for the MVP; add (B) as a stretch if time allows.

## Immediate next steps
1. Daniel's agent builds the **phone sensor web app** (reuse the MediaPipe page) + the **Pi ingest**
   (phones -> `remote_contacts` -> `hud_fusion`), target node ids `R1`/`R2`.
2. Felix confirms HUD (A) vs (B) and adjusts WS4; WS1/WS2 briefs retire or repoint.
3. INTERFACES.md edits (remove the local-camera assumptions) land as a PR Felix merges.
