# Seeker HUD demo script (2 minutes)

Setup: Felix wears the helmet with the visor. The laptop shows the MJPEG view of the HUD (video
mode) so the judges see what he sees, plus a terminal with SSH to the Pi. **A phone (the remote
camera, IP camera app, relayed through Daniel's Mac)** watches a spot the wearer can't see (behind a
partition or a door). Daniel runs the laptop and plays the "hidden person"; every move is
pre-staged. (Solo fallback: a volunteer plays the hidden person and the node runs on Felix's laptop.) Keep the
on-screen terms (`FUSION LOST`, `FUS->HUD`, `R1`) identical to the narration.

| Time | Beat | Do | Say |
| --- | --- | --- | --- |
| 0:00 | Hook | Put on the helmet | "Firefighters lose people in smoke. Fighter pilots have helmets that show them who's around. This is Seeker: a helmet HUD on QNX that marks people, even ones you can't see." |
| 0:15 | The HUD | Look at a judge | (brackets on the judge, range, heading tape) "A camera on the helmet, a Raspberry Pi 5 on QNX, and a see-through combiner. Everything is detected on the device: TensorFlow Lite from QNX's open-source ports." |
| 0:30 | Range and colour | Daniel walks toward Felix | "Range comes from how tall a person looks in the frame. Green far, amber closer, red when someone's right on top of you or closing fast." |
| 0:45 | **The twist** | Daniel steps behind the partition | (cyan diamond and radar contact appear) "Now he's behind a wall. A phone camera across the room feeds a detector and sends only what it sees to the helmet, so I still see him, marked cyan." |
| 1:00 | **Kill fusion** | In the terminal: `slay hud_fusion` | "Let's kill the tracker." (`FUSION LOST`, video and tape keep running) "The HUD never froze: it's a separate process at the highest priority. The supervisor restarts the tracker..." (contacts come back) "...in about a second and a half." |
| 1:20 | **Latency under stress** | Point at the laptop chart; press `s` for stress | "Here's camera-to-display latency with every core maxed out. Flat line: the HUD at top priority. Spiky line: the same code at normal priority. That's why this runs on QNX, not a web page on QNX." |
| 1:40 | `pidin` | Show the priorities | "HUD 50 at the top, supervisor 45, fusion 30 below. The display can't be starved." |
| 1:50 | Close | | "Seeker: a fighter HUD for the people who run into buildings. Efficient, real-time, and it never freezes." |

## Optional beats (only if shipped)

- **Head tracking (WS8):** at 0:45, turn the head; the cyan marker stays fixed on the partition. "It
  stays locked to the world with AirPods head tracking."
- **Stereo range (WS3):** at 0:30, "and with two cameras, range from stereo disparity."
- **EagleEye wearer view:** `hud_display --view wearer` with a helmet phone `W`: solid box for
  people the helmet sees, dashed cyan `XRAY` box for people behind the wall, edge arrows for people
  behind. 60-second beat and setup in [drafts/pi-lane/docs/EAGLEEYE.md](drafts/pi-lane/docs/EAGLEEYE.md#demo-script-60-s-inside-the-main-2-minute-demo).

## Rehearsal notes

- Dim the room near the demo table if allowed; the combiner needs contrast.
- Have the `slay` command typed and ready; never type it live.
- Pre-record the latency chart as well as running it live; show the recorded one if the live run is slow.
- If the visor isn't readable, demo in video mode on the laptop and hand the visor to a judge.
- If running on the Radxa Cubie fallback: say "embedded Linux board", drop `pidin` (use `ps -eLo
  pid,cls,rtprio,comm`), and never claim QNX ([prize-strategy.md](reference/prize-strategy.md#fallback-no-qnx-pi-5)).
- Q&A crib: why QNX (process isolation, priorities, determinism under load); why on-device (no
  network in a fire, latency); what's next (thermal camera for smoke, a custom helmet PCB, mesh
  networking between helmets).
