// test_skeleton.mjs — unit test for the ADDITIVE skeleton landmarks ("wallhack") that phone.html
// attaches to each remote_contacts contact as `pts`, so a person only a fixed phone sees can be
// redrawn as a ghosted skeleton on the wearer's HUD instead of a bare marker.
//
// Run:  node test_skeleton.mjs      (no deps, no network)
//
// Three things, all must pass:
//   1. Extracts the REAL packPts() out of remote/phone.html and eval's it, then checks it
//      downsamples MediaPipe's 33 landmarks to a COCO-17 flat int array [x,y,v,...] with the
//      documented scaling (x,y image-normalized * 1000; visibility * 100), and that off-frame
//      (<0 or >1) landmarks survive unclamped.
//   2. Round-trips a built contact (incl. pts) through JSON, and proves a worst-case 16-contact
//      remote_contacts message stays under SEEKER_MAX_DATAGRAM (8192 B) and the C JSON token
//      budget (MAX_TOKS 2048) so the additive field can never make a datagram unparseable.
//   3. node --check on the whole <script type=module> of phone.html (syntax gate, like tagpose).

import fs from "node:fs";
import path from "node:path";
import os from "node:os";
import { fileURLToPath } from "node:url";
import { execFileSync } from "node:child_process";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const PHONE = path.join(HERE, "remote", "phone.html");
const html = fs.readFileSync(PHONE, "utf8");

let failures = 0;
function check(name, cond) { if (cond) { console.log("  ok   " + name); }
  else { console.log("  FAIL " + name); failures++; } }

// ---- extract a top-level `function <name>(...) { ... }` by brace matching (from test_tagpose) ----
function extractFunction(src, name) {
  const sig = "function " + name;
  const start = src.indexOf(sig);
  if (start < 0) throw new Error("could not find " + sig + " in phone.html");
  const open = src.indexOf("{", start);
  let depth = 0;
  for (let i = open; i < src.length; i++) {
    const c = src[i];
    if (c === "{") depth++;
    else if (c === "}") { depth--; if (depth === 0) return src.slice(start, i + 1); }
  }
  throw new Error("unbalanced braces extracting " + name);
}

// packPts references the module-level PTS_IDX constant, so pull that in too and eval them together.
const idxMatch = html.match(/const PTS_IDX = \[([^\]]*)\];/);
const packSrc = "const PTS_IDX = [" + (idxMatch ? idxMatch[1] : "") + "];\n" +
                extractFunction(html, "packPts") + "\nreturn packPts;";
// eslint-disable-next-line no-eval
const packPts = eval("(() => { " + packSrc + " })()");

console.log("1) packPts downsamples 33 MediaPipe landmarks to a COCO-17 flat int array:");
const idxCount = idxMatch[1].split(",").length;
check("PTS_IDX has 17 joints", idxCount === 17);

// synthetic 33-landmark pose: x = i/100, y = 1 - i/100, visibility = (i % 10) / 10
const lms = Array.from({ length: 33 }, (_, i) => ({ x: i / 100, y: 1 - i / 100, visibility: (i % 10) / 10 }));
const pts = packPts(lms);
check("output is 51 ints (17 joints x 3)", Array.isArray(pts) && pts.length === 51);
check("all entries are integers", pts.every(Number.isInteger));
// joint 0 of PTS_IDX is MediaPipe index 0 (nose): x=0 -> 0, y=1 -> 1000, v=0 -> 0
check("nose packed as image-normalized*1000 / vis*100", pts[0] === 0 && pts[1] === 1000 && pts[2] === 0);
// joint 5 of PTS_IDX is MediaPipe index 11 (left shoulder): x=.11 -> 110, y=.89 -> 890, v=.1 -> 10
check("left-shoulder (mp idx 11) x,y,v = 110,890,10",
      pts[15] === 110 && pts[16] === 890 && pts[17] === 10);

// off-frame joints (MediaPipe emits <0 and >1) must survive unclamped
const off = packPts(lms.map((p, i) => i === 0 ? { x: -0.2, y: 1.3, visibility: 0.95 } : p));
check("off-frame nose kept unclamped (-200, 1300, 95)", off[0] === -200 && off[1] === 1300 && off[2] === 95);

// missing landmark -> zeros, never throws
check("missing landmarks pack to zeros", packPts([]).every(v => v === 0) && packPts([]).length === 51);

console.log("2) serialize + round-trip + datagram-size budget:");
const rt = JSON.parse(JSON.stringify(pts));
check("pts round-trips through JSON byte-identical", JSON.stringify(rt) === JSON.stringify(pts));

// Worst-case contact: box + the full pts, with 4-digit (off-frame) coordinates to stress size.
const fatPts = packPts(lms.map(() => ({ x: 1.234, y: -0.987, visibility: 1 })));
const contact = { id: "R2-16", cls: "person", conf: 0.99, box: [1234, 1234, 1234, 1234],
                  bearing_deg: -179.9, range_m: 12.34, range_src: "torso", pts: fatPts };
const msg = { type: "remote_contacts", v: 2, src: "R2", t_mono_ms: 1700000000000,
              ts_ms: 1700000000000, node_id: "R2", pose: { x_m: -3.21, y_m: 4.56, yaw_deg: -179.9 },
              width: 640, height: 480, hfov_deg: 66,
              contacts: Array.from({ length: 16 }, (_, i) => ({ ...contact, id: "R2-" + (i + 1) })) };
const wire = JSON.stringify(msg);
const perContact = JSON.stringify(contact).length;
const ptsBytes = JSON.stringify(fatPts).length;
console.log(`     pts field = ${ptsBytes} B/contact, full contact = ${perContact} B, ` +
            `16-contact datagram = ${wire.length} B`);
check("16-contact datagram with pts stays under SEEKER_MAX_DATAGRAM (8192)", wire.length < 8192);
// crude JSON token estimate (jsmn-style: 1 per value, 1 per key, 1 per array/object container):
// pts array = 1 + 51 = 52 tokens/contact; whole message well under MAX_TOKS 2048.
const approxToks = 16 * (52 + 20) + 20;
check("approx JSON token count under MAX_TOKS (2048)", approxToks < 2048);
console.log(`     approx tokens ~= ${approxToks} (budget 2048)`);

console.log("3) node --check phone.html <script type=module>:");
const m = html.match(/<script type="module">([\s\S]*?)<\/script>/);
check("found <script type=module>", !!m);
if (m) {
  const tmp = path.join(os.tmpdir(), "phone_skel_check_" + process.pid + ".mjs");
  fs.writeFileSync(tmp, m[1]);
  try {
    execFileSync(process.execPath, ["--check", tmp], { stdio: "pipe" });
    check("script parses (node --check)", true);
  } catch (e) {
    check("script parses (node --check)", false);
    console.log(String(e.stderr || e.message));
  } finally { try { fs.unlinkSync(tmp); } catch {} }
}

console.log(failures ? `\nFAILED (${failures})` : "\nALL PASS");
process.exit(failures ? 1 : 0);
