// test_tagpose.mjs — unit test for the ?tag=1 fiducial self-localization math in remote/phone.html.
//
// Run:  node test_tagpose.mjs      (no deps, no network)
//
// It does two things, both must pass:
//   1. Extracts the REAL cameraPoseFromMarker() source out of remote/phone.html and eval's it, then
//      checks it recovers a known camera (x_m, y_m, yaw_deg) from a synthesized marker pose, for the
//      default tagmap=xz and a couple of alternative mappings.
//   2. Extracts the whole <script type="module"> from phone.html and runs `node --check` on it, so a
//      syntax error in the page (including the parts that aren't headless-testable) fails CI.
//
// NOTE ON SCOPE (honest): test #1 is self-consistent — it builds R,t with the same inversion
// convention the function uses, so it proves the linear algebra + axis mappings, NOT that js-aruco2's
// POSIT emits R,t in that exact convention. That second half WAS verified separately against the real
// js-aruco2 POSIT library (bestTranslation in mm with +z, det(R)=+1, position recovered) outside this
// committed test (it needs a network fetch of the library). The physical room-frame axis/sign
// (tagmap / tagyaw) still needs a quick on-device confirmation against the printed marker.

import fs from "node:fs";
import path from "node:path";
import os from "node:os";
import { fileURLToPath } from "node:url";
import { execFileSync } from "node:child_process";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const PHONE = path.join(HERE, "remote", "phone.html");
const html = fs.readFileSync(PHONE, "utf8");

let failures = 0;
const approx = (a, b, tol) => Math.abs(a - b) <= tol;
function check(name, cond) { if (cond) { console.log("  ok   " + name); }
  else { console.log("  FAIL " + name); failures++; } }

// ---- extract a top-level `function <name>(...) { ... }` by brace matching ----
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

const camSrc = extractFunction(html, "cameraPoseFromMarker");
// eslint-disable-next-line no-eval
const cameraPoseFromMarker = eval("(" + camSrc + ")");

// ---- tiny 3x3 vector helpers for synthesizing a marker pose ----
const matVec = (M, v) => [
  M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
  M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
  M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2]];
const transpose = M => [[M[0][0],M[1][0],M[2][0]],[M[0][1],M[1][1],M[2][1]],[M[0][2],M[1][2],M[2][2]]];
const cross = (a,b) => [a[1]*b[2]-a[2]*b[1], a[2]*b[0]-a[0]*b[2], a[0]*b[1]-a[1]*b[0]];
const norm  = v => { const n = Math.hypot(...v); return [v[0]/n, v[1]/n, v[2]/n]; };
const d2r = d => d*Math.PI/180;

// Build R_cam (camera->marker) with 3rd column = fwd (any valid orthonormal completion; the
// recovered x,y,yaw depend only on the 3rd column and the translation, not on cols 0/1).
function rotFromForward(fwd) {
  const f = norm(fwd);
  const up = Math.abs(f[1]) > 0.99 ? [1,0,0] : [0,1,0];
  const right = norm(cross(up, f));
  const trueUp = cross(f, right);
  return [[right[0], trueUp[0], f[0]],
          [right[1], trueUp[1], f[1]],
          [right[2], trueUp[2], f[2]]];           // columns: right, up, forward
}

// Synthesize the marker's pose-in-camera (R, t) for a target camera room pose under `tagmap`.
// t_cam = camera position in marker coords (mm); fwd = camera forward in marker coords.
function synth(tagmap, X, Y, yawDeg) {
  const y = d2r(yawDeg), H = -1700, Zc = 2000;     // arbitrary off-floor components
  let tcam, fwd;
  switch (tagmap) {
    case "xz":        tcam = [X*1000, H,   Y*1000]; fwd = [Math.sin(y), 0,  Math.cos(y)]; break;
    case "xz_rot180": tcam = [-X*1000, H, -Y*1000]; fwd = [-Math.sin(y), 0, -Math.cos(y)]; break;
    case "xy":        tcam = [X*1000, Y*1000, Zc];  fwd = [Math.sin(y), Math.cos(y), 0];  break;
    default: throw new Error("synth: unhandled map " + tagmap);
  }
  const Rcam = rotFromForward(fwd);
  const R = transpose(Rcam);                        // marker->camera
  const t = matVec(R, tcam).map(v => -v);           // t = -R * t_cam  (marker centre in camera frame)
  return { R, t };
}

function caseTest(tagmap, X, Y, yawDeg) {
  const { R, t } = synth(tagmap, X, Y, yawDeg);
  const got = cameraPoseFromMarker(R, t, tagmap, 0);
  const label = `${tagmap} (${X},${Y},${yawDeg}deg) -> ` +
                `x=${got.x_m} y=${got.y_m} yaw=${got.yaw_deg}`;
  check(label, approx(got.x_m, X, 0.01) && approx(got.y_m, Y, 0.01) && approx(got.yaw_deg, yawDeg, 0.2));
}

console.log("1) cameraPoseFromMarker recovers the camera pose:");
// default mapping — required
caseTest("xz", 1.5, 3.0, 30);
caseTest("xz", -2.0, 4.2, -90);
caseTest("xz", 0.0, 2.5, 0);
caseTest("xz", 0.75, 1.1, 170);
// alternative mappings exposed via ?tagmap= (mirror maps removed per Felix review)
caseTest("xz_rot180", 1.5, 3.0, 45);
caseTest("xy", -1.2, 2.8, -60);
// the additive yaw offset (?tagyaw=) is applied
(() => {
  const { R, t } = synth("xz", 1.0, 2.0, 10);
  const got = cameraPoseFromMarker(R, t, "xz", 25);
  check(`xz yaw offset +25 -> ${got.yaw_deg}`, approx(got.yaw_deg, 35, 0.2));
})();

// ---- 2) node --check the extracted module script of phone.html ----
console.log("2) node --check phone.html <script type=module>:");
const m = html.match(/<script type="module">([\s\S]*?)<\/script>/);
check("found <script type=module>", !!m);
if (m) {
  const tmp = path.join(os.tmpdir(), "phone_script_check_" + process.pid + ".mjs");
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
