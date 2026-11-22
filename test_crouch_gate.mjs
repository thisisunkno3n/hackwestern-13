// test_crouch_gate.mjs: phone.html's upright gate (isUpright) must not drop a crouched person.
//
// Run:  node test_crouch_gate.mjs      (no deps, no network)
//
// The gate drops detections whose landmark box is under ?minaspect (1.5) tall per wide: clutter,
// bags, draped cloth. A crouched or sitting person is about as wide as tall, so the wearer's
// phone dropped them and the HUD lost its SEEN box (the Pi then drew another phone's x-ray ghost,
// or a projected standing-size box, tall and narrow). Now a pose that shows a body (shoulders,
// hips and a whole leg over visibility 0.5) is kept down to ?crouchaspect (0.7).
//
// Extracts the REAL isUpright() and bodyShown() from remote/phone.html and eval's them.

import fs from "node:fs";
import path from "node:path";
import { fileURLToPath } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const html = fs.readFileSync(path.join(HERE, "remote", "phone.html"), "utf8");

let failures = 0;
function check(name, cond) { if (cond) { console.log("  ok   " + name); }
  else { console.log("  FAIL " + name); failures++; } }

function extractFunction(src, name) {
  const sig = "function " + name + "(";
  const start = src.indexOf(sig);
  if (start < 0) return "";
  const open = src.indexOf("{", start);
  let depth = 0;
  for (let i = open; i < src.length; i++) {
    const c = src[i];
    if (c === "{") depth++;
    else if (c === "}") { depth--; if (depth === 0) return src.slice(start, i + 1); }
  }
  throw new Error("unbalanced braces extracting " + name);
}
function constLine(name) {
  const m = html.match(new RegExp("^const " + name + "\\s*=.*$", "m"));
  return m ? m[0] : "";
}

// The gate with a given query string, as phone.html builds it.
function gate(query) {
  const src = "const qs = new URLSearchParams(" + JSON.stringify(query) + ");\n" +
    ["VIS_MIN", "MIN_ASPECT", "BODY_ASPECT", "vis"].map(constLine).join("\n") + "\n" +
    extractFunction(html, "bodyShown") + "\n" + extractFunction(html, "isUpright") + "\nreturn isUpright;";
  // eslint-disable-next-line no-eval
  return eval("(() => { " + src + " })()");
}

// 33 MediaPipe landmarks from a few named joints (image-normalized, y down); the rest unseen.
const IDX = { nose: 0, lsh: 11, rsh: 12, lhip: 23, rhip: 24, lkn: 25, rkn: 26, lank: 27, rank: 28 };
function pose(j, legVis = 0.9) {
  const lms = Array.from({ length: 33 }, () => ({ x: 0.5, y: 0.5, visibility: 0 }));
  for (const [k, [x, y]] of Object.entries(j))
    lms[IDX[k]] = { x, y, visibility: /kn|ank/.test(k) ? legVis : 0.9 };
  return lms;
}
function boxOf(lms, W = 640, H = 480) {
  let x0 = 1, y0 = 1, x1 = 0, y1 = 0;
  for (const p of lms) { if (p.visibility < 0.3) continue;
    x0 = Math.min(x0, p.x); y0 = Math.min(y0, p.y); x1 = Math.max(x1, p.x); y1 = Math.max(y1, p.y); }
  return [x0 * W, y0 * H, (x1 - x0) * W, (y1 - y0) * H];
}
const aspect = b => b[3] / b[2];

// Standing 3 m away, a crouch facing the phone (knees out, hands wide), and a full squat.
const stand = pose({ nose: [0.50, 0.20], lsh: [0.54, 0.30], rsh: [0.46, 0.30], lhip: [0.53, 0.52],
                     rhip: [0.47, 0.52], lkn: [0.53, 0.68], rkn: [0.47, 0.68], lank: [0.53, 0.84], rank: [0.47, 0.84] });
const crouchJ = { nose: [0.50, 0.44], lsh: [0.56, 0.52], rsh: [0.44, 0.52], lhip: [0.53, 0.68],
                  rhip: [0.47, 0.68], lkn: [0.61, 0.72], rkn: [0.39, 0.72], lank: [0.56, 0.84], rank: [0.44, 0.84] };
const crouch = pose(crouchJ);
const squatJ = { nose: [0.50, 0.56], lsh: [0.56, 0.62], rsh: [0.44, 0.62], lhip: [0.53, 0.76],
                 rhip: [0.47, 0.76], lkn: [0.63, 0.74], rkn: [0.37, 0.74], lank: [0.57, 0.84], rank: [0.43, 0.84] };
const squat = pose(squatJ);
// The same squat shape with no leg the model is sure of (a bag, a chair with a coat on it).
const clutter = pose(squatJ, 0.35);

console.log("1) the default gate:");
const g = gate("");
check("the functions were found in phone.html", typeof g === "function" && constLine("BODY_ASPECT") !== "");
check("standing person kept (box " + aspect(boxOf(stand)).toFixed(2) + " tall per wide)", g(stand, boxOf(stand)) === true);
check("crouched person kept (box " + aspect(boxOf(crouch)).toFixed(2) + ")", aspect(boxOf(crouch)) < 1.5 && g(crouch, boxOf(crouch)) === true);
check("full squat kept (box " + aspect(boxOf(squat)).toFixed(2) + ")", aspect(boxOf(squat)) < 1.5 && g(squat, boxOf(squat)) === true);
check("squat shape without a sure leg still dropped", g(clutter, boxOf(clutter)) === false);
const flat = pose({ ...squatJ, lkn: [0.80, 0.74], rkn: [0.20, 0.74] });
check("a body wider than 1 / 0.7 of its height still dropped (box " + aspect(boxOf(flat)).toFixed(2) + ")",
      aspect(boxOf(flat)) < 0.7 && g(flat, boxOf(flat)) === false);
const upside = pose({ ...crouchJ, lsh: [0.56, 0.70], rsh: [0.44, 0.70], lhip: [0.53, 0.52], rhip: [0.47, 0.52] });
check("shoulders below hips still dropped", g(upside, boxOf(upside)) === false);

console.log("2) URL parameters stay compatible:");
check("?crouchaspect=1.5 is the old gate (crouch dropped)", gate("crouchaspect=1.5")(crouch, boxOf(crouch)) === false);
check("?minaspect=1.0 still loosens the gate for everything", gate("minaspect=1.0")(clutter, boxOf(clutter)) ===
      (aspect(boxOf(clutter)) >= 1.0));
check("?minaspect below crouchaspect wins for a body", gate("minaspect=0.5")(flat, boxOf(flat)) === (aspect(boxOf(flat)) >= 0.5));

console.log(failures ? failures + " FAILED" : "all passed");
process.exit(failures ? 1 : 0);
