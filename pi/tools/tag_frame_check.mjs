// tag_frame_check.mjs: end-to-end check of the R1 ArUco pose frame against what hud_fusion and
// hud_display expect (IF-1: +x right, +y forward, yaw_deg clockwise from +y, bearing positive to
// the right). Host only, a pre-filter: the real check is the phone in front of the printed tag.
//
//   node tools/tag_frame_check.mjs [path/to/phone.html]
//     PHONE_HTML  Daniel's remote/phone.html (default ../../../hackwestern-13/remote/phone.html)
//     ARUCO_DIR   folder with aruco_esm.js and posit_esm.js (the jsDelivr +esm bundles phone.html
//                 imports); fetched from jsDelivr when not set
//     TAG_MM      printed black square side in mm (186)
//
// What it does, with no phone and no browser:
//   1. Loads js-aruco2 the way phone.html does (`(await import(URL)).default`) and checks that
//      the default export really has Detector / Posit.
//   2. Renders printed marker ARUCO_MIP_36h12 id 0 (js-aruco2's own generateSVG layout) taped
//      upright on a wall, through a pinhole camera at known room poses, at the phone's tag
//      detection widths (320 and 640 px wide frames from 640x480 video).
//   3. Runs the real AR.Detector and POS.Posit on it, then phone.html's own cameraPoseFromMarker
//      (extracted from the page), and compares x_m, y_m, yaw_deg with the truth.
//   4. Puts a person in the room, computes the bearing the phone would send (atan((u - cx) / f)),
//      and places it with hud_fusion's seeker_pose_to_world (fusion/seeker_pose.c). The person
//      must land where they stand, for every ?tagmap= that claims a wall-mounted marker.
//
// Room frame this checks (default tagmap=xz, marker upright on a wall): origin at the tag,
// +y = the way you face when you look straight at the tag (into the wall), +x = your right,
// yaw 0 = facing the tag, 90 = facing right. Everyone in front of the tag has y < 0.
//
// Exit code: 0 all pass, 1 any check failed, 2 setup error.

import fs from "node:fs";
import path from "node:path";
import os from "node:os";
import { fileURLToPath, pathToFileURL } from "node:url";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const PHONE = process.argv[2] || process.env.PHONE_HTML ||
  path.resolve(HERE, "../../../../hackwestern-13/remote/phone.html");
const TAG_MM = parseFloat(process.env.TAG_MM || "186");
const HFOV = 66;
const ARUCO_URL = "https://cdn.jsdelivr.net/npm/js-aruco2@2.0.0/src/aruco.js/+esm";
const POSIT_URL = "https://cdn.jsdelivr.net/npm/js-aruco2@2.0.0/src/posit1.js/+esm";

let fails = 0;
const ok = (name, cond, extra = "") => {
  console.log(`  ${cond ? "PASS" : "FAIL"} ${name}${extra ? "  " + extra : ""}`);
  if (!cond) fails++;
  return cond;
};
const d2r = (d) => (d * Math.PI) / 180, r2d = (r) => (r * 180) / Math.PI;
const wrap = (d) => ((((d + 180) % 360) + 360) % 360) - 180;

async function loadBundle(file, url) {
  let p;
  if (process.env.ARUCO_DIR) {
    p = path.join(process.env.ARUCO_DIR, file);
  } else {
    const r = await fetch(url);
    if (!r.ok) throw new Error(`fetch ${url}: HTTP ${r.status}`);
    p = path.join(os.tmpdir(), `tag_frame_check_${process.pid}_${file}`);
    fs.writeFileSync(p, await r.text());
  }
  return import(pathToFileURL(p).href);
}

function extractFunction(src, name) {
  const start = src.indexOf("function " + name);
  if (start < 0) throw new Error(`no function ${name} in ${PHONE}`);
  for (let i = src.indexOf("{", start), depth = 0; i < src.length; i++) {
    if (src[i] === "{") depth++;
    else if (src[i] === "}" && --depth === 0) return src.slice(start, i + 1);
  }
  throw new Error(`unbalanced braces in ${name}`);
}

// hud_fusion fusion/seeker_pose.c seeker_pose_to_world, line for line.
function poseToWorld(p, bearing, range) {
  const a = d2r(p.yaw_deg + bearing);
  return { x: p.x_m + range * Math.sin(a), y: p.y_m + range * Math.cos(a) };
}

// Camera at room (x, y, height h), yaw clockwise from +y, level. Room 3D: X right, Y into the
// wall, Z up. Returns basis vectors and a projector for room points.
function camera(x, y, h, yawDeg, W, H) {
  const a = d2r(yawDeg), f = W / 2 / Math.tan(d2r(HFOV / 2));
  const fwd = [Math.sin(a), Math.cos(a), 0], right = [Math.cos(a), -Math.sin(a), 0];
  return { C: [x, y, h], fwd, right, up: [0, 0, 1], f, W, H };
}

// Marker id 0 upright on the wall plane Y = 0, centre at (0, 0, hm): js-aruco2 generateSVG layout
// (8 x 8 cells: black border, 6 x 6 bits row-major from the top, '1' = white), white paper margin.
function render(cam, code, sideM, hm) {
  const { W, H, f, C, fwd, right, up } = cam, cell = sideM / 8, ss = 3;
  const data = new Uint8ClampedArray(W * H * 4);
  for (let v = 0; v < H; v++) {
    for (let u = 0; u < W; u++) {
      let acc = 0;
      for (let sy = 0; sy < ss; sy++) {
        for (let sx = 0; sx < ss; sx++) {
          const pu = u + (sx + 0.5) / ss - W / 2, pv = H / 2 - (v + (sy + 0.5) / ss);
          const d = [0, 1, 2].map((k) => fwd[k] * f + right[k] * pu + up[k] * pv);
          let g = 170; /* wall */
          if (d[1] > 1e-9) {
            const t = -C[1] / d[1];
            const mx = C[0] + t * d[0], my = C[2] + t * d[2] - hm;
            const ix = Math.floor((mx + sideM / 2) / cell), iy = Math.floor((sideM / 2 - my) / cell);
            if (ix >= 0 && ix < 8 && iy >= 0 && iy < 8) {
              const border = ix === 0 || ix === 7 || iy === 0 || iy === 7;
              g = !border && code[(iy - 1) * 6 + (ix - 1)] === "1" ? 235 : 20;
            } else if (Math.abs(mx) < sideM / 2 + 1.5 * cell && Math.abs(my) < sideM / 2 + 1.5 * cell) {
              g = 235; /* paper */
            }
          }
          acc += g;
        }
      }
      const g = acc / (ss * ss), i = (v * W + u) * 4;
      data[i] = data[i + 1] = data[i + 2] = g;
      data[i + 3] = 255;
    }
  }
  return { width: W, height: H, data };
}

// Bearing (deg, + right) and range the phone would report for a room point, as phone.html does it.
function phoneSees(cam, px, py) {
  const dx = px - cam.C[0], dy = py - cam.C[1];
  const xc = dx * cam.right[0] + dy * cam.right[1], zc = dx * cam.fwd[0] + dy * cam.fwd[1];
  const u = cam.W / 2 + (cam.f * xc) / zc;
  return { bearing: r2d(Math.atan((u - cam.W / 2) / cam.f)), range: Math.hypot(dx, dy), zc };
}

// Physical room point -> the 2D frame a given ?tagmap= reports in (marker coords m = [X, up, Y]).
const MAPS = {
  xz: (X, Y) => ({ x: X, y: Y }),
  xz_negz: (X, Y) => ({ x: X, y: -Y }),
  xz_negx: (X, Y) => ({ x: -X, y: Y }),
  zx: (X, Y) => ({ x: Y, y: X }),
};

async function main() {
  if (!fs.existsSync(PHONE)) {
    console.error(`no phone.html at ${PHONE} (pass the path or set PHONE_HTML)`);
    return 2;
  }
  const html = fs.readFileSync(PHONE, "utf8");
  const cameraPoseFromMarker = eval("(" + extractFunction(html, "cameraPoseFromMarker") + ")");
  let arMod, posMod;
  try {
    arMod = (await loadBundle("aruco_esm.js", ARUCO_URL)).default;
    posMod = (await loadBundle("posit_esm.js", POSIT_URL)).default;
  } catch (e) {
    console.error(`cannot load js-aruco2 (${e.message}); set ARUCO_DIR for offline runs`);
    return 2;
  }

  console.log(`phone.html: ${PHONE}`);
  console.log("1) js-aruco2 module shape, loaded the way phone.html loads it:");
  const loadsLikePhone = /AR\s*=\s*\(await import\(ARUCO_URL\)\)\.default\s*;/.test(html);
  ok("AR = (await import(ARUCO_URL)).default has Detector", typeof arMod.Detector === "function",
     typeof arMod.Detector !== "function" ? `default export keys: ${Object.keys(arMod)}` : "");
  ok("POS = (await import(POSIT_URL)).default has Posit", typeof posMod.Posit === "function",
     typeof posMod.Posit !== "function" ? `default export keys: ${Object.keys(posMod)}` : "");
  if (loadsLikePhone && typeof arMod.Detector !== "function")
    console.log("       phone.html would show TAG LOAD FAIL; use .default.AR and .default.POS");
  const AR = arMod.AR || arMod, POS = posMod.POS || posMod;

  const det = new AR.Detector({ dictionaryName: "ARUCO_MIP_36h12" });
  const code = det.dictionary.codeList[0];
  const side = TAG_MM / 1000, hm = 1.4, hc = 1.7;

  // One detection pass exactly like phone.html tagDetect() on a W x H detection frame.
  function phonePose(cam, tagmap) {
    const img = render(cam, code, side, hm);
    const mk = det.detect(img).find((m) => m.id === 0);
    if (!mk) return null;
    const fpx = cam.W / 2 / Math.tan(d2r(HFOV / 2));
    const posit = new POS.Posit(TAG_MM, fpx);
    const pts = mk.corners.map((c) => ({ x: c.x - cam.W / 2, y: cam.H / 2 - c.y }));
    const p = posit.pose(pts);
    return cameraPoseFromMarker(p.bestRotation, p.bestTranslation, tagmap, 0);
  }

  console.log(`2) how far the tag decodes, straight on (marker ${TAG_MM} mm, HFOV ${HFOV}):`);
  for (const W of [320, 640]) {
    let far = 0;
    for (let d = 0.6; d <= 5.01; d += 0.1)
      if (phonePose(camera(0, -d, hc, 0, W, (W * 3) / 4), "xz")) far = d;
    console.log(`  INFO ?tagw=${W}: decodes out to ${far.toFixed(1)} m` +
                (W === 320 ? " (phone.html default)" : " (full 640x480 frame)"));
  }

  console.log(`2b) tag pose vs truth (default tagmap=xz):`);
  const cases = [
    // [x, y, yaw, detect width]: y < 0 is in front of the wall
    [0.0, -1.5, 0, 640], [0.6, -2.0, -15, 640], [-0.5, -2.2, 12, 640], [0.3, -1.2, -25, 640],
    [0.0, -0.9, 0, 320], [0.2, -1.0, -10, 320],
  ];
  const got = [];
  for (const [x, y, yaw, W] of cases) {
    const cam = camera(x, y, hc, yaw, W, (W * 3) / 4);
    const p = phonePose(cam, "xz");
    const name = `truth ${x},${y},${yaw} @ ${W} px`;
    if (!p) { ok(name, false, "marker not detected"); continue; }
    const e = Math.hypot(p.x_m - x, p.y_m - y), ey = Math.abs(wrap(p.yaw_deg - yaw));
    const tolM = Math.max(0.15, 0.08 * Math.hypot(x, y)), tolDeg = W >= 640 ? 6 : 10;
    ok(name, e <= tolM && ey <= tolDeg,
       `got ${p.x_m.toFixed(2)},${p.y_m.toFixed(2)},${p.yaw_deg.toFixed(1)}  ` +
       `err ${e.toFixed(2)} m ${ey.toFixed(1)} deg`);
    got.push({ cam, p });
  }

  console.log("2c) POSIT two-solution flips over a 3 x 3 x 3 sweep at 640 px (INFO):");
  {
    let n = 0, flips = 0, altRight = 0;
    for (const x of [-0.6, 0, 0.6])
      for (const d of [1.2, 1.8, 2.4])
        for (const yaw of [-20, 0, 20]) {
          const cam = camera(x, -d, hc, yaw, 640, 480);
          const mk = det.detect(render(cam, code, side, hm)).find((m) => m.id === 0);
          if (!mk) continue;
          const posit = new POS.Posit(TAG_MM, cam.f);
          const p = posit.pose(mk.corners.map((c) => ({ x: c.x - 320, y: 240 - c.y })));
          const best = cameraPoseFromMarker(p.bestRotation, p.bestTranslation, "xz", 0);
          const alt = cameraPoseFromMarker(p.alternativeRotation, p.alternativeTranslation, "xz", 0);
          const bad = (q) => Math.abs(wrap(q.yaw_deg - yaw)) > 8 || Math.abs(q.x_m - x) > 0.25;
          n++;
          if (bad(best)) { flips++; if (!bad(alt)) altRight++; }
        }
    console.log(`  INFO bestRotation is the mirror solution in ${flips} of ${n} poses; ` +
                `alternativeRotation is right in ${altRight} of those`);
  }

  console.log("3) sign of yaw and x (turn right = yaw up, step right = x up):");
  {
    const a = phonePose(camera(0, -2, hc, 0, 640, 480), "xz");
    const b = phonePose(camera(0, -2, hc, 12, 640, 480), "xz");
    const c = phonePose(camera(0.5, -2, hc, 0, 640, 480), "xz");
    ok("turning right raises yaw_deg (clockwise, IF-1)", a && b && b.yaw_deg > a.yaw_deg + 6,
       a && b ? `${a.yaw_deg} -> ${b.yaw_deg}` : "not detected");
    ok("stepping right raises x_m", a && c && c.x_m > a.x_m + 0.3,
       a && c ? `${a.x_m} -> ${c.x_m}` : "not detected");
    ok("standing in front of the tag gives y_m < 0", a && a.y_m < -1.5, a ? `y_m ${a.y_m}` : "");
  }

  console.log("4) person placed by hud_fusion from the phone's bearing (every wall tagmap):");
  const cam = camera(0.4, -2.2, hc, -12, 640, 480);
  const person = { x: 1.2, y: -0.6 }; /* to the right of boresight, in front of the wall */
  for (const tagmap of Object.keys(MAPS)) {
    const p = phonePose(cam, tagmap);
    if (!p) { ok(`tagmap=${tagmap}`, false, "marker not detected"); continue; }
    const s = phoneSees(cam, person.x, person.y);
    const want = MAPS[tagmap](person.x, person.y), at = poseToWorld(p, s.bearing, s.range);
    const e = Math.hypot(at.x - want.x, at.y - want.y);
    ok(`tagmap=${tagmap}: person lands within 0.3 m`, e <= 0.3,
       `bearing ${s.bearing.toFixed(1)} -> ${at.x.toFixed(2)},${at.y.toFixed(2)} ` +
       `want ${want.x.toFixed(2)},${want.y.toFixed(2)} (off ${e.toFixed(2)} m)` +
       (e > 0.3 ? "  MIRRORED frame: bearings turn the wrong way" : ""));
  }

  console.log(fails ? `\n${fails} check(s) FAILED` : "\nALL PASS");
  return fails ? 1 : 0;
}

main().then((c) => process.exit(c), (e) => { console.error(e); process.exit(2); });
