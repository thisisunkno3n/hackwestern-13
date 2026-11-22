// test_phone_link.mjs: unit test for how remote/phone.html comes back after a reload or reconnect.
//
// Run:  node test_phone_link.mjs      (no deps, no network)
//
//   1. Extracts the REAL restoreQuery() from phone.html and runs it against stubbed location,
//      history and sessionStorage: a link re-opened as just ?node=R2 (Safari autocomplete) gets
//      &pose= and the rest back from the last load of the same node, keys in the link win, another
//      node or ?fresh=1 restores nothing, and cal=clear is never replayed.
//   2. Evaluates the real VIDEO_DEFAULT rule: ?video=1 streams at once on R1 and W but not on R2
//      (R2 waits for the relay's video_ctrl), ?video=always forces it on R2.
//   3. node --check on the whole <script type=module> (syntax gate, like the other phone tests).

import fs from "node:fs";
import path from "node:path";
import os from "node:os";
import { fileURLToPath } from "node:url";
import { execFileSync } from "node:child_process";

const HERE = path.dirname(fileURLToPath(import.meta.url));
const html = fs.readFileSync(path.join(HERE, "remote", "phone.html"), "utf8");

let failures = 0;
function check(name, cond) { if (cond) console.log("  ok   " + name); else { console.log("  FAIL " + name); failures++; } }

function extractFunction(src, name) {
  const start = src.indexOf("function " + name);
  if (start < 0) throw new Error("could not find function " + name + " in phone.html");
  let depth = 0;
  for (let i = src.indexOf("{", start); i < src.length; i++) {
    if (src[i] === "{") depth++;
    else if (src[i] === "}" && --depth === 0) return src.slice(start, i + 1);
  }
  throw new Error("unbalanced braces extracting " + name);
}

const qsKey = html.match(/const QS_KEY = "([^"]+)";/);
const restoreSrc = extractFunction(html, "restoreQuery");
// eslint-disable-next-line no-new-func
const makeRestore = new Function("location", "history", "sessionStorage",
  "const QS_KEY = " + JSON.stringify(qsKey ? qsKey[1] : "") + ";\n" + restoreSrc + "\nreturn restoreQuery;");

function tab() {   // one Safari tab: its sessionStorage survives reloads and same-tab navigations
  const store = new Map();
  return { getItem: k => (store.has(k) ? store.get(k) : null), setItem: (k, v) => store.set(k, String(v)) };
}
function load(storage, search) {
  const loc = { pathname: "/phone.html", search, hash: "" };
  const hist = { state: null, replaceState(s, t, url) { loc.replaced = url; } };
  const q = makeRestore(loc, hist, storage)();
  return { q, replaced: loc.replaced };
}

console.log("1) restoreQuery refills a truncated link from this tab's last load of the same node:");
check("QS_KEY found", !!qsKey);
let t = tab();
load(t, "?node=R2&pose=2,3,215&hfov=66");
let r = load(t, "?node=R2");
check("pose restored on ?node=R2", r.q.get("pose") === "2,3,215");
check("hfov restored", r.q.get("hfov") === "66");
check("address bar rewritten with literal commas", r.replaced === "/phone.html?node=R2&pose=2,3,215&hfov=66");
r = load(t, "?node=R2&pose=1,1,90");
check("a key in the link wins over the saved one", r.q.get("pose") === "1,1,90");
r = load(t, "?node=R2");
check("the newer pose is what comes back next time", r.q.get("pose") === "1,1,90");
r = load(t, "?node=R1");
check("another node restores nothing", r.q.get("pose") === null && r.replaced === undefined);
t = tab(); load(t, "?node=R2&pose=2,3,215&cal=clear");
r = load(t, "?node=R2");
check("cal=clear is not replayed", r.q.get("cal") === null && r.q.get("pose") === "2,3,215");
r = load(t, "?node=R2&fresh=1");
check("?fresh=1 restores nothing", r.q.get("pose") === null);
r = load(tab(), "?node=R2");
check("a new tab restores nothing", r.q.get("pose") === null && r.replaced === undefined);
const broken = { getItem: () => "{not json", setItem: () => { throw new Error("quota"); } };
r = load(broken, "?node=R2&pose=2,3,215");
check("corrupt or full sessionStorage never breaks the page", r.q.get("pose") === "2,3,215");

console.log("2) video default: R2 waits for video_ctrl, other nodes honour ?video=1:");
const vd = html.match(/const VIDEO_DEFAULT = ([^;]+);/);
check("VIDEO_DEFAULT found", !!vd);
// eslint-disable-next-line no-new-func
const videoDefault = (node, v) => new Function("NODE_ID", "VIDEO_Q", "return " + vd[1] + ";")(node, v);
check("R1 ?video=1 streams", videoDefault("R1", "1") === true);
check("W ?video=1 streams", videoDefault("W", "1") === true);
check("R2 ?video=1 does not stream until asked", videoDefault("R2", "1") === false);
check("R2 ?video=always streams", videoDefault("R2", "always") === true);
check("no ?video: off", videoDefault("R1", null) === false && videoDefault("R2", null) === false);

console.log("3) the whole module script parses:");
const mod = html.match(/<script type="module">([\s\S]*?)<\/script>/);
const tmp = path.join(fs.mkdtempSync(path.join(os.tmpdir(), "phone-")), "phone.mjs");
fs.writeFileSync(tmp, mod ? mod[1] : "");
let parsed = true;
try { execFileSync(process.execPath, ["--check", tmp], { stdio: "pipe" }); }
catch (e) { parsed = false; console.log(String(e.stderr || e)); }
check("script parses (node --check)", !!mod && parsed);

console.log(failures ? "\n" + failures + " FAILED" : "\nALL PASS");
process.exit(failures ? 1 : 0);
