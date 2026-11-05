// page /healthbot: start a check, follow the procedure and its reasons
//
// two sources: the websocket brings pose and short status at the state rate
// (and takes the commands), /healthbot/status the full details (timeline,
// signs, nudges, skeleton), so the state message stays small for all other
// clients

"use strict";

const el = (id) => document.getElementById(id);

const ui = {
  status: el("status"),
  statusText: el("status-text"),
  alarm: el("hb-alarm"),
  alarmText: el("hb-alarm-text"),
  steps: el("hb-steps"),
  state: el("hb-state"),
  step: el("hb-step"),
  startHere: el("hb-start-here"),
  startGoal: el("hb-start-goal"),
  stop: el("hb-stop"),
  watch: el("hb-watch"),
  nudgeOff: el("hb-nudge-off"),
  map: el("hb-map"),
  goalText: el("hb-goal-text"),
  image: el("hb-image"),
  skeleton: el("hb-skeleton"),
  checks: el("hb-checks"),
  verdict: el("hb-verdict"),
  noNudges: el("hb-no-nudges"),
  nudges: el("hb-nudges"),
  answerForm: el("hb-answer-form"),
  answer: el("hb-answer"),
  sim: el("hb-sim"),
  simText: el("hb-sim-text"),
  simButtons: el("hb-sim-buttons"),
  timeline: el("hb-timeline"),
};

// display text for the procedure states (health/procedure.py)
const STATE_TEXT = {
  bereit: "bereit",
  anfahrt: "Anfahrt",
  gesamtbild: "Gesamtbild",
  ansprechen: "Ansprechen",
  anstupsen: "Anstupsen",
  nachfrage: "Nachfrage",
  hilfe: "Hilfe gerufen",
  ansprechbar: "ansprechbar",
  unauffaellig: "unauffällig",
  keine_person: "keine Person",
  abgebrochen: "abgebrochen",
};
// order of the step display; every final state counts as "end"
const SEQUENCE = ["anfahrt", "gesamtbild", "ansprechen", "anstupsen"];
const FINAL = ["nachfrage", "hilfe", "ansprechbar", "unauffaellig", "keine_person", "abgebrochen"];

const SCENE_TEXT = {
  liegend_regungslos: "Person liegt regungslos",
  liegend_bewegt: "Person liegt, bewegt sich",
};

let socket = null;
let pose = null;          // {x, y, winkel} from the state message, odometry frame
let goal = null;          // place tapped by the user {x, y}
let status = null;        // last answer of /healthbot/status
let scan = [];            // [[angle deg, distance m], ...] relative to the robot
let occupancy = null;     // occupancy map of the mapping as an image, if there is one
let skeletonLines = [];
let lastTimeline = -1;

// ------------------------------------------------------------------ websocket

function connect() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  socket = new WebSocket(`${proto}//${location.host}/ws`);
  socket.onopen = () => setConnected(true);
  socket.onclose = () => {
    setConnected(false);
    setTimeout(connect, 1500);
  };
  socket.onerror = () => {
    // onclose follows right after and reconnects
  };
  socket.onmessage = (ev) => {
    let msg;
    try {
      msg = JSON.parse(ev.data);
    } catch {
      return;
    }
    if (msg.type === "state") {
      if (msg.pose) pose = msg.pose;
      if (msg.healthbot) showShort(msg.healthbot);
    } else if (msg.type === "alert" && msg.level === "emergency") {
      showAlarm(msg.text);
    }
  };
}

function setConnected(yes) {
  ui.status.className = `status ${yes ? "online" : "offline"}`;
  ui.statusText.textContent = yes ? "verbunden" : "getrennt";
}

function send(message) {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(message));
    return true;
  }
  return false;
}

// ------------------------------------------------------------------ controls

ui.startHere.addEventListener("click", () => send({ type: "health_check" }));
ui.startGoal.addEventListener("click", () => {
  if (goal) send({ type: "health_check", x: goal.x, y: goal.y });
});
ui.stop.addEventListener("click", () => send({ type: "health_stop" }));
ui.watch.addEventListener("change", () => send({ type: "health_wache", an: ui.watch.checked }));

ui.answerForm.addEventListener("submit", (ev) => {
  ev.preventDefault();
  const text = ui.answer.value.trim();
  if (text && send({ type: "speech", text })) ui.answer.value = "";
});

// ------------------------------------------------------------------ display

function showShort(hb) {
  // the short status arrives more often than the poll, so step and alarm show at once
  showState(hb.zustand, hb.schritt, hb.aktiv);
  ui.alarm.hidden = !hb.alarm;
  ui.watch.checked = !!hb.wache;
}

function showAlarm(text) {
  ui.alarm.hidden = false;
  ui.alarmText.textContent = text;
}

function showState(state, step, active) {
  ui.state.textContent = STATE_TEXT[state] || state || "–";
  ui.state.className = `hb-badge z-${state}`;
  ui.step.textContent = step || "";
  ui.startHere.disabled = !!active;
  ui.startGoal.disabled = !!active || !goal;
  const now = SEQUENCE.indexOf(state);
  const finished = FINAL.includes(state);
  for (const li of ui.steps.children) {
    const s = li.dataset.state;
    const i = SEQUENCE.indexOf(s);
    li.classList.toggle("current", s === state || (s === "end" && finished));
    li.classList.toggle("done", i >= 0 && (now > i || finished));
  }
}

function showChecks(checks) {
  ui.checks.replaceChildren(
    ...checks.map((c) => {
      const d = document.createElement("div");
      d.className = `hb-check ${c.zustand}`;
      const light = document.createElement("span");
      light.className = "hb-check-light";
      light.setAttribute("aria-hidden", "true");
      const title = document.createElement("strong");
      title.textContent = c.titel;
      const verdict = document.createElement("span");
      verdict.className = "hb-check-verdict";
      verdict.textContent = { auffaellig: "auffällig", ok: "unauffällig" }[c.zustand] || "noch offen";
      const text = document.createElement("p");
      text.textContent = c.text || "";
      d.append(light, title, verdict, text);
      return d;
    }),
  );
}

function showVerdict(u) {
  const level = u ? u.stufe : "unbekannt";
  ui.verdict.className = `hb-verdict ${level}`;
  ui.verdict.querySelector(".hb-verdict-level").textContent =
    { notfall: "NOTFALL", auffaellig: "auffällig", ansprechbar: "ansprechbar", unauffaellig: "unauffällig" }[level] || "–";
  ui.verdict.querySelector(".hb-verdict-text").textContent = u ? u.text : "noch keine Bewertung";
  ui.verdict.querySelector(".hb-points").textContent = u ? `${u.punkte} von 4 Zeichen` : "";
}

function showNudges(nudges) {
  ui.noNudges.hidden = nudges.length > 0;
  ui.nudges.replaceChildren(
    ...nudges.map((s) => {
      const li = document.createElement("li");
      // the reaction is only known once the check after the nudge has finished
      const reaction = s.reaktion === undefined ? "" : s.reaktion ? " · Reaktion!" : " · keine Reaktion";
      li.textContent = `Stups ${s.nr}: ${(s.weg_m * 100).toFixed(0)} cm vor, Ende durch ${s.grund}${reaction}`;
      return li;
    }),
  );
}

function clockTime(t) {
  return new Date(t * 1000).toLocaleTimeString("de-DE");
}

function showTimeline(entries) {
  // rebuild only when something was added, otherwise a text selection jumps while reading
  const key = entries.length ? entries.length + entries[entries.length - 1].zeit : 0;
  if (key === lastTimeline) return;
  lastTimeline = key;
  ui.timeline.replaceChildren(
    ...entries
      .slice()
      .reverse()
      .map((e) => {
        const li = document.createElement("li");
        li.className = `z-${e.zustand}`;
        const time = document.createElement("time");
        time.textContent = clockTime(e.zeit);
        const badge = document.createElement("span");
        badge.className = `hb-badge z-${e.zustand}`;
        badge.textContent = STATE_TEXT[e.zustand] || e.zustand;
        const text = document.createElement("span");
        text.textContent = e.text;
        li.append(time, badge, text);
        return li;
      }),
  );
}

function showSimulation(sim, scenes) {
  ui.sim.hidden = !scenes || scenes.length === 0;
  if (ui.sim.hidden) return;
  ui.simText.textContent = sim && sim.szene
    ? `Jetzt: ${SCENE_TEXT[sim.szene] || sim.szene} bei (${sim.x.toFixed(2)}, ${sim.y.toFixed(2)}).`
    : "Jetzt: keine Person.";
  if (ui.simButtons.children.length) return;
  for (const scene of [...scenes, null]) {
    const b = document.createElement("button");
    b.type = "button";
    b.className = "btn";
    b.textContent = scene ? SCENE_TEXT[scene] || scene : "Person entfernen";
    b.addEventListener("click", () => simulate(scene));
    ui.simButtons.append(b);
  }
}

async function simulate(scene) {
  const place = goal || { x: 1.5, y: 0.0 };
  try {
    await fetch("/healthbot/simulation", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ szene: scene, x: place.x, y: place.y }),
    });
  } catch {
    // the next poll shows whether it worked
  }
  poll();
}

// ------------------------------------------------------------------ polling

async function poll() {
  try {
    const r = await fetch("/healthbot/status", { cache: "no-store" });
    if (!r.ok) return;
    status = await r.json();
  } catch {
    return; // connection briefly gone, next tick tries again
  }
  showState(status.zustand, status.schritt, status.aktiv);
  ui.alarm.hidden = !status.alarm;
  if (status.alarm && status.urteil) ui.alarmText.textContent = status.urteil.text;
  ui.watch.checked = !!status.wache;
  ui.nudgeOff.hidden = status.stupsen_aktiv !== false;
  showChecks(status.checks || []);
  showVerdict(status.urteil);
  showNudges(status.stupser || []);
  showTimeline(status.timeline || []);
  showSimulation(status.simulation, status.szenen);
  drawSkeleton();
}

async function fetchScan() {
  try {
    const r = await fetch("/lidar/scan", { cache: "no-store" });
    if (r.ok) scan = (await r.json()).punkte || [];
  } catch {
    scan = [];
  }
}

async function fetchOccupancy() {
  // the occupancy map only exists while or after mapping ran
  try {
    const r = await fetch("/api/mapping/map", { cache: "no-store" });
    if (!r.ok) {
      occupancy = null;
      return;
    }
    occupancy = occupancyImage(await r.json());
  } catch {
    occupancy = null;
  }
}

function occupancyImage(m) {
  const info = m.info || {};
  const w = info.width;
  const h = info.height;
  if (!w || !h || !m.data_b64) return null;
  const raw = atob(m.data_b64);
  const c = document.createElement("canvas");
  c.width = w;
  c.height = h;
  const ctx = c.getContext("2d");
  const img = ctx.createImageData(w, h);
  for (let i = 0; i < w * h; i++) {
    let v = raw.charCodeAt(i);
    if (v > 127) v -= 256; // int8, -1 = unknown
    // map row 0 is at the bottom (ros) but at the top in the image, so flip
    const row = h - 1 - Math.floor(i / w);
    const p = (row * w + (i % w)) * 4;
    const occupied = v > 50;
    img.data[p] = img.data[p + 1] = img.data[p + 2] = occupied ? 220 : 120;
    img.data[p + 3] = v < 0 ? 0 : occupied ? 200 : 40;
  }
  ctx.putImageData(img, 0, 0);
  const o = (info.origin && info.origin.position) || { x: 0, y: 0 };
  return { image: c, resolution: info.resolution, x0: o.x, y0: o.y, width: w, height: h };
}

// ------------------------------------------------------------------ map

// view large enough for robot, start point and goal
function viewport() {
  const points = [[0, 0]];
  if (pose) points.push([pose.x, pose.y]);
  if (goal) points.push([goal.x, goal.y]);
  const xs = points.map((p) => p[0]);
  const ys = points.map((p) => p[1]);
  const mx = (Math.min(...xs) + Math.max(...xs)) / 2;
  const my = (Math.min(...ys) + Math.max(...ys)) / 2;
  const half = Math.max(2.5, (Math.max(...xs) - Math.min(...xs)) / 2 + 1.5, (Math.max(...ys) - Math.min(...ys)) / 2 + 1.5);
  return { mx, my, half };
}

// world coordinates (x forward, y left) to pixels with x pointing up and y pointing left
function toPixel(v, x, y) {
  const c = ui.map;
  const s = c.width / (2 * v.half);
  return [c.width / 2 - (y - v.my) * s, c.height / 2 - (x - v.mx) * s];
}

function fromPixel(v, px, py) {
  const c = ui.map;
  const s = c.width / (2 * v.half);
  return { x: v.mx + (c.height / 2 - py) / s, y: v.my + (c.width / 2 - px) / s };
}

let currentViewport = viewport();

function drawMap() {
  const c = ui.map;
  const ctx = c.getContext("2d");
  const v = (currentViewport = viewport());
  const s = c.width / (2 * v.half);
  ctx.clearRect(0, 0, c.width, c.height);

  if (occupancy) {
    // the map has its own frame with the mapping start as origin, so it only
    // lines up with the odometry start point if both began at the same place
    ctx.save();
    const [px, py] = toPixel(v, occupancy.x0, occupancy.y0);
    ctx.translate(px, py);
    ctx.rotate(-Math.PI / 2);
    ctx.scale(occupancy.resolution * s, occupancy.resolution * s);
    ctx.drawImage(occupancy.image, 0, -occupancy.height);
    ctx.restore();
  }

  // one meter grid
  ctx.strokeStyle = "#2a2e37";
  ctx.lineWidth = 1;
  for (let m = Math.floor(v.mx - v.half); m <= v.mx + v.half; m++) {
    const [, py] = toPixel(v, m, 0);
    ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(c.width, py); ctx.stroke();
  }
  for (let m = Math.floor(v.my - v.half); m <= v.my + v.half; m++) {
    const [px] = toPixel(v, 0, m);
    ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, c.height); ctx.stroke();
  }

  // lidar points are relative to the robot, so rotate them into world coordinates
  if (pose && scan.length) {
    const th = (pose.winkel * Math.PI) / 180;
    ctx.fillStyle = "#4da3ff";
    for (const [w, d] of scan) {
      const r = th + (w * Math.PI) / 180;
      const [px, py] = toPixel(v, pose.x + d * Math.cos(r), pose.y + d * Math.sin(r));
      ctx.fillRect(px - 1, py - 1, 2, 2);
    }
  }

  // start point as a house
  const [hx, hy] = toPixel(v, 0, 0);
  ctx.fillStyle = "#3ecf8e";
  ctx.beginPath();
  ctx.moveTo(hx, hy - 9); ctx.lineTo(hx + 8, hy); ctx.lineTo(hx + 5, hy); ctx.lineTo(hx + 5, hy + 7);
  ctx.lineTo(hx - 5, hy + 7); ctx.lineTo(hx - 5, hy); ctx.lineTo(hx - 8, hy); ctx.closePath();
  ctx.fill();

  // the tapped goal and the one the healthbot is working with
  const hbGoal = status && status.ziel;
  for (const g of [goal, hbGoal]) {
    if (!g) continue;
    const [gx, gy] = toPixel(v, g.x, g.y);
    ctx.strokeStyle = g === hbGoal ? "#f05d5d" : "#f5a524";
    ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.moveTo(gx - 7, gy - 7); ctx.lineTo(gx + 7, gy + 7);
    ctx.moveTo(gx + 7, gy - 7); ctx.lineTo(gx - 7, gy + 7); ctx.stroke();
    ctx.beginPath(); ctx.arc(gx, gy, 11, 0, 2 * Math.PI); ctx.stroke();
  }

  // robot as a triangle pointing where it looks
  if (pose) {
    const [rx, ry] = toPixel(v, pose.x, pose.y);
    const th = (pose.winkel * Math.PI) / 180;
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(-th);
    ctx.fillStyle = "#e6e8ec";
    ctx.beginPath();
    ctx.moveTo(0, -12); ctx.lineTo(8, 9); ctx.lineTo(-8, 9); ctx.closePath();
    ctx.fill();
    ctx.restore();
  }
}

ui.map.addEventListener("click", (ev) => {
  const r = ui.map.getBoundingClientRect();
  const px = ((ev.clientX - r.left) / r.width) * ui.map.width;
  const py = ((ev.clientY - r.top) / r.height) * ui.map.height;
  const p = fromPixel(currentViewport, px, py);
  goal = { x: Math.round(p.x * 100) / 100, y: Math.round(p.y * 100) / 100 };
  ui.goalText.textContent = `Ziel: x ${goal.x.toFixed(2)} m (vorn), y ${goal.y.toFixed(2)} m (links).`;
  ui.startGoal.disabled = !!(status && status.aktiv);
});

// ------------------------------------------------------------------ skeleton

async function fetchSkeletonLines() {
  // the server sends the keypoint connections along with the pose
  try {
    const r = await fetch("/interaktion/pose", { cache: "no-store" });
    if (r.ok) skeletonLines = (await r.json()).skelett || [];
  } catch {
    // without lines only the points are drawn
  }
}

function drawSkeleton() {
  const c = ui.skeleton;
  const img = ui.image;
  c.width = img.clientWidth;
  c.height = img.clientHeight;
  const ctx = c.getContext("2d");
  ctx.clearRect(0, 0, c.width, c.height);
  const p = status && status.person;
  if (!p || !c.width) return;
  const [x1, y1, x2, y2] = p.box;
  ctx.strokeStyle = "rgba(62, 207, 142, 0.6)";
  ctx.lineWidth = 2;
  ctx.strokeRect(x1 * c.width, y1 * c.height, (x2 - x1) * c.width, (y2 - y1) * c.height);
  const kp = p.kp || [];
  ctx.strokeStyle = "#3ecf8e";
  ctx.lineWidth = 3;
  for (const [a, b] of skeletonLines) {
    const pa = kp[a];
    const pb = kp[b];
    if (!pa || !pb || pa[2] < 0.3 || pb[2] < 0.3) continue;
    ctx.beginPath();
    ctx.moveTo(pa[0] * c.width, pa[1] * c.height);
    ctx.lineTo(pb[0] * c.width, pb[1] * c.height);
    ctx.stroke();
  }
  ctx.fillStyle = "#e6e8ec";
  for (const k of kp) {
    if (k[2] < 0.3) continue;
    ctx.beginPath();
    ctx.arc(k[0] * c.width, k[1] * c.height, 3, 0, 2 * Math.PI);
    ctx.fill();
  }
}

// ------------------------------------------------------------------ timing

connect();
fetchSkeletonLines();
poll();
fetchOccupancy();
setInterval(poll, 700);
setInterval(fetchScan, 500);
setInterval(fetchOccupancy, 5000);
(function paint() {
  drawMap();
  requestAnimationFrame(paint);
})();
