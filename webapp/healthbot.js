// page /healthbot: start a check, follow the procedure and its reasons, and
// watch the live condition detection (formerly its own page /zustand)
//
// sources: the websocket brings pose and short status at the state rate (and
// takes the commands), /healthbot/status the full details of a check
// (timeline, signs, nudges, skeleton), /zustand/status the live view of the
// pose detection (asking it keeps the pose detection on), /api/mapping/ortung
// the pose in the map, its start and the healthbot drive like on /map

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
  startError: el("hb-start-error"),
  stop: el("hb-stop"),
  watch: el("hb-watch"),
  nudgeOff: el("hb-nudge-off"),
  map: el("hb-map"),
  mapTitle: el("hb-map-title"),
  tapMode: el("hb-tap-mode"),
  goalText: el("hb-goal-text"),
  mapChoice: el("hb-map-choice"),
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
  // live condition detection
  live: el("zs-live"),
  person: el("zs-person"),
  liveChecks: el("zs-checks"),
  posture: el("zs-posture"),
  angle: el("zs-angle"),
  frames: el("zs-frames"),
  torso: el("zs-torso"),
  thresholds: el("zs-thresholds"),
  boxes: el("zs-boxes"),
  pose: el("zs-pose"),
  model: el("zs-model"),
  fps: el("zs-fps"),
  age: el("zs-age"),
  last: el("zs-last"),
  camera: el("zs-camera"),
  error: el("zs-error"),
  watchState: el("zs-watch"),
  lyingSince: el("zs-lying-since"),
  progressBar: el("zs-progress-bar"),
  watchNote: el("zs-watch-note"),
  benchEmpty: el("zs-bench-empty"),
  bench: el("zs-bench"),
  benchRows: el("zs-bench-rows"),
  benchNote: el("zs-bench-note"),
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

// body postures (health/assessment.py posture())
const POSTURE_TEXT = {
  liegt: "liegt",
  sitzt: "sitzt aufrecht",
  steht: "steht",
  aufrecht: "aufrecht (Beine nicht im Bild)",
  zusammengesackt: "zusammengesackt",
};

// same keypoint order as the coco model in sensors/pose.py
const SHOULDERS_HIPS = [5, 6, 11, 12];

let socket = null;
let odomPose = null;      // {x, y, winkel} from the state message, odometry frame
let mapBrief = null;      // state.karte: name, pose, sicher of the map in use, or null
let located = null;       // last answer of /api/mapping/ortung, null without map control
let savedMaps = [];       // names of the saved maps
let useMap = true;        // show and check in the map frame when a map is in use
let goal = null;          // place tapped by the user {x, y, rahmen}
let status = null;        // last answer of /healthbot/status
let live = null;          // last answer of /zustand/status
let scan = [];            // [[angle deg, distance m], ...] relative to the robot
let occupancy = null;     // map image from mapview.js occupancyCanvas()
let occupancyKey = null;  // name/live of the map the image belongs to
let occupancyTime = 0;
let skeletonLines = [];
let lastTimeline = -1;
let minConf = 0.3;

const fmt = (v, digits = 1, unit = "") =>
  v === null || v === undefined || Number.isNaN(v) ? "–" : `${Number(v).toFixed(digits)}${unit}`;

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
      if (msg.pose) odomPose = msg.pose;
      mapBrief = msg.karte || null;
      updateFrame();
      if (msg.healthbot) showShort(msg.healthbot);
    } else if (msg.type === "alert" && msg.level === "emergency") {
      showAlarm(msg.text);
    } else if (msg.type === "alert" && msg.level === "warning" && /healthbot|karte|lage/i.test(msg.text || "")) {
      showStartError(msg.text);
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

function showStartError(text) {
  ui.startError.hidden = !text;
  ui.startError.textContent = text || "";
}

async function postJson(path, body) {
  const r = await fetch(path, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify(body),
  });
  const data = await r.json().catch(() => ({}));
  if (!r.ok) throw new Error(data.detail || data.fehler || `HTTP ${r.status}`);
  return data;
}

// ------------------------------------------------------------------ controls

ui.startHere.addEventListener("click", () => {
  showStartError(null);
  send({ type: "health_check" });
});
ui.startGoal.addEventListener("click", async () => {
  if (!goal) return;
  showStartError(null);
  if (goal.rahmen === "karte") {
    // same entry as tapping a person on /map: same checks, same reasons when refused
    try {
      located = await postJson("/api/mapping/person", { x: goal.x, y: goal.y });
    } catch (e) {
      showStartError(`Start abgelehnt: ${e.message}`);
    }
    poll();
    return;
  }
  send({ type: "health_check", x: goal.x, y: goal.y });
});
ui.stop.addEventListener("click", () => send({ type: "health_stop" }));
ui.watch.addEventListener("change", () => send({ type: "health_wache", an: ui.watch.checked }));

ui.answerForm.addEventListener("submit", (ev) => {
  ev.preventDefault();
  const text = ui.answer.value.trim();
  if (text && send({ type: "speech", text })) ui.answer.value = "";
});

// ------------------------------------------------------------------ map choice

function mapState() {
  return (located && located.ortung) || null;
}

function mapLoaded() {
  const o = mapState();
  return !!((o && o.karte) || mapBrief);
}

function frame() {
  return useMap && mapLoaded() ? "karte" : "odom";
}

// name, live, pose and sicher of the map in use; the state message is newer,
// the ortung poll fills in what it lacks (start)
function mapInfo() {
  const o = mapState() || {};
  const b = mapBrief || {};
  return {
    name: b.name !== undefined ? b.name : o.name,
    live: b.live !== undefined ? b.live : o.live,
    pose: b.pose || o.pose || null,
    sicher: b.sicher !== undefined ? b.sicher : o.sicher,
    text: b.text || o.text || "",
    start: o.start || null,
  };
}

function robotPose() {
  return frame() === "karte" ? mapInfo().pose : odomPose;
}

function updateFrame() {
  const f = frame();
  // a goal tapped in the other frame would be a different place
  if (goal && goal.rahmen !== f) {
    goal = null;
    ui.goalText.textContent = "Kein Ziel gesetzt.";
  }
  ui.tapMode.hidden = f !== "karte";
  showMapChoice();
  showMapTitle();
  updateGoalButton();
}

function showMapTitle() {
  if (frame() !== "karte") {
    ui.mapTitle.className = "hb-map-title";
    ui.mapTitle.textContent = mapLoaded()
      ? "Angezeigt: Odometrie ohne Karte (Startpunkt = wo der Server gestartet ist)"
      : "Angezeigt: Odometrie – keine Karte geladen";
    return;
  }
  const m = mapInfo();
  const name = m.name ? `Karte „${m.name}“` : "Live-Karte";
  const where = m.sicher ? "Lage sicher" : "Lage unsicher";
  ui.mapTitle.className = `hb-map-title ${m.sicher ? "sicher" : "unsicher"}`;
  ui.mapTitle.textContent = `Angezeigt: ${name}${m.live && m.name ? " (wächst mit)" : ""} · ${where}`;
  ui.mapTitle.title = m.text || "";
}

function updateGoalButton(active = !!(status && status.aktiv)) {
  const unsure = frame() === "karte" && !mapInfo().sicher;
  ui.startGoal.disabled = active || !goal || unsure;
  ui.startGoal.title = !goal ? "Erst in der Karte einen Ort antippen"
    : unsure ? "Lage in der Karte unsicher – „Hier stehe ich“ wählen und antippen, wo er steht"
      : "";
}

function showMapChoice() {
  const select = ui.mapChoice;
  if (document.activeElement === select) return; // do not jump while the user picks
  const m = mapInfo();
  const options = [["", "Ohne Karte (Odometrie)"]];
  if (mapLoaded() && !m.name) options.push(["*", "Live-Karte"]);
  else if (mapLoaded() && !savedMaps.includes(m.name)) options.push([m.name, `Karte „${m.name}“`]);
  for (const name of savedMaps) options.push([name, `Karte „${name}“`]);
  const wanted = options.map(([v, t]) => `${v}|${t}`).join("/");
  if (select.dataset.options !== wanted) {
    select.replaceChildren(...options.map(([value, text]) => {
      const option = document.createElement("option");
      option.value = value;
      option.textContent = text;
      return option;
    }));
    select.dataset.options = wanted;
  }
  select.value = frame() === "karte" ? m.name || "*" : "";
}

async function fetchSavedMaps() {
  try {
    const r = await fetch("/api/mapping/maps", { cache: "no-store" });
    if (r.ok) savedMaps = (await r.json()).map((m) => m.name);
  } catch {
    // keep the old list, the next call tries again
  }
  showMapChoice();
}

ui.mapChoice.addEventListener("change", async () => {
  const name = ui.mapChoice.value;
  ui.mapChoice.blur();
  if (!name) {
    useMap = false;
  } else if (name === "*" || (mapLoaded() && mapInfo().name === name)) {
    useMap = true;
  } else {
    ui.goalText.textContent = `Lade Karte „${name}“ …`;
    try {
      await postJson("/api/mapping/laden", { name });
      useMap = true;
      ui.goalText.textContent = `Karte „${name}“ geladen. Er sucht seine Lage darin.`;
    } catch (e) {
      ui.goalText.textContent = `Karte laden fehlgeschlagen: ${e.message}`;
    }
    // the state message still names the old map for a moment
    mapBrief = null;
    await fetchLocated();
  }
  updateFrame();
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
  updateGoalButton(!!active);
  const now = SEQUENCE.indexOf(state);
  const finished = FINAL.includes(state);
  for (const li of ui.steps.children) {
    const s = li.dataset.state;
    const i = SEQUENCE.indexOf(s);
    li.classList.toggle("current", s === state || (s === "end" && finished));
    li.classList.toggle("done", i >= 0 && (now > i || finished));
  }
}

function signBox(sign, title, openText = "noch offen") {
  const d = document.createElement("div");
  d.className = `hb-check ${sign.zustand}`;
  const light = document.createElement("span");
  light.className = "hb-check-light";
  light.setAttribute("aria-hidden", "true");
  const strong = document.createElement("strong");
  strong.textContent = title;
  const verdict = document.createElement("span");
  verdict.className = "hb-check-verdict";
  verdict.textContent = { auffaellig: "auffällig", ok: "unauffällig" }[sign.zustand] || openText;
  const text = document.createElement("p");
  text.textContent = sign.text || "";
  d.append(light, strong, verdict, text);
  return d;
}

function showChecks(checks) {
  ui.checks.replaceChildren(...checks.map((c) => signBox(c, c.titel)));
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
  // the simulated room lives in the odometry frame
  const place = goal && goal.rahmen === "odom" ? goal : { x: 1.5, y: 0.0 };
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

// ------------------------------------------------------------------ live condition detection

function detectorText(boxes) {
  if (!boxes) return "–";
  if (!boxes.aktiv) return "aus";
  return { hailo: "Hailo (aus der Pose)", hog: "HOG auf der CPU" }[boxes.verfahren] || "wartet auf Bild";
}

function showDetection(d) {
  const pose = d.pose || {};
  const boxes = d.personenboxen;
  ui.boxes.textContent = detectorText(boxes);
  ui.boxes.className = boxes && boxes.verfahren === "hog" ? "zs-warn" : "";
  ui.pose.textContent = !pose.verfuegbar ? "nicht verfügbar"
    : pose.simuliert ? "simuliert" : pose.aktiv ? "läuft (Hailo)" : "aus";
  ui.model.textContent = d.modell || "–";
  ui.fps.textContent = fmt(pose.fps, 1);
  ui.age.textContent = fmt(pose.bild_alter_s, 2, " s");
  ui.last.textContent = fmt(pose.letzte_erkennung_vor_s, 1, " s");
  ui.camera.textContent = pose.kamera || "–";
  const errors = [pose.fehler, boxes && boxes.fehler].filter(Boolean);
  ui.error.hidden = errors.length === 0;
  ui.error.textContent = errors.join(" · ");
}

function showCondition(s) {
  const z = s.zustand;
  const tracked = s.personen.find((p) => p.verfolgt);
  const n = s.personen.length;
  const posture = z.haltung || (z.bewegung && z.bewegung.haltung) || null;
  if (!tracked) {
    ui.person.textContent = n ? "Person gesehen, wird gerade zugeordnet" : "Keine Person im Bild";
    ui.person.className = "zs-big";
  } else {
    const lying = z.liegend_ueber_zeit.zustand === "auffaellig";
    const slumped = posture === "zusammengesackt";
    ui.person.textContent = lying ? "Person liegt"
      : posture ? `Person ${POSTURE_TEXT[posture] || posture}`
        : z.liegend_ueber_zeit.zustand === "ok" ? "Person aufrecht" : "Person gesehen";
    ui.person.className = `zs-big ${lying ? "zs-danger" : slumped ? "zs-warn" : "zs-ok"}`;
    if (n > 1) ui.person.textContent += ` (${n} Personen im Bild)`;
  }
  ui.liveChecks.replaceChildren(
    signBox(z.liegend, "Liegend – dieses Bild", "unklar"),
    signBox(z.liegend_ueber_zeit, `Liegend – letzte ${fmt(z.zeitraum_s, 0)} s`, "unklar"),
    signBox(z.bewegung, "Bewegung", "unklar"),
  );
  ui.posture.textContent = posture ? POSTURE_TEXT[posture] || posture : "–";
  ui.angle.textContent = fmt(z.rumpfwinkel, 0, "°");
  ui.frames.textContent = z.bilder ?? "–";
  ui.torso.textContent = tracked
    ? SHOULDERS_HIPS.map((i) => (tracked.kp[i] ? tracked.kp[i][2].toFixed(2) : "–")).join(" · ")
    : "–";
  const t = z.schwellen;
  let text =
    `Liegend unter ${t.HEALTH_LYING_DEG}° Rumpfwinkel oder Umriss breiter als ${t.HEALTH_LYING_ASPECT}×; ` +
    `über die Zeit liegend, wenn mindestens ${Math.round(t.HEALTH_LYING_SHARE * 100)} % der Bilder liegend sind. `;
  if (t.HEALTH_UPRIGHT_DEG !== undefined) {
    text += `Ruhig sitzen oder stehen (Rumpf über ${t.HEALTH_UPRIGHT_DEG}°, Kopf über den Schultern) ` +
      "ist kein Warnzeichen; regungslos zählt nur liegend oder zusammengesackt. ";
  }
  ui.thresholds.textContent = text + `Punkte unter ${t.POSE_KP_MIN_CONF} Sicherheit zählen nicht.`;
}

function showWatch(w) {
  if (!w) {
    ui.watchState.textContent = "–";
    return;
  }
  ui.watchState.textContent = w.an ? "an" : "aus";
  ui.watchState.className = w.an ? "zs-ok" : "";
  ui.lyingSince.textContent = w.liegt_seit_s === null ? "–" : `${fmt(w.liegt_seit_s, 1)} s von ${fmt(w.noetig_s, 0)} s`;
  const share = w.liegt_seit_s === null ? 0 : Math.min(1, w.liegt_seit_s / w.noetig_s);
  ui.progressBar.style.width = `${share * 100}%`;
  ui.watchNote.textContent = w.an
    ? `Liegt jemand ${fmt(w.noetig_s, 0)} s lang, startet der Healthbot die Prüfung von selbst.`
    : "Die Wache ist aus: der Roboter startet keine Prüfung von selbst.";
}

function cell(text) {
  const td = document.createElement("td");
  td.textContent = text;
  return td;
}

function showBenchmark(b) {
  const has = b && Array.isArray(b.modelle) && b.modelle.length > 0;
  ui.benchEmpty.hidden = has;
  ui.bench.hidden = !has;
  if (!has) return;
  ui.benchRows.replaceChildren(
    ...b.modelle.map((m) => {
      const tr = document.createElement("tr");
      tr.append(cell(m.modell));
      if (m.fehler) {
        const td = cell(`Fehler: ${m.fehler}`);
        td.colSpan = 6;
        tr.append(td);
        return tr;
      }
      tr.append(
        cell(fmt(m.hailo_ms, 1, " ms")),
        cell(fmt(m.total_ms, 1, " ms")),
        cell(fmt(m.fps_max, 1)),
        cell(fmt(m.cpu_ms_per_frame, 1, " ms")),
        cell(m.people_median ?? "–"),
        cell(fmt(m.torso_conf_mean, 2)),
      );
      return tr;
    }),
  );
  const when = b.zeit ? new Date(b.zeit * 1000).toLocaleString("de-DE") : "unbekannt";
  const frames = b.modelle.find((m) => m.bilder)?.bilder;
  ui.benchNote.textContent =
    `Gemessen am ${when}${frames ? ` mit ${frames} Bildern je Modell` : ""} (Mittelwert/Median), Bild: ${b.bild || "–"}. ` +
    "Rumpf-Sicherheit: mittlere Sicherheit von Schultern und Hüften – höher heißt verlässlicher für „liegend“.";
}

function showLiveOff() {
  live = null;
  ui.person.textContent = "Live-Erkennung aus";
  ui.person.className = "zs-big";
  ui.liveChecks.replaceChildren();
  for (const k of ["posture", "angle", "frames", "torso"]) ui[k].textContent = "–";
}

async function pollLive() {
  if (!ui.live.checked) return;
  try {
    const r = await fetch("/zustand/status", { cache: "no-store" });
    if (!r.ok) return;
    live = await r.json();
  } catch {
    return;
  }
  if (!ui.live.checked) return; // switched off while waiting
  minConf = live.zustand.schwellen.POSE_KP_MIN_CONF ?? 0.3;
  showDetection(live.erkennung);
  showCondition(live);
  showWatch(live.wache);
  showBenchmark(live.modellvergleich);
}

ui.live.checked = localStorage.getItem("hb-live") !== "aus";
ui.live.addEventListener("change", () => {
  localStorage.setItem("hb-live", ui.live.checked ? "an" : "aus");
  if (ui.live.checked) pollLive();
  else showLiveOff();
});

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
  updateGoalButton();
  ui.alarm.hidden = !status.alarm;
  if (status.alarm && status.urteil) ui.alarmText.textContent = status.urteil.text;
  ui.watch.checked = !!status.wache;
  ui.nudgeOff.hidden = status.stupsen_aktiv !== false;
  showChecks(status.checks || []);
  showVerdict(status.urteil);
  showNudges(status.stupser || []);
  showTimeline(status.timeline || []);
  showSimulation(status.simulation, status.szenen);
  showCamera();
}

// the keypoints belong to the pose camera; drawn over another camera they
// would sit beside the person
let shownCamera;
function showCamera() {
  const fromLive = live && live.erkennung && live.erkennung.pose && live.erkennung.pose.kamera;
  const name = (status && status.kamera) || fromLive || null;
  if (name === shownCamera && ui.image.src) return;
  shownCamera = name;
  ui.image.src = name ? `/camera/${encodeURIComponent(name)}/stream` : "/camera/stream";
}

async function fetchScan() {
  try {
    const r = await fetch("/lidar/scan", { cache: "no-store" });
    if (r.ok) scan = (await r.json()).punkte || [];
  } catch {
    scan = [];
  }
}

async function fetchLocated() {
  // without map control (e.g. no mapping module) there is only odometry
  try {
    const r = await fetch("/api/mapping/ortung", { cache: "no-store" });
    located = r.ok ? await r.json() : null;
  } catch {
    return;
  }
  updateFrame();
  fetchOccupancyIfNeeded();
}

async function fetchOccupancyIfNeeded() {
  const o = mapState();
  if (!o || !o.karte) {
    occupancy = null;
    occupancyKey = null;
    return;
  }
  const key = `${o.name || "*"}|${o.live}`;
  // a saved map does not change; a growing one is fetched again every few seconds
  if (key === occupancyKey && !(o.live && Date.now() - occupancyTime > 5000)) return;
  occupancyTime = Date.now();
  try {
    const r = await fetch("/api/mapping/map", { cache: "no-store" });
    if (!r.ok) return;
    const image = occupancyCanvas(await r.json());
    if (image) {
      occupancy = image;
      occupancyKey = key;
    }
  } catch {
    // the next round tries again
  }
}

// ------------------------------------------------------------------ map

const view = new MapView(480, 480);

// canvas in css px, sharp on high dpi screens
function prepareCanvas() {
  const c = ui.map;
  const width = c.clientWidth || 480;
  const height = c.clientHeight || width;
  const dpr = Math.max(1, window.devicePixelRatio || 1);
  if (c.width !== Math.round(width * dpr) || c.height !== Math.round(height * dpr)) {
    c.width = Math.round(width * dpr);
    c.height = Math.round(height * dpr);
  }
  view.width = width;
  view.height = height;
  const ctx = c.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return ctx;
}

// the healthbot's own target, if it lies in the frame shown
function healthbotTarget() {
  if (frame() === "karte") {
    const h = (located && located.healthbot) || {};
    return h.aktiv && h.ziel ? { x: h.ziel[0], y: h.ziel[1], radius: h.radius_m, fahrt: h.fahrt } : null;
  }
  const z = status && status.aktiv && status.ziel;
  return z && (z.rahmen || "odom") === "odom" ? { x: z.x, y: z.y } : null;
}

function fitOdometry(pose, target) {
  const points = [[0, 0]];
  if (pose) points.push([pose.x, pose.y]);
  if (goal) points.push([goal.x, goal.y]);
  if (target) points.push([target.x, target.y]);
  const xs = points.map((p) => p[0]);
  const ys = points.map((p) => p[1]);
  const half = Math.max(2.5, (Math.max(...xs) - Math.min(...xs)) / 2 + 1.5,
    (Math.max(...ys) - Math.min(...ys)) / 2 + 1.5);
  const mx = (Math.min(...xs) + Math.max(...xs)) / 2;
  const my = (Math.min(...ys) + Math.max(...ys)) / 2;
  view.fit(mx - half, my - half, mx + half, my + half, 1.0);
}

function drawCross(ctx, x, y, color) {
  const [gx, gy] = view.toCanvas(x, y);
  ctx.strokeStyle = color;
  ctx.lineWidth = 2.5;
  ctx.beginPath();
  ctx.moveTo(gx - 7, gy - 7); ctx.lineTo(gx + 7, gy + 7);
  ctx.moveTo(gx + 7, gy - 7); ctx.lineTo(gx - 7, gy + 7);
  ctx.stroke();
  ctx.beginPath();
  ctx.arc(gx, gy, 11, 0, 2 * Math.PI);
  ctx.stroke();
}

function drawMap() {
  const ctx = prepareCanvas();
  ctx.clearRect(0, 0, view.width, view.height);
  const byMap = frame() === "karte";
  const pose = robotPose();
  const target = healthbotTarget();

  if (byMap && occupancy) {
    view.fitMap(occupancy);
    view.drawMap(ctx, occupancy);
  } else {
    // the map has its own frame, so without the map frame there is only the grid
    fitOdometry(pose, target);
    view.drawGrid(ctx);
  }

  // lidar points are relative to the robot, so rotate them into the frame shown
  if (pose && scan.length) {
    const th = (pose.winkel * Math.PI) / 180;
    ctx.fillStyle = "#4da3ff";
    for (const [w, d] of scan) {
      const r = th + (w * Math.PI) / 180;
      const [px, py] = view.toCanvas(pose.x + d * Math.cos(r), pose.y + d * Math.sin(r));
      ctx.fillRect(px - 1, py - 1, 2, 2);
    }
  }

  // starting point: the map's start (like on /map) or the odometry origin
  const start = byMap ? mapInfo().start : [0, 0];
  if (start) {
    const [sx, sy] = view.toCanvas(start[0], start[1]);
    ctx.strokeStyle = "#c792ea";
    ctx.lineWidth = 2;
    ctx.strokeRect(sx - 6, sy - 6, 12, 12);
  }

  // area the healthbot searches and its drive there, like on /map
  if (target) {
    if (target.radius) {
      const [hx, hy] = view.toCanvas(target.x, target.y);
      ctx.strokeStyle = "#ff6b8a";
      ctx.fillStyle = "#ff6b8a22";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(hx, hy, view.meters(target.radius), 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();
    }
    const drive = target.fahrt || {};
    if (pose && drive.aktiv && drive.weg && drive.weg.length) {
      view.drawLine(ctx, [[pose.x, pose.y], ...drive.weg], "#ff6b8a", 2.5, [7, 5]);
    }
    drawCross(ctx, target.x, target.y, "#f05d5d");
  }
  if (goal) drawCross(ctx, goal.x, goal.y, "#f5a524");

  if (pose) {
    const sure = !byMap || mapInfo().sicher;
    view.drawRobot(ctx, pose, sure ? "#25b879" : "#f0a020");
  }
  if (byMap && !occupancy) {
    ctx.fillStyle = "#e6e8ec";
    ctx.font = "14px system-ui, sans-serif";
    ctx.fillText("Karte wird geladen …", 12, 24);
  }
}

function tapMode() {
  const checked = document.querySelector('input[name="hb-tap"]:checked');
  return checked ? checked.value : "ziel";
}

ui.map.addEventListener("click", async (ev) => {
  const r = ui.map.getBoundingClientRect();
  const [x, y] = view.toWorld(ev.clientX - r.left, ev.clientY - r.top);
  const f = frame();
  if (f === "karte" && tapMode() === "pose") {
    // tell it where it stands, like "Hier stehe ich" on /map
    ui.goalText.textContent = "Er sucht seine Lage um den angetippten Punkt …";
    try {
      located = await postJson("/api/mapping/pose", { x, y });
      ui.goalText.textContent = "Lage gesetzt. Er gleicht sie jetzt mit dem Lidar ab.";
    } catch (e) {
      ui.goalText.textContent = `Lage setzen fehlgeschlagen: ${e.message}`;
    }
    document.querySelector('input[name="hb-tap"][value="ziel"]').checked = true;
    updateFrame();
    return;
  }
  goal = { x: Math.round(x * 100) / 100, y: Math.round(y * 100) / 100, rahmen: f };
  ui.goalText.textContent = f === "karte"
    ? `Ziel in der Karte: x ${goal.x.toFixed(2)} m, y ${goal.y.toFixed(2)} m – er sucht die Person im Umkreis.`
    : `Ziel: x ${goal.x.toFixed(2)} m (vorn), y ${goal.y.toFixed(2)} m (links).`;
  showStartError(null);
  updateGoalButton();
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

function kpColor(c) {
  if (c >= 0.7) return "#3ecf8e";
  if (c >= minConf) return "#f5a524";
  return "rgba(240, 93, 93, 0.75)";
}

function drawPerson(ctx, p, lines, w, h, label) {
  const [x1, y1, x2, y2] = p.box;
  const lying = p.liegend && p.liegend.zustand === "auffaellig";
  const color = lying ? "#f05d5d" : p.verfolgt ? "#3ecf8e" : "#8a93a3";
  ctx.strokeStyle = color;
  ctx.lineWidth = p.verfolgt ? 3 : 2;
  ctx.setLineDash(p.verfolgt ? [] : [6, 4]);
  ctx.strokeRect(x1 * w, y1 * h, (x2 - x1) * w, (y2 - y1) * h);
  ctx.setLineDash([]);
  const text = label || `${p.verfolgt ? "beurteilt" : "Person"} ${fmt(p.score, 2)}${lying ? " · liegt" : ""}`;
  ctx.font = "12px system-ui, sans-serif";
  const tw = ctx.measureText(text).width + 8;
  ctx.fillStyle = color;
  ctx.fillRect(x1 * w, Math.max(0, y1 * h - 16), tw, 16);
  ctx.fillStyle = "#0b0d10";
  ctx.fillText(text, x1 * w + 4, Math.max(12, y1 * h - 4));

  const kp = p.kp || [];
  ctx.strokeStyle = p.verfolgt ? "rgba(62, 207, 142, 0.85)" : "rgba(180, 186, 196, 0.7)";
  ctx.lineWidth = p.verfolgt ? 3 : 2;
  for (const [a, b] of lines) {
    const pa = kp[a];
    const pb = kp[b];
    if (!pa || !pb || pa[2] < minConf || pb[2] < minConf) continue;
    ctx.beginPath();
    ctx.moveTo(pa[0] * w, pa[1] * h);
    ctx.lineTo(pb[0] * w, pb[1] * h);
    ctx.stroke();
  }
  // unsure points stay visible in red so you can see what the model missed
  for (const k of kp) {
    ctx.fillStyle = kpColor(k[2]);
    ctx.beginPath();
    ctx.arc(k[0] * w, k[1] * h, k[2] >= minConf ? 4 : 3, 0, 2 * Math.PI);
    ctx.fill();
  }
}

function drawSkeleton() {
  const c = ui.skeleton;
  const img = ui.image;
  c.width = img.clientWidth;
  c.height = img.clientHeight;
  const ctx = c.getContext("2d");
  ctx.clearRect(0, 0, c.width, c.height);
  if (!c.width) return;
  const lines = (live && live.skelett) || skeletonLines;
  const checking = status && status.aktiv && status.person;
  if (live) {
    // others first, so the tracked person is drawn on top; during a check the
    // healthbot's own person is drawn below as the judged one
    const people = [...live.personen].sort((a, b) => Number(a.verfolgt) - Number(b.verfolgt));
    for (const p of people) drawPerson(ctx, checking ? { ...p, verfolgt: false } : p, lines, c.width, c.height);
  }
  if (checking) {
    drawPerson(ctx, { ...status.person, verfolgt: true }, lines, c.width, c.height, "Healthbot prüft");
  }
}

// ------------------------------------------------------------------ timing

connect();
fetchSkeletonLines();
poll();
pollLive();
fetchLocated();
fetchSavedMaps();
if (!ui.live.checked) showLiveOff();
setInterval(fetchSavedMaps, 15000);
setInterval(poll, 700);
setInterval(pollLive, 700);
setInterval(fetchLocated, 1000);
setInterval(fetchScan, 500);
(function paint() {
  drawMap();
  drawSkeleton();
  requestAnimationFrame(paint);
})();
