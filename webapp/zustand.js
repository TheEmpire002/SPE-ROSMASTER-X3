// page /zustand: live view of the condition detection
//
// everything comes from /zustand/status; asking it also keeps the pose
// detection on the hailo running, so the page polls even when nothing changes

"use strict";

const el = (id) => document.getElementById(id);

const ui = {
  status: el("status"),
  statusText: el("status-text"),
  alarm: el("zs-alarm"),
  image: el("zs-image"),
  skeleton: el("zs-skeleton"),
  person: el("zs-person"),
  checks: el("zs-checks"),
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
  watch: el("zs-watch"),
  lyingSince: el("zs-lying-since"),
  progressBar: el("zs-progress-bar"),
  watchNote: el("zs-watch-note"),
  hbState: el("zs-hb-state"),
  hbStep: el("zs-hb-step"),
  hbVerdict: el("zs-hb-verdict"),
  benchEmpty: el("zs-bench-empty"),
  bench: el("zs-bench"),
  benchRows: el("zs-bench-rows"),
  benchNote: el("zs-bench-note"),
};

// same keypoint order as the coco model in sensors/pose.py
const SHOULDERS_HIPS = [5, 6, 11, 12];
const STATE_TEXT = {
  bereit: "bereit", anfahrt: "Anfahrt", gesamtbild: "Gesamtbild", ansprechen: "Ansprechen",
  anstupsen: "Anstupsen", nachfrage: "Nachfrage", hilfe: "Hilfe gerufen", ansprechbar: "ansprechbar",
  unauffaellig: "unauffällig", keine_person: "keine Person", abgebrochen: "abgebrochen",
};

let status = null;
let streamCamera = null;
let minConf = 0.3;

function setConnected(yes) {
  ui.status.className = `status ${yes ? "online" : "offline"}`;
  ui.statusText.textContent = yes ? "verbunden" : "getrennt";
}

const fmt = (v, digits = 1, unit = "") =>
  v === null || v === undefined || Number.isNaN(v) ? "–" : `${Number(v).toFixed(digits)}${unit}`;

// ------------------------------------------------------------------ poll

async function poll() {
  try {
    const r = await fetch("/zustand/status", { cache: "no-store" });
    if (!r.ok) throw new Error(r.status);
    status = await r.json();
    setConnected(true);
  } catch {
    setConnected(false);
    return;
  }
  minConf = status.zustand.schwellen.POSE_KP_MIN_CONF ?? 0.3;
  showCamera(status.erkennung);
  showDetection(status.erkennung);
  showCondition(status);
  showWatch(status.wache, status.healthbot);
  showBenchmark(status.modellvergleich);
}

// ------------------------------------------------------------------ display

function showCamera(d) {
  // the pose runs on its own camera; only switch the stream when that changes
  const name = (d.pose && d.pose.kamera) || null;
  if (name === streamCamera && ui.image.src) return;
  streamCamera = name;
  ui.image.src = name ? `/camera/${encodeURIComponent(name)}/stream` : "/camera/stream";
}

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

function signBox(sign, title) {
  const d = document.createElement("div");
  d.className = `hb-check ${sign.zustand}`;
  const light = document.createElement("span");
  light.className = "hb-check-light";
  light.setAttribute("aria-hidden", "true");
  const strong = document.createElement("strong");
  strong.textContent = title;
  const verdict = document.createElement("span");
  verdict.className = "hb-check-verdict";
  verdict.textContent = { auffaellig: "auffällig", ok: "unauffällig" }[sign.zustand] || "unklar";
  const text = document.createElement("p");
  text.textContent = sign.text || "";
  d.append(light, strong, verdict, text);
  return d;
}

function showCondition(s) {
  const z = s.zustand;
  const tracked = s.personen.find((p) => p.verfolgt);
  const n = s.personen.length;
  if (!tracked) {
    ui.person.textContent = n ? "Person gesehen, wird gerade zugeordnet" : "Keine Person im Bild";
    ui.person.className = "zs-big";
  } else {
    const lying = z.liegend_ueber_zeit.zustand === "auffaellig";
    ui.person.textContent = lying ? "Person liegt" : z.liegend_ueber_zeit.zustand === "ok" ? "Person aufrecht" : "Person gesehen";
    ui.person.className = `zs-big ${lying ? "zs-danger" : "zs-ok"}`;
    if (n > 1) ui.person.textContent += ` (${n} Personen im Bild)`;
  }
  ui.checks.replaceChildren(
    signBox(z.liegend, "Liegend – dieses Bild"),
    signBox(z.liegend_ueber_zeit, `Liegend – letzte ${fmt(z.zeitraum_s, 0)} s`),
    signBox(z.bewegung, "Bewegung"),
  );
  ui.angle.textContent = fmt(z.rumpfwinkel, 0, "°");
  ui.frames.textContent = z.bilder ?? "–";
  ui.torso.textContent = tracked
    ? SHOULDERS_HIPS.map((i) => (tracked.kp[i] ? tracked.kp[i][2].toFixed(2) : "–")).join(" · ")
    : "–";
  const t = z.schwellen;
  ui.thresholds.textContent =
    `Liegend unter ${t.HEALTH_LYING_DEG}° Rumpfwinkel oder Umriss breiter als ${t.HEALTH_LYING_ASPECT}×; ` +
    `über die Zeit liegend, wenn mindestens ${Math.round(t.HEALTH_LYING_SHARE * 100)} % der Bilder liegend sind. ` +
    `Punkte unter ${t.POSE_KP_MIN_CONF} Sicherheit zählen nicht.`;
}

function showWatch(w, hb) {
  if (!w) {
    ui.watch.textContent = "–";
  } else {
    ui.watch.textContent = w.an ? "an" : "aus";
    ui.watch.className = w.an ? "zs-ok" : "";
    ui.lyingSince.textContent = w.liegt_seit_s === null ? "–" : `${fmt(w.liegt_seit_s, 1)} s von ${fmt(w.noetig_s, 0)} s`;
    const share = w.liegt_seit_s === null ? 0 : Math.min(1, w.liegt_seit_s / w.noetig_s);
    ui.progressBar.style.width = `${share * 100}%`;
    ui.watchNote.textContent = w.an
      ? `Liegt jemand ${fmt(w.noetig_s, 0)} s lang, startet der Healthbot die Prüfung von selbst.`
      : "Die Wache ist aus: der Roboter startet keine Prüfung von selbst.";
  }
  if (!hb) return;
  ui.hbState.textContent = STATE_TEXT[hb.zustand] || hb.zustand || "–";
  ui.hbState.className = `hb-badge z-${hb.zustand}`;
  ui.hbStep.textContent = hb.schritt || "";
  ui.hbVerdict.textContent = hb.urteil ? `Letzte Bewertung: ${hb.urteil.text}` : "";
  ui.alarm.hidden = !hb.alarm;
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

// ------------------------------------------------------------------ skeleton

function kpColor(c) {
  if (c >= 0.7) return "#3ecf8e";
  if (c >= minConf) return "#f5a524";
  return "rgba(240, 93, 93, 0.75)";
}

function drawPerson(ctx, p, lines, w, h) {
  const [x1, y1, x2, y2] = p.box;
  const lying = p.liegend && p.liegend.zustand === "auffaellig";
  const color = lying ? "#f05d5d" : p.verfolgt ? "#3ecf8e" : "#8a93a3";
  ctx.strokeStyle = color;
  ctx.lineWidth = p.verfolgt ? 3 : 2;
  ctx.setLineDash(p.verfolgt ? [] : [6, 4]);
  ctx.strokeRect(x1 * w, y1 * h, (x2 - x1) * w, (y2 - y1) * h);
  ctx.setLineDash([]);
  const label = `${p.verfolgt ? "beurteilt" : "Person"} ${fmt(p.score, 2)}${lying ? " · liegt" : ""}`;
  ctx.font = "12px system-ui, sans-serif";
  const tw = ctx.measureText(label).width + 8;
  ctx.fillStyle = color;
  ctx.fillRect(x1 * w, Math.max(0, y1 * h - 16), tw, 16);
  ctx.fillStyle = "#0b0d10";
  ctx.fillText(label, x1 * w + 4, Math.max(12, y1 * h - 4));

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
  if (!status || !c.width) return;
  // others first, so the tracked person is drawn on top
  const people = [...status.personen].sort((a, b) => Number(a.verfolgt) - Number(b.verfolgt));
  for (const p of people) drawPerson(ctx, p, status.skelett || [], c.width, c.height);
}

// ------------------------------------------------------------------ timing

poll();
setInterval(poll, 700);
(function paint() {
  drawSkeleton();
  requestAnimationFrame(paint);
})();
