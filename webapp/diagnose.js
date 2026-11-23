/*
 * Diagnostics page (/diagnose, also /logs).
 *
 * Fetches GET /diagnose/status (problems, measurements, timeline) and
 * GET /diagnose/protokoll (only new log lines) every few seconds. The whole
 * evaluation -- what counts as a problem, how bad, what to do -- happens on
 * the robot (diagnostics.py). The page only displays; that way all
 * interfaces show the same thing.
 *
 * Texts from the server always go into the page via textContent, never as
 * HTML: the log can contain anything.
 */

(() => {

const INTERVAL_MS = 3000;
const LOG_MAX = 1000;
const LEVELS = { DEBUG: 0, INFO: 1, WARNING: 2, ERROR: 3, CRITICAL: 4 };
const SEVERITY_TEXT = { kritisch: "Kritisch", warnung: "Warnung", hinweis: "Hinweis", info: "Info" };

const el = (id) => document.getElementById(id);

let paused = false;
let lastLogId = 0;
let logLines = [];
let lastStatus = null;
let timer = null;

// --- small helpers ---------------------------------------------------------------

function make(tag, cls, text) {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  if (text !== undefined && text !== null) e.textContent = text;
  return e;
}

function clockTime(sec) {
  return new Date(sec * 1000).toLocaleTimeString("de-DE");
}

/** "vor 12 s", "vor 3 min" -- people do not think in Unix time. */
function ago(sec) {
  if (sec < 5) return "gerade eben";
  if (sec < 60) return `vor ${Math.round(sec)} s`;
  if (sec < 3600) return `vor ${Math.round(sec / 60)} min`;
  return `vor ${(sec / 3600).toFixed(1)} h`;
}

function duration(sec) {
  if (sec === null || sec === undefined) return "–";
  const h = Math.floor(sec / 3600);
  const m = Math.floor((sec % 3600) / 60);
  if (h > 0) return `${h} h ${m} min`;
  if (m > 0) return `${m} min ${Math.floor(sec % 60)} s`;
  return `${Math.floor(sec)} s`;
}

function showMessage(text, isError = false) {
  const m = el("diag-meldung");
  m.textContent = text;
  m.classList.toggle("fehler", isError);
  m.hidden = false;
  clearTimeout(showMessage.timer);
  showMessage.timer = setTimeout(() => { m.hidden = true; }, isError ? 8000 : 3000);
}

async function fetchJson(path) {
  const response = await fetch(path, { cache: "no-store" });
  if (!response.ok) throw new Error(`HTTP ${response.status}`);
  return response.json();
}

async function acknowledge(data) {
  try {
    const response = await fetch("/diagnose/quittieren", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(data),
    });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
  } catch {
    showMessage("Roboter nicht erreichbar", true);
    return;
  }
  refresh();
}

// --- Overall state ---------------------------------------------------------------

function showOverall(st) {
  const box = el("diag-gesamt");
  box.className = `panel wide-panel diag-gesamt ${st.gesamt.stufe}`;
  const title = { ok: "Alles in Ordnung", warnung: "Achtung", kritisch: "Problem!" }[st.gesamt.stufe];
  el("diag-gesamt-titel").textContent = title || st.gesamt.stufe;

  const open = st.probleme.filter((p) => !p.quittiert);
  let subtitle = st.gesamt.text;
  if (open.length) subtitle += ` ${open.length} offene Meldung${open.length === 1 ? "" : "en"}.`;
  if (st.simulation) subtitle += " (Simulation -- Werte sind ausgedacht.)";
  el("diag-gesamt-unter").textContent = subtitle;
  el("diag-alle-quittieren").hidden = open.length === 0;
}

// --- Problems --------------------------------------------------------------------

function problemCard(p) {
  const card = make("article", `diag-problem ${p.schwere}${p.quittiert ? " gesehen" : ""}`);

  const head = make("div", "diag-problem-kopf");
  head.append(make("span", `diag-schwere ${p.schwere}`, SEVERITY_TEXT[p.schwere] || p.schwere));
  head.append(make("strong", "diag-problem-titel", p.titel));
  card.append(head);

  // When and how often -- that distinguishes "happened once" from "constantly".
  let when;
  if (p.besteht) {
    when = `besteht seit ${clockTime(p.seit)}`;
  } else if (p.anzahl > 1) {
    when = `${p.anzahl}× seit ${clockTime(p.seit)}, zuletzt ${ago(p.vor_s)} (${clockTime(p.zuletzt)})`;
  } else {
    when = `${ago(p.vor_s)} (${clockTime(p.zuletzt)})`;
  }
  if (p.besteht && p.anzahl > 1) when += ` · ${p.anzahl}× aufgetreten`;
  if (p.zustand && !p.besteht) when += " · jetzt wieder in Ordnung";
  card.append(make("div", "diag-problem-wann", when));

  card.append(make("p", "diag-problem-was", p.was));

  if (p.tun && p.tun.length) {
    const todo = make("div", "diag-problem-tun");
    todo.append(make("span", "diag-tun-titel", "Was tun?"));
    const list = make("ol");
    for (const step of p.tun) list.append(make("li", null, step));
    todo.append(list);
    card.append(todo);
  }

  if (!p.quittiert) {
    const button = make("button", "btn diag-quittieren", "Gesehen");
    button.type = "button";
    button.title = "Ausblenden, bis es erneut auftritt";
    button.addEventListener("click", () => acknowledge({ id: p.id }));
    card.append(button);
  }
  return card;
}

function showProblems(st) {
  const open = st.probleme.filter((p) => !p.quittiert);
  const seen = st.probleme.filter((p) => p.quittiert);

  el("diag-probleme").replaceChildren(...open.map(problemCard));
  el("diag-keine-probleme").hidden = open.length > 0;

  el("diag-gesehen").hidden = seen.length === 0;
  el("diag-gesehen-anzahl").textContent = seen.length;
  el("diag-probleme-gesehen").replaceChildren(...seen.map(problemCard));
}

// --- Measurements and battery curve ------------------------------------------------

function showMeasurements(st) {
  const m = st.messwerte;
  const s = st.schwellen;

  const battery = el("diag-akku-v");
  if (m.akku_v === null || m.akku_v === undefined) {
    battery.textContent = "–";
    battery.className = "";
    el("diag-akku-prozent").textContent = "Akku nicht messbar";
  } else {
    battery.textContent = `${m.akku_v.toFixed(2)} V`;
    battery.className = m.akku_v < s.akku_kritisch_v ? "kritisch" : m.akku_v < s.akku_warn_v ? "warnung" : "ok";
    el("diag-akku-prozent").textContent = `ca. ${m.akku_prozent} % (grob geschätzt)`;
  }
  drawCurve(st.akku_verlauf, s);

  const rows = [];
  const row = (name, value, level, help) => rows.push({ name, value, level, help });

  if (m.temp_c !== null && m.temp_c !== undefined) {
    row("Temperatur Pi", `${m.temp_c.toFixed(1)} °C`,
      m.temp_c >= s.temp_warn_c ? "warnung" : "ok", `Warnung ab ${s.temp_warn_c} °C`);
  }
  if (m.cpu_max_mhz) {
    const current = m.cpu_ist_mhz ? `${m.cpu_ist_mhz} / ` : "";
    row("CPU-Takt (jetzt / max)", `${current}${m.cpu_max_mhz} MHz`,
      m.cpu_max_mhz > s.cpu_max_mhz ? "warnung" : "ok", `Höchsttakt sollte ≤ ${s.cpu_max_mhz} MHz sein`);
  }
  if (m.last_1min !== null && m.last_1min !== undefined) {
    const cores = m.kerne || 4;
    row("Auslastung (1 min)", `${m.last_1min} von ${cores} Kernen`,
      m.last_1min > cores ? "warnung" : "ok", "Über der Kernzahl wartet Arbeit");
  }
  if (m.usb_ueberlast_gesamt !== null && m.usb_ueberlast_gesamt !== undefined) {
    const since = m.usb_ueberlast_seit_serverstart ?? 0;
    row("USB-Überlast", `${since} seit Serverstart · ${m.usb_ueberlast_gesamt} seit Booten`,
      since > 0 ? "warnung" : "ok", "Wie oft der Pi die USB-Buchsen abschalten musste");
  }
  if (m.drosselung) {
    const d = m.drosselung;
    let text = d.jetzt.length ? `jetzt: ${d.jetzt.join(", ")}` : "jetzt: nichts";
    if (d.seit_start.length) text += ` · seit Start: ${d.seit_start.join(", ")}`;
    row("Spannung/Drosselung Pi", text,
      d.jetzt.includes("Unterspannung") ? "kritisch" : d.jetzt.length || d.seit_start.length ? "warnung" : "ok",
      `vcgencmd get_throttled = ${d.roh}`);
  }
  if (m.bau_prozesse && m.bau_prozesse.length) {
    row("Installation läuft", m.bau_prozesse.join(", "), "warnung", "Erzeugt Last und Wärme");
  }
  row("Server läuft seit", duration(m.server_laufzeit_s), "", "");
  if (m.pi_laufzeit_s) row("Pi eingeschaltet seit", duration(m.pi_laufzeit_s), "", "");

  const dl = el("diag-messwerte");
  dl.replaceChildren();
  for (const r of rows) {
    const dt = make("dt", null, r.name);
    const dd = make("dd", r.level, r.value);
    if (r.help) { dt.title = r.help; dd.title = r.help; }
    dl.append(dt, dd);
  }
}

function drawCurve(points, s) {
  const canvas = el("diag-akku-kurve");
  // Sharp on high-DPI screens: match the pixels to the real width.
  const width = canvas.clientWidth || 320;
  const height = canvas.clientHeight || 90;
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(width * dpr)) {
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);

  if (!points || points.length < 2) {
    ctx.fillStyle = "#9aa1ad";
    ctx.font = "12px system-ui";
    ctx.fillText(points && points.length ? "sammle Messwerte …" : "keine Akkuwerte", 8, height / 2);
    return;
  }

  const values = points.map((p) => p[1]);
  const lo = Math.min(...values, s.akku_kritisch_v) - 0.2;
  const hi = Math.max(...values, 12.6) + 0.1;
  const t0 = points[0][0];
  const t1 = points[points.length - 1][0];
  const x = (t) => ((t - t0) / Math.max(1, t1 - t0)) * (width - 4) + 2;
  const y = (v) => height - 4 - ((v - lo) / (hi - lo)) * (height - 8);

  // Thresholds as dashed lines -- shows how much headroom is left.
  ctx.setLineDash([4, 4]);
  ctx.lineWidth = 1;
  for (const [v, color] of [[s.akku_warn_v, "#f5a524"], [s.akku_kritisch_v, "#f05d5d"]]) {
    ctx.strokeStyle = color;
    ctx.beginPath();
    ctx.moveTo(0, y(v));
    ctx.lineTo(width, y(v));
    ctx.stroke();
  }
  ctx.setLineDash([]);

  ctx.strokeStyle = "#4da3ff";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  points.forEach(([t, v], i) => (i ? ctx.lineTo(x(t), y(v)) : ctx.moveTo(x(t), y(v))));
  ctx.stroke();

  ctx.fillStyle = "#9aa1ad";
  ctx.font = "11px system-ui";
  ctx.fillText(`${Math.max(...values).toFixed(1)} V`, 4, 12);
  ctx.fillText(`letzte ${duration(t1 - t0)}`, width - 90, height - 6);
}

// --- Devices ---------------------------------------------------------------------

function showDevices(st) {
  const ul = el("diag-geraete");
  ul.replaceChildren();
  for (const g of st.geraete) {
    const level = g.ok === true ? "ok" : g.ok === false ? "fehler" : "ruht";
    const li = make("li", level);
    li.append(make("span", "diag-punkt-ampel"));
    li.append(make("span", "diag-geraet-name", g.name));
    let text = g.text;
    if (g.neu) text += ` · ${g.neu}× neu verbunden`;
    li.append(make("span", "diag-geraet-text", text));
    ul.append(li);
  }

  const q = st.quellen;
  const missing = [];
  if (q.kernel_log !== "ok") missing.push(`Kernel-Log (${q.kernel_log})`);
  if (!q.usb_zaehler) missing.push("USB-Zähler");
  if (!q.vcgencmd) missing.push("vcgencmd");
  if (!q.akku) missing.push("Akkuspannung");
  el("diag-quellen").textContent = missing.length
    ? `Nicht verfügbar: ${missing.join(", ")}. Diese Werte fehlen auf der Seite.`
    : "Alle Messquellen verfügbar.";
}

// --- Crashes & hangs -------------------------------------------------------------

const END_TEXT = {
  beendet: "sauber beendet",
};

function showCrash(st) {
  const a = st.absturz;
  if (!a) return;

  const v = a.vorher;
  el("diag-vorher").hidden = !v;
  if (v) {
    el("diag-vorher").className = `diag-vorher ${v.art === "prozess_weg" ? "warnung" : "kritisch"}`;
    el("diag-vorher-titel").textContent = `Voriger Lauf: ${v.titel}`;
    el("diag-vorher-was").textContent = v.was;
    const parts = [];
    if (v.kernel && v.kernel.length) parts.push("Kernel-Log:", ...v.kernel, "");
    if (v.log && v.log.length) parts.push("Letzte Log-Zeilen:", ...v.log, "");
    if (v.dump && v.dump.length) parts.push("Stack-Dump:", ...v.dump);
    el("diag-vorher-text").textContent = parts.join("\n") || "Nichts aufgezeichnet.";
  }

  const rows = [];
  const row = (name, value, level = "", help = "") => rows.push({ name, value, level, help });
  row("Erfassung", a.aktiv ? "aktiv" : "aus (Ordner nicht beschreibbar)", a.aktiv ? "ok" : "warnung");
  row("Journal überlebt Neustart", a.journal_dauerhaft ? "ja" : "nein", a.journal_dauerhaft ? "ok" : "warnung",
    "Ohne dauerhaftes Journal ist das Kernel-Log nach einem Neustart weg");
  if (a.puls_alter_s !== null && a.puls_alter_s !== undefined) {
    row("Herzschlag des Servers", `vor ${a.puls_alter_s.toFixed(1)} s`, a.puls_alter_s > 2 ? "kritisch" : "ok",
      "Kommt keiner, hängt die Ereignisschleife");
  }
  row("Längste Verzögerung", `${a.puls_max_s.toFixed(2)} s`, a.puls_max_s > 1 ? "warnung" : "ok",
    "Wie lange die Ereignisschleife höchstens blockiert war");
  row("Hänger seit Start", String(a.haenger), a.haenger ? "warnung" : "ok");
  const sp = a.speicher || {};
  if (sp.server_mb !== undefined) row("Speicher Server", `${sp.server_mb} MB, ${sp.threads} Threads, ${sp.dateien} Dateien`);
  if (sp.frei_mb !== undefined) row("Speicher frei", `${sp.frei_mb} von ${sp.gesamt_mb} MB`, sp.frei_mb < 300 ? "kritisch" : "ok");
  if (a.usb_pro_min !== null && a.usb_pro_min !== undefined) {
    row("USB-Überlast", `${a.usb_pro_min} / min`, a.usb_pro_min >= 20 ? "kritisch" : a.usb_pro_min ? "warnung" : "ok",
      "Jede Meldung trennt kurz alle USB-Geräte");
  }
  const dl = el("diag-absturz-werte");
  dl.replaceChildren();
  for (const r of rows) {
    const dt = make("dt", null, r.name);
    const dd = make("dd", r.level, r.value);
    if (r.help) { dt.title = r.help; dd.title = r.help; }
    dl.append(dt, dd);
  }

  el("diag-blockiert").hidden = !a.blockiert;
  if (a.blockiert) {
    el("diag-blockiert-text").textContent =
      `${clockTime(a.blockiert.zeit)} -- stand ${a.blockiert.dauer_s} s:\n${a.blockiert.stapel}`;
  }

  const ul = el("diag-laeufe");
  ul.replaceChildren();
  for (const l of a.laeufe) {
    const alive = !l.ende && l.pid && l === a.laeufe[0];
    const level = alive ? "ok" : !l.ende ? "kritisch" : l.ende === "beendet" ? "" : "warnung";
    const end = alive ? "läuft (dieser Lauf)" : l.ende ? (END_TEXT[l.ende] || l.ende) : "abgestürzt (kein Ende vermerkt)";
    const li = make("li", level);
    li.append(make("time", null, new Date(l.start * 1000).toLocaleString("de-DE")));
    li.append(make("span", null, `${duration(Math.max(0, l.zuletzt - l.start))} · ${end}`));
    ul.append(li);
  }
  if (!a.laeufe.length) ul.append(make("li", null, "Noch keine Läufe aufgezeichnet."));

  const pu = el("diag-usb-ports");
  pu.replaceChildren();
  const ports = Object.entries(a.usb_ports || {});
  for (const [port, n] of ports) {
    const newN = (a.usb_ports_neu || {})[port] || 0;
    pu.append(make("li", newN ? "kritisch" : n ? "warnung" : "ok", `${port}: ${n}${newN ? ` (+${newN} seit Serverstart)` : ""}`));
  }
  if (!ports.length) pu.append(make("li", null, "Keine Zähler lesbar (nur am Pi)."));

  el("diag-absturz-ordner").textContent = a.ordner
    ? `Dateien nach einem Absturz: ${a.ordner} (server.log, laeufe.json, absturz.txt, blockiert.txt)`
    : "";
}

// --- Timeline --------------------------------------------------------------------

function showEvents(st) {
  const all = el("diag-zeit-info").checked;
  const list = st.ereignisse.filter((e) => all || e.schwere !== "info");
  const ol = el("diag-ereignisse");
  ol.replaceChildren();
  for (const e of list) {
    const li = make("li", e.schwere);
    li.append(make("time", null, clockTime(e.zeit)));
    li.append(make("span", `diag-schwere ${e.schwere}`, SEVERITY_TEXT[e.schwere] || e.schwere));
    li.append(make("span", "diag-ereignis-text", e.text));
    ol.append(li);
  }
  el("diag-keine-ereignisse").hidden = list.length > 0;
}

// --- Server log ------------------------------------------------------------------

function showLog() {
  const min = LEVELS[el("diag-log-stufe").value] ?? 0;
  const search = el("diag-log-suche").value.trim().toLowerCase();
  const box = el("diag-log");
  const follow = el("diag-log-folgen").checked;

  box.replaceChildren();
  for (const z of logLines) {
    if ((LEVELS[z.stufe] ?? 0) < min) continue;
    if (search && !`${z.quelle} ${z.text}`.toLowerCase().includes(search)) continue;
    const line = make("div", `diag-log-zeile ${z.stufe.toLowerCase()}`);
    line.append(make("time", null, clockTime(z.zeit)));
    line.append(make("span", "stufe", z.stufe));
    line.append(make("span", "quelle", z.quelle));
    line.append(make("span", "text", z.text));
    box.append(line);
  }
  if (!box.childElementCount) {
    box.append(make("div", "diag-leer", "Keine passenden Zeilen."));
  }
  if (follow) box.scrollTop = box.scrollHeight;
}

async function fetchNewLog() {
  const response = await fetchJson(`/diagnose/protokoll?nach=${lastLogId}`);
  // Server restarted? Then the numbers start at 1 again.
  if (response.letzte_id < lastLogId) {
    lastLogId = 0;
    logLines = [];
    return fetchNewLog();
  }
  if (response.zeilen.length) {
    logLines = logLines.concat(response.zeilen).slice(-LOG_MAX);
    lastLogId = response.letzte_id;
    showLog();
  } else if (!el("diag-log").childElementCount) {
    showLog();
  }
}

// --- Report ----------------------------------------------------------------------

/** Everything important as plain text -- for pasting into a chat, ticket or docs. */
function report() {
  const st = lastStatus;
  if (!st) return "Noch keine Daten.";
  const z = [];
  z.push(`ROSMASTER X3 -- Diagnosebericht ${new Date(st.zeit * 1000).toLocaleString("de-DE")}`);
  z.push(`Gesamt: ${st.gesamt.stufe} -- ${st.gesamt.text}${st.simulation ? " (Simulation)" : ""}`);
  z.push("");
  z.push("Probleme:");
  if (!st.probleme.length) z.push("  keine");
  for (const p of st.probleme) {
    z.push(`  [${p.schwere}] ${p.titel} -- ${p.anzahl}×, zuletzt ${clockTime(p.zuletzt)}` +
      `${p.besteht ? ", besteht noch" : ""}${p.quittiert ? ", gesehen" : ""}`);
  }
  z.push("");
  z.push("Messwerte:");
  for (const [k, v] of Object.entries(st.messwerte)) {
    z.push(`  ${k}: ${typeof v === "object" && v !== null ? JSON.stringify(v) : v}`);
  }
  z.push("");
  z.push("Geräte:");
  for (const g of st.geraete) z.push(`  ${g.ok === false ? "✗" : "✓"} ${g.name}: ${g.text}`);
  z.push("");
  if (st.absturz) {
    const a = st.absturz;
    z.push("Abstürze & Hänger:");
    if (a.vorher) z.push(`  Voriger Lauf: ${a.vorher.titel}`);
    z.push(`  Hänger: ${a.haenger}, längste Verzögerung ${a.puls_max_s} s, USB-Überlast ${a.usb_pro_min ?? "?"}/min`);
    for (const l of a.laeufe.slice(0, 10)) {
      z.push(`  Lauf ${new Date(l.start * 1000).toLocaleString("de-DE")}: ${l.ende || "kein Ende vermerkt"}`);
    }
    if (a.blockiert) z.push(`  Hing zuletzt:\n${a.blockiert.stapel}`);
    z.push("");
  }
  z.push("Zeitleiste (neueste zuerst):");
  for (const e of st.ereignisse.slice(0, 50)) z.push(`  ${clockTime(e.zeit)} [${e.schwere}] ${e.text}`);
  z.push("");
  z.push("Server-Log (Warnungen/Fehler, letzte 100):");
  for (const l of logLines.filter((l) => LEVELS[l.stufe] >= 2).slice(-100)) {
    z.push(`  ${clockTime(l.zeit)} ${l.stufe} ${l.quelle}: ${l.text}`);
  }
  return z.join("\n");
}

async function copyReport() {
  const text = report();
  // navigator.clipboard only exists over https or localhost -- so not on
  // the robot (http://10.42.0.62). Then the old way.
  try {
    if (navigator.clipboard && window.isSecureContext) {
      await navigator.clipboard.writeText(text);
    } else {
      const field = document.createElement("textarea");
      field.value = text;
      field.style.position = "fixed";
      field.style.opacity = "0";
      document.body.append(field);
      field.select();
      const ok = document.execCommand("copy");
      field.remove();
      if (!ok) throw new Error("copy");
    }
    showMessage("Bericht in die Zwischenablage kopiert");
  } catch {
    // Last resort: download as a file.
    const a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([text], { type: "text/plain" }));
    a.download = `x3-diagnose-${new Date().toISOString().slice(0, 19).replace(/:/g, "-")}.txt`;
    a.click();
    URL.revokeObjectURL(a.href);
    showMessage("Kopieren nicht möglich -- Bericht als Datei heruntergeladen");
  }
}

// --- Main loop -------------------------------------------------------------------

async function refresh() {
  clearTimeout(timer);
  try {
    const st = await fetchJson("/diagnose/status");
    lastStatus = st;
    showOverall(st);
    showProblems(st);
    showMeasurements(st);
    showDevices(st);
    showCrash(st);
    showEvents(st);
    await fetchNewLog();
    el("diag-aktualisiert").textContent = `aktualisiert ${new Date().toLocaleTimeString("de-DE")}`;
    el("diag-aktualisiert").classList.remove("fehler");
  } catch {
    el("diag-aktualisiert").textContent = "Roboter nicht erreichbar";
    el("diag-aktualisiert").classList.add("fehler");
    const box = el("diag-gesamt");
    box.className = "panel wide-panel diag-gesamt unbekannt";
    el("diag-gesamt-titel").textContent = "Keine Verbindung zum Roboter";
    el("diag-gesamt-unter").textContent =
      "Läuft der Server? Ist dieser Rechner im WLAN des Roboters? Die Seite versucht es weiter.";
  }
  if (!paused) timer = setTimeout(refresh, INTERVAL_MS);
}

el("diag-pause").addEventListener("click", () => {
  paused = !paused;
  el("diag-pause").textContent = paused ? "Weiter" : "Pause";
  el("diag-pause").classList.toggle("btn-primary", paused);
  if (paused) {
    clearTimeout(timer);
    el("diag-aktualisiert").textContent = "pausiert";
  } else {
    refresh();
  }
});
el("diag-alle-quittieren").addEventListener("click", () => acknowledge({ alle: true }));
el("diag-zeit-info").addEventListener("change", () => lastStatus && showEvents(lastStatus));
el("diag-log-stufe").addEventListener("change", showLog);
el("diag-log-suche").addEventListener("input", showLog);
el("diag-bericht").addEventListener("click", copyReport);
window.addEventListener("resize", () => lastStatus && drawCurve(lastStatus.akku_verlauf, lastStatus.schwellen));

refresh();

})();
