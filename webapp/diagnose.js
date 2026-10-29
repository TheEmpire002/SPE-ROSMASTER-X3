/*
 * Diagnoseseite (/diagnose, auch /logs).
 *
 * Holt alle paar Sekunden GET /diagnose/status (Probleme, Messwerte,
 * Zeitleiste) und GET /diagnose/protokoll (nur neue Log-Zeilen). Die ganze
 * Auswertung -- was ist ein Problem, wie schlimm, was tun -- passiert auf
 * dem Roboter (diagnose.py). Die Seite stellt nur dar; so sehen alle
 * Oberflächen dasselbe.
 *
 * Texte vom Server kommen immer per textContent in die Seite, nie als
 * HTML: im Log kann alles Mögliche stehen.
 */

(() => {

const INTERVALL_MS = 3000;
const LOG_MAX = 1000;
const STUFEN = { DEBUG: 0, INFO: 1, WARNING: 2, ERROR: 3, CRITICAL: 4 };
const SCHWERE_TEXT = { kritisch: "Kritisch", warnung: "Warnung", hinweis: "Hinweis", info: "Info" };

const el = (id) => document.getElementById(id);

let pausiert = false;
let letzteLogId = 0;
let logZeilen = [];
let letzterStatus = null;
let timer = null;

// --- kleine Helfer ---------------------------------------------------------------

function neu(tag, klasse, text) {
  const e = document.createElement(tag);
  if (klasse) e.className = klasse;
  if (text !== undefined && text !== null) e.textContent = text;
  return e;
}

function uhrzeit(sek) {
  return new Date(sek * 1000).toLocaleTimeString("de-DE");
}

/** "vor 12 s", "vor 3 min" -- Menschen denken nicht in Unix-Zeit. */
function vor(sek) {
  if (sek < 5) return "gerade eben";
  if (sek < 60) return `vor ${Math.round(sek)} s`;
  if (sek < 3600) return `vor ${Math.round(sek / 60)} min`;
  return `vor ${(sek / 3600).toFixed(1)} h`;
}

function dauer(sek) {
  if (sek === null || sek === undefined) return "–";
  const h = Math.floor(sek / 3600);
  const m = Math.floor((sek % 3600) / 60);
  if (h > 0) return `${h} h ${m} min`;
  if (m > 0) return `${m} min ${Math.floor(sek % 60)} s`;
  return `${Math.floor(sek)} s`;
}

function zeigeMeldung(text, fehler = false) {
  const m = el("diag-meldung");
  m.textContent = text;
  m.classList.toggle("fehler", fehler);
  m.hidden = false;
  clearTimeout(zeigeMeldung.timer);
  zeigeMeldung.timer = setTimeout(() => { m.hidden = true; }, fehler ? 8000 : 3000);
}

async function holen(pfad) {
  const antwort = await fetch(pfad, { cache: "no-store" });
  if (!antwort.ok) throw new Error(`HTTP ${antwort.status}`);
  return antwort.json();
}

async function quittieren(daten) {
  try {
    const antwort = await fetch("/diagnose/quittieren", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(daten),
    });
    if (!antwort.ok) throw new Error(`HTTP ${antwort.status}`);
  } catch {
    zeigeMeldung("Roboter nicht erreichbar", true);
    return;
  }
  aktualisieren();
}

// --- Gesamtzustand ---------------------------------------------------------------

function zeigeGesamt(st) {
  const box = el("diag-gesamt");
  box.className = `panel wide-panel diag-gesamt ${st.gesamt.stufe}`;
  const titel = { ok: "Alles in Ordnung", warnung: "Achtung", kritisch: "Problem!" }[st.gesamt.stufe];
  el("diag-gesamt-titel").textContent = titel || st.gesamt.stufe;

  const offen = st.probleme.filter((p) => !p.quittiert);
  let unter = st.gesamt.text;
  if (offen.length) unter += ` ${offen.length} offene Meldung${offen.length === 1 ? "" : "en"}.`;
  if (st.simulation) unter += " (Simulation -- Werte sind ausgedacht.)";
  el("diag-gesamt-unter").textContent = unter;
  el("diag-alle-quittieren").hidden = offen.length === 0;
}

// --- Probleme --------------------------------------------------------------------

function problemKarte(p) {
  const karte = neu("article", `diag-problem ${p.schwere}${p.quittiert ? " gesehen" : ""}`);

  const kopf = neu("div", "diag-problem-kopf");
  kopf.append(neu("span", `diag-schwere ${p.schwere}`, SCHWERE_TEXT[p.schwere] || p.schwere));
  kopf.append(neu("strong", "diag-problem-titel", p.titel));
  karte.append(kopf);

  // Wann und wie oft -- das unterscheidet "einmal passiert" von "dauernd".
  let wann;
  if (p.besteht) {
    wann = `besteht seit ${uhrzeit(p.seit)}`;
  } else if (p.anzahl > 1) {
    wann = `${p.anzahl}× seit ${uhrzeit(p.seit)}, zuletzt ${vor(p.vor_s)} (${uhrzeit(p.zuletzt)})`;
  } else {
    wann = `${vor(p.vor_s)} (${uhrzeit(p.zuletzt)})`;
  }
  if (p.besteht && p.anzahl > 1) wann += ` · ${p.anzahl}× aufgetreten`;
  if (p.zustand && !p.besteht) wann += " · jetzt wieder in Ordnung";
  karte.append(neu("div", "diag-problem-wann", wann));

  karte.append(neu("p", "diag-problem-was", p.was));

  if (p.tun && p.tun.length) {
    const tun = neu("div", "diag-problem-tun");
    tun.append(neu("span", "diag-tun-titel", "Was tun?"));
    const liste = neu("ol");
    for (const schritt of p.tun) liste.append(neu("li", null, schritt));
    tun.append(liste);
    karte.append(tun);
  }

  if (!p.quittiert) {
    const knopf = neu("button", "btn diag-quittieren", "Gesehen");
    knopf.type = "button";
    knopf.title = "Ausblenden, bis es erneut auftritt";
    knopf.addEventListener("click", () => quittieren({ id: p.id }));
    karte.append(knopf);
  }
  return karte;
}

function zeigeProbleme(st) {
  const offen = st.probleme.filter((p) => !p.quittiert);
  const gesehen = st.probleme.filter((p) => p.quittiert);

  el("diag-probleme").replaceChildren(...offen.map(problemKarte));
  el("diag-keine-probleme").hidden = offen.length > 0;

  el("diag-gesehen").hidden = gesehen.length === 0;
  el("diag-gesehen-anzahl").textContent = gesehen.length;
  el("diag-probleme-gesehen").replaceChildren(...gesehen.map(problemKarte));
}

// --- Messwerte und Akku-Kurve ------------------------------------------------------

function zeigeMesswerte(st) {
  const m = st.messwerte;
  const s = st.schwellen;

  const akku = el("diag-akku-v");
  if (m.akku_v === null || m.akku_v === undefined) {
    akku.textContent = "–";
    akku.className = "";
    el("diag-akku-prozent").textContent = "Akku nicht messbar";
  } else {
    akku.textContent = `${m.akku_v.toFixed(2)} V`;
    akku.className = m.akku_v < s.akku_kritisch_v ? "kritisch" : m.akku_v < s.akku_warn_v ? "warnung" : "ok";
    el("diag-akku-prozent").textContent = `ca. ${m.akku_prozent} % (grob geschätzt)`;
  }
  zeichneKurve(st.akku_verlauf, s);

  const zeilen = [];
  const zeile = (name, wert, stufe, hilfe) => zeilen.push({ name, wert, stufe, hilfe });

  if (m.temp_c !== null && m.temp_c !== undefined) {
    zeile("Temperatur Pi", `${m.temp_c.toFixed(1)} °C`,
      m.temp_c >= s.temp_warn_c ? "warnung" : "ok", `Warnung ab ${s.temp_warn_c} °C`);
  }
  if (m.cpu_max_mhz) {
    const ist = m.cpu_ist_mhz ? `${m.cpu_ist_mhz} / ` : "";
    zeile("CPU-Takt (jetzt / max)", `${ist}${m.cpu_max_mhz} MHz`,
      m.cpu_max_mhz > s.cpu_max_mhz ? "warnung" : "ok", `Höchsttakt sollte ≤ ${s.cpu_max_mhz} MHz sein`);
  }
  if (m.last_1min !== null && m.last_1min !== undefined) {
    const kerne = m.kerne || 4;
    zeile("Auslastung (1 min)", `${m.last_1min} von ${kerne} Kernen`,
      m.last_1min > kerne ? "warnung" : "ok", "Über der Kernzahl wartet Arbeit");
  }
  if (m.usb_ueberlast_gesamt !== null && m.usb_ueberlast_gesamt !== undefined) {
    const seit = m.usb_ueberlast_seit_serverstart ?? 0;
    zeile("USB-Überlast", `${seit} seit Serverstart · ${m.usb_ueberlast_gesamt} seit Booten`,
      seit > 0 ? "warnung" : "ok", "Wie oft der Pi die USB-Buchsen abschalten musste");
  }
  if (m.drosselung) {
    const d = m.drosselung;
    let text = d.jetzt.length ? `jetzt: ${d.jetzt.join(", ")}` : "jetzt: nichts";
    if (d.seit_start.length) text += ` · seit Start: ${d.seit_start.join(", ")}`;
    zeile("Spannung/Drosselung Pi", text,
      d.jetzt.includes("Unterspannung") ? "kritisch" : d.jetzt.length || d.seit_start.length ? "warnung" : "ok",
      `vcgencmd get_throttled = ${d.roh}`);
  }
  if (m.bau_prozesse && m.bau_prozesse.length) {
    zeile("Installation läuft", m.bau_prozesse.join(", "), "warnung", "Erzeugt Last und Wärme");
  }
  zeile("Server läuft seit", dauer(m.server_laufzeit_s), "", "");
  if (m.pi_laufzeit_s) zeile("Pi eingeschaltet seit", dauer(m.pi_laufzeit_s), "", "");

  const dl = el("diag-messwerte");
  dl.replaceChildren();
  for (const z of zeilen) {
    const dt = neu("dt", null, z.name);
    const dd = neu("dd", z.stufe, z.wert);
    if (z.hilfe) { dt.title = z.hilfe; dd.title = z.hilfe; }
    dl.append(dt, dd);
  }
}

function zeichneKurve(punkte, s) {
  const canvas = el("diag-akku-kurve");
  // Scharf auf hochauflösenden Bildschirmen: Pixel an die echte Breite anpassen.
  const breite = canvas.clientWidth || 320;
  const hoehe = canvas.clientHeight || 90;
  const dpr = window.devicePixelRatio || 1;
  if (canvas.width !== Math.round(breite * dpr)) {
    canvas.width = Math.round(breite * dpr);
    canvas.height = Math.round(hoehe * dpr);
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, breite, hoehe);

  if (!punkte || punkte.length < 2) {
    ctx.fillStyle = "#9aa1ad";
    ctx.font = "12px system-ui";
    ctx.fillText(punkte && punkte.length ? "sammle Messwerte …" : "keine Akkuwerte", 8, hoehe / 2);
    return;
  }

  const werte = punkte.map((p) => p[1]);
  const lo = Math.min(...werte, s.akku_kritisch_v) - 0.2;
  const hi = Math.max(...werte, 12.6) + 0.1;
  const t0 = punkte[0][0];
  const t1 = punkte[punkte.length - 1][0];
  const x = (t) => ((t - t0) / Math.max(1, t1 - t0)) * (breite - 4) + 2;
  const y = (v) => hoehe - 4 - ((v - lo) / (hi - lo)) * (hoehe - 8);

  // Schwellen als gestrichelte Linien -- so sieht man, wie viel Luft noch ist.
  ctx.setLineDash([4, 4]);
  ctx.lineWidth = 1;
  for (const [v, farbe] of [[s.akku_warn_v, "#f5a524"], [s.akku_kritisch_v, "#f05d5d"]]) {
    ctx.strokeStyle = farbe;
    ctx.beginPath();
    ctx.moveTo(0, y(v));
    ctx.lineTo(breite, y(v));
    ctx.stroke();
  }
  ctx.setLineDash([]);

  ctx.strokeStyle = "#4da3ff";
  ctx.lineWidth = 1.5;
  ctx.beginPath();
  punkte.forEach(([t, v], i) => (i ? ctx.lineTo(x(t), y(v)) : ctx.moveTo(x(t), y(v))));
  ctx.stroke();

  ctx.fillStyle = "#9aa1ad";
  ctx.font = "11px system-ui";
  ctx.fillText(`${Math.max(...werte).toFixed(1)} V`, 4, 12);
  ctx.fillText(`letzte ${dauer(t1 - t0)}`, breite - 90, hoehe - 6);
}

// --- Geräte ----------------------------------------------------------------------

function zeigeGeraete(st) {
  const ul = el("diag-geraete");
  ul.replaceChildren();
  for (const g of st.geraete) {
    const stufe = g.ok === true ? "ok" : g.ok === false ? "fehler" : "ruht";
    const li = neu("li", stufe);
    li.append(neu("span", "diag-punkt-ampel"));
    li.append(neu("span", "diag-geraet-name", g.name));
    let text = g.text;
    if (g.neu) text += ` · ${g.neu}× neu verbunden`;
    li.append(neu("span", "diag-geraet-text", text));
    ul.append(li);
  }

  const q = st.quellen;
  const fehlt = [];
  if (q.kernel_log !== "ok") fehlt.push(`Kernel-Log (${q.kernel_log})`);
  if (!q.usb_zaehler) fehlt.push("USB-Zähler");
  if (!q.vcgencmd) fehlt.push("vcgencmd");
  if (!q.akku) fehlt.push("Akkuspannung");
  el("diag-quellen").textContent = fehlt.length
    ? `Nicht verfügbar: ${fehlt.join(", ")}. Diese Werte fehlen auf der Seite.`
    : "Alle Messquellen verfügbar.";
}

// --- Abstürze & Hänger ------------------------------------------------------------

const ENDE_TEXT = {
  beendet: "sauber beendet",
};

function zeigeAbsturz(st) {
  const a = st.absturz;
  if (!a) return;

  const v = a.vorher;
  el("diag-vorher").hidden = !v;
  if (v) {
    el("diag-vorher").className = `diag-vorher ${v.art === "prozess_weg" ? "warnung" : "kritisch"}`;
    el("diag-vorher-titel").textContent = `Voriger Lauf: ${v.titel}`;
    el("diag-vorher-was").textContent = v.was;
    const teile = [];
    if (v.kernel && v.kernel.length) teile.push("Kernel-Log:", ...v.kernel, "");
    if (v.log && v.log.length) teile.push("Letzte Log-Zeilen:", ...v.log, "");
    if (v.dump && v.dump.length) teile.push("Stack-Dump:", ...v.dump);
    el("diag-vorher-text").textContent = teile.join("\n") || "Nichts aufgezeichnet.";
  }

  const zeilen = [];
  const zeile = (name, wert, stufe = "", hilfe = "") => zeilen.push({ name, wert, stufe, hilfe });
  zeile("Erfassung", a.aktiv ? "aktiv" : "aus (Ordner nicht beschreibbar)", a.aktiv ? "ok" : "warnung");
  zeile("Journal überlebt Neustart", a.journal_dauerhaft ? "ja" : "nein", a.journal_dauerhaft ? "ok" : "warnung",
    "Ohne dauerhaftes Journal ist das Kernel-Log nach einem Neustart weg");
  if (a.puls_alter_s !== null && a.puls_alter_s !== undefined) {
    zeile("Herzschlag des Servers", `vor ${a.puls_alter_s.toFixed(1)} s`, a.puls_alter_s > 2 ? "kritisch" : "ok",
      "Kommt keiner, hängt die Ereignisschleife");
  }
  zeile("Längste Verzögerung", `${a.puls_max_s.toFixed(2)} s`, a.puls_max_s > 1 ? "warnung" : "ok",
    "Wie lange die Ereignisschleife höchstens blockiert war");
  zeile("Hänger seit Start", String(a.haenger), a.haenger ? "warnung" : "ok");
  const sp = a.speicher || {};
  if (sp.server_mb !== undefined) zeile("Speicher Server", `${sp.server_mb} MB, ${sp.threads} Threads, ${sp.dateien} Dateien`);
  if (sp.frei_mb !== undefined) zeile("Speicher frei", `${sp.frei_mb} von ${sp.gesamt_mb} MB`, sp.frei_mb < 300 ? "kritisch" : "ok");
  if (a.usb_pro_min !== null && a.usb_pro_min !== undefined) {
    zeile("USB-Überlast", `${a.usb_pro_min} / min`, a.usb_pro_min >= 20 ? "kritisch" : a.usb_pro_min ? "warnung" : "ok",
      "Jede Meldung trennt kurz alle USB-Geräte");
  }
  const dl = el("diag-absturz-werte");
  dl.replaceChildren();
  for (const z of zeilen) {
    const dt = neu("dt", null, z.name);
    const dd = neu("dd", z.stufe, z.wert);
    if (z.hilfe) { dt.title = z.hilfe; dd.title = z.hilfe; }
    dl.append(dt, dd);
  }

  el("diag-blockiert").hidden = !a.blockiert;
  if (a.blockiert) {
    el("diag-blockiert-text").textContent =
      `${uhrzeit(a.blockiert.zeit)} -- stand ${a.blockiert.dauer_s} s:\n${a.blockiert.stapel}`;
  }

  const ul = el("diag-laeufe");
  ul.replaceChildren();
  for (const l of a.laeufe) {
    const lebt = !l.ende && l.pid && l === a.laeufe[0];
    const stufe = lebt ? "ok" : !l.ende ? "kritisch" : l.ende === "beendet" ? "" : "warnung";
    const ende = lebt ? "läuft (dieser Lauf)" : l.ende ? (ENDE_TEXT[l.ende] || l.ende) : "abgestürzt (kein Ende vermerkt)";
    const li = neu("li", stufe);
    li.append(neu("time", null, new Date(l.start * 1000).toLocaleString("de-DE")));
    li.append(neu("span", null, `${dauer(Math.max(0, l.zuletzt - l.start))} · ${ende}`));
    ul.append(li);
  }
  if (!a.laeufe.length) ul.append(neu("li", null, "Noch keine Läufe aufgezeichnet."));

  const pu = el("diag-usb-ports");
  pu.replaceChildren();
  const ports = Object.entries(a.usb_ports || {});
  for (const [port, n] of ports) {
    const neuN = (a.usb_ports_neu || {})[port] || 0;
    pu.append(neu("li", neuN ? "kritisch" : n ? "warnung" : "ok", `${port}: ${n}${neuN ? ` (+${neuN} seit Serverstart)` : ""}`));
  }
  if (!ports.length) pu.append(neu("li", null, "Keine Zähler lesbar (nur am Pi)."));

  el("diag-absturz-ordner").textContent = a.ordner
    ? `Dateien nach einem Absturz: ${a.ordner} (server.log, laeufe.json, absturz.txt, blockiert.txt)`
    : "";
}

// --- Zeitleiste ------------------------------------------------------------------

function zeigeEreignisse(st) {
  const alle = el("diag-zeit-info").checked;
  const liste = st.ereignisse.filter((e) => alle || e.schwere !== "info");
  const ol = el("diag-ereignisse");
  ol.replaceChildren();
  for (const e of liste) {
    const li = neu("li", e.schwere);
    li.append(neu("time", null, uhrzeit(e.zeit)));
    li.append(neu("span", `diag-schwere ${e.schwere}`, SCHWERE_TEXT[e.schwere] || e.schwere));
    li.append(neu("span", "diag-ereignis-text", e.text));
    ol.append(li);
  }
  el("diag-keine-ereignisse").hidden = liste.length > 0;
}

// --- Server-Log ------------------------------------------------------------------

function zeigeLog() {
  const min = STUFEN[el("diag-log-stufe").value] ?? 0;
  const suche = el("diag-log-suche").value.trim().toLowerCase();
  const box = el("diag-log");
  const folgen = el("diag-log-folgen").checked;

  box.replaceChildren();
  for (const z of logZeilen) {
    if ((STUFEN[z.stufe] ?? 0) < min) continue;
    if (suche && !`${z.quelle} ${z.text}`.toLowerCase().includes(suche)) continue;
    const zeile = neu("div", `diag-log-zeile ${z.stufe.toLowerCase()}`);
    zeile.append(neu("time", null, uhrzeit(z.zeit)));
    zeile.append(neu("span", "stufe", z.stufe));
    zeile.append(neu("span", "quelle", z.quelle));
    zeile.append(neu("span", "text", z.text));
    box.append(zeile);
  }
  if (!box.childElementCount) {
    box.append(neu("div", "diag-leer", "Keine passenden Zeilen."));
  }
  if (folgen) box.scrollTop = box.scrollHeight;
}

async function logNachladen() {
  const antwort = await holen(`/diagnose/protokoll?nach=${letzteLogId}`);
  // Server neu gestartet? Dann fangen die Nummern wieder bei 1 an.
  if (antwort.letzte_id < letzteLogId) {
    letzteLogId = 0;
    logZeilen = [];
    return logNachladen();
  }
  if (antwort.zeilen.length) {
    logZeilen = logZeilen.concat(antwort.zeilen).slice(-LOG_MAX);
    letzteLogId = antwort.letzte_id;
    zeigeLog();
  } else if (!el("diag-log").childElementCount) {
    zeigeLog();
  }
}

// --- Bericht ---------------------------------------------------------------------

/** Alles Wichtige als Klartext -- zum Einfügen in Chat, Ticket oder Doku. */
function bericht() {
  const st = letzterStatus;
  if (!st) return "Noch keine Daten.";
  const z = [];
  z.push(`ROSMASTER X3 -- Diagnosebericht ${new Date(st.zeit * 1000).toLocaleString("de-DE")}`);
  z.push(`Gesamt: ${st.gesamt.stufe} -- ${st.gesamt.text}${st.simulation ? " (Simulation)" : ""}`);
  z.push("");
  z.push("Probleme:");
  if (!st.probleme.length) z.push("  keine");
  for (const p of st.probleme) {
    z.push(`  [${p.schwere}] ${p.titel} -- ${p.anzahl}×, zuletzt ${uhrzeit(p.zuletzt)}` +
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
  for (const e of st.ereignisse.slice(0, 50)) z.push(`  ${uhrzeit(e.zeit)} [${e.schwere}] ${e.text}`);
  z.push("");
  z.push("Server-Log (Warnungen/Fehler, letzte 100):");
  for (const l of logZeilen.filter((l) => STUFEN[l.stufe] >= 2).slice(-100)) {
    z.push(`  ${uhrzeit(l.zeit)} ${l.stufe} ${l.quelle}: ${l.text}`);
  }
  return z.join("\n");
}

async function berichtKopieren() {
  const text = bericht();
  // navigator.clipboard gibt es nur über https oder localhost -- am
  // Roboter (http://10.42.0.62) also nicht. Dann der alte Weg.
  try {
    if (navigator.clipboard && window.isSecureContext) {
      await navigator.clipboard.writeText(text);
    } else {
      const feld = document.createElement("textarea");
      feld.value = text;
      feld.style.position = "fixed";
      feld.style.opacity = "0";
      document.body.append(feld);
      feld.select();
      const ok = document.execCommand("copy");
      feld.remove();
      if (!ok) throw new Error("copy");
    }
    zeigeMeldung("Bericht in die Zwischenablage kopiert");
  } catch {
    // Letzter Ausweg: als Datei herunterladen.
    const a = document.createElement("a");
    a.href = URL.createObjectURL(new Blob([text], { type: "text/plain" }));
    a.download = `x3-diagnose-${new Date().toISOString().slice(0, 19).replace(/:/g, "-")}.txt`;
    a.click();
    URL.revokeObjectURL(a.href);
    zeigeMeldung("Kopieren nicht möglich -- Bericht als Datei heruntergeladen");
  }
}

// --- Ablauf ----------------------------------------------------------------------

async function aktualisieren() {
  clearTimeout(timer);
  try {
    const st = await holen("/diagnose/status");
    letzterStatus = st;
    zeigeGesamt(st);
    zeigeProbleme(st);
    zeigeMesswerte(st);
    zeigeGeraete(st);
    zeigeAbsturz(st);
    zeigeEreignisse(st);
    await logNachladen();
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
  if (!pausiert) timer = setTimeout(aktualisieren, INTERVALL_MS);
}

el("diag-pause").addEventListener("click", () => {
  pausiert = !pausiert;
  el("diag-pause").textContent = pausiert ? "Weiter" : "Pause";
  el("diag-pause").classList.toggle("btn-primary", pausiert);
  if (pausiert) {
    clearTimeout(timer);
    el("diag-aktualisiert").textContent = "pausiert";
  } else {
    aktualisieren();
  }
});
el("diag-alle-quittieren").addEventListener("click", () => quittieren({ alle: true }));
el("diag-zeit-info").addEventListener("change", () => letzterStatus && zeigeEreignisse(letzterStatus));
el("diag-log-stufe").addEventListener("change", zeigeLog);
el("diag-log-suche").addEventListener("input", zeigeLog);
el("diag-bericht").addEventListener("click", berichtKopieren);
window.addEventListener("resize", () => letzterStatus && zeichneKurve(letzterStatus.akku_verlauf, letzterStatus.schwellen));

aktualisieren();

})();
