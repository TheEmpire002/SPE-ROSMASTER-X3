// Seite /healthbot: Prüfung starten, Ablauf und Begründung verfolgen.
//
// Zwei Quellen: Der WebSocket liefert Pose und Kurzstatus im Takt der
// Statusmeldungen (und nimmt die Befehle an), /healthbot/status die
// ausführlichen Daten (Zeitleiste, Zeichen, Stupser, Skelett). So bleibt die
// Statusmeldung für alle anderen Clients klein.

"use strict";

const el = (id) => document.getElementById(id);

const ui = {
  status: el("status"),
  statusText: el("status-text"),
  alarm: el("hb-alarm"),
  alarmText: el("hb-alarm-text"),
  schritte: el("hb-schritte"),
  zustand: el("hb-zustand"),
  schritt: el("hb-schritt"),
  startHier: el("hb-start-hier"),
  startZiel: el("hb-start-ziel"),
  stop: el("hb-stop"),
  wache: el("hb-wache"),
  stupsenAus: el("hb-stupsen-aus"),
  karte: el("hb-karte"),
  zielText: el("hb-ziel-text"),
  bild: el("hb-bild"),
  skelett: el("hb-skelett"),
  checks: el("hb-checks"),
  urteil: el("hb-urteil"),
  keineStupser: el("hb-keine-stupser"),
  stupser: el("hb-stupser"),
  antwortForm: el("hb-antwort-form"),
  antwort: el("hb-antwort"),
  sim: el("hb-sim"),
  simText: el("hb-sim-text"),
  simKnoepfe: el("hb-sim-knoepfe"),
  timeline: el("hb-timeline"),
};

// Klartext für die Zustände des Ablaufs (zustandserkennung/ablauf.py).
const ZUSTAND_TEXT = {
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
// Reihenfolge der Schrittanzeige; alles danach ist "Ergebnis".
const REIHE = ["anfahrt", "gesamtbild", "ansprechen", "anstupsen"];
const ENDE = ["nachfrage", "hilfe", "ansprechbar", "unauffaellig", "keine_person", "abgebrochen"];

const SZENE_TEXT = {
  liegend_regungslos: "Person liegt regungslos",
  liegend_bewegt: "Person liegt, bewegt sich",
};

let socket = null;
let pose = null;          // {x, y, winkel} aus der Statusmeldung
let ziel = null;          // vom Nutzer angetippter Ort {x, y}
let status = null;        // letzte Antwort von /healthbot/status
let scan = [];            // [[winkel°, abstand m], ...] relativ zum Roboter
let karte = null;         // Belegungskarte der Kartierung als Bild, falls vorhanden
let skelettLinien = [];
let letzteTimeline = -1;

// ------------------------------------------------------------------ WebSocket

function verbinden() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  socket = new WebSocket(`${proto}//${location.host}/ws`);
  socket.onopen = () => setzeVerbunden(true);
  socket.onclose = () => {
    setzeVerbunden(false);
    setTimeout(verbinden, 1500);
  };
  socket.onerror = () => {
    // onclose folgt direkt danach und baut neu auf.
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
      if (msg.healthbot) zeigeKurz(msg.healthbot);
    } else if (msg.type === "alert" && msg.level === "emergency") {
      zeigeAlarm(msg.text);
    }
  };
}

function setzeVerbunden(ja) {
  ui.status.className = `status ${ja ? "online" : "offline"}`;
  ui.statusText.textContent = ja ? "verbunden" : "getrennt";
}

function senden(nachricht) {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(nachricht));
    return true;
  }
  return false;
}

// ------------------------------------------------------------------ Bedienung

ui.startHier.addEventListener("click", () => senden({ type: "health_check" }));
ui.startZiel.addEventListener("click", () => {
  if (ziel) senden({ type: "health_check", x: ziel.x, y: ziel.y });
});
ui.stop.addEventListener("click", () => senden({ type: "health_stop" }));
ui.wache.addEventListener("change", () => senden({ type: "health_wache", an: ui.wache.checked }));

ui.antwortForm.addEventListener("submit", (ev) => {
  ev.preventDefault();
  const text = ui.antwort.value.trim();
  if (text && senden({ type: "speech", text })) ui.antwort.value = "";
});

// ------------------------------------------------------------------ Anzeige

function zeigeKurz(hb) {
  // Der Kurzstatus kommt öfter als die Abfrage: Schritt und Alarm sofort zeigen.
  zeigeZustand(hb.zustand, hb.schritt, hb.aktiv);
  ui.alarm.hidden = !hb.alarm;
  ui.wache.checked = !!hb.wache;
}

function zeigeAlarm(text) {
  ui.alarm.hidden = false;
  ui.alarmText.textContent = text;
}

function zeigeZustand(zustand, schritt, aktiv) {
  ui.zustand.textContent = ZUSTAND_TEXT[zustand] || zustand || "–";
  ui.zustand.className = `hb-marke z-${zustand}`;
  ui.schritt.textContent = schritt || "";
  ui.startHier.disabled = !!aktiv;
  ui.startZiel.disabled = !!aktiv || !ziel;
  const jetzt = REIHE.indexOf(zustand);
  const fertig = ENDE.includes(zustand);
  for (const li of ui.schritte.children) {
    const z = li.dataset.z;
    const i = REIHE.indexOf(z);
    li.classList.toggle("aktiv", z === zustand || (z === "ende" && fertig));
    li.classList.toggle("erledigt", (i >= 0 && jetzt > i) || (i >= 0 && fertig));
  }
}

function zeigeChecks(checks) {
  ui.checks.replaceChildren(
    ...checks.map((c) => {
      const d = document.createElement("div");
      d.className = `hb-check ${c.zustand}`;
      const ampel = document.createElement("span");
      ampel.className = "hb-check-ampel";
      ampel.setAttribute("aria-hidden", "true");
      const titel = document.createElement("strong");
      titel.textContent = c.titel;
      const urteil = document.createElement("span");
      urteil.className = "hb-check-urteil";
      urteil.textContent = { auffaellig: "auffällig", ok: "unauffällig" }[c.zustand] || "noch offen";
      const text = document.createElement("p");
      text.textContent = c.text || "";
      d.append(ampel, titel, urteil, text);
      return d;
    }),
  );
}

function zeigeUrteil(u) {
  const stufe = u ? u.stufe : "unbekannt";
  ui.urteil.className = `hb-urteil ${stufe}`;
  ui.urteil.querySelector(".hb-urteil-stufe").textContent =
    { notfall: "NOTFALL", auffaellig: "auffällig", ansprechbar: "ansprechbar", unauffaellig: "unauffällig" }[stufe] || "–";
  ui.urteil.querySelector(".hb-urteil-text").textContent = u ? u.text : "noch keine Bewertung";
  ui.urteil.querySelector(".hb-punkte").textContent = u ? `${u.punkte} von 4 Zeichen` : "";
}

function zeigeStupser(stupser) {
  ui.keineStupser.hidden = stupser.length > 0;
  ui.stupser.replaceChildren(
    ...stupser.map((s) => {
      const li = document.createElement("li");
      const reaktion = s.reaktion === undefined ? "" : s.reaktion ? " · Reaktion!" : " · keine Reaktion";
      li.textContent = `Stups ${s.nr}: ${(s.weg_m * 100).toFixed(0)} cm vor, Ende durch ${s.grund}${reaktion}`;
      return li;
    }),
  );
}

function uhrzeit(t) {
  return new Date(t * 1000).toLocaleTimeString("de-DE");
}

function zeigeTimeline(eintraege) {
  // Nur neu aufbauen, wenn etwas dazukam; sonst springt die Auswahl beim Lesen.
  const kennung = eintraege.length ? eintraege.length + eintraege[eintraege.length - 1].zeit : 0;
  if (kennung === letzteTimeline) return;
  letzteTimeline = kennung;
  ui.timeline.replaceChildren(
    ...eintraege
      .slice()
      .reverse()
      .map((e) => {
        const li = document.createElement("li");
        li.className = `z-${e.zustand}`;
        const zeit = document.createElement("time");
        zeit.textContent = uhrzeit(e.zeit);
        const marke = document.createElement("span");
        marke.className = `hb-marke z-${e.zustand}`;
        marke.textContent = ZUSTAND_TEXT[e.zustand] || e.zustand;
        const text = document.createElement("span");
        text.textContent = e.text;
        li.append(zeit, marke, text);
        return li;
      }),
  );
}

function zeigeSimulation(sim, szenen) {
  ui.sim.hidden = !szenen || szenen.length === 0;
  if (ui.sim.hidden) return;
  ui.simText.textContent = sim && sim.szene
    ? `Jetzt: ${SZENE_TEXT[sim.szene] || sim.szene} bei (${sim.x.toFixed(2)}, ${sim.y.toFixed(2)}).`
    : "Jetzt: keine Person.";
  if (ui.simKnoepfe.children.length) return;
  for (const szene of [...szenen, null]) {
    const b = document.createElement("button");
    b.type = "button";
    b.className = "btn";
    b.textContent = szene ? SZENE_TEXT[szene] || szene : "Person entfernen";
    b.addEventListener("click", () => simulieren(szene));
    ui.simKnoepfe.append(b);
  }
}

async function simulieren(szene) {
  const ort = ziel || { x: 1.5, y: 0.0 };
  try {
    await fetch("/healthbot/simulation", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ szene, x: ort.x, y: ort.y }),
    });
  } catch {
    // Beim nächsten Abfragen sieht man, ob es geklappt hat.
  }
  abfragen();
}

// ------------------------------------------------------------------ Abfragen

async function abfragen() {
  try {
    const r = await fetch("/healthbot/status", { cache: "no-store" });
    if (!r.ok) return;
    status = await r.json();
  } catch {
    return; // Verbindung kurz weg: beim nächsten Takt wieder.
  }
  zeigeZustand(status.zustand, status.schritt, status.aktiv);
  ui.alarm.hidden = !status.alarm;
  if (status.alarm && status.urteil) ui.alarmText.textContent = status.urteil.text;
  ui.wache.checked = !!status.wache;
  ui.stupsenAus.hidden = status.stupsen_aktiv !== false;
  zeigeChecks(status.checks || []);
  zeigeUrteil(status.urteil);
  zeigeStupser(status.stupser || []);
  zeigeTimeline(status.timeline || []);
  zeigeSimulation(status.simulation, status.szenen);
  skelettZeichnen();
}

async function scanHolen() {
  try {
    const r = await fetch("/lidar/scan", { cache: "no-store" });
    if (r.ok) scan = (await r.json()).punkte || [];
  } catch {
    scan = [];
  }
}

async function karteHolen() {
  // Die Belegungskarte gibt es nur, solange bzw. nachdem kartiert wurde.
  try {
    const r = await fetch("/api/mapping/map", { cache: "no-store" });
    if (!r.ok) {
      karte = null;
      return;
    }
    karte = karteAlsBild(await r.json());
  } catch {
    karte = null;
  }
}

function karteAlsBild(m) {
  const info = m.info || {};
  const b = info.width;
  const h = info.height;
  if (!b || !h || !m.data_b64) return null;
  const roh = atob(m.data_b64);
  const c = document.createElement("canvas");
  c.width = b;
  c.height = h;
  const ctx = c.getContext("2d");
  const bild = ctx.createImageData(b, h);
  for (let i = 0; i < b * h; i++) {
    let v = roh.charCodeAt(i);
    if (v > 127) v -= 256; // int8: -1 = unbekannt
    // Zeile 0 der Karte liegt unten (ROS), im Bild oben: spiegeln.
    const zeile = h - 1 - Math.floor(i / b);
    const p = (zeile * b + (i % b)) * 4;
    const belegt = v > 50;
    bild.data[p] = bild.data[p + 1] = bild.data[p + 2] = belegt ? 220 : 120;
    bild.data[p + 3] = v < 0 ? 0 : belegt ? 200 : 40;
  }
  ctx.putImageData(bild, 0, 0);
  const o = (info.origin && info.origin.position) || { x: 0, y: 0 };
  return { bild: c, aufloesung: info.resolution, x0: o.x, y0: o.y, breite: b, hoehe: h };
}

// ------------------------------------------------------------------ Karte

// Ausschnitt: so groß, dass Roboter, Startpunkt und Ziel hineinpassen.
function ausschnitt() {
  const punkte = [[0, 0]];
  if (pose) punkte.push([pose.x, pose.y]);
  if (ziel) punkte.push([ziel.x, ziel.y]);
  const xs = punkte.map((p) => p[0]);
  const ys = punkte.map((p) => p[1]);
  const mx = (Math.min(...xs) + Math.max(...xs)) / 2;
  const my = (Math.min(...ys) + Math.max(...ys)) / 2;
  const halb = Math.max(2.5, (Math.max(...xs) - Math.min(...xs)) / 2 + 1.5, (Math.max(...ys) - Math.min(...ys)) / 2 + 1.5);
  return { mx, my, halb };
}

// Weltkoordinaten (x vorn, y links) -> Bildpunkte: x nach oben, y nach links.
function inBild(a, x, y) {
  const c = ui.karte;
  const s = c.width / (2 * a.halb);
  return [c.width / 2 - (y - a.my) * s, c.height / 2 - (x - a.mx) * s];
}

function ausBild(a, px, py) {
  const c = ui.karte;
  const s = c.width / (2 * a.halb);
  return { x: a.mx + (c.height / 2 - py) / s, y: a.my + (c.width / 2 - px) / s };
}

let aktuellerAusschnitt = ausschnitt();

function karteZeichnen() {
  const c = ui.karte;
  const ctx = c.getContext("2d");
  const a = (aktuellerAusschnitt = ausschnitt());
  const s = c.width / (2 * a.halb);
  ctx.clearRect(0, 0, c.width, c.height);

  if (karte) {
    // Karte im eigenen Rahmen: Ursprung = Start der Kartierung, also wie der
    // Startpunkt der Odometrie, wenn beide am selben Ort begonnen haben.
    ctx.save();
    const [px, py] = inBild(a, karte.x0, karte.y0);
    ctx.translate(px, py);
    ctx.rotate(-Math.PI / 2);
    ctx.scale(karte.aufloesung * s, karte.aufloesung * s);
    ctx.drawImage(karte.bild, 0, -karte.hoehe);
    ctx.restore();
  }

  // Meterraster
  ctx.strokeStyle = "#2a2e37";
  ctx.lineWidth = 1;
  for (let m = Math.floor(a.mx - a.halb); m <= a.mx + a.halb; m++) {
    const [, py] = inBild(a, m, 0);
    ctx.beginPath(); ctx.moveTo(0, py); ctx.lineTo(c.width, py); ctx.stroke();
  }
  for (let m = Math.floor(a.my - a.halb); m <= a.my + a.halb; m++) {
    const [px] = inBild(a, 0, m);
    ctx.beginPath(); ctx.moveTo(px, 0); ctx.lineTo(px, c.height); ctx.stroke();
  }

  // Lidar, vom Roboter aus in Weltkoordinaten gedreht.
  if (pose && scan.length) {
    const th = (pose.winkel * Math.PI) / 180;
    ctx.fillStyle = "#4da3ff";
    for (const [w, d] of scan) {
      const r = th + (w * Math.PI) / 180;
      const [px, py] = inBild(a, pose.x + d * Math.cos(r), pose.y + d * Math.sin(r));
      ctx.fillRect(px - 1, py - 1, 2, 2);
    }
  }

  // Startpunkt
  const [hx, hy] = inBild(a, 0, 0);
  ctx.fillStyle = "#3ecf8e";
  ctx.beginPath();
  ctx.moveTo(hx, hy - 9); ctx.lineTo(hx + 8, hy); ctx.lineTo(hx + 5, hy); ctx.lineTo(hx + 5, hy + 7);
  ctx.lineTo(hx - 5, hy + 7); ctx.lineTo(hx - 5, hy); ctx.lineTo(hx - 8, hy); ctx.closePath();
  ctx.fill();

  // Ziel: das angetippte und das, mit dem der Healthbot gerade arbeitet.
  const zielHb = status && status.ziel;
  for (const z of [ziel, zielHb]) {
    if (!z) continue;
    const [zx, zy] = inBild(a, z.x, z.y);
    ctx.strokeStyle = z === zielHb ? "#f05d5d" : "#f5a524";
    ctx.lineWidth = 2.5;
    ctx.beginPath(); ctx.moveTo(zx - 7, zy - 7); ctx.lineTo(zx + 7, zy + 7);
    ctx.moveTo(zx + 7, zy - 7); ctx.lineTo(zx - 7, zy + 7); ctx.stroke();
    ctx.beginPath(); ctx.arc(zx, zy, 11, 0, 2 * Math.PI); ctx.stroke();
  }

  // Roboter als Dreieck in Blickrichtung
  if (pose) {
    const [rx, ry] = inBild(a, pose.x, pose.y);
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

ui.karte.addEventListener("click", (ev) => {
  const r = ui.karte.getBoundingClientRect();
  const px = ((ev.clientX - r.left) / r.width) * ui.karte.width;
  const py = ((ev.clientY - r.top) / r.height) * ui.karte.height;
  const p = ausBild(aktuellerAusschnitt, px, py);
  ziel = { x: Math.round(p.x * 100) / 100, y: Math.round(p.y * 100) / 100 };
  ui.zielText.textContent = `Ziel: x ${ziel.x.toFixed(2)} m (vorn), y ${ziel.y.toFixed(2)} m (links).`;
  ui.startZiel.disabled = !!(status && status.aktiv);
});

// ------------------------------------------------------------------ Skelett

async function skelettLinienHolen() {
  // Die Verbindungslinien der Keypoints liefert der Server einmal mit.
  try {
    const r = await fetch("/interaktion/pose", { cache: "no-store" });
    if (r.ok) skelettLinien = (await r.json()).skelett || [];
  } catch {
    // ohne Linien zeichnen wir nur die Punkte
  }
}

function skelettZeichnen() {
  const c = ui.skelett;
  const img = ui.bild;
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
  for (const [a, b] of skelettLinien) {
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

// ------------------------------------------------------------------ Takt

verbinden();
skelettLinienHolen();
abfragen();
karteHolen();
setInterval(abfragen, 700);
setInterval(scanHolen, 500);
setInterval(karteHolen, 5000);
(function malen() {
  karteZeichnen();
  requestAnimationFrame(malen);
})();
