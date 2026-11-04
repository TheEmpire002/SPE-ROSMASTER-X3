/*
 * Web-App zur Fernsteuerung des ROSMASTER X3.
 *
 * Arbeitsweise: Tasten und Buttons setzen nur, in welche Richtung gefahren
 * werden soll. Ein Timer schickt daraus 10-mal pro Sekunde einen Fahrbefehl.
 *
 * Der Dauertakt ist Absicht und kein verschwendeter Traffic: der Watchdog auf
 * dem Roboter hält an, sobald die Befehle ausbleiben. Hängt die App oder
 * bricht das WLAN ab, bleibt der Roboter dadurch von selbst stehen.
 */

const SEND_HZ = 10;

const RECONNECT_MS = 1500;

/*
 * Obergrenzen. Der Server schickt sie direkt nach dem Verbinden und
 * überschreibt diese Werte -- sie sind nur das, womit wir in der ersten
 * Sekunde rechnen. Einzustellen sind die Grenzen in config.py auf dem
 * Roboter, nicht hier.
 */
const limits = { vx: 0.4, vy: 0.4, omega: 1.5 };

/*
 * Selbst gewähltes Maximum in m/s. Immer <= limits.vx -- der Roboter kappt
 * ohnehin auf sein eigenes Limit, aber die App soll gar nicht erst mehr
 * anfordern, sonst stimmt die Anzeige nicht mehr mit der Fahrt überein.
 *
 * Wird im Browser gespeichert, damit die Einstellung einen Neuladen
 * übersteht -- sonst steht man nach jedem Reload wieder auf Vollgas.
 */
const STORAGE_KEY = "rosmaster.maxSpeed";
let userMax = 0.4;

// --- Zustand -----------------------------------------------------------------

/** Richtung als -1/0/1 je Achse; wird mit dem Tempo multipliziert. */
const dir = { x: 0, y: 0, rot: 0 };

let speed = 0.4;          // 0..1
let socket = null;
let sendTimer = null;

// --- Oberfläche --------------------------------------------------------------

const el = (id) => document.getElementById(id);

const ui = {
  status: el("status"),
  statusText: el("status-text"),
  speed: el("speed"),
  speedOut: el("speed-out"),
  maxSpeed: el("maxspeed"),
  maxSpeedOut: el("maxspeed-out"),
  camPanel: el("cam-panel"),
  camFeld: el("cam-feld"),
  einstDialog: el("einst-dialog"),
  dist: el("val-dist"),
  pos: el("val-pos"),
  winkel: el("val-winkel"),
  btnHome: el("btn-home"),
  btnSetHome: el("btn-sethome"),
  collision: el("collision"),
  collisionRow: el("collision-row"),
  collisionDist: el("collision-dist"),
  collisionOut: el("collision-out"),
  collisionState: el("collision-state"),
  log: el("log"),
  vx: el("val-vx"),
  vy: el("val-vy"),
  omega: el("val-omega"),
  driver: el("val-driver"),
  bpm: el("val-bpm"),
  motion: el("val-motion"),
  btnExplore: el("btn-explore"),
  btnRecord: el("btn-record"),
  autoZustand: el("val-auto-zustand"),
  autoFrei: el("val-auto-frei"),
  autoDauer: el("val-auto-dauer"),
  autoMenschen: el("val-auto-menschen"),
  autoKamera: el("val-auto-kamera"),
  autoText: el("auto-text"),
  hindernisse: el("hindernisse"),
  camWahl: el("cam-wahl"),
  btnInteraktion: el("btn-interaktion"),
  btnFolgen: el("btn-folgen"),
  iaModus: el("val-ia-modus"),
  iaPerson: el("val-ia-person"),
  iaGeste: el("val-ia-geste"),
  iaZiel: el("val-ia-ziel"),
  btnIaPause: el("btn-ia-pause"),
  btnIaLoslassen: el("btn-ia-loslassen"),
  iaText: el("ia-text"),
  stimme: el("val-stimme"),
  sprache: el("val-sprache"),
  speechForm: el("speech-form"),
  speechText: el("speech-text"),
  sayForm: el("say-form"),
  sayText: el("say-text"),
};

function logLine(text, level = "info") {
  const li = document.createElement("li");
  li.className = level;
  const time = new Date().toLocaleTimeString("de-DE");
  li.textContent = `${time}  ${text}`;
  ui.log.prepend(li);
  while (ui.log.children.length > 60) ui.log.lastChild.remove();
}

function setConnected(connected) {
  ui.status.className = `status ${connected ? "online" : "offline"}`;
  ui.statusText.textContent = connected ? "verbunden" : "getrennt";
  onConnectionChange(connected);
}

// --- Verbindung ---------------------------------------------------------------

function connect() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  socket = new WebSocket(`${proto}//${location.host}/ws`);

  socket.onopen = () => {
    setConnected(true);
    logLine("Verbindung hergestellt");
  };

  socket.onclose = () => {
    setConnected(false);
    stopAll();
    logLine("Verbindung verloren — neuer Versuch …", "warning");
    setTimeout(connect, RECONNECT_MS);
  };

  socket.onerror = () => {
    // onclose folgt direkt danach und kümmert sich um den Neuaufbau.
  };

  socket.onmessage = (event) => handleMessage(JSON.parse(event.data));
}

function handleMessage(msg) {
  switch (msg.type) {
    case "limits":
      limits.vx = msg.max_vx;
      limits.vy = msg.max_vy;
      limits.omega = msg.max_omega;
      logLine(`Roboter erlaubt bis ${limits.vx} m/s`);
      applyLimits();
      break;

    case "state":
      ui.vx.textContent = `${msg.vx.toFixed(2)} m/s`;
      ui.vy.textContent = `${msg.vy.toFixed(2)} m/s`;
      ui.omega.textContent = `${msg.omega.toFixed(2)} rad/s`;
      ui.driver.textContent = msg.driver + (msg.connected ? "" : " (getrennt)");
      zeigePose(msg.pose);
      setzeHomeKnopf(msg.homing === true);
      zeigeKollision(msg.collision);
      zeigeAutonomie(msg.autonomie);
      zeigeInteraktion(msg.interaktion);
      zeigeStimme(msg.stimme);
      zeigeSprache(msg.sprache);
      break;

    case "collision":
      zeigeKollision(msg);
      break;

    case "vitals":
      ui.bpm.textContent = msg.bpm != null ? `${msg.bpm} bpm` : "—";
      ui.motion.textContent =
        msg.motion == null ? "—" : msg.motion ? "ja" : "nein";
      break;

    case "alert":
      logLine(msg.text, msg.level);
      break;

    case "error":
      logLine(msg.text, "error");
      break;

    case "pong":
      break;

    default:
      logLine(`Unbekannte Nachricht: ${msg.type}`, "warning");
  }
}

function send(message) {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(message));
  }
}

// --- Fahrbefehle --------------------------------------------------------------

function sendDrive() {
  // Seitwärts und Drehen werden im gleichen Verhältnis gedrosselt wie
  // vorwärts. Sonst würde ein zahmes Max-Tempo trotzdem zu einem
  // Roboter führen, der sich hektisch um die eigene Achse dreht.
  const factor = limits.vx > 0 ? userMax / limits.vx : 0;

  send({
    type: "drive",
    vx: dir.x * userMax * speed,
    vy: dir.y * limits.vy * factor * speed,
    omega: dir.rot * limits.omega * factor * speed,
  });
}

function stopAll() {
  dir.x = dir.y = dir.rot = 0;
  document.querySelectorAll(".pad-btn.active").forEach((b) =>
    b.classList.remove("active")
  );
  send({ type: "stop" });
}

// --- Buttons (Maus und Touch) ---------------------------------------------------

function press(btn) {
  btn.classList.add("active");
  dir.x = Number(btn.dataset.vx ?? 0);
  dir.y = Number(btn.dataset.vy ?? 0);
  dir.rot = Number(btn.dataset.omega ?? 0);
}

function release(btn) {
  btn.classList.remove("active");
  dir.x = dir.y = dir.rot = 0;
}

document.querySelectorAll(".pad-btn[data-vx], .pad-btn[data-omega]").forEach((btn) => {
  // Pointer-Events decken Maus, Touch und Stift gemeinsam ab.
  btn.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    // Sorgt dafür, dass pointerup auch ankommt, wenn der Finger den
    // Button verlässt -- sonst fährt der Roboter weiter.
    btn.setPointerCapture(e.pointerId);
    press(btn);
  });
  btn.addEventListener("pointerup", () => release(btn));
  btn.addEventListener("pointercancel", () => release(btn));
  btn.addEventListener("lostpointercapture", () => release(btn));
});

el("stop").addEventListener("click", () => {
  stopAll();
  logLine("Not-Stopp ausgelöst", "warning");
});

// --- Tastatur -------------------------------------------------------------------

const KEYS = {
  KeyW: () => (dir.x = 1),
  KeyS: () => (dir.x = -1),
  KeyA: () => (dir.y = 1),
  KeyD: () => (dir.y = -1),
  KeyQ: () => (dir.rot = 1),
  KeyE: () => (dir.rot = -1),
};

const KEYS_UP = {
  KeyW: () => (dir.x = 0),
  KeyS: () => (dir.x = 0),
  KeyA: () => (dir.y = 0),
  KeyD: () => (dir.y = 0),
  KeyQ: () => (dir.rot = 0),
  KeyE: () => (dir.rot = 0),
};

window.addEventListener("keydown", (e) => {
  if (e.repeat) return;
  // In einem offenen Fenster (Einstellungen, MUKKE) wird getippt -- ein
  // "e" im Zahlenfeld oder ein "w" im YouTube-Link darf den Roboter nicht
  // bewegen. Ebenso in Textfeldern (Schieberegler ausgenommen, mit denen
  // man nebenbei das Tempo stellt).
  if (document.querySelector("dialog[open]")) return;
  if (e.target.closest?.("input:not([type=range]):not([type=checkbox]), textarea, select")) return;
  if (e.code === "Space") {
    e.preventDefault();
    stopAll();
    return;
  }
  const fn = KEYS[e.code];
  if (fn) {
    e.preventDefault();
    fn();
  }
});

window.addEventListener("keyup", (e) => {
  const fn = KEYS_UP[e.code];
  if (fn) fn();
});

// Fenster verlassen oder Tab wechseln: anhalten. Sonst fährt der Roboter
// weiter, obwohl niemand mehr hinschaut.
//
// Ausnahme autonomes Fahren: Da steuert niemand von Hand, und ein Stopp
// würde die Fahrt beenden, sobald man z. B. ins Terminal klickt. Die
// Handeingabe wird trotzdem gelöscht. Bricht die Verbindung ab, hält der
// Server den Roboter ohnehin an.
function fensterVerlassen() {
  if (autonomAktiv) {
    dir.x = dir.y = dir.rot = 0;
    document.querySelectorAll(".pad-btn.active").forEach((b) =>
      b.classList.remove("active")
    );
    return;
  }
  stopAll();
}
window.addEventListener("blur", fensterVerlassen);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) fensterVerlassen();
});

// --- Tempo ------------------------------------------------------------------------

ui.speed.addEventListener("input", () => {
  speed = Number(ui.speed.value) / 100;
  updateLabels();
});

ui.maxSpeed.addEventListener("input", () => {
  userMax = Number(ui.maxSpeed.value);
  try {
    localStorage.setItem(STORAGE_KEY, String(userMax));
  } catch {
    // Privater Modus o.Ä. -- dann gilt die Einstellung nur diese Sitzung.
  }
  updateLabels();
});

function updateLabels() {
  ui.maxSpeedOut.textContent = `${userMax.toFixed(2)} m/s`;
  // Prozent allein sagt wenig, deshalb die resultierende Geschwindigkeit dazu.
  ui.speedOut.textContent =
    `${ui.speed.value} % · ${(userMax * speed).toFixed(2)} m/s`;
}

/**
 * Uebernimmt die Grenzwerte des Roboters in den Max-Tempo-Regler.
 *
 * Laeuft erst, wenn der Server seine Limits gemeldet hat -- vorher weiss die
 * App nicht, wie weit der Regler ueberhaupt gehen darf.
 */
function applyLimits() {
  ui.maxSpeed.max = String(limits.vx);

  let wunsch = limits.vx;
  try {
    const gespeichert = Number(localStorage.getItem(STORAGE_KEY));
    if (gespeichert > 0) wunsch = gespeichert;
  } catch {
    // kein localStorage -- dann gilt das Roboterlimit
  }

  // Nie ueber das, was der Roboter erlaubt: ein gespeicherter Wert kann aus
  // einer Konfiguration mit hoeherem Limit stammen.
  userMax = Math.min(wunsch, limits.vx);
  ui.maxSpeed.value = String(userMax);
  updateLabels();
}

// --- Kollisionsvermeidung ----------------------------------------------------------

/*
 * Die Einstellung gehört dem Server, nicht dem Browser: sie greift in die
 * Sicherheit ein, und bei mehreren offenen Tabs muss klar sein, was gilt.
 * Die Bedienelemente schicken also nur Wünsche und zeigen danach an, was
 * der Server zurückmeldet.
 */
function sendeKollision() {
  send({
    type: "collision",
    enabled: ui.collision.checked,
    distance: Number(ui.collisionDist.value),
  });
}

ui.collision.addEventListener("change", sendeKollision);
ui.collisionDist.addEventListener("change", sendeKollision);
ui.collisionDist.addEventListener("input", () => {
  ui.collisionOut.textContent = `${Number(ui.collisionDist.value).toFixed(2)} m`;
});

function zeigeKollision(z) {
  if (!z) return;
  ui.collision.checked = z.enabled === true;
  ui.collisionRow.hidden = !z.enabled;
  if (typeof z.distance === "number") {
    ui.collisionDist.value = String(z.distance);
    ui.collisionOut.textContent = `${z.distance.toFixed(2)} m`;
  }

  let text = "aus";
  let klasse = "switch-state";
  if (z.enabled && z.verfuegbar === false) {
    // Eingeschaltet ohne Sensor wäre trügerisch -- das muss man sehen.
    text = "kein Lidar!";
    klasse += " blockiert";
  } else if (z.blockiert) {
    text = "hält";
    klasse += " blockiert";
  } else if (z.enabled) {
    text = "an";
    klasse += " an";
  }
  ui.collisionState.textContent = text;
  ui.collisionState.className = klasse;
}

// --- Lidar-Anzeige ---------------------------------------------------------------

/*
 * Draufsicht: der Roboter in der Mitte, vorn ist oben. Die Punkte kommen
 * per HTTP statt über den WebSocket -- eine Umdrehung sind einige hundert
 * Werte, und die würden den Steuerkanal unnötig belasten.
 */
const LIDAR_HZ = 3;
const LIDAR_REICHWEITE_M = 5;

const lidarCanvas = el("lidar");
const lidarInfo = el("lidar-info");
const lidarCtx = lidarCanvas ? lidarCanvas.getContext("2d") : null;

async function lidarHolen() {
  if (!lidarCtx) return;
  try {
    const antwort = await fetch(`/lidar/scan?t=${Date.now()}`);
    const daten = await antwort.json();
    lidarZeichnen(daten.punkte || []);
    lidarInfo.textContent = daten.punkte && daten.punkte.length
      ? `${daten.punkte.length} Punkte · Kreise alle 1 m · Reichweite ${LIDAR_REICHWEITE_M} m`
      : (daten.hinweis || "keine Daten");
  } catch {
    lidarInfo.textContent = "Lidar nicht erreichbar";
  }
}

function lidarZeichnen(punkte) {
  const b = lidarCanvas.width;
  const mitte = b / 2;
  const skala = (b / 2 - 10) / LIDAR_REICHWEITE_M;

  lidarCtx.clearRect(0, 0, b, b);

  // Entfernungsringe als Maßstab, sonst sagt die Punktwolke nichts.
  lidarCtx.strokeStyle = "#2e323c";
  lidarCtx.lineWidth = 1;
  for (let m = 1; m <= LIDAR_REICHWEITE_M; m++) {
    lidarCtx.beginPath();
    lidarCtx.arc(mitte, mitte, m * skala, 0, Math.PI * 2);
    lidarCtx.stroke();
  }

  lidarCtx.fillStyle = "#4da3ff";
  for (const [grad, meter] of punkte) {
    const a = (grad * Math.PI) / 180;
    // Bildschirm: x nach rechts, y nach unten. Roboter: x vorwärts,
    // y links. Deshalb vertauscht und gespiegelt, damit "vorn" oben ist.
    const px = mitte - meter * Math.sin(a) * skala;
    const py = mitte - meter * Math.cos(a) * skala;
    lidarCtx.fillRect(px - 1, py - 1, 2, 2);
  }

  hindernisseZeichnen(mitte, skala);

  // Der Roboter selbst, mit Blickrichtung nach oben.
  lidarCtx.fillStyle = "#3ecf8e";
  lidarCtx.beginPath();
  lidarCtx.moveTo(mitte, mitte - 7);
  lidarCtx.lineTo(mitte - 5, mitte + 5);
  lidarCtx.lineTo(mitte + 5, mitte + 5);
  lidarCtx.closePath();
  lidarCtx.fill();
}

if (lidarCtx) setInterval(lidarHolen, 1000 / LIDAR_HZ);

// --- Autonomes Fahren ----------------------------------------------------------

/*
 * Der Server schickt den Zustand in jeder state-Nachricht mit
 * (state.autonomie). Die App zeigt nur an und schickt explore /
 * explore_stop -- entschieden wird auf dem Roboter.
 */
let autonomAktiv = false;
let hindernisse = [];
let personenBoxen = [];
// Auf welchem Kamerabild die Personenrahmen liegen (vom Server gemeldet).
// null = unbekannt, etwa bei einem älteren Server -- dann wie bisher immer.
let personenKamera = null;

/** Farbe je Hindernisart, gleich in Liste, Lidar-Bild und Kamera. */
const ART_FARBE = {
  "Wand": "#9aa1ad",
  "Stuhl": "#f5a524",
  "Tisch": "#c58af9",
  "Mensch": "#f05d5d",
  "Bein (Mensch?)": "#ff9a8a",
  "Bein (Stuhl/Tisch?)": "#e8c56a",
  "Objekt": "#4da3ff",
};
const farbe = (art) => ART_FARBE[art] || "#4da3ff";

function setzeExploreKnopf(aktiv) {
  autonomAktiv = aktiv;
  ui.btnExplore.textContent = aktiv ? "Autonomes Fahren beenden" : "Autonom fahren";
  ui.btnExplore.classList.toggle("aktiv", aktiv);
}

ui.btnExplore.addEventListener("click", () => {
  send({ type: autonomAktiv ? "explore_stop" : "explore" });
  // Wie beim Home-Knopf: sofort umschalten, die nächste Statusmeldung
  // korrigiert, falls der Server ablehnt (z. B. kein Lidar).
  setzeExploreKnopf(!autonomAktiv);
});

// Fahrtenschreiber: Der Server antwortet mit einer Meldung (Dateiname).
ui.btnRecord.addEventListener("click", () => {
  send({ type: "record_save" });
});

function zeigeAutonomie(a) {
  if (!a) return;
  setzeExploreKnopf(a.aktiv === true);
  ui.btnExplore.disabled = a.verfuegbar === false;

  ui.autoZustand.textContent = a.aktiv ? a.zustand : (a.verfuegbar === false ? "kein Lidar" : "aus");
  ui.autoFrei.textContent = typeof a.frei_vorn === "number" ? `${a.frei_vorn.toFixed(2)} m` : "—";
  ui.autoDauer.textContent = a.aktiv && typeof a.dauer_s === "number"
    ? `${Math.floor(a.dauer_s / 60)}:${String(Math.floor(a.dauer_s % 60)).padStart(2, "0")} min · ${a.ausweichen ?? 0}× ausgewichen`
    : "—";

  hindernisse = a.hindernisse || [];
  personenBoxen = a.personen || [];

  const menschen = hindernisse.filter((h) => h.art === "Mensch").length;
  ui.autoMenschen.textContent = `${menschen} im Lidar · ${personenBoxen.length} im Bild`;

  const pe = a.personenerkennung;
  if (pe && pe.kamera) personenKamera = pe.kamera;
  ui.autoKamera.textContent = !pe ? "—"
    : !pe.verfuegbar ? (pe.fehler || "nicht verfügbar")
    : pe.aktiv ? "erkennt Personen" : "bereit (läuft beim Fahren)";

  let text = a.text || "—";
  if (!a.aktiv && a.grund) text = `Zuletzt beendet: ${a.grund}`;
  ui.autoText.textContent = text;

  zeigeHindernisListe();
  personenZeichnen();
}

function zeigeHindernisListe() {
  ui.hindernisse.replaceChildren(...hindernisse.map((h) => {
    const li = document.createElement("li");
    const art = document.createElement("span");
    art.className = "art";
    const punkt = document.createElement("span");
    punkt.className = "farbe";
    punkt.style.background = farbe(h.art);
    art.append(punkt, document.createTextNode(h.art));
    const wo = document.createElement("span");
    wo.className = "wo";
    // Winkel wie beim Lidar: positiv = links. Für Menschen lesbarer als Grad.
    const seite = Math.abs(h.winkel) < 15 ? "vorn"
      : Math.abs(h.winkel) > 165 ? "hinten"
      : h.winkel > 0 ? "links" : "rechts";
    wo.textContent = `${h.abstand.toFixed(2)} m · ${seite} (${h.winkel.toFixed(0)}°)`;
    li.append(art, wo);
    return li;
  }));
  if (!hindernisse.length) {
    const li = document.createElement("li");
    li.textContent = "keine im Umkreis";
    ui.hindernisse.append(li);
  }
}

/** Markierungen ins Lidar-Bild, gleiche Umrechnung wie die Punkte. */
function hindernisseZeichnen(mitte, skala) {
  lidarCtx.font = "10px system-ui, sans-serif";
  for (const h of hindernisse) {
    if (h.art === "Wand") continue; // Wände sieht man an der Punktreihe
    const px = mitte - h.y * skala;
    const py = mitte - h.x * skala;
    const r = Math.max(5, (h.breite / 2) * skala + 3);
    lidarCtx.strokeStyle = farbe(h.art);
    lidarCtx.lineWidth = 2;
    lidarCtx.beginPath();
    lidarCtx.arc(px, py, r, 0, Math.PI * 2);
    lidarCtx.stroke();
    lidarCtx.fillStyle = farbe(h.art);
    lidarCtx.fillText(h.art, px + r + 2, py + 3);
  }
}

/*
 * Personenrahmen über dem Kamerabild. Das Bild ist mit object-fit:
 * contain eingepasst, also kann es links/rechts oder oben/unten Rand
 * geben -- die Rahmen müssen auf die tatsächliche Bildfläche.
 */
function personenZeichnen() {
  kacheln.forEach((k, i) => {
    const passt = zeigtPersonenKamera(k, i);
    rahmenZeichnen(k, passt ? personenBoxen : [], passt);
    // Antippen nur dort, wo die Skelette auch liegen.
    k.el.classList.toggle("waehlbar", passt && interaktionAktiv && skelette.length > 0);
  });
}

// Die Rahmen gehören zum Bild der Personenkamera. Über einer anderen
// Kamera stünden sie an der falschen Stelle. Kennt der Server keine
// Personenkamera (älterer Stand), dann wie früher über die erste.
function zeigtPersonenKamera(kachel, i) {
  return personenKamera
    ? (kachel.name ?? camStandard) === personenKamera
    : i === 0;
}

/** Wo das Bild in der Kachel liegt (eingepasst, mit Rand). */
function bildFlaeche(kachel, w, h) {
  const img = kachel.img;
  const bildVerh = img.naturalWidth / img.naturalHeight;
  let bw = w, bh = w / bildVerh;
  if (bh > h) {
    bh = h;
    bw = h * bildVerh;
  }
  return { ox: (w - bw) / 2, oy: (h - bh) / 2, bw, bh };
}

// zeigtPersonen: diese Kachel zeigt die Personenkamera -- nur dort gehören
// Rahmen und Skelette hin (beide kommen aus der C270).
function rahmenZeichnen(kachel, boxen, zeigtPersonen) {
  const c = kachel.overlay;
  const img = kachel.img;
  const w = c.clientWidth;
  const h = c.clientHeight;
  if (c.width !== w || c.height !== h) {
    c.width = w;
    c.height = h;
  }
  const ctx = c.getContext("2d");
  ctx.clearRect(0, 0, w, h);
  const skelettHier = zeigtPersonen && skelette.length > 0;
  if ((!boxen.length && !skelettHier) || !img.naturalWidth) return;

  const { ox, oy, bw, bh } = bildFlaeche(kachel, w, h);

  ctx.strokeStyle = farbe("Mensch");
  ctx.fillStyle = farbe("Mensch");
  ctx.lineWidth = 2;
  ctx.font = "12px system-ui, sans-serif";
  for (const b of boxen) {
    const x = ox + b.x1 * bw;
    const y = oy + b.y1 * bh;
    ctx.strokeRect(x, y, (b.x2 - b.x1) * bw, (b.y2 - b.y1) * bh);
    ctx.fillText(`Mensch ${b.score.toFixed(1)}`, x + 3, y + 14);
  }

  if (skelettHier) skeletteZeichnen(ctx, ox, oy, bw, bh);
}

window.addEventListener("resize", personenZeichnen);

// --- Interaktion --------------------------------------------------------------

/*
 * Wie beim autonomen Fahren: Der Roboter entscheidet, die App zeigt an und
 * schickt interaction_start / interaction_stop / follow / follow_stop.
 * Die Skelette holt sie sich nur, solange die Interaktion läuft -- sonst
 * rechnet der Hailo ohnehin nicht.
 *
 * Ziel-Lock: Tippen auf eine Person schickt follow mit ihrer Track-ID. Nur
 * so wird bei mehreren Personen eine bestimmte zum Ziel; der Roboter wechselt
 * nie von selbst. Pause hält an und behält das Ziel, Loslassen gibt es frei.
 */
const POSE_HZ = 8;
/** Körperpunkte unter dieser Sicherheit werden nicht gezeichnet. */
const KP_MIN = 0.3;

const MODUS_TEXT = {
  bereit: "wartet auf Handzeichen",
  folgen: "folgt",
  herkommen: "kommt her",
  weggehen: "weicht zurück",
  sucht: "sucht",
};

/** Zustände des Ziel-Locks (state.interaktion.ziel.zustand). */
const ZIEL_TEXT = {
  idle: () => "kein Ziel",
  acquiring_target: (id) => `erfasst Person ${id} …`,
  following: (id) => `folgt Person ${id}`,
  target_temporarily_lost: () => "Ziel kurz verloren",
  searching_for_target: (id) => `sucht Person ${id} …`,
  safe_stop: () => "sicher angehalten – bitte neu wählen",
};

let interaktionAktiv = false;
let interaktionModus = null;
let interaktionPausiert = false;
let skelette = [];
let skelettLinien = [];
let poseLaeuft = false;

function setzeInteraktionKnoepfe(aktiv, modus) {
  interaktionAktiv = aktiv;
  interaktionModus = modus;
  ui.btnInteraktion.textContent = aktiv ? "Interaktion aus" : "Interaktion an";
  ui.btnInteraktion.classList.toggle("aktiv", aktiv);
  const folgt = aktiv && modus === "folgen";
  ui.btnFolgen.textContent = folgt ? "Nicht mehr folgen" : "Folge mir";
}

ui.btnInteraktion.addEventListener("click", () => {
  send({ type: interaktionAktiv ? "interaction_stop" : "interaction_start" });
  // Sofort umschalten, die nächste Statusmeldung rückt es gerade.
  setzeInteraktionKnoepfe(!interaktionAktiv, interaktionAktiv ? null : "bereit");
});

ui.btnFolgen.addEventListener("click", () => {
  const folgt = interaktionAktiv && interaktionModus === "folgen";
  send({ type: folgt ? "follow_stop" : "follow" });
  setzeInteraktionKnoepfe(true, folgt ? "bereit" : "folgen");
});

ui.btnIaPause.addEventListener("click", () => {
  send({ type: interaktionPausiert ? "follow_resume" : "follow_pause" });
});

ui.btnIaLoslassen.addEventListener("click", () => send({ type: "follow_stop" }));

/** Tippen ins Kamerabild: die getroffene Person wird zum Ziel. */
function personAntippen(kachel, ev) {
  if (!interaktionAktiv || !skelette.length) return;
  if (!zeigtPersonenKamera(kachel, kacheln.indexOf(kachel))) return;
  if (!kachel.img.naturalWidth) return;
  const r = kachel.overlay.getBoundingClientRect();
  const { ox, oy, bw, bh } = bildFlaeche(kachel, r.width, r.height);
  const x = (ev.clientX - r.left - ox) / bw;
  const y = (ev.clientY - r.top - oy) / bh;
  // Etwas Rand zugeben (Finger sind breit). Bei Überlappung die kleinere
  // Box nehmen -- das ist meist die weiter hinten stehende, gezielt gemeinte.
  const RAND = 0.03;
  let treffer = null;
  let flaeche = Infinity;
  for (const s of skelette) {
    const b = s.box;
    if (!b || x < b[0] - RAND || x > b[2] + RAND || y < b[1] - RAND || y > b[3] + RAND) continue;
    const f = (b[2] - b[0]) * (b[3] - b[1]);
    if (f < flaeche) {
      flaeche = f;
      treffer = s;
    }
  }
  if (!treffer) return;
  if (treffer.id == null) {
    logLine("Person noch nicht erfasst – gleich noch einmal antippen", "warning");
    return;
  }
  send({ type: "follow", ziel_id: treffer.id });
  logLine(`Folge Person ${treffer.id}`);
}

function zeigeZiel(ia) {
  const z = ia.ziel;
  const fahrend = ia.aktiv === true && !!ia.modus && ia.modus !== "bereit";
  interaktionPausiert = ia.pausiert === true;
  ui.btnIaPause.textContent = interaktionPausiert ? "Weiter" : "Pause";
  ui.btnIaPause.classList.toggle("aktiv", interaktionPausiert);
  ui.btnIaPause.disabled = !fahrend;
  // Älterer Server ohne Ziel-Lock: keine Zielanzeige.
  if (!z || !ia.aktiv) {
    ui.iaZiel.textContent = "—";
    ui.btnIaLoslassen.disabled = !fahrend;
    return;
  }
  let text = (ZIEL_TEXT[z.zustand] || (() => z.zustand))(z.id);
  if (z.id != null && z.zustand !== "idle" && z.zustand !== "safe_stop") {
    text += ` · ${Math.round((z.sicherheit || 0) * 100)} %`;
  }
  if (interaktionPausiert) text += " · pausiert";
  if (z.grund && (z.zustand === "idle" || z.zustand === "safe_stop")) text += ` (${z.grund})`;
  ui.iaZiel.textContent = text;
  ui.btnIaLoslassen.disabled = !fahrend && z.zustand === "idle";
}

function zeigeInteraktion(ia) {
  if (!ia) return;
  setzeInteraktionKnoepfe(ia.aktiv === true, ia.modus);
  ui.btnInteraktion.disabled = ui.btnFolgen.disabled = ia.verfuegbar === false;
  zeigeZiel(ia);

  ui.iaModus.textContent = ia.aktiv
    ? (MODUS_TEXT[ia.modus] || ia.modus)
    : (ia.verfuegbar === false ? "nicht verfügbar" : "aus");

  const p = ia.person;
  if (p) {
    const seite = Math.abs(p.winkel) < 5 ? "mittig" : p.winkel > 0 ? "links" : "rechts";
    const abstand = typeof p.abstand === "number" ? `${p.abstand.toFixed(2)} m · ` : "";
    ui.iaPerson.textContent = `${abstand}${seite} (${p.winkel.toFixed(0)}°) · ${ia.personen} im Bild`;
  } else {
    ui.iaPerson.textContent = ia.aktiv ? `${ia.personen} im Bild` : "—";
  }
  ui.iaGeste.textContent = ia.geste || "—";

  let text = ia.text || "—";
  if (!ia.aktiv && ia.grund) text = `Zuletzt beendet: ${ia.grund}`;
  if (ia.verfuegbar === false) text = "Braucht Lidar und Posenerkennung — Details unter /interaktion/status";
  ui.iaText.textContent = text;

  if (!ia.aktiv && skelette.length) {
    skelette = [];
    personenZeichnen();
  }
}

// --- Stimme -----------------------------------------------------------------

/*
 * Der Roboter spricht selbst (Gesten, Moduswechsel); hier lässt sich
 * zusätzlich ein eigener Satz vorlesen. Ohne Lautsprecher landet er nur im
 * Log des Roboters -- das zeigt die Anzeige an.
 */
ui.sayForm.addEventListener("submit", (ev) => {
  ev.preventDefault();
  const text = ui.sayText.value.trim();
  if (!text) return;
  send({ type: "say", text });
  ui.sayText.value = "";
});

// Wie ins Mikrofon gesprochen -- zum Ausprobieren ohne Mikrofon.
ui.speechForm.addEventListener("submit", (ev) => {
  ev.preventDefault();
  const text = ui.speechText.value.trim();
  if (!text) return;
  send({ type: "speech", text });
  ui.speechText.value = "";
});

function zeigeSprache(sp) {
  if (!sp) return;
  if (sp.pause) {
    ui.sprache.textContent = `Mikrofon aus: ${sp.pause}`;
  } else if (!sp.verfuegbar) {
    ui.sprache.textContent = "kein Mikrofon (/sprache/status)";
  } else if (sp.befehl) {
    ui.sprache.textContent = `Befehl: ${sp.befehl}`;
  } else if (sp.geweckt) {
    ui.sprache.textContent = "hört zu …";
  } else {
    ui.sprache.textContent = sp.gehoert ? `gehört: „${sp.gehoert}“` : "wartet auf „Robi“";
  }
}

function zeigeStimme(st) {
  if (!st) return;
  if (st.verfuegbar === false) {
    ui.stimme.textContent = "kein Lautsprecher (/stimme/status)";
  } else if (st.spricht) {
    ui.stimme.textContent = `spricht: „${st.text || ""}“`;
  } else {
    ui.stimme.textContent = st.text ? `zuletzt: „${st.text}“` : "bereit";
  }
}

async function poseHolen() {
  if (!interaktionAktiv || poseLaeuft) return;
  poseLaeuft = true; // keine Anfragen auftürmen, wenn das WLAN hängt
  try {
    const r = await fetch("/interaktion/pose", { cache: "no-store" });
    if (r.ok) {
      const d = await r.json();
      skelette = d.personen || [];
      skelettLinien = d.skelett || [];
      personenZeichnen();
    }
  } catch {
    // Verbindung kurz weg: beim nächsten Takt neu versuchen.
  } finally {
    poseLaeuft = false;
  }
}

setInterval(poseHolen, 1000 / POSE_HZ);

/** Skelette auf die eingepasste Bildfläche (ox/oy/bw/bh aus personenZeichnen). */
function skeletteZeichnen(ctx, ox, oy, bw, bh) {
  if (!skelette.length) return;
  ctx.lineWidth = 3;
  ctx.font = "13px system-ui, sans-serif";
  // Das gesperrte Ziel grün, alle anderen grau. Ältere Server ohne Ziel-Lock
  // liefern kein "ziel" -- dann wie früher die größte Person (vorn).
  const mitZiel = skelette.some((s) => "ziel" in s);
  skelette.forEach((s, i) => {
    const istZiel = mitZiel ? s.ziel === true : i === 0;
    const col = istZiel ? "#3ddc97" : "#9aa1ad";
    ctx.strokeStyle = col;
    ctx.fillStyle = col;
    // Nummer über dem Kopf, damit „folgt Person 3“ zuzuordnen ist.
    if (s.id != null && s.box) {
      ctx.fillText(`Person ${s.id}`, ox + s.box[0] * bw + 3, Math.max(oy + 14, oy + s.box[1] * bh - 4));
    }
    const kp = s.kp || [];
    for (const [a, b] of skelettLinien) {
      const pa = kp[a];
      const pb = kp[b];
      if (!pa || !pb || pa[2] < KP_MIN || pb[2] < KP_MIN) continue;
      ctx.beginPath();
      ctx.moveTo(ox + pa[0] * bw, oy + pa[1] * bh);
      ctx.lineTo(ox + pb[0] * bw, oy + pb[1] * bh);
      ctx.stroke();
    }
    for (const [x, y, c] of kp) {
      if (c < KP_MIN) continue;
      ctx.beginPath();
      ctx.arc(ox + x * bw, oy + y * bh, 3, 0, Math.PI * 2);
      ctx.fill();
    }
  });
}

// --- Heimfahrt ----------------------------------------------------------------

let faehrtHeim = false;

function zeigePose(pose) {
  if (!pose) {
    ui.dist.textContent = ui.pos.textContent = ui.winkel.textContent = "—";
    return;
  }
  const entfernung = Math.hypot(pose.x, pose.y);
  // Ohne Messung stammt die Position nur aus den Fahrbefehlen. Das muss
  // man sehen, bevor man dem Home-Knopf vertraut.
  const güte = pose.gemessen === false ? " (geschätzt)" : "";
  ui.dist.textContent = `${entfernung.toFixed(2)} m${güte}`;
  ui.pos.textContent = `${pose.x.toFixed(2)} / ${pose.y.toFixed(2)} m`;
  ui.winkel.textContent = `${pose.winkel.toFixed(0)}°`;
}

/*
 * Derselbe Knopf startet und bricht ab. Zwei getrennte Knöpfe wären ein
 * Risiko: wer den Roboter stoppen will, soll nicht erst den richtigen
 * suchen müssen.
 */
function setzeHomeKnopf(aktiv) {
  if (aktiv === faehrtHeim) return;
  faehrtHeim = aktiv;
  ui.btnHome.textContent = aktiv ? "Heimfahrt abbrechen" : "Nach Hause";
  ui.btnHome.classList.toggle("aktiv", aktiv);
}

ui.btnHome.addEventListener("click", () => {
  if (faehrtHeim) {
    send({ type: "home_cancel" });
  } else {
    send({ type: "home" });
  }
  // Nicht auf die Antwort warten: der Knopf soll sofort reagieren. Die
  // nächste Statusmeldung rückt die Anzeige gerade, falls es schiefging.
  setzeHomeKnopf(!faehrtHeim);
});

ui.btnSetHome.addEventListener("click", () => {
  send({ type: "set_home" });
});

// --- Kamera ----------------------------------------------------------------

/*
 * Zwei Wege zum Bild:
 *
 *   MJPEG     Ein Dauerstrom in einem <img>. Sparsam, aber nicht jeder
 *             Browser kann multipart/x-mixed-replace -- Safari auf
 *             iPhone und iPad zeigt dann einfach nichts an.
 *   Einzelbilder  Immer wieder .../snapshot laden. Funktioniert
 *             überall, kostet etwas mehr Overhead.
 *
 * Erst wird MJPEG versucht. Hat der Browser nach PRUEFZEIT_MS nichts
 * dekodiert, wird automatisch auf Einzelbilder umgeschaltet.
 *
 * Jede angezeigte Kamera ist eine eigene Kachel mit eigenem Bild,
 * eigener Fehleranzeige und eigenem Rahmen-Overlay. So lassen sich auch
 * alle Kameras nebeneinander zeigen.
 */
const PRUEFZEIT_MS = 4000;
const EINZELBILD_FPS = 10;

class CamKachel {
  constructor() {
    this.el = document.createElement("div");
    this.el.className = "cam-wrap";

    // MJPEG: der Browser zeigt den Stream direkt an, ohne JavaScript.
    this.img = document.createElement("img");
    this.img.className = "cam";
    this.img.alt = "Kamerabild";
    // Rahmen um erkannte Personen beim autonomen Fahren.
    this.overlay = document.createElement("canvas");
    this.overlay.className = "cam-overlay";
    this.titel = document.createElement("span");
    this.titel.className = "cam-titel";
    this.titel.hidden = true;

    this.fehler = document.createElement("p");
    this.fehler.className = "cam-error";
    this.fehler.hidden = true;
    const nochmal = document.createElement("button");
    nochmal.type = "button";
    nochmal.textContent = "Erneut versuchen";
    nochmal.addEventListener("click", () => {
      ladeKameras(); // vielleicht ist die Kamera inzwischen da
      this.start(this.name);
    });
    this.fehler.append("Kein Bild. ", nochmal);

    this.el.append(this.img, this.overlay, this.titel, this.fehler);
    this.el.addEventListener("click", (ev) => personAntippen(this, ev));

    this.name = null; // null = Standardkamera über /camera/...
    this.modus = "aus";
    this.timer = null;
    this.pruefung = null;

    this.img.addEventListener("error", () => {
      if (this.modus !== "mjpeg") return; // Einzelbildmodus hat eigene Behandlung
      this.fehler.hidden = false;
      logLine(`Kamerabild abgerissen${this.name ? ` (${this.name})` : ""}`, "warning");
    });
  }

  /* Ohne Kameranamen die alten Endpunkte -- die kennt auch ein Server
     ohne /cameras. */
  basis() {
    const tiefe = tiefenAnsichten.find((a) => a.name === this.name);
    if (tiefe) return tiefe.basis;
    return this.name ? `/camera/${encodeURIComponent(this.name)}` : "/camera";
  }

  stop() {
    clearTimeout(this.timer);
    clearTimeout(this.pruefung);
    this.timer = this.pruefung = null;
    this.modus = "aus";
    this.img.onload = null;
    this.img.onerror = null;
    // Die Quelle wirklich wegnehmen: ein entferntes <img> hielte seinen
    // MJPEG-Strom sonst offen, und der Browser erlaubt nur wenige
    // gleichzeitige Verbindungen zum selben Rechner.
    this.img.src = "data:,";
  }

  /*
   * Der Zeitstempel in der URL ist nötig, weil der Browser sonst die
   * abgebrochene Antwort aus dem Cache nimmt und das Bild schwarz bleibt.
   */
  start(name) {
    this.stop();
    this.name = name;
    this.fehler.hidden = true;
    this.modus = "mjpeg";
    this.gestartetMit = this.basis(); // ändert sich, sobald /cameras geladen ist
    this.img.src = `${this.basis()}/stream?t=${Date.now()}`;

    // naturalWidth > 0 heißt: mindestens ein Bild wurde dekodiert. Das
    // load-Ereignis taugt hier nicht -- bei einem Dauerstrom feuert es
    // je nach Browser nie.
    this.pruefung = setTimeout(() => {
      if (this.modus === "mjpeg" && this.img.naturalWidth === 0) {
        logLine("Dauerstrom zeigt nichts — wechsle auf Einzelbilder", "warning");
        this.einzelbilder();
      }
    }, PRUEFZEIT_MS);
  }

  einzelbilder() {
    clearTimeout(this.pruefung);
    this.fehler.hidden = true;
    this.modus = "einzelbild";

    // Das nächste Bild erst anfordern, wenn das vorige da ist. Ein fester
    // Takt würde bei langsamer Verbindung Anfragen auftürmen.
    const naechstes = () => {
      if (this.modus !== "einzelbild") return;
      this.timer = setTimeout(() => {
        this.img.src = `${this.basis()}/snapshot?t=${Date.now()}`;
      }, 1000 / EINZELBILD_FPS);
    };

    this.img.onload = naechstes;
    this.img.onerror = () => {
      this.fehler.hidden = false;
      naechstes();
    };
    this.img.src = `${this.basis()}/snapshot?t=${Date.now()}`;
  }
}

/*
 * Mehrere Kameras: /cameras sagt, welche es gibt. Angezeigt wird
 *   null     die Standardkamera (/camera/stream) -- geht auch mit einem
 *            Server, der /cameras noch nicht kennt,
 *   "alle"   alle Kameras nebeneinander,
 *   <name>   genau diese eine.
 * Die Wahl bleibt im Browser gespeichert.
 */
const ALLE = "alle";
let camAnsicht = localStorage.getItem("camAnsicht");
let camStandard = null;
let camListe = [];
let kacheln = [];
// Ansichten der Tiefenkamera (/tiefe/stream, über dem Farbbild): wie
// Kameras wählbar, aber nicht bei "alle" -- jede schaltet den Projektor
// ein, und der zieht viel Strom am USB-Hub.
let tiefenAnsichten = [];

function anzuzeigen() {
  if (camAnsicht === ALLE) {
    return camListe.length ? camListe.map((k) => k.name) : [null];
  }
  return [camAnsicht];
}

function waehleAnsicht(ansicht) {
  camAnsicht = ansicht;
  if (ansicht) localStorage.setItem("camAnsicht", ansicht);
  else localStorage.removeItem("camAnsicht");
  zeigeKameras();
}

function kameraTitel(name) {
  const info = camListe.find((c) => c.name === name)
    || tiefenAnsichten.find((c) => c.name === name);
  return info ? (info.kamera || info.name) : (name || "");
}

function zeigeKameras() {
  for (const k of kacheln) k.stop();
  kacheln = anzuzeigen().map((name) => {
    const k = new CamKachel();
    k.start(name);
    return k;
  });
  ui.camFeld.replaceChildren(...kacheln.map((k) => k.el));

  // Mehrere Bilder brauchen Platz: dann über die ganze Breite.
  const mehrere = kacheln.length > 1;
  ui.camFeld.classList.toggle("mehrere", mehrere);
  ui.camPanel.classList.toggle("wide-panel", mehrere);
  for (const k of kacheln) {
    k.titel.textContent = kameraTitel(k.name);
    k.titel.hidden = !mehrere;
  }
  markiereKamera();
  personenZeichnen();
}

function markiereKamera() {
  const aktiv = camAnsicht ?? camStandard;
  for (const b of ui.camWahl.children) {
    b.classList.toggle("aktiv", b.dataset.name === aktiv);
  }
}

function wahlKnopf(name, text, titel) {
  const b = document.createElement("button");
  b.type = "button";
  b.dataset.name = name;
  b.textContent = text;
  if (titel) b.title = titel;
  b.addEventListener("click", () => waehleAnsicht(name));
  return b;
}

async function ladeKameras() {
  let daten;
  try {
    const antwort = await fetch("/cameras", { cache: "no-store" });
    if (!antwort.ok) return;
    daten = await antwort.json();
  } catch {
    return; // Server nicht erreichbar -- beim nächsten Verbinden erneut
  }
  camListe = daten.kameras || [];
  camStandard = daten.standard;
  if (daten.personen) personenKamera = daten.personen;
  tiefenAnsichten = daten.tiefe || [];

  // Nach einem Neustart des Roboters kann die gewählte Kamera fehlen.
  if (camAnsicht && camAnsicht !== ALLE && !camListe.some((k) => k.name === camAnsicht)
      && !tiefenAnsichten.some((k) => k.name === camAnsicht)) {
    camAnsicht = null;
    localStorage.removeItem("camAnsicht");
  }

  const knoepfe = camListe.map((k) => {
    let text = k.kamera || k.name;
    if (k.name === daten.personen) text += " · Personen";
    if (!k.geoeffnet) text += " (kein Bild)";
    return wahlKnopf(k.name, text,
      k.geoeffnet ? (k.geraet || k.name) : (k.letzter_fehler || "nicht geöffnet"));
  });
  if (camListe.length > 1) {
    knoepfe.push(wahlKnopf(ALLE, camListe.length === 2 ? "Beide" : "Alle",
      "Alle Kameras nebeneinander"));
  }
  for (const t of tiefenAnsichten) {
    knoepfe.push(wahlKnopf(t.name, t.kamera,
      "Was die Tiefenkamera misst: rot nah, blau fern, magenta Hindernis"));
  }
  ui.camWahl.replaceChildren(...knoepfe);
  ui.camWahl.hidden = camListe.length + tiefenAnsichten.length < 2;

  // Nur neu aufbauen, wenn sich wirklich etwas ändert -- sonst risse
  // jedes Nachladen alle Ströme kurz ab.
  const soll = anzuzeigen().join("|");
  const ist = kacheln.map((k) => k.name).join("|");
  if (soll !== ist || kacheln.some((k) => k.gestartetMit !== k.basis())) {
    zeigeKameras();
    return;
  }
  for (const k of kacheln) k.titel.textContent = kameraTitel(k.name);
  markiereKamera();
}

// Der Stream reißt ab, wenn der Roboter neu startet oder das WLAN kurz
// weg war. Sobald die Steuerverbindung wieder steht, lohnt ein neuer
// Versuch -- sonst müsste man die Seite von Hand neu laden.
// Startet als true, weil der Stream beim Laden der Seite schon angestoßen
// wird -- sonst liefe er direkt zweimal los.
let camWasConnected = true;

function onConnectionChange(connected) {
  if (connected && !camWasConnected) {
    for (const k of kacheln) k.start(k.name);
    ladeKameras();
  }
  camWasConnected = connected;
}

// --- Einstellungen (Modal) ----------------------------------------------------

el("btn-einst").addEventListener("click", () => {
  // Handeingaben loslassen: Tasten, die beim Öffnen gedrückt waren,
  // bekämen ihr keyup sonst nicht mehr mit, und er führe weiter.
  dir.x = dir.y = dir.rot = 0;
  ui.einstDialog.showModal();
  window.einstellungenLaden?.();
});

el("einst-schliessen").addEventListener("click", () => ui.einstDialog.close());

// Klick auf den abgedunkelten Rand schließt -- wie man es gewohnt ist.
ui.einstDialog.addEventListener("click", (e) => {
  if (e.target === ui.einstDialog) ui.einstDialog.close();
});

// Nach dem Schließen die Kameraliste auffrischen: Auflösung oder
// Bildrate können sich geändert haben.
ui.einstDialog.addEventListener("close", () => ladeKameras());

// --- Start ----------------------------------------------------------------

// Steuerverbindung zuerst: sie ist das Wichtigste, und ein Fehler in der
// Kameraanzeige darf sie nicht verhindern.
applyLimits();
sendTimer = setInterval(sendDrive, 1000 / SEND_HZ);
connect();

try {
  zeigeKameras(); // sofort die gewählte Ansicht, die Liste kommt hinterher
} catch (err) {
  logLine(`Kameraanzeige gestört: ${err.message}`, "error");
}
ladeKameras();

// --- Diagnose-Punkt -------------------------------------------------------

// Ein farbiger Punkt am Diagnoseknopf, solange es offene Probleme gibt --
// so merkt man USB-Ausfälle oder schwachen Akku auch ohne die Seite zu
// öffnen. Selten abfragen: die Steuerung hat Vorrang.
async function diagnosePunkt() {
  const punkt = document.getElementById("diag-punkt");
  if (!punkt) return;
  try {
    const antwort = await fetch("/diagnose/status", { cache: "no-store" });
    if (!antwort.ok) throw new Error(`HTTP ${antwort.status}`);
    const st = await antwort.json();
    const stufe = st.gesamt.stufe;
    punkt.hidden = stufe === "ok";
    punkt.classList.toggle("kritisch", stufe === "kritisch");
    el("btn-diagnose").title = `Diagnose & Logs: ${st.gesamt.text}`;
  } catch {
    punkt.hidden = true; // keine Verbindung zeigt schon die Statusanzeige
  }
}
diagnosePunkt();
setInterval(diagnosePunkt, 10000);
