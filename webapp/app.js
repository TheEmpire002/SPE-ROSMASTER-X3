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
  cam: el("cam"),
  camError: el("cam-error"),
  camRetry: el("cam-retry"),
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
  autoZustand: el("val-auto-zustand"),
  autoFrei: el("val-auto-frei"),
  autoDauer: el("val-auto-dauer"),
  autoMenschen: el("val-auto-menschen"),
  autoKamera: el("val-auto-kamera"),
  autoText: el("auto-text"),
  hindernisse: el("hindernisse"),
  camOverlay: el("cam-overlay"),
  camWahl: el("cam-wahl"),
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
  const c = ui.camOverlay;
  if (!c) return;
  const w = c.clientWidth;
  const h = c.clientHeight;
  if (c.width !== w || c.height !== h) {
    c.width = w;
    c.height = h;
  }
  const ctx = c.getContext("2d");
  ctx.clearRect(0, 0, w, h);
  if (!personenBoxen.length || !ui.cam.naturalWidth) return;
  // Die Rahmen gehören zum Bild der Personenkamera. Über einer anderen
  // Kamera stünden sie an der falschen Stelle.
  if (personenKamera && personenKamera !== aktuelleKamera()) return;

  const bildVerh = ui.cam.naturalWidth / ui.cam.naturalHeight;
  let bw = w, bh = w / bildVerh;
  if (bh > h) {
    bh = h;
    bw = h * bildVerh;
  }
  const ox = (w - bw) / 2;
  const oy = (h - bh) / 2;

  ctx.strokeStyle = farbe("Mensch");
  ctx.fillStyle = farbe("Mensch");
  ctx.lineWidth = 2;
  ctx.font = "12px system-ui, sans-serif";
  for (const b of personenBoxen) {
    const x = ox + b.x1 * bw;
    const y = oy + b.y1 * bh;
    ctx.strokeRect(x, y, (b.x2 - b.x1) * bw, (b.y2 - b.y1) * bh);
    ctx.fillText(`Mensch ${b.score.toFixed(1)}`, x + 3, y + 14);
  }
}

window.addEventListener("resize", personenZeichnen);

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
 *   Einzelbilder  Immer wieder /camera/snapshot laden. Funktioniert
 *             überall, kostet etwas mehr Overhead.
 *
 * Erst wird MJPEG versucht. Hat der Browser nach PRUEFZEIT_MS nichts
 * dekodiert, wird automatisch auf Einzelbilder umgeschaltet.
 */
const PRUEFZEIT_MS = 4000;
const EINZELBILD_FPS = 10;

let camModus = "mjpeg";
let camTimer = null;
let camPruefung = null;

/*
 * Mehrere Kameras: /cameras sagt, welche es gibt. camName = null heißt
 * Standardkamera über /camera/stream -- das funktioniert auch mit einem
 * Server, der /cameras noch nicht kennt.
 */
let camName = null;
let camStandard = null;

const camBasis = () => (camName ? `/camera/${encodeURIComponent(camName)}` : "/camera");
const aktuelleKamera = () => camName ?? camStandard;

function markiereKamera() {
  for (const b of ui.camWahl.children) {
    b.classList.toggle("aktiv", b.dataset.name === aktuelleKamera());
  }
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
  const liste = daten.kameras || [];
  camStandard = daten.standard;
  if (daten.personen) personenKamera = daten.personen;
  // Nach einem Neustart des Roboters kann die gewählte Kamera fehlen.
  if (camName && !liste.some((k) => k.name === camName)) {
    camName = null;
    startCamera();
  }

  ui.camWahl.hidden = liste.length < 2;
  ui.camWahl.replaceChildren(...liste.map((k) => {
    const b = document.createElement("button");
    b.type = "button";
    b.dataset.name = k.name;
    let text = k.kamera || k.name;
    if (k.name === daten.personen) text += " · Personen";
    if (!k.geoeffnet) text += " (kein Bild)";
    b.textContent = text;
    b.title = k.geoeffnet ? (k.geraet || k.name) : (k.letzter_fehler || "nicht geöffnet");
    b.addEventListener("click", () => {
      camName = k.name;
      markiereKamera();
      personenZeichnen();
      startCamera();
    });
    return b;
  }));
  markiereKamera();
}

function stopCamera() {
  if (camTimer !== null) {
    clearTimeout(camTimer);
    camTimer = null;
  }
  if (camPruefung !== null) {
    clearTimeout(camPruefung);
    camPruefung = null;
  }
}

/*
 * Der Zeitstempel in der URL ist nötig, weil der Browser sonst die
 * abgebrochene Antwort aus dem Cache nimmt und das Bild schwarz bleibt.
 */
function startCamera() {
  stopCamera();
  ui.camError.hidden = true;
  camModus = "mjpeg";
  // Die Handler aus dem Einzelbildmodus abräumen, sonst löst jedes
  // geladene Bild dort die nächste Anfrage aus.
  ui.cam.onload = null;
  ui.cam.onerror = null;
  ui.cam.src = `${camBasis()}/stream?t=${Date.now()}`;

  // naturalWidth > 0 heißt: mindestens ein Bild wurde dekodiert. Das
  // load-Ereignis taugt hier nicht -- bei einem Dauerstrom feuert es
  // je nach Browser nie.
  camPruefung = setTimeout(() => {
    if (camModus === "mjpeg" && ui.cam.naturalWidth === 0) {
      logLine("Dauerstrom zeigt nichts — wechsle auf Einzelbilder", "warning");
      starteEinzelbilder();
    }
  }, PRUEFZEIT_MS);
}

function starteEinzelbilder() {
  stopCamera();
  ui.camError.hidden = true;
  camModus = "einzelbild";

  // Das nächste Bild erst anfordern, wenn das vorige da ist. Ein fester
  // Takt würde bei langsamer Verbindung Anfragen auftürmen.
  const naechstes = () => {
    if (camModus !== "einzelbild") return;
    camTimer = setTimeout(() => {
      ui.cam.src = `${camBasis()}/snapshot?t=${Date.now()}`;
    }, 1000 / EINZELBILD_FPS);
  };

  ui.cam.onload = naechstes;
  ui.cam.onerror = () => {
    ui.camError.hidden = false;
    naechstes();
  };
  ui.cam.src = `${camBasis()}/snapshot?t=${Date.now()}`;
}

ui.cam.addEventListener("error", () => {
  if (camModus !== "mjpeg") return; // Einzelbildmodus hat eigene Behandlung
  ui.camError.hidden = false;
  logLine("Kamerabild abgerissen", "warning");
});

ui.camRetry.addEventListener("click", () => {
  ladeKameras(); // vielleicht ist die Kamera inzwischen da
  startCamera();
});

// Der Stream reißt ab, wenn der Roboter neu startet oder das WLAN kurz
// weg war. Sobald die Steuerverbindung wieder steht, lohnt ein neuer
// Versuch -- sonst müsste man die Seite von Hand neu laden.
// Startet als true, weil der Stream beim Laden der Seite schon angestoßen
// wird -- sonst liefe er direkt zweimal los.
let camWasConnected = true;

function onConnectionChange(connected) {
  if (connected && !camWasConnected) {
    ladeKameras();
    startCamera();
  }
  camWasConnected = connected;
}

// --- Start ----------------------------------------------------------------

ladeKameras();
startCamera();
applyLimits();
sendTimer = setInterval(sendDrive, 1000 / SEND_HZ);
connect();
