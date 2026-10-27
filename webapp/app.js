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
  log: el("log"),
  vx: el("val-vx"),
  vy: el("val-vy"),
  omega: el("val-omega"),
  driver: el("val-driver"),
  bpm: el("val-bpm"),
  motion: el("val-motion"),
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
window.addEventListener("blur", stopAll);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) stopAll();
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

// --- Kamera ----------------------------------------------------------------

/*
 * Den Stream neu anstoßen. Der Zeitstempel in der URL ist nötig, weil der
 * Browser sonst die abgebrochene Antwort aus dem Cache nimmt und das Bild
 * schwarz bleibt.
 */
function startCamera() {
  ui.camError.hidden = true;
  ui.cam.src = `/camera/stream?t=${Date.now()}`;
}

ui.cam.addEventListener("error", () => {
  ui.camError.hidden = false;
  logLine("Kamerabild abgerissen", "warning");
});

ui.camRetry.addEventListener("click", startCamera);

// Der Stream reißt ab, wenn der Roboter neu startet oder das WLAN kurz
// weg war. Sobald die Steuerverbindung wieder steht, lohnt ein neuer
// Versuch -- sonst müsste man die Seite von Hand neu laden.
// Startet als true, weil der Stream beim Laden der Seite schon angestoßen
// wird -- sonst liefe er direkt zweimal los.
let camWasConnected = true;

function onConnectionChange(connected) {
  if (connected && !camWasConnected) startCamera();
  camWasConnected = connected;
}

// --- Start ----------------------------------------------------------------

startCamera();
applyLimits();
sendTimer = setInterval(sendDrive, 1000 / SEND_HZ);
connect();
