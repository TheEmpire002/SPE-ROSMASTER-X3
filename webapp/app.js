/*
 * Web app for remote-controlling the ROSMASTER X3.
 *
 * How it works: keys and buttons only set the direction to drive in.
 * A timer turns that into a drive command 10 times per second.
 *
 * The constant rate is intentional and not wasted traffic: the watchdog on
 * the robot stops as soon as the commands stop arriving. If the app hangs
 * or the Wi-Fi drops, the robot therefore stops by itself.
 */

const SEND_HZ = 10;

const RECONNECT_MS = 1500;

/*
 * Upper limits. The server sends them right after connecting and
 * overwrites these values -- they are only what we work with during the
 * first second. The limits are configured in config.py on the robot,
 * not here.
 */
const limits = { vx: 0.4, vy: 0.4, omega: 1.5 };

/*
 * User-chosen maximum in m/s. Always <= limits.vx -- the robot caps to its
 * own limit anyway, but the app should not even request more, otherwise
 * the display no longer matches the actual motion.
 *
 * Stored in the browser so the setting survives a reload -- otherwise you
 * would be back at full throttle after every reload.
 */
const STORAGE_KEY = "rosmaster.maxSpeed";
let userMax = 0.4;

// --- State -------------------------------------------------------------------

/** Direction as -1/0/1 per axis; multiplied by the speed. */
const dir = { x: 0, y: 0, rot: 0 };

let speed = 0.4;          // 0..1
let socket = null;
let sendTimer = null;

// --- User interface ----------------------------------------------------------

const el = (id) => document.getElementById(id);

const ui = {
  status: el("status"),
  statusText: el("status-text"),
  speed: el("speed"),
  speedOut: el("speed-out"),
  maxSpeed: el("maxspeed"),
  maxSpeedOut: el("maxspeed-out"),
  camPanel: el("cam-panel"),
  camGrid: el("cam-feld"),
  settingsDialog: el("einst-dialog"),
  dist: el("val-dist"),
  pos: el("val-pos"),
  heading: el("val-winkel"),
  btnHome: el("btn-home"),
  btnSetHome: el("btn-sethome"),
  collision: el("collision"),
  collisionRow: el("collision-row"),
  collisionDist: el("collision-dist"),
  collisionOut: el("collision-out"),
  collisionState: el("collision-state"),
  log: el("log"),
  camSelect: el("cam-wahl"),
  btnInteraction: el("btn-interaktion"),
  btnFollow: el("btn-folgen"),
  iaMode: el("val-ia-modus"),
  iaPerson: el("val-ia-person"),
  iaGesture: el("val-ia-geste"),
  iaTarget: el("val-ia-ziel"),
  btnIaPause: el("btn-ia-pause"),
  btnIaRelease: el("btn-ia-loslassen"),
  iaText: el("ia-text"),
  voice: el("val-stimme"),
  speech: el("val-sprache"),
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

// --- Connection ---------------------------------------------------------------

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
    // onclose follows right after and takes care of reconnecting.
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
      showPose(msg.pose);
      setHomeButton(msg.homing === true);
      showCollision(msg.collision);
      showAutonomy(msg.autonomie);
      showInteraction(msg.interaktion);
      showVoice(msg.stimme);
      showSpeech(msg.sprache);
      showCameraTilt(msg.kamera_neigung);
      break;

    case "collision":
      showCollision(msg);
      break;

    case "vitals":
      break; // pulse panel removed, the esp still sends them

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

// --- Camera tilt (servo of the person camera) ----------------------------------

// while dragging, the slider shows the hand, not the robot's (older) value
let tiltDragging = false;
let tiltSent = 0;

function showCameraTilt(st) {
  const on = !!(st && st.verfuegbar);
  el("neigung").hidden = !on;
  el("neigung-auto-zeile").hidden = !on;
  if (!on) {
    el("val-neigung").textContent = "—";
    return;
  }
  const deg = Math.round(st.winkel);
  const where = deg > 0 ? `${deg}° hoch` : deg < 0 ? `${-deg}° runter` : "waagerecht";
  el("val-neigung").textContent = where + (st.auto ? (st.verfolgt ? " · folgt der Person" : " · automatisch") : "");
  const slider = el("neigung-regler");
  slider.min = st.min;
  slider.max = st.max;
  if (!tiltDragging) slider.value = st.auto ? st.winkel : st.ziel;
  el("neigung-auto").checked = st.auto === true;
  el("neigung-auto-state").textContent = st.auto ? "an" : "aus";
}

function sendTilt(force) {
  // at most ten per second while dragging; the robot moves smoothly in between
  const now = Date.now();
  if (!force && now - tiltSent < 100) return;
  tiltSent = now;
  send({ type: "camera_tilt", winkel: Number(el("neigung-regler").value) });
}
el("neigung-regler").addEventListener("input", () => { tiltDragging = true; sendTilt(false); });
el("neigung-regler").addEventListener("change", () => { sendTilt(true); tiltDragging = false; });
el("btn-neigung-gerade").addEventListener("click", () => send({ type: "camera_tilt", winkel: 0 }));
el("neigung-auto").addEventListener("change", (e) => send({ type: "camera_tilt", auto: e.target.checked }));

function send(message) {
  if (socket && socket.readyState === WebSocket.OPEN) {
    socket.send(JSON.stringify(message));
  }
}

// --- Drive commands -----------------------------------------------------------

function sendDrive() {
  // Sideways and rotation are throttled in the same ratio as forward.
  // Otherwise a tame max speed would still lead to a robot that spins
  // frantically around its own axis.
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

// --- Buttons (mouse and touch) --------------------------------------------------

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
  // Pointer events cover mouse, touch and pen together.
  btn.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    // Makes sure pointerup still arrives when the finger leaves the
    // button -- otherwise the robot keeps driving.
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

// --- Keyboard -------------------------------------------------------------------

// u/i held: tilt the camera in small steps, the robot moves the servo
// smoothly in between
const TILT_KEY_STEP_DEG = 3;
const TILT_KEY_EVERY_MS = 100;
let tiltTimer = null;

function startTilt(sign) {
  stopTilt();
  const step = () => send({ type: "camera_tilt", schritt: sign * TILT_KEY_STEP_DEG });
  step();
  tiltTimer = setInterval(step, TILT_KEY_EVERY_MS);
}

function stopTilt() {
  if (tiltTimer !== null) clearInterval(tiltTimer);
  tiltTimer = null;
}

const KEYS = {
  KeyU: () => startTilt(1),
  KeyI: () => startTilt(-1),
  KeyW: () => (dir.x = 1),
  KeyS: () => (dir.x = -1),
  KeyA: () => (dir.y = 1),
  KeyD: () => (dir.y = -1),
  KeyQ: () => (dir.rot = 1),
  KeyE: () => (dir.rot = -1),
};

const KEYS_UP = {
  KeyU: stopTilt,
  KeyI: stopTilt,
  KeyW: () => (dir.x = 0),
  KeyS: () => (dir.x = 0),
  KeyA: () => (dir.y = 0),
  KeyD: () => (dir.y = 0),
  KeyQ: () => (dir.rot = 0),
  KeyE: () => (dir.rot = 0),
};

window.addEventListener("keydown", (e) => {
  if (e.repeat) return;
  // shortcuts are not driving: alt+d (address bar), ctrl+s, ctrl+a and the
  // like reach the page as d, s, a too
  if (e.ctrlKey || e.altKey || e.metaKey) return;
  // In an open dialog (settings, MUKKE) the user is typing -- an "e" in a
  // number field or a "w" in a YouTube link must not move the robot.
  // Same in text fields (except sliders, which you use to set the speed
  // on the side).
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

// leaving the window or switching tabs ends manual input: a key held down
// would never get its keyup and the robot would keep driving. only then a
// stop goes out; a goal drive, 3d scan, homing or autonomous driving that
// nobody steers by hand goes on (switching to /map must not end it). if the
// connection drops, the server stops the robot regardless
function manualInput() {
  return dir.x !== 0 || dir.y !== 0 || dir.rot !== 0
    || document.querySelector(".pad-btn.active") !== null;
}

function onWindowLeave() {
  stopTilt();
  const steering = manualInput();
  dir.x = dir.y = dir.rot = 0;
  document.querySelectorAll(".pad-btn.active").forEach((b) =>
    b.classList.remove("active")
  );
  if (steering) send({ type: "stop" });
}
window.addEventListener("blur", onWindowLeave);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) onWindowLeave();
});

// --- Speed ------------------------------------------------------------------------

ui.speed.addEventListener("input", () => {
  speed = Number(ui.speed.value) / 100;
  updateLabels();
});

ui.maxSpeed.addEventListener("input", () => {
  userMax = Number(ui.maxSpeed.value);
  try {
    localStorage.setItem(STORAGE_KEY, String(userMax));
  } catch {
    // Private mode or similar -- then the setting only lasts this session.
  }
  updateLabels();
});

function updateLabels() {
  ui.maxSpeedOut.textContent = `${userMax.toFixed(2)} m/s`;
  // A percentage alone says little, so add the resulting speed.
  ui.speedOut.textContent =
    `${ui.speed.value} % · ${(userMax * speed).toFixed(2)} m/s`;
}

/**
 * Applies the robot's limits to the max-speed slider.
 *
 * Runs only once the server has reported its limits -- before that the
 * app does not know how far the slider may go at all.
 */
function applyLimits() {
  ui.maxSpeed.max = String(limits.vx);

  let wanted = limits.vx;
  try {
    const stored = Number(localStorage.getItem(STORAGE_KEY));
    if (stored > 0) wanted = stored;
  } catch {
    // no localStorage -- then the robot limit applies
  }

  // Never above what the robot allows: a stored value may come from a
  // configuration with a higher limit.
  userMax = Math.min(wanted, limits.vx);
  ui.maxSpeed.value = String(userMax);
  updateLabels();
}

// --- Collision avoidance -----------------------------------------------------------

/*
 * The setting belongs to the server, not the browser: it affects safety,
 * and with several open tabs it must be clear which value applies.
 * So the controls only send requests and then display what the server
 * reports back.
 */
function sendCollision() {
  send({
    type: "collision",
    enabled: ui.collision.checked,
    distance: Number(ui.collisionDist.value),
  });
}

ui.collision.addEventListener("change", sendCollision);
ui.collisionDist.addEventListener("change", sendCollision);
ui.collisionDist.addEventListener("input", () => {
  ui.collisionOut.textContent = `${Number(ui.collisionDist.value).toFixed(2)} m`;
});

function showCollision(c) {
  if (!c) return;
  ui.collision.checked = c.enabled === true;
  ui.collisionRow.hidden = !c.enabled;
  if (typeof c.distance === "number") {
    ui.collisionDist.value = String(c.distance);
    ui.collisionOut.textContent = `${c.distance.toFixed(2)} m`;
  }

  let text = "aus";
  let cls = "switch-state";
  if (c.enabled && c.verfuegbar === false) {
    // Switched on without a sensor would be deceptive -- that must be visible.
    text = "kein Lidar!";
    cls += " blockiert";
  } else if (c.blockiert) {
    text = "hält";
    cls += " blockiert";
  } else if (c.enabled) {
    text = "an";
    cls += " an";
  }
  ui.collisionState.textContent = text;
  ui.collisionState.className = cls;
}

// --- Autonomous driving --------------------------------------------------------

/*
 * The server sends the state with every state message
 * (state.autonomie). The panel with the explore button is gone; driving
 * autonomously starts by voice („Robi, erkunde“), and
 * the app only keeps the state it still needs.
 */
let autonomyActive = false;
let personBoxes = [];
// Which camera image the person boxes belong to (reported by the server).
// null = unknown, e.g. with an older server -- then always, as before.
let personCamera = null;

const PERSON_COLOR = "#f05d5d";

// the panel is gone, but the person boxes over the camera still need the
// autonomy state
function showAutonomy(a) {
  if (!a) return;
  autonomyActive = a.aktiv === true;
  personBoxes = a.personen || [];
  const pe = a.personenerkennung;
  if (pe && pe.kamera) personCamera = pe.kamera;
  drawPersons();
}

/*
 * Person boxes over the camera image. The image is fitted with object-fit:
 * contain, so there may be margins left/right or top/bottom -- the boxes
 * must go onto the actual image area.
 */
function drawPersons() {
  tiles.forEach((t, i) => {
    // person boxes only over the colour image, not over a depth/object view
    const matches = !t.view && showsPersonCamera(t, i);
    drawBoxes(t, matches ? personBoxes : [], matches);
  });
}

// The boxes belong to the person camera's image. Over another camera
// they would be in the wrong place. If the server does not know a
// person camera (older version), then over the first one as before.
function showsPersonCamera(tile, i) {
  return personCamera
    ? (tile.name ?? camDefault) === personCamera
    : i === 0;
}

/** Where the image lies within the tile (fitted, with margins). */
function imageArea(tile, w, h) {
  const img = tile.img;
  const aspect = img.naturalWidth / img.naturalHeight;
  let bw = w, bh = w / aspect;
  if (bh > h) {
    bh = h;
    bw = h * aspect;
  }
  return { ox: (w - bw) / 2, oy: (h - bh) / 2, bw, bh };
}

// showsPersons: this tile shows the person camera -- only there do boxes
// and skeletons belong (both come from the C270).
function drawBoxes(tile, boxes, showsPersons) {
  const c = tile.overlay;
  const img = tile.img;
  const w = c.clientWidth;
  const h = c.clientHeight;
  if (c.width !== w || c.height !== h) {
    c.width = w;
    c.height = h;
  }
  const ctx = c.getContext("2d");
  ctx.clearRect(0, 0, w, h);
  const skeletonHere = showsPersons && skeletons.length > 0;
  if ((!boxes.length && !skeletonHere) || !img.naturalWidth) return;

  const { ox, oy, bw, bh } = imageArea(tile, w, h);

  ctx.strokeStyle = PERSON_COLOR;
  ctx.fillStyle = PERSON_COLOR;
  ctx.lineWidth = 2;
  ctx.font = "12px system-ui, sans-serif";
  for (const b of boxes) {
    const x = ox + b.x1 * bw;
    const y = oy + b.y1 * bh;
    ctx.strokeRect(x, y, (b.x2 - b.x1) * bw, (b.y2 - b.y1) * bh);
    ctx.fillText(`Mensch ${b.score.toFixed(1)}`, x + 3, y + 14);
  }

  if (skeletonHere) drawSkeletons(ctx, ox, oy, bw, bh);
}

window.addEventListener("resize", drawPersons);

// --- Interaction --------------------------------------------------------------

/*
 * As with autonomous driving: the robot decides, the app displays and
 * sends interaction_start / interaction_stop / follow / follow_stop.
 * It only fetches the skeletons while the interaction is running --
 * otherwise the Hailo is not computing anyway.
 *
 * Target lock: tapping a person sends follow with their track ID. Only
 * this way does a specific one become the target when there are several
 * people; the robot never switches by itself. Pause stops and keeps the
 * target, release frees it.
 */
const POSE_HZ = 8;
/** Keypoints below this confidence are not drawn. */
const KP_MIN = 0.3;

const MODE_TEXT = {
  bereit: "wartet auf Handzeichen",
  folgen: "folgt",
  herkommen: "kommt her",
  weggehen: "weicht zurück",
  sucht: "sucht",
};

/** States of the target lock (state.interaktion.ziel.zustand). */
const TARGET_TEXT = {
  idle: () => "kein Ziel",
  acquiring_target: () => "sucht sich die Person …",
  following: () => "folgt",
  target_temporarily_lost: () => "Ziel kurz verloren",
  searching_for_target: () => "sucht die Person …",
  // Only reported by servers from before 04.11.
  safe_stop: () => "sicher angehalten",
};

let interactionActive = false;
let interactionMode = null;
let interactionPaused = false;
let skeletons = [];
let skeletonLines = [];
let poseFetchRunning = false;

function setInteractionButtons(active, mode) {
  interactionActive = active;
  interactionMode = mode;
  ui.btnInteraction.textContent = active ? "Interaktion aus" : "Interaktion an";
  ui.btnInteraction.classList.toggle("aktiv", active);
  const following = active && mode === "folgen";
  ui.btnFollow.textContent = following ? "Nicht mehr folgen" : "Folge mir";
}

ui.btnInteraction.addEventListener("click", () => {
  send({ type: interactionActive ? "interaction_stop" : "interaction_start" });
  // Toggle immediately, the next status message sets it straight.
  setInteractionButtons(!interactionActive, interactionActive ? null : "bereit");
});

ui.btnFollow.addEventListener("click", () => {
  const following = interactionActive && interactionMode === "folgen";
  send({ type: following ? "follow_stop" : "follow" });
  setInteractionButtons(true, following ? "bereit" : "folgen");
});

ui.btnIaPause.addEventListener("click", () => {
  send({ type: interactionPaused ? "follow_resume" : "follow_pause" });
});

ui.btnIaRelease.addEventListener("click", () => send({ type: "follow_stop" }));

function showTarget(ia) {
  const z = ia.ziel;
  const driving = ia.aktiv === true && !!ia.modus && ia.modus !== "bereit";
  interactionPaused = ia.pausiert === true;
  ui.btnIaPause.textContent = interactionPaused ? "Weiter" : "Pause";
  ui.btnIaPause.classList.toggle("aktiv", interactionPaused);
  ui.btnIaPause.disabled = !driving;
  // Older server without target lock: no target display.
  if (!z || !ia.aktiv) {
    ui.iaTarget.textContent = "—";
    ui.btnIaRelease.disabled = !driving;
    return;
  }
  let text = (TARGET_TEXT[z.zustand] || (() => z.zustand))();
  if (interactionPaused) text += " · pausiert";
  if (z.grund && (z.zustand === "idle" || z.zustand === "safe_stop")) text += ` (${z.grund})`;
  ui.iaTarget.textContent = text;
  ui.btnIaRelease.disabled = !driving && z.zustand === "idle";
}

function showInteraction(ia) {
  if (!ia) return;
  setInteractionButtons(ia.aktiv === true, ia.modus);
  ui.btnInteraction.disabled = ui.btnFollow.disabled = ia.verfuegbar === false;
  showTarget(ia);

  ui.iaMode.textContent = ia.aktiv
    ? (MODE_TEXT[ia.modus] || ia.modus)
    : (ia.verfuegbar === false ? "nicht verfügbar" : "aus");

  const p = ia.person;
  if (p) {
    const side = Math.abs(p.winkel) < 5 ? "mittig" : p.winkel > 0 ? "links" : "rechts";
    const distance = typeof p.abstand === "number" ? `${p.abstand.toFixed(2)} m · ` : "";
    ui.iaPerson.textContent = `${distance}${side} (${p.winkel.toFixed(0)}°) · ${ia.personen} im Bild`;
  } else {
    ui.iaPerson.textContent = ia.aktiv ? `${ia.personen} im Bild` : "—";
  }
  ui.iaGesture.textContent = ia.geste || "—";

  let text = ia.text || "—";
  if (!ia.aktiv && ia.grund) text = `Zuletzt beendet: ${ia.grund}`;
  if (ia.verfuegbar === false) text = "Braucht Lidar und Posenerkennung — Details unter /interaktion/status";
  ui.iaText.textContent = text;

  if (!ia.aktiv && skeletons.length) {
    skeletons = [];
    drawPersons();
  }
}

// --- Voice ------------------------------------------------------------------

/*
 * The robot speaks by itself (gestures, mode changes); here you can also
 * have your own sentence read out. Without a speaker it only ends up in
 * the robot's log -- the display shows that.
 */
ui.sayForm.addEventListener("submit", (ev) => {
  ev.preventDefault();
  const text = ui.sayText.value.trim();
  if (!text) return;
  send({ type: "say", text });
  ui.sayText.value = "";
});

// As if spoken into the microphone -- for trying things out without a microphone.
ui.speechForm.addEventListener("submit", (ev) => {
  ev.preventDefault();
  const text = ui.speechText.value.trim();
  if (!text) return;
  send({ type: "speech", text });
  ui.speechText.value = "";
});

function showSpeech(sp) {
  if (!sp) return;
  if (sp.pause) {
    ui.speech.textContent = `Mikrofon aus: ${sp.pause}`;
  } else if (!sp.verfuegbar) {
    ui.speech.textContent = "kein Mikrofon (/sprache/status)";
  } else if (sp.befehl) {
    ui.speech.textContent = `Befehl: ${sp.befehl}`;
  } else if (sp.geweckt) {
    ui.speech.textContent = "hört zu …";
  } else {
    ui.speech.textContent = sp.gehoert ? `gehört: „${sp.gehoert}“` : "wartet auf „Robi“";
  }
}

function showVoice(st) {
  if (!st) return;
  if (st.verfuegbar === false) {
    ui.voice.textContent = "kein Lautsprecher (/stimme/status)";
  } else if (st.spricht) {
    ui.voice.textContent = `spricht: „${st.text || ""}“`;
  } else {
    ui.voice.textContent = st.text ? `zuletzt: „${st.text}“` : "bereit";
  }
}

async function fetchPose() {
  if (!interactionActive || poseFetchRunning) return;
  poseFetchRunning = true; // do not pile up requests when the Wi-Fi hangs
  try {
    const r = await fetch("/interaktion/pose", { cache: "no-store" });
    if (r.ok) {
      const d = await r.json();
      skeletons = d.personen || [];
      skeletonLines = d.skelett || [];
      drawPersons();
    }
  } catch {
    // Connection briefly gone: try again on the next tick.
  } finally {
    poseFetchRunning = false;
  }
}

setInterval(fetchPose, 1000 / POSE_HZ);

/** Skeletons onto the fitted image area (ox/oy/bw/bh from drawPersons). */
function drawSkeletons(ctx, ox, oy, bw, bh) {
  if (!skeletons.length) return;
  ctx.lineWidth = 3;
  ctx.font = "13px system-ui, sans-serif";
  // The person it follows green, all others grey. Older servers without
  // "ziel" -- then the largest person (front) as before.
  const withTarget = skeletons.some((s) => "ziel" in s);
  skeletons.forEach((s, i) => {
    const isTarget = withTarget ? s.ziel === true : i === 0;
    const col = isTarget ? "#3ddc97" : "#9aa1ad";
    ctx.strokeStyle = col;
    ctx.fillStyle = col;
    const kp = s.kp || [];
    for (const [a, b] of skeletonLines) {
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

// --- Return home --------------------------------------------------------------

let homing = false;

function showPose(pose) {
  if (!pose) {
    ui.dist.textContent = ui.pos.textContent = ui.heading.textContent = "—";
    return;
  }
  const distance = Math.hypot(pose.x, pose.y);
  // Without measurement the position comes only from the drive commands.
  // You need to see that before trusting the home button.
  const quality = pose.gemessen === false ? " (geschätzt)" : "";
  ui.dist.textContent = `${distance.toFixed(2)} m${quality}`;
  ui.pos.textContent = `${pose.x.toFixed(2)} / ${pose.y.toFixed(2)} m`;
  ui.heading.textContent = `${pose.winkel.toFixed(0)}°`;
}

/*
 * The same button starts and cancels. Two separate buttons would be a
 * risk: whoever wants to stop the robot should not have to look for the
 * right one first.
 */
function setHomeButton(active) {
  if (active === homing) return;
  homing = active;
  ui.btnHome.textContent = active ? "Heimfahrt abbrechen" : "Nach Hause";
  ui.btnHome.classList.toggle("aktiv", active);
}

ui.btnHome.addEventListener("click", () => {
  if (homing) {
    send({ type: "home_cancel" });
  } else {
    send({ type: "home" });
  }
  // Do not wait for the response: the button should react immediately. The
  // next status message sets the display straight if something went wrong.
  setHomeButton(!homing);
});

ui.btnSetHome.addEventListener("click", () => {
  send({ type: "set_home" });
});

// --- Camera ----------------------------------------------------------------

/*
 * Two ways to get the image:
 *
 *   MJPEG     A continuous stream in an <img>. Economical, but not every
 *             browser handles multipart/x-mixed-replace -- Safari on
 *             iPhone and iPad then simply shows nothing.
 *   Snapshots Load .../snapshot over and over. Works everywhere, costs
 *             a bit more overhead.
 *
 * MJPEG is tried first. If the browser has decoded nothing after
 * CHECK_TIME_MS, it automatically switches to snapshots.
 *
 * Each displayed camera is a tile of its own with its own image, its own
 * error display and its own box overlay. That way all cameras can also
 * be shown side by side.
 */
const CHECK_TIME_MS = 4000;
const SNAPSHOT_FPS = 10;

class CamTile {
  constructor() {
    this.el = document.createElement("div");
    this.el.className = "cam-wrap";

    // MJPEG: the browser displays the stream directly, without JavaScript.
    this.img = document.createElement("img");
    this.img.className = "cam";
    this.img.alt = "Kamerabild";
    // Boxes around detected persons during autonomous driving.
    this.overlay = document.createElement("canvas");
    this.overlay.className = "cam-overlay";
    this.label = document.createElement("span");
    this.label.className = "cam-titel";
    this.label.hidden = true;

    this.errorBox = document.createElement("p");
    this.errorBox.className = "cam-error";
    this.errorBox.hidden = true;
    const retry = document.createElement("button");
    retry.type = "button";
    retry.textContent = "Erneut versuchen";
    retry.addEventListener("click", () => {
      loadCameras(); // maybe the camera is there by now
      this.start(this.name, this.view);
    });
    this.errorBox.append("Kein Bild. ", retry);

    this.el.append(this.img, this.overlay, this.label, this.errorBox);

    this.name = null; // null = default camera via /camera/...
    this.view = null; // depth or object view shown instead of the colour image
    this.mode = "aus";
    this.timer = null;
    this.checkTimer = null;

    this.img.addEventListener("error", () => {
      if (this.mode !== "mjpeg") return; // snapshot mode has its own handling
      this.errorBox.hidden = false;
      logLine(`Kamerabild abgerissen${this.name ? ` (${this.name})` : ""}`, "warning");
    });
  }

  /* Without a camera name the old endpoints -- a server without /cameras
     knows those too. */
  base() {
    const depth = depthViews.find((a) => a.name === this.view);
    if (depth) return depth.basis;
    return this.name ? `/camera/${encodeURIComponent(this.name)}` : "/camera";
  }

  stop() {
    clearTimeout(this.timer);
    clearTimeout(this.checkTimer);
    this.timer = this.checkTimer = null;
    this.mode = "aus";
    this.img.onload = null;
    this.img.onerror = null;
    // Really remove the source: a removed <img> would otherwise keep its
    // MJPEG stream open, and the browser only allows a few simultaneous
    // connections to the same host.
    this.img.src = "data:,";
  }

  /*
   * The timestamp in the URL is needed because the browser would otherwise
   * take the aborted response from the cache and the image would stay black.
   */
  start(name, view = null) {
    this.stop();
    this.name = name;
    this.view = view;
    this.errorBox.hidden = true;
    this.mode = "mjpeg";
    this.startedWith = this.base(); // changes as soon as /cameras is loaded
    this.img.src = `${this.base()}/stream?t=${Date.now()}`;

    // naturalWidth > 0 means: at least one frame was decoded. The load
    // event is no good here -- with a continuous stream it never fires
    // in some browsers.
    this.checkTimer = setTimeout(() => {
      if (this.mode === "mjpeg" && this.img.naturalWidth === 0) {
        logLine("Dauerstrom zeigt nichts — wechsle auf Einzelbilder", "warning");
        this.startSnapshots();
      }
    }, CHECK_TIME_MS);
  }

  startSnapshots() {
    clearTimeout(this.checkTimer);
    this.errorBox.hidden = true;
    this.mode = "einzelbild";

    // Request the next image only once the previous one has arrived. A
    // fixed rate would pile up requests on a slow connection.
    const next = () => {
      if (this.mode !== "einzelbild") return;
      this.timer = setTimeout(() => {
        this.img.src = `${this.base()}/snapshot?t=${Date.now()}`;
      }, 1000 / SNAPSHOT_FPS);
    };

    this.img.onload = next;
    this.img.onerror = () => {
      this.errorBox.hidden = false;
      next();
    };
    this.img.src = `${this.base()}/snapshot?t=${Date.now()}`;
  }
}

/*
 * Several cameras: /cameras says which exist, and all of them are always
 * shown side by side. The mode (colour, depth, depth + colour, objects)
 * only changes the tile of the camera the view belongs to (its "quelle");
 * the other cameras keep their colour image. The mode is stored in the
 * browser.
 */
const COLOR_MODE = "farbe";
let camMode = localStorage.getItem("camModus") || COLOR_MODE;
let camDefault = null;
let camList = [];
let tiles = [];
// views beside the colour images: depth images and the object view, each
// with its own address and the camera it replaces
let depthViews = [];

function camerasToShow() {
  return camList.length ? camList.map((k) => k.name) : [null];
}

/* Which view a camera tile shows in the current mode (null = colour). */
function viewFor(name, shown) {
  const v = depthViews.find((d) => d.name === camMode);
  if (!v) return null;
  const source = v.quelle ?? camDefault;
  // source camera missing (no colour image): the first tile takes the view
  const target = shown.includes(source) ? source : shown[0];
  return (name ?? camDefault) === (target ?? camDefault) ? v.name : null;
}

function selectMode(mode) {
  camMode = mode;
  localStorage.setItem("camModus", mode);
  showCameras();
}

function cameraTitle(name, view) {
  const info = camList.find((c) => c.name === name);
  const title = info ? (info.kamera || info.name) : (name || "");
  const v = depthViews.find((c) => c.name === view);
  return v ? `${title} · ${v.kamera}` : title;
}

function showCameras() {
  for (const k of tiles) k.stop();
  const shown = camerasToShow();
  tiles = shown.map((name) => {
    const k = new CamTile();
    k.start(name, viewFor(name, shown));
    return k;
  });
  ui.camGrid.replaceChildren(...tiles.map((k) => k.el));

  // Several images need space: then use the full width.
  const several = tiles.length > 1;
  ui.camGrid.classList.toggle("mehrere", several);
  ui.camPanel.classList.toggle("wide-panel", several);
  for (const k of tiles) {
    k.label.textContent = cameraTitle(k.name, k.view);
    k.label.hidden = !several && !k.view;
  }
  markCamera();
  drawPersons();
}

function markCamera() {
  for (const b of ui.camSelect.children) {
    b.classList.toggle("aktiv", b.dataset.name === camMode);
  }
}

function selectButton(name, text, title) {
  const b = document.createElement("button");
  b.type = "button";
  b.dataset.name = name;
  b.textContent = text;
  if (title) b.title = title;
  b.addEventListener("click", () => selectMode(name));
  return b;
}

async function loadCameras() {
  let data;
  try {
    const response = await fetch("/cameras", { cache: "no-store" });
    if (!response.ok) return;
    data = await response.json();
  } catch {
    return; // server not reachable -- try again on the next connect
  }
  camList = data.kameras || [];
  camDefault = data.standard;
  if (data.personen) personCamera = data.personen;
  depthViews = [...(data.tiefe || []), ...(data.objekte || [])];

  // after a robot restart the selected view may be missing
  if (camMode !== COLOR_MODE && !depthViews.some((k) => k.name === camMode)) {
    camMode = COLOR_MODE;
    localStorage.setItem("camModus", camMode);
  }

  const buttons = [selectButton(COLOR_MODE, "Farbe", "Farbbild aller Kameras")];
  for (const t of depthViews) {
    buttons.push(selectButton(t.name, t.kamera, t.name === "objekte"
      ? "Was die Kamera-KI erkennt: Rahmen und Namen um Stühle, Tische, Menschen …"
      : "Was die Tiefenkamera misst: rot nah, blau fern, magenta Hindernis"));
  }
  ui.camSelect.replaceChildren(...buttons);
  ui.camSelect.hidden = depthViews.length === 0;

  // Only rebuild if something really changes -- otherwise every reload
  // would briefly tear down all streams.
  const shown = camerasToShow();
  const wanted = shown.map((n) => `${n}:${viewFor(n, shown)}`).join("|");
  const current = tiles.map((k) => `${k.name}:${k.view}`).join("|");
  if (wanted !== current || tiles.some((k) => k.startedWith !== k.base())) {
    showCameras();
    return;
  }
  for (const k of tiles) k.label.textContent = cameraTitle(k.name, k.view);
  markCamera();
}

// The stream breaks off when the robot restarts or the Wi-Fi was briefly
// gone. As soon as the control connection is back, a new attempt is
// worthwhile -- otherwise you would have to reload the page by hand.
// Starts as true because the stream is already kicked off when the page
// loads -- otherwise it would start twice right away.
let camWasConnected = true;

function onConnectionChange(connected) {
  if (connected && !camWasConnected) {
    for (const k of tiles) k.start(k.name, k.view);
    loadCameras();
  }
  camWasConnected = connected;
}

// --- Settings (modal) ---------------------------------------------------------

el("btn-einst").addEventListener("click", () => {
  // Release manual inputs: keys held down when opening would otherwise
  // never get their keyup, and the robot would keep driving.
  dir.x = dir.y = dir.rot = 0;
  ui.settingsDialog.showModal();
  window.einstellungenLaden?.();
});

el("einst-schliessen").addEventListener("click", () => ui.settingsDialog.close());

// Clicking the dimmed backdrop closes it -- as people are used to.
ui.settingsDialog.addEventListener("click", (e) => {
  if (e.target === ui.settingsDialog) ui.settingsDialog.close();
});

// After closing, refresh the camera list: resolution or frame rate may
// have changed.
ui.settingsDialog.addEventListener("close", () => loadCameras());

// --- Start ----------------------------------------------------------------

// Control connection first: it is the most important thing, and an error
// in the camera display must not prevent it.
applyLimits();
sendTimer = setInterval(sendDrive, 1000 / SEND_HZ);
connect();

try {
  showCameras(); // show the selected view right away, the list follows
} catch (err) {
  logLine(`Kameraanzeige gestört: ${err.message}`, "error");
}
loadCameras();

// --- Diagnostics dot -------------------------------------------------------

// A colored dot on the diagnostics button as long as there are open
// problems -- so you notice USB failures or a weak battery without opening
// the page. Poll rarely: driving control has priority.
async function diagnosticsDot() {
  const dot = document.getElementById("diag-punkt");
  if (!dot) return;
  try {
    const response = await fetch("/diagnose/status", { cache: "no-store" });
    if (!response.ok) throw new Error(`HTTP ${response.status}`);
    const st = await response.json();
    const level = st.gesamt.stufe;
    dot.hidden = level === "ok";
    dot.classList.toggle("kritisch", level === "kritisch");
    el("btn-diagnose").title = `Diagnose & Logs: ${st.gesamt.text}`;
  } catch {
    dot.hidden = true; // no connection is already shown by the status display
  }
}
diagnosticsDot();
setInterval(diagnosticsDot, 10000);
