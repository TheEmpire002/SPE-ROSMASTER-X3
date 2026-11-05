/*
 * Live microphone: read along (what the speech recognition understands) and
 * listen in (the robot microphone's audio in the browser). Each can be
 * switched separately and stays off until someone turns it on -- then it
 * costs nothing on the robot.
 *
 * Reading along polls GET /sprache/live, listening in uses the WebSocket
 * /sprache/ton (first {"rate": Hz}, then raw 16-bit mono audio as binary data).
 * Wrapped in a function so the names do not collide with app.js.
 */

(() => {

const $ = (id) => document.getElementById(id);
const readToggle = $("mikro-lesen");
const readState = $("mikro-lesen-state");
const listenToggle = $("mikro-hoeren");
const listenState = $("mikro-hoeren-state");
const live = $("mikro-live");
const levelBar = $("mikro-pegel-balken");
const partial = $("mikro-teil");
const eventList = $("mikro-ereignisse");

/** Poll interval while reading along. The robot delivers the level 4x per second. */
const POLL_MS = 250;
/** Listening in: schedule this far ahead and lag behind at most this much. */
const LEAD_S = 0.15;
const MAX_LAG_S = 1.0;
/** The robot reports at least once per second (audio or heartbeat).
 *  If nothing arrives for this long, the connection hangs: reconnect. */
const SILENCE_MS = 5000;
/** Reading along does not wait longer than this for a response. */
const POLL_TIMEOUT_MS = 2000;

function status(el, text, on) {
  el.textContent = text;
  el.classList.toggle("an", on);
}

// --- Reading along -----------------------------------------------------------

let readTimer = null;

const VERDICT = {
  befehl: (e) => `Befehl: ${e.befehl}`,
  verworfen: (e) => `verworfen (${e.grund || e.befehl})`,
  weckwort: () => "Weckwort",
  ohne: () => "kein Befehl",
};

function showLive(d) {
  if (!d.verfuegbar) {
    status(readState, "kein Mikrofon", false);
    partial.textContent = "Spracherkennung: " + (d.zustand || "aus");
    levelBar.style.width = "0";
  } else {
    status(readState, d.taub ? "spricht selbst" : (d.geweckt ? "hört zu …" : "an"), true);
    levelBar.style.width = `${Math.round(d.pegel * 100)}%`;
    partial.textContent = d.taub ? "(hört weg, solange er spricht)" : (d.teil ? `„${d.teil} …“` : "—");
  }
  live.classList.toggle("taub", !!d.taub);

  eventList.replaceChildren(...d.ereignisse.slice().reverse().map((e) => {
    const li = document.createElement("li");
    li.className = e.ergebnis;
    const time = new Date(e.zeit * 1000).toLocaleTimeString("de-DE");
    const text = document.createElement("span");
    text.textContent = `${time}  „${e.text}“`;
    const verdict = document.createElement("span");
    verdict.className = "urteil";
    verdict.textContent = (VERDICT[e.ergebnis] || VERDICT.ohne)(e);
    li.append(text, verdict);
    return li;
  }));
}

async function poll() {
  try {
    // With a timeout: if the robot hangs, fetch would otherwise stay open
    // forever and reading along would stall even after it is long back.
    const response = await fetch("/sprache/live", { signal: AbortSignal.timeout(POLL_TIMEOUT_MS) });
    if (response.ok) showLive(await response.json());
  } catch {
    status(readState, "Roboter nicht erreichbar", false);
  }
  if (readToggle.checked) readTimer = setTimeout(poll, POLL_MS);
}

readToggle.addEventListener("change", () => {
  clearTimeout(readTimer);
  live.hidden = !readToggle.checked;
  if (readToggle.checked) {
    poll();
  } else {
    status(readState, "aus", false);
  }
});

// --- Listening in ------------------------------------------------------------

let ws = null;
let ctx = null;
let rate = 16000;
let nextStart = 0;
let lastHeard = 0;  // when something last arrived from the robot (performance.now)
let reconnectTimer = null;

function play(buffer) {
  const samples = new Int16Array(buffer);
  if (!samples.length) return;
  const audio = ctx.createBuffer(1, samples.length, rate);
  const channel = audio.getChannelData(0);
  for (let i = 0; i < samples.length; i++) channel[i] = samples[i] / 32768;

  const now = ctx.currentTime;
  // If playback lags behind (Wi-Fi congestion), better to skip than to
  // hear everything later and later.
  if (nextStart < now + 0.02 || nextStart > now + MAX_LAG_S) {
    nextStart = now + LEAD_S;
  }
  const source = ctx.createBufferSource();
  source.buffer = audio;
  source.connect(ctx.destination);
  source.start(nextStart);
  nextStart += audio.duration;
}

function listenOn() {
  // The AudioContext must be created from a click, otherwise it stays
  // silent in the browser -- that is why it happens here and not on page load.
  ctx = ctx || new AudioContext();
  ctx.resume();
  nextStart = 0;
  const proto = location.protocol === "https:" ? "wss" : "ws";
  ws = new WebSocket(`${proto}://${location.host}/sprache/ton`);
  ws.binaryType = "arraybuffer";
  lastHeard = performance.now();
  status(listenState, "verbinde …", false);
  ws.onmessage = (ev) => {
    lastHeard = performance.now();
    // The browser occasionally suspends audio (tab in background,
    // screen lock); then quietly resume it.
    if (ctx.state === "suspended") ctx.resume();
    if (typeof ev.data === "string") {
      const msg = JSON.parse(ev.data);
      if (msg.rate) {
        rate = msg.rate;
        status(listenState, `an (${rate / 1000} kHz)`, true);
      } else {
        status(listenState, "kein Mikrofon", false);
      }
      return;
    }
    play(ev.data);
  };
  ws.onclose = () => {
    ws = null;
    reconnect("getrennt");
  };
}

/** Drop the old connection without waiting for it to end. */
function disconnect() {
  if (!ws) return;
  const old = ws;
  ws = null;
  old.onclose = null;
  old.onmessage = null;
  old.close();
}

function reconnect(reason) {
  if (!listenToggle.checked) return;
  // Server restarted, Pi hung or Wi-Fi gone: quietly retry until it is
  // back.
  status(listenState, `${reason} -- verbinde neu …`, false);
  clearTimeout(reconnectTimer);
  reconnectTimer = setTimeout(() => { if (listenToggle.checked && !ws) listenOn(); }, 2000);
}

// A hanging connection does not report itself (no onclose when the Pi
// freezes). So watch for heartbeats ourselves.
setInterval(() => {
  if (ws && performance.now() - lastHeard > SILENCE_MS) {
    disconnect();
    reconnect("keine Antwort");
  }
}, 1000);

function listenOff() {
  clearTimeout(reconnectTimer);
  disconnect();
  if (ctx) ctx.suspend();
  status(listenState, "aus", false);
}

listenToggle.addEventListener("change", () => {
  if (listenToggle.checked) listenOn(); else listenOff();
});

// --- Announcement: speak through the robot ------------------------------------
//
// Own microphone (laptop/phone) -> WebSocket /sprache/durchsage -> the robot's
// speaker. First {"rate": Hz}, then raw 16-bit mono audio.

const speakToggle = $("mikro-sprechen");
const speakState = $("mikro-sprechen-state");
const hint = $("mikro-sprechen-hinweis");
/** Enough for speech; keeps Wi-Fi traffic small (32 kB/s). */
const SEND_RATE = 16000;
/** Port of the HTTPS variant (config.HTTPS_PORT). */
const HTTPS_PORT = 8443;

let dsWs = null;
let dsCtx = null;
let dsStream = null;
let dsTimer = null;

function showHint(nodes) {
  hint.replaceChildren(...(nodes ? [nodes].flat() : []));
  hint.hidden = !nodes;
}

/** Resample microphone audio to SEND_RATE and pack it as 16 bit. */
function pack(input, step) {
  const n = Math.floor(input.length / step);
  const out = new Int16Array(n);
  for (let i = 0; i < n; i++) {
    // Average over the window: a simple low-pass, otherwise it sounds harsh.
    const from = Math.floor(i * step);
    const to = Math.min(input.length, Math.floor((i + 1) * step));
    let sum = 0;
    for (let j = from; j < to; j++) sum += input[j];
    const value = Math.max(-1, Math.min(1, sum / Math.max(1, to - from)));
    out[i] = value * 32767;
  }
  return out;
}

async function speakOn() {
  // Browsers only grant microphone access on secure pages (HTTPS or
  // localhost). Under http://10.42.0.62:8000 getUserMedia does not exist at all.
  if (!window.isSecureContext || !navigator.mediaDevices?.getUserMedia) {
    const target = `https://${location.hostname}:${HTTPS_PORT}${location.pathname}`;
    const link = document.createElement("a");
    link.href = target;
    link.textContent = target;
    showHint(["Das Mikrofon gibt der Browser nur über HTTPS frei: ", link,
              " öffnen und die Zertifikatswarnung einmal bestätigen."]);
    speakToggle.checked = false;
    status(speakState, "nur über HTTPS", false);
    return;
  }
  showHint(null);
  status(speakState, "Mikrofon …", false);
  try {
    dsStream = await navigator.mediaDevices.getUserMedia({
      audio: { channelCount: 1, echoCancellation: true, noiseSuppression: true, autoGainControl: true },
    });
  } catch (err) {
    speakToggle.checked = false;
    status(speakState, "kein Zugriff", false);
    showHint(`Mikrofon nicht freigegeben (${err.name}). In den Seiteneinstellungen des Browsers erlauben.`);
    return;
  }
  if (!speakToggle.checked) { speakOff(); return; }  // switched off again meanwhile

  dsCtx = new AudioContext();
  const source = dsCtx.createMediaStreamSource(dsStream);
  // ScriptProcessor is deprecated but works in all browsers without a
  // separate worklet file. 2048 samples at 48 kHz are a good 40 ms.
  const processor = dsCtx.createScriptProcessor(2048, 1, 1);
  const step = dsCtx.sampleRate / SEND_RATE;
  processor.onaudioprocess = (ev) => {
    if (!dsWs || dsWs.readyState !== WebSocket.OPEN) return;
    // If Wi-Fi congests, better to drop audio than to arrive later and later.
    if (dsWs.bufferedAmount > SEND_RATE) return;
    dsWs.send(pack(ev.inputBuffer.getChannelData(0), step).buffer);
  };
  source.connect(processor);
  // The node only processes when connected to the output -- muted, otherwise
  // you would hear yourself from your own speaker.
  const mute = dsCtx.createGain();
  mute.gain.value = 0;
  processor.connect(mute);
  mute.connect(dsCtx.destination);
  dsConnect();
}

function dsConnect() {
  const proto = location.protocol === "https:" ? "wss" : "ws";
  dsWs = new WebSocket(`${proto}://${location.host}/sprache/durchsage`);
  dsWs.binaryType = "arraybuffer";
  status(speakState, "verbinde …", false);
  dsWs.onopen = () => dsWs.send(JSON.stringify({ rate: SEND_RATE }));
  dsWs.onmessage = (ev) => {
    if (typeof ev.data !== "string") return;
    const msg = JSON.parse(ev.data);
    if (msg.bereit) {
      status(speakState, "an -- sprich", true);
    } else {
      status(speakState, msg.grund || "geht nicht", false);
    }
  };
  dsWs.onclose = () => {
    dsWs = null;
    if (!speakToggle.checked) return;
    // Server restarted or Wi-Fi gone: reconnect, the microphone stays open.
    status(speakState, "getrennt -- verbinde neu …", false);
    clearTimeout(dsTimer);
    dsTimer = setTimeout(() => { if (speakToggle.checked && !dsWs) dsConnect(); }, 2000);
  };
}

function speakOff() {
  clearTimeout(dsTimer);
  if (dsWs) {
    const old = dsWs;
    dsWs = null;
    old.onclose = null;
    old.close();
  }
  // Really release the microphone, otherwise the browser keeps showing "recording".
  if (dsStream) dsStream.getTracks().forEach((t) => t.stop());
  dsStream = null;
  if (dsCtx) dsCtx.close();
  dsCtx = null;
  status(speakState, "aus", false);
}

speakToggle.addEventListener("change", () => {
  if (speakToggle.checked) speakOn(); else speakOff();
});

})();
